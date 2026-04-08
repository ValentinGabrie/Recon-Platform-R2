/**
 * @file sim_motor_node.cpp
 * @brief Simulated motor controller — drop-in replacement for motor_controller.
 *
 * Subscribes /cmd_vel, integrates differential-drive kinematics, publishes
 * /odom (Odometry), /roomba/pose (PoseStamped), and broadcasts odom→base_link
 * TF.  Collision detection against a shared ground-truth map prevents driving
 * through walls.
 *
 * Real-time C++17 node — part of roomba_hardware.
 */

#include <chrono>
#include <cmath>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"

/**
 * @class SimMotorNode
 * @brief Simulated differential-drive motor controller with collision
 *        detection and odometry publishing.
 */
class SimMotorNode : public rclcpp::Node
{
public:
    SimMotorNode()
    : Node("sim_motor_node"),
      x_(0.0), y_(0.0), theta_(0.0),
      linear_vel_(0.0), angular_vel_(0.0),
      motors_stopped_(true),
      has_ground_truth_(false),
      paused_(false),
      collision_count_(0)
    {
        // --- Parameters ---
        this->declare_parameter<double>("wheel_separation_m", 0.20);
        this->declare_parameter<double>("wheel_radius_m", 0.033);
        this->declare_parameter<double>("max_linear_vel", 0.5);
        this->declare_parameter<double>("max_angular_vel", 1.5);
        this->declare_parameter<double>("odom_rate_hz", 50.0);
        this->declare_parameter<double>("odom_noise_linear", 0.005);
        this->declare_parameter<double>("odom_noise_angular", 0.01);
        this->declare_parameter<int>("watchdog_timeout_ms", 500);
        this->declare_parameter<double>("collision_radius_m", 0.15);
        this->declare_parameter<double>("spawn_x", 1.0);
        this->declare_parameter<double>("spawn_y", 1.0);
        this->declare_parameter<double>("spawn_theta", 0.0);

        wheel_sep_ = this->get_parameter("wheel_separation_m").as_double();
        wheel_rad_ = this->get_parameter("wheel_radius_m").as_double();
        max_lin_ = this->get_parameter("max_linear_vel").as_double();
        max_ang_ = this->get_parameter("max_angular_vel").as_double();
        odom_rate_ = this->get_parameter("odom_rate_hz").as_double();
        noise_lin_ = this->get_parameter("odom_noise_linear").as_double();
        noise_ang_ = this->get_parameter("odom_noise_angular").as_double();
        watchdog_ms_ = this->get_parameter("watchdog_timeout_ms").as_int();
        collision_r_ = this->get_parameter("collision_radius_m").as_double();

        x_ = this->get_parameter("spawn_x").as_double();
        y_ = this->get_parameter("spawn_y").as_double();
        theta_ = this->get_parameter("spawn_theta").as_double();

        if (wheel_sep_ <= 0.0 || wheel_rad_ <= 0.0) {
            throw std::runtime_error(
                "wheel_separation_m and wheel_radius_m must be > 0");
        }
        if (odom_rate_ <= 0.0) {
            throw std::runtime_error("odom_rate_hz must be > 0");
        }

        // --- RNG ---
        rng_.seed(std::random_device{}());
        noise_lin_dist_ = std::normal_distribution<double>(0.0, noise_lin_);
        noise_ang_dist_ = std::normal_distribution<double>(0.0, noise_ang_);

        // --- Subscribers ---
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&SimMotorNode::cmdVelCallback, this,
                std::placeholders::_1));

        ground_truth_sub_ = this->create_subscription<
            nav_msgs::msg::OccupancyGrid>(
            "/sim/ground_truth", 1,
            std::bind(&SimMotorNode::groundTruthCallback, this,
                std::placeholders::_1));

        // --- Publishers ---
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom", 50);
        pose_pub_ = this->create_publisher<
            geometry_msgs::msg::PoseStamped>("/roomba/pose", 50);
        events_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/robot/events", 10);

        // --- /sim/command subscriber for web UI control ---
        sim_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/sim/command", 10,
            std::bind(&SimMotorNode::simCommandCallback, this,
                std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(
            *this);

        // --- Timers ---
        double dt = 1.0 / odom_rate_;
        odom_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(dt)),
            std::bind(&SimMotorNode::odomCallback, this));

        auto wd_period = std::chrono::milliseconds(watchdog_ms_);
        watchdog_timer_ = this->create_wall_timer(
            wd_period,
            std::bind(&SimMotorNode::watchdogCallback, this));

        last_cmd_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "sim_motor_node started — spawn (%.2f, %.2f, %.2f) "
            "odom %.0f Hz, noise lin=%.4f ang=%.4f",
            x_, y_, theta_, odom_rate_, noise_lin_, noise_ang_);
    }

private:
    /**
     * @brief Receive velocity commands — same interface as motor_controller.
     */
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        linear_vel_ = std::clamp(msg->linear.x, -max_lin_, max_lin_);
        angular_vel_ = std::clamp(msg->angular.z, -max_ang_, max_ang_);
        last_cmd_time_ = this->now();
        motors_stopped_ = false;
    }

    /**
     * @brief Handle simulation commands from /sim/command.
     *        Commands: "reset_pose", "pause", "resume"
     */
    void simCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        const auto & cmd = msg->data;
        if (cmd == "reset_pose") {
            std::lock_guard<std::mutex> lock(mutex_);
            x_ = this->get_parameter("spawn_x").as_double();
            y_ = this->get_parameter("spawn_y").as_double();
            theta_ = this->get_parameter("spawn_theta").as_double();
            linear_vel_ = 0.0;
            angular_vel_ = 0.0;
            motors_stopped_ = true;
            collision_count_ = 0;
            RCLCPP_INFO(this->get_logger(),
                "Pose reset to spawn (%.2f, %.2f, %.2f)",
                x_, y_, theta_);
            auto event = std_msgs::msg::String();
            event.data = "SIM_POSE_RESET";
            events_pub_->publish(event);
        } else if (cmd == "pause") {
            paused_ = true;
            RCLCPP_INFO(this->get_logger(), "Simulation paused");
        } else if (cmd == "resume") {
            paused_ = false;
            RCLCPP_INFO(this->get_logger(), "Simulation resumed");
        }
    }

    /**
     * @brief Receive ground truth map for collision detection.
     */
    void groundTruthCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        ground_truth_ = *msg;
        has_ground_truth_ = true;
    }

    /**
     * @brief Periodic odometry update — integrates kinematics, checks
     *        collision, publishes odom + pose + TF.
     */
    void odomCallback()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        double dt = 1.0 / odom_rate_;

        // Add noise to the velocities
        double noisy_lin = linear_vel_ + noise_lin_dist_(rng_);
        double noisy_ang = angular_vel_ + noise_ang_dist_(rng_);

        if (motors_stopped_ || paused_) {
            noisy_lin = 0.0;
            noisy_ang = 0.0;
        }

        // Integrate kinematics
        double new_theta = theta_ + noisy_ang * dt;
        // Normalise to [-pi, pi]
        new_theta = std::atan2(std::sin(new_theta), std::cos(new_theta));

        double new_x = x_ + noisy_lin * std::cos(theta_) * dt;
        double new_y = y_ + noisy_lin * std::sin(theta_) * dt;

        // Collision check — only update position if not colliding
        if (!checkCollision(new_x, new_y)) {
            x_ = new_x;
            y_ = new_y;
            theta_ = new_theta;
        } else {
            // Blocked — allow rotation but not translation
            theta_ = new_theta;
            ++collision_count_;
            RCLCPP_DEBUG(this->get_logger(),
                "Collision at (%.2f, %.2f) — movement blocked (total: %d)",
                new_x, new_y, collision_count_);
        }

        auto stamp = this->now();

        // Publish Odometry
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.z = std::sin(theta_ / 2.0);
        odom.pose.pose.orientation.w = std::cos(theta_ / 2.0);
        odom.twist.twist.linear.x = motors_stopped_ ? 0.0 : linear_vel_;
        odom.twist.twist.angular.z = motors_stopped_ ? 0.0 : angular_vel_;
        odom_pub_->publish(odom);

        // Publish PoseStamped for web UI (odom frame — use slam_toolbox /pose for map frame)
        auto pose = geometry_msgs::msg::PoseStamped();
        pose.header.stamp = stamp;
        pose.header.frame_id = "odom";
        pose.pose.position.x = x_;
        pose.pose.position.y = y_;
        pose.pose.orientation.z = std::sin(theta_ / 2.0);
        pose.pose.orientation.w = std::cos(theta_ / 2.0);
        pose_pub_->publish(pose);

        // Broadcast TF: odom → base_link
        auto tf = geometry_msgs::msg::TransformStamped();
        tf.header.stamp = stamp;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.rotation.z = std::sin(theta_ / 2.0);
        tf.transform.rotation.w = std::cos(theta_ / 2.0);
        tf_broadcaster_->sendTransform(tf);
    }

    /**
     * @brief Watchdog — stops motors if no cmd_vel within timeout.
     *        Same contract as real motor_controller.
     */
    void watchdogCallback()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        double elapsed_ms =
            (this->now() - last_cmd_time_).seconds() * 1000.0;
        if (elapsed_ms > static_cast<double>(watchdog_ms_)) {
            if (!motors_stopped_) {
                RCLCPP_WARN(this->get_logger(),
                    "Watchdog triggered — no cmd_vel for %.0f ms",
                    elapsed_ms);
                motors_stopped_ = true;
                linear_vel_ = 0.0;
                angular_vel_ = 0.0;

                auto event = std_msgs::msg::String();
                event.data = "WATCHDOG_STOP";
                events_pub_->publish(event);
            }
        }
    }

    /**
     * @brief Check if the robot would collide with an obstacle at (cx, cy).
     * @param cx World-frame X position to test.
     * @param cy World-frame Y position to test.
     * @return true if collision detected.
     */
    bool checkCollision(double cx, double cy)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!has_ground_truth_) {
            return false;  // No map yet — allow movement
        }

        const auto & info = ground_truth_.info;
        double res = info.resolution;
        double ox = info.origin.position.x;
        double oy = info.origin.position.y;
        int w = static_cast<int>(info.width);
        int h = static_cast<int>(info.height);

        // Check cells within collision_radius_ of (cx, cy)
        int radius_cells = static_cast<int>(
            std::ceil(collision_r_ / res));

        int center_gx = static_cast<int>((cx - ox) / res);
        int center_gy = static_cast<int>((cy - oy) / res);

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                int gx = center_gx + dx;
                int gy = center_gy + dy;

                if (gx < 0 || gx >= w || gy < 0 || gy >= h) {
                    return true;  // Out of bounds = collision
                }

                // Check if this cell is within the circular radius
                double cell_x = ox + (gx + 0.5) * res;
                double cell_y = oy + (gy + 0.5) * res;
                double dist = std::hypot(cell_x - cx, cell_y - cy);
                if (dist > collision_r_) {
                    continue;
                }

                int idx = gy * w + gx;
                if (ground_truth_.data[idx] > 50) {
                    return true;  // Occupied cell
                }
            }
        }
        return false;
    }

    // Parameters
    double wheel_sep_;
    double wheel_rad_;
    double max_lin_;
    double max_ang_;
    double odom_rate_;
    double noise_lin_;
    double noise_ang_;
    int watchdog_ms_;
    double collision_r_;

    // Pose state
    double x_;
    double y_;
    double theta_;
    double linear_vel_;
    double angular_vel_;
    bool motors_stopped_;
    rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

    // Simulation control state
    bool paused_;
    int collision_count_;

    // Ground truth map for collision
    nav_msgs::msg::OccupancyGrid ground_truth_;
    bool has_ground_truth_;
    std::mutex map_mutex_;

    // Noise
    std::mt19937 rng_;
    std::normal_distribution<double> noise_lin_dist_;
    std::normal_distribution<double> noise_ang_dist_;

    // Mutex
    std::mutex mutex_;

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
        ground_truth_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sim_cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SimMotorNode>());
    } catch (const std::runtime_error & e) {
        RCLCPP_FATAL(rclcpp::get_logger("sim_motor_node"),
            "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
