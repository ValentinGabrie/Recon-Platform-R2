/**
 * @file bt_sim_node.cpp
 * @brief Bluetooth/controller simulation node — publishes synthetic /joy and
 *        /cmd_vel data for pre-hardware testing of the controller pipeline.
 *
 * Real-time C++17 node. Publishes identical message types on identical topics
 * as the real joy_node + joy_control_node, so no downstream node needs any
 * awareness of whether data comes from simulation or real hardware.
 *
 * Supports three waveform profiles: idle, sine, patrol.
 */

#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

// TODO: Replace with generated service once BtSimInject.srv is built
// #include "roomba_control/srv/bt_sim_inject.hpp"

/**
 * @class BtSimNode
 * @brief Simulates controller input using configurable waveform profiles.
 */
class BtSimNode : public rclcpp::Node
{
public:
    BtSimNode()
    : Node("bt_sim_node"),
      start_time_(this->now())
    {
        // Declare parameters
        this->declare_parameter<std::string>("bt_sim_profile", "sine");
        this->declare_parameter<double>("sine_amplitude", 0.5);
        this->declare_parameter<double>("sine_period_s", 8.0);
        this->declare_parameter<double>("patrol_forward_speed", 0.5);
        this->declare_parameter<double>("patrol_turn_speed", 0.5);
        this->declare_parameter<double>("patrol_straight_s", 3.0);
        this->declare_parameter<double>("patrol_turn_s", 1.0);
        this->declare_parameter<double>("max_linear_vel", 0.5);
        this->declare_parameter<double>("max_angular_vel", 1.5);

        // Retrieve parameters
        profile_ = this->get_parameter("bt_sim_profile").as_string();
        sine_amplitude_ = this->get_parameter("sine_amplitude").as_double();
        sine_period_ = this->get_parameter("sine_period_s").as_double();
        patrol_fwd_ = this->get_parameter(
            "patrol_forward_speed").as_double();
        patrol_turn_ = this->get_parameter(
            "patrol_turn_speed").as_double();
        patrol_straight_s_ = this->get_parameter(
            "patrol_straight_s").as_double();
        patrol_turn_s_ = this->get_parameter(
            "patrol_turn_s").as_double();
        max_linear_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_ = this->get_parameter("max_angular_vel").as_double();

        // Validate profile
        if (profile_ != "idle" && profile_ != "sine" &&
            profile_ != "patrol")
        {
            throw std::runtime_error(
                "Invalid bt_sim_profile: '" + profile_ +
                "'. Must be one of: idle, sine, patrol");
        }

        // Publishers
        joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(
            "/joy", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/bt_sim/status", 10);

        // Timers — /joy at 50 Hz, /cmd_vel at 20 Hz
        joy_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50 Hz
            std::bind(&BtSimNode::joyTimerCallback, this));

        cmd_vel_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20 Hz
            std::bind(&BtSimNode::cmdVelTimerCallback, this));

        // Status at 1 Hz
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&BtSimNode::statusTimerCallback, this));

        RCLCPP_INFO(this->get_logger(),
            "bt_sim_node active — real controller NOT required. "
            "Profile: %s", profile_.c_str());
    }

private:
    /**
     * @brief Get elapsed time since node start in seconds.
     */
    double elapsedSeconds() const
    {
        return (this->now() - start_time_).seconds();
    }

    /**
     * @brief Compute simulated left_y axis value based on profile.
     */
    double getLeftY() const
    {
        double t = elapsedSeconds();
        if (profile_ == "idle") {
            return 0.0;
        } else if (profile_ == "sine") {
            return sine_amplitude_ *
                std::sin(2.0 * M_PI * t / sine_period_);
        } else if (profile_ == "patrol") {
            double cycle = patrol_straight_s_ + patrol_turn_s_;
            double phase = std::fmod(t, cycle);
            return patrol_fwd_;  // Always moving forward in patrol
        }
        return 0.0;
    }

    /**
     * @brief Compute simulated right_x axis value based on profile.
     */
    double getRightX() const
    {
        double t = elapsedSeconds();
        if (profile_ == "idle") {
            return 0.0;
        } else if (profile_ == "sine") {
            // Phase-shifted sine for angular
            return sine_amplitude_ *
                std::sin(2.0 * M_PI * t / sine_period_ + M_PI / 4.0);
        } else if (profile_ == "patrol") {
            double cycle = patrol_straight_s_ + patrol_turn_s_;
            double phase = std::fmod(t, cycle);
            if (phase < patrol_straight_s_) {
                return 0.0;  // Straight
            } else {
                return patrol_turn_;  // Turning
            }
        }
        return 0.0;
    }

    /**
     * @brief Publish Joy message at 50 Hz.
     */
    void joyTimerCallback()
    {
        auto msg = sensor_msgs::msg::Joy();
        msg.header.stamp = this->now();

        // 6 axes: left_x, left_y, right_x, right_y, lt, rt
        msg.axes.resize(6, 0.0f);
        msg.axes[1] = static_cast<float>(getLeftY());   // left_y
        msg.axes[2] = static_cast<float>(getRightX());  // right_x

        // 14 buttons — all unpressed
        msg.buttons.resize(14, 0);

        joy_pub_->publish(msg);
    }

    /**
     * @brief Publish Twist message at 20 Hz.
     */
    void cmdVelTimerCallback()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = getLeftY() * max_linear_;
        msg.angular.z = getRightX() * max_angular_;
        cmd_vel_pub_->publish(msg);
    }

    /**
     * @brief Publish simulation status at 1 Hz.
     */
    void statusTimerCallback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "BT_SIM_ACTIVE";
        status_pub_->publish(msg);
    }

    // Parameters
    std::string profile_;
    double sine_amplitude_;
    double sine_period_;
    double patrol_fwd_;
    double patrol_turn_;
    double patrol_straight_s_;
    double patrol_turn_s_;
    double max_linear_;
    double max_angular_;

    // State
    rclcpp::Time start_time_;
    std::mutex mutex_;

    // ROS2 interfaces
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr joy_timer_;
    rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<BtSimNode>());
    } catch (const std::runtime_error & e) {
        RCLCPP_FATAL(rclcpp::get_logger("bt_sim_node"),
            "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
