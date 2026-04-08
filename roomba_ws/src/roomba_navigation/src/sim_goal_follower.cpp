/**
 * @file sim_goal_follower.cpp
 * @brief Goal follower with fuzzy obstacle avoidance for simulate-hw mode.
 *
 * Subscribes to /goal_pose (from recon_node), /odom (from sim_motor_node),
 * and /sensors/{front,left,right} (from sim_sensor_node).
 *
 * Uses proportional control for goal tracking and a Mamdani fuzzy
 * inference system on ultrasonic readings for obstacle avoidance.
 * The fuzzy output modulates the proportional controller's speed
 * and adds a turn bias — same pattern that would be used on real HW.
 *
 * Fuzzy inputs:
 *   - front_dist, left_dist, right_dist (metres, from ultrasonics)
 *
 * Fuzzy outputs:
 *   - speed_factor (0–1 multiplier on linear velocity)
 *   - turn_bias    (angular velocity modifier, rad/s)
 *
 * Real-time C++17 node — part of roomba_navigation.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"

// ================================================================
// Fuzzy helpers (same as recon_node — lightweight, no shared lib)
// ================================================================

inline double trapmf(double x, double a, double b, double c, double d)
{
    if (x <= a || x >= d) { return 0.0; }
    if (x >= b && x <= c) { return 1.0; }
    if (x < b) { return (b > a) ? (x - a) / (b - a) : 1.0; }
    return (d > c) ? (d - x) / (d - c) : 1.0;
}

inline double trimf(double x, double a, double b, double c)
{
    return trapmf(x, a, b, b, c);
}

// ================================================================
// SimGoalFollower
// ================================================================

class SimGoalFollower : public rclcpp::Node
{
public:
    SimGoalFollower()
    : Node("sim_goal_follower"),
      has_goal_(false),
      has_odom_(false),
      has_us_(false),
      front_dist_(4.0),
      left_dist_(4.0),
      right_dist_(4.0)
    {
        // ---- Goal-following parameters ----
        this->declare_parameter<double>("kp_linear", 0.5);
        this->declare_parameter<double>("kp_angular", 2.0);
        this->declare_parameter<double>("max_linear_vel", 0.3);
        this->declare_parameter<double>("max_angular_vel", 1.0);
        this->declare_parameter<double>("goal_tolerance", 0.3);
        this->declare_parameter<double>("heading_tolerance", 0.15);
        this->declare_parameter<double>("control_rate_hz", 20.0);
        this->declare_parameter<double>("stuck_timeout_s", 10.0);

        // ---- Fuzzy obstacle avoidance MFs (trapezoidal: a,b,c,d) ----
        // Front distance
        this->declare_parameter<std::vector<double>>(
            "fuzzy_front_close", {0.0, 0.0, 0.15, 0.4});
        this->declare_parameter<std::vector<double>>(
            "fuzzy_front_near", {0.25, 0.5, 0.5, 0.9});
        this->declare_parameter<std::vector<double>>(
            "fuzzy_front_far", {0.7, 1.2, 4.0, 4.0});
        // Side distance (used for both left and right)
        this->declare_parameter<std::vector<double>>(
            "fuzzy_side_close", {0.0, 0.0, 0.1, 0.35});
        this->declare_parameter<std::vector<double>>(
            "fuzzy_side_near", {0.2, 0.4, 0.4, 0.7});
        this->declare_parameter<std::vector<double>>(
            "fuzzy_side_far", {0.5, 0.8, 4.0, 4.0});
        // Max fuzzy turn bias (rad/s)
        this->declare_parameter<double>("fuzzy_max_turn_bias", 0.8);

        // Retrieve params
        kp_lin_ = this->get_parameter("kp_linear").as_double();
        kp_ang_ = this->get_parameter("kp_angular").as_double();
        max_lin_ = this->get_parameter("max_linear_vel").as_double();
        max_ang_ = this->get_parameter("max_angular_vel").as_double();
        goal_tol_ = this->get_parameter("goal_tolerance").as_double();
        heading_tol_ = this->get_parameter("heading_tolerance").as_double();
        control_rate_ = this->get_parameter("control_rate_hz").as_double();
        stuck_timeout_ = this->get_parameter(
            "stuck_timeout_s").as_double();
        max_turn_bias_ = this->get_parameter(
            "fuzzy_max_turn_bias").as_double();

        mf_fc_ = this->get_parameter(
            "fuzzy_front_close").as_double_array();
        mf_fn_ = this->get_parameter(
            "fuzzy_front_near").as_double_array();
        mf_ff_ = this->get_parameter(
            "fuzzy_front_far").as_double_array();
        mf_sc_ = this->get_parameter(
            "fuzzy_side_close").as_double_array();
        mf_sn_ = this->get_parameter(
            "fuzzy_side_near").as_double_array();
        mf_sf_ = this->get_parameter(
            "fuzzy_side_far").as_double_array();

        // Subscribers
        goal_sub_ = this->create_subscription<
            geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&SimGoalFollower::goalCallback, this,
                std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&SimGoalFollower::odomCallback, this,
                std::placeholders::_1));

        // Ultrasonic subscribers (same topics as esp32_sensor_node)
        us_front_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/sensors/front", 10,
            std::bind(&SimGoalFollower::usFrontCallback, this,
                std::placeholders::_1));
        us_left_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/sensors/left", 10,
            std::bind(&SimGoalFollower::usLeftCallback, this,
                std::placeholders::_1));
        us_right_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/sensors/right", 10,
            std::bind(&SimGoalFollower::usRightCallback, this,
                std::placeholders::_1));

        // Publisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        events_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/robot/events", 10);

        // Control loop timer
        auto period = std::chrono::duration<double>(
            1.0 / control_rate_);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SimGoalFollower::controlLoop, this));

        RCLCPP_INFO(this->get_logger(),
            "sim_goal_follower started — P-control + fuzzy obstacle "
            "avoidance, kp_lin=%.2f kp_ang=%.2f goal_tol=%.2f m",
            kp_lin_, kp_ang_, goal_tol_);
    }

private:
    // ================================================================
    // Callbacks
    // ================================================================

    void goalCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        double new_x = msg->pose.position.x;
        double new_y = msg->pose.position.y;

        // Only reset stuck timer when goal actually changes
        bool goal_changed = !has_goal_ ||
            std::hypot(new_x - goal_x_, new_y - goal_y_) > 0.1;

        goal_x_ = new_x;
        goal_y_ = new_y;
        has_goal_ = true;

        if (goal_changed) {
            goal_time_ = this->now();
            stuck_recovering_ = false;
            RCLCPP_INFO(this->get_logger(),
                "New goal: (%.2f, %.2f)", goal_x_, goal_y_);
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        robot_theta_ = 2.0 * std::atan2(qz, qw);
        has_odom_ = true;
    }

    void usFrontCallback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        front_dist_ = msg->range;
        has_us_ = true;
    }

    void usLeftCallback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        left_dist_ = msg->range;
    }

    void usRightCallback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        right_dist_ = msg->range;
    }

    // ================================================================
    // Control loop
    // ================================================================

    void controlLoop()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        auto cmd = geometry_msgs::msg::Twist();

        if (!has_goal_ || !has_odom_) {
            cmd_pub_->publish(cmd);
            return;
        }

        double dx = goal_x_ - robot_x_;
        double dy = goal_y_ - robot_y_;
        double dist = std::hypot(dx, dy);

        // Goal reached
        if (dist < goal_tol_) {
            has_goal_ = false;
            cmd_pub_->publish(cmd);
            RCLCPP_INFO(this->get_logger(),
                "Goal reached (%.2f, %.2f)", goal_x_, goal_y_);
            return;
        }

        // Stuck timeout — back up briefly then abandon
        double elapsed = (this->now() - goal_time_).seconds();
        if (elapsed > stuck_timeout_) {
            if (!stuck_recovering_) {
                // Phase 1: back up and turn for 2 seconds
                stuck_recovering_ = true;
                recover_end_time_ = this->now() +
                    rclcpp::Duration::from_seconds(2.0);
                RCLCPP_WARN(this->get_logger(),
                    "Stuck (%.1fs) — backing up from (%.2f, %.2f)",
                    elapsed, goal_x_, goal_y_);
            }

            if (this->now() < recover_end_time_) {
                // Back up and turn
                cmd.linear.x = -0.15;
                cmd.angular.z = 0.8;
                cmd_pub_->publish(cmd);
                return;
            }

            // Recovery complete — abandon goal
            has_goal_ = false;
            stuck_recovering_ = false;
            cmd_pub_->publish(cmd);
            RCLCPP_WARN(this->get_logger(),
                "Goal timeout — abandoning (%.2f, %.2f)",
                goal_x_, goal_y_);

            // Publish event so recon_node can blacklist this goal
            auto event = std_msgs::msg::String();
            event.data = "GOAL_FAILED";
            events_pub_->publish(event);
            return;
        }

        // ---- Proportional goal tracking ----
        double desired_heading = std::atan2(dy, dx);
        double heading_err = desired_heading - robot_theta_;
        while (heading_err > M_PI) { heading_err -= 2.0 * M_PI; }
        while (heading_err < -M_PI) { heading_err += 2.0 * M_PI; }

        double p_linear = 0.0;
        double p_angular = 0.0;

        if (std::abs(heading_err) > heading_tol_) {
            p_angular = std::clamp(
                kp_ang_ * heading_err, -max_ang_, max_ang_);
            p_linear = std::clamp(
                kp_lin_ * dist * 0.2, 0.0, max_lin_ * 0.2);
        } else {
            p_linear = std::clamp(kp_lin_ * dist, 0.0, max_lin_);
            p_angular = std::clamp(
                kp_ang_ * heading_err, -max_ang_, max_ang_);
        }

        // ---- Fuzzy obstacle avoidance ----
        double speed_factor = 1.0;
        double turn_bias = 0.0;

        if (has_us_) {
            fuzzyAvoidance(front_dist_, left_dist_, right_dist_,
                           speed_factor, turn_bias);
        }

        // ---- Combine P-control + fuzzy avoidance ----
        cmd.linear.x = p_linear * speed_factor;
        cmd.angular.z = std::clamp(
            p_angular + turn_bias, -max_ang_, max_ang_);

        cmd_pub_->publish(cmd);
    }

    // ================================================================
    // Fuzzy obstacle avoidance (Mamdani)
    // ================================================================

    /**
     * @brief Compute fuzzy avoidance modifiers from ultrasonic readings.
     *
     * @param front   Front ultrasonic distance (m).
     * @param left    Left ultrasonic distance (m).
     * @param right   Right ultrasonic distance (m).
     * @param[out] speed_factor  Multiplier for linear velocity (0–1).
     * @param[out] turn_bias     Additional angular velocity (rad/s).
     *
     * Rules:
     *   R1: front=CLOSE                       → speed=STOP
     *   R2: front=CLOSE ∧ left=FAR ∧ right≠FAR → turn=LEFT
     *   R3: front=CLOSE ∧ right=FAR ∧ left≠FAR → turn=RIGHT
     *   R4: front=CLOSE ∧ left≥right          → turn=RIGHT
     *   R5: front=NEAR                        → speed=SLOW
     *   R6: front=NEAR ∧ left=CLOSE           → turn=SOFT_RIGHT
     *   R7: front=NEAR ∧ right=CLOSE          → turn=SOFT_LEFT
     *   R8: front=FAR ∧ left=FAR ∧ right=FAR  → speed=FAST
     *   R9: left=CLOSE ∧ right≠CLOSE          → turn=SOFT_RIGHT
     *   R10: right=CLOSE ∧ left≠CLOSE         → turn=SOFT_LEFT
     */
    void fuzzyAvoidance(double front, double left, double right,
                        double & speed_factor,
                        double & turn_bias) const
    {
        // ---- Fuzzify inputs ----
        double f_close = trapmf(front, mf_fc_[0], mf_fc_[1],
                                       mf_fc_[2], mf_fc_[3]);
        double f_near  = trapmf(front, mf_fn_[0], mf_fn_[1],
                                       mf_fn_[2], mf_fn_[3]);
        double f_far   = trapmf(front, mf_ff_[0], mf_ff_[1],
                                       mf_ff_[2], mf_ff_[3]);

        double l_close = trapmf(left,  mf_sc_[0], mf_sc_[1],
                                       mf_sc_[2], mf_sc_[3]);
        double l_far   = trapmf(left,  mf_sf_[0], mf_sf_[1],
                                       mf_sf_[2], mf_sf_[3]);

        double r_close = trapmf(right, mf_sc_[0], mf_sc_[1],
                                       mf_sc_[2], mf_sc_[3]);
        double r_far   = trapmf(right, mf_sf_[0], mf_sf_[1],
                                       mf_sf_[2], mf_sf_[3]);

        // ---- Speed rules (aggregate into output MFs) ----
        // Output sets: STOP(0.0), SLOW(0.3), MEDIUM(0.6), FAST(1.0)
        double agg_spd[4] = {0.0, 0.0, 0.0, 0.0};

        // R1: front=CLOSE → STOP
        agg_spd[0] = std::max(agg_spd[0], f_close);
        // R5: front=NEAR → SLOW
        agg_spd[1] = std::max(agg_spd[1], f_near);
        // R8: front=FAR ∧ left=FAR ∧ right=FAR → FAST
        agg_spd[3] = std::max(agg_spd[3],
            std::min({f_far, l_far, r_far}));
        // Default: if sides close but front OK → MEDIUM
        agg_spd[2] = std::max(agg_spd[2],
            std::min(f_far, std::max(l_close, r_close)));

        // Defuzzify speed: centroid over [0, 1]
        constexpr int N = 51;
        double s_num = 0.0, s_den = 0.0;
        for (int i = 0; i < N; ++i) {
            double x = static_cast<double>(i) / (N - 1);
            double mu = 0.0;
            mu = std::max(mu, std::min(agg_spd[0],
                trimf(x, -0.1, 0.0, 0.2)));
            mu = std::max(mu, std::min(agg_spd[1],
                trimf(x, 0.1, 0.3, 0.5)));
            mu = std::max(mu, std::min(agg_spd[2],
                trimf(x, 0.4, 0.6, 0.8)));
            mu = std::max(mu, std::min(agg_spd[3],
                trimf(x, 0.7, 1.0, 1.1)));
            s_num += x * mu;
            s_den += mu;
        }
        speed_factor = (s_den > 0.0) ? s_num / s_den : 0.5;

        // ---- Turn rules (aggregate into output MFs) ----
        // Output sets: HARD_LEFT(1), SOFT_LEFT(0.5), STRAIGHT(0),
        //              SOFT_RIGHT(-0.5), HARD_RIGHT(-1)
        // Positive = turn left (positive angular.z)
        double agg_turn[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

        // R2: front=CLOSE ∧ left=FAR → HARD_LEFT
        agg_turn[0] = std::max(agg_turn[0],
            std::min(f_close, l_far));
        // R3: front=CLOSE ∧ right=FAR → HARD_RIGHT
        agg_turn[4] = std::max(agg_turn[4],
            std::min(f_close, r_far));
        // R4: front=CLOSE ∧ left_closer → HARD_RIGHT
        //     (implemented as front=CLOSE ∧ l_close)
        agg_turn[4] = std::max(agg_turn[4],
            std::min(f_close, l_close));
        // R6: front=NEAR ∧ l_close → SOFT_RIGHT
        agg_turn[3] = std::max(agg_turn[3],
            std::min(f_near, l_close));
        // R7: front=NEAR ∧ r_close → SOFT_LEFT
        agg_turn[1] = std::max(agg_turn[1],
            std::min(f_near, r_close));
        // R8: all FAR → STRAIGHT
        agg_turn[2] = std::max(agg_turn[2],
            std::min({f_far, l_far, r_far}));
        // R9: l_close ∧ !r_close → SOFT_RIGHT
        agg_turn[3] = std::max(agg_turn[3],
            std::min(l_close, 1.0 - r_close));
        // R10: r_close ∧ !l_close → SOFT_LEFT
        agg_turn[1] = std::max(agg_turn[1],
            std::min(r_close, 1.0 - l_close));

        // Defuzzify turn: centroid over [-1, 1]
        double t_num = 0.0, t_den = 0.0;
        for (int i = 0; i < N; ++i) {
            double x = -1.0 + 2.0 * static_cast<double>(i) / (N - 1);
            double mu = 0.0;
            mu = std::max(mu, std::min(agg_turn[0],
                trimf(x, 0.5, 1.0, 1.5)));      // HARD_LEFT
            mu = std::max(mu, std::min(agg_turn[1],
                trimf(x, 0.1, 0.4, 0.7)));       // SOFT_LEFT
            mu = std::max(mu, std::min(agg_turn[2],
                trimf(x, -0.2, 0.0, 0.2)));      // STRAIGHT
            mu = std::max(mu, std::min(agg_turn[3],
                trimf(x, -0.7, -0.4, -0.1)));    // SOFT_RIGHT
            mu = std::max(mu, std::min(agg_turn[4],
                trimf(x, -1.5, -1.0, -0.5)));    // HARD_RIGHT
            t_num += x * mu;
            t_den += mu;
        }
        double turn_norm = (t_den > 0.0) ? t_num / t_den : 0.0;
        turn_bias = turn_norm * max_turn_bias_;
    }

    // ================================================================
    // Members
    // ================================================================

    // Goal-following params
    double kp_lin_;
    double kp_ang_;
    double max_lin_;
    double max_ang_;
    double goal_tol_;
    double heading_tol_;
    double control_rate_;
    double stuck_timeout_;
    double max_turn_bias_;

    // Fuzzy MF params
    std::vector<double> mf_fc_, mf_fn_, mf_ff_;  // front
    std::vector<double> mf_sc_, mf_sn_, mf_sf_;  // sides

    // State
    bool has_goal_;
    bool has_odom_;
    bool has_us_;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double front_dist_;
    double left_dist_;
    double right_dist_;
    rclcpp::Time goal_time_{0, 0, RCL_ROS_TIME};
    bool stuck_recovering_ = false;
    rclcpp::Time recover_end_time_{0, 0, RCL_ROS_TIME};
    std::mutex mutex_;

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us_front_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr us_right_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SimGoalFollower>());
    } catch (const std::runtime_error & e) {
        RCLCPP_FATAL(rclcpp::get_logger("sim_goal_follower"),
            "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
