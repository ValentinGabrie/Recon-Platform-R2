/**
 * @file joy_control_node.cpp
 * @brief Xbox controller to cmd_vel bridge — subscribes to /joy and publishes
 *        /cmd_vel when manual mode is active.
 *
 * Real-time C++17 node. Button mapping is loaded from parameters (controller.yaml).
 */

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class JoyControlNode
 * @brief Converts Joy messages to Twist commands for differential drive control.
 *        Only publishes when robot is in MANUAL mode.
 */
class JoyControlNode : public rclcpp::Node
{
public:
    JoyControlNode()
    : Node("joy_control_node"),
      current_mode_("IDLE")
    {
        // Declare axis mapping parameters (joy_linux + xpadneo defaults)
        this->declare_parameter<int>("axes.left_x", 0);
        this->declare_parameter<int>("axes.left_y", 1);
        this->declare_parameter<int>("axes.right_x", 3);
        this->declare_parameter<int>("axes.right_y", 4);

        // Declare button mapping parameters (joy_linux + xpadneo defaults)
        this->declare_parameter<int>("buttons.start", 7);
        this->declare_parameter<int>("buttons.b", 1);
        this->declare_parameter<int>("buttons.y", 3);
        this->declare_parameter<int>("buttons.x", 2);

        // Declare velocity parameters
        this->declare_parameter<double>("max_linear_vel", 0.5);
        this->declare_parameter<double>("max_angular_vel", 1.5);
        this->declare_parameter<double>("deadzone", 0.1);

        // Retrieve parameters
        axis_left_y_ = this->get_parameter("axes.left_y").as_int();
        axis_right_x_ = this->get_parameter("axes.right_x").as_int();
        btn_start_ = this->get_parameter("buttons.start").as_int();
        btn_b_ = this->get_parameter("buttons.b").as_int();
        btn_y_ = this->get_parameter("buttons.y").as_int();
        btn_x_ = this->get_parameter("buttons.x").as_int();
        max_linear_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_ = this->get_parameter("max_angular_vel").as_double();
        deadzone_ = this->get_parameter("deadzone").as_double();

        // Subscribers
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyControlNode::joyCallback, this,
                std::placeholders::_1));

        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot/mode", 10,
            std::bind(&JoyControlNode::modeCallback, this,
                std::placeholders::_1));

        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        events_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/robot/events", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/robot/mode", 10);

        RCLCPP_INFO(this->get_logger(),
            "joy_control_node started — max_linear: %.2f m/s, "
            "max_angular: %.2f rad/s",
            max_linear_, max_angular_);
    }

private:
    /**
     * @brief Apply deadzone to an axis value.
     */
    double applyDeadzone(double value) const
    {
        if (std::abs(value) < deadzone_) {
            return 0.0;
        }
        return value;
    }

    /**
     * @brief Callback for /robot/mode topic updates.
     */
    void modeCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (current_mode_ != msg->data) {
            RCLCPP_INFO(this->get_logger(),
                "Mode changed: %s -> %s",
                current_mode_.c_str(), msg->data.c_str());
            current_mode_ = msg->data;
        }
    }

    /**
     * @brief Callback for /joy messages — maps axes/buttons to cmd_vel.
     */
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Resize prev_buttons_ on first message
        if (prev_buttons_.empty()) {
            prev_buttons_.resize(msg->buttons.size(), 0);
        }

        // Check button presses (EDGE DETECTION — only on 0→1 transition)
        if (buttonJustPressed(msg, btn_start_)) {
            // Toggle between MANUAL and IDLE
            auto mode_msg = std_msgs::msg::String();
            if (current_mode_ == "MANUAL") {
                mode_msg.data = "IDLE";
            } else {
                mode_msg.data = "MANUAL";
            }
            mode_pub_->publish(mode_msg);
            RCLCPP_INFO(this->get_logger(),
                "Start button pressed — mode: %s",
                mode_msg.data.c_str());
        }

        if (buttonJustPressed(msg, btn_b_)) {
            // Emergency stop
            auto event = std_msgs::msg::String();
            event.data = "EMERGENCY_STOP";
            events_pub_->publish(event);

            // Publish zero velocity
            auto stop = geometry_msgs::msg::Twist();
            cmd_vel_pub_->publish(stop);
            RCLCPP_WARN(this->get_logger(), "Emergency stop triggered!");
        }

        if (buttonJustPressed(msg, btn_y_)) {
            // Trigger RECON mode
            auto mode_msg = std_msgs::msg::String();
            mode_msg.data = "RECON";
            mode_pub_->publish(mode_msg);
            RCLCPP_INFO(this->get_logger(),
                "Y button pressed — entering RECON mode");
        }

        if (buttonJustPressed(msg, btn_x_)) {
            // Save current map
            auto event = std_msgs::msg::String();
            event.data = "SAVE_MAP";
            events_pub_->publish(event);
            RCLCPP_INFO(this->get_logger(),
                "X button pressed — map save requested");
        }

        // Store current buttons for next edge comparison
        prev_buttons_.assign(msg->buttons.begin(), msg->buttons.end());

        // Only publish cmd_vel in MANUAL mode
        if (current_mode_ != "MANUAL") {
            return;
        }

        // Map axes to velocities
        // Note: joy_linux + xpadneo reports Y axes inverted (up = negative),
        // so we negate left_y for forward = positive linear.x
        if (static_cast<size_t>(axis_left_y_) < msg->axes.size() &&
            static_cast<size_t>(axis_right_x_) < msg->axes.size())
        {
            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = applyDeadzone(
                -msg->axes[axis_left_y_]) * max_linear_;
            twist.angular.z = applyDeadzone(
                msg->axes[axis_right_x_]) * max_angular_;
            cmd_vel_pub_->publish(twist);
        }
    }

    /**
     * @brief Check if a button index is valid and currently pressed.
     */
    bool isButtonPressed(
        const sensor_msgs::msg::Joy::SharedPtr & msg, int index) const
    {
        if (index < 0 ||
            static_cast<size_t>(index) >= msg->buttons.size()) {
            return false;
        }
        return msg->buttons[index] != 0;
    }

    /**
     * @brief Edge-detected button press (only fires on 0→1 transition).
     */
    bool buttonJustPressed(
        const sensor_msgs::msg::Joy::SharedPtr & msg, int index) const
    {
        if (index < 0 ||
            static_cast<size_t>(index) >= msg->buttons.size()) {
            return false;
        }
        int cur = msg->buttons[index];
        int prev = (static_cast<size_t>(index) < prev_buttons_.size())
                   ? prev_buttons_[index] : 0;
        return cur != 0 && prev == 0;
    }

    // Parameters
    int axis_left_y_;
    int axis_right_x_;
    int btn_start_;
    int btn_b_;
    int btn_y_;
    int btn_x_;
    double max_linear_;
    double max_angular_;
    double deadzone_;

    // State
    std::string current_mode_;
    std::vector<int> prev_buttons_;  // For edge detection

    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyControlNode>());
    rclcpp::shutdown();
    return 0;
}
