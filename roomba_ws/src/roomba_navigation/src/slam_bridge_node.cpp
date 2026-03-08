/**
 * @file slam_bridge_node.cpp
 * @brief SLAM bridge — converts individual ultrasonic Range messages into a
 *        synthesised LaserScan for consumption by slam_toolbox.
 *
 * Real-time C++17 node. Subscribes to /sensors/{front,left,right} and publishes
 * a combined /scan LaserScan message.
 */

#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"

/**
 * @class SlamBridgeNode
 * @brief Synthesises a LaserScan from three ultrasonic Range messages for SLAM.
 */
class SlamBridgeNode : public rclcpp::Node
{
public:
    SlamBridgeNode()
    : Node("slam_bridge_node")
    {
        // Declare LaserScan parameters
        this->declare_parameter<double>("angle_min", -1.0472);  // -60 deg
        this->declare_parameter<double>("angle_max", 1.0472);   //  60 deg
        this->declare_parameter<double>("angle_increment", 1.0472);  // 60 deg
        this->declare_parameter<double>("range_min", 0.02);
        this->declare_parameter<double>("range_max", 4.0);
        this->declare_parameter<double>("scan_rate_hz", 10.0);
        this->declare_parameter<std::string>("scan_frame_id", "base_link");

        // Retrieve parameters
        angle_min_ = this->get_parameter("angle_min").as_double();
        angle_max_ = this->get_parameter("angle_max").as_double();
        angle_increment_ = this->get_parameter(
            "angle_increment").as_double();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        scan_rate_hz_ = this->get_parameter("scan_rate_hz").as_double();
        scan_frame_id_ = this->get_parameter("scan_frame_id").as_string();

        // Initialise latest ranges to inf
        latest_ranges_.fill(std::numeric_limits<float>::infinity());

        // Subscribers for each ultrasonic sensor
        sub_front_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/sensors/front", 10,
            [this](const sensor_msgs::msg::Range::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_ranges_[1] = msg->range;  // Center = index 1
            });

        sub_left_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/sensors/left", 10,
            [this](const sensor_msgs::msg::Range::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_ranges_[2] = msg->range;  // Left = index 2
            });

        sub_right_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/sensors/right", 10,
            [this](const sensor_msgs::msg::Range::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                latest_ranges_[0] = msg->range;  // Right = index 0
            });

        // Publisher
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", 10);

        // Timer to publish synthesised scan
        auto period = std::chrono::duration<double>(
            1.0 / scan_rate_hz_);
        scan_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SlamBridgeNode::publishScan, this));

        RCLCPP_WARN(this->get_logger(),
            "Ultrasonic SLAM has limited resolution. "
            "Consider LIDAR for production use.");
        RCLCPP_INFO(this->get_logger(),
            "slam_bridge_node started — scan rate: %.1f Hz",
            scan_rate_hz_);
    }

private:
    /**
     * @brief Publish a synthesised LaserScan from the latest Range data.
     */
    void publishScan()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        auto scan = sensor_msgs::msg::LaserScan();
        scan.header.stamp = this->now();
        scan.header.frame_id = scan_frame_id_;
        scan.angle_min = static_cast<float>(angle_min_);
        scan.angle_max = static_cast<float>(angle_max_);
        scan.angle_increment = static_cast<float>(angle_increment_);
        scan.time_increment = 0.0f;
        scan.scan_time = static_cast<float>(1.0 / scan_rate_hz_);
        scan.range_min = static_cast<float>(range_min_);
        scan.range_max = static_cast<float>(range_max_);

        // 3 beams: right (-60°), front (0°), left (+60°)
        scan.ranges.assign(
            latest_ranges_.begin(), latest_ranges_.end());

        scan_pub_->publish(scan);
    }

    // Parameters
    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double range_min_;
    double range_max_;
    double scan_rate_hz_;
    std::string scan_frame_id_;

    // State — 3 range values: [right, front, left]
    std::array<float, 3> latest_ranges_;
    std::mutex mutex_;

    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_front_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_left_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_right_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr scan_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
