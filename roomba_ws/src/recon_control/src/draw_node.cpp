/**
 * @file draw_node.cpp
 * @brief Web-driven drawing canvas for testing map persistence.
 *
 * Maintains a 2-D occupancy grid that callers paint via plain-text commands
 * on /draw/command (std_msgs/String). The grid is republished on /map at a
 * fixed rate so the web UI shows the canvas live, and a SAVE_MAP event is
 * emitted on /robot/events when the caller requests it — the same path
 * the real-hardware SAVE button will use in H2.
 *
 * Command grammar (whitespace-separated, case-insensitive):
 *   paint <x> <y>   — set cells around (x, y) to 100 (occupied)
 *   erase <x> <y>   — set cells around (x, y) to 0   (free)
 *   clear           — reset every cell to -1 (unknown)
 *   brush <n>       — set brush radius (clamped 1..10)
 *   save            — emit SAVE_MAP on /robot/events
 *
 * Replaces an earlier Xbox-controller (joy) implementation. The web UI in
 * H5 will drive the same /draw/command topic via the existing ROS bridge.
 */

#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DrawNode : public rclcpp::Node
{
public:
    DrawNode()
    : Node("draw_node")
    {
        declare_parameter<int>("draw.grid_width", 100);
        declare_parameter<int>("draw.grid_height", 100);
        declare_parameter<double>("draw.resolution", 0.05);
        declare_parameter<int>("draw.brush_size", 2);
        declare_parameter<double>("draw.publish_rate", 5.0);

        grid_w_ = get_parameter("draw.grid_width").as_int();
        grid_h_ = get_parameter("draw.grid_height").as_int();
        resolution_ = get_parameter("draw.resolution").as_double();
        brush_size_ = get_parameter("draw.brush_size").as_int();
        const double pub_rate = get_parameter("draw.publish_rate").as_double();

        grid_.assign(
            static_cast<size_t>(grid_w_) * static_cast<size_t>(grid_h_), -1);

        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        event_pub_ = create_publisher<std_msgs::msg::String>(
            "/robot/events", 10);

        cmd_sub_ = create_subscription<std_msgs::msg::String>(
            "/draw/command", 10,
            std::bind(&DrawNode::commandCallback, this, std::placeholders::_1));

        const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / pub_rate));
        map_timer_ = create_wall_timer(
            period, std::bind(&DrawNode::publishMap, this));

        RCLCPP_INFO(get_logger(),
            "draw_node started — grid %dx%d @ %.2fm  brush=%d  rate=%.0fHz",
            grid_w_, grid_h_, resolution_, brush_size_, pub_rate);
        RCLCPP_INFO(get_logger(),
            "Listening on /draw/command — commands: paint, erase, clear, "
            "brush, save");
    }

private:
    // =====================================================================
    // /draw/command parsing
    // =====================================================================

    void commandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::istringstream iss(msg->data);
        std::string verb;
        if (!(iss >> verb)) {
            return;
        }
        std::transform(verb.begin(), verb.end(), verb.begin(),
            [](unsigned char c) { return std::tolower(c); });

        if (verb == "paint" || verb == "erase") {
            int x = 0;
            int y = 0;
            if (!(iss >> x >> y)) {
                RCLCPP_WARN(get_logger(),
                    "Bad %s command — expected '<verb> <x> <y>'", verb.c_str());
                return;
            }
            paint(x, y, verb == "paint" ? 100 : 0);
            return;
        }
        if (verb == "clear") {
            std::fill(grid_.begin(), grid_.end(), -1);
            RCLCPP_INFO(get_logger(), "Grid cleared");
            return;
        }
        if (verb == "brush") {
            int n = 0;
            if (!(iss >> n)) {
                RCLCPP_WARN(get_logger(),
                    "Bad brush command — expected 'brush <n>'");
                return;
            }
            brush_size_ = std::clamp(n, 1, 10);
            RCLCPP_INFO(get_logger(), "Brush size set to %d", brush_size_);
            return;
        }
        if (verb == "save") {
            std_msgs::msg::String ev;
            ev.data = "SAVE_MAP";
            event_pub_->publish(ev);
            RCLCPP_INFO(get_logger(), "SAVE_MAP requested via /draw/command");
            return;
        }
        RCLCPP_WARN(get_logger(), "Unknown draw command: '%s'", verb.c_str());
    }

    // =====================================================================
    // Grid mutation + publishing
    // =====================================================================

    void paint(int cx, int cy, int8_t value)
    {
        const int r = brush_size_;
        for (int dy = -r + 1; dy < r; ++dy) {
            for (int dx = -r + 1; dx < r; ++dx) {
                const int px = cx + dx;
                const int py = cy + dy;
                if (px >= 0 && px < grid_w_ && py >= 0 && py < grid_h_) {
                    grid_[static_cast<size_t>(py * grid_w_ + px)] = value;
                }
            }
        }
    }

    void publishMap()
    {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.info.resolution = static_cast<float>(resolution_);
        msg.info.width = static_cast<uint32_t>(grid_w_);
        msg.info.height = static_cast<uint32_t>(grid_h_);
        msg.info.origin.position.x = -(grid_w_ * resolution_) / 2.0;
        msg.info.origin.position.y = -(grid_h_ * resolution_) / 2.0;
        msg.data.assign(grid_.begin(), grid_.end());
        map_pub_->publish(msg);
    }

    int grid_w_{100};
    int grid_h_{100};
    double resolution_{0.05};
    int brush_size_{2};
    std::vector<int8_t> grid_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr map_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawNode>());
    rclcpp::shutdown();
    return 0;
}
