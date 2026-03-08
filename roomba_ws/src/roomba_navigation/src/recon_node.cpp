/**
 * @file recon_node.cpp
 * @brief Autonomous reconnaissance node — implements frontier-based exploration.
 *
 * Real-time C++17 node. When /robot/mode is RECON, reads the OccupancyGrid,
 * identifies frontiers, and sends navigation goals to nav2.
 */

#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief Represents a frontier cell in the occupancy grid.
 */
struct FrontierCell
{
    int x;
    int y;
    double world_x;
    double world_y;
    double distance_from_origin;
};

/**
 * @class ReconNode
 * @brief Frontier-based autonomous exploration within a configurable radius.
 */
class ReconNode : public rclcpp::Node
{
public:
    ReconNode()
    : Node("recon_node"),
      mode_("IDLE"),
      has_map_(false),
      origin_set_(false)
    {
        // Declare parameters
        this->declare_parameter<double>("recon_radius", 5.0);
        this->declare_parameter<double>("frontier_check_rate_hz", 1.0);
        this->declare_parameter<double>("goal_tolerance", 0.3);

        recon_radius_ = this->get_parameter("recon_radius").as_double();
        check_rate_hz_ = this->get_parameter(
            "frontier_check_rate_hz").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

        if (recon_radius_ <= 0.0) {
            throw std::runtime_error("recon_radius must be > 0");
        }

        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&ReconNode::mapCallback, this,
                std::placeholders::_1));

        mode_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot/mode", 10,
            std::bind(&ReconNode::modeCallback, this,
                std::placeholders::_1));

        // Publishers
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);
        events_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/robot/events", 10);

        // Frontier check timer
        auto period = std::chrono::duration<double>(
            1.0 / check_rate_hz_);
        frontier_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&ReconNode::frontierCheckCallback, this));

        RCLCPP_INFO(this->get_logger(),
            "recon_node started — radius: %.1f m, "
            "check rate: %.1f Hz",
            recon_radius_, check_rate_hz_);
    }

private:
    /**
     * @brief Callback for /robot/mode topic.
     */
    void modeCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string new_mode = msg->data;

        if (new_mode == "RECON" && mode_ != "RECON") {
            RCLCPP_INFO(this->get_logger(), "Entering RECON mode");
            // Snapshot origin pose
            // TODO: Read actual pose from /odom or /amcl_pose
            origin_x_ = 0.0;
            origin_y_ = 0.0;
            origin_set_ = true;
        } else if (new_mode != "RECON" && mode_ == "RECON") {
            RCLCPP_INFO(this->get_logger(),
                "Exiting RECON mode — triggering map save");
            auto event = std_msgs::msg::String();
            event.data = "SAVE_MAP";
            events_pub_->publish(event);
            origin_set_ = false;
        }

        mode_ = new_mode;
    }

    /**
     * @brief Callback for /map updates.
     */
    void mapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_map_ = *msg;
        has_map_ = true;
    }

    /**
     * @brief Periodic callback — finds frontiers and sends goals.
     */
    void frontierCheckCallback()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (mode_ != "RECON" || !has_map_ || !origin_set_) {
            return;
        }

        auto frontiers = findFrontiers();

        if (frontiers.empty()) {
            RCLCPP_INFO(this->get_logger(),
                "No frontiers found — exploration complete");
            auto event = std_msgs::msg::String();
            event.data = "RECON_COMPLETE";
            events_pub_->publish(event);
            return;
        }

        // Select nearest frontier within radius
        const FrontierCell * best = nullptr;
        double best_dist = std::numeric_limits<double>::max();

        for (const auto & f : frontiers) {
            if (f.distance_from_origin > recon_radius_) {
                continue;  // Outside allowed radius
            }
            // Distance from current position (approximated by origin for now)
            // TODO: Use actual robot pose for distance calculation
            double dist = std::hypot(
                f.world_x - origin_x_, f.world_y - origin_y_);
            if (dist < best_dist) {
                best_dist = dist;
                best = &f;
            }
        }

        if (best == nullptr) {
            RCLCPP_INFO(this->get_logger(),
                "No reachable frontiers within radius %.1f m",
                recon_radius_);
            return;
        }

        // Publish goal
        auto goal = geometry_msgs::msg::PoseStamped();
        goal.header.stamp = this->now();
        goal.header.frame_id = "map";
        goal.pose.position.x = best->world_x;
        goal.pose.position.y = best->world_y;
        goal.pose.orientation.w = 1.0;

        goal_pub_->publish(goal);
        RCLCPP_DEBUG(this->get_logger(),
            "Goal published: (%.2f, %.2f) dist: %.2f m",
            best->world_x, best->world_y, best_dist);
    }

    /**
     * @brief Find frontier cells in the occupancy grid.
     *        Frontiers are free cells adjacent to unknown cells.
     */
    std::vector<FrontierCell> findFrontiers() const
    {
        std::vector<FrontierCell> frontiers;

        const auto & info = latest_map_.info;
        const auto & data = latest_map_.data;
        int width = static_cast<int>(info.width);
        int height = static_cast<int>(info.height);

        for (int y = 1; y < height - 1; ++y) {
            for (int x = 1; x < width - 1; ++x) {
                int idx = y * width + x;

                // Must be free space (0)
                if (data[idx] != 0) {
                    continue;
                }

                // Check 4-connected neighbours for unknown (-1)
                bool adjacent_unknown = false;
                int neighbours[] = {
                    (y - 1) * width + x,
                    (y + 1) * width + x,
                    y * width + (x - 1),
                    y * width + (x + 1)
                };

                for (int n : neighbours) {
                    if (data[n] == -1) {
                        adjacent_unknown = true;
                        break;
                    }
                }

                if (adjacent_unknown) {
                    FrontierCell fc;
                    fc.x = x;
                    fc.y = y;
                    fc.world_x = info.origin.position.x +
                        (x + 0.5) * info.resolution;
                    fc.world_y = info.origin.position.y +
                        (y + 0.5) * info.resolution;
                    fc.distance_from_origin = std::hypot(
                        fc.world_x - origin_x_,
                        fc.world_y - origin_y_);
                    frontiers.push_back(fc);
                }
            }
        }

        return frontiers;
    }

    // Parameters
    double recon_radius_;
    double check_rate_hz_;
    double goal_tolerance_;

    // State
    std::string mode_;
    bool has_map_;
    bool origin_set_;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    nav_msgs::msg::OccupancyGrid latest_map_;
    std::mutex mutex_;

    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
    rclcpp::TimerBase::SharedPtr frontier_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ReconNode>());
    } catch (const std::runtime_error & e) {
        RCLCPP_FATAL(rclcpp::get_logger("recon_node"),
            "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
