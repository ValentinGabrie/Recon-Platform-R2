/**
 * @file recon_node.cpp
 * @brief Autonomous reconnaissance — fuzzy frontier-based exploration.
 *
 * When /robot/mode is RECON, reads the OccupancyGrid, clusters frontier
 * cells, and uses a Mamdani fuzzy inference system to evaluate each cluster
 * based on distance, size, and heading alignment.  The most desirable
 * frontier is published as a navigation goal.
 *
 * Fuzzy inputs:
 *   - distance:  robot → cluster centroid (metres)
 *   - size:      number of cells in the frontier cluster
 *   - alignment: |heading error| to cluster centroid (radians, 0 = ahead)
 *
 * Fuzzy output:
 *   - desirability (0–100 scale), defuzzified via centroid method.
 *
 * Real-time C++17 node — part of roomba_navigation.
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

// ================================================================
// Fuzzy membership helpers
// ================================================================

/**
 * @brief Trapezoidal membership function.
 *        Rises a→b, flat b→c, falls c→d.
 *        Left-shoulder: a == b.  Right-shoulder: c == d.
 */
inline double trapmf(double x, double a, double b, double c, double d)
{
    if (x <= a || x >= d) { return 0.0; }
    if (x >= b && x <= c) { return 1.0; }
    if (x < b) { return (b > a) ? (x - a) / (b - a) : 1.0; }
    return (d > c) ? (d - x) / (d - c) : 1.0;
}

/** @brief Triangular membership — special case of trapezoidal. */
inline double trimf(double x, double a, double b, double c)
{
    return trapmf(x, a, b, b, c);
}

// ================================================================
// Data structures
// ================================================================

/**
 * @brief A cluster of adjacent frontier cells with fuzzy evaluation.
 */
struct FrontierCluster
{
    double centroid_x = 0.0;
    double centroid_y = 0.0;
    int size = 0;
    double distance = 0.0;
    double alignment = 0.0;
    double desirability = 0.0;
};

// ================================================================
// ReconNode
// ================================================================

class ReconNode : public rclcpp::Node
{
public:
    ReconNode()
    : Node("recon_node"),
      mode_("IDLE"),
      has_map_(false),
      origin_set_(false),
      has_odom_(false)
    {
        // ---- Exploration parameters ----
        this->declare_parameter<double>("recon_radius", 5.0);
        this->declare_parameter<double>("frontier_check_rate_hz", 1.0);
        this->declare_parameter<double>("goal_tolerance", 0.3);
        this->declare_parameter<int>("min_cluster_size", 3);

        // ---- Fuzzy MF parameters (trapezoidal: a, b, c, d) ----
        // Distance (metres)
        this->declare_parameter<std::vector<double>>(
            "fuzzy_dist_near", {0.0, 0.0, 1.0, 2.5});
        this->declare_parameter<std::vector<double>>(
            "fuzzy_dist_medium", {1.5, 3.0, 3.0, 5.0});
        this->declare_parameter<std::vector<double>>(
            "fuzzy_dist_far", {4.0, 6.0, 100.0, 100.0});

        // Cluster size (cell count)
        this->declare_parameter<std::vector<double>>(
            "fuzzy_size_small", {0.0, 0.0, 3.0, 8.0});
        this->declare_parameter<std::vector<double>>(
            "fuzzy_size_medium", {5.0, 15.0, 15.0, 30.0});
        this->declare_parameter<std::vector<double>>(
            "fuzzy_size_large", {20.0, 40.0, 1000.0, 1000.0});

        // Heading alignment (|error| radians, 0 = perfectly aligned)
        this->declare_parameter<std::vector<double>>(
            "fuzzy_align_aligned", {0.0, 0.0, 0.3, 0.8});
        this->declare_parameter<std::vector<double>>(
            "fuzzy_align_oblique", {0.5, 1.2, 1.2, 2.0});
        this->declare_parameter<std::vector<double>>(
            "fuzzy_align_behind", {1.5, 2.2, 3.15, 3.15});

        // Retrieve exploration params
        recon_radius_ = this->get_parameter("recon_radius").as_double();
        check_rate_hz_ = this->get_parameter(
            "frontier_check_rate_hz").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        min_cluster_size_ = this->get_parameter(
            "min_cluster_size").as_int();

        if (recon_radius_ <= 0.0) {
            throw std::runtime_error("recon_radius must be > 0");
        }

        // Retrieve fuzzy MF params
        mf_dn_ = this->get_parameter("fuzzy_dist_near").as_double_array();
        mf_dm_ = this->get_parameter("fuzzy_dist_medium").as_double_array();
        mf_df_ = this->get_parameter("fuzzy_dist_far").as_double_array();
        mf_ss_ = this->get_parameter("fuzzy_size_small").as_double_array();
        mf_sm_ = this->get_parameter("fuzzy_size_medium").as_double_array();
        mf_sl_ = this->get_parameter("fuzzy_size_large").as_double_array();
        mf_aa_ = this->get_parameter(
            "fuzzy_align_aligned").as_double_array();
        mf_ao_ = this->get_parameter(
            "fuzzy_align_oblique").as_double_array();
        mf_ab_ = this->get_parameter(
            "fuzzy_align_behind").as_double_array();

        // Validate all MFs have 4 elements
        for (const auto * mf : {&mf_dn_, &mf_dm_, &mf_df_,
                                &mf_ss_, &mf_sm_, &mf_sl_,
                                &mf_aa_, &mf_ao_, &mf_ab_})
        {
            if (mf->size() != 4) {
                throw std::runtime_error(
                    "All fuzzy_* parameters must have exactly 4 values "
                    "(a, b, c, d)");
            }
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

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&ReconNode::odomCallback, this,
                std::placeholders::_1));

        events_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot/events", 10,
            std::bind(&ReconNode::eventsCallback, this,
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
            "recon_node started — fuzzy frontier selection, "
            "radius: %.1f m, min_cluster: %d, check rate: %.1f Hz",
            recon_radius_, min_cluster_size_, check_rate_hz_);
    }

private:
    // ================================================================
    // ROS2 callbacks
    // ================================================================

    void modeCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string new_mode = msg->data;

        if (new_mode == "RECON" && mode_ != "RECON") {
            RCLCPP_INFO(this->get_logger(), "Entering RECON mode");
            origin_x_ = robot_x_;
            origin_y_ = robot_y_;
            origin_set_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Origin set to (%.2f, %.2f)", origin_x_, origin_y_);
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

    void mapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_map_ = *msg;
        has_map_ = true;
    }

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        robot_theta_ = 2.0 * std::atan2(qz, qw);
        has_odom_ = true;
    }

    void eventsCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "GOAL_FAILED") {
            std::lock_guard<std::mutex> lock(mutex_);
            // Blacklist current goal area for 30 seconds
            if (last_goal_valid_) {
                blacklisted_.push_back(
                    {last_goal_x_, last_goal_y_, this->now()});
                RCLCPP_INFO(this->get_logger(),
                    "Blacklisted goal area (%.2f, %.2f) for 30s",
                    last_goal_x_, last_goal_y_);
            }
        }
    }

    // ================================================================
    // Frontier exploration with fuzzy logic
    // ================================================================

    /**
     * @brief Periodic callback — find, cluster, evaluate, and navigate
     *        to the best frontier.
     */
    void frontierCheckCallback()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (mode_ != "RECON" || !has_map_ || !origin_set_ || !has_odom_) {
            return;
        }

        // 1. Find frontier cells
        auto frontier_cells = findFrontierCells();
        if (frontier_cells.empty()) {
            RCLCPP_INFO(this->get_logger(),
                "No frontiers found — exploration complete");
            auto event = std_msgs::msg::String();
            event.data = "RECON_COMPLETE";
            events_pub_->publish(event);
            mode_ = "IDLE";  // Stop further checks until re-triggered
            return;
        }

        // 2. Cluster adjacent frontier cells
        auto clusters = clusterFrontiers(frontier_cells);

        // 3. Evaluate each cluster with fuzzy logic
        const FrontierCluster * best = nullptr;
        double best_score = -1.0;
        int filtered_radius = 0;
        int filtered_small = 0;
        int filtered_blacklist = 0;

        // Expire old blacklist entries (> 30s)
        auto now = this->now();
        blacklisted_.erase(
            std::remove_if(blacklisted_.begin(), blacklisted_.end(),
                [&now](const BlacklistEntry & e) {
                    return (now - e.time).seconds() > 30.0;
                }),
            blacklisted_.end());

        for (auto & cluster : clusters) {
            // Filter: outside recon_radius
            double dist_from_origin = std::hypot(
                cluster.centroid_x - origin_x_,
                cluster.centroid_y - origin_y_);
            if (dist_from_origin > recon_radius_) {
                ++filtered_radius;
                continue;
            }

            // Filter: too small
            if (cluster.size < min_cluster_size_) {
                ++filtered_small;
                continue;
            }

            // Filter: blacklisted (recently failed goal area)
            bool is_blacklisted = false;
            for (const auto & bl : blacklisted_) {
                if (std::hypot(cluster.centroid_x - bl.x,
                               cluster.centroid_y - bl.y) < 1.0)
                {
                    is_blacklisted = true;
                    break;
                }
            }
            if (is_blacklisted) {
                ++filtered_blacklist;
                continue;
            }

            // Distance from robot
            cluster.distance = std::hypot(
                cluster.centroid_x - robot_x_,
                cluster.centroid_y - robot_y_);

            // Heading alignment
            double desired = std::atan2(
                cluster.centroid_y - robot_y_,
                cluster.centroid_x - robot_x_);
            double err = desired - robot_theta_;
            while (err > M_PI) { err -= 2.0 * M_PI; }
            while (err < -M_PI) { err += 2.0 * M_PI; }
            cluster.alignment = std::abs(err);

            // Fuzzy evaluation
            cluster.desirability = fuzzyDesirability(
                cluster.distance,
                static_cast<double>(cluster.size),
                cluster.alignment);

            if (cluster.desirability > best_score) {
                best_score = cluster.desirability;
                best = &cluster;
            }
        }

        if (best == nullptr) {
            RCLCPP_INFO(this->get_logger(),
                "No valid frontier clusters within radius %.1f m "
                "(%zu clusters: %d outside radius, %d too small, "
                "%d blacklisted)",
                recon_radius_, clusters.size(),
                filtered_radius, filtered_small, filtered_blacklist);
            return;
        }

        // 4. Publish goal
        auto goal = geometry_msgs::msg::PoseStamped();
        goal.header.stamp = this->now();
        goal.header.frame_id = "map";
        goal.pose.position.x = best->centroid_x;
        goal.pose.position.y = best->centroid_y;
        goal.pose.orientation.w = 1.0;

        goal_pub_->publish(goal);
        last_goal_x_ = best->centroid_x;
        last_goal_y_ = best->centroid_y;
        last_goal_valid_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Fuzzy goal: (%.2f, %.2f) score=%.1f dist=%.2f "
            "size=%d align=%.2f rad=%.2f",
            best->centroid_x, best->centroid_y,
            best->desirability, best->distance,
            best->size, best->alignment,
            std::hypot(best->centroid_x - origin_x_,
                       best->centroid_y - origin_y_));
    }

    // ================================================================
    // Frontier detection
    // ================================================================

    /**
     * @brief Find all frontier cells in the occupancy grid.
     *        A frontier cell is a free cell (0) adjacent to unknown (-1).
     * @return Vector of (grid_x, grid_y) pairs.
     */
    std::vector<std::pair<int, int>> findFrontierCells() const
    {
        std::vector<std::pair<int, int>> frontiers;

        const auto & info = latest_map_.info;
        const auto & data = latest_map_.data;
        int width = static_cast<int>(info.width);
        int height = static_cast<int>(info.height);

        for (int y = 1; y < height - 1; ++y) {
            for (int x = 1; x < width - 1; ++x) {
                int idx = y * width + x;
                if (data[idx] != 0) {
                    continue;
                }

                // 4-connected neighbour check for unknown
                const int neighbours[] = {
                    (y - 1) * width + x,
                    (y + 1) * width + x,
                    y * width + (x - 1),
                    y * width + (x + 1)
                };

                for (int n : neighbours) {
                    if (data[n] == -1) {
                        frontiers.emplace_back(x, y);
                        break;
                    }
                }
            }
        }

        return frontiers;
    }

    // ================================================================
    // Frontier clustering (BFS on adjacent frontier cells)
    // ================================================================

    /**
     * @brief Group adjacent frontier cells into clusters using BFS.
     *        Computes cluster centroid in world coordinates.
     */
    std::vector<FrontierCluster> clusterFrontiers(
        const std::vector<std::pair<int, int>> & cells) const
    {
        const auto & info = latest_map_.info;
        int width = static_cast<int>(info.width);
        int height = static_cast<int>(info.height);

        // Build a set for O(1) lookup
        std::vector<bool> is_frontier(
            static_cast<size_t>(width) * height, false);
        for (const auto & [fx, fy] : cells) {
            is_frontier[static_cast<size_t>(fy * width + fx)] = true;
        }

        std::vector<bool> visited(
            static_cast<size_t>(width) * height, false);
        std::vector<FrontierCluster> clusters;

        for (const auto & [sx, sy] : cells) {
            size_t si = static_cast<size_t>(sy * width + sx);
            if (visited[si]) {
                continue;
            }

            // BFS from this frontier cell
            FrontierCluster cluster;
            double sum_x = 0.0;
            double sum_y = 0.0;
            std::queue<std::pair<int, int>> q;
            q.push({sx, sy});
            visited[si] = true;

            while (!q.empty()) {
                auto [cx, cy] = q.front();
                q.pop();

                cluster.size++;
                double wx = info.origin.position.x +
                    (cx + 0.5) * info.resolution;
                double wy = info.origin.position.y +
                    (cy + 0.5) * info.resolution;
                sum_x += wx;
                sum_y += wy;

                // Explore 4-connected frontier neighbours
                const int dx[] = {1, -1, 0, 0};
                const int dy[] = {0, 0, 1, -1};
                for (int d = 0; d < 4; ++d) {
                    int nx = cx + dx[d];
                    int ny = cy + dy[d];
                    if (nx < 0 || nx >= width ||
                        ny < 0 || ny >= height)
                    {
                        continue;
                    }
                    size_t ni = static_cast<size_t>(ny * width + nx);
                    if (!visited[ni] && is_frontier[ni]) {
                        visited[ni] = true;
                        q.push({nx, ny});
                    }
                }
            }

            cluster.centroid_x = sum_x / cluster.size;
            cluster.centroid_y = sum_y / cluster.size;
            clusters.push_back(cluster);
        }

        return clusters;
    }

    // ================================================================
    // Mamdani fuzzy inference
    // ================================================================

    /**
     * @brief Evaluate a frontier cluster's desirability using Mamdani
     *        fuzzy inference with 15 rules.
     *
     * @param dist     Distance from robot to cluster centroid (m).
     * @param size     Number of frontier cells in cluster.
     * @param align    |heading error| to centroid (radians, 0=ahead).
     * @return         Crisp desirability score [0, 100].
     */
    double fuzzyDesirability(double dist, double size,
                             double align) const
    {
        // ---- Fuzzify inputs ----
        // Distance
        double d_near   = trapmf(dist,  mf_dn_[0], mf_dn_[1],
                                        mf_dn_[2], mf_dn_[3]);
        double d_medium = trapmf(dist,  mf_dm_[0], mf_dm_[1],
                                        mf_dm_[2], mf_dm_[3]);
        double d_far    = trapmf(dist,  mf_df_[0], mf_df_[1],
                                        mf_df_[2], mf_df_[3]);

        // Cluster size
        double s_small  = trapmf(size,  mf_ss_[0], mf_ss_[1],
                                        mf_ss_[2], mf_ss_[3]);
        double s_medium = trapmf(size,  mf_sm_[0], mf_sm_[1],
                                        mf_sm_[2], mf_sm_[3]);
        double s_large  = trapmf(size,  mf_sl_[0], mf_sl_[1],
                                        mf_sl_[2], mf_sl_[3]);

        // Heading alignment
        double a_aligned = trapmf(align, mf_aa_[0], mf_aa_[1],
                                         mf_aa_[2], mf_aa_[3]);
        double a_behind  = trapmf(align, mf_ab_[0], mf_ab_[1],
                                         mf_ab_[2], mf_ab_[3]);

        // ---- Apply rules (AND = min, aggregation = max) ----
        // Output sets: 0=VERY_LOW, 1=LOW, 2=MEDIUM, 3=HIGH, 4=VERY_HIGH
        double agg[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

        // R1:  NEAR ∧ LARGE ∧ ALIGNED    → VERY_HIGH
        agg[4] = std::max(agg[4],
            std::min({d_near, s_large, a_aligned}));
        // R2:  NEAR ∧ LARGE              → HIGH
        agg[3] = std::max(agg[3],
            std::min(d_near, s_large));
        // R3:  NEAR ∧ MEDIUM_S ∧ ALIGNED → HIGH
        agg[3] = std::max(agg[3],
            std::min({d_near, s_medium, a_aligned}));
        // R4:  NEAR ∧ MEDIUM_S           → MEDIUM
        agg[2] = std::max(agg[2],
            std::min(d_near, s_medium));
        // R5:  NEAR ∧ SMALL ∧ ALIGNED    → MEDIUM
        agg[2] = std::max(agg[2],
            std::min({d_near, s_small, a_aligned}));
        // R6:  NEAR ∧ SMALL              → LOW
        agg[1] = std::max(agg[1],
            std::min(d_near, s_small));
        // R7:  MED_D ∧ LARGE ∧ ALIGNED   → HIGH
        agg[3] = std::max(agg[3],
            std::min({d_medium, s_large, a_aligned}));
        // R8:  MED_D ∧ LARGE             → MEDIUM
        agg[2] = std::max(agg[2],
            std::min(d_medium, s_large));
        // R9:  MED_D ∧ MED_S ∧ ALIGNED   → MEDIUM
        agg[2] = std::max(agg[2],
            std::min({d_medium, s_medium, a_aligned}));
        // R10: MED_D ∧ MED_S             → LOW
        agg[1] = std::max(agg[1],
            std::min(d_medium, s_medium));
        // R11: MED_D ∧ SMALL             → LOW
        agg[1] = std::max(agg[1],
            std::min(d_medium, s_small));
        // R12: FAR ∧ LARGE ∧ ALIGNED     → MEDIUM
        agg[2] = std::max(agg[2],
            std::min({d_far, s_large, a_aligned}));
        // R13: FAR ∧ LARGE               → LOW
        agg[1] = std::max(agg[1],
            std::min(d_far, s_large));
        // R14: FAR ∧ BEHIND              → VERY_LOW
        agg[0] = std::max(agg[0],
            std::min(d_far, a_behind));
        // R15: FAR ∧ SMALL               → VERY_LOW
        agg[0] = std::max(agg[0],
            std::min(d_far, s_small));

        // ---- Defuzzify (centroid method) ----
        // Output MFs on universe [0, 100]
        //   VERY_LOW:  trimf(0, 10, 20)
        //   LOW:       trimf(15, 30, 45)
        //   MEDIUM:    trimf(35, 50, 65)
        //   HIGH:      trimf(55, 70, 85)
        //   VERY_HIGH: trimf(80, 90, 100)
        constexpr int N_SAMPLES = 101;
        double numerator = 0.0;
        double denominator = 0.0;

        for (int i = 0; i < N_SAMPLES; ++i) {
            double x = static_cast<double>(i);
            double mu = 0.0;
            mu = std::max(mu, std::min(agg[0], trimf(x, 0, 10, 20)));
            mu = std::max(mu, std::min(agg[1], trimf(x, 15, 30, 45)));
            mu = std::max(mu, std::min(agg[2], trimf(x, 35, 50, 65)));
            mu = std::max(mu, std::min(agg[3], trimf(x, 55, 70, 85)));
            mu = std::max(mu, std::min(agg[4], trimf(x, 80, 90, 100)));
            numerator += x * mu;
            denominator += mu;
        }

        return (denominator > 0.0) ? numerator / denominator : 0.0;
    }

    // ================================================================
    // Member variables
    // ================================================================

    // Exploration params
    double recon_radius_;
    double check_rate_hz_;
    double goal_tolerance_;
    int min_cluster_size_;

    // Fuzzy MF params (trapezoidal: a, b, c, d)
    std::vector<double> mf_dn_, mf_dm_, mf_df_;  // distance
    std::vector<double> mf_ss_, mf_sm_, mf_sl_;  // size
    std::vector<double> mf_aa_, mf_ao_, mf_ab_;  // alignment

    // State
    std::string mode_;
    bool has_map_;
    bool origin_set_;
    bool has_odom_;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    double last_goal_x_ = 0.0;
    double last_goal_y_ = 0.0;
    bool last_goal_valid_ = false;
    nav_msgs::msg::OccupancyGrid latest_map_;
    std::mutex mutex_;

    // Blacklist for failed goals (unreachable areas)
    struct BlacklistEntry {
        double x;
        double y;
        rclcpp::Time time;
    };
    std::vector<BlacklistEntry> blacklisted_;

    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr events_sub_;
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
