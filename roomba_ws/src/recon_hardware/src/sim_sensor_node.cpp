/**
 * @file sim_sensor_node.cpp
 * @brief Simulated sensor node — generates a random room and raycasts
 *        ultrasonic + LIDAR readings from the robot's current pose.
 *
 * Drop-in replacement for esp32_sensor_node.  Publishes on the same topics
 * with the same message types.  Also publishes a simulated LaserScan
 * directly (360-beam) and a ground-truth OccupancyGrid for debug display.
 *
 * Real-time C++17 node — part of roomba_hardware.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>
#include <queue>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

/**
 * @class SimSensorNode
 * @brief Generates a random room with obstacles and raycasts sensor data
 *        from the robot's current pose obtained via /odom.
 */
class SimSensorNode : public rclcpp::Node
{
public:
    SimSensorNode()
    : Node("sim_sensor_node"),
      robot_x_(1.0), robot_y_(1.0), robot_theta_(0.0),
      has_odom_(false)
    {
        // ---- Room parameters ----
        this->declare_parameter<double>("room_width_m", 10.0);
        this->declare_parameter<double>("room_height_m", 10.0);
        this->declare_parameter<double>("resolution", 0.05);
        this->declare_parameter<int>("wall_thickness_cells", 2);
        this->declare_parameter<int>("seed", 0);
        this->declare_parameter<int>("min_partitions", 1);
        this->declare_parameter<int>("max_partitions", 3);
        this->declare_parameter<double>("doorway_width_m", 1.0);
        this->declare_parameter<int>("min_obstacles", 4);
        this->declare_parameter<int>("max_obstacles", 8);
        this->declare_parameter<double>("obstacle_min_size_m", 0.3);
        this->declare_parameter<double>("obstacle_max_size_m", 1.5);
        this->declare_parameter<double>("spawn_x", 1.0);
        this->declare_parameter<double>("spawn_y", 1.0);

        // ---- Ultrasonic parameters ----
        this->declare_parameter<double>("poll_rate_hz", 10.0);
        this->declare_parameter<double>("health_rate_hz", 1.0);
        this->declare_parameter<double>("us_max_range_m", 4.0);
        this->declare_parameter<double>("us_min_range_m", 0.02);
        this->declare_parameter<double>("us_fov_rad", 0.26);
        this->declare_parameter<double>("us_noise_stddev_m", 0.015);
        this->declare_parameter<double>("us_front_angle_rad", 0.0);
        this->declare_parameter<double>("us_left_angle_rad", 1.0472);
        this->declare_parameter<double>("us_right_angle_rad", -1.0472);

        // ---- LIDAR parameters ----
        this->declare_parameter<bool>("lidar_enabled", true);
        this->declare_parameter<int>("lidar_num_beams", 360);
        this->declare_parameter<double>("lidar_max_range_m", 12.0);
        this->declare_parameter<double>("lidar_min_range_m", 0.15);
        this->declare_parameter<double>("lidar_noise_stddev_m", 0.005);
        this->declare_parameter<double>("lidar_scan_rate_hz", 10.0);
        this->declare_parameter<double>("lidar_angle_min", -M_PI);
        this->declare_parameter<double>("lidar_angle_max", M_PI);

        // ---- Ground truth ----
        this->declare_parameter<double>("ground_truth_rate_hz", 0.5);

        // ---- Discovered map (fog of war) ----
        this->declare_parameter<double>("map_publish_rate_hz", 2.0);

        // Retrieve all params
        room_w_ = this->get_parameter("room_width_m").as_double();
        room_h_ = this->get_parameter("room_height_m").as_double();
        res_ = this->get_parameter("resolution").as_double();
        wall_thick_ = this->get_parameter("wall_thickness_cells").as_int();
        int seed = this->get_parameter("seed").as_int();
        min_parts_ = this->get_parameter("min_partitions").as_int();
        max_parts_ = this->get_parameter("max_partitions").as_int();
        door_w_ = this->get_parameter("doorway_width_m").as_double();
        min_obs_ = this->get_parameter("min_obstacles").as_int();
        max_obs_ = this->get_parameter("max_obstacles").as_int();
        obs_min_sz_ = this->get_parameter("obstacle_min_size_m").as_double();
        obs_max_sz_ = this->get_parameter("obstacle_max_size_m").as_double();
        spawn_x_ = this->get_parameter("spawn_x").as_double();
        spawn_y_ = this->get_parameter("spawn_y").as_double();

        poll_rate_ = this->get_parameter("poll_rate_hz").as_double();
        health_rate_ = this->get_parameter("health_rate_hz").as_double();
        us_max_ = this->get_parameter("us_max_range_m").as_double();
        us_min_ = this->get_parameter("us_min_range_m").as_double();
        us_fov_ = this->get_parameter("us_fov_rad").as_double();
        us_noise_ = this->get_parameter("us_noise_stddev_m").as_double();
        us_front_ang_ = this->get_parameter("us_front_angle_rad").as_double();
        us_left_ang_ = this->get_parameter("us_left_angle_rad").as_double();
        us_right_ang_ = this->get_parameter("us_right_angle_rad").as_double();

        lidar_enabled_ = this->get_parameter("lidar_enabled").as_bool();
        lidar_beams_ = this->get_parameter("lidar_num_beams").as_int();
        lidar_max_ = this->get_parameter("lidar_max_range_m").as_double();
        lidar_min_ = this->get_parameter("lidar_min_range_m").as_double();
        lidar_noise_ = this->get_parameter("lidar_noise_stddev_m").as_double();
        lidar_rate_ = this->get_parameter("lidar_scan_rate_hz").as_double();
        lidar_ang_min_ = this->get_parameter("lidar_angle_min").as_double();
        lidar_ang_max_ = this->get_parameter("lidar_angle_max").as_double();

        gt_rate_ = this->get_parameter("ground_truth_rate_hz").as_double();
        map_pub_rate_ = this->get_parameter("map_publish_rate_hz").as_double();

        // Grid dimensions
        grid_w_ = static_cast<int>(room_w_ / res_);
        grid_h_ = static_cast<int>(room_h_ / res_);

        // Init RNG
        if (seed == 0) {
            rng_.seed(std::random_device{}());
            current_seed_ = 0;
        } else {
            rng_.seed(static_cast<unsigned>(seed));
            current_seed_ = seed;
        }
        us_noise_dist_ = std::normal_distribution<double>(0.0, us_noise_);
        lidar_noise_dist_ = std::normal_distribution<double>(
            0.0, lidar_noise_);

        // ---- Generate room ----
        generateRoom();

        // ---- Initialise discovered map (all unknown) ----
        discovered_.assign(
            static_cast<size_t>(grid_w_) * grid_h_, -1);

        robot_x_ = spawn_x_;
        robot_y_ = spawn_y_;

        // ---- Subscriber for robot pose ----
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&SimSensorNode::odomCallback, this,
                std::placeholders::_1));

        // ---- Ultrasonic publishers (same as esp32_sensor_node) ----
        pub_front_ = this->create_publisher<sensor_msgs::msg::Range>(
            "/sensors/front", 10);
        pub_left_ = this->create_publisher<sensor_msgs::msg::Range>(
            "/sensors/left", 10);
        pub_right_ = this->create_publisher<sensor_msgs::msg::Range>(
            "/sensors/right", 10);
        pub_health_ = this->create_publisher<std_msgs::msg::UInt8>(
            "/sensors/health", 10);

        // ---- LIDAR publisher ----
        if (lidar_enabled_) {
            pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
                "/scan", 10);
        }

        // ---- Ground truth publisher ----
        pub_gt_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/sim/ground_truth", 1);

        // ---- Discovered map publisher (fog of war for debug) ----
        pub_map_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/sim/discovered", 10);

        // ---- Status publisher (for web UI sim info card) ----
        pub_status_ = this->create_publisher<std_msgs::msg::String>(
            "/sim/status", 1);

        // ---- Command subscriber for web UI control ----
        sim_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/sim/command", 10,
            std::bind(&SimSensorNode::simCommandCallback, this,
                std::placeholders::_1));

        // ---- Event publisher ----
        events_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/robot/events", 10);

        // ---- Timers ----
        auto us_period = std::chrono::duration<double>(1.0 / poll_rate_);
        us_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(us_period),
            std::bind(&SimSensorNode::ultrasonicCallback, this));

        auto health_period = std::chrono::duration<double>(
            1.0 / health_rate_);
        health_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                health_period),
            std::bind(&SimSensorNode::healthCallback, this));

        if (lidar_enabled_) {
            auto lidar_period = std::chrono::duration<double>(
                1.0 / lidar_rate_);
            lidar_timer_ = this->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    lidar_period),
                std::bind(&SimSensorNode::lidarCallback, this));
        }

        auto gt_period = std::chrono::duration<double>(1.0 / gt_rate_);
        gt_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(gt_period),
            std::bind(&SimSensorNode::groundTruthCallback, this));

        // Discovered map publish timer
        auto map_period = std::chrono::duration<double>(
            1.0 / map_pub_rate_);
        map_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(map_period),
            std::bind(&SimSensorNode::publishDiscoveredMap, this));

        // Status at 1 Hz
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SimSensorNode::statusCallback, this));

        // Publish initial status
        publishStatus();

        RCLCPP_INFO(this->get_logger(),
            "sim_sensor_node started — room %dx%d cells (%.1fx%.1f m), "
            "%zu obstacles, LIDAR %s",
            grid_w_, grid_h_, room_w_, room_h_,
            obstacle_count_,
            lidar_enabled_ ? "ON" : "OFF");
    }

private:
    // ================================================================
    // Room generation
    // ================================================================

    /**
     * @brief Generate the room layout: walls, partitions, obstacles.
     *        Flood-fill validates reachability from spawn.
     */
    void generateRoom()
    {
        grid_.assign(
            static_cast<size_t>(grid_w_) * grid_h_, 0);  // All free

        // Outer walls
        for (int y = 0; y < grid_h_; ++y) {
            for (int x = 0; x < grid_w_; ++x) {
                if (x < wall_thick_ || x >= grid_w_ - wall_thick_ ||
                    y < wall_thick_ || y >= grid_h_ - wall_thick_)
                {
                    grid_[static_cast<size_t>(y * grid_w_ + x)] = 100;
                }
            }
        }

        // Internal partitions
        std::uniform_int_distribution<int> part_count(
            min_parts_, max_parts_);
        int n_parts = part_count(rng_);
        partition_count_ = static_cast<size_t>(n_parts);

        for (int i = 0; i < n_parts; ++i) {
            addPartition();
        }

        // Random obstacles
        std::uniform_int_distribution<int> obs_count(min_obs_, max_obs_);
        int n_obs = obs_count(rng_);
        obstacle_count_ = 0;

        for (int i = 0; i < n_obs; ++i) {
            if (addObstacle()) {
                ++obstacle_count_;
            }
        }

        // Validate spawn is reachable and enough area is free
        int spawn_gx = static_cast<int>(spawn_x_ / res_);
        int spawn_gy = static_cast<int>(spawn_y_ / res_);

        // Clamp spawn to valid range
        spawn_gx = std::clamp(spawn_gx, wall_thick_ + 1,
            grid_w_ - wall_thick_ - 2);
        spawn_gy = std::clamp(spawn_gy, wall_thick_ + 1,
            grid_h_ - wall_thick_ - 2);

        // Clear a small area around spawn
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
                int gx = spawn_gx + dx;
                int gy = spawn_gy + dy;
                if (gx >= 0 && gx < grid_w_ && gy >= 0 && gy < grid_h_) {
                    grid_[static_cast<size_t>(gy * grid_w_ + gx)] = 0;
                }
            }
        }

        // Flood fill from spawn to check connectivity
        removeUnreachableObstacles(spawn_gx, spawn_gy);

        RCLCPP_INFO(this->get_logger(),
            "Room generated: %d partitions, %zu obstacles, "
            "spawn (%d, %d)", n_parts, obstacle_count_,
            spawn_gx, spawn_gy);
    }

    /**
     * @brief Add a partition wall (horizontal or vertical) with a doorway.
     */
    void addPartition()
    {
        int margin = wall_thick_ + 10;  // Stay away from outer walls
        if (margin >= grid_w_ / 2 || margin >= grid_h_ / 2) {
            return;
        }

        std::uniform_int_distribution<int> orient(0, 1);
        bool horizontal = orient(rng_) == 0;

        int door_cells = static_cast<int>(door_w_ / res_);

        if (horizontal) {
            std::uniform_int_distribution<int> y_dist(
                margin, grid_h_ - margin);
            int py = y_dist(rng_);

            // Draw wall
            for (int x = wall_thick_; x < grid_w_ - wall_thick_; ++x) {
                for (int t = 0; t < wall_thick_; ++t) {
                    int gy = py + t;
                    if (gy >= 0 && gy < grid_h_) {
                        grid_[static_cast<size_t>(gy * grid_w_ + x)] = 100;
                    }
                }
            }

            // Cut doorway
            std::uniform_int_distribution<int> door_pos(
                wall_thick_ + 5, grid_w_ - wall_thick_ - door_cells - 5);
            int dx = door_pos(rng_);
            for (int x = dx; x < dx + door_cells; ++x) {
                for (int t = 0; t < wall_thick_; ++t) {
                    int gy = py + t;
                    if (gy >= 0 && gy < grid_h_ &&
                        x >= 0 && x < grid_w_)
                    {
                        grid_[static_cast<size_t>(gy * grid_w_ + x)] = 0;
                    }
                }
            }
        } else {
            // Vertical partition
            std::uniform_int_distribution<int> x_dist(
                margin, grid_w_ - margin);
            int px = x_dist(rng_);

            for (int y = wall_thick_; y < grid_h_ - wall_thick_; ++y) {
                for (int t = 0; t < wall_thick_; ++t) {
                    int gx = px + t;
                    if (gx >= 0 && gx < grid_w_) {
                        grid_[static_cast<size_t>(y * grid_w_ + gx)] = 100;
                    }
                }
            }

            std::uniform_int_distribution<int> door_pos(
                wall_thick_ + 5, grid_h_ - wall_thick_ - door_cells - 5);
            int dy = door_pos(rng_);
            for (int y = dy; y < dy + door_cells; ++y) {
                for (int t = 0; t < wall_thick_; ++t) {
                    int gx = px + t;
                    if (y >= 0 && y < grid_h_ &&
                        gx >= 0 && gx < grid_w_)
                    {
                        grid_[static_cast<size_t>(y * grid_w_ + gx)] = 0;
                    }
                }
            }
        }
    }

    /**
     * @brief Add a random rectangular obstacle. Returns true if placed.
     */
    bool addObstacle()
    {
        std::uniform_real_distribution<double> sz_dist(
            obs_min_sz_, obs_max_sz_);
        double w_m = sz_dist(rng_);
        double h_m = sz_dist(rng_);
        int w_cells = static_cast<int>(w_m / res_);
        int h_cells = static_cast<int>(h_m / res_);

        int margin = wall_thick_ + 3;
        if (margin >= grid_w_ - w_cells - margin ||
            margin >= grid_h_ - h_cells - margin)
        {
            return false;
        }

        std::uniform_int_distribution<int> x_dist(
            margin, grid_w_ - w_cells - margin);
        std::uniform_int_distribution<int> y_dist(
            margin, grid_h_ - h_cells - margin);

        int ox = x_dist(rng_);
        int oy = y_dist(rng_);

        // Don't place on spawn
        int spawn_gx = static_cast<int>(spawn_x_ / res_);
        int spawn_gy = static_cast<int>(spawn_y_ / res_);
        if (std::abs(ox - spawn_gx) < 8 && std::abs(oy - spawn_gy) < 8) {
            return false;  // Too close to spawn
        }

        for (int y = oy; y < oy + h_cells && y < grid_h_; ++y) {
            for (int x = ox; x < ox + w_cells && x < grid_w_; ++x) {
                grid_[static_cast<size_t>(y * grid_w_ + x)] = 100;
            }
        }
        return true;
    }

    /**
     * @brief BFS flood-fill from spawn.  Any obstacle cell that makes
     *        too much area unreachable is cleared.
     */
    void removeUnreachableObstacles(int spawn_gx, int spawn_gy)
    {
        size_t total = static_cast<size_t>(grid_w_) * grid_h_;
        std::vector<bool> visited(total, false);
        std::queue<std::pair<int, int>> q;

        q.push({spawn_gx, spawn_gy});
        visited[static_cast<size_t>(spawn_gy * grid_w_ + spawn_gx)] = true;
        int reachable = 0;

        while (!q.empty()) {
            auto [cx, cy] = q.front();
            q.pop();
            ++reachable;

            const int dx[] = {1, -1, 0, 0};
            const int dy[] = {0, 0, 1, -1};
            for (int d = 0; d < 4; ++d) {
                int nx = cx + dx[d];
                int ny = cy + dy[d];
                if (nx < 0 || nx >= grid_w_ || ny < 0 || ny >= grid_h_) {
                    continue;
                }
                size_t ni = static_cast<size_t>(ny * grid_w_ + nx);
                if (!visited[ni] && grid_[ni] == 0) {
                    visited[ni] = true;
                    q.push({nx, ny});
                }
            }
        }

        // Count total free cells
        int total_free = 0;
        for (size_t i = 0; i < total; ++i) {
            if (grid_[i] == 0) {
                ++total_free;
            }
        }

        // If less than 70% of free space is reachable, clear unreachable
        // obstacles to improve connectivity
        if (total_free > 0 &&
            static_cast<double>(reachable) / total_free < 0.7)
        {
            RCLCPP_WARN(this->get_logger(),
                "Only %d/%d free cells reachable (%.0f%%) — "
                "clearing unreachable obstacles",
                reachable, total_free,
                100.0 * reachable / total_free);

            for (size_t i = 0; i < total; ++i) {
                if (grid_[i] == 100 && !visited[i]) {
                    // Check if this is an interior obstacle (not outer wall)
                    int y = static_cast<int>(i) / grid_w_;
                    int x = static_cast<int>(i) % grid_w_;
                    if (x >= wall_thick_ &&
                        x < grid_w_ - wall_thick_ &&
                        y >= wall_thick_ &&
                        y < grid_h_ - wall_thick_)
                    {
                        grid_[i] = 0;
                    }
                }
            }
        }
    }

    // ================================================================
    // Raycasting
    // ================================================================

    /**
     * @brief Raycast from (ox, oy) at angle `angle` and return the
     *        distance to the first occupied cell, up to max_range.
     */
    double raycast(double ox, double oy, double angle,
                   double max_range) const
    {
        double step = res_ * 0.5;  // Sub-cell stepping
        double dx = std::cos(angle) * step;
        double dy = std::sin(angle) * step;

        double cx = ox;
        double cy = oy;
        int steps = static_cast<int>(max_range / step);

        for (int i = 0; i < steps; ++i) {
            cx += dx;
            cy += dy;

            int gx = static_cast<int>(cx / res_);
            int gy = static_cast<int>(cy / res_);

            if (gx < 0 || gx >= grid_w_ || gy < 0 || gy >= grid_h_) {
                return max_range;
            }

            if (grid_[static_cast<size_t>(gy * grid_w_ + gx)] > 50) {
                return std::hypot(cx - ox, cy - oy);
            }
        }
        return max_range;
    }

    // ================================================================
    // Callbacks
    // ================================================================

    /**
     * @brief Update robot pose from odometry.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        // Extract yaw from quaternion (z, w only for 2D)
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        robot_theta_ = 2.0 * std::atan2(qz, qw);
        has_odom_ = true;
    }

    /**
     * @brief Publish ultrasonic Range messages — same topics as
     *        esp32_sensor_node.
     */
    void ultrasonicCallback()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!has_odom_) {
            return;  // Wait for first odom
        }

        auto stamp = this->now();

        // Raycast 3 ultrasonic beams
        double front_dist = raycast(
            robot_x_, robot_y_,
            robot_theta_ + us_front_ang_, us_max_);
        double left_dist = raycast(
            robot_x_, robot_y_,
            robot_theta_ + us_left_ang_, us_max_);
        double right_dist = raycast(
            robot_x_, robot_y_,
            robot_theta_ + us_right_ang_, us_max_);

        // Add noise
        front_dist += us_noise_dist_(rng_);
        left_dist += us_noise_dist_(rng_);
        right_dist += us_noise_dist_(rng_);

        // Clamp
        front_dist = std::clamp(front_dist, us_min_, us_max_);
        left_dist = std::clamp(left_dist, us_min_, us_max_);
        right_dist = std::clamp(right_dist, us_min_, us_max_);

        auto makeRange = [&](const std::string & frame_id,
                             double dist) -> sensor_msgs::msg::Range
        {
            sensor_msgs::msg::Range msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = frame_id;
            msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
            msg.field_of_view = static_cast<float>(us_fov_);
            msg.min_range = static_cast<float>(us_min_);
            msg.max_range = static_cast<float>(us_max_);
            msg.range = static_cast<float>(dist);
            return msg;
        };

        pub_front_->publish(makeRange("ultrasonic_front", front_dist));
        pub_left_->publish(makeRange("ultrasonic_left", left_dist));
        pub_right_->publish(makeRange("ultrasonic_right", right_dist));
    }

    /**
     * @brief Publish sensor health — all OK in simulation.
     */
    void healthCallback()
    {
        auto msg = std_msgs::msg::UInt8();
        msg.data = 0x07;  // All 3 sensors OK (bits 0,1,2)
        pub_health_->publish(msg);
    }

    /**
     * @brief Publish simulated LIDAR LaserScan with many beams.
     */
    void lidarCallback()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!has_odom_) {
            return;
        }

        auto scan = sensor_msgs::msg::LaserScan();
        scan.header.stamp = this->now();
        scan.header.frame_id = "base_link";
        scan.angle_min = static_cast<float>(lidar_ang_min_);
        scan.angle_max = static_cast<float>(lidar_ang_max_);
        scan.angle_increment = static_cast<float>(
            (lidar_ang_max_ - lidar_ang_min_) / lidar_beams_);
        scan.time_increment = 0.0f;
        scan.scan_time = static_cast<float>(1.0 / lidar_rate_);
        scan.range_min = static_cast<float>(lidar_min_);
        scan.range_max = static_cast<float>(lidar_max_);

        scan.ranges.resize(static_cast<size_t>(lidar_beams_));

        double angle = lidar_ang_min_;
        double inc = (lidar_ang_max_ - lidar_ang_min_) / lidar_beams_;

        for (int i = 0; i < lidar_beams_; ++i) {
            double dist = raycast(
                robot_x_, robot_y_,
                robot_theta_ + angle, lidar_max_);
            dist += lidar_noise_dist_(rng_);
            dist = std::clamp(dist, lidar_min_, lidar_max_);
            scan.ranges[static_cast<size_t>(i)] = static_cast<float>(dist);

            // --- Mark discovered cells along this ray ---
            markRayDiscovered(
                robot_x_, robot_y_,
                robot_theta_ + angle, dist);

            angle += inc;
        }

        pub_scan_->publish(scan);
    }

    /**
     * @brief Publish the ground-truth occupancy grid for debug/display.
     *        -1 (unknown) is NOT used here — this is the full revealed map.
     */
    void groundTruthCallback()
    {
        auto msg = nav_msgs::msg::OccupancyGrid();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.info.resolution = static_cast<float>(res_);
        msg.info.width = static_cast<uint32_t>(grid_w_);
        msg.info.height = static_cast<uint32_t>(grid_h_);
        msg.info.origin.position.x = 0.0;
        msg.info.origin.position.y = 0.0;

        msg.data.resize(static_cast<size_t>(grid_w_) * grid_h_);
        for (size_t i = 0; i < grid_.size(); ++i) {
            msg.data[i] = static_cast<int8_t>(grid_[i]);
        }

        pub_gt_->publish(msg);
    }

    /**
     * @brief Mark cells along a LIDAR ray as discovered in the fog-of-war
     *        map.  Free cells become 0, the hit cell becomes 100 (wall).
     */
    void markRayDiscovered(double ox, double oy, double angle,
                           double hit_dist)
    {
        double step = res_ * 0.5;
        double dx = std::cos(angle) * step;
        double dy = std::sin(angle) * step;

        double cx = ox;
        double cy = oy;
        int steps = static_cast<int>(hit_dist / step);

        for (int i = 0; i < steps; ++i) {
            cx += dx;
            cy += dy;

            int gx = static_cast<int>(cx / res_);
            int gy = static_cast<int>(cy / res_);

            if (gx < 0 || gx >= grid_w_ || gy < 0 || gy >= grid_h_) {
                return;
            }

            size_t idx = static_cast<size_t>(gy * grid_w_ + gx);
            // Mark traversed cells as free (if ground truth is free)
            if (discovered_[idx] == -1) {
                discovered_[idx] = grid_[idx] > 50
                    ? static_cast<int8_t>(100)
                    : static_cast<int8_t>(0);
            }
        }

        // Mark the hit cell as occupied if within range
        if (hit_dist < lidar_max_ - 0.01) {
            double hx = ox + std::cos(angle) * hit_dist;
            double hy = oy + std::sin(angle) * hit_dist;
            int gx = static_cast<int>(hx / res_);
            int gy = static_cast<int>(hy / res_);
            if (gx >= 0 && gx < grid_w_ && gy >= 0 && gy < grid_h_) {
                size_t idx = static_cast<size_t>(gy * grid_w_ + gx);
                discovered_[idx] = static_cast<int8_t>(100);
            }
        }
    }

    /**
     * @brief Publish the fog-of-war discovered map on /map.
     *        Cells start as unknown (-1), become 0 (free) or 100 (wall)
     *        as LIDAR rays traverse or hit them.
     */
    void publishDiscoveredMap()
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        auto msg = nav_msgs::msg::OccupancyGrid();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.info.resolution = static_cast<float>(res_);
        msg.info.width = static_cast<uint32_t>(grid_w_);
        msg.info.height = static_cast<uint32_t>(grid_h_);
        msg.info.origin.position.x = 0.0;
        msg.info.origin.position.y = 0.0;

        msg.data.resize(static_cast<size_t>(grid_w_) * grid_h_);
        for (size_t i = 0; i < discovered_.size(); ++i) {
            msg.data[i] = discovered_[i];
        }

        pub_map_->publish(msg);
    }

    /**
     * @brief Handle simulation commands from /sim/command.
     *        Commands: "regenerate", "regenerate:seed=<N>"
     */
    void simCommandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        const auto & cmd = msg->data;
        if (cmd == "regenerate" || cmd.substr(0, 11) == "regenerate:") {
            // Parse optional seed
            if (cmd.size() > 16 && cmd.substr(11, 5) == "seed=") {
                try {
                    int new_seed = std::stoi(cmd.substr(16));
                    current_seed_ = new_seed;
                    rng_.seed(static_cast<unsigned>(new_seed));
                    RCLCPP_INFO(this->get_logger(),
                        "Regenerating room with seed=%d", new_seed);
                } catch (...) {
                    rng_.seed(std::random_device{}());
                    current_seed_ = 0;
                    RCLCPP_WARN(this->get_logger(),
                        "Invalid seed in command — using random");
                }
            } else {
                rng_.seed(std::random_device{}());
                current_seed_ = 0;
                RCLCPP_INFO(this->get_logger(),
                    "Regenerating room with random seed");
            }

            // Regenerate
            generateRoom();
            discovered_.assign(
                static_cast<size_t>(grid_w_) * grid_h_, -1);
            publishStatus();

            // Force immediate ground truth publish
            groundTruthCallback();

            auto event = std_msgs::msg::String();
            event.data = "SIM_MAP_REGENERATED";
            events_pub_->publish(event);
        }
    }

    /**
     * @brief Periodic status publication for the web UI sim info card.
     */
    void statusCallback()
    {
        publishStatus();
    }

    /**
     * @brief Publish sim status as JSON string on /sim/status.
     */
    void publishStatus()
    {
        // Count free and wall cells
        int free_cells = 0;
        int wall_cells = 0;
        for (size_t i = 0; i < grid_.size(); ++i) {
            if (grid_[i] == 0) ++free_cells;
            else if (grid_[i] > 50) ++wall_cells;
        }

        std::string json = "{";
        json += "\"room_w\":" + std::to_string(room_w_) + ",";
        json += "\"room_h\":" + std::to_string(room_h_) + ",";
        json += "\"resolution\":" + std::to_string(res_) + ",";
        json += "\"grid_w\":" + std::to_string(grid_w_) + ",";
        json += "\"grid_h\":" + std::to_string(grid_h_) + ",";
        json += "\"obstacles\":" + std::to_string(obstacle_count_) + ",";
        json += "\"partitions\":" + std::to_string(partition_count_) + ",";
        json += "\"free_cells\":" + std::to_string(free_cells) + ",";
        json += "\"wall_cells\":" + std::to_string(wall_cells) + ",";
        json += "\"seed\":" + std::to_string(current_seed_);
        json += "}";

        auto msg = std_msgs::msg::String();
        msg.data = json;
        pub_status_->publish(msg);
    }

    // ---- Room params ----
    double room_w_;
    double room_h_;
    double res_;
    int wall_thick_;
    int min_parts_;
    int max_parts_;
    double door_w_;
    int min_obs_;
    int max_obs_;
    double obs_min_sz_;
    double obs_max_sz_;
    double spawn_x_;
    double spawn_y_;
    int grid_w_;
    int grid_h_;
    size_t obstacle_count_ = 0;
    size_t partition_count_ = 0;
    int current_seed_ = 0;

    // ---- Sensor params ----
    double poll_rate_;
    double health_rate_;
    double us_max_;
    double us_min_;
    double us_fov_;
    double us_noise_;
    double us_front_ang_;
    double us_left_ang_;
    double us_right_ang_;

    bool lidar_enabled_;
    int lidar_beams_;
    double lidar_max_;
    double lidar_min_;
    double lidar_noise_;
    double lidar_rate_;
    double lidar_ang_min_;
    double lidar_ang_max_;

    double gt_rate_;
    double map_pub_rate_;

    // ---- Room data ----
    std::vector<int8_t> grid_;        // Ground truth map
    std::vector<int8_t> discovered_;  // Fog-of-war map (-1=unknown)

    // ---- Robot pose ----
    double robot_x_;
    double robot_y_;
    double robot_theta_;
    bool has_odom_;
    std::mutex pose_mutex_;

    // ---- RNG ----
    std::mt19937 rng_;
    std::normal_distribution<double> us_noise_dist_;
    std::normal_distribution<double> lidar_noise_dist_;

    // ---- ROS2 interfaces ----
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_front_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_left_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_right_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_health_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_gt_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sim_cmd_sub_;

    rclcpp::TimerBase::SharedPtr us_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    rclcpp::TimerBase::SharedPtr gt_timer_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SimSensorNode>());
    } catch (const std::runtime_error & e) {
        RCLCPP_FATAL(rclcpp::get_logger("sim_sensor_node"),
            "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
