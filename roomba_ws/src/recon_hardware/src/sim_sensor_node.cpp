/**
 * @file sim_sensor_node.cpp
 * @brief Simulated LIDAR source for handheld-scanner bench testing.
 *
 * Generates a randomised walled room with optional internal partitions and
 * obstacles, then publishes a 360-beam LaserScan raycast from a fixed spawn
 * pose. Also publishes the ground-truth OccupancyGrid for debug overlays.
 *
 * The node intentionally does NOT track motion. Without an EKF/IMU the
 * device sits at spawn and the scan is constant — useful for exercising
 * slam_toolbox startup, web UI map rendering, and the headless save path.
 * A future sim_walker node may publish poses on /scanner/pose to drive
 * synthetic motion; until then this file has no pose subscriber.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <random>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class SimSensorNode : public rclcpp::Node
{
public:
    SimSensorNode()
    : Node("sim_sensor_node")
    {
        // ---- Room parameters ----
        declare_parameter<double>("room_width_m", 10.0);
        declare_parameter<double>("room_height_m", 10.0);
        declare_parameter<double>("resolution", 0.05);
        declare_parameter<int>("wall_thickness_cells", 2);
        declare_parameter<int>("seed", 0);
        declare_parameter<int>("min_partitions", 1);
        declare_parameter<int>("max_partitions", 3);
        declare_parameter<double>("doorway_width_m", 1.0);
        declare_parameter<int>("min_obstacles", 4);
        declare_parameter<int>("max_obstacles", 8);
        declare_parameter<double>("obstacle_min_size_m", 0.3);
        declare_parameter<double>("obstacle_max_size_m", 1.5);

        // ---- Spawn pose (static — sim is stationary) ----
        declare_parameter<double>("spawn_x", 1.0);
        declare_parameter<double>("spawn_y", 1.0);
        declare_parameter<double>("spawn_theta", 0.0);

        // ---- LIDAR parameters ----
        declare_parameter<int>("lidar_num_beams", 360);
        declare_parameter<double>("lidar_max_range_m", 12.0);
        declare_parameter<double>("lidar_min_range_m", 0.15);
        declare_parameter<double>("lidar_noise_stddev_m", 0.005);
        declare_parameter<double>("lidar_scan_rate_hz", 10.0);
        declare_parameter<double>("lidar_angle_min", -M_PI);
        declare_parameter<double>("lidar_angle_max", M_PI);

        // ---- Ground-truth debug rate ----
        declare_parameter<double>("ground_truth_rate_hz", 0.5);

        room_w_ = get_parameter("room_width_m").as_double();
        room_h_ = get_parameter("room_height_m").as_double();
        res_ = get_parameter("resolution").as_double();
        wall_thick_ = get_parameter("wall_thickness_cells").as_int();
        const int seed = get_parameter("seed").as_int();
        min_parts_ = get_parameter("min_partitions").as_int();
        max_parts_ = get_parameter("max_partitions").as_int();
        door_w_ = get_parameter("doorway_width_m").as_double();
        min_obs_ = get_parameter("min_obstacles").as_int();
        max_obs_ = get_parameter("max_obstacles").as_int();
        obs_min_sz_ = get_parameter("obstacle_min_size_m").as_double();
        obs_max_sz_ = get_parameter("obstacle_max_size_m").as_double();
        spawn_x_ = get_parameter("spawn_x").as_double();
        spawn_y_ = get_parameter("spawn_y").as_double();
        spawn_theta_ = get_parameter("spawn_theta").as_double();

        lidar_beams_ = get_parameter("lidar_num_beams").as_int();
        lidar_max_ = get_parameter("lidar_max_range_m").as_double();
        lidar_min_ = get_parameter("lidar_min_range_m").as_double();
        lidar_noise_ = get_parameter("lidar_noise_stddev_m").as_double();
        lidar_rate_ = get_parameter("lidar_scan_rate_hz").as_double();
        lidar_ang_min_ = get_parameter("lidar_angle_min").as_double();
        lidar_ang_max_ = get_parameter("lidar_angle_max").as_double();

        gt_rate_ = get_parameter("ground_truth_rate_hz").as_double();

        grid_w_ = static_cast<int>(room_w_ / res_);
        grid_h_ = static_cast<int>(room_h_ / res_);

        if (seed == 0) {
            rng_.seed(std::random_device{}());
        } else {
            rng_.seed(static_cast<unsigned>(seed));
        }
        lidar_noise_dist_ = std::normal_distribution<double>(0.0, lidar_noise_);

        generateRoom();

        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        gt_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/sim/ground_truth", 1);

        const auto lidar_period = std::chrono::duration<double>(1.0 / lidar_rate_);
        lidar_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(lidar_period),
            std::bind(&SimSensorNode::publishScan, this));

        const auto gt_period = std::chrono::duration<double>(1.0 / gt_rate_);
        gt_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(gt_period),
            std::bind(&SimSensorNode::publishGroundTruth, this));

        RCLCPP_INFO(get_logger(),
            "sim_sensor_node started — room %dx%d cells (%.1fx%.1f m), "
            "%zu obstacles, spawn=(%.2f, %.2f, %.2f rad)",
            grid_w_, grid_h_, room_w_, room_h_,
            obstacle_count_, spawn_x_, spawn_y_, spawn_theta_);
    }

private:
    // =====================================================================
    // Room generation
    // =====================================================================

    void generateRoom()
    {
        grid_.assign(static_cast<size_t>(grid_w_) * grid_h_, 0);

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

        std::uniform_int_distribution<int> part_count(min_parts_, max_parts_);
        const int n_parts = part_count(rng_);
        for (int i = 0; i < n_parts; ++i) {
            addPartition();
        }

        std::uniform_int_distribution<int> obs_count(min_obs_, max_obs_);
        const int n_obs = obs_count(rng_);
        obstacle_count_ = 0;
        for (int i = 0; i < n_obs; ++i) {
            if (addObstacle()) {
                ++obstacle_count_;
            }
        }

        int spawn_gx = std::clamp(static_cast<int>(spawn_x_ / res_),
            wall_thick_ + 1, grid_w_ - wall_thick_ - 2);
        int spawn_gy = std::clamp(static_cast<int>(spawn_y_ / res_),
            wall_thick_ + 1, grid_h_ - wall_thick_ - 2);

        // Clear a small area around spawn so the device starts in free space
        for (int dy = -2; dy <= 2; ++dy) {
            for (int dx = -2; dx <= 2; ++dx) {
                const int gx = spawn_gx + dx;
                const int gy = spawn_gy + dy;
                if (gx >= 0 && gx < grid_w_ && gy >= 0 && gy < grid_h_) {
                    grid_[static_cast<size_t>(gy * grid_w_ + gx)] = 0;
                }
            }
        }

        removeUnreachableObstacles(spawn_gx, spawn_gy);

        RCLCPP_INFO(get_logger(),
            "Room generated: %d partitions, %zu obstacles, spawn cell (%d, %d)",
            n_parts, obstacle_count_, spawn_gx, spawn_gy);
    }

    void addPartition()
    {
        const int margin = wall_thick_ + 10;
        if (margin >= grid_w_ / 2 || margin >= grid_h_ / 2) {
            return;
        }

        std::uniform_int_distribution<int> orient(0, 1);
        const bool horizontal = orient(rng_) == 0;
        const int door_cells = static_cast<int>(door_w_ / res_);

        if (horizontal) {
            std::uniform_int_distribution<int> y_dist(margin, grid_h_ - margin);
            const int py = y_dist(rng_);
            for (int x = wall_thick_; x < grid_w_ - wall_thick_; ++x) {
                for (int t = 0; t < wall_thick_; ++t) {
                    const int gy = py + t;
                    if (gy >= 0 && gy < grid_h_) {
                        grid_[static_cast<size_t>(gy * grid_w_ + x)] = 100;
                    }
                }
            }
            std::uniform_int_distribution<int> door_pos(
                wall_thick_ + 5, grid_w_ - wall_thick_ - door_cells - 5);
            const int dx = door_pos(rng_);
            for (int x = dx; x < dx + door_cells; ++x) {
                for (int t = 0; t < wall_thick_; ++t) {
                    const int gy = py + t;
                    if (gy >= 0 && gy < grid_h_ && x >= 0 && x < grid_w_) {
                        grid_[static_cast<size_t>(gy * grid_w_ + x)] = 0;
                    }
                }
            }
        } else {
            std::uniform_int_distribution<int> x_dist(margin, grid_w_ - margin);
            const int px = x_dist(rng_);
            for (int y = wall_thick_; y < grid_h_ - wall_thick_; ++y) {
                for (int t = 0; t < wall_thick_; ++t) {
                    const int gx = px + t;
                    if (gx >= 0 && gx < grid_w_) {
                        grid_[static_cast<size_t>(y * grid_w_ + gx)] = 100;
                    }
                }
            }
            std::uniform_int_distribution<int> door_pos(
                wall_thick_ + 5, grid_h_ - wall_thick_ - door_cells - 5);
            const int dy = door_pos(rng_);
            for (int y = dy; y < dy + door_cells; ++y) {
                for (int t = 0; t < wall_thick_; ++t) {
                    const int gx = px + t;
                    if (y >= 0 && y < grid_h_ && gx >= 0 && gx < grid_w_) {
                        grid_[static_cast<size_t>(y * grid_w_ + gx)] = 0;
                    }
                }
            }
        }
    }

    bool addObstacle()
    {
        std::uniform_real_distribution<double> sz_dist(obs_min_sz_, obs_max_sz_);
        const double w_m = sz_dist(rng_);
        const double h_m = sz_dist(rng_);
        const int w_cells = static_cast<int>(w_m / res_);
        const int h_cells = static_cast<int>(h_m / res_);

        const int margin = wall_thick_ + 3;
        if (margin >= grid_w_ - w_cells - margin ||
            margin >= grid_h_ - h_cells - margin)
        {
            return false;
        }

        std::uniform_int_distribution<int> x_dist(
            margin, grid_w_ - w_cells - margin);
        std::uniform_int_distribution<int> y_dist(
            margin, grid_h_ - h_cells - margin);
        const int ox = x_dist(rng_);
        const int oy = y_dist(rng_);

        const int spawn_gx = static_cast<int>(spawn_x_ / res_);
        const int spawn_gy = static_cast<int>(spawn_y_ / res_);
        if (std::abs(ox - spawn_gx) < 8 && std::abs(oy - spawn_gy) < 8) {
            return false;
        }

        for (int y = oy; y < oy + h_cells && y < grid_h_; ++y) {
            for (int x = ox; x < ox + w_cells && x < grid_w_; ++x) {
                grid_[static_cast<size_t>(y * grid_w_ + x)] = 100;
            }
        }
        return true;
    }

    void removeUnreachableObstacles(int spawn_gx, int spawn_gy)
    {
        const size_t total = static_cast<size_t>(grid_w_) * grid_h_;
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
                const int nx = cx + dx[d];
                const int ny = cy + dy[d];
                if (nx < 0 || nx >= grid_w_ || ny < 0 || ny >= grid_h_) {
                    continue;
                }
                const size_t ni = static_cast<size_t>(ny * grid_w_ + nx);
                if (!visited[ni] && grid_[ni] == 0) {
                    visited[ni] = true;
                    q.push({nx, ny});
                }
            }
        }

        int total_free = 0;
        for (size_t i = 0; i < total; ++i) {
            if (grid_[i] == 0) {
                ++total_free;
            }
        }

        if (total_free > 0 &&
            static_cast<double>(reachable) / total_free < 0.7)
        {
            RCLCPP_WARN(get_logger(),
                "Only %d/%d free cells reachable (%.0f%%) — clearing "
                "unreachable interior obstacles",
                reachable, total_free, 100.0 * reachable / total_free);
            for (size_t i = 0; i < total; ++i) {
                if (grid_[i] == 100 && !visited[i]) {
                    const int y = static_cast<int>(i) / grid_w_;
                    const int x = static_cast<int>(i) % grid_w_;
                    if (x >= wall_thick_ && x < grid_w_ - wall_thick_ &&
                        y >= wall_thick_ && y < grid_h_ - wall_thick_)
                    {
                        grid_[i] = 0;
                    }
                }
            }
        }
    }

    // =====================================================================
    // Raycasting
    // =====================================================================

    double raycast(double ox, double oy, double angle, double max_range) const
    {
        const double step = res_ * 0.5;
        const double dx = std::cos(angle) * step;
        const double dy = std::sin(angle) * step;
        double cx = ox;
        double cy = oy;
        const int steps = static_cast<int>(max_range / step);
        for (int i = 0; i < steps; ++i) {
            cx += dx;
            cy += dy;
            const int gx = static_cast<int>(cx / res_);
            const int gy = static_cast<int>(cy / res_);
            if (gx < 0 || gx >= grid_w_ || gy < 0 || gy >= grid_h_) {
                return max_range;
            }
            if (grid_[static_cast<size_t>(gy * grid_w_ + gx)] > 50) {
                return std::hypot(cx - ox, cy - oy);
            }
        }
        return max_range;
    }

    // =====================================================================
    // Publishers
    // =====================================================================

    void publishScan()
    {
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = now();
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

        const double inc =
            (lidar_ang_max_ - lidar_ang_min_) / lidar_beams_;
        double angle = lidar_ang_min_;
        for (int i = 0; i < lidar_beams_; ++i) {
            double dist = raycast(
                spawn_x_, spawn_y_, spawn_theta_ + angle, lidar_max_);
            dist += lidar_noise_dist_(rng_);
            dist = std::clamp(dist, lidar_min_, lidar_max_);
            scan.ranges[static_cast<size_t>(i)] = static_cast<float>(dist);
            angle += inc;
        }

        scan_pub_->publish(scan);
    }

    void publishGroundTruth()
    {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.info.resolution = static_cast<float>(res_);
        msg.info.width = static_cast<uint32_t>(grid_w_);
        msg.info.height = static_cast<uint32_t>(grid_h_);
        msg.info.origin.position.x = 0.0;
        msg.info.origin.position.y = 0.0;
        msg.data.assign(grid_.begin(), grid_.end());
        gt_pub_->publish(msg);
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
    double spawn_theta_;
    int grid_w_;
    int grid_h_;
    size_t obstacle_count_ = 0;

    // ---- LIDAR params ----
    int lidar_beams_;
    double lidar_max_;
    double lidar_min_;
    double lidar_noise_;
    double lidar_rate_;
    double lidar_ang_min_;
    double lidar_ang_max_;
    double gt_rate_;

    // ---- Grid + RNG ----
    std::vector<int8_t> grid_;
    std::mt19937 rng_;
    std::normal_distribution<double> lidar_noise_dist_;

    // ---- ROS interfaces ----
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gt_pub_;
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    rclcpp::TimerBase::SharedPtr gt_timer_;
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
