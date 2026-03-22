/**
 * @file draw_node.cpp
 * @brief Controller-driven drawing canvas for testing map persistence.
 *
 * Publishes OccupancyGrid on /map so the web UI shows the canvas and
 * db_node can save it on SAVE_MAP events.
 *
 * Controls (Xbox controller via joy_linux + xpadneo):
 *   Left stick  — move cursor
 *   Right trigger — draw (mark cells as occupied, value 100)
 *   Left trigger  — erase (mark cells as free, value 0)
 *   A button      — stamp (draw at cursor without moving)
 *   X button      — save map (publishes SAVE_MAP to /robot/events)
 *   Y button      — clear grid (reset all cells to unknown, -1)
 *   D-pad up/down — increase/decrease brush size
 *   Start button  — reset cursor to centre
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class DrawNode
 * @brief Interactive drawing canvas controlled by an Xbox gamepad.
 *
 * Subscribes to /joy and publishes a drawn-on OccupancyGrid to /map
 * at a configurable rate.  Intended for testing the full
 * controller → /map → db_node → PostgreSQL save pipeline.
 */
class DrawNode : public rclcpp::Node
{
public:
    DrawNode()
    : Node("draw_node")
    {
        // --- Parameters (all from YAML or defaults) ---
        declare_parameter<int>("draw.grid_width", 100);
        declare_parameter<int>("draw.grid_height", 100);
        declare_parameter<double>("draw.resolution", 0.05);
        declare_parameter<double>("draw.cursor_speed", 30.0);
        declare_parameter<int>("draw.brush_size", 2);
        declare_parameter<double>("draw.publish_rate", 5.0);

        // Button / axis indices reuse controller.yaml mapping
        declare_parameter<int>("axes.left_x", 0);
        declare_parameter<int>("axes.left_y", 1);
        declare_parameter<int>("axes.left_trigger", 2);
        declare_parameter<int>("axes.right_trigger", 5);
        declare_parameter<int>("axes.dpad_y", 7);
        declare_parameter<int>("buttons.a", 0);
        declare_parameter<int>("buttons.x", 2);
        declare_parameter<int>("buttons.y", 3);
        declare_parameter<int>("buttons.start", 7);
        declare_parameter<double>("deadzone", 0.1);

        grid_w_ = get_parameter("draw.grid_width").as_int();
        grid_h_ = get_parameter("draw.grid_height").as_int();
        resolution_ = get_parameter("draw.resolution").as_double();
        cursor_speed_ = get_parameter("draw.cursor_speed").as_double();
        brush_size_ = get_parameter("draw.brush_size").as_int();
        double pub_rate = get_parameter("draw.publish_rate").as_double();

        ax_lx_ = get_parameter("axes.left_x").as_int();
        ax_ly_ = get_parameter("axes.left_y").as_int();
        ax_lt_ = get_parameter("axes.left_trigger").as_int();
        ax_rt_ = get_parameter("axes.right_trigger").as_int();
        ax_dpad_y_ = get_parameter("axes.dpad_y").as_int();
        btn_a_ = get_parameter("buttons.a").as_int();
        btn_x_ = get_parameter("buttons.x").as_int();
        btn_y_ = get_parameter("buttons.y").as_int();
        btn_start_ = get_parameter("buttons.start").as_int();
        deadzone_ = get_parameter("deadzone").as_double();

        // --- Initialise grid (all unknown) ---
        grid_.resize(
            static_cast<size_t>(grid_w_) * static_cast<size_t>(grid_h_), -1);

        cursor_x_ = grid_w_ / 2.0;
        cursor_y_ = grid_h_ / 2.0;

        // --- Publishers ---
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        event_pub_ = create_publisher<std_msgs::msg::String>(
            "/robot/events", 10);

        // --- Subscriber ---
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&DrawNode::joyCallback, this, std::placeholders::_1));

        // --- Timers ---
        auto map_period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / pub_rate));
        map_timer_ = create_wall_timer(
            map_period, std::bind(&DrawNode::publishMap, this));

        // 60 Hz internal cursor update
        cursor_timer_ = create_wall_timer(
            std::chrono::milliseconds(16),
            std::bind(&DrawNode::updateCursor, this));

        RCLCPP_INFO(get_logger(),
            "draw_node started — grid %dx%d @ %.2fm  brush=%d  rate=%.0fHz",
            grid_w_, grid_h_, resolution_, brush_size_, pub_rate);
        RCLCPP_INFO(get_logger(),
            "Controls: LStick=move  RT=draw  LT=erase  A=stamp  "
            "X=save  Y=clear  DpadUD=brush  Start=centre");
    }

private:
    // =========================================================================
    // Joy callback — cache analog/button state, detect edge-triggered actions
    // =========================================================================
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto axis = [&](int idx) -> double {
            return (idx >= 0 && static_cast<size_t>(idx) < msg->axes.size())
                ? static_cast<double>(msg->axes[idx]) : 0.0;
        };
        auto btn = [&](int idx) -> bool {
            return (idx >= 0 && static_cast<size_t>(idx) < msg->buttons.size())
                && msg->buttons[idx];
        };

        // Analog sticks — both axes negated: xpadneo reports
        // right-as-negative (axis 0) and up-as-negative (axis 1)
        joy_lx_ = -axis(ax_lx_);
        joy_ly_ = -axis(ax_ly_);

        // Triggers: driver 1.0 (released) → -1.0 (pressed) ⇒ 0..1
        draw_intensity_ = (1.0 - axis(ax_rt_)) / 2.0;
        erase_intensity_ = (1.0 - axis(ax_lt_)) / 2.0;

        a_held_ = btn(btn_a_);

        // --- Edge-triggered buttons ---
        bool x_now = btn(btn_x_);
        bool y_now = btn(btn_y_);
        bool start_now = btn(btn_start_);
        double dpad_y_now = axis(ax_dpad_y_);

        if (x_now && !x_prev_) {
            auto ev = std_msgs::msg::String();
            ev.data = "SAVE_MAP";
            event_pub_->publish(ev);
            RCLCPP_INFO(get_logger(), "SAVE_MAP requested (X button)");
        }
        if (y_now && !y_prev_) {
            std::fill(grid_.begin(), grid_.end(), -1);
            RCLCPP_INFO(get_logger(), "Grid cleared (Y button)");
        }
        if (start_now && !start_prev_) {
            cursor_x_ = grid_w_ / 2.0;
            cursor_y_ = grid_h_ / 2.0;
            RCLCPP_INFO(get_logger(), "Cursor reset to centre (Start button)");
        }

        // D-pad up/down for brush size (edge-triggered)
        if (dpad_y_now < -0.5 && dpad_y_prev_ >= -0.5) {
            brush_size_ = std::min(brush_size_ + 1, 10);
            RCLCPP_INFO(get_logger(), "Brush size increased to %d", brush_size_);
        }
        if (dpad_y_now > 0.5 && dpad_y_prev_ <= 0.5) {
            brush_size_ = std::max(brush_size_ - 1, 1);
            RCLCPP_INFO(get_logger(), "Brush size decreased to %d", brush_size_);
        }

        x_prev_ = x_now;
        y_prev_ = y_now;
        start_prev_ = start_now;
        dpad_y_prev_ = dpad_y_now;
    }

    // =========================================================================
    // Cursor update — runs at 60 Hz
    // =========================================================================
    void updateCursor()
    {
        constexpr double dt = 1.0 / 60.0;

        double lx = std::abs(joy_lx_) > deadzone_ ? joy_lx_ : 0.0;
        double ly = std::abs(joy_ly_) > deadzone_ ? joy_ly_ : 0.0;

        cursor_x_ += lx * cursor_speed_ * dt;
        cursor_y_ += ly * cursor_speed_ * dt;

        cursor_x_ = std::clamp(cursor_x_, 0.0,
            static_cast<double>(grid_w_ - 1));
        cursor_y_ = std::clamp(cursor_y_, 0.0,
            static_cast<double>(grid_h_ - 1));

        int cx = static_cast<int>(cursor_x_);
        int cy = static_cast<int>(cursor_y_);

        if (draw_intensity_ > 0.3 || a_held_) {
            paint(cx, cy, 100);
        } else if (erase_intensity_ > 0.3) {
            paint(cx, cy, 0);
        }
    }

    // =========================================================================
    // Paint / publish helpers
    // =========================================================================

    /** @brief Paint a square of size brush_size_ around (cx, cy). */
    void paint(int cx, int cy, int8_t value)
    {
        int half = brush_size_;
        for (int dy = -half + 1; dy < half; ++dy) {
            for (int dx = -half + 1; dx < half; ++dx) {
                int px = cx + dx;
                int py = cy + dy;
                if (px >= 0 && px < grid_w_ && py >= 0 && py < grid_h_) {
                    grid_[static_cast<size_t>(py * grid_w_ + px)] = value;
                }
            }
        }
    }

    /** @brief Publish the current grid as an OccupancyGrid with cursor overlay. */
    void publishMap()
    {
        auto msg = nav_msgs::msg::OccupancyGrid();
        msg.header.stamp = now();
        msg.header.frame_id = "map";
        msg.info.resolution = static_cast<float>(resolution_);
        msg.info.width = static_cast<uint32_t>(grid_w_);
        msg.info.height = static_cast<uint32_t>(grid_h_);
        msg.info.origin.position.x = -(grid_w_ * resolution_) / 2.0;
        msg.info.origin.position.y = -(grid_h_ * resolution_) / 2.0;

        // Copy grid data
        msg.data.assign(grid_.begin(), grid_.end());

        // Overlay cursor crosshair (value 50 = grey) so it's visible in web UI
        int cx = static_cast<int>(cursor_x_);
        int cy = static_cast<int>(cursor_y_);
        int arm = brush_size_ + 1;
        for (int i = -arm; i <= arm; ++i) {
            // Horizontal arm
            int hx = cx + i;
            if (hx >= 0 && hx < grid_w_) {
                msg.data[static_cast<size_t>(cy * grid_w_ + hx)] = 50;
            }
            // Vertical arm
            int vy = cy + i;
            if (vy >= 0 && vy < grid_h_) {
                msg.data[static_cast<size_t>(vy * grid_w_ + cx)] = 50;
            }
        }

        map_pub_->publish(msg);
    }

    // --- Grid state ---
    int grid_w_{100};
    int grid_h_{100};
    double resolution_{0.05};
    std::vector<int8_t> grid_;

    // --- Cursor ---
    double cursor_x_{0.0};
    double cursor_y_{0.0};
    double cursor_speed_{30.0};
    int brush_size_{2};

    // --- Cached joy state ---
    double joy_lx_{0.0};
    double joy_ly_{0.0};
    double draw_intensity_{0.0};
    double erase_intensity_{0.0};
    bool a_held_{false};
    double deadzone_{0.1};

    // --- Edge detection ---
    bool x_prev_{false};
    bool y_prev_{false};
    bool start_prev_{false};
    double dpad_y_prev_{0.0};

    // --- Axis / button indices ---
    int ax_lx_{0};
    int ax_ly_{1};
    int ax_lt_{2};
    int ax_rt_{5};
    int ax_dpad_y_{7};
    int btn_a_{0};
    int btn_x_{2};
    int btn_y_{3};
    int btn_start_{7};

    // --- ROS interfaces ---
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr cursor_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrawNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
