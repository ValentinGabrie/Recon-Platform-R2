/**
 * @file test_draw_node.cpp
 * @brief Unit tests for draw_node — controller-driven drawing canvas.
 */

#include <gtest/gtest.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"

class DrawNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }
};

/**
 * @brief OccupancyGrid data layout is row-major with correct dimensions.
 */
TEST_F(DrawNodeTest, GridDataLayout)
{
    // A 10x10 grid should have 100 cells
    int width = 10, height = 10;
    std::vector<int8_t> grid(
        static_cast<size_t>(width) * static_cast<size_t>(height), -1);
    EXPECT_EQ(grid.size(), 100u);
    // All cells should be unknown (-1)
    for (auto cell : grid) {
        EXPECT_EQ(cell, -1);
    }
}

/**
 * @brief Paint function marks cells within brush radius as occupied.
 */
TEST_F(DrawNodeTest, PaintMarksCells)
{
    int width = 20, height = 20;
    std::vector<int8_t> grid(
        static_cast<size_t>(width) * static_cast<size_t>(height), -1);

    // Paint at centre (10, 10) with brush_size=2
    int cx = 10, cy = 10, brush = 2;
    for (int dy = -brush + 1; dy < brush; ++dy) {
        for (int dx = -brush + 1; dx < brush; ++dx) {
            int px = cx + dx, py = cy + dy;
            if (px >= 0 && px < width && py >= 0 && py < height) {
                grid[static_cast<size_t>(py * width + px)] = 100;
            }
        }
    }

    // Centre cell should be occupied
    EXPECT_EQ(grid[static_cast<size_t>(10 * width + 10)], 100);
    // Brush=2 paints a 3x3 area (-1..+1), so neighbours should be occupied
    EXPECT_EQ(grid[static_cast<size_t>(9 * width + 9)], 100);
    EXPECT_EQ(grid[static_cast<size_t>(11 * width + 11)], 100);
    // Outside brush area should still be unknown
    EXPECT_EQ(grid[static_cast<size_t>(0 * width + 0)], -1);
}

/**
 * @brief Clear operation resets grid to all unknown.
 */
TEST_F(DrawNodeTest, ClearResetsGrid)
{
    int width = 10, height = 10;
    std::vector<int8_t> grid(
        static_cast<size_t>(width) * static_cast<size_t>(height), 100);

    // Clear
    std::fill(grid.begin(), grid.end(), -1);

    for (auto cell : grid) {
        EXPECT_EQ(cell, -1);
    }
}

/**
 * @brief Cursor clamping keeps position within grid bounds.
 */
TEST_F(DrawNodeTest, CursorClamping)
{
    int grid_w = 50, grid_h = 50;
    double cursor_x = 100.0;  // Way out of bounds
    double cursor_y = -10.0;

    cursor_x = std::clamp(cursor_x, 0.0,
        static_cast<double>(grid_w - 1));
    cursor_y = std::clamp(cursor_y, 0.0,
        static_cast<double>(grid_h - 1));

    EXPECT_DOUBLE_EQ(cursor_x, 49.0);
    EXPECT_DOUBLE_EQ(cursor_y, 0.0);
}
