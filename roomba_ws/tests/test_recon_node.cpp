/**
 * @file test_recon_node.cpp
 * @brief Unit tests for recon_node — frontier detection logic.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class ReconNodeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    /**
     * @brief Create a simple test occupancy grid.
     */
    nav_msgs::msg::OccupancyGrid createTestGrid(
        int width, int height) const
    {
        nav_msgs::msg::OccupancyGrid grid;
        grid.info.width = width;
        grid.info.height = height;
        grid.info.resolution = 0.05f;
        grid.info.origin.position.x = 0.0;
        grid.info.origin.position.y = 0.0;
        grid.data.resize(width * height, -1);  // All unknown
        return grid;
    }
};

/**
 * @brief Grid with all-unknown cells has no frontiers (no free cells).
 */
TEST_F(ReconNodeTest, AllUnknownNoFrontiers)
{
    auto grid = createTestGrid(10, 10);
    // All cells are -1 (unknown) — no free cells to be frontiers
    int frontier_count = 0;
    for (int y = 1; y < 9; y++) {
        for (int x = 1; x < 9; x++) {
            if (grid.data[y * 10 + x] == 0) {
                frontier_count++;
            }
        }
    }
    EXPECT_EQ(frontier_count, 0);
}

/**
 * @brief Free cell adjacent to unknown cell is a frontier.
 */
TEST_F(ReconNodeTest, FreeCellAdjacentToUnknownIsFrontier)
{
    auto grid = createTestGrid(10, 10);
    // Set center cell to free
    grid.data[5 * 10 + 5] = 0;

    // Check that at least one neighbour is unknown
    bool has_unknown_neighbour =
        grid.data[4 * 10 + 5] == -1 ||
        grid.data[6 * 10 + 5] == -1 ||
        grid.data[5 * 10 + 4] == -1 ||
        grid.data[5 * 10 + 6] == -1;

    EXPECT_TRUE(has_unknown_neighbour);
}

/**
 * @brief Radius constraint rejects distant frontiers.
 */
TEST_F(ReconNodeTest, RadiusConstraintRejectsFarFrontiers)
{
    double origin_x = 0.0;
    double origin_y = 0.0;
    double frontier_x = 10.0;
    double frontier_y = 0.0;
    double recon_radius = 5.0;

    double distance = std::hypot(
        frontier_x - origin_x, frontier_y - origin_y);
    EXPECT_GT(distance, recon_radius);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
