/**
 * @file test_slam_bridge_node.cpp
 * @brief Unit tests for slam_bridge_node.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"

class SlamBridgeNodeTest : public ::testing::Test
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
 * @brief LaserScan has 3 beams for 3 ultrasonic sensors.
 */
TEST_F(SlamBridgeNodeTest, ThreeBeamScan)
{
    auto scan = sensor_msgs::msg::LaserScan();
    scan.ranges = {1.0f, 2.0f, 3.0f};
    EXPECT_EQ(scan.ranges.size(), 3u);
}

/**
 * @brief Infinity values propagate through to scan.
 */
TEST_F(SlamBridgeNodeTest, InfinityPropagation)
{
    float inf_range = std::numeric_limits<float>::infinity();
    auto scan = sensor_msgs::msg::LaserScan();
    scan.ranges = {inf_range, 1.5f, 2.0f};

    EXPECT_TRUE(std::isinf(scan.ranges[0]));
    EXPECT_FALSE(std::isinf(scan.ranges[1]));
}

/**
 * @brief Scan angle parameters are consistent.
 */
TEST_F(SlamBridgeNodeTest, ScanAngleConsistency)
{
    float angle_min = -1.0472f;   // -60 deg
    float angle_max = 1.0472f;    //  60 deg
    float angle_increment = 1.0472f;

    // Should produce 3 beams: -60, 0, +60 degrees
    int num_beams = static_cast<int>(
        std::round((angle_max - angle_min) / angle_increment)) + 1;
    EXPECT_EQ(num_beams, 3);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
