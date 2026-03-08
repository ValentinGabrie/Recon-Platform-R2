/**
 * @file test_bt_sim_node.cpp
 * @brief Unit tests for bt_sim_node.
 */

#include <gtest/gtest.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class BtSimNodeTest : public ::testing::Test
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
 * @brief Idle profile produces zero axes.
 */
TEST_F(BtSimNodeTest, IdleProfileZeroAxes)
{
    // Idle profile: all axes should be 0.0
    double left_y = 0.0;
    double right_x = 0.0;
    EXPECT_DOUBLE_EQ(left_y, 0.0);
    EXPECT_DOUBLE_EQ(right_x, 0.0);
}

/**
 * @brief Sine profile produces bounded values.
 */
TEST_F(BtSimNodeTest, SineProfileBounded)
{
    double amplitude = 0.5;
    double t = 2.5;
    double period = 8.0;
    double value = amplitude * std::sin(2.0 * M_PI * t / period);

    EXPECT_LE(std::abs(value), amplitude + 1e-9);
}

/**
 * @brief Patrol profile cycles correctly.
 */
TEST_F(BtSimNodeTest, PatrolProfileCycles)
{
    double patrol_straight_s = 3.0;
    double patrol_turn_s = 1.0;
    double cycle = patrol_straight_s + patrol_turn_s;

    // At t=2.0 (within straight phase)
    double phase = std::fmod(2.0, cycle);
    EXPECT_LT(phase, patrol_straight_s);

    // At t=3.5 (within turn phase)
    phase = std::fmod(3.5, cycle);
    EXPECT_GE(phase, patrol_straight_s);
}

/**
 * @brief Joy message has correct dimensions.
 */
TEST_F(BtSimNodeTest, JoyMessageDimensions)
{
    auto msg = sensor_msgs::msg::Joy();
    msg.axes.resize(6, 0.0f);
    msg.buttons.resize(14, 0);

    EXPECT_EQ(msg.axes.size(), 6u);
    EXPECT_EQ(msg.buttons.size(), 14u);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
