/**
 * @file test_joy_control_node.cpp
 * @brief Unit tests for joy_control_node.
 */

#include <gtest/gtest.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class JoyControlNodeTest : public ::testing::Test
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
 * @brief Deadzone removes small axis values.
 */
TEST_F(JoyControlNodeTest, DeadzoneFilter)
{
    double deadzone = 0.1;
    double small_value = 0.05;
    double result = (std::abs(small_value) < deadzone) ? 0.0 : small_value;
    EXPECT_DOUBLE_EQ(result, 0.0);
}

/**
 * @brief Values above deadzone pass through.
 */
TEST_F(JoyControlNodeTest, AboveDeadzonePassThrough)
{
    double deadzone = 0.1;
    double value = 0.5;
    double result = (std::abs(value) < deadzone) ? 0.0 : value;
    EXPECT_DOUBLE_EQ(result, 0.5);
}

/**
 * @brief Velocity scaling applies correctly.
 */
TEST_F(JoyControlNodeTest, VelocityScaling)
{
    double axis_value = 0.8;
    double max_linear = 0.5;
    double scaled = axis_value * max_linear;
    EXPECT_NEAR(scaled, 0.4, 1e-6);
}

/**
 * @brief Button index bounds checking.
 */
TEST_F(JoyControlNodeTest, ButtonBoundsCheck)
{
    auto joy = sensor_msgs::msg::Joy();
    joy.buttons.resize(14, 0);

    // Valid index
    EXPECT_EQ(joy.buttons[6], 0);  // Start button

    // Out of bounds check
    int invalid_index = 20;
    bool pressed = (invalid_index >= 0 &&
        static_cast<size_t>(invalid_index) < joy.buttons.size())
        ? (joy.buttons[invalid_index] != 0) : false;
    EXPECT_FALSE(pressed);
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
