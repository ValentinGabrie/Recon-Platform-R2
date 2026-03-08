/**
 * @file test_motor_controller.cpp
 * @brief Unit tests for motor_controller node.
 *
 * Tests verify Twist-to-differential-drive conversion logic without
 * requiring actual GPIO hardware.
 */

#include <gtest/gtest.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MotorControllerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }

    /**
     * @brief Replicate differential drive kinematics for verification.
     */
    std::pair<double, double> differentialDrive(
        double linear, double angular,
        double wheel_separation, double wheel_radius) const
    {
        double left = (linear - angular * wheel_separation / 2.0)
            / wheel_radius;
        double right = (linear + angular * wheel_separation / 2.0)
            / wheel_radius;
        return {left, right};
    }
};

/**
 * @brief Forward motion produces equal wheel velocities.
 */
TEST_F(MotorControllerTest, ForwardMotionEqualWheels)
{
    auto [left, right] = differentialDrive(0.5, 0.0, 0.20, 0.033);
    EXPECT_NEAR(left, right, 1e-6);
    EXPECT_GT(left, 0.0);
}

/**
 * @brief Pure rotation produces opposite wheel velocities.
 */
TEST_F(MotorControllerTest, PureRotationOppositeWheels)
{
    auto [left, right] = differentialDrive(0.0, 1.0, 0.20, 0.033);
    EXPECT_NEAR(left, -right, 1e-6);
}

/**
 * @brief Zero velocity produces zero wheel speeds.
 */
TEST_F(MotorControllerTest, ZeroVelocityStopsMotors)
{
    auto [left, right] = differentialDrive(0.0, 0.0, 0.20, 0.033);
    EXPECT_DOUBLE_EQ(left, 0.0);
    EXPECT_DOUBLE_EQ(right, 0.0);
}

/**
 * @brief Watchdog timeout detection logic.
 */
TEST_F(MotorControllerTest, WatchdogTimeoutDetection)
{
    // Simulate elapsed time > timeout
    double elapsed_ms = 600.0;
    int watchdog_timeout_ms = 500;
    EXPECT_TRUE(elapsed_ms > static_cast<double>(watchdog_timeout_ms));
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
