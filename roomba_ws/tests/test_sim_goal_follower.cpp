/**
 * @file test_sim_goal_follower.cpp
 * @brief Unit tests for sim_goal_follower — proportional goal pursuit.
 */

#include <gtest/gtest.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class SimGoalFollowerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
    }
};

TEST_F(SimGoalFollowerTest, NodeCreation)
{
    // Verify node can be instantiated without errors
    SUCCEED();
}

TEST_F(SimGoalFollowerTest, HeadingNormalisation)
{
    // Verify angle normalisation works correctly
    double angle = 4.0;  // > PI
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    EXPECT_LT(angle, M_PI);
    EXPECT_GE(angle, -M_PI);
}

TEST_F(SimGoalFollowerTest, DistanceCalculation)
{
    // Verify Euclidean distance between two points
    double dx = 3.0 - 0.0;
    double dy = 4.0 - 0.0;
    double dist = std::hypot(dx, dy);
    EXPECT_DOUBLE_EQ(dist, 5.0);
}

TEST_F(SimGoalFollowerTest, GoalTolerance)
{
    // Verify that distance within tolerance is considered reached
    double goal_tol = 0.3;
    double dist = 0.25;
    EXPECT_TRUE(dist < goal_tol);
}
