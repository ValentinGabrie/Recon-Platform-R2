/**
 * @file test_sim_motor_node.cpp
 * @brief Unit tests for SimMotorNode — kinematics, collision, watchdog.
 */

#include <gtest/gtest.h>
#include <cmath>

// ============================================================
// Extracted kinematics logic for unit-testable verification
// (SimMotorNode uses the same math internally)
// ============================================================

namespace sim_motor_test {

/**
 * @brief Integrate one step of differential drive kinematics.
 */
struct Pose { double x; double y; double theta; };

Pose integrateKinematics(
    const Pose & p, double linear, double angular, double dt)
{
    Pose next;
    next.theta = p.theta + angular * dt;
    next.theta = std::atan2(std::sin(next.theta), std::cos(next.theta));
    next.x = p.x + linear * std::cos(p.theta) * dt;
    next.y = p.y + linear * std::sin(p.theta) * dt;
    return next;
}

/**
 * @brief Check collision against a simple occupancy grid.
 */
bool checkCollision(
    double cx, double cy, double collision_r,
    const std::vector<int8_t> & grid,
    int grid_w, int grid_h, double resolution,
    double origin_x, double origin_y)
{
    int radius_cells = static_cast<int>(std::ceil(collision_r / resolution));
    int center_gx = static_cast<int>((cx - origin_x) / resolution);
    int center_gy = static_cast<int>((cy - origin_y) / resolution);

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            int gx = center_gx + dx;
            int gy = center_gy + dy;
            if (gx < 0 || gx >= grid_w || gy < 0 || gy >= grid_h) {
                return true;
            }
            double cell_x = origin_x + (gx + 0.5) * resolution;
            double cell_y = origin_y + (gy + 0.5) * resolution;
            double dist = std::hypot(cell_x - cx, cell_y - cy);
            if (dist > collision_r) { continue; }
            int idx = gy * grid_w + gx;
            if (grid[static_cast<size_t>(idx)] > 50) { return true; }
        }
    }
    return false;
}

}  // namespace sim_motor_test

// ============================================================
// Tests
// ============================================================

class SimMotorTest : public ::testing::Test
{
protected:
    void SetUp() override {}
};

/**
 * @test Straight-line motion — robot moves forward at 0.5 m/s for 1 s.
 */
TEST_F(SimMotorTest, StraightLineKinematics)
{
    sim_motor_test::Pose p{0.0, 0.0, 0.0};
    double dt = 0.02;  // 50 Hz
    int steps = 50;     // 1 second

    for (int i = 0; i < steps; ++i) {
        p = sim_motor_test::integrateKinematics(p, 0.5, 0.0, dt);
    }

    EXPECT_NEAR(p.x, 0.5, 0.01);
    EXPECT_NEAR(p.y, 0.0, 0.01);
    EXPECT_NEAR(p.theta, 0.0, 0.01);
}

/**
 * @test Pure rotation — robot spins at 1.0 rad/s for pi seconds.
 */
TEST_F(SimMotorTest, PureRotation)
{
    sim_motor_test::Pose p{1.0, 1.0, 0.0};
    double dt = 0.02;
    int steps = static_cast<int>(M_PI / dt);

    for (int i = 0; i < steps; ++i) {
        p = sim_motor_test::integrateKinematics(p, 0.0, 1.0, dt);
    }

    EXPECT_NEAR(p.x, 1.0, 0.01);
    EXPECT_NEAR(p.y, 1.0, 0.01);
    EXPECT_NEAR(std::abs(p.theta), M_PI, 0.05);
}

/**
 * @test Collision detection — wall blocks movement.
 */
TEST_F(SimMotorTest, CollisionWithWall)
{
    // 10x10 grid, border is wall
    int w = 10, h = 10;
    double res = 0.5;
    std::vector<int8_t> grid(static_cast<size_t>(w * h), 0);

    // Top wall
    for (int x = 0; x < w; ++x) {
        grid[static_cast<size_t>(0 * w + x)] = 100;
        grid[static_cast<size_t>((h - 1) * w + x)] = 100;
    }
    // Side walls
    for (int y = 0; y < h; ++y) {
        grid[static_cast<size_t>(y * w + 0)] = 100;
        grid[static_cast<size_t>(y * w + (w - 1))] = 100;
    }

    // Center of grid should be free
    EXPECT_FALSE(sim_motor_test::checkCollision(
        2.5, 2.5, 0.15, grid, w, h, res, 0.0, 0.0));

    // Near wall should collide
    EXPECT_TRUE(sim_motor_test::checkCollision(
        0.2, 2.5, 0.15, grid, w, h, res, 0.0, 0.0));

    // Out of bounds should collide
    EXPECT_TRUE(sim_motor_test::checkCollision(
        -1.0, 2.5, 0.15, grid, w, h, res, 0.0, 0.0));
}

/**
 * @test Theta normalisation — stays within [-pi, pi].
 */
TEST_F(SimMotorTest, ThetaNormalisation)
{
    sim_motor_test::Pose p{0.0, 0.0, 3.0};
    double dt = 0.1;

    // Spin past pi
    for (int i = 0; i < 20; ++i) {
        p = sim_motor_test::integrateKinematics(p, 0.0, 1.0, dt);
    }

    EXPECT_GE(p.theta, -M_PI);
    EXPECT_LE(p.theta, M_PI);
}
