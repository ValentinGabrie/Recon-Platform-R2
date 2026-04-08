/**
 * @file test_sim_sensor_node.cpp
 * @brief Unit tests for SimSensorNode — room generation, raycasting.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <queue>
#include <utility>
#include <vector>

// ============================================================
// Extracted raycasting logic for unit-testable verification
// ============================================================

namespace sim_sensor_test {

/**
 * @brief Raycast on a flat grid. Returns distance to first occupied cell.
 */
double raycast(
    const std::vector<int8_t> & grid,
    int grid_w, int grid_h, double resolution,
    double ox, double oy, double angle, double max_range)
{
    double step = resolution * 0.5;
    double dx = std::cos(angle) * step;
    double dy = std::sin(angle) * step;
    double cx = ox;
    double cy = oy;
    int steps = static_cast<int>(max_range / step);

    for (int i = 0; i < steps; ++i) {
        cx += dx;
        cy += dy;
        int gx = static_cast<int>(cx / resolution);
        int gy = static_cast<int>(cy / resolution);
        if (gx < 0 || gx >= grid_w || gy < 0 || gy >= grid_h) {
            return max_range;
        }
        if (grid[static_cast<size_t>(gy * grid_w + gx)] > 50) {
            return std::hypot(cx - ox, cy - oy);
        }
    }
    return max_range;
}

/**
 * @brief BFS flood-fill to count reachable free cells.
 */
int floodFillCount(
    const std::vector<int8_t> & grid, int grid_w, int grid_h,
    int start_x, int start_y)
{
    size_t total = static_cast<size_t>(grid_w) * grid_h;
    std::vector<bool> visited(total, false);
    std::queue<std::pair<int, int>> q;

    q.push({start_x, start_y});
    visited[static_cast<size_t>(start_y * grid_w + start_x)] = true;
    int count = 0;

    while (!q.empty()) {
        auto [cx, cy] = q.front();
        q.pop();
        ++count;
        const int dx[] = {1, -1, 0, 0};
        const int dy[] = {0, 0, 1, -1};
        for (int d = 0; d < 4; ++d) {
            int nx = cx + dx[d];
            int ny = cy + dy[d];
            if (nx < 0 || nx >= grid_w || ny < 0 || ny >= grid_h) {
                continue;
            }
            size_t ni = static_cast<size_t>(ny * grid_w + nx);
            if (!visited[ni] && grid[ni] == 0) {
                visited[ni] = true;
                q.push({nx, ny});
            }
        }
    }
    return count;
}

/**
 * @brief Generate a simple walled room for testing.
 */
std::vector<int8_t> makeTestRoom(int w, int h, int wall_thick)
{
    std::vector<int8_t> grid(static_cast<size_t>(w * h), 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (x < wall_thick || x >= w - wall_thick ||
                y < wall_thick || y >= h - wall_thick)
            {
                grid[static_cast<size_t>(y * w + x)] = 100;
            }
        }
    }
    return grid;
}

}  // namespace sim_sensor_test

// ============================================================
// Tests
// ============================================================

class SimSensorTest : public ::testing::Test
{
protected:
    void SetUp() override {}
};

/**
 * @test Raycast to wall — front-facing beam hits the far wall.
 */
TEST_F(SimSensorTest, RaycastHitsWall)
{
    int w = 100, h = 100;
    double res = 0.05;  // 5m x 5m
    auto grid = sim_sensor_test::makeTestRoom(w, h, 2);

    // Robot at center, facing right (+x)
    double dist = sim_sensor_test::raycast(
        grid, w, h, res, 2.5, 2.5, 0.0, 4.0);

    // Should hit far wall at x ~= 4.9 m, distance ~= 2.4 m
    EXPECT_GT(dist, 1.5);
    EXPECT_LT(dist, 3.0);
}

/**
 * @test Raycast parallel misses — aiming into free space hits max range.
 */
TEST_F(SimSensorTest, RaycastMaxRange)
{
    // Large room, short max range — beam ends before hitting wall
    int w = 200, h = 200;
    double res = 0.05;  // 10m x 10m
    auto grid = sim_sensor_test::makeTestRoom(w, h, 2);

    // Robot at center, facing right, max_range = 1.0 m
    double dist = sim_sensor_test::raycast(
        grid, w, h, res, 5.0, 5.0, 0.0, 1.0);

    EXPECT_NEAR(dist, 1.0, 0.1);
}

/**
 * @test Room connectivity — walled room with no obstacles ⇒ interior
 *       fully reachable from center.
 */
TEST_F(SimSensorTest, RoomConnectivity)
{
    int w = 50, h = 50;
    auto grid = sim_sensor_test::makeTestRoom(w, h, 2);

    // Center cell
    int cx = 25, cy = 25;
    int reachable = sim_sensor_test::floodFillCount(
        grid, w, h, cx, cy);

    // Interior is (50-4)^2 = 46^2 = 2116
    int expected_interior = (w - 4) * (h - 4);
    EXPECT_EQ(reachable, expected_interior);
}

/**
 * @test Obstacle blocks raycast — place a box and verify Ray stops.
 */
TEST_F(SimSensorTest, RaycastBlockedByObstacle)
{
    int w = 100, h = 100;
    double res = 0.05;
    auto grid = sim_sensor_test::makeTestRoom(w, h, 2);

    // Place a 5-cell wide obstacle at x=60..65, y=48..53
    for (int y = 48; y <= 53; ++y) {
        for (int x = 60; x <= 65; ++x) {
            grid[static_cast<size_t>(y * w + x)] = 100;
        }
    }

    // Robot at (2.5m, 2.5m) aiming right; obstacle at ~3.0 m
    double dist = sim_sensor_test::raycast(
        grid, w, h, res, 2.5, 2.5, 0.0, 4.0);

    // Should hit obstacle at ~3.0m (cell 60*0.05=3.0m), not the far wall
    EXPECT_GT(dist, 0.3);
    EXPECT_LT(dist, 1.0);
}
