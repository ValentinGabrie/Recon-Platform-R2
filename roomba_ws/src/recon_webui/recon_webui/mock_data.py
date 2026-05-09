"""Mock data generators for Recon-Platform-R2 web UI.

ALL mock logic lives here and nowhere else. No mock logic in routes,
templates, or other modules.

Each generator returns the current mock value when called.
"""

import math
import time
from typing import Any


def mock_robot_pose() -> dict[str, float]:
    """Slow circular path, radius 1.5 m, period 30 s.

    Used by the map viewer when no real /tf or /scanner/pose is available
    (e.g. before the IMU/EKF pipeline is running).

    Returns:
        Dict with x, y, theta.
    """
    t = time.time()
    radius = 1.5
    period = 30.0
    angle = 2.0 * math.pi * t / period
    return {
        "x": radius * math.cos(angle),
        "y": radius * math.sin(angle),
        "theta": angle + math.pi / 2.0,
    }


def mock_occupancy_grid() -> dict[str, Any]:
    """Static pre-baked 20x20 grid: walls on edges, free space inside.

    Returns:
        Dict representing a simplified OccupancyGrid.
    """
    width = 20
    height = 20
    data = []
    for y in range(height):
        for x in range(width):
            if x == 0 or x == width - 1 or y == 0 or y == height - 1:
                data.append(100)
            else:
                data.append(0)
    return {
        "width": width,
        "height": height,
        "resolution": 0.05,
        "origin_x": -0.5,
        "origin_y": -0.5,
        "data": data,
    }
