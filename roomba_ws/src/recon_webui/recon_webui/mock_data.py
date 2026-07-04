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


def mock_imu_sample() -> dict[str, float]:
    """Synthetic IMU sample: device "flat" with a slow sinusoidal wobble
    on the gyro so the Stats page sparklines have something to draw.
    """
    t = time.time()
    return {
        "stamp_ns": int(t * 1e9),
        "wall_s":   t,
        "ax":  0.05 * math.sin(0.5 * t),
        "ay": -0.03 * math.sin(0.4 * t),
        "az":  9.80665,
        "gx":  0.02 * math.sin(0.7 * t),
        "gy": -0.02 * math.sin(0.8 * t),
        "gz":  0.01 * math.sin(0.3 * t),
    }


def mock_bridge_health() -> dict[str, Any]:
    """Mock /esp32/diagnostics payload — shows the Stats page something
    sensible when the bridge isn't running. Counts and uptime simulate a
    bridge that's been running ~30 s.
    """
    t = time.time()
    uptime_ms = int(((t * 1000) % 600000))  # 0..600 s loop
    return {
        "port": "/dev/ttyUSB0",
        "baud": 115200,
        "port_open": False,
        "frame_counts": {
            "imu":       int(uptime_ms / 10),     # ~100 Hz
            "heartbeat": int(uptime_ms / 1000),
            "button":    0,
            "status":    1,
            "crc_fail":  0,
            "bad_len":   0,
        },
        "seconds_since_last_frame": None,
        "esp32_uptime_ms": uptime_ms,
        "recent_buttons": [],
        "status": {
            "flags": 0x03,
            "imu_ok": True,
            "who_am_i": 0x68,
            "accel_cfg": 0x08,
            "gyro_cfg":  0x08,
            "afs_sel":   1,
            "fs_sel":    1,
            "za_offset_before": 1544,
            "za_offset_after":  1544,
        },
    }
