"""Mock data generators for roomba web UI.

ALL mock logic lives here and nowhere else. No mock logic in routes,
templates, or other modules.

Each generator returns the current mock value when called.
"""

import math
import time
from typing import Any


def mock_sensor_distances() -> dict[str, float]:
    """Sine-wave oscillation 0.2–3.5 m per sensor, different phase offsets.

    Returns:
        Dict with front, left, right distances in metres.
    """
    t = time.time()
    mid = 1.85
    amp = 1.65
    return {
        "front": mid + amp * math.sin(t * 0.5),
        "left": mid + amp * math.sin(t * 0.5 + 2.094),      # +120°
        "right": mid + amp * math.sin(t * 0.5 + 4.189),      # +240°
    }


def mock_robot_pose() -> dict[str, float]:
    """Slow circular path, radius 1.5 m, period 30 s.

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
                data.append(100)   # Occupied (wall)
            else:
                data.append(0)     # Free
    return {
        "width": width,
        "height": height,
        "resolution": 0.05,
        "origin_x": -0.5,
        "origin_y": -0.5,
        "data": data,
    }


def mock_sensor_health() -> dict[str, bool]:
    """All sensors OK.

    Returns:
        Dict with front_ok, left_ok, right_ok.
    """
    return {
        "front_ok": True,
        "left_ok": True,
        "right_ok": True,
    }


def mock_controller_axes() -> dict[str, float]:
    """Gentle sine wave on left-Y, all other axes 0.0.

    Returns:
        Dict of axis values.
    """
    t = time.time()
    return {
        "left_x": 0.0,
        "left_y": 0.3 * math.sin(t * 0.3),
        "right_x": 0.0,
        "right_y": 0.0,
        "left_trigger": 0.0,
        "right_trigger": 0.0,
    }


def mock_controller_buttons() -> dict[str, bool]:
    """All unpressed.

    Returns:
        Dict of button states.
    """
    return {
        "a": False, "b": False, "x": False, "y": False,
        "lb": False, "rb": False, "start": False, "select": False,
        "left_stick": False, "right_stick": False,
        "dpad_up": False, "dpad_down": False,
        "dpad_left": False, "dpad_right": False,
    }


def mock_controller_state() -> dict[str, Any]:
    """Full controller state — connected is always False in mock.

    Returns:
        Dict with axes, buttons, connected, battery_pct.
    """
    return {
        "axes": mock_controller_axes(),
        "buttons": mock_controller_buttons(),
        "connected": False,
        "battery_pct": None,
    }


def mock_bluetooth_status() -> dict[str, Any]:
    """No device (mock).

    Returns:
        Dict with Bluetooth device info.
    """
    return {
        "name": "No device (mock)",
        "mac": "—",
        "connected": False,
        "paired": False,
        "trusted": False,
        "battery_pct": None,
    }
