"""Unit tests for the pure-function helpers in imu_yaw_integrator.

The full node needs rclpy + a live serial publisher to exercise — what
we *can* test in isolation is the integration math: dt clamping, wrap
behaviour, quaternion form.
"""

import math

import pytest

from recon_hardware.imu_yaw_integrator import (
    integrate_yaw,
    yaw_to_quaternion_xyzw,
)


def test_zero_dt_returns_unchanged():
    assert integrate_yaw(1.234, 5.0, 0.0) == 1.234


def test_negative_dt_returns_unchanged():
    assert integrate_yaw(1.234, 5.0, -0.001) == 1.234


def test_huge_dt_returns_unchanged():
    # 1 s+ between samples almost certainly means a dropout; don't trust
    # the integration over that gap.
    assert integrate_yaw(0.0, 1.0, 1.5) == 0.0


def test_normal_integration_step():
    # 0.1 rad/s for 0.01 s = +0.001 rad
    new_yaw = integrate_yaw(0.0, 0.1, 0.01)
    assert new_yaw == pytest.approx(0.001, abs=1e-9)


def test_negative_gyro_decrements_yaw():
    new_yaw = integrate_yaw(0.5, -1.0, 0.1)
    assert new_yaw == pytest.approx(0.4, abs=1e-9)


def test_wraparound_above_pi():
    # Start near +π, push past — should wrap to negative side.
    new_yaw = integrate_yaw(math.pi - 0.05, 0.5, 0.2)  # +0.1 rad
    # Expect ~ -π + 0.05
    assert new_yaw == pytest.approx(-math.pi + 0.05, abs=1e-9)


def test_wraparound_below_minus_pi():
    new_yaw = integrate_yaw(-math.pi + 0.05, -0.5, 0.2)  # -0.1 rad
    # Expect ~ +π - 0.05
    assert new_yaw == pytest.approx(math.pi - 0.05, abs=1e-9)


def test_quaternion_zero_yaw_is_identity():
    qx, qy, qz, qw = yaw_to_quaternion_xyzw(0.0)
    assert (qx, qy, qz, qw) == (0.0, 0.0, 0.0, 1.0)


def test_quaternion_90_degrees():
    # 90° around Z → quaternion (0, 0, sin(45°), cos(45°))
    qx, qy, qz, qw = yaw_to_quaternion_xyzw(math.pi / 2.0)
    assert qx == 0.0 and qy == 0.0
    assert qz == pytest.approx(math.sqrt(2) / 2, abs=1e-9)
    assert qw == pytest.approx(math.sqrt(2) / 2, abs=1e-9)


def test_quaternion_180_degrees():
    qx, qy, qz, qw = yaw_to_quaternion_xyzw(math.pi)
    assert qx == 0.0 and qy == 0.0
    assert qz == pytest.approx(1.0, abs=1e-9)
    assert qw == pytest.approx(0.0, abs=1e-9)


def test_quaternion_unit_norm():
    # Every quaternion we produce must be unit-length (orthonormality
    # is non-negotiable for TF consumers).
    for yaw in (-math.pi, -1.0, 0.0, 0.123, 1.0, math.pi):
        qx, qy, qz, qw = yaw_to_quaternion_xyzw(yaw)
        norm_sq = qx*qx + qy*qy + qz*qz + qw*qw
        assert norm_sq == pytest.approx(1.0, abs=1e-9)


def test_long_integration_converges_correctly():
    """1 rad/s for 100 ms × 10 = +1 rad total, no wrap involved."""
    yaw = 0.0
    for _ in range(10):
        yaw = integrate_yaw(yaw, 1.0, 0.1)
    assert yaw == pytest.approx(1.0, abs=1e-9)
