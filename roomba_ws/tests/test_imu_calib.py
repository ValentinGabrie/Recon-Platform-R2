"""Unit tests for recon_hardware.imu_calib.ImuCalibrator.

The calibrator is a plain class (no rclpy), so the bias-removal logic and
the still-detection autocal can be exercised in isolation — the same
philosophy as test_imu_yaw_integrator.py.
"""

import pytest

from recon_hardware.imu_calib import ImuCalibrator, GRAVITY


def test_static_accel_bias_subtracted():
    # Z reads the factory-biased ~15.37; subtracting 5.56 lands near gravity.
    cal = ImuCalibrator(autocal=False, accel_bias=(0.0, 0.0, 5.56))
    ax, ay, az, gx, gy, gz = cal.apply(-0.53, 0.12, 15.37, 0.0, 0.0, 0.0)
    assert az == pytest.approx(GRAVITY, abs=0.05)
    assert ax == pytest.approx(-0.53)  # untouched (bias 0)
    assert ay == pytest.approx(0.12)


def test_static_gyro_bias_subtracted_when_autocal_off():
    cal = ImuCalibrator(autocal=False, gyro_bias=(0.01, -0.02, 0.016))
    _, _, _, gx, gy, gz = cal.apply(0.0, 0.0, 9.81, 0.10, 0.10, 0.10)
    assert gx == pytest.approx(0.09)
    assert gy == pytest.approx(0.12)
    assert gz == pytest.approx(0.084)
    assert cal.calibrated is True  # static config is final immediately


def test_autocal_estimates_bias_when_still():
    cal = ImuCalibrator(autocal=True, autocal_samples=50, still_thresh=0.05)
    # Feed a constant gyro offset (perfectly still device biased at +0.016 Z).
    for _ in range(50):
        cal.apply(0.0, 0.0, 9.81, -0.006, 0.023, 0.016)
    assert cal.calibrated is True
    assert cal.rejected is False
    bx, by, bz = cal.gyro_bias
    assert bz == pytest.approx(0.016, abs=1e-6)
    assert bx == pytest.approx(-0.006, abs=1e-6)
    # Once calibrated, that bias is removed from new samples.
    *_, gz = cal.apply(0.0, 0.0, 9.81, -0.006, 0.023, 0.016)
    assert gz == pytest.approx(0.0, abs=1e-6)


def test_autocal_rejected_when_moving_keeps_fallback():
    cal = ImuCalibrator(
        autocal=True, autocal_samples=50, still_thresh=0.05,
        gyro_bias=(0.0, 0.0, 0.0),
    )
    # Large alternating gyro → stddev well above the still threshold.
    for i in range(50):
        sign = 1.0 if i % 2 == 0 else -1.0
        cal.apply(0.0, 0.0, 9.81, 0.0, 0.0, sign * 0.5)
    assert cal.calibrated is True
    assert cal.rejected is True
    assert cal.gyro_bias == (0.0, 0.0, 0.0)  # fallback, not the moving mean


def test_collecting_flag_tracks_window():
    cal = ImuCalibrator(autocal=True, autocal_samples=10)
    assert cal.collecting is True
    for _ in range(9):
        cal.apply(0.0, 0.0, 9.81, 0.0, 0.0, 0.01)
    assert cal.collecting is True   # not full yet
    cal.apply(0.0, 0.0, 9.81, 0.0, 0.0, 0.01)
    assert cal.collecting is False  # window completed
