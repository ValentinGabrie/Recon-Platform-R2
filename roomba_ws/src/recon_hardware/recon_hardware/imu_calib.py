"""imu_calib — bias removal for raw MPU-6050 samples.

The ESP32 ships the IMU's *raw factory-calibrated counts* on purpose (the
firmware deliberately leaves the accelerometer's factory ZA_OFFSET intact —
see docs/STATUS.md §7 issue #2, and the long comment in
firmware/esp32/src/imu.cpp). That means two biases reach the Pi untouched:

  * **Gyro bias** — a small constant per-axis offset (a stationary MPU-6050
    typically reads a few tenths of a °/s). Left alone it integrates into a
    steadily drifting yaw, which is exactly what `imu_yaw_integrator` feeds
    slam_toolbox as a scan-match prior.

  * **Accel bias** — the factory ZA_OFFSET makes gravity read ~15 m/s² on Z
    instead of 9.81 on this unit. Nothing downstream removed it (the H3 EKF
    ignores `linear_acceleration` entirely, issue #1), so it just showed up
    as obviously-wrong numbers on the Stats page.

`ImuCalibrator` removes both. It's a plain class (no rclpy) so it can be
unit-tested in isolation, the same way the integrator's math is.

Gyro bias is *auto-estimated* at startup: average the first
``autocal_samples`` samples, but only accept that average as the bias if the
device was actually still during the window (per-axis sample stddev under
``still_thresh``). If it was moving, we keep the static ``gyro_bias`` fallback
from config rather than baking motion into the bias. Accel bias is a fixed
configured offset — gravity is a real signal on the up-axis, so it can't be
auto-zeroed without knowing the device's orientation.
"""

from __future__ import annotations

from statistics import mean, pstdev


GRAVITY = 9.80665  # m/s² — reference magnitude, for documentation/callers


class ImuCalibrator:
    """Subtract gyro + accel bias from raw IMU samples.

    Args:
        autocal:          estimate gyro bias from the first still window.
        autocal_samples:  samples to average for the gyro-bias estimate
                          (at 100 Hz, 200 ≈ 2 s).
        still_thresh:     max per-axis gyro stddev (rad/s) over the window
                          for it to count as "still". Above this the device
                          was moving and the static fallback is kept.
        gyro_bias:        (x, y, z) static gyro bias (rad/s). Used as the
                          fallback when autocal is off or rejected, and as
                          the active bias until autocal completes.
        accel_bias:       (x, y, z) static accel offset (m/s²) subtracted
                          from every sample.
    """

    def __init__(
        self,
        *,
        autocal: bool = True,
        autocal_samples: int = 200,
        still_thresh: float = 0.05,
        gyro_bias: tuple[float, float, float] = (0.0, 0.0, 0.0),
        accel_bias: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self._autocal = bool(autocal)
        self._n_target = max(1, int(autocal_samples))
        self._still_thresh = float(still_thresh)
        self._static_gyro_bias = tuple(float(v) for v in gyro_bias)
        self._accel_bias = tuple(float(v) for v in accel_bias)

        # Active gyro bias — starts at the static fallback; autocal may
        # replace it once the startup window completes.
        self._gyro_bias = self._static_gyro_bias
        # "calibrated" means the gyro bias is final. With autocal off the
        # static config is already final, so we're calibrated from the start.
        self._calibrated = not self._autocal
        self._rejected = False  # True if autocal ran but the device was moving

        # Per-axis accumulators for the autocal window.
        self._gx: list[float] = []
        self._gy: list[float] = []
        self._gz: list[float] = []

    # ---- introspection (surfaced in /esp32/diagnostics) --------------------
    @property
    def gyro_bias(self) -> tuple[float, float, float]:
        return self._gyro_bias

    @property
    def accel_bias(self) -> tuple[float, float, float]:
        return self._accel_bias

    @property
    def calibrated(self) -> bool:
        return self._calibrated

    @property
    def collecting(self) -> bool:
        """True while the autocal window is still filling."""
        return self._autocal and not self._calibrated

    @property
    def rejected(self) -> bool:
        """True if autocal completed but the device was moving (fallback kept)."""
        return self._rejected

    # ---- hot path ----------------------------------------------------------
    def apply(
        self,
        ax: float, ay: float, az: float,
        gx: float, gy: float, gz: float,
    ) -> tuple[float, float, float, float, float, float]:
        """Return the bias-corrected (ax, ay, az, gx, gy, gz)."""
        if self._autocal and not self._calibrated:
            self._gx.append(gx)
            self._gy.append(gy)
            self._gz.append(gz)
            if len(self._gz) >= self._n_target:
                self._finish_autocal()

        bx, by, bz = self._gyro_bias
        cx, cy, cz = self._accel_bias
        return (ax - cx, ay - cy, az - cz, gx - bx, gy - by, gz - bz)

    def _finish_autocal(self) -> None:
        """Accept the windowed mean as the gyro bias iff the device was still."""
        spread = max(pstdev(self._gx), pstdev(self._gy), pstdev(self._gz))
        if spread <= self._still_thresh:
            self._gyro_bias = (mean(self._gx), mean(self._gy), mean(self._gz))
        else:
            # Moving during calibration — don't bake motion into the bias.
            self._gyro_bias = self._static_gyro_bias
            self._rejected = True
        self._calibrated = True
        # Free the accumulators — we never reopen the window.
        self._gx = self._gy = self._gz = []
