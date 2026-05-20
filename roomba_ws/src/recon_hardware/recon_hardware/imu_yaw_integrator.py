"""imu_yaw_integrator — gyro-Z → quaternion republisher for slam_toolbox.

slam_toolbox's `imu_topic` parameter expects a `sensor_msgs/Imu` message
with the **orientation quaternion** populated (it uses the yaw component
as a prior for scan matching). The ESP32 bridge publishes only
accelerometer + gyro (no fusion — there's no magnetometer on an
MPU-6050 and the chip's factory ZA_OFFSET makes naive accel-based tilt
unreliable, see docs/STATUS.md §7).

This node bridges the gap with the minimum thing that works:

  in:   /imu/data_raw   (sensor_msgs/Imu, BEST_EFFORT, ~100 Hz)
  out:  /imu/data       (sensor_msgs/Imu, RELIABLE,    ~100 Hz)

It integrates ``angular_velocity.z * dt`` into a running yaw estimate
and writes the corresponding (0, 0, sin(yaw/2), cos(yaw/2)) quaternion
into the output. Roll and pitch are left at zero — handheld scanning
is effectively 2-D, and the accel-derived tilt would be wrong anyway.

Drift handling: none in this node. Gyro bias adds up to a few degrees
per minute. slam_toolbox's scan matcher continuously corrects yaw using
the LIDAR data, so drift accumulated by this integrator is wiped out
each time a successful scan-match runs. Acceptable for typical scan
sessions; see docs/ROADMAP.md §H3 for the next steps if that changes.

Parameters:
    input_topic   (string)  default /imu/data_raw
    output_topic  (string)  default /imu/data
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Imu


def _stamp_seconds(stamp) -> float:
    """Convert a builtin_interfaces/Time stamp to a float of seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def integrate_yaw(prev_yaw: float, gyro_z: float, dt: float) -> float:
    """Pure-function version of the integrator step (kept testable).

    Returns the new yaw wrapped to (-π, π]. dt < 0 or > 1 s is clamped
    to 0 — sensor dropouts shouldn't produce huge jumps.
    """
    if dt <= 0.0 or dt > 1.0:
        return prev_yaw
    new_yaw = prev_yaw + float(gyro_z) * float(dt)
    # Wrap to (-π, π].
    new_yaw = math.fmod(new_yaw + math.pi, 2.0 * math.pi)
    if new_yaw <= 0.0:
        new_yaw += 2.0 * math.pi
    return new_yaw - math.pi


def yaw_to_quaternion_xyzw(yaw: float) -> tuple[float, float, float, float]:
    """Pure 2-D yaw → quaternion (x, y, z, w). Roll = pitch = 0."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class ImuYawIntegrator(Node):
    def __init__(self) -> None:
        super().__init__("imu_yaw_integrator")

        self.declare_parameter("input_topic",  "/imu/data_raw")
        self.declare_parameter("output_topic", "/imu/data")

        self._in_topic  = str(self.get_parameter("input_topic").value)
        self._out_topic = str(self.get_parameter("output_topic").value)

        self._yaw: float = 0.0
        self._last_t: Optional[float] = None

        # Match the bridge's BEST_EFFORT QoS on the input side; publish
        # RELIABLE because slam_toolbox subscribes with default reliable.
        be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self._sub = self.create_subscription(Imu, self._in_topic,
                                             self._on_imu, be)
        self._pub = self.create_publisher(Imu, self._out_topic, 10)

        self.get_logger().info(
            f"imu_yaw_integrator started — {self._in_topic} → {self._out_topic}")

    def _on_imu(self, msg: Imu) -> None:
        t_now = _stamp_seconds(msg.header.stamp)
        # If the publisher didn't fill the stamp, fall back to wall-clock
        # so the first dt isn't a huge number.
        if t_now == 0.0:
            t_now = self.get_clock().now().nanoseconds * 1e-9

        if self._last_t is not None:
            dt = t_now - self._last_t
            self._yaw = integrate_yaw(self._yaw,
                                       msg.angular_velocity.z,
                                       dt)
        self._last_t = t_now

        out = Imu()
        out.header = msg.header
        out.linear_acceleration = msg.linear_acceleration
        out.angular_velocity = msg.angular_velocity
        qx, qy, qz, qw = yaw_to_quaternion_xyzw(self._yaw)
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw
        # Mark orientation as "estimated with unknown covariance" — set
        # the first diagonal to a moderate value so EKFs downstream don't
        # treat the orientation as gospel.
        out.orientation_covariance[0] = 0.05  # rad²  (roll  — unknown, zeroed)
        out.orientation_covariance[4] = 0.05  # rad²  (pitch — unknown, zeroed)
        out.orientation_covariance[8] = 0.10  # rad²  (yaw   — gyro drift)
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuYawIntegrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
