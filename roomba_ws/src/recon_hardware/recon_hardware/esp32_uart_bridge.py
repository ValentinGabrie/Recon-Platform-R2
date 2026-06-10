"""esp32_uart_bridge — ROS2 node that turns ESP32 UART frames into ROS topics.

Reads the binary frame stream coming from firmware/esp32/ (see
docs/UART_PROTOCOL.md) on a configurable serial port and republishes:

  * sensor_msgs/Imu       on /imu/data_raw         (raw IMU @ ~100 Hz)
  * std_msgs/Empty        on /buttons/save         (one per PRESSED edge)
  * std_msgs/Empty        on /buttons/startstop    (formerly /buttons/reset)
  * std_msgs/Empty        on /buttons/shutdown_request   (PRESSED) +
                          on /buttons/shutdown_longpress (LONGPRESS)
  * std_msgs/String       on /esp32/diagnostics    (JSON, 1 Hz)

The diagnostics topic is a small JSON blob the web UI consumes to render
its link-health card — frame counts, CRC failures, ESP32 uptime, the
boot STATUS payload (who_am_i, AFS_SEL, ZA_offset).

Threading: serial I/O happens in a Python `threading.Thread` (real OS
thread, not rclpy executor) because pyserial calls would otherwise
block the ROS2 callback queue. Frames are parsed there and published
via rclpy publishers, which are thread-safe.

Parameters (declare via --ros-args -p name:=value):
    port          (string)  default /dev/ttyUSB0
    baud          (int)     default 115200
    frame_id      (string)  default imu_link
    diag_period_s (double)  default 1.0      — diagnostics publish period
    quiet_imu_warn(bool)    default false    — suppress per-frame warns
"""

from __future__ import annotations

import fcntl
import json
import math
import os
import pty
import termios
import threading
import time
import tty
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool

from recon_hardware.framing import (
    ButtonId,
    ButtonState,
    FrameParser,
    FrameType,
    StatusFrame,
    encode_lidar_en,
)
from recon_hardware.imu_calib import ImuCalibrator


class Esp32UartBridge(Node):
    def __init__(self) -> None:
        super().__init__("esp32_uart_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 460800)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("diag_period_s", 1.0)
        self.declare_parameter("quiet_imu_warn", False)
        # While the LIDAR motor is enabled the bridge re-sends LIDAR_EN=1
        # on this period so the ESP32 watchdog (3 s) never trips. Keep this
        # comfortably under the firmware's LIDAR_WATCHDOG_MS.
        self.declare_parameter("lidar_refresh_s", 1.0)
        # Stable symlink for the LD14P driver to open. The pty itself is at
        # /dev/pts/N — we symlink so the driver config stays human-readable.
        self.declare_parameter("lidar_pty_link", "/tmp/lidar_pty")

        # ---- IMU bias calibration (see recon_hardware.imu_calib) -----------
        # The ESP32 sends raw factory-calibrated counts; these knobs remove
        # the residual gyro + accel bias on the Pi side. Defaults: auto-zero
        # the gyro from a 2 s still window at startup, and subtract the
        # measured accel ZA_OFFSET so gravity reads ~9.81 m/s² on Z.
        self.declare_parameter("imu_gyro_autocal", True)
        self.declare_parameter("imu_gyro_autocal_samples", 200)
        self.declare_parameter("imu_gyro_still_thresh", 0.05)
        self.declare_parameter("imu_gyro_bias", [0.0, 0.0, 0.0])
        self.declare_parameter("imu_accel_bias", [0.0, 0.0, 0.0])

        self._port    = self.get_parameter("port").value
        self._baud    = int(self.get_parameter("baud").value)
        self._frame   = str(self.get_parameter("frame_id").value)
        self._diag_dt = float(self.get_parameter("diag_period_s").value)
        self._quiet   = bool(self.get_parameter("quiet_imu_warn").value)
        self._lidar_refresh_dt = float(self.get_parameter("lidar_refresh_s").value)
        self._lidar_pty_link   = str(self.get_parameter("lidar_pty_link").value)

        # IMU calibrator — applied to every IMU sample before publish.
        self._calib = ImuCalibrator(
            autocal=bool(self.get_parameter("imu_gyro_autocal").value),
            autocal_samples=int(self.get_parameter("imu_gyro_autocal_samples").value),
            still_thresh=float(self.get_parameter("imu_gyro_still_thresh").value),
            gyro_bias=tuple(float(v) for v in self.get_parameter("imu_gyro_bias").value),
            accel_bias=tuple(float(v) for v in self.get_parameter("imu_accel_bias").value),
        )

        # ---- Publishers -----------------------------------------------------
        # IMU @ 100 Hz wants best-effort to avoid backpressure if the UI lags.
        be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self._imu_pub  = self.create_publisher(Imu,    "/imu/data_raw", be)
        self._diag_pub = self.create_publisher(String, "/esp32/diagnostics", 10)
        self._btn_save_pub      = self.create_publisher(Empty, "/buttons/save", 10)
        self._btn_startstop_pub = self.create_publisher(Empty, "/buttons/startstop", 10)
        self._btn_shutdown_pub  = self.create_publisher(Empty, "/buttons/shutdown_request", 10)
        self._btn_longpress_pub = self.create_publisher(Empty, "/buttons/shutdown_longpress", 10)

        # ---- Counters / state for diagnostics ------------------------------
        self._lock = threading.Lock()
        self._counts = {"imu": 0, "button": 0, "heartbeat": 0,
                        "status": 0, "lidar_frame": 0, "lidar_ack": 0,
                        "lidar_bytes": 0, "crc_fail": 0, "bad_len": 0}
        self._last_status: Optional[StatusFrame] = None
        self._last_heartbeat_ms: Optional[int] = None
        self._last_frame_wall: Optional[float] = None
        self._port_open: bool = False
        self._last_button_events: list[dict] = []  # ring of recent for diag

        # ---- LIDAR motor state ---------------------------------------------
        # Source of truth — set by the /lidar_enable service. The bridge
        # re-sends LIDAR_EN=1 on a timer while True; sends LIDAR_EN=0 once
        # when set to False or on node shutdown.
        self._lidar_desired: bool = False
        # Last state the ESP32 has acknowledged (mirrors hardware reality).
        self._lidar_acked:   Optional[bool] = None
        # Shared serial handle for both the reader thread (read) and the
        # service/timer (write). pyserial's Serial is thread-safe for
        # interleaved read/write on different threads.
        self._serial = None  # set inside reader thread once port opens

        # ---- LIDAR pty (LD14P data path) -----------------------------------
        # The ESP32 relays LD14P UART bytes inside LIDAR_FRAME envelopes.
        # We unwrap them onto a pseudo-tty that the ldlidar_stl_ros2 driver
        # opens as if it were a real /dev/ttyAMA0. Slave fd is closed
        # immediately; holding the master keeps the pty alive and the slave
        # path openable. O_NONBLOCK on the master means a slow reader drops
        # bytes instead of stalling the whole bridge.
        self._pty_master_fd: Optional[int] = None
        self._pty_slave_path: Optional[str] = None
        self._pty_overflow_count: int = 0
        self._setup_lidar_pty()

        # ---- Serial reader thread -----------------------------------------
        self._stop = threading.Event()
        self._reader = threading.Thread(
            target=self._reader_loop, daemon=True, name="esp32_serial_reader")
        self._reader.start()

        # ---- /lidar_enable service ----------------------------------------
        self._lidar_srv = self.create_service(
            SetBool, "/lidar_enable", self._on_lidar_enable)

        # ---- Periodic timers ----------------------------------------------
        self._diag_timer = self.create_timer(self._diag_dt, self._emit_diagnostics)
        self._lidar_refresh_timer = self.create_timer(
            self._lidar_refresh_dt, self._refresh_lidar)

        self.get_logger().info(
            f"esp32_uart_bridge started — port={self._port} baud={self._baud} "
            f"frame_id={self._frame} diag_period={self._diag_dt}s "
            f"lidar_refresh={self._lidar_refresh_dt}s")

    # =========================================================================
    # Serial reader thread
    # =========================================================================
    def _reader_loop(self) -> None:
        """Open the port + parse frames in a real OS thread."""
        # pyserial is imported lazily so a missing module fails with a clear
        # message at runtime rather than at colcon build / import time.
        try:
            import serial  # type: ignore
        except ImportError:
            self.get_logger().error(
                "pyserial not installed — run `sudo apt install python3-serial` "
                "or `pip install pyserial` in your env. Bridge will idle.")
            return

        parser = FrameParser()
        backoff = 1.0
        while not self._stop.is_set():
            try:
                ser = serial.Serial(self._port, self._baud, timeout=0.2)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warn(
                    f"Cannot open {self._port}: {exc} — retrying in {backoff:.1f}s")
                self._port_open = False
                time.sleep(backoff)
                backoff = min(backoff * 2, 10.0)
                continue

            self._port_open = True
            self._serial = ser  # publish handle for the writer side
            backoff = 1.0
            self.get_logger().info(f"Opened {self._port} @ {self._baud} baud")

            # Catch ANY exception in the read+parse hot path so a bug in
            # frame dispatch (or rclpy publish from this thread) doesn't
            # silently kill the reader and leave port_open=True forever.
            try:
                while not self._stop.is_set():
                    chunk = ser.read(128)  # blocks up to timeout for any bytes
                    if not chunk:
                        continue
                    try:
                        for frame in parser.feed_bytes(chunk):
                            self._handle_frame(frame)
                    except Exception as inner:
                        # Don't tear down the reader for a single bad frame —
                        # log throttled and keep going.
                        self.get_logger().error(
                            f"Frame handler raised: {inner!r} — skipping frame",
                            throttle_duration_sec=1.0)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warn(f"Serial read error: {exc} — reopening")
                self._port_open = False
            except Exception as exc:
                # Truly unexpected — log loudly, drop port_open, retry the
                # outer connection loop.
                self.get_logger().error(
                    f"Reader thread crashed: {exc!r} — reopening port",
                    exc_info=True)
                self._port_open = False
            finally:
                self._serial = None
                try:
                    ser.close()
                except Exception:
                    pass

    # =========================================================================
    # Frame dispatch
    # =========================================================================
    def _handle_frame(self, frame) -> None:
        kind, payload = frame[0], frame[1]
        with self._lock:
            self._last_frame_wall = time.time()
            if kind == FrameType.IMU:
                self._counts["imu"] += 1
                self._publish_imu(payload)
            elif kind == FrameType.BUTTON:
                self._counts["button"] += 1
                self._publish_button(payload)
            elif kind == FrameType.HEARTBEAT:
                self._counts["heartbeat"] += 1
                self._last_heartbeat_ms = int(payload)
            elif kind == FrameType.STATUS:
                self._counts["status"] += 1
                self._last_status = payload
                self.get_logger().info(
                    f"ESP32 STATUS — flags=0x{payload.flags:02X} "
                    f"who_am_i={payload.who_am_i} accel_cfg={payload.accel_cfg} "
                    f"gyro_cfg={payload.gyro_cfg}")
            elif kind == FrameType.LIDAR_FRAME:
                self._counts["lidar_frame"] += 1
                self._counts["lidar_bytes"] += len(payload)
                self._write_lidar_bytes(payload)
            elif kind == FrameType.LIDAR_ACK:
                self._counts["lidar_ack"] += 1
                if self._lidar_acked != bool(payload):
                    self.get_logger().info(
                        f"LIDAR motor → {'ON' if payload else 'OFF'} (ESP32 ack)")
                self._lidar_acked = bool(payload)
            elif kind == "CRC_FAIL":
                self._counts["crc_fail"] += 1
                if not self._quiet:
                    self.get_logger().debug(f"CRC fail: {payload}")
            elif kind == "BAD_LEN":
                self._counts["bad_len"] += 1
                self.get_logger().debug(f"Bad length: {payload}")

    # =========================================================================
    # Publishers
    # =========================================================================
    def _publish_imu(self, sample) -> None:
        # Remove the MPU-6050's gyro + accel bias before publishing. During
        # the startup autocal window this just subtracts the static fallback;
        # once the still-window completes it subtracts the measured gyro bias.
        ax, ay, az, gx, gy, gz = self._calib.apply(
            sample.ax, sample.ay, sample.az,
            sample.gx, sample.gy, sample.gz)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        # Orientation left zero (-1, -1, -1 in covariance[0] would also be
        # valid — Madgwick in H3 will compute it). Covariances left at 0
        # which downstream EKFs read as "use default trust"; tune in H3.
        self._imu_pub.publish(msg)

    def _publish_button(self, evt) -> None:
        empty = Empty()
        # Record for diagnostics regardless of state.
        self._last_button_events.insert(0, {
            "id": evt.id, "name": evt.name,
            "state": evt.state, "state_name": evt.state_name,
            "wall": time.time(),
        })
        del self._last_button_events[8:]  # keep last 8 events for the UI

        if evt.state == ButtonState.PRESSED:
            if evt.id == ButtonId.SAVE:
                self._btn_save_pub.publish(empty)
            elif evt.id == ButtonId.STARTSTOP:
                self._btn_startstop_pub.publish(empty)
            elif evt.id == ButtonId.SHUTDOWN:
                self._btn_shutdown_pub.publish(empty)
        elif evt.state == ButtonState.LONGPRESS:
            if evt.id == ButtonId.SHUTDOWN:
                self._btn_longpress_pub.publish(empty)
        # RELEASED is recorded for diagnostics but no event topic.

    # =========================================================================
    # LIDAR pty (raw byte path for the LD14P driver)
    # =========================================================================
    def _setup_lidar_pty(self) -> None:
        """Create the master/slave pty pair and publish the slave path as a
        stable symlink. Failures are logged but non-fatal — the bridge still
        works for IMU/buttons/motor control without the data path."""
        try:
            master_fd, slave_fd = pty.openpty()
        except OSError as exc:
            self.get_logger().error(
                f"openpty() failed: {exc!r} — LIDAR data path disabled")
            return
        slave_path = os.ttyname(slave_fd)
        # Raw mode + non-blocking writes on the master. We close the slave
        # immediately; the kernel keeps the slave path openable as long as
        # we hold the master fd.
        try:
            tty.setraw(master_fd, termios.TCSANOW)
            fl = fcntl.fcntl(master_fd, fcntl.F_GETFL)
            fcntl.fcntl(master_fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        except OSError as exc:
            os.close(master_fd)
            os.close(slave_fd)
            self.get_logger().error(
                f"pty configuration failed: {exc!r} — LIDAR data path disabled")
            return
        os.close(slave_fd)

        # Refresh the symlink (it may be stale from a previous run that
        # crashed without cleanup).
        try:
            os.unlink(self._lidar_pty_link)
        except FileNotFoundError:
            pass
        except OSError as exc:
            # Could be a permission issue if /tmp/lidar_pty is owned by root
            # from a sudo-run instance. Log and continue with whatever's there.
            self.get_logger().warn(
                f"unlink({self._lidar_pty_link}) failed: {exc!r}")
        try:
            os.symlink(slave_path, self._lidar_pty_link)
        except OSError as exc:
            self.get_logger().error(
                f"symlink({self._lidar_pty_link} → {slave_path}) failed: "
                f"{exc!r}. LD14P driver will need to open {slave_path} directly.")

        self._pty_master_fd  = master_fd
        self._pty_slave_path = slave_path
        self.get_logger().info(
            f"LIDAR pty ready — slave={slave_path} link={self._lidar_pty_link}")

    def _write_lidar_bytes(self, data: bytes) -> None:
        """Forward raw LD14P bytes to the pty master. Drops the chunk on
        EAGAIN (no reader yet, or reader is too slow) rather than blocking
        the reader thread."""
        fd = self._pty_master_fd
        if fd is None:
            return
        try:
            os.write(fd, data)
        except BlockingIOError:
            self._pty_overflow_count += 1
        except OSError as exc:
            self.get_logger().warn(
                f"pty write failed: {exc!r}",
                throttle_duration_sec=2.0)

    # =========================================================================
    # LIDAR motor control
    # =========================================================================
    def _send_lidar_en(self, enable: bool) -> bool:
        """Write a LIDAR_EN frame on the open serial port. Returns False if the
        port isn't open right now (caller decides whether to retry)."""
        ser = self._serial
        if ser is None:
            return False
        try:
            ser.write(encode_lidar_en(enable))
            return True
        except Exception as exc:
            self.get_logger().warn(f"LIDAR_EN write failed: {exc!r}")
            return False

    def _on_lidar_enable(self, req, resp):
        """std_srvs/SetBool: set the desired LIDAR motor state."""
        self._lidar_desired = bool(req.data)
        ok = self._send_lidar_en(self._lidar_desired)
        # Send an immediate explicit OFF when transitioning to disabled so the
        # motor stops without waiting for the watchdog.
        resp.success = ok
        resp.message = (
            f"LIDAR_EN={'1' if self._lidar_desired else '0'} "
            f"{'sent' if ok else 'queued (port not yet open)'}")
        return resp

    def _refresh_lidar(self) -> None:
        """Re-send LIDAR_EN=1 every `lidar_refresh_s` while the motor should be
        on, keeping the ESP32 watchdog fed. When the desired state is OFF we do
        nothing here — the ESP32 already knows (the service handler sent OFF)
        and the watchdog will also drop it if anything went missing."""
        if self._lidar_desired:
            self._send_lidar_en(True)

    # =========================================================================
    # Diagnostics
    # =========================================================================
    def _emit_diagnostics(self) -> None:
        with self._lock:
            now = time.time()
            secs_since_frame = (now - self._last_frame_wall) if self._last_frame_wall else None
            diag: dict[str, Any] = {
                "port": self._port,
                "baud": self._baud,
                "port_open": self._port_open,
                "frame_counts": dict(self._counts),
                "seconds_since_last_frame": (
                    round(secs_since_frame, 2) if secs_since_frame is not None else None),
                "esp32_uptime_ms": self._last_heartbeat_ms,
                "recent_buttons": list(self._last_button_events),
            }
            diag["imu_calib"] = {
                "calibrated":  self._calib.calibrated,
                "collecting":  self._calib.collecting,
                "rejected":    self._calib.rejected,
                "gyro_bias":   [round(v, 5) for v in self._calib.gyro_bias],
                "accel_bias":  [round(v, 4) for v in self._calib.accel_bias],
            }
            diag["lidar"] = {
                "desired_on":    self._lidar_desired,
                "acked_on":      self._lidar_acked,
                "frames":        self._counts["lidar_frame"],
                "bytes":         self._counts["lidar_bytes"],
                "pty_slave":     self._pty_slave_path,
                "pty_link":      self._lidar_pty_link,
                "pty_overflows": self._pty_overflow_count,
            }
            if self._last_status is not None:
                s = self._last_status
                diag["status"] = {
                    "flags": s.flags,
                    "imu_ok": s.imu_ok,
                    "who_am_i": s.who_am_i,
                    "accel_cfg": s.accel_cfg,
                    "gyro_cfg":  s.gyro_cfg,
                    "afs_sel": (s.accel_cfg >> 3) & 0x03 if s.accel_cfg is not None else None,
                    "fs_sel":  (s.gyro_cfg  >> 3) & 0x03 if s.gyro_cfg  is not None else None,
                    "za_offset_before": s.za_offset_before,
                    "za_offset_after":  s.za_offset_after,
                }
            else:
                diag["status"] = None

        msg = String()
        msg.data = json.dumps(diag, separators=(",", ":"))
        self._diag_pub.publish(msg)

    # =========================================================================
    # Shutdown
    # =========================================================================
    def destroy_node(self) -> bool:
        # Best-effort: tell the ESP32 to drop the motor before we tear down.
        # The 3-second firmware watchdog catches us even if this fails.
        try:
            self._lidar_desired = False
            self._send_lidar_en(False)
        except Exception:
            pass
        # Clean up the pty + symlink so the next run starts fresh.
        try:
            os.unlink(self._lidar_pty_link)
        except FileNotFoundError:
            pass
        except OSError:
            pass
        if self._pty_master_fd is not None:
            try:
                os.close(self._pty_master_fd)
            except OSError:
                pass
            self._pty_master_fd = None
        self._stop.set()
        if self._reader.is_alive():
            self._reader.join(timeout=1.5)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Esp32UartBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
