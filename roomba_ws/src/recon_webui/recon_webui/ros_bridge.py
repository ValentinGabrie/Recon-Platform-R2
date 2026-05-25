"""ros_bridge — ROS2 subscriber bridge for the Recon-Platform-R2 web UI.

Subscribes to (handheld-scanner topic set):
    - /map                 (nav_msgs/OccupancyGrid)  → channels["map"]
    - /tf                  (tf2_msgs/TFMessage)       → tracks map→odom +
                                                         derives scanner pose
    - /scanner/pose         (geometry_msgs/PoseStamped) → channels["pose"]
    - /robot/events        (std_msgs/String)          → event log
    - /imu/data_raw        (sensor_msgs/Imu)          → channels["imu"]
    - /esp32/diagnostics   (std_msgs/String, JSON)    → channels["bridge_health"]
    - /buttons/{save,reset,shutdown_request,shutdown_longpress}
                           (std_msgs/Empty)           → event log entries

Falls back gracefully if rclpy is not available (pure demo mode).

Architecture:
    - rclpy.spin() runs in a REAL OS thread (not eventlet green thread)
    - Subscriber callbacks convert ROS2 messages to Python dicts
    - Dicts are passed into DataChannel.on_ros_message() (thread-safe)
    - The Flask emit_loop reads from channels and pushes to WebSocket
"""

import json
import logging
import math
import queue
import time
from typing import Any, Optional

logger = logging.getLogger(__name__)

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Empty, String
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from slam_toolbox.srv import Reset as SlamReset
    from std_srvs.srv import SetBool
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                           QoSHistoryPolicy)

    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False
    logger.info("rclpy not available — ROS2 bridge disabled, using mock data")


# slam_toolbox in Jazzy ships an empty-request "Pause" service whose .srv comment
# advertises it as a toggle, but the binary implementation is a one-way set-to-paused
# — three back-to-back CLI calls all return status=True and never resume mapping.
# The reliable, idempotent way to drive paused state is the runtime parameter
# `paused_new_measurements` (bool), set via /slam_toolbox/set_parameters.
SLAM_PARAM_SVC = "/slam_toolbox/set_parameters"
SLAM_PAUSE_PARAM = "paused_new_measurements"


class RosBridge:
    """Bridge between ROS2 topics and web UI DataChannels.

    Subscribes to:
        - /tf               (tf2_msgs/TFMessage)         → map→odom transform
        - /scanner/pose      (geometry_msgs/PoseStamped)  → channels["pose"]
        - /map              (nav_msgs/OccupancyGrid)     → channels["map"]
        - /robot/events     (std_msgs/String)            → event log

    Publishes:
        - /robot/mode       (std_msgs/String)            → mode change from web UI
    """

    def __init__(self, channels: dict, event_callback=None) -> None:
        """Initialise the bridge.

        Args:
            channels: Dict of DataChannel instances keyed by name.
            event_callback: Optional callable(event_dict) for robot events.
        """
        self._channels = channels
        self._event_callback = event_callback
        self._node: Optional[Any] = None
        self._thread = None
        self._current_mode = "IDLE"
        self._mode_pub = None
        # H4-prep: scan-pause control. False = SLAM is paused, no map
        # integration. True = mapping. Starts paused so the UI gates the
        # mapping explicitly — slam_toolbox would otherwise integrate from
        # the moment it's launched.
        self._scan_active: bool = False
        self._slam_pause_client = None
        # map→odom TF transform from slam_toolbox (dx, dy, dtheta)
        self._map_odom_tf: Optional[tuple] = None
        # odom→base_link TF transform (identity until H3's EKF lands, then
        # dynamic from robot_localization). Tracked separately so we can
        # compose map→base_link = (map→odom) ∘ (odom→base_link).
        self._odom_base_tf: tuple = (0.0, 0.0, 0.0)
        # Thread-safe queue for events that must be emitted on the
        # eventlet thread (socketio.emit is NOT safe from rclpy thread)
        self._event_queue: queue.Queue = queue.Queue(maxsize=64)

    @property
    def available(self) -> bool:
        """Return True if rclpy is available."""
        return HAS_RCLPY

    @property
    def running(self) -> bool:
        """Return True if the bridge spin thread is alive."""
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> bool:
        """Start the ROS2 bridge in a background OS thread.

        Returns:
            True if the bridge started successfully, False otherwise.
        """
        if not HAS_RCLPY:
            logger.info(
                "ROS2 bridge not starting — rclpy unavailable, "
                "all channels will use mock data"
            )
            return False

        # Use real OS threading, not eventlet green threads
        import eventlet.patcher
        real_threading = eventlet.patcher.original("threading")

        try:
            rclpy.init()
        except RuntimeError:
            pass

        self._node = rclpy.create_node("recon_webui_bridge")

        self._node.create_subscription(
            String, "/robot/mode", self._mode_callback, 10
        )
        self._node.create_subscription(
            TFMessage, "/tf", self._tf_callback, 10
        )
        self._node.create_subscription(
            PoseStamped, "/scanner/pose", self._pose_callback, 10
        )
        self._node.create_subscription(
            OccupancyGrid, "/map", self._map_callback, 10
        )
        self._node.create_subscription(
            String, "/robot/events", self._events_callback, 10
        )
        # H3: robot_localization publishes /odom. Subscribe so we have a
        # pose even when slam_toolbox isn't running (e.g. imu-test mode).
        # When slam_toolbox *is* running, the TF-composition path
        # (_tf_callback) takes precedence because it includes the
        # map→odom correction and updates more frequently.
        self._node.create_subscription(
            Odometry, "/odom", self._odom_callback, 10
        )

        # ---- ESP32 bridge topics (H2.1) -----------------------------------
        # IMU is best-effort to match the publisher's QoS — otherwise the
        # subscription silently won't connect.
        be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self._node.create_subscription(
            Imu, "/imu/data_raw", self._imu_callback, be
        )
        self._node.create_subscription(
            String, "/esp32/diagnostics", self._diag_callback, 10
        )
        for topic, label in (
            ("/buttons/save",                "SAVE button"),
            ("/buttons/reset",               "RESET button"),
            ("/buttons/shutdown_request",    "SHUTDOWN button (press)"),
            ("/buttons/shutdown_longpress",  "SHUTDOWN button (long-press)"),
        ):
            # The empty-msg lambda closes over `label`; default-arg trick to
            # avoid late-binding all four to the last label.
            self._node.create_subscription(
                Empty, topic,
                lambda _msg, label=label: self._button_callback(label),
                10,
            )

        self._mode_pub = self._node.create_publisher(
            String, "/robot/mode", 10
        )

        # Service client for pausing/resuming SLAM integration.
        # Created here even if the server doesn't exist yet — it just
        # won't be ready until slam_toolbox comes up. set_scanning()
        # gracefully skips when wait_for_service times out.
        self._slam_pause_client = self._node.create_client(SetParameters, SLAM_PARAM_SVC)

        # Service client for the ESP32 LIDAR motor enable line. Lives on the
        # esp32_uart_bridge node (H2.1) — same graceful-no-op semantics if
        # the server isn't up yet.
        self._lidar_enable_client = self._node.create_client(
            SetBool, "/lidar_enable")

        # /slam_toolbox/reset — empties the SLAM pose graph + occupancy grid
        # without restarting the node. Request has `pause_new_measurements`
        # (set, not toggle); we always pass `false` so the caller's scan
        # state (Start/Pause) is preserved across the reset.
        self._slam_reset_client = self._node.create_client(
            SlamReset, "/slam_toolbox/reset")

        # NOTE: auto-pause used to live here as a 5-second one-shot rclpy
        # timer that called the sync service helpers below. That worked when
        # only SLAM was being paused, but adding the second service call
        # (/lidar_enable) made the timer hold the rclpy client's internal
        # lock long enough that the eventlet hub running on the main thread
        # would race against it and crash with
        #   greenlet.error: Cannot switch to a different thread
        # The sync helpers `_call_slam_pause` / `_call_lidar_enable` are
        # safe ONLY from an eventlet greenlet (Flask routes, the emit_loop)
        # — they poll a future with `time.sleep` which is greened and yields
        # to the hub. From a real OS thread (rclpy spin) they break the
        # invariant. So the auto-pause is now scheduled from app.py via
        # eventlet.spawn_after; this node just exposes `auto_pause()` to
        # be invoked from the greenlet side.
        self._auto_pause_timer = None

        logger.info("ROS2 bridge node created — starting spin thread")

        self._thread = real_threading.Thread(
            target=self._spin_loop, daemon=True, name="ros2_bridge_spin"
        )
        self._thread.start()
        return True

    def _spin_loop(self) -> None:
        """Run rclpy.spin in a dedicated thread."""
        try:
            rclpy.spin(self._node)
        except Exception as e:
            logger.error(f"ROS2 spin error: {e}")

    # =========================================================================
    # Subscriber callbacks — these run in the rclpy spin thread
    # =========================================================================

    def _mode_callback(self, msg: Any) -> None:
        """Track current robot mode from /robot/mode."""
        old = self._current_mode
        self._current_mode = msg.data
        if old != msg.data:
            try:
                self._event_queue.put_nowait({
                    "type": "MODE_CHANGE",
                    "message": f"Mode changed to {msg.data}",
                })
            except queue.Full:
                pass

    def _tf_callback(self, msg: Any) -> None:
        """Compose map→odom and odom→base_link into the scanner's pose.

        Pre-H3: slam_toolbox publishes map→odom and a static publisher
        gives the identity for odom→base_link. The scanner sits at the
        origin of base_link, so map→odom *is* the device pose.

        H3+: robot_localization's ekf_node publishes a *non-identity*
        odom→base_link (gyro-integrated yaw). We have to compose both to
        get the actual pose in map: map → base_link = (map→odom) ∘ (odom→base_link).
        """
        new_pose = False
        for tf in msg.transforms:
            t = tf.transform
            q = t.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            dtheta = math.atan2(siny, cosy)
            triple = (float(t.translation.x),
                      float(t.translation.y),
                      dtheta)
            if tf.header.frame_id == "map" and tf.child_frame_id == "odom":
                self._map_odom_tf = triple
                new_pose = True
            elif tf.header.frame_id == "odom" and tf.child_frame_id == "base_link":
                self._odom_base_tf = triple
                new_pose = True

        if new_pose and "pose" in self._channels and self._map_odom_tf is not None:
            mx, my, mth = self._map_odom_tf
            ox, oy, oth = self._odom_base_tf
            # 2-D pose composition: rotate odom-frame offset by map→odom yaw,
            # then translate by map→odom origin.
            c, s = math.cos(mth), math.sin(mth)
            x = mx + c * ox - s * oy
            y = my + s * ox + c * oy
            theta = mth + oth
            # Wrap theta into (-π, π].
            theta = math.atan2(math.sin(theta), math.cos(theta))
            self._channels["pose"].on_ros_message({
                "x": x, "y": y, "theta": theta,
            })

    def _pose_callback(self, msg: Any) -> None:
        """Convert PoseStamped (/scanner/pose, odom frame) → map frame → channel.

        Applies the map→odom TF transform when available (from slam_toolbox)
        so the device position is correctly placed on the SLAM map.
        """
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        x_odom = float(msg.pose.position.x)
        y_odom = float(msg.pose.position.y)

        if self._map_odom_tf is not None:
            dx, dy, dtheta = self._map_odom_tf
            cos_dt = math.cos(dtheta)
            sin_dt = math.sin(dtheta)
            x = cos_dt * x_odom - sin_dt * y_odom + dx
            y = sin_dt * x_odom + cos_dt * y_odom + dy
            theta = theta + dtheta
        else:
            x = x_odom
            y = y_odom

        pose = {"x": x, "y": y, "theta": theta}

        if "pose" in self._channels:
            self._channels["pose"].on_ros_message(pose)

    def _map_callback(self, msg: Any) -> None:
        """Convert nav_msgs/OccupancyGrid → compact dict → channel."""
        grid = {
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "data": list(msg.data),
        }

        if "map" in self._channels:
            self._channels["map"].on_ros_message(grid)

    def _events_callback(self, msg: Any) -> None:
        """Queue robot events for the eventlet thread to emit.

        CRITICAL: Do NOT call socketio.emit from this thread!
        The rclpy spin thread is a real OS thread; socketio.emit
        can only safely be called from the eventlet event loop.
        """
        try:
            self._event_queue.put_nowait({
                "type": "ROS_EVENT",
                "message": msg.data,
            })
        except queue.Full:
            pass

    def _odom_callback(self, msg: Any) -> None:
        """nav_msgs/Odometry → channels["pose"] in the odom frame.

        Used standalone in imu-test mode (no SLAM, no map→odom).
        In sensor-test mode the _tf_callback overwrites this with the
        map-frame composition, which is what we actually want for the UI.
        """
        # Only fill in if the TF-composed pose isn't already publishing.
        # When map→odom is known we trust that path; this is the fallback.
        if self._map_odom_tf is not None:
            return
        p = msg.pose.pose
        q = p.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny, cosy)
        if "pose" in self._channels:
            self._channels["pose"].on_ros_message({
                "x": float(p.position.x),
                "y": float(p.position.y),
                "theta": theta,
            })

    def _imu_callback(self, msg: Any) -> None:
        """sensor_msgs/Imu → channels["imu"] as a compact dict.

        Stamp is the *publisher's* stamp so the UI can compute end-to-end
        latency if it wants. Orientation is left out — Madgwick (H3) is what
        fills that in; until then the publisher sends only accel + gyro.
        """
        sample = {
            "stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000
                        + int(msg.header.stamp.nanosec),
            "wall_s":   time.time(),
            "ax": float(msg.linear_acceleration.x),
            "ay": float(msg.linear_acceleration.y),
            "az": float(msg.linear_acceleration.z),
            "gx": float(msg.angular_velocity.x),
            "gy": float(msg.angular_velocity.y),
            "gz": float(msg.angular_velocity.z),
        }
        if "imu" in self._channels:
            self._channels["imu"].on_ros_message(sample)

    def _diag_callback(self, msg: Any) -> None:
        """Parse JSON diagnostics from esp32_uart_bridge → channels["bridge_health"]."""
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError) as exc:
            logger.debug(f"Bad esp32 diag JSON: {exc}")
            return
        if "bridge_health" in self._channels:
            self._channels["bridge_health"].on_ros_message(payload)

    def _button_callback(self, label: str) -> None:
        """Push a BUTTON event into the same event queue the UI drains."""
        try:
            self._event_queue.put_nowait({
                "type": "BUTTON",
                "message": f"{label} pressed",
            })
        except queue.Full:
            pass

    # =========================================================================
    # SLAM pause/resume (called from Flask thread)
    # =========================================================================

    def _call_slam_pause(self, paused: bool, timeout_s: float = 3.0) -> bool:
        """Drive slam_toolbox into the requested paused state.

        Sets the runtime parameter `paused_new_measurements` via
        /slam_toolbox/set_parameters. Idempotent — calling with paused=True
        twice leaves it paused; calling with paused=False resumes. Returns
        True on success, False if rclpy isn't available, the service isn't
        up, or the call times out. Safe to call before SLAM has come up.

        timeout_s defaults to 3.0 because the real round-trip on the Pi 5 is
        ~2.7 s once DDS discovery has settled; 1 s was too tight and made
        every call look like a timeout.
        """
        if not HAS_RCLPY or self._slam_pause_client is None:
            return False
        if not self._slam_pause_client.wait_for_service(timeout_sec=timeout_s):
            logger.debug(f"{SLAM_PARAM_SVC} not available — skipping")
            return False
        req = SetParameters.Request()
        param = Parameter()
        param.name = SLAM_PAUSE_PARAM
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_BOOL
        param.value.bool_value = bool(paused)
        req.parameters = [param]
        future = self._slam_pause_client.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout_s:
            time.sleep(0.02)
        if not future.done():
            logger.warning(f"{SLAM_PARAM_SVC} timed out (paused={paused})")
            return False
        results = future.result().results
        if not results or not results[0].successful:
            reason = results[0].reason if results else "no result"
            logger.warning(f"{SLAM_PAUSE_PARAM} set failed: {reason}")
            return False
        return True

    def _call_lidar_enable(self, enable: bool, timeout_s: float = 3.0) -> bool:
        """Drive the ESP32 LIDAR motor enable line via /lidar_enable.

        Returns True on success, False if the esp32_uart_bridge service
        isn't up, the call times out, or rclpy is unavailable. Safe to
        call before the bridge has come up — it just no-ops.
        """
        if not HAS_RCLPY or self._lidar_enable_client is None:
            return False
        if not self._lidar_enable_client.wait_for_service(timeout_sec=timeout_s):
            logger.debug("/lidar_enable not available — skipping")
            return False
        req = SetBool.Request()
        req.data = bool(enable)
        future = self._lidar_enable_client.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout_s:
            time.sleep(0.02)
        if not future.done():
            logger.warning(f"/lidar_enable timed out (enable={enable})")
            return False
        try:
            return bool(future.result().success)
        except Exception:
            return False

    def clear_map(self, timeout_s: float = 3.0) -> dict:
        """Reset slam_toolbox's pose graph + occupancy grid. The current
        scan-pause state is preserved (request.pause_new_measurements=False).

        Must be called from an eventlet greenlet (same constraint as the
        other sync helpers in this file — see `_call_slam_pause`).

        Returns a Flask-ready dict {'success', 'slam_responded', 'message'}.
        """
        if not HAS_RCLPY or self._slam_reset_client is None:
            return {"success": False, "slam_responded": False,
                    "message": "rclpy or slam_toolbox not available"}
        if not self._slam_reset_client.wait_for_service(timeout_sec=timeout_s):
            return {"success": False, "slam_responded": False,
                    "message": "/slam_toolbox/reset not available — is SLAM running?"}
        req = SlamReset.Request()
        req.pause_new_measurements = False
        future = self._slam_reset_client.call_async(req)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout_s:
            time.sleep(0.02)
        if not future.done():
            return {"success": False, "slam_responded": False,
                    "message": "/slam_toolbox/reset timed out"}
        try:
            result_code = int(future.result().result)
        except Exception as exc:
            return {"success": False, "slam_responded": True,
                    "message": f"reset call failed: {exc!r}"}
        ok = (result_code == SlamReset.Response.RESULT_SUCCESS)
        try:
            self._event_queue.put_nowait({
                "type": "MAP_CLEAR",
                "message": "Map cleared" if ok else f"Map clear failed (code={result_code})",
            })
        except queue.Full:
            pass
        return {"success": ok, "slam_responded": True,
                "message": f"reset result_code={result_code}"}

    def auto_pause(self) -> None:
        """Pause SLAM + cut the LIDAR motor at startup so the UI gates the
        scan explicitly. Even if `/lidar_enable` hasn't come up yet, the
        ESP32's own boot-default already keeps the motor off, so a failure
        here is purely cosmetic. **Must be called from an eventlet greenlet,
        not from the rclpy spin thread** — the sync helpers below poll a
        future with `time.sleep`, which is eventlet-greened and crashes
        from real OS threads. app.py schedules this via
        `eventlet.spawn_after(5.0, ros_bridge.auto_pause)`.
        """
        slam_ok  = self._call_slam_pause(True, timeout_s=3.0)
        lidar_ok = self._call_lidar_enable(False, timeout_s=3.0)
        if slam_ok or lidar_ok:
            logger.info(
                f"Auto-paused at startup — slam={'ok' if slam_ok else 'n/a'} "
                f"lidar_motor={'off' if lidar_ok else 'n/a'} "
                f"— press 'Start scan' to begin")

    def set_scanning(self, active: bool) -> dict:
        """Start (active=True) or pause (active=False) SLAM integration.

        Returns a dict {'active', 'slam_responded', 'lidar_responded', 'mode'}
        suitable for returning straight from a Flask route.

        Ordering matters across the two service calls:
          * STARTING (active=True):  enable LIDAR motor FIRST, then unpause
            SLAM. This way slam_toolbox sees fresh scans as soon as it
            resumes integration. With the reverse order, the first second
            after resume would integrate partial/missing /scan packets.
          * STOPPING (active=False): pause SLAM FIRST, then cut LIDAR motor.
            This way slam_toolbox stops integrating BEFORE the motor spins
            down and starts emitting partial frames.
        """
        if active:
            lidar_responded = self._call_lidar_enable(True)
            responded       = self._call_slam_pause(not active)
        else:
            responded       = self._call_slam_pause(not active)
            lidar_responded = self._call_lidar_enable(False)
        self._scan_active = bool(active)
        new_mode = "SCAN" if active else "IDLE"
        self.publish_mode(new_mode)
        # Surface the transition as a robot_event so the dashboard log shows it.
        try:
            self._event_queue.put_nowait({
                "type": "MODE_CHANGE",
                "message": f"Scan {'started' if active else 'paused'}"
                            + ("" if responded else " (slam_toolbox not responding — UI state only)")
                            + ("" if lidar_responded else " (LIDAR motor svc not responding)"),
            })
        except queue.Full:
            pass
        return {"active": self._scan_active,
                "slam_responded":  responded,
                "lidar_responded": lidar_responded,
                "mode": new_mode}

    def is_scanning(self) -> bool:
        return self._scan_active

    # =========================================================================
    # Public API (called from Flask thread)
    # =========================================================================

    def get_mode(self) -> str:
        """Return the current scanner mode."""
        return self._current_mode

    def drain_events(self) -> list[dict]:
        """Drain all queued events (call from eventlet thread only)."""
        events = []
        while not self._event_queue.empty():
            try:
                events.append(self._event_queue.get_nowait())
            except queue.Empty:
                break
        return events

    def publish_mode(self, mode: str) -> None:
        """Publish a mode change to /robot/mode."""
        if not HAS_RCLPY or self._node is None or self._mode_pub is None:
            logger.debug("Cannot publish mode — ROS2 bridge not running")
            return

        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)
        self._current_mode = mode
        logger.info(f"Published mode change: {mode}")

    def shutdown(self) -> None:
        """Shutdown the ROS2 bridge cleanly."""
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if HAS_RCLPY:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        logger.info("ROS2 bridge shut down")
