"""ros_bridge — ROS2 subscriber bridge for the Recon-Platform-R2 web UI.

Subscribes to a small set of topics relevant to the handheld scanner:
    - /map           (nav_msgs/OccupancyGrid)     → channels["map"]
    - /tf            (tf2_msgs/TFMessage)          → tracks map→odom
    - /scanner/pose   (geometry_msgs/PoseStamped)   → channels["pose"]
    - /robot/events  (std_msgs/String)             → event log

Falls back gracefully if rclpy is not available (pure demo mode).

Architecture:
    - rclpy.spin() runs in a REAL OS thread (not eventlet green thread)
    - Subscriber callbacks convert ROS2 messages to Python dicts
    - Dicts are passed into DataChannel.on_ros_message() (thread-safe)
    - The Flask emit_loop reads from channels and pushes to WebSocket
"""

import logging
import math
import queue
from typing import Any, Optional

logger = logging.getLogger(__name__)

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from nav_msgs.msg import OccupancyGrid
    from tf2_msgs.msg import TFMessage

    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False
    logger.info("rclpy not available — ROS2 bridge disabled, using mock data")


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
        # map→odom TF transform from slam_toolbox (dx, dy, dtheta)
        self._map_odom_tf: Optional[tuple] = None
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

        self._mode_pub = self._node.create_publisher(
            String, "/robot/mode", 10
        )

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
        """Track the map→odom transform from slam_toolbox.

        With the current sensor-test launch (static identity odom→base_link),
        this transform also represents the scanner's pose in the map frame —
        so we push it straight into channels["pose"]. Once H3 lands the
        EKF, /scanner/pose will be published explicitly and override this
        path via _pose_callback.
        """
        for tf in msg.transforms:
            if tf.header.frame_id == "map" and tf.child_frame_id == "odom":
                t = tf.transform
                q = t.rotation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                dtheta = math.atan2(siny, cosy)
                self._map_odom_tf = (
                    float(t.translation.x),
                    float(t.translation.y),
                    dtheta,
                )
                if "pose" in self._channels:
                    self._channels["pose"].on_ros_message({
                        "x": self._map_odom_tf[0],
                        "y": self._map_odom_tf[1],
                        "theta": self._map_odom_tf[2],
                    })
                break

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
