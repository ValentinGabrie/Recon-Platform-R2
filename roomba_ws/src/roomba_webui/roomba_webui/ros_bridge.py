"""ros_bridge — ROS2 subscriber bridge for the web UI.

Subscribes to all relevant ROS2 topics and feeds data into DataChannels.
Falls back gracefully if rclpy is not available (pure demo mode).

Architecture:
  - rclpy.spin() runs in a REAL OS thread (not eventlet green thread)
  - Subscriber callbacks convert ROS2 messages to Python dicts
  - Dicts are passed into DataChannel.on_ros_message() (thread-safe)
  - The Flask emit_loop reads from channels and pushes to WebSocket
"""

import logging
import math
import os
import queue
from typing import Any, Optional

import yaml

logger = logging.getLogger(__name__)

# Attempt rclpy import — graceful fallback if unavailable
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import String
    from nav_msgs.msg import OccupancyGrid

    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False
    logger.info("rclpy not available — ROS2 bridge disabled, using mock data")


def _load_controller_config() -> dict[str, Any]:
    """Load controller.yaml for axis/button name mapping.

    Returns:
        Parsed YAML config dict, or empty dict on failure.
    """
    config_paths = [
        os.path.join(
            os.path.dirname(__file__),
            "..", "..", "..", "config", "controller.yaml",
        ),
        os.path.join(
            os.path.dirname(__file__),
            "..", "..", "config", "controller.yaml",
        ),
        "/home/gabi/roomba/roomba_ws/config/controller.yaml",
    ]
    for path in config_paths:
        real = os.path.realpath(path)
        if os.path.exists(real):
            with open(real, "r") as f:
                return yaml.safe_load(f) or {}
    return {}


class RosBridge:
    """Bridge between ROS2 topics and web UI DataChannels.

    Subscribes to:
      - /joy              (sensor_msgs/Joy)       → channels["controller"]
      - /robot/mode       (std_msgs/String)        → internal mode tracking
      - /roomba/pose      (geometry_msgs/PoseStamped) → channels["pose"]
      - /map              (nav_msgs/OccupancyGrid) → channels["map"]
      - /robot/events     (std_msgs/String)        → event log (via callback)

    Publishes:
      - /robot/mode       (std_msgs/String)        → mode change from web UI
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
        # Thread-safe queue for events that must be emitted on the
        # eventlet thread (socketio.emit is NOT safe from rclpy thread)
        self._event_queue: queue.Queue = queue.Queue(maxsize=64)

        # Load controller axis/button mapping from YAML
        config = _load_controller_config()
        self._axis_map: dict[str, int] = config.get("axes", {
            "left_x": 0, "left_y": 1, "right_x": 2, "right_y": 3,
            "left_trigger": 4, "right_trigger": 5,
        })
        self._button_map: dict[str, int] = config.get("buttons", {
            "a": 0, "b": 1, "x": 2, "y": 3,
            "lb": 4, "rb": 5, "start": 6, "select": 7,
            "left_stick": 8, "right_stick": 9,
            "dpad_up": 10, "dpad_down": 11,
            "dpad_left": 12, "dpad_right": 13,
        })

        # Inverted maps: index → name (for fast lookup in callbacks)
        self._axis_names: dict[int, str] = {
            v: k for k, v in self._axis_map.items()
        }
        self._button_names: dict[int, str] = {
            v: k for k, v in self._button_map.items()
        }

        # Per-axis inversion flags from config
        invert_cfg = config.get("invert", {})
        self._invert: dict[str, bool] = {
            "left_x": bool(invert_cfg.get("left_x", False)),
            "left_y": bool(invert_cfg.get("left_y", True)),
            "right_x": bool(invert_cfg.get("right_x", False)),
            "right_y": bool(invert_cfg.get("right_y", True)),
        }

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
            # Already initialised (e.g. in tests or re-init)
            pass

        self._node = rclpy.create_node("roomba_webui_bridge")

        # --- Subscribers ---
        self._node.create_subscription(
            Joy, "/joy", self._joy_callback, 10
        )
        self._node.create_subscription(
            String, "/robot/mode", self._mode_callback, 10
        )
        self._node.create_subscription(
            PoseStamped, "/roomba/pose", self._pose_callback, 10
        )
        self._node.create_subscription(
            OccupancyGrid, "/map", self._map_callback, 10
        )
        self._node.create_subscription(
            String, "/robot/events", self._events_callback, 10
        )

        # --- Publishers ---
        self._mode_pub = self._node.create_publisher(
            String, "/robot/mode", 10
        )

        logger.info("ROS2 bridge node created — starting spin thread")

        # Spin in a REAL OS thread
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

    def _joy_callback(self, msg: Any) -> None:
        """Convert sensor_msgs/Joy → controller state dict → channel.

        Applies axis corrections for joy_linux + xpadneo:
        - Y axes are inverted (up = negative in driver, we want up = positive)
        - Triggers range from 1.0 (released) to -1.0 (pressed); we normalise
          to 0.0 (released) → 1.0 (pressed).
        - D-pad is reported as axes (dpad_x, dpad_y), synthesise virtual
          buttons dpad_up/down/left/right for the web UI.
        """
        axes: dict[str, float] = {}
        for idx, name in self._axis_names.items():
            if idx < len(msg.axes):
                raw = float(msg.axes[idx])
                # Apply per-axis inversion from config
                if self._invert.get(name, False):
                    raw = -raw
                # Normalise triggers: driver 1.0→-1.0  ⇒  UI 0.0→1.0
                if name in ("left_trigger", "right_trigger"):
                    raw = (1.0 - raw) / 2.0
                axes[name] = raw
            else:
                axes[name] = 0.0

        buttons: dict[str, bool] = {}
        for idx, name in self._button_names.items():
            if idx < len(msg.buttons):
                buttons[name] = bool(msg.buttons[idx])
            else:
                buttons[name] = False

        # Synthesise D-pad buttons from axes for web UI button grid
        dpad_x = axes.get("dpad_x", 0.0)
        dpad_y = axes.get("dpad_y", 0.0)
        buttons["dpad_up"] = dpad_y > 0.5
        buttons["dpad_down"] = dpad_y < -0.5
        buttons["dpad_left"] = dpad_x < -0.5
        buttons["dpad_right"] = dpad_x > 0.5

        state = {
            "axes": axes,
            "buttons": buttons,
            "connected": True,
            "battery_pct": None,  # Not available from Joy message
        }

        if "controller" in self._channels:
            self._channels["controller"].on_ros_message(state)

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

    def _pose_callback(self, msg: Any) -> None:
        """Convert geometry_msgs/PoseStamped → {x, y, theta} → channel."""
        q = msg.pose.orientation
        # Quaternion → yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        pose = {
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "theta": float(theta),
        }

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
            pass  # Drop oldest-style — queue is bounded

    # =========================================================================
    # Public API (called from Flask thread)
    # =========================================================================

    def get_mode(self) -> str:
        """Return the current robot mode.

        Returns:
            Mode string (IDLE, MANUAL, RECON, etc.).
        """
        return self._current_mode

    def drain_events(self) -> list[dict]:
        """Drain all queued events (call from eventlet thread only).

        Returns:
            List of event dicts ready for socketio.emit.
        """
        events = []
        while not self._event_queue.empty():
            try:
                events.append(self._event_queue.get_nowait())
            except queue.Empty:
                break
        return events

    def publish_mode(self, mode: str) -> None:
        """Publish a mode change to /robot/mode.

        Args:
            mode: New mode string.
        """
        if not HAS_RCLPY or self._node is None or self._mode_pub is None:
            logger.debug(
                "Cannot publish mode — ROS2 bridge not running"
            )
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
