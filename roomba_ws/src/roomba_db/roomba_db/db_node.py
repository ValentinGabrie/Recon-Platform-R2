"""db_node — ROS2 node exposing database services for map persistence.

Subscribes to /robot/events for SAVE_MAP triggers (from controller X button
or web UI). Maintains a reference to the latest OccupancyGrid from /map so
saves are instantaneous.

All DB operations run in a thread pool executor to avoid blocking the
ROS2 callback thread.
"""

import json
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from roomba_db.models import MapRecord, MapEvent, SessionRecord, get_session_factory


class DbNode(Node):
    """ROS2 node providing database persistence services.

    Subscribes to:
      - /robot/events (String) — triggers SAVE_MAP, RECON_COMPLETE, etc.
      - /robot/mode   (String) — records session changes
      - /map          (OccupancyGrid) — caches latest map for saving
    """

    def __init__(self) -> None:
        super().__init__("db_node")

        # Initialise database
        try:
            self._session_factory = get_session_factory()
            self.get_logger().info("Database connected successfully")
        except Exception as e:
            raise RuntimeError(
                f"Failed to initialise database: {e}"
            ) from e

        # Thread pool for async DB operations
        self._executor = ThreadPoolExecutor(max_workers=2)

        # Subscribe to robot events for session tracking and map saves
        self._event_sub = self.create_subscription(
            String, "/robot/events", self._on_robot_event, 10
        )

        # Subscribe to mode changes for session tracking
        self._mode_sub = self.create_subscription(
            String, "/robot/mode", self._on_mode_change, 10
        )

        # Subscribe to /map to cache latest OccupancyGrid for saving
        self._map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._on_map, 10
        )

        self._current_session_id: Optional[int] = None
        self._latest_map: Optional[dict[str, Any]] = None
        self._map_save_counter = 0

        self.get_logger().info("db_node started — listening for SAVE_MAP events")

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Cache the latest OccupancyGrid for saving on demand."""
        self.get_logger().debug(
            f"Map received — width={msg.info.width} height={msg.info.height} resolution={msg.info.resolution:.3f}"
        )
        self._latest_map = {
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "data": list(msg.data),
        }

    def _on_robot_event(self, msg: String) -> None:
        """Handle robot events that require DB actions.

        Args:
            msg: Event message (e.g., SAVE_MAP, RECON_COMPLETE).
        """
        event = msg.data
        self.get_logger().info(f"Robot event received: {event}")

        if event == "SAVE_MAP":
            self.get_logger().info("SAVE_MAP event — saving map to database")
            self._executor.submit(self._save_map)

    def _save_map(self) -> None:
        """Save the latest cached map to the database with a date-based name.

        Called by the thread pool executor when a SAVE_MAP event is received
        from the controller (X button). Works headlessly — no web UI required.
        Also records a MapEvent so the web UI can detect the new save via polling.
        """
        if not self._latest_map:
            self.get_logger().warning(
                "SAVE_MAP requested but no map data received yet"
            )
            return

        self._map_save_counter += 1
        name = f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{self._map_save_counter}"
        self.get_logger().info(
            f"Saving map — name={name} width={self._latest_map['width']} height={self._latest_map['height']}"
        )

        try:
            session = self._session_factory()
            record = MapRecord(
                name=name,
                map_data=json.dumps(self._latest_map["data"]).encode("utf-8"),
                origin_x=self._latest_map.get("origin_x", 0.0),
                origin_y=self._latest_map.get("origin_y", 0.0),
                resolution=self._latest_map.get("resolution", 0.05),
                width=self._latest_map["width"],
                height=self._latest_map["height"],
            )
            session.add(record)
            session.flush()
            # Record a MapEvent so the web UI can detect this save
            event = MapEvent(
                event_type="SAVED",
                map_id=record.id,
                map_name=name,
            )
            session.add(event)
            session.commit()
            self.get_logger().info(
                f"Map saved — id={record.id} name={name} event_id={event.id}"
            )
            session.close()
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")

    def _on_mode_change(self, msg: String) -> None:
        """Track mode changes in the sessions table.

        Args:
            msg: New mode string (MANUAL, RECON, IDLE).
        """
        mode = msg.data
        self.get_logger().info(f"Mode change: {mode}")
        self._executor.submit(self._record_session, mode)

    def _record_session(self, mode: str) -> None:
        """Record a new session in the database.

        Args:
            mode: Operating mode for this session.
        """
        try:
            session = self._session_factory()
            record = SessionRecord(mode=mode)
            session.add(record)
            session.commit()
            self._current_session_id = record.id
            self.get_logger().debug(
                f"Session recorded: id={record.id}, mode={mode}"
            )
            session.close()
        except Exception as e:
            self.get_logger().error(f"Failed to record session: {e}")


def main(args=None) -> None:
    """Entry point for db_node."""
    rclpy.init(args=args)
    try:
        node = DbNode()
        rclpy.spin(node)
    except RuntimeError as e:
        rclpy.logging.get_logger("db_node").fatal(str(e))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
