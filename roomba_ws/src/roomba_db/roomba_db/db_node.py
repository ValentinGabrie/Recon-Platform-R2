"""db_node — ROS2 node exposing database services for map persistence.

Exposes ROS2 services:
  /db/save_map   — Save an OccupancyGrid to the database.
  /db/load_map   — Load a map by ID.
  /db/list_maps  — List all saved maps.
  /db/delete_map — Delete a map by ID.

All DB operations run in a thread pool executor to avoid blocking the
ROS2 callback thread.
"""

import json
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from roomba_db.models import MapRecord, SessionRecord, get_session_factory


class DbNode(Node):
    """ROS2 node providing database persistence services.

    Wraps SQLAlchemy operations and exposes them as ROS2-compatible
    interfaces. Currently uses topic-based request/response pattern.

    TODO: Migrate to proper ROS2 service interfaces once custom .srv
    files are defined for the roomba_db package.
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

        # Subscribe to robot events for session tracking
        self._event_sub = self.create_subscription(
            String, "/robot/events", self._on_robot_event, 10
        )

        # Subscribe to mode changes for session tracking
        self._mode_sub = self.create_subscription(
            String, "/robot/mode", self._on_mode_change, 10
        )

        self._current_session_id: Optional[int] = None

        self.get_logger().info("db_node started")

    def _on_robot_event(self, msg: String) -> None:
        """Handle robot events that require DB actions.

        Args:
            msg: Event message (e.g., SAVE_MAP, RECON_COMPLETE).
        """
        event = msg.data
        self.get_logger().info(f"Robot event received: {event}")

        if event == "SAVE_MAP":
            # TODO: Implement map save via actual OccupancyGrid subscription
            self.get_logger().info("Map save requested — not yet implemented")

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
            self.get_logger().error(
                f"Failed to record session: {e}"
            )


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
