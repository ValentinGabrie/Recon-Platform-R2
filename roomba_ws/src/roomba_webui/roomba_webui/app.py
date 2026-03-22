"""roomba_webui.app — Flask + SocketIO web server for the roomba project.

Runs as a ROS2 node, subscribes to all relevant topics passively, and
serves the web UI with automatic real/mock data fallback per channel.

TODO: Add authentication if exposed beyond LAN.
"""

import eventlet
eventlet.monkey_patch()

import os
from typing import Any

import yaml
from eventlet import tpool
from flask import Flask, jsonify, render_template, request
from flask_socketio import SocketIO

from roomba_webui.bluetooth_manager import BluetoothManager
from roomba_webui.data_channels import DataChannel
from roomba_webui.logging_config import get_logger, setup_logging
from roomba_webui.ros_bridge import RosBridge
from roomba_webui import mock_data
from roomba_db.models import MapRecord, MapEvent, get_session_factory

logger = get_logger(__name__)

# Flask app setup
app = Flask(__name__)
app.config["SECRET_KEY"] = "roomba-secret-key"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="eventlet")

# Bluetooth manager
bt_manager = BluetoothManager()

# Data channels — initialised in setup_channels()
channels: dict[str, DataChannel] = {}

# ROS2 bridge — initialised in main()
ros_bridge: RosBridge | None = None

# Database session factory — initialised in main()
db_factory = None


def load_webui_config() -> dict[str, Any]:
    """Load webui.yaml configuration file.

    Returns:
        Configuration dictionary.
    """
    config_paths = [
        os.path.join(os.path.dirname(__file__), "..", "..", "config", "webui.yaml"),
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "config", "webui.yaml"),
        "/home/gabi/roomba/roomba_ws/config/webui.yaml",
    ]
    for path in config_paths:
        if os.path.exists(path):
            with open(path, "r") as f:
                return yaml.safe_load(f) or {}
    return {}


def setup_channels(config: dict[str, Any]) -> None:
    """Initialise all data channels with mock fallbacks.

    Args:
        config: Loaded webui.yaml configuration.
    """
    timeouts = config.get("webui", {}).get("channel_timeouts", {})
    logger.info("Initialising data channels with timeouts: %s", timeouts)

    channels["sensors"] = DataChannel(
        topic="/sensors/front",
        timeout_s=timeouts.get("sensors", 2.0),
        mock_fn=mock_data.mock_sensor_distances,
    )
    channels["controller"] = DataChannel(
        topic="/joy",
        timeout_s=timeouts.get("controller", 3.0),
        mock_fn=mock_data.mock_controller_state,
    )
    channels["map"] = DataChannel(
        topic="/map",
        timeout_s=timeouts.get("map", 10.0),
        mock_fn=mock_data.mock_occupancy_grid,
    )
    channels["pose"] = DataChannel(
        topic="/odom",
        timeout_s=timeouts.get("pose", 2.0),
        mock_fn=mock_data.mock_robot_pose,
    )
    channels["bluetooth"] = DataChannel(
        topic="/bt_status",
        timeout_s=timeouts.get("bluetooth", 5.0),
        mock_fn=mock_data.mock_bluetooth_status,
    )


# =============================================================================
# HTTP Routes
# =============================================================================

@app.route("/")
def index():
    """Main dashboard page."""
    return render_template("index.html", active_page="dashboard")


@app.route("/map")
def map_page():
    """Live map viewer page."""
    return render_template("map.html", active_page="map")


@app.route("/controller")
def controller_page():
    """Controller input monitor page."""
    return render_template("controller.html", active_page="controller")


@app.route("/bluetooth")
def bluetooth_page():
    """Bluetooth device management page."""
    return render_template("bluetooth.html", active_page="bluetooth")


@app.route("/api/robot/status")
def api_robot_status():
    """Current robot status — mode, sensor readings."""
    sensor_data = channels["sensors"].get()
    mode = ros_bridge.get_mode() if ros_bridge else "IDLE"
    return jsonify({
        "mode": mode,
        "sensors": sensor_data,
        "sensor_live": channels["sensors"].is_live(),
    })


@app.route("/api/debug/channels")
def api_debug_channels():
    """Debug endpoint — shows DataChannel liveness and bridge status."""
    status = {}
    for name, ch in channels.items():
        status[name] = {
            "live": ch.is_live(),
            "last_seen_s": round(ch.last_seen_seconds(), 1),
            "topic": ch.topic,
        }
    bridge_info = {
        "available": ros_bridge.available if ros_bridge else False,
        "running": ros_bridge.running if ros_bridge else False,
    }
    return jsonify({"channels": status, "bridge": bridge_info})


@app.route("/api/debug/controller")
def api_debug_controller():
    """Debug endpoint — shows current controller channel data as JSON."""
    data = channels["controller"].get() if "controller" in channels else {}
    return jsonify(data)


@app.route("/api/robot/mode", methods=["POST"])
def api_set_mode():
    """Set robot operating mode via REST API."""
    data = request.get_json()
    mode = data.get("mode", "IDLE")
    if ros_bridge:
        ros_bridge.publish_mode(mode)
    return jsonify({"success": True, "mode": mode})


@app.route("/api/maps")
def api_list_maps():
    """List saved maps from database."""
    logger.debug("Listing all maps")

    def _query():
        session = db_factory()
        try:
            maps = session.query(MapRecord).order_by(MapRecord.created_at.desc()).all()
            return [
                {
                    "id": m.id,
                    "name": m.name,
                    "created_at": m.created_at.isoformat() if m.created_at else None,
                    "resolution": m.resolution,
                    "width": m.width,
                    "height": m.height,
                }
                for m in maps
            ]
        finally:
            session.close()

    result = tpool.execute(_query)
    logger.debug("Listed %d maps", len(result))
    return jsonify(result)


@app.route("/api/maps/<int:map_id>")
def api_get_map(map_id: int):
    """Get a single map's metadata."""
    logger.debug("Getting map metadata — id=%d", map_id)

    def _query():
        session = db_factory()
        try:
            m = session.query(MapRecord).get(map_id)
            if not m:
                return None
            return {
                "id": m.id,
                "name": m.name,
                "created_at": m.created_at.isoformat() if m.created_at else None,
                "updated_at": m.updated_at.isoformat() if m.updated_at else None,
                "origin_x": m.origin_x,
                "origin_y": m.origin_y,
                "resolution": m.resolution,
                "width": m.width,
                "height": m.height,
            }
        finally:
            session.close()

    result = tpool.execute(_query)
    if result is None:
        return jsonify({"success": False, "message": "Map not found"}), 404
    return jsonify(result)


@app.route("/api/maps/<int:map_id>/data")
def api_get_map_data(map_id: int):
    """Get map grid data."""
    logger.debug("Getting map grid data — id=%d", map_id)
    import json as _json

    def _query():
        session = db_factory()
        try:
            m = session.query(MapRecord).get(map_id)
            if not m:
                return None
            return {
                "width": m.width,
                "height": m.height,
                "resolution": m.resolution,
                "origin_x": m.origin_x,
                "origin_y": m.origin_y,
                "data": _json.loads(m.map_data) if m.map_data else [],
            }
        finally:
            session.close()

    result = tpool.execute(_query)
    if result is None:
        return jsonify({"success": False, "message": "Map not found"}), 404
    return jsonify(result)


@app.route("/api/maps", methods=["POST"])
def api_save_map():
    """Save current map to database.

    Accepts JSON body with optional 'name'. If no body, saves the current
    live map channel data with an auto-generated name.
    """
    import json as _json

    data = request.get_json(silent=True) or {}
    name = data.get("name", "").strip()

    # Get current map data from the live channel
    map_data = channels["map"].get() if "map" in channels else None
    if not map_data:
        return jsonify({"success": False, "message": "No map data available"}), 400

    if not name:
        from datetime import datetime
        name = f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    logger.info("Saving map — name=%s width=%s height=%s",
                name, map_data.get("width"), map_data.get("height"))

    def _save():
        session = db_factory()
        try:
            grid_data = map_data.get("data", [])
            record = MapRecord(
                name=name,
                map_data=_json.dumps(grid_data).encode("utf-8"),
                origin_x=map_data.get("origin_x", 0.0),
                origin_y=map_data.get("origin_y", 0.0),
                resolution=map_data.get("resolution", 0.05),
                width=map_data.get("width", 0),
                height=map_data.get("height", 0),
            )
            session.add(record)
            session.flush()
            event = MapEvent(
                event_type="SAVED",
                map_id=record.id,
                map_name=name,
            )
            session.add(event)
            session.commit()
            map_id = record.id
            session.close()
            return map_id
        except Exception:
            session.rollback()
            session.close()
            raise

    try:
        map_id = tpool.execute(_save)
        logger.info("Map saved — id=%d name=%s", map_id, name)
        return jsonify({"success": True, "id": map_id, "message": f"Map '{name}' saved"})
    except Exception as exc:
        logger.error("Map save failed: %s", exc)
        return jsonify({"success": False, "message": str(exc)}), 500


@app.route("/api/maps/<int:map_id>", methods=["PUT"])
def api_rename_map(map_id: int):
    """Rename a saved map."""
    data = request.get_json()
    new_name = data.get("name", "").strip() if data else ""
    if not new_name:
        return jsonify({"success": False, "message": "Name is required"}), 400

    logger.info("Renaming map — id=%d new_name=%s", map_id, new_name)

    def _rename():
        session = db_factory()
        try:
            m = session.query(MapRecord).get(map_id)
            if not m:
                session.close()
                return False
            m.name = new_name
            session.commit()
            session.close()
            return True
        except Exception:
            session.rollback()
            session.close()
            raise

    try:
        found = tpool.execute(_rename)
    except Exception as exc:
        logger.error("Map rename failed — id=%d: %s", map_id, exc)
        return jsonify({"success": False, "message": str(exc)}), 500
    if not found:
        return jsonify({"success": False, "message": "Map not found"}), 404
    logger.info("Map renamed — id=%d new_name=%s", map_id, new_name)
    return jsonify({"success": True, "message": f"Map renamed to '{new_name}'"})


@app.route("/api/maps/<int:map_id>", methods=["DELETE"])
def api_delete_map(map_id: int):
    """Delete a map."""
    logger.info("Deleting map — id=%d", map_id)

    def _delete():
        session = db_factory()
        try:
            m = session.query(MapRecord).get(map_id)
            if not m:
                session.close()
                return None
            deleted_name = m.name
            session.delete(m)
            event = MapEvent(
                event_type="DELETED",
                map_id=map_id,
                map_name=deleted_name,
            )
            session.add(event)
            session.commit()
            session.close()
            return deleted_name
        except Exception:
            session.rollback()
            session.close()
            raise

    try:
        deleted_name = tpool.execute(_delete)
    except Exception as exc:
        logger.error("Map delete failed — id=%d: %s", map_id, exc)
        return jsonify({"success": False, "message": str(exc)}), 500
    if deleted_name is None:
        return jsonify({"success": False, "message": "Map not found"}), 404
    logger.info("Map deleted — id=%d name=%s", map_id, deleted_name)
    return jsonify({"success": True, "message": "Map deleted"})


# =============================================================================
# Map Events API — polled by UI to detect headless saves from db_node
# =============================================================================

@app.route("/api/maps/events")
def api_map_events():
    """Return map events since a given event id.

    Query params:
        since: Return events with id > this value (default 0 = all).
    """
    since_id = request.args.get("since", 0, type=int)

    def _query():
        session = db_factory()
        try:
            events = (
                session.query(MapEvent)
                .filter(MapEvent.id > since_id)
                .order_by(MapEvent.id.asc())
                .all()
            )
            return [
                {
                    "id": e.id,
                    "event_type": e.event_type,
                    "map_id": e.map_id,
                    "map_name": e.map_name,
                    "created_at": e.created_at.isoformat() if e.created_at else None,
                }
                for e in events
            ]
        finally:
            session.close()

    result = tpool.execute(_query)
    return jsonify(result)


# Bluetooth API routes — run in tpool to avoid blocking eventlet loop
@app.route("/api/bluetooth/devices")
def api_bt_devices():
    """List known/paired Bluetooth devices."""
    devices = tpool.execute(bt_manager.list_devices)
    return jsonify(devices)


@app.route("/api/bluetooth/scan", methods=["POST"])
def api_bt_scan():
    """Start a 10-second BT scan (runs in thread pool)."""
    logger.info("BT scan requested")
    devices = tpool.execute(bt_manager.scan, 10)
    logger.info("BT scan complete — found %d devices", len(devices))
    return jsonify(devices)


@app.route("/api/bluetooth/pair", methods=["POST"])
def api_bt_pair():
    """Pair a device by MAC address."""
    data = request.get_json()
    mac = data.get("mac", "")
    logger.info("BT pair requested — mac=%s", mac)
    result = tpool.execute(bt_manager.pair, mac)
    logger.info("BT pair result — mac=%s success=%s msg=%s", mac, result["success"], result["message"])
    return jsonify({"success": result["success"], "mac": mac, "message": result["message"]})


@app.route("/api/bluetooth/connect", methods=["POST"])
def api_bt_connect():
    """Connect a paired device by MAC address."""
    data = request.get_json()
    mac = data.get("mac", "")
    logger.info("BT connect requested — mac=%s", mac)
    result = tpool.execute(bt_manager.connect, mac)
    logger.info("BT connect result — mac=%s success=%s msg=%s", mac, result["success"], result["message"])
    return jsonify({"success": result["success"], "mac": mac, "message": result["message"]})


@app.route("/api/bluetooth/disconnect", methods=["POST"])
def api_bt_disconnect():
    """Disconnect a device by MAC address."""
    data = request.get_json()
    mac = data.get("mac", "")
    logger.info("BT disconnect requested — mac=%s", mac)
    result = tpool.execute(bt_manager.disconnect, mac)
    logger.info("BT disconnect result — mac=%s success=%s", mac, result["success"])
    return jsonify({"success": result["success"], "mac": mac, "message": result["message"]})


@app.route("/api/bluetooth/trust", methods=["POST"])
def api_bt_trust():
    """Trust a device by MAC address."""
    data = request.get_json()
    mac = data.get("mac", "")
    result = tpool.execute(bt_manager.trust, mac)
    return jsonify({"success": result["success"], "mac": mac, "message": result["message"]})


@app.route("/api/bluetooth/remove", methods=["POST"])
def api_bt_remove():
    """Remove (forget) a device by MAC address."""
    data = request.get_json()
    mac = data.get("mac", "")
    logger.info("BT remove requested — mac=%s", mac)
    result = tpool.execute(bt_manager.remove, mac)
    logger.info("BT remove result — mac=%s success=%s", mac, result["success"])
    return jsonify({"success": result["success"], "mac": mac, "message": result["message"]})


@app.route("/api/bluetooth/setup", methods=["POST"])
def api_bt_setup():
    """One-shot pair + trust + connect for a new controller."""
    data = request.get_json()
    mac = data.get("mac", "")
    logger.info("BT setup_controller requested — mac=%s", mac)
    result = tpool.execute(bt_manager.setup_controller, mac)
    logger.info("BT setup_controller result — mac=%s success=%s msg=%s", mac, result["success"], result["message"])
    return jsonify({"success": result["success"], "mac": mac, "message": result["message"]})


# =============================================================================
# WebSocket Events
# =============================================================================

@socketio.on("connect")
def on_connect():
    """Handle new WebSocket connection."""
    logger.info("WebSocket client connected")


@socketio.on("set_mode")
def on_set_mode(data: dict):
    """Handle mode change request from WebSocket client.

    Args:
        data: Dict with 'mode' key.
    """
    mode = data.get("mode", "IDLE")
    logger.info("WebSocket mode change requested — mode=%s", mode)
    if ros_bridge:
        ros_bridge.publish_mode(mode)
    socketio.emit("robot_event", {
        "type": "MODE_CHANGE",
        "message": f"Mode changed to {mode}",
    })


def emit_loop() -> None:
    """Background thread that periodically emits data to WebSocket clients.

    Rates are configured in webui.yaml.
    """
    import time

    config = load_webui_config()
    rates = config.get("webui", {}).get("emit_rates", {})

    # Track last emit time per event
    last_emit: dict[str, float] = {}

    while True:
        try:
            now = time.time()

            def should_emit(name: str, rate_hz: float) -> bool:
                interval = 1.0 / rate_hz if rate_hz > 0 else 999.0
                last = last_emit.get(name, 0.0)
                if now - last >= interval:
                    last_emit[name] = now
                    return True
                return False

            # --- Drain ROS2 events (from rclpy thread via queue) ---
            if ros_bridge and ros_bridge.running:
                for event in ros_bridge.drain_events():
                    socketio.emit("robot_event", event)

            # Sensor data at 5 Hz
            if should_emit("sensor_data", rates.get("sensor_data", 5.0)):
                socketio.emit("sensor_data", channels["sensors"].get())

            # Sensor health at 1 Hz
            if should_emit("sensor_health", rates.get("sensor_health", 1.0)):
                socketio.emit("sensor_health", mock_data.mock_sensor_health())

            # Robot pose at 5 Hz
            if should_emit("robot_pose", rates.get("robot_pose", 5.0)):
                socketio.emit("robot_pose", channels["pose"].get())

            # Map update at 2 Hz
            if should_emit("map_update", rates.get("map_update", 2.0)):
                socketio.emit("map_update", channels["map"].get())

            # Controller state at 20 Hz
            if should_emit(
                "controller_state",
                rates.get("controller_state", 20.0)
            ):
                socketio.emit("controller_state", channels["controller"].get())

            # Bluetooth status at 2 Hz
            if should_emit(
                "bluetooth_status",
                rates.get("bluetooth_status", 2.0)
            ):
                socketio.emit("bluetooth_status", channels["bluetooth"].get())

            # Channel status at 0.5 Hz
            if should_emit(
                "channel_status",
                rates.get("channel_status", 0.5)
            ):
                status = {}
                for name, ch in channels.items():
                    status[name] = {
                        "live": ch.is_live(),
                        "last_seen_s": round(ch.last_seen_seconds(), 1),
                    }
                socketio.emit("channel_status", status)

        except Exception as exc:
            logger.error("emit_loop error: %s", exc, exc_info=True)

        # Yield to eventlet — must use eventlet.sleep, not time.sleep
        eventlet.sleep(0.01)


def bt_status_loop() -> None:
    """Background task that polls real Bluetooth controller status.

    Updates the bluetooth DataChannel with actual connection info
    from bluetoothctl, so the web UI shows real BT status when
    a controller is connected or paired.
    """
    while True:
        try:
            # list_devices now returns paired OR connected devices
            devices = bt_manager.list_devices()

            # Check status of each device, prefer connected ones
            best_status = None
            for dev in devices:
                status = bt_manager.get_connection_status(dev["mac"])
                if status.get("connected"):
                    best_status = status
                    break
                if best_status is None:
                    best_status = status

            # Always push status if we know about any device
            if best_status and "bluetooth" in channels:
                channels["bluetooth"].on_ros_message(best_status)
        except Exception as exc:
            logger.debug("bt_status_loop error (non-critical): %s", exc)

        eventlet.sleep(2.0)


def main(args=None) -> None:
    """Entry point — start Flask-SocketIO server."""
    global ros_bridge, db_factory

    # Activate project-wide logging (console INFO+ / file DEBUG+)
    log_path = setup_logging()
    logger.info("Logging initialised — log_dir=%s", log_path)

    config = load_webui_config()
    setup_channels(config)

    # Initialise database session factory
    try:
        db_factory = get_session_factory()
        logger.info("Database session factory initialised")
    except Exception as exc:
        logger.error("Database init failed — map CRUD disabled: %s", exc)

    webui_config = config.get("webui", {})
    host = webui_config.get("host", "0.0.0.0")
    port = int(os.environ.get("WEBUI_PORT", webui_config.get("port", 5000)))

    # Start ROS2 bridge (graceful fallback if rclpy unavailable)
    ros_bridge = RosBridge(channels)
    bridge_ok = ros_bridge.start()
    if bridge_ok:
        logger.info("ROS2 bridge started — real topic data enabled")
    else:
        logger.info("ROS2 bridge unavailable — using mock data only")

    # Start background emit task (uses eventlet green thread)
    socketio.start_background_task(emit_loop)
    logger.info("emit_loop background task started")

    # Start Bluetooth status polling task
    socketio.start_background_task(bt_status_loop)
    logger.info("bt_status_loop background task started")

    logger.info("Starting Flask-SocketIO server on %s:%d", host, port)
    # TODO: Add authentication if exposed beyond LAN
    socketio.run(app, host=host, port=port, debug=False)


if __name__ == "__main__":
    main()
