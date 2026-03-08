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
from roomba_webui.ros_bridge import RosBridge
from roomba_webui import mock_data

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
    """List saved maps from database.

    TODO: Call db_node service.
    """
    return jsonify([])


@app.route("/api/maps/<int:map_id>")
def api_get_map(map_id: int):
    """Get a single map's metadata.

    TODO: Call db_node service.
    """
    return jsonify({"error": "not implemented"}), 501


@app.route("/api/maps/<int:map_id>/data")
def api_get_map_data(map_id: int):
    """Get map grid data.

    TODO: Call db_node service.
    """
    return jsonify({"error": "not implemented"}), 501


@app.route("/api/maps/<int:map_id>", methods=["DELETE"])
def api_delete_map(map_id: int):
    """Delete a map.

    TODO: Call db_node service.
    """
    return jsonify({"error": "not implemented"}), 501


# Bluetooth API routes — run in tpool to avoid blocking eventlet loop
@app.route("/api/bluetooth/devices")
def api_bt_devices():
    """List known/paired Bluetooth devices."""
    devices = tpool.execute(bt_manager.list_devices)
    return jsonify(devices)


@app.route("/api/bluetooth/scan", methods=["POST"])
def api_bt_scan():
    """Start a 10-second BT scan (runs in thread pool)."""
    devices = tpool.execute(bt_manager.scan, 10)
    return jsonify(devices)


@app.route("/api/bluetooth/pair", methods=["POST"])
def api_bt_pair():
    """Pair a device by MAC address."""
    data = request.get_json()
    mac = data.get("mac", "")
    success = tpool.execute(bt_manager.pair, mac)
    return jsonify({"success": success, "mac": mac})


@app.route("/api/bluetooth/connect", methods=["POST"])
def api_bt_connect():
    """Connect a paired device by MAC address."""
    data = request.get_json()
    mac = data.get("mac", "")
    success = tpool.execute(bt_manager.connect, mac)
    return jsonify({"success": success, "mac": mac})


@app.route("/api/bluetooth/disconnect", methods=["POST"])
def api_bt_disconnect():
    """Disconnect a device by MAC address."""
    data = request.get_json()
    mac = data.get("mac", "")
    success = tpool.execute(bt_manager.disconnect, mac)
    return jsonify({"success": success, "mac": mac})


@app.route("/api/bluetooth/trust", methods=["POST"])
def api_bt_trust():
    """Trust a device by MAC address."""
    data = request.get_json()
    mac = data.get("mac", "")
    success = tpool.execute(bt_manager.trust, mac)
    return jsonify({"success": success, "mac": mac})


# =============================================================================
# WebSocket Events
# =============================================================================

@socketio.on("connect")
def on_connect():
    """Handle new WebSocket connection."""
    pass


@socketio.on("set_mode")
def on_set_mode(data: dict):
    """Handle mode change request from WebSocket client.

    Args:
        data: Dict with 'mode' key.
    """
    mode = data.get("mode", "IDLE")
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
            app.logger.error(f"emit_loop error: {exc}")

        # Yield to eventlet — must use eventlet.sleep, not time.sleep
        eventlet.sleep(0.01)


def bt_status_loop() -> None:
    """Background task that polls real Bluetooth controller status.

    Updates the bluetooth DataChannel with actual connection info
    from bluetoothctl, so the web UI shows real BT status when
    a controller is paired.
    """
    import time

    # Known Xbox controller MAC — will be set when a device is connected
    known_mac: str | None = None

    while True:
        try:
            # Check if we have a known controller
            if known_mac:
                status = bt_manager.get_connection_status(known_mac)
                if "bluetooth" in channels:
                    channels["bluetooth"].on_ros_message(status)
            else:
                # Try to find a connected device from paired list
                devices = bt_manager.list_devices()
                for dev in devices:
                    name_lower = dev.get("name", "").lower()
                    if any(
                        k in name_lower
                        for k in ["xbox", "controller", "gamepad"]
                    ):
                        known_mac = dev["mac"]
                        break
        except Exception:
            pass  # Non-critical — channel falls back to mock

        eventlet.sleep(3.0)  # Poll every 3 seconds


def main(args=None) -> None:
    """Entry point — start Flask-SocketIO server."""
    global ros_bridge

    config = load_webui_config()
    setup_channels(config)

    webui_config = config.get("webui", {})
    host = webui_config.get("host", "0.0.0.0")
    port = int(os.environ.get("WEBUI_PORT", webui_config.get("port", 5000)))

    # Start ROS2 bridge (graceful fallback if rclpy unavailable)
    ros_bridge = RosBridge(channels)
    bridge_ok = ros_bridge.start()
    if bridge_ok:
        app.logger.info("ROS2 bridge started — real topic data enabled")
    else:
        app.logger.info("ROS2 bridge unavailable — using mock data only")

    # Start background emit task (uses eventlet green thread)
    socketio.start_background_task(emit_loop)

    # Start Bluetooth status polling task
    socketio.start_background_task(bt_status_loop)

    # TODO: Add authentication if exposed beyond LAN
    socketio.run(app, host=host, port=port, debug=False)


if __name__ == "__main__":
    main()
