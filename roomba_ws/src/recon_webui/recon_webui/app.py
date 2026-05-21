"""recon_webui.app — Flask + SocketIO web server for Recon-Platform-R2.

Runs as a non-RT process, subscribes passively to /map, /tf, /scanner/pose,
and /robot/events via the ROS2 bridge, and serves the Dashboard + Map Viewer
pages with automatic real/mock data fallback per channel.

Bluetooth, controller monitor, and simulation control pages were removed in
the handheld pivot (Stage H1, 2026-05-09); only Dashboard and Map Viewer
remain.

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

from recon_webui.data_channels import DataChannel
from recon_webui.logging_config import get_logger, setup_logging
from recon_webui.ros_bridge import RosBridge
from recon_webui import mock_data
from recon_db.models import MapRecord, MapEvent, ProcessedMap, get_session_factory
from recon_db.postprocess import ALGORITHM_NAME, process_grid

logger = get_logger(__name__)

app = Flask(__name__)
app.config["SECRET_KEY"] = "recon-secret-key"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="eventlet")

channels: dict[str, DataChannel] = {}

ros_bridge: RosBridge | None = None

db_factory = None

# Ring buffer of recent IMU samples for the Stats page sparklines. Updated
# in the emit_loop; consumed by /api/stats (and pushed via WebSocket).
# Kept short (last 5 s @ 20 Hz emit ≈ 100 samples) — the UI shows a sliding
# window, the backend doesn't need long-term history.
IMU_HISTORY_MAX = 120
_imu_history: list[dict] = []



def load_webui_config() -> dict[str, Any]:
    """Load config/webui.yaml from the workspace.

    Searches a few locations relative to this file plus
    ``$RECON_WS/config/webui.yaml`` if the env var is set, which lets the
    config sit outside the installed package.
    """
    here = os.path.dirname(__file__)
    candidates = [
        os.path.join(here, "..", "..", "config", "webui.yaml"),
        os.path.join(here, "..", "..", "..", "config", "webui.yaml"),
    ]
    if (ws := os.environ.get("RECON_WS")):
        candidates.append(os.path.join(ws, "config", "webui.yaml"))
    for path in candidates:
        if os.path.exists(path):
            with open(path, "r") as f:
                return yaml.safe_load(f) or {}
    return {}


def setup_channels(config: dict[str, Any]) -> None:
    """Initialise data channels with mock fallbacks."""
    timeouts = config.get("webui", {}).get("channel_timeouts", {})
    logger.info(f"Initialising data channels with timeouts: {timeouts}")

    channels["map"] = DataChannel(
        topic="/map",
        timeout_s=timeouts.get("map", 10.0),
        mock_fn=mock_data.mock_occupancy_grid,
    )
    channels["pose"] = DataChannel(
        topic="/scanner/pose",
        timeout_s=timeouts.get("pose", 2.0),
        mock_fn=mock_data.mock_robot_pose,
    )
    channels["imu"] = DataChannel(
        topic="/imu/data_raw",
        timeout_s=timeouts.get("imu", 1.0),
        mock_fn=mock_data.mock_imu_sample,
    )
    channels["bridge_health"] = DataChannel(
        topic="/esp32/diagnostics",
        timeout_s=timeouts.get("bridge_health", 3.0),
        mock_fn=mock_data.mock_bridge_health,
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


@app.route("/stats")
def stats_page():
    """Telemetry page: ESP32 link health, IMU live values + sparklines, SLAM stats."""
    return render_template("stats.html", active_page="stats")


@app.route("/api/robot/status")
def api_robot_status():
    """Current scanner status — mode."""
    mode = ros_bridge.get_mode() if ros_bridge else "IDLE"
    scan_active = ros_bridge.is_scanning() if ros_bridge else False
    return jsonify({
        "mode": mode,
        "scan_active": scan_active,
    })


@app.route("/api/scan/state")
def api_scan_state():
    """Current scanning state — True when slam_toolbox is integrating scans."""
    active = ros_bridge.is_scanning() if ros_bridge else False
    return jsonify({"active": active})


@app.route("/api/scan/start", methods=["POST"])
def api_scan_start():
    if ros_bridge is None:
        return jsonify({"success": False, "message": "ROS bridge not running"}), 503
    result = ros_bridge.set_scanning(True)
    socketio.emit("scan_state", {"active": result["active"], "mode": result["mode"]})
    return jsonify({"success": True, **result})


@app.route("/api/scan/pause", methods=["POST"])
def api_scan_pause():
    if ros_bridge is None:
        return jsonify({"success": False, "message": "ROS bridge not running"}), 503
    result = ros_bridge.set_scanning(False)
    socketio.emit("scan_state", {"active": result["active"], "mode": result["mode"]})
    return jsonify({"success": True, **result})


def _build_stats_snapshot() -> dict[str, Any]:
    """Single source of truth for the /stats page and the WebSocket emit.

    Pulls live data from channels (which auto-fall back to mock when stale).
    Always returns a fully-formed dict — never None — so the client can
    render unconditionally.
    """
    imu_ch    = channels.get("imu")
    map_ch    = channels.get("map")
    pose_ch   = channels.get("pose")
    health_ch = channels.get("bridge_health")

    # SLAM cell counts: from real /map data if available, otherwise from
    # mock_slam_stats. Sum is O(N) — fine at 2 Hz, but skip if the grid is
    # huge to keep emit_loop responsive.
    if map_ch is not None:
        grid = map_ch.get() or mock_data.mock_occupancy_grid()
        cells = grid.get("data", [])
        if len(cells) <= 200 * 200:  # ≤ 40k cells, ~ms to count
            slam = {
                "width":      grid.get("width", 0),
                "height":     grid.get("height", 0),
                "resolution": grid.get("resolution", 0.0),
                "origin_x":   grid.get("origin_x", 0.0),
                "origin_y":   grid.get("origin_y", 0.0),
                "cell_counts": {
                    "free":    sum(1 for v in cells if v == 0),
                    "wall":    sum(1 for v in cells if v >= 50),
                    "unknown": sum(1 for v in cells if v == -1),
                },
                "live": map_ch.is_live(),
                "last_seen_s": round(map_ch.last_seen_seconds(), 2),
            }
        else:
            slam = mock_data.mock_slam_stats() | {"live": False, "last_seen_s": None}
    else:
        slam = mock_data.mock_slam_stats() | {"live": False, "last_seen_s": None}

    pose = (pose_ch.get() if pose_ch is not None else None) or mock_data.mock_robot_pose()
    pose_live = pose_ch.is_live() if pose_ch is not None else False

    return {
        "imu": {
            "live": imu_ch.is_live() if imu_ch is not None else False,
            "last_seen_s": round(imu_ch.last_seen_seconds(), 2) if imu_ch else None,
            "current": imu_ch.get() if imu_ch is not None else mock_data.mock_imu_sample(),
            "history": list(_imu_history),
        },
        "bridge_health": {
            "live": health_ch.is_live() if health_ch is not None else False,
            "last_seen_s": round(health_ch.last_seen_seconds(), 2) if health_ch else None,
            "data": health_ch.get() if health_ch is not None else mock_data.mock_bridge_health(),
        },
        "slam": slam,
        "pose": {**pose, "live": pose_live},
        "mode": ros_bridge.get_mode() if ros_bridge else "IDLE",
    }


@app.route("/api/stats")
def api_stats():
    """Combined Stats-page snapshot — ESP32 link health + IMU + SLAM + pose."""
    return jsonify(_build_stats_snapshot())


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


@app.route("/api/robot/mode", methods=["POST"])
def api_set_mode():
    """Set scanner operating mode via REST API."""
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
    logger.debug(f"Listed {len(result)} maps")
    return jsonify(result)


@app.route("/api/maps/<int:map_id>")
def api_get_map(map_id: int):
    """Get a single map's metadata."""
    logger.debug(f"Getting map metadata — id={map_id}")

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
    logger.debug(f"Getting map grid data — id={map_id}")
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

    map_data = channels["map"].get() if "map" in channels else None
    if not map_data:
        return jsonify({"success": False, "message": "No map data available"}), 400

    if not name:
        from datetime import datetime
        name = f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    logger.info(
        f"Saving map — name={name} width={map_data.get('width')} "
        f"height={map_data.get('height')}"
    )

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
        logger.info(f"Map saved — id={map_id} name={name}")
        return jsonify({"success": True, "id": map_id, "message": f"Map '{name}' saved"})
    except Exception as exc:
        logger.error(f"Map save failed: {exc}")
        return jsonify({"success": False, "message": str(exc)}), 500


@app.route("/api/maps/<int:map_id>", methods=["PUT"])
def api_rename_map(map_id: int):
    """Rename a saved map."""
    data = request.get_json()
    new_name = data.get("name", "").strip() if data else ""
    if not new_name:
        return jsonify({"success": False, "message": "Name is required"}), 400

    logger.info(f"Renaming map — id={map_id} new_name={new_name}")

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
        logger.error(f"Map rename failed — id={map_id}: {exc}")
        return jsonify({"success": False, "message": str(exc)}), 500
    if not found:
        return jsonify({"success": False, "message": "Map not found"}), 404
    logger.info(f"Map renamed — id={map_id} new_name={new_name}")
    return jsonify({"success": True, "message": f"Map renamed to '{new_name}'"})


@app.route("/api/maps/<int:map_id>", methods=["DELETE"])
def api_delete_map(map_id: int):
    """Delete a map."""
    logger.info(f"Deleting map — id={map_id}")

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
        logger.error(f"Map delete failed — id={map_id}: {exc}")
        return jsonify({"success": False, "message": str(exc)}), 500
    if deleted_name is None:
        return jsonify({"success": False, "message": "Map not found"}), 404
    logger.info(f"Map deleted — id={map_id} name={deleted_name}")
    return jsonify({"success": True, "message": "Map deleted"})


@app.route("/api/maps/<int:map_id>/process", methods=["POST"])
def api_process_map(map_id: int):
    """Run the Tier 2 post-processing pipeline on a saved map.

    Body (optional JSON): override default pipeline params, e.g.
        {"min_cluster_size": 12, "closing_iterations": 2}.

    Returns the new processed_map_id; the row is saved in `processed_maps`
    with a back-reference to the source map.
    """
    import json as _json
    overrides = request.get_json(silent=True) or {}

    def _do():
        session = db_factory()
        try:
            src = session.query(MapRecord).get(map_id)
            if not src:
                return None
            grid = _json.loads(src.map_data) if src.map_data else []
            width = src.width or 0
            height = src.height or 0
            if not grid or width == 0 or height == 0:
                return {"error": "Source map has no grid data"}
            # Heavy lifting — runs on this thread (already inside tpool).
            result = process_grid(grid, width, height, params=overrides)
            row = ProcessedMap(
                source_map_id=src.id,
                algorithm=ALGORITHM_NAME,
                parameters=_json.dumps(result.parameters,
                                       separators=(",", ":")),
                map_data=result.to_json_bytes(),
                n_clusters=result.n_clusters,
                n_noise_cells=result.n_noise_cells,
            )
            session.add(row)
            session.commit()
            return {
                "id": row.id,
                "n_clusters": row.n_clusters,
                "n_noise_cells": row.n_noise_cells,
                "algorithm": row.algorithm,
            }
        except Exception:
            session.rollback()
            raise
        finally:
            session.close()

    try:
        out = tpool.execute(_do)
    except Exception as exc:
        logger.error(f"Map process failed — id={map_id}: {exc}", exc_info=True)
        return jsonify({"success": False, "message": str(exc)}), 500
    if out is None:
        return jsonify({"success": False, "message": "Map not found"}), 404
    if "error" in out:
        return jsonify({"success": False, "message": out["error"]}), 400
    logger.info(f"Map processed — source={map_id} processed_id={out['id']} "
                f"n_clusters={out['n_clusters']}")
    return jsonify({"success": True, **out})


@app.route("/api/maps/<int:map_id>/processed")
def api_list_processed(map_id: int):
    """List processed-map rows for a given source map id."""
    def _q():
        session = db_factory()
        try:
            rows = (session.query(ProcessedMap)
                    .filter_by(source_map_id=map_id)
                    .order_by(ProcessedMap.created_at.desc())
                    .all())
            return [{
                "id": r.id,
                "algorithm": r.algorithm,
                "n_clusters": r.n_clusters,
                "n_noise_cells": r.n_noise_cells,
                "created_at": r.created_at.isoformat() if r.created_at else None,
            } for r in rows]
        finally:
            session.close()
    return jsonify(tpool.execute(_q))


@app.route("/api/processed/<int:processed_id>/data")
def api_get_processed_data(processed_id: int):
    """Get a processed map's full payload (cleaned grid + cluster_labels)."""
    import json as _json

    def _q():
        session = db_factory()
        try:
            row = session.query(ProcessedMap).get(processed_id)
            if not row:
                return None
            src = session.query(MapRecord).get(row.source_map_id)
            payload = _json.loads(row.map_data)
            # Attach geometry from the source map so the client doesn't need
            # a second round-trip to render the processed grid.
            if src is not None:
                payload["resolution"] = src.resolution
                payload["origin_x"]   = src.origin_x
                payload["origin_y"]   = src.origin_y
            return payload
        finally:
            session.close()

    result = tpool.execute(_q)
    if result is None:
        return jsonify({"success": False, "message": "Processed map not found"}), 404
    return jsonify(result)


@app.route("/api/maps/events")
def api_map_events():
    """Return map events since a given event id (for headless save polling).

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


# =============================================================================
# WebSocket Events
# =============================================================================

@socketio.on("connect")
def on_connect():
    """Handle new WebSocket connection — send current state immediately."""
    logger.info("WebSocket client connected")
    mode = ros_bridge.get_mode() if ros_bridge else "IDLE"
    active = ros_bridge.is_scanning() if ros_bridge else False
    socketio.emit("robot_mode", {"mode": mode})
    socketio.emit("scan_state", {"active": active, "mode": mode})
    if "map" in channels:
        socketio.emit("map_update", channels["map"].get())


@socketio.on("set_mode")
def on_set_mode(data: dict):
    """Handle mode change request from WebSocket client."""
    mode = data.get("mode", "IDLE")
    logger.info(f"WebSocket mode change requested — mode={mode}")
    if ros_bridge:
        ros_bridge.publish_mode(mode)
    socketio.emit("robot_event", {
        "type": "MODE_CHANGE",
        "message": f"Mode changed to {mode}",
    })


def emit_loop() -> None:
    """Background task that periodically emits data to WebSocket clients.

    Rates are configured in webui.yaml.
    """
    import time

    config = load_webui_config()
    rates = config.get("webui", {}).get("emit_rates", {})

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

            if ros_bridge and ros_bridge.running:
                for event in ros_bridge.drain_events():
                    socketio.emit("robot_event", event)

            if should_emit("robot_pose", rates.get("robot_pose", 5.0)):
                socketio.emit("robot_pose", channels["pose"].get())

            if should_emit("map_update", rates.get("map_update", 2.0)):
                socketio.emit("map_update", channels["map"].get())

            if should_emit("imu_data", rates.get("imu_data", 20.0)):
                imu_sample = channels["imu"].get() if "imu" in channels else None
                if imu_sample is not None:
                    _imu_history.append(imu_sample)
                    if len(_imu_history) > IMU_HISTORY_MAX:
                        # Drop from the front cheaply — list slicing is O(N)
                        # but N is bounded at ~120 so it's negligible.
                        del _imu_history[:len(_imu_history) - IMU_HISTORY_MAX]
                    socketio.emit("imu_data", imu_sample)

            if should_emit("bridge_health", rates.get("bridge_health", 1.0)):
                if "bridge_health" in channels:
                    socketio.emit("bridge_health", channels["bridge_health"].get())

            if should_emit("stats_update", rates.get("stats_update", 2.0)):
                # The Stats page primarily listens to the per-stream events
                # above; this consolidated snapshot is a fallback for clients
                # that just want one event to render the whole page.
                socketio.emit("stats_update", _build_stats_snapshot())

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

            if should_emit("robot_mode", 1.0):
                mode = ros_bridge.get_mode() if ros_bridge else "IDLE"
                socketio.emit("robot_mode", {"mode": mode})

        except Exception as exc:
            logger.error(f"emit_loop error: {exc}", exc_info=True)

        eventlet.sleep(0.01)


def main(args=None) -> None:
    """Entry point — start Flask-SocketIO server."""
    global ros_bridge, db_factory

    log_path = setup_logging()
    logger.info(f"Logging initialised — log_dir={log_path}")

    config = load_webui_config()
    setup_channels(config)

    try:
        db_factory = get_session_factory()
        logger.info("Database session factory initialised")
    except Exception as exc:
        logger.error(f"Database init failed — map CRUD disabled: {exc}")

    webui_config = config.get("webui", {})
    host = webui_config.get("host", "0.0.0.0")
    port = int(os.environ.get("WEBUI_PORT", webui_config.get("port", 5000)))

    ros_bridge = RosBridge(channels)
    bridge_ok = ros_bridge.start()
    if bridge_ok:
        logger.info("ROS2 bridge started — real topic data enabled")
    else:
        logger.info("ROS2 bridge unavailable — using mock data only")

    socketio.start_background_task(emit_loop)
    logger.info("emit_loop background task started")

    logger.info(f"Starting Flask-SocketIO server on {host}:{port}")
    socketio.run(app, host=host, port=port, debug=False)


if __name__ == "__main__":
    main()
