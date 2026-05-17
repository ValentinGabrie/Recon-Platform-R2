# Recon-Platform-R2 — Architecture

> Companion to [`SPEC.md`](SPEC.md). Where SPEC says **what** the system
> is, this doc shows **how data moves through it**.

---

## 1. Hardware block diagram

```
                                ┌──────────────────────────────────────────┐
                                │            Raspberry Pi 5                 │
                                │     Ubuntu 24.04 · ROS2 Jazzy             │
                                │                                            │
   ┌──────────┐                 │  ┌──────────────┐   ┌──────────────────┐ │
   │  LD14P   │ UART /ttyAMA0   │  │ ldlidar_node │──►│   slam_toolbox   │ │
   │  LIDAR   │═════════════════►  │   (vendor)   │   │  online_async    │ │
   └──────────┘   230400 baud   │  └──────────────┘   └──────┬───────────┘ │
                                │                            │ /map, /tf   │
                                │                            ▼              │
                                │            ┌─────────────────────────┐   │
                                │            │     recon_webui_bridge  │   │
                                │            │   (rclpy in OS thread)  │   │
   ┌──────────┐                 │            └────┬───────────┬─────────┘   │
   │  ESP32   │ USB-Serial      │                 │           │             │
   │ + IMU    │ /dev/ttyUSB0   ─┼──► esp32_uart_  │           │             │
   │ + 3 BTNs │ 115200 8N1      │   bridge (H2.1) │           │             │
   └──────────┘                 │       (H3) ┌────▼──────┐    │             │
                                │            │imu_filter │    │             │
                                │            │ madgwick  │    │             │
                                │            └────┬──────┘    │             │
                                │                 │ /imu/data │             │
                                │            ┌────▼──────┐    │             │
                                │            │ ekf_node  │    │             │
                                │            │ (H3)      │    │             │
                                │            └────┬──────┘    │             │
                                │             /odom │          │             │
                                │                  ▼          ▼             │
                                │           ┌────────────────────────┐     │
                                │           │     recon_webui (Flask) │     │
                                │           │     SocketIO / eventlet │     │
                                │           │     :80   AP "Recon"    │     │
                                │           └─┬─────────────────────┬┘     │
                                │             │                     │       │
                                │   /robot/   │ /api/maps           │ DB    │
                                │   events    │ /api/maps/events    │ R/W   │
                                │             ▼                     ▼       │
                                │      ┌────────────┐       ┌────────────┐ │
                                │      │  db_node   │       │ PostgreSQL │ │
                                │      │ rclpy node │──────►│  Docker    │ │
                                │      └────────────┘       └────────────┘ │
                                │                                           │
                                └──────────────────────────────────────────┘
                                          │ ap0 10.0.0.1                ▲
                                          │ wlan0 LAN                   │
                                          ▼                             │
                                       ┌────────────────────────────────┘
                                       │   Browser
                                       │   http://recon.local/map
                                       └────────────
```

H2.1 / H3 boxes are **dashed** in spirit — they don't exist yet but the
shape of the dataflow is final.

---

## 2. ROS2 node graph (today, sensor-test mode)

```
       ldlidar_node
            │ /scan
            ▼
       slam_toolbox ────► /map ──┐
            │                    │
            │ /tf (map→odom)     │
            ▼                    │
                                 │
   tf2_ros static_tf ────► odom→base_link (identity)
                                 │
                                 │
                       ┌─────────┴───────────┐
                       ▼                     ▼
              recon_webui_bridge        db_node
                       │
                       │ pose (from map→odom TF)
                       │ map (from /map)
                       │ events (from /robot/events)
                       ▼
                   recon_webui (Flask + SocketIO)
                       │
                       ▼
                   Browser
```

The currently-stubbed-out arrows for H2.1 / H3:

```
   esp32_uart_bridge (H2.1) ──► /imu/data_raw ──► imu_filter_madgwick ──► /imu/data
                            └─► /buttons/{save,reset,shutdown_request}

   imu_filter_madgwick + /odom (wheel placeholder = none) ──► ekf_node (H3) ──► /odom
                                                                           └─► /scanner/pose
```

---

## 3. Data flows in detail

### 3.1 LIDAR → SLAM → web UI

1. **Driver:** `ldlidar_node` reads UART frames at 230 400 baud and
   publishes `sensor_msgs/LaserScan` on `/scan` at ~10 Hz.
2. **SLAM:** `slam_toolbox` (online_async, Ceres solver) consumes `/scan`,
   maintains a pose graph, and publishes:
   - `nav_msgs/OccupancyGrid` on `/map` every `map_update_interval` seconds
     (currently 1.0 s — see [`SPEC.md`](SPEC.md) §5.5 for tuning).
   - `tf2_msgs/TFMessage` on `/tf` carrying the `map → odom` transform at
     ~50 Hz (`transform_publish_period: 0.02`).
3. **Web bridge:** `recon_webui_bridge` (a node embedded in
   [`ros_bridge.py`](../roomba_ws/src/recon_webui/recon_webui/ros_bridge.py))
   subscribes to `/map` and `/tf`. On every `/tf` it derives the scanner's
   pose in the map frame (since `odom→base_link` is identity in H1) and
   pushes both into per-channel buffers (`DataChannel`).
4. **WebSocket emit loop:** `app.py` runs an eventlet green thread that
   reads each channel at its configured rate and emits `map_update` /
   `robot_pose` events to all connected clients.
5. **Client renderer:** [`map.html`](../roomba_ws/src/recon_webui/recon_webui/templates/map.html)
   anchors the scanner at the canvas centre (constant `PX_PER_M`) and
   pans the map under it. Off-viewport cells are skipped on every redraw.

### 3.2 Save flow (today)

Two entry points, one DB write:

```
  Web UI POST /api/maps                          ╲
                                                  ╲
  /robot/events: "SAVE_MAP"  (from db_node       ╱── INSERT INTO maps
                              future hardware    ╱       + INSERT INTO map_events
                              SAVE button)      ╱
```

Either path ends at `db_node`'s `/robot/events` subscriber, which
inserts a row into `maps` and a `MapEvent(SAVED)` into `map_events`.
The web UI polls `/api/maps/events?since=<id>` every 3 s to pick up
saves that originated headlessly.

### 3.3 Mode toggle

```
  Browser click on IDLE / SCAN button
              │
              ▼
  socket.emit("set_mode", {mode})
              │
              ▼
  app.py on_set_mode() → ros_bridge.publish_mode()
              │
              ▼
  /robot/mode (std_msgs/String)
              │
              ├──► db_node     (logs mode in MapEvent? — TBD H5)
              └──► back into recon_webui_bridge → emit "robot_mode" to all clients
```

Mode is currently free-text `"IDLE" | "SCAN"`. H5 introduces a proper
session state machine (IDLE → SCAN → SAVE → IDLE) that gates DB writes.

### 3.4 ESP32 → Pi (H2.1+)

```
  ESP32 sketch (firmware/esp32/src/main.cpp)
        │ Serial.write(framing::send_imu(...))
        │ binary: [0xA5 0x5A][TYPE][LEN][PAYLOAD][CRC8]
        ▼
  Pi /dev/ttyUSB0 (115 200 8N1)
        │
        ▼
  esp32_uart_bridge.py (H2.1 — recon_hardware)
        │ pyserial reader thread
        │ frame parser (sync hunt → length read → CRC verify)
        ▼
  rclpy publishes sensor_msgs/Imu on /imu/data_raw
                  std_msgs/Empty   on /buttons/save  /buttons/reset  /buttons/shutdown_request
```

The frame parser must handle byte loss: lost a sync byte → drop until
next 0xA5 0x5A; bad CRC → drop frame, log at debug. See
[`UART_PROTOCOL.md`](UART_PROTOCOL.md) §4 for the resync algorithm.

---

## 4. TF tree (today and target)

### Today (H1.6, sensor-test mode)

```
map ──► odom ──► base_link ──► laser_frame
       (slam) (static, identity)  (static, from ldlidar launch)
```

The `odom → base_link` transform is the **identity** — published by a
static `tf2_ros static_transform_publisher` because we have no source
of motion estimate. Result: the scanner's pose in the map frame is
literally `map → odom`, so we read it straight off `/tf`.

### After H3 (IMU EKF lands)

```
map ──► odom ──► base_link ──► laser_frame
        (ekf_node from /imu/data,        (static)
         no wheel odom)
```

`robot_localization` ekf_node will consume `/imu/data` (orientation +
filtered angular velocity) and publish a real `odom → base_link` TF +
`/odom`. SLAM still publishes `map → odom`. The web UI keeps deriving
pose from TF — no client change needed.

### After H6 (enclosure)

The static `base_link → laser_frame` transform will reflect the actual
mechanical offset between the scanner's mounting point and the LIDAR
optics, so map coordinates correspond to a physical reference on the
device.

---

## 5. Threading model (Pi-side webui)

Three concurrent execution contexts in `recon_webui` Python process:

```
  ┌─────────────────────────────────┐
  │ eventlet green thread loop      │  ← Flask + SocketIO live here
  │  - HTTP request handlers        │  ← `tpool.execute()` for blocking calls
  │  - WebSocket emit loop          │
  │  - DB queries (via tpool)       │
  └────────────┬────────────────────┘
               │  queue.Queue (events out of rclpy)
  ┌────────────▼────────────────────┐
  │ Real OS thread (`ros2_bridge_spin`) │  ← rclpy.spin(node)
  │  - Subscriber callbacks         │
  │  - Pushes events to queue       │
  └─────────────────────────────────┘
               ▲
               │  socketio.emit() is FORBIDDEN here.
               │  Use the queue.
```

Hard rules (see [`AGENT_RULES.md`](AGENT_RULES.md) §6):

1. **`eventlet.monkey_patch()` is the first executable line** of `app.py`.
   Imports come AFTER monkey-patch.
2. **`rclpy.spin()` runs on a real OS thread**, not an eventlet green
   thread. Use `eventlet.patcher.original("threading")` to escape the
   greened threading module.
3. **Never call `socketio.emit()` from the rclpy thread.** Use a
   `queue.Queue(maxsize=64)` to hand events to the eventlet loop, and
   have `emit_loop()` drain it.
4. **Use `tpool.execute()` for blocking calls** (DB queries, subprocess)
   inside Flask routes — otherwise they freeze the green-thread loop.

---

## 6. Mock-fallback architecture

The web UI doesn't have separate "demo" and "live" code paths. Every
data source is wrapped in a `DataChannel`:

```python
DataChannel(
    topic="/scanner/pose",
    timeout_s=2.0,
    mock_fn=mock_data.mock_robot_pose,
)
```

- `is_live()` returns True iff a real ROS2 message arrived within
  `timeout_s`.
- `get()` returns the most recent real value when live, otherwise the
  result of `mock_fn()`.

Switching from demo (`setup.sh demo`, no ROS2) to sensor-test
(`setup.sh sensor-test`, real LIDAR) requires **zero code changes** —
the channels self-detect.

---

## 7. ESP32 firmware structure (single sketch, no FreeRTOS)

```
   setup()
     ├─ pinMode + Serial.begin
     ├─ Button.begin × 3
     ├─ imu::begin()
     ├─ framing::send_status(STATUS_BOOT | STATUS_IMU_OK?)
     └─ schedule next IMU/heartbeat/LED deadlines

   loop() — runs as fast as possible (~kHz)
     ├─ Button.update × 3      (debounce + edge → framing::send_button)
     ├─ if (now ≥ next_imu_ms)
     │     imu::read() → framing::send_imu()
     │     on failure: imu::begin() retry, framing::send_status()
     ├─ if (now ≥ next_heartbeat_ms)
     │     framing::send_heartbeat(uptime)
     └─ update_status_led()
```

No FreeRTOS tasks. Cooperative scheduling on `millis()` deadlines is
sufficient for this workload (100 Hz IMU + occasional buttons +
1 Hz heartbeat = ~3 KB/s of UART traffic, well below 11.5 KB/s budget
at 115 200 baud).

---

## 8. Key invariants

These are properties the system relies on. Breaking them silently
breaks downstream stages.

| Invariant | Why it matters | Enforced by |
| --------- | -------------- | ----------- |
| `slam_toolbox` always publishes `map → odom` TF, even when the device is stationary | The web UI derives pose from this TF; no TF = no red arrow | `slam_params.yaml: minimum_travel_distance: 0.0` |
| Eventlet monkey-patch runs before any other import in `app.py` | Prevents stdlib `threading.RLock` instances from being non-greened | Code review + `# noqa` comment marker |
| `socketio.emit()` only from eventlet thread | Calling from rclpy thread caused a documented deadlock | `RosBridge` uses `queue.Queue` exclusively |
| ESP32 frame CRC covers TYPE+LEN+PAYLOAD (not the sync bytes) | Pi-side parser must match | [`UART_PROTOCOL.md`](UART_PROTOCOL.md) is canonical |
| The Postgres container stays named `roomba_postgres` | Renaming would orphan the existing scan data on dev devices | `docker/docker-compose.yaml` left alone in H1 |
| All ROS2 nodes launched via `setup.sh` (never raw `ros2 launch`) | Bare launch doesn't activate the venv → SQLAlchemy/eventlet missing | Documented in `AGENT_RULES.md` |
