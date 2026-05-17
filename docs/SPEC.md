# Recon-Platform-R2 — Technical Specification

> Canonical spec as of 2026-05-10. Supersedes the autonomous-robot
> [`project_requirements.md`](archive/2026-05-10_pre-handheld/project_requirements.md).

This document describes **what the system is** at every layer. For wiring
diagrams and data-flow narratives see [`ARCHITECTURE.md`](ARCHITECTURE.md).
For "what's actually working today" see [`STATUS.md`](STATUS.md).

---

## 1. Purpose & high-level behaviour

A **handheld LIDAR scanner.** The user holds the device and walks through
a space; the device builds a 2-D occupancy grid map of the walls and
saves it to disk on demand.

There is no autonomy, no motors, no controller. The only inputs are
the device's motion (interpreted by SLAM + IMU fusion), the LIDAR scan,
and three buttons.

| Goal                                | Means                                                 |
| ----------------------------------- | ----------------------------------------------------- |
| 2-D map of indoor walls             | LD14P 360° LIDAR + slam_toolbox online async          |
| Track device pose during scanning   | MPU-6050 IMU + Madgwick fusion + EKF                  |
| Dynamic obstacle handling (people)  | slam_toolbox `map_update_interval=1.0` + raytracing   |
| Save scans for later                | Web UI / SAVE button → PostgreSQL `maps` table        |
| Live-monitor the scan               | Flask + SocketIO web UI on port 80                    |
| Network access in the field         | hostapd AP (SSID `Recon`) + dnsmasq                   |
| Power management                    | Mechanical SPST switch on battery; SHUTDOWN button = soft Pi shutdown |

---

## 2. Hardware

### 2.1 Compute & sensors

| Component        | Model / details                                          |
| ---------------- | -------------------------------------------------------- |
| SBC              | Raspberry Pi 5, 4 GB or 8 GB                             |
| OS               | Ubuntu Server 24.04 LTS (ARM64), kernel 6.8.0-1051-raspi |
| LIDAR            | LD14P (LD-D200), UART, 230 400 baud, 10 Hz scans         |
| IMU              | MPU-6050, I²C 0x68, ±4 g / ±500 °/s, 100 Hz output       |
| IMU coprocessor  | ESP32-D (DevKit V1, WROOM-32 module)                     |
| Buttons          | 3 × momentary push, normally-open, SPST                  |
| Status LED       | ESP32 onboard blue LED on GPIO 2                         |
| Power            | Battery → mechanical SPST switch → 5 V buck → Pi + ESP32 |

### 2.2 LIDAR wiring (LD14P → Pi 5)

The LD14P uses **non-standard wire colours** — confirmed empirically.

| LD14P wire | Function     | Pi pin                      |
| ---------- | ------------ | --------------------------- |
| Black      | VCC (5 V)    | Pin 4 (5V)                  |
| Green      | GND          | Pin 6 (GND)                 |
| White      | TX (data)    | Pin 10 (GPIO 15, UART0 RXD) |
| Red        | RX (unused)  | Leave disconnected          |

Pi UART0 / `/dev/ttyAMA0` is dedicated to the LIDAR. The serial console
must be disabled and `dtoverlay=miniuart-bt` set so PL011 is free —
[`environment.sh`](../roomba_ws/environment.sh) Section 2 handles this.

### 2.3 ESP32 wiring

| Function          | ESP32 pin        | Notes                                       |
| ----------------- | ---------------- | ------------------------------------------- |
| MPU-6050 SDA      | GPIO 21          | I²C 400 kHz                                 |
| MPU-6050 SCL      | GPIO 22          |                                             |
| MPU-6050 VCC      | 3V3              | 3.3 V module — NOT 5 V                      |
| MPU-6050 AD0      | GND              | I²C address `0x68`                          |
| Button SHUTDOWN   | GPIO 25 → GND    | INPUT_PULLUP, active LOW                    |
| Button RESET      | GPIO 26 → GND    | "                                           |
| Button SAVE       | GPIO 27 → GND    | "                                           |
| Status LED        | GPIO 2           | Onboard                                     |
| UART to Pi        | USB micro        | Shared with on-board USB-Serial bridge      |

The ESP32 plugs into a Pi USB port for **both** power and comms during
bench bring-up; the same connection becomes the data link in the
finished enclosure.

### 2.4 Power

The battery rail is gated by a **mechanical SPST switch** — physically
turning the device on/off. The ESP32 SHUTDOWN button is a *signal* only;
when held, it triggers a graceful Pi `shutdown -h now` via the bridge.
Cutting actual power requires flicking the SPST.

---

## 3. Software stack

### 3.1 Pi 5 — system layer

| Component       | Version / purpose                                           |
| --------------- | ----------------------------------------------------------- |
| Ubuntu          | 24.04 LTS (ARM64)                                           |
| Python          | 3.12 (system) + venv at `roomba_ws/.venv` (`--system-site-packages`) |
| ROS2            | Jazzy Jalisco (`/opt/ros/jazzy/`)                           |
| Docker          | for the PostgreSQL container only                           |
| hostapd + dnsmasq | WiFi AP `Recon` on `ap0` virtual iface, IP 10.0.0.1       |

Provisioning is a single idempotent script: [`environment.sh`](../roomba_ws/environment.sh).
Run with no args for full install, with `--check` for verification only.

### 3.2 Pi 5 — ROS2 packages

The colcon workspace lives at `roomba_ws/`. Its source dir contains
five `recon_*` packages plus the vendored LIDAR driver.

| Package          | Lang  | Role                                                                    |
| ---------------- | ----- | ----------------------------------------------------------------------- |
| `recon_hardware` | C++17 | `sim_sensor_node` — simulated LIDAR for SLAM bench-testing without hardware |
| `recon_control`  | C++17 | `draw_node` — web-driven OccupancyGrid canvas for end-to-end DB testing |
| `recon_db`       | Python| `db_node` — saves maps + emits `MapEvent` rows on SAVE_MAP              |
| `recon_webui`    | Python| Flask + SocketIO web server with embedded `RosBridge` (rclpy)           |
| `recon_bringup`  | Python| `full_system.launch.py` — bringup glue (placeholder until H4)           |
| `ldlidar_stl_ros2` | C++ | Vendored LD14P driver (publishes `/scan`)                               |

Coding rules per [`AGENT_RULES.md`](AGENT_RULES.md):
- C++17 for real-time / hardware paths.
- Python 3.12 for non-RT (web, DB).
- One node per file. Never two ROS2 nodes in one source file.
- Config over constants — every tunable comes from YAML or ROS2 params.

### 3.3 ESP32 — firmware

Single PlatformIO project at [`firmware/esp32/`](../firmware/esp32/),
`board=esp32dev`, `framework=arduino`. Source files:

| File                  | Role                                                  |
| --------------------- | ----------------------------------------------------- |
| `src/main.cpp`        | `setup()`/`loop()` cooperative scheduler              |
| `src/config.h`        | Pinout, rates, frame types, status flags              |
| `src/framing.{h,cpp}` | UART frame serialiser + Dallas/Maxim CRC8             |
| `src/imu.{h,cpp}`     | Register-level MPU-6050 driver via `Wire.h`           |
| `src/buttons.{h,cpp}` | Debounce + long-press state machine                   |

No external libraries — only `Arduino.h` and `Wire.h` from the framework.

Frame types emitted:

| Type | Name      | Payload                                | Rate         |
| ---- | --------- | -------------------------------------- | ------------ |
| 0x01 | IMU       | 6 × float32: ax,ay,az / gx,gy,gz       | 100 Hz       |
| 0x02 | BUTTON    | uint8 id, uint8 state                  | edge events  |
| 0x03 | HEARTBEAT | uint32 uptime_ms                       | 1 Hz         |
| 0x04 | STATUS    | uint8 flags, uint8 reserved            | boot + on IMU error |

Wire format is canonical in [`UART_PROTOCOL.md`](UART_PROTOCOL.md).

### 3.4 ESP32 ↔ Pi link

USB-Serial via the ESP32's on-board CP2102/CH340 bridge. UART0
(GPIO 1/3) at **115 200 8N1**. Pi sees the device as
`/dev/ttyUSB0` (or `/dev/ttyACM0` depending on the bridge chip).
The Pi-side bridge node lands in **H2.1**.

---

## 4. ROS2 topics & node graph

This section is the contract — what publishes what, what subscribes to
what. See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the data-flow narrative.

### 4.1 Live topics (sensor-test mode, today)

| Topic              | Type                              | Pub                       | Sub                       |
| ------------------ | --------------------------------- | ------------------------- | ------------------------- |
| `/scan`            | sensor_msgs/LaserScan             | `ldlidar_node`            | `slam_toolbox`            |
| `/tf`              | tf2_msgs/TFMessage                | `slam_toolbox`, static_tf | `recon_webui_bridge`, slam |
| `/map`             | nav_msgs/OccupancyGrid            | `slam_toolbox`            | `recon_webui_bridge`, `db_node` |
| `/scanner/pose`    | geometry_msgs/PoseStamped         | (H3 EKF — currently none) | `recon_webui_bridge`      |
| `/robot/mode`      | std_msgs/String                   | `recon_webui` (set_mode)  | `recon_webui_bridge`, `db_node` |
| `/robot/events`    | std_msgs/String                   | `draw_node`, `db_node`    | `db_node`, `recon_webui_bridge` |
| `/draw/command`    | std_msgs/String                   | (web UI — H5)             | `draw_node`               |
| `/sim/ground_truth`| nav_msgs/OccupancyGrid            | `sim_sensor_node`         | (debug viz only)          |

### 4.2 Topics introduced by upcoming stages

| Stage | Topic              | Type                   | Pub                          | Notes                               |
| ----- | ------------------ | ---------------------- | ---------------------------- | ----------------------------------- |
| H2.1  | `/imu/data_raw`    | sensor_msgs/Imu        | `esp32_uart_bridge`          | Raw accel + gyro from MPU-6050      |
| H2.1  | `/buttons/save`    | std_msgs/Empty         | `esp32_uart_bridge`          | One per SAVE press                  |
| H2.1  | `/buttons/reset`   | std_msgs/Empty         | `esp32_uart_bridge`          |                                     |
| H2.1  | `/buttons/shutdown_request` | std_msgs/Empty | `esp32_uart_bridge`          | Long-press → soft Pi shutdown       |
| H3    | `/imu/data`        | sensor_msgs/Imu        | `imu_filter_madgwick`        | Orientation-fused IMU               |
| H3    | `/odom`            | nav_msgs/Odometry      | `robot_localization` ekf_node | Replaces the static identity TF    |
| H3    | `/scanner/pose`    | geometry_msgs/PoseStamped | small republisher          | `/odom.pose` repacked for the UI    |

### 4.3 TF tree

```
                       (slam_toolbox publishes)
              map ────────────────────────► odom
                                              │
                  (H1: static identity from setup.sh)   ← replaced in H3
                  (H3: from robot_localization EKF)
                                              ▼
                                          base_link
                                              │
                  (static, from ldlidar_stl_ros2 launch)
                                              ▼
                                         laser_frame
```

`base_link` and `laser_frame` are coincident in H1; once an enclosure
exists in H6 the static TF will reflect the actual offset.

---

## 5. Web UI

### 5.1 Pages

| Path    | Title       | Purpose                                                |
| ------- | ----------- | ------------------------------------------------------ |
| `/`     | Dashboard   | Live pose, mode toggle (IDLE/SCAN), event log           |
| `/map`  | Live Map    | OccupancyGrid centred on the scanner, save/list maps    |

The Dashboard subscribes to `robot_pose` + `robot_mode` + `robot_event`
WebSocket events; the map page also subscribes to `map_update`. Channels
fall back to mock data automatically via `DataChannel.is_live()`
timeouts — no LIVE/DEMO mode flag.

### 5.2 REST API

| Route                                | Method | Returns / Body                                  |
| ------------------------------------ | ------ | ----------------------------------------------- |
| `/api/robot/status`                  | GET    | `{mode}`                                        |
| `/api/robot/mode`                    | POST   | `{mode}` → publishes on `/robot/mode`           |
| `/api/debug/channels`                | GET    | `{channels:{...}, bridge:{available, running}}` |
| `/api/maps`                          | GET    | `[{id, name, created_at, resolution, w, h}, …]` |
| `/api/maps`                          | POST   | `{name?}` → save current `/map` channel         |
| `/api/maps/<id>`                     | GET    | metadata                                        |
| `/api/maps/<id>`                     | PUT    | `{name}` → rename                               |
| `/api/maps/<id>`                     | DELETE | delete + `MapEvent(DELETED)`                    |
| `/api/maps/<id>/data`                | GET    | `{width, height, resolution, origin_*, data}`   |
| `/api/maps/events?since=<id>`        | GET    | `[MapEvent, …]` for headless-save polling       |

### 5.3 WebSocket events

| Direction   | Event             | Payload                                             |
| ----------- | ----------------- | --------------------------------------------------- |
| server→client | `robot_pose`    | `{x, y, theta}` (map frame, m + rad)                |
| server→client | `map_update`    | `{width, height, resolution, origin_x, origin_y, data}` |
| server→client | `robot_mode`    | `{mode: "IDLE"\|"SCAN"}`                            |
| server→client | `robot_event`   | `{type, message}`                                   |
| server→client | `channel_status`| `{<name>: {live, last_seen_s}, …}`                  |
| client→server | `set_mode`      | `{mode}`                                            |
| client→server | `connect`       | (none — handler emits initial state)                |

### 5.4 Threading model

- `app.py` calls `eventlet.monkey_patch()` as the very first executable
  statement.
- All HTTP routes that touch the DB go through `eventlet.tpool.execute(...)`
  to avoid blocking the green-thread loop.
- `RosBridge` runs `rclpy.spin()` on a **real** OS thread (via
  `eventlet.patcher.original("threading")`).
- Cross-thread events from rclpy → eventlet go through a `queue.Queue`
  drained by `emit_loop`. **`socketio.emit()` is never called from the
  rclpy thread.** This rule is binding (see `AGENT_RULES.md` §6).

### 5.5 Performance tunables

| Knob                              | File                       | Default | Effect                          |
| --------------------------------- | -------------------------- | ------- | ------------------------------- |
| `emit_rates.map_update`           | `config/webui.yaml`        | 5 Hz    | WebSocket map push rate         |
| `emit_rates.robot_pose`           | `config/webui.yaml`        | 10 Hz   | Pose push rate                  |
| `slam_toolbox.map_update_interval`| `config/slam_params.yaml`  | 1.0 s   | Lower → faster transient-obstacle clearing, more CPU |
| `slam_toolbox.minimum_time_interval` | `config/slam_params.yaml`| 0.1 s   | Lower → faster scan-matcher updates |
| `PX_PER_M`                        | `templates/map.html`       | 60      | Map zoom (px per metre)         |

---

## 6. Database

PostgreSQL 16 in Docker. Container name **`roomba_postgres`** (kept
post-pivot to preserve historical scan history); env var `RECON_DB_URL`
points the app at it.

### 6.1 Schema

| Table        | Columns                                                       |
| ------------ | ------------------------------------------------------------- |
| `maps`       | `id PK, name, map_data BYTEA, origin_x, origin_y, resolution, width, height, created_at, updated_at` |
| `sessions`   | `id PK, mode, map_id FK→maps.id ON DELETE SET NULL, created_at` |
| `map_events` | `id PK, event_type ENUM(SAVED/DELETED), map_id, map_name, created_at` |

### 6.2 ORM

[`recon_db/models.py`](../roomba_ws/src/recon_db/recon_db/models.py) —
SQLAlchemy 2.x with `QueuePool` for PostgreSQL and `StaticPool` for
SQLite (used in tests). Schema is auto-created via
`Base.metadata.create_all()` at startup. Alembic is scaffolded in
`recon_db/migrations/` but not yet driving migrations.

### 6.3 Save pipeline

```
SAVE button (H3+)            ╲
                              ╲   /robot/events:"SAVE_MAP"
draw_node "save" command      ╱── ─────────────────────────►   db_node
                              ╱                                    │
Web UI POST /api/maps        ╱                                     │
                                                                   ▼
                                                       INSERT INTO maps,
                                                       INSERT INTO map_events
                                                                   │
                                                                   ▼
                                                  Web UI polls /api/maps/events?since
                                                  every 3 s and refreshes the table.
```

---

## 7. Configuration files

All YAML lives under `roomba_ws/config/`:

| File             | Purpose                                                              |
| ---------------- | -------------------------------------------------------------------- |
| `webui.yaml`     | Flask host/port, WebSocket emit rates, channel timeouts              |
| `slam_params.yaml` | slam_toolbox online-async parameters (Ceres, ranges, rates)        |
| `hardware.yaml`  | LIDAR serial port, baud, range limits                                |
| `simulation.yaml` | Random-room generation params for `sim_sensor_node`                 |

ESP32-side tunables (pinout, IMU rate, frame sync bytes) live in
[`firmware/esp32/src/config.h`](../firmware/esp32/src/config.h).

---

## 8. Build & run

### 8.1 Build the workspace

```bash
cd ~/Recon-Platform-R2/roomba_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

### 8.2 Run modes (`setup.sh`)

| Mode          | What it brings up                                              |
| ------------- | -------------------------------------------------------------- |
| `kill`        | Tears down tmux session + stale recon processes                |
| `demo`        | Web UI only, mock data — no ROS2, no DB                        |
| `web`         | Web UI + DB node, ROS2 running, no hardware                    |
| `sensor-test` | LIDAR + static `odom→base_link` + slam_toolbox + DB + Web UI   |

`setup.sh` is the **only** supported entry point. Direct `ros2 launch`
will run nodes in subprocesses without venv activation, causing
`ModuleNotFoundError` for sqlalchemy/eventlet.

### 8.3 Build the ESP32 firmware

```bash
cd ~/Recon-Platform-R2/firmware/esp32
pio run                  # build
pio run -t upload        # flash
pio device monitor       # 115200 baud serial console
```

---

## 9. Testing

| Suite                     | Runner             | Count        | Notes                          |
| ------------------------- | ------------------ | ------------ | ------------------------------ |
| `tests/test_db_node.py`   | pytest (in-mem SQLite) | 9 tests   | CRUD, relationships, cascade   |
| `tests/test_recon_webui.py` | pytest             | 6 tests    | DataChannel + mock data        |
| `tests/test_draw_node.cpp`  | gtest via colcon   | 4 tests    | Grid layout, paint, clear, brush clamp |
| `tests/test_sim_sensor_node.cpp` | gtest         | 4 tests    | Raycast, room connectivity     |

ESP32 firmware has no unit tests yet — bench testing planned for H2.1
once the Pi-side bridge can decode the frames.

---

## 10. Out of scope (today)

- **Multi-floor / 3D mapping.** Single-storey, 2-D occupancy only.
- **Loop closure across long sessions.** slam_toolbox does loop closure
  within a session; cross-session merging is a manual workflow.
- **Indoor navigation / pathfinding.** No autonomy, no `nav2`.
- **Cloud sync.** Maps live in the Pi's PostgreSQL; export is on the user.
- **Authentication on the web UI.** Designed for the AP-only operating
  model. If exposed to the LAN, add auth before trusting it.

See [`ROADMAP.md`](ROADMAP.md) for what's explicitly planned.
