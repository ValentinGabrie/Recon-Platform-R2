# Recon-Platform-R2 — Technical Specification

> Canonical spec as of 2026-05-25. Supersedes the autonomous-robot
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
| Power management                    | 22.5 W USB-C power bank → Pi 5; SHUTDOWN button = soft Pi shutdown |

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
| Power            | 22.5 W USB-C power bank → Pi 5 → (USB) ESP32 → (5 V rail) LD14P |

### 2.2 LIDAR wiring (LD14P → ESP32 → Pi)

The LD14P uses **non-standard wire colours** — confirmed against the
datasheet pinout (the GND/RX colours had been swapped in an earlier
note, corrected 2026-05-23):

| LD14P wire | Function    | Connects to                                        |
| ---------- | ----------- | -------------------------------------------------- |
| Black      | VCC (5 V)   | ESP32 5V (VIN) rail                                |
| White      | TX (data)   | ESP32 GPIO 16 (Serial2 RX) — 3.3 V CMOS, no level-shifter needed |
| Green      | RX          | S8050 collector (alongside Red) — "pull to ground for internal speed control"; with the transistor on, sees LIDAR-local GND |
| Red        | GND         | S8050 collector                                    |

Both the Red (GND) and Green (RX) wires terminate at the **S8050 NPN
collector** — switching both simultaneously prevents phantom-powering
through the LIDAR's RX pull-up when the motor is off.

```
ESP32 GPIO 4 ──[ 1 kΩ ]── S8050 base
ESP32 GND   ────────────── S8050 emitter
LD14P Red + Green ──────── S8050 collector
LD14P Black ─────────────── ESP32 5V (VIN)
LD14P White ─────────────── ESP32 GPIO 16 (Serial2 RX @ 230 400)
```

When GPIO 4 is HIGH (3.3 V) the S8050 saturates → LIDAR ground completes
→ motor spins. When GPIO 4 is LOW the transistor cuts off → LIDAR is
unpowered → motor stops. Firmware drives this line via the `LIDAR_EN`
opcode from the Pi (see §3.3).

**Pi UART0 is no longer used.** The previous direct `/dev/ttyAMA0` path
is electrically disconnected — both PL011 and the kernel serial console
config in [`environment.sh`](../roomba_ws/environment.sh) Section 2 are
left in place but unused for LIDAR.

### 2.3 ESP32 wiring

| Function           | ESP32 pin        | Notes                                       |
| ------------------ | ---------------- | ------------------------------------------- |
| MPU-6050 SDA       | GPIO 21          | I²C 400 kHz                                 |
| MPU-6050 SCL       | GPIO 22          |                                             |
| MPU-6050 VCC       | 3V3              | 3.3 V module — NOT 5 V                      |
| MPU-6050 AD0       | GND              | I²C address `0x68`                          |
| Button SHUTDOWN    | GPIO 25 → GND    | INPUT_PULLUP, active LOW                    |
| Button START/STOP  | GPIO 26 → GND    | " (formerly "RESET"; `PIN_BTN_STARTSTOP`)   |
| Button SAVE        | GPIO 27 → GND    | "                                           |
| Status LED         | GPIO 2           | Onboard                                     |
| **LIDAR enable**   | **GPIO 4**       | Drives S8050 base via 1 kΩ. Set LOW as the first line of `setup()` so the motor stays off through boot. |
| **LIDAR data (RX)**| **GPIO 16**      | Serial2 RX @ 230 400 baud. TX (GPIO 17) unused. |
| UART to Pi         | USB micro        | Shared with on-board CP2102. **460 800 baud** since Inc 1 (was 115 200). |

The ESP32 plugs into a Pi USB port for **both** power and comms during
bench bring-up; the same connection becomes the data link in the
finished enclosure. LD14P current draw (~200–300 mA) comes from the
ESP32's 5V rail, which itself is sourced from the Pi USB port — the
total budget stays under the USB 500 mA limit.

### 2.4 Power

The whole device runs from a single **22.5 W USB-C power bank** plugged
into the Pi 5's USB-C input. The Pi powers the ESP32 from one of its USB
ports, and the ESP32's 5 V rail in turn powers the LD14P — one source, no
wall outlet, nothing to wire on the high-current side. The power bank's
own button is the hard on/off.

The ESP32 SHUTDOWN button is a *signal* only; when held, it triggers a
graceful Pi `shutdown -h now` via the bridge. Cutting actual power means
switching the power bank off (or unplugging the USB-C cable).

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
| `ldlidar_stl_ros2` | C++ | Vendored LD14P driver (publishes `/scan`). **Locally patched** (nested commits `35b3c8c` + `42688f6`): `ld14p.launch.py` opens `/tmp/lidar_pty`; `demo.cpp` waits indefinitely for first packet AND publishes scans with a fixed 720-beam (0.5°) geometry so slam_toolbox's first-scan-locks-the-count check stops rejecting later rotations. |

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

Frame types (bidirectional since Inc 1):

| Type | Name          | Dir   | Payload                                | Rate / trigger |
| ---- | ------------- | ----- | -------------------------------------- | ------------------- |
| 0x01 | IMU           | ESP→Pi| 6 × float32: ax,ay,az / gx,gy,gz       | 100 Hz              |
| 0x02 | BUTTON        | ESP→Pi| uint8 id, uint8 state                  | edge events         |
| 0x03 | HEARTBEAT     | ESP→Pi| uint32 uptime_ms                       | 1 Hz                |
| 0x04 | STATUS        | ESP→Pi| flags + diagnostic                     | boot + on IMU error |
| 0x05 | **LIDAR_FRAME** | ESP→Pi | 1..64 raw LD14P bytes               | as bytes arrive (motor on only) |
| 0x06 | **LIDAR_EN**  | Pi→ESP|  1 B (0=off, 1=on)                     | event + 1 Hz refresh |
| 0x07 | **LIDAR_ACK** | ESP→Pi|  1 B (current motor state)             | on state change     |

Wire format and full opcode reference are canonical in
[`UART_PROTOCOL.md`](UART_PROTOCOL.md). `MAX_PAYLOAD = 64`.

The firmware sets `PIN_LIDAR_EN` (GPIO 4) LOW as the **first instruction**
in `setup()`, before any `delay` or `Serial.begin`, so the motor stays
off through the ~200 ms ESP32 boot window even without an external
pull-down resistor. A 3 s watchdog forces the motor off if the Pi
stops refreshing `LIDAR_EN=1`.

### 3.4 ESP32 ↔ Pi link

USB-Serial via the ESP32's on-board CP2102/CH340 bridge. UART0
(GPIO 1/3) at **460 800 8N1**. Pi sees the device as `/dev/ttyUSB0`
(CP2102) or `/dev/ttyACM0` (CH340 / native USB variants). The baud was
bumped from 115 200 in Inc 1 to carry LIDAR data (~23 KB/s) + IMU
(~3 KB/s) + framing overhead — 460 800 8N1 has ~46 KB/s headroom.

The link is **bidirectional**: the Pi-side bridge writes `LIDAR_EN`
frames to control the motor, and the firmware reads them with the same
framing parser used on the Pi side (mirrored implementation in C++).

---

## 4. ROS2 topics & node graph

This section is the contract — what publishes what, what subscribes to
what. See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the data-flow narrative.

### 4.1 Live topics (full mode, today)

| Topic / service              | Type                              | Pub                       | Sub                       |
| ---------------------------- | --------------------------------- | ------------------------- | ------------------------- |
| `/scan`                      | sensor_msgs/LaserScan             | `ldlidar_stl_ros2_node`   | `slam_toolbox`            |
| `/tf`                        | tf2_msgs/TFMessage                | `slam_toolbox`, ekf_node, static_tf | `recon_webui_bridge`, slam |
| `/map`                       | nav_msgs/OccupancyGrid            | `slam_toolbox`            | `recon_webui_bridge`, `db_node` |
| `/imu/data_raw`              | sensor_msgs/Imu                   | `esp32_uart_bridge`       | `imu_yaw_integrator`      |
| `/imu/data`                  | sensor_msgs/Imu                   | `imu_yaw_integrator`      | ekf_node, slam_toolbox    |
| `/odom`                      | nav_msgs/Odometry                 | ekf_node                  | `recon_webui_bridge`      |
| `/buttons/*`                 | std_msgs/Empty                    | `esp32_uart_bridge`       | `db_node`, `recon_webui_bridge` |
| `/esp32/diagnostics`         | std_msgs/String (JSON)            | `esp32_uart_bridge`       | `recon_webui_bridge`      |
| `/robot/mode`                | std_msgs/String                   | `recon_webui` (set_mode)  | `recon_webui_bridge`, `db_node` |
| `/robot/events`              | std_msgs/String                   | `draw_node`, `db_node`    | `db_node`, `recon_webui_bridge` |
| `/draw/command`              | std_msgs/String                   | (web UI — H5)             | `draw_node`               |
| `/sim/ground_truth`          | nav_msgs/OccupancyGrid            | `sim_sensor_node`         | (debug viz only)          |
| **`/lidar_enable` (service)** | **std_srvs/SetBool**             | `esp32_uart_bridge`       | `recon_webui_bridge` (called from `set_scanning()`) |
| **`/slam_toolbox/set_parameters`** (service) | **rcl_interfaces/SetParameters** | `slam_toolbox` | `recon_webui_bridge` (sets `paused_new_measurements` to gate scan integration) |
| **`/slam_toolbox/reset`** (service) | **slam_toolbox/srv/Reset**       | `slam_toolbox` | `recon_webui_bridge` (called from `clear_map()` via `POST /api/map/clear`; request passes `pause_new_measurements=false` to preserve scan state across the reset) |

### 4.2 Channel notes

- **`/imu/data_raw`** (H2.1) — raw accel + gyro from MPU-6050, BEST_EFFORT QoS, ~100 Hz.
- **`/imu/data`** (H3) — orientation quaternion populated by `imu_yaw_integrator` (yaw only, roll/pitch=0). slam_toolbox uses it as a scan-match prior via the `imu_topic` param.
- **`/odom`** (H3) — EKF fuses `/imu/data` yaw + yaw-rate in 2-D mode (accel disabled because of the chip's factory ZA_OFFSET bias). Position stays at origin until slam_toolbox supplies translation via `map→odom`.
- **`/tf` (`odom → base_link`)** (H3) — dynamic, published by ekf_node, replaces the static identity TF.
- **`/buttons/*`** — SAVE / START/STOP (`/buttons/startstop`, formerly `/buttons/reset`) / SHUTDOWN_REQUEST / SHUTDOWN_LONGPRESS edges from the ESP32. `recon_webui` acts on them (see §5.6).
- **`/esp32/diagnostics`** — JSON link-health blob @ 1 Hz: port_open, frame_counts (now incl. `lidar_frame`, `lidar_bytes`, `lidar_ack`), uptime, boot STATUS, **`lidar.desired_on / acked_on / pty_overflows`**.
- **`/lidar_enable`** (Inc 1) — `std_srvs/SetBool`. Called by `ros_bridge.set_scanning()` in lockstep with the SLAM pause parameter. The bridge also refreshes `LIDAR_EN=1` every 1 s while the motor is on so the firmware watchdog never trips.
- **`/scanner/pose`** — not directly published. The web bridge composes `map→odom ∘ odom→base_link` for the UI pose channel.

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
| `/stats`| Telemetry   | ESP32 link health, IMU live values + sparklines, SLAM stats (H2.1) |

The Dashboard subscribes to `robot_pose` + `robot_mode` + `robot_event`
WebSocket events; the map page also subscribes to `map_update`. Channels
fall back to mock data automatically via `DataChannel.is_live()`
timeouts — no LIVE/DEMO mode flag.

### 5.2 REST API

| Route                                | Method | Returns / Body                                  |
| ------------------------------------ | ------ | ----------------------------------------------- |
| `/api/robot/status`                  | GET    | `{mode}`                                        |
| `/api/robot/mode`                    | POST   | `{mode}` → publishes on `/robot/mode`           |
| `/api/scan/state`                    | GET    | `{active}`                                      |
| `/api/scan/start`                    | POST   | `{active, slam_responded, lidar_responded, mode}` — enables LIDAR motor then unpauses SLAM |
| `/api/scan/pause`                    | POST   | same shape — pauses SLAM then disables LIDAR motor |
| `/api/map/clear`                     | POST   | `{success, slam_responded, message}` — calls `/slam_toolbox/reset` to wipe the live pose graph + occupancy grid; preserves the current Start/Pause state. UI also wipes the canvas locally on success so the empty state is visible even while SLAM is paused. |
| `/api/debug/channels`                | GET    | `{channels:{...}, bridge:{available, running}}` |
| `/api/maps`                          | GET    | `[{id, name, created_at, resolution, w, h}, …]` |
| `/api/maps`                          | POST   | `{name?}` → save current `/map` channel         |
| `/api/maps/<id>`                     | GET    | metadata                                        |
| `/api/maps/<id>`                     | PUT    | `{name}` → rename                               |
| `/api/maps/<id>`                     | DELETE | delete + `MapEvent(DELETED)`                    |
| `/api/maps/<id>/data`                | GET    | `{width, height, resolution, origin_*, data}`   |
| `/api/maps/<id>/process`             | POST   | Run Tier-2 post-processing (median + morphology + connected components + Hough walls + Manhattan deskew) and persist a ProcessedMap row |
| `/api/maps/<id>/processed`           | GET    | List ProcessedMap rows for a saved map          |
| `/api/processed/<id>/data`           | GET    | Cleaned grid + per-cell cluster labels + `line_segments`, `n_lines`, `deskew_deg` (degrees the grid was rotated to axis-align dominant walls; 0 when left as-is) |
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

### 5.6 Hardware front-panel buttons

`recon_webui` subscribes to the ESP32 `/buttons/*` topics and turns each press
into an action. The button callback runs on the rclpy spin thread, so it only
*enqueues* an action; the eventlet `emit_loop` drains the queue and executes
each handler in its own greenlet (where the greened SLAM/LIDAR service helpers
and the DB write are safe).

| Button | Topic | Trigger | Action |
| ------ | ----- | ------- | ------ |
| **SAVE** | `/buttons/save` | press | **State toggle.** *Scanning (LIDAR on)* → save the live map to PostgreSQL, then `clear_map()` (reset the SLAM grid), then `set_scanning(False)` (pause SLAM + cut the LIDAR motor); if there's no map to save the clear is skipped so nothing is lost. *Stopped (LIDAR off)* → `set_scanning(True)` to start the LIDAR; any cached map is kept (SLAM builds on top of it) — no save, no clear. |
| **START/STOP** | `/buttons/startstop` | press | Restart the whole scanner stack via `buttons.restart_cmd` (default `sudo systemctl --no-block restart recon-stack.service`). Single-service model: the in-stack handler can't cold-start itself after a stop, so the closest to "off then on" is a full restart. |
| **SHUTDOWN** | `/buttons/shutdown_longpress` | **long-press ≥ 2 s** | Best-effort `set_scanning(False)`, then power off the Pi via `buttons.shutdown_cmd` (default `sudo shutdown -h now`). A short SHUTDOWN press (`/buttons/shutdown_request`) is log-only — guards against an accidental tap powering the device off. |

Config lives under `webui.buttons` in `config/webui.yaml`: `enabled` (master
safety switch — when `false` the privileged START/STOP and SHUTDOWN actions are
skipped, SAVE still works), `restart_cmd`, `shutdown_cmd`. The two privileged
commands need a NOPASSWD sudoers rule, installed by `environment.sh` Section 9.6
at `/etc/sudoers.d/recon-buttons`.

---

## 6. Database

PostgreSQL 16 in Docker. Container name **`roomba_postgres`** (kept
post-pivot to preserve historical scan history); env var `RECON_DB_URL`
points the app at it.

### 6.1 Schema

| Table             | Columns                                                       |
| ----------------- | ------------------------------------------------------------- |
| `maps`            | `id PK, name, map_data BYTEA, origin_x, origin_y, resolution, width, height, created_at, updated_at` |
| `sessions`        | `id PK, mode, map_id FK→maps.id ON DELETE SET NULL, created_at` |
| `map_events`      | `id PK, event_type ENUM(SAVED/DELETED), map_id, map_name, created_at` |
| `processed_maps`  | `id PK, source_map_id FK→maps.id ON DELETE CASCADE, algorithm, parameters JSONB, processed_data BYTEA, n_clusters, n_noise_cells, created_at` |

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

| File                  | Purpose                                                              |
| --------------------- | -------------------------------------------------------------------- |
| `webui.yaml`          | Flask host/port, WebSocket emit rates, channel timeouts              |
| `slam_params.yaml`    | slam_toolbox online-async parameters (Ceres, ranges, rates, `imu_topic`, `paused_new_measurements: true` boot default) |
| `hardware.yaml`       | LIDAR serial port, baud, range limits                                |
| `simulation.yaml`     | Random-room generation params for `sim_sensor_node`                  |
| `ekf.yaml`            | robot_localization ekf_node — 2-D mode, IMU yaw + yaw-rate only, accel disabled |
| `esp32_bridge.yaml`   | esp32_uart_bridge — `port`, `baud=460800`, `frame_id`, `diag_period_s`, `lidar_refresh_s`, `lidar_pty_link` |

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
| `imu-test`    | ESP32 bridge + yaw integrator + EKF + static `base_link→imu_link` TF + DB + Web UI (H2.1+H3) |
| `sensor-test` | LIDAR + slam_toolbox + (ESP32 bridge+EKF, optional) + DB + Web UI. Diagnostic mode; pass `--no-esp32` for LIDAR-only. |
| `full`        | **The canonical scanning mode.** LIDAR + ESP32 + yaw integrator + IMU TF + EKF + slam_toolbox + DB + Web UI. No optional fallbacks — every prereq must be present. `/map` page draws a blue trail of your past poses. |

`setup.sh` is the **only** supported entry point. Direct `ros2 launch`
will run nodes in subprocesses without venv activation, causing
`ModuleNotFoundError` for sqlalchemy/eventlet.

### 8.3 Build the ESP32 firmware

```bash
cd ~/Recon-Platform-R2/firmware/esp32
pio run                  # build
pio run -t upload        # flash
pio device monitor -b 460800   # serial console at the project baud
```

---

## 9. Testing

| Suite                              | Runner             | Count   | Notes                                              |
| ---------------------------------- | ------------------ | ------- | -------------------------------------------------- |
| `tests/test_db_node.py`            | pytest (in-mem SQLite) | 9    | CRUD, relationships, cascade                       |
| `tests/test_recon_webui.py`        | pytest             | 6       | DataChannel + mock data                            |
| `tests/test_esp32_uart_bridge.py`  | pytest             | 12      | CRC8, IMU/BUTTON/HEARTBEAT/STATUS frames, resync, bad CRC, oversized LEN, chunked input |
| `tests/test_imu_yaw_integrator.py` | pytest             | 12      | Yaw integration step (dt clamp, wrap), quaternion form |
| `tests/test_postprocess.py`        | pytest             | 33      | Tier-2 pipeline — median, morphology, connected components, Hough, Manhattan deskew + snap + concentration guard |
| `tests/test_imu_calib.py`          | pytest             | 5       | IMU gyro-bias autocal (still/moving), accel-bias subtraction |
| `tests/test_draw_node.cpp`         | gtest via colcon   | 4       | Grid layout, paint, clear, brush clamp             |
| `tests/test_sim_sensor_node.cpp`   | gtest via colcon   | 4       | Raycast, room connectivity                          |

LIDAR_FRAME / LIDAR_EN / LIDAR_ACK opcodes are exercised live via the
ESP32 round-trip — pytest coverage for those is a follow-up.

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
