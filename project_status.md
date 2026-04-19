# PROJECT STATUS — AUTONOMOUS MAPPING ROBOT
### Codename: `roomba`
**Date:** 2026-04-20 (last updated)  
**Platform:** Raspberry Pi 5 · Ubuntu Server 24.04 LTS (ARM64) · Kernel 6.8.0-1047-raspi  
**ROS2:** Jazzy Jalisco · Python 3.12 · C++17  

---

## 1. EXECUTIVE SUMMARY

The project has completed **environment setup**, **full workspace scaffolding**, **web UI implementation**, **real Xbox controller pipeline testing**, **draw-test DB persistence pipeline**, a **full documentation audit**, **WiFi Access Point networking**, **headless save pipeline rearchitecture**, and **simulated hardware testing (Stage 4c)**. All 6 ROS2 packages build successfully. The `simulate-hw` mode has been tested extensively: SLAM builds maps from simulated LIDAR, fuzzy frontier-based exploration covers the full room, and the web UI provides a streamlined side-by-side map view with inline mode controls. The controller pipeline (joy_linux_node → joy_control_node → webui) has been tested live with a real Xbox controller connected via Bluetooth. The draw-test pipeline (joy_linux_node → draw_node → db_node → webui) has been tested end-to-end with controller-driven drawing and headless saves. The web UI serves all 4 pages on port 80 with mock/real data fallback working. The Pi runs a WiFi hotspot (SSID `Roomba`) via hostapd+dnsmasq concurrently with its existing WiFi connection, so clients can access the UI at `http://10.0.0.1/` or `http://roomba.local/` without any client-side configuration.

Since the initial status snapshot, the following work has been completed:
- **Bluetooth connectivity fix:** `bluetooth_manager.py` rewritten from scratch with clean two-helper architecture: `_bt_quick()` for fast synchronous ops (info/disconnect/remove) and `_bt_wait()` for async ops (pair/trust/connect) that keep the bluetoothctl session alive while polling device info in separate sessions. Key fixes: `pair()` only accepts `Paired: yes` (not `Connected: yes`), all async ops consistently use polling, `setup_controller()` checks ERTM before starting, `get_connection_status()` returns paired/trusted/connected fields. Bluetooth UI shows paired/trusted state. 21/21 tests pass.
- `environment.sh` updated to current standard (arithmetic bug fixed, joy_linux/eventlet/pigpio verification checks added)
- `environment.sh` hardened: `--check` mode added (103-check verify-only run), pigpiod ExecStop fixed, hostapd.conf mode 600, dnsmasq.conf idempotent, socket.io sha256 check, roomba-ap-start.sh error handling
- Fixed `cap_net_bind_service` + `LD_LIBRARY_PATH` conflict: capability on python3.12 (for port 80) caused linker to ignore `LD_LIBRARY_PATH`, breaking all ROS2 C extensions — fixed via `/etc/ld.so.conf.d/ros2-jazzy.conf` ldconfig entry
- `project_requirements.md` updated from v1.6 → v2.1 with all implementation corrections from live testing
- All requirements-update items identified in Section 6 of this document have been applied
- **Stage 4b (draw-test) completed:** draw_node X-axis inversion fix, draw mode float detection fix, saved maps table (replaces dropdown), live/saved view mode with Back to Live button
- **Headless save pipeline:** db_node saves maps directly on SAVE_MAP events from controller X button, writes MapEvent rows for UI polling. Web UI polls `/api/maps/events` every 3s to auto-refresh saved maps table.
- **db_node RcutilsLogger crash fixed:** ROS2 `RcutilsLogger` does not support %-style format args — all logger calls in db_node.py converted to f-strings
- **All test files have real assertions** — 12 test files with 68 total test cases
- **Stage 4c (simulate-hw) tested:** SLAM pipeline working (sim_sensor → /scan → slam_toolbox → /map + TF), fuzzy frontier exploration covers full room, stuck recovery with back-up + goal blacklisting, web UI map page redesigned to side-by-side layout with inline mode controls
- **33 bugs fixed** — see Section 4 for full list
- **Web UI streamlined (v3.7):** Map page side-by-side layout (no scrolling), mode controls on map page, RECON amber badge, consolidated JS utilities (showToast/esc/dismissToast → common.js), RECON_COMPLETE and GOAL_FAILED event toasts

**Current stage:** Stage 5 Complete (LIDAR Bench Test — real LD14P LIDAR data flowing through SLAM toolbox to web UI map). Stage 6 (Motor Integration) next.

---

## 2. CODEBASE METRICS

| Metric | Value |
|---|---|
| Total source lines (src + tests + config + templates) | ~11,750 |
| C++ source files | 10 |
| Python source files | 8 (+ 2 `__init__.py`) |
| HTML templates | 5 (base + 4 pages) |
| JS + CSS assets | 2 files (411 lines) |
| YAML config files | 6 (401 lines) |
| Test files | 12 (68 test cases with real assertions) |
| ROS2 packages | 6 |
| Shell scripts (setup.sh + environment.sh + rebuild.sh) | 1,689 lines |
| Launch files | 1 (135 lines) |
| CMakeLists.txt (3 C++ packages) | 306 lines |

### Package Breakdown

| Package | Language | Lines | Nodes | Status |
|---|---|---|---|---|
| `roomba_hardware` | C++17 | ~1,600 | `motor_controller` (233), `sim_sensor_node` (951), `sim_motor_node` (410) | HW motor node scaffolded (untested on hardware); **sim nodes implemented & tested** — room generation, raycasting, diff-drive kinematics, collision detection |
| `roomba_control` | C++17 | 788 | `joy_control_node` (243), `bt_sim_node` (229), `draw_node` (316) | **joy_control_node tested live** with real controller; **draw_node tested** — X-axis inversion fixed, both axes negated |
| `roomba_navigation` | C++17 | ~1,220 | `recon_node` (700), `sim_goal_follower` (522) | `recon_node` — **Mamdani fuzzy inference** for frontier selection (15 rules, 3 inputs, centroid defuzz) + **goal blacklisting** on GOAL_FAILED; `sim_goal_follower` — P-control + **fuzzy obstacle avoidance** (10 rules, 2 outputs) + **stuck recovery** (2s back-up + GOAL_FAILED event) |
| `roomba_db` | Python | 368 | `db_node` (186) + `models` (181) | **Implemented** — PostgreSQL in Docker, 7 map CRUD/event endpoints, MapEvent table for headless save polling |
| `roomba_webui` | Python | 2,142 | Flask+SocketIO server: `app.py` (804) + `ros_bridge.py` (527) + `bluetooth_manager.py` (420) + `data_channels.py` (95) + `mock_data.py` (144) + `logging_config.py` (151) | **Tested live** — 13 REST endpoints, 10 WebSocket events, side-by-side map layout, inline mode controls |
| `roomba_bringup` | Python | 135 | `full_system.launch.py` | Scaffolded |

---

## 3. COMPONENT STATUS

### 3.1 Environment & Build — ✅ COMPLETE & HARDENED

- `environment.sh` — Idempotent install script with `--check` verify-only mode:
  - **`--check` mode:** Skips all installs, runs 103-check verification across 12 categories. Use on already-provisioned devices.
  - Fixed `((FAIL_COUNT++))` arithmetic bug that caused false failure with `set -e`
  - Fixed `pipefail` + `grep -q` SIGPIPE (exit 141) in check functions
  - Fixed pigpiod `ExecStop` — removed recursive `systemctl kill` call
  - `hostapd.conf` set to mode 600 (WPA passphrase not world-readable)
  - `dnsmasq.conf` write guarded with `if [[ ! -f ]]` for idempotency
  - `dnsmasq` standalone service explicitly disabled (conflicts with systemd-resolved)
  - `hostapd` unmasked after install (Ubuntu auto-masks it)
  - `roomba-ap-start.sh` validates wlan0 exists with error messages before creating ap0
  - `socket.io.min.js` download verified via sha256 integrity check
  - `cap_net_bind_service` set on `python3.12` (bind port 80 without root)
  - ROS2 library paths registered in `/etc/ld.so.conf.d/ros2-jazzy.conf` — **required** because Linux capabilities cause the dynamic linker to ignore `LD_LIBRARY_PATH`
  - Removed dead `screen` package from install list
  - Simplified `--system-site-packages` check
  - Syntax validated clean via `bash -n`
  - **103 checks, 101 PASS, 0 FAIL, 2 WARN** (hardware-dependent: pigpiod inactive, no ESP32)
- `colcon build --symlink-install` — All 6 packages build clean
- Python venv at `roomba_ws/.venv` with `--system-site-packages`
- ROS2 Jazzy sourced from `/opt/ros/jazzy/setup.bash`

### 3.2 setup.sh — ✅ COMPLETE & TESTED

Tiered startup script with 9 modes:

| Mode | Status | Tested |
|---|---|---|
| `kill` | ✅ Working | Yes — kills tmux session + processes + port 80/5000 |
| `demo` | ✅ Working | Yes |
| `web` | ✅ Working | Partial |
| `bt-test` | ✅ Working | Partial |
| `controller` | ✅ **Working** | **Yes — full live test passed** |
| `hardware` | ✅ Scaffolded | No — requires hardware |
| `full` | ✅ Scaffolded | No — requires hardware |
| `draw-test` | ✅ Working | Yes — controller drawing + save tested |
| `simulate-hw` | ✅ Working | **Yes — full sim stack tested** |
| `sensor-test` | ✅ Working | **Yes — real LD14P LIDAR → SLAM → map → web UI** |

Key features implemented:
- Auto-kills stale processes before every start
- Split `source_ros2_cmd()` vs `venv_ros2_cmd()` for C++ vs Python nodes
- Uses `python3 -m roomba_webui.app` (not `ros2 run`) for webui — avoids venv/system Python conflict
- Uses `python3 -m roomba_db.db_node` (not `ros2 run`) for db_node — avoids shebang bypassing venv
- Uses `ros2 run joy_linux joy_linux_node` (not `ros2 run joy ...`)
- `--dry-run` and `--no-kill` flags
- SIGINT/SIGTERM trap for clean shutdown
- tmux session management
- `check_slam_toolbox()` pre-flight check for `simulate-hw` and `full` modes
- `recon_node`, `sim_goal_follower`, and `joy_control_node` launch with `--params-file`

### 3.3 Xbox Controller Pipeline — ✅ COMPLETE & TESTED

| Component | Status |
|---|---|
| Xbox controller pairing (MAC: `78:86:2E:AC:13:C9`) | ✅ Paired, trusted, connected |
| xpadneo DKMS driver (v0.10-pre-292) | ✅ Loaded, kernel module active |
| bluez 5.72 | ✅ Running |
| `/dev/input/js0` | ✅ Created on connection |
| `joy_linux_node` → `/joy` at ~20 Hz | ✅ Verified |
| `joy_control_node` — axis/button mapping | ✅ Corrected for joy_linux + xpadneo |
| Edge detection (`buttonJustPressed()`) | ✅ Implemented |
| Per-axis inversion config | ✅ Working |
| Trigger normalization (1.0→-1.0 becomes 0.0→1.0) | ✅ In ros_bridge.py |
| D-pad as virtual buttons from axes[6,7] | ✅ In ros_bridge.py |

**Axis mapping (joy_linux + xpadneo):**

| Index | Axis | Rest Value |
|---|---|---|
| 0 | left_x | 0.0 |
| 1 | left_y | 0.0 |
| 2 | left_trigger | 1.0 (released) |
| 3 | right_x | 0.0 |
| 4 | right_y | 0.0 |
| 5 | right_trigger | 1.0 (released) |
| 6 | dpad_x | 0.0 |
| 7 | dpad_y | 0.0 |

**Button mapping (joy_linux + xpadneo):** 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB, 6=Select, 7=Start, 8=Xbox, 9=Left Stick, 10=Right Stick.

**Inversion flags (controller.yaml):** `left_x: true, left_y: false, right_x: true, right_y: false`

### 3.4 Web UI — ✅ COMPLETE

| Page | Route | Status |
|---|---|---|
| Dashboard | `/` | ✅ Renders, data channels functional, auto-detects draw mode, mode buttons |
| Map Viewer | `/map` | ✅ Side-by-side layout, SLAM/ground-truth canvas, sim controls, mode buttons, saved maps |
| Controller Monitor | `/controller` | ✅ Renders, live axis/button display, velocity output, connected banner |
| Bluetooth Manager | `/bluetooth` | ✅ Renders, scan/pair/connect/disconnect/setup with interactive wait, persistent toast feedback |

**Draw Mode UI (dashboard + map page):**
- Dashboard auto-detects draw canvas (100×100, res ≈0.05 with float tolerance, cursor value 50) and shows a "Draw Mode" card with canvas dimensions, drawn cell count, save button, and control hints
- Map page: "Save Map" button (prompts for name, POST `/api/maps`), color legend (Free/Wall/Unknown/Cursor swatches)
- Draw mode detection uses `Math.abs(resolution - 0.05) < 0.001` to handle float32→float64 precision loss
- `drawMap()` renders cursor value 50 as blue (`#58a6ff`), Y-axis flipped (origin at canvas bottom-left matching `drawRobot()`)
- Toast notifications for save/delete/view success/error on both pages

**Map page layout (v3.7 streamlined):**
- Side-by-side layout: live map canvas on the left, controls panel on the right — no scrolling needed
- Mode buttons (IDLE/MANUAL/RECON) in the map controls bar — switch modes while watching the map
- RECON mode has distinct amber styling (badge + mode button)
- Right panel: Robot Position, Simulation Controls, Room Info, Saved Maps — compact card layout
- Canvas responsive: scales to viewport height via `aspect-ratio: 1` + `max-width: calc(100vh - 210px)`
- Right panel scrollable independently (`overflow-y: auto`) for small viewports
- Responsive breakpoint at 900px: stacks to single column on mobile
- Recon events: RECON_COMPLETE and GOAL_FAILED events show toast notifications

**Draw Controls Reference (map page):**
- Auto-shown card with 8-row table for all Xbox controller inputs (Left Stick, RT, LT, A, X, Y, D-pad, Start)
- Card auto-hides when not in draw mode

**Saved Maps Table (map page — replaces dropdown):**
- Compact table in right panel showing Name, Size, View/Delete buttons
- Permanent "⚡ Live" row always first in the table
- "⚡ Back to Live" button and view label shown when viewing a saved map
- `viewingLive` flag pauses live canvas updates when viewing a saved map
- `savedMapData` cache prevents saved map from being overwritten by live data
- `robot_pose` handler guarded with `if (viewingLive)` to prevent overwriting saved map view

**Save Pipeline (headless-first architecture):**
- Controller X button → draw_node publishes `SAVE_MAP` on `/robot/events` → db_node saves map with date-based auto-name → writes `MapEvent(SAVED)` to `map_events` table
- Web UI "Save Map" button → POST `/api/maps` with optional name → also writes `MapEvent(SAVED)`
- Web UI polls `GET /api/maps/events?since=<id>` every 3s to detect new saves from db_node (headless) and refreshes saved maps table automatically
- Works without web UI: db_node saves directly to PostgreSQL on SAVE_MAP event with auto-generated `map_YYYYMMDD_HHMMSS_N` name
- `map_events` table: tracks SAVED/DELETED events with map_id, map_name, created_at for efficient UI polling

**Networking:**
- Port changed from 5000 to **80** (no `:5000` needed in URLs)
- `socket.io` client library bundled locally at `static/js/socket.io.min.js` (no CDN dependency)
- Accessible via WiFi hotspot at `http://10.0.0.1/` or `http://roomba.local/`
- Accessible on existing LAN via Pi's DHCP IP or `http://gabi.local/`

**Backend architecture:**
- `eventlet.monkey_patch()` MUST be first import (before Flask, SocketIO, rclpy)
- `queue.Queue(maxsize=64)` bridges rclpy OS threads → eventlet green threads (prevents deadlock)
- `emit_loop` drains event queue safely on eventlet green thread
- `bt_status_loop` polls BluetoothManager in background
- Debug endpoints: `/api/debug/channels`, `/api/debug/controller`
- `showToast()`, `dismissToast()`, `esc()` consolidated in `common.js` (removed duplicates from index/map/bluetooth templates)
- `syncModeUI()` supports RECON badge color (amber `badge-recon`)

**API endpoints:**
- `GET /api/maps` — list saved maps (200)
- `POST /api/maps` — save current map with optional name (200)
- `DELETE /api/maps/<id>` — delete a saved map (200)
- `GET /api/maps/events?since=<id>` — poll for new map save/delete events (200)
- Other POST endpoints return 501 (not yet connected to hardware)

### 3.5 ROS2 Bridge (ros_bridge.py — 527 lines) — ✅ COMPLETE

- Subscribes: `/joy`, `/robot/mode`, `/roomba/pose`, `/tf`, `/map`, `/robot/events`, `/sensors/front`, `/sensors/left`, `/sensors/right`, `/sensors/health`, `/sim/status`, `/sim/ground_truth`
- Publishes: `/robot/mode`, `/sim/command`
- **Pose transform:** Tracks `map→odom` TF from slam_toolbox and applies it to `/roomba/pose` (odom frame) to produce map-frame coordinates for accurate SLAM map display
- Thread-safe queue.Queue event passing (maxsize=64)
- Per-axis inversion from config
- Trigger normalization: `1.0 → -1.0` mapped to `0.0 → 1.0`
- D-pad axes synthesized as virtual buttons (dpad_up, dpad_down, dpad_left, dpad_right)
- Controller config loaded from `controller.yaml` with `/**:/ros__parameters:` navigation; logs load success/failure
- Sensor callbacks update `channels["sensors"]` DataChannel with real Range data from sim/hardware nodes
- `get_sensor_health()` exposes health status from `/sensors/health` topic for `emit_loop` consumption
- Sim callbacks: `_sim_status_callback()` parses JSON, `_ground_truth_callback()` stores OccupancyGrid for web UI overlay

### 3.6 DataChannel System — ✅ COMPLETE

Universal fallback-to-mock pattern:
- Each channel independently tracks `last_real_timestamp`
- `is_live()` returns True if real data received within `timeout_s`
- `get()` returns real data if live, mock data otherwise
- **Logging:** State transitions (LIVE↔MOCK) logged at INFO, first real message logged per channel
- No mode flags, no environment variables
- Configurable timeouts per channel in `config/webui.yaml`

### 3.7 Logging Infrastructure — ✅ COMPLETE

- `logging_config.py` — `setup_logging()` called in `app.py main()` before any other init
- **Console output:** INFO+ with structured timestamp/component format
- **File output:** DEBUG+ rotating files at `/tmp/roomba_logs/roomba_<timestamp>.log` (10 MB, 5 backups)
- `StructuredFormatter` — ISO8601 timestamps, component names, func/line context
- `ComponentFilter` — extracts short component name from dotted module path
- Third-party loggers suppressed: `urllib3`, `werkzeug`, `eventlet` set to WARNING
- **Coverage:** `app.py` (startup, all BT API routes, WebSocket events, emit_loop, bt_status_loop), `bluetooth_manager.py` (all operations at INFO, bluetoothctl output at DEBUG), `data_channels.py` (state transitions at INFO), `ros_bridge.py` (bridge lifecycle, mode changes, errors)
- All logger calls in webui Python code use %-style formatting (lazy evaluation for standard `logging`)
- **ROS2 Python nodes** (`db_node.py`) use f-strings for `self.get_logger()` calls — `RcutilsLogger` does not support %-style positional args

### 3.8 Governance Files — ✅ COMPLETE

- `rules.md` — 10-section binding constraints document (Documentation, Logging, Language & Architecture, Error Handling, Security, Threading & Concurrency, Testing, Web UI, Shell Scripts, Process)
- `.github/copilot-instructions.md` — Auto-loaded agent instructions with mandatory reading table, scripts table, new-node checklist, key constraints summary

### 3.9 Hardware & Navigation Nodes — ⚠️ ARCHITECTURE UPDATED (v2.4)

**Architecture change (v2.4/v2.5):** Ultrasonic sensors removed. LD14P (LD-D200) 360° LIDAR connects directly to Pi 5 via UART (`/dev/ttyAMA0`, 230400 baud). The `ldlidar_stl_ros2` external package provides the ROS2 driver node, publishing `/scan` (LaserScan). `esp32_sensor_node` and `slam_bridge_node` are no longer built. **ESP32 retained as motor coprocessor** — receives motor commands from Pi over I2C (`/dev/i2c-1`, address `0x42`) and drives H-bridge motor drivers. `pigpio` removed (motors not on Pi GPIO).

| Node | Lines | Status |
|---|---|---|
| `ldlidar_stl_ros2` (external) | — | ✅ **Cloned, built, tested on real hardware.** Publishes `/scan` from LD14P via `/dev/ttyAMA0` at 230400 baud. Launch file: `ld14p.launch.py` with `product_name='LDLiDAR_LD19'` (LD14P uses LD19 protocol). Includes static TF publisher `base_link→laser_frame`. |
| `motor_controller.cpp` | 233 | ⚠️ **Needs rewrite:** currently uses pigpio (direct Pi GPIO) — must be changed to I2C communication with ESP32 motor coprocessor. Wheel measurements TBD. |
| `recon_node.cpp` | 700 | Frontier-based exploration with Mamdani fuzzy inference (15 rules). Goal blacklisting on GOAL_FAILED. Origin pose hardcoded 0,0 — TODO: read from /odom. |
| `sim_goal_follower.cpp` | 522 | P-control + fuzzy obstacle avoidance (10 rules). Stuck recovery with 2s back-up + GOAL_FAILED event. |

### 3.10 Database Layer — ✅ IMPLEMENTED

- **PostgreSQL 16** running in Docker (postgres:16-alpine, arm64)
- `docker/docker-compose.yaml`: Container definition with health check, 256MB memory limit, named volume
- `docker/.env`: Credentials (gitignored), `docker/.env.example` template committed
- `models.py`: SQLAlchemy ORM — `MapRecord` + `SessionRecord` + `MapEvent`, QueuePool for PostgreSQL, StaticPool for SQLite (tests). Schema auto-created via `Base.metadata.create_all()` on startup.
- `db_node.py`: Subscribes to `/robot/events` (SAVE_MAP trigger — saves headlessly with date-based name), `/robot/mode` (session tracking), `/map` (caches latest OccupancyGrid). On SAVE_MAP: saves map to `maps` table and records a `MapEvent(SAVED)` in `map_events` table. Works without web UI. All logger calls use f-strings (RcutilsLogger requirement).
- **13 REST API endpoints** in `app.py`:
  - `GET /` `/map` `/controller` `/bluetooth` — 4 HTML pages
  - `GET /api/robot/status` — current mode + sensor readings
  - `POST /api/robot/mode` — set robot mode
  - `GET /api/maps` — list all maps
  - `GET /api/maps/<id>` — map metadata
  - `GET /api/maps/<id>/data` — full grid data
  - `POST /api/maps` — save current live map (also writes MapEvent)
  - `PUT /api/maps/<id>` — rename map
  - `DELETE /api/maps/<id>` — delete map (also writes MapEvent)
  - `GET /api/maps/events?since=<id>` — poll for map save/delete events
  - `GET /api/sim/status` — simulation room info
  - `POST /api/sim/command` — send sim command (whitelist-validated)
  - `GET /api/sim/ground_truth` — ground truth map data for overlay
  - `GET/POST /api/bluetooth/*` — 7 BT endpoints (devices/scan/pair/connect/disconnect/trust/remove/setup)
  - `GET /api/debug/channels` `/api/debug/controller` — 2 debug endpoints
- **10 WebSocket events** — 7 server→client (channel_status, robot_event, robot_mode, sensor_data, sensor_health, robot_pose, controller_state, map_update, bluetooth_status, sim_status) + 3 client→server (set_mode, sim_command, connect)
- **Alembic** scaffolded with initial migration (`alembic.ini`, `migrations/` directory with `0001_initial_schema.py`). Currently inactive — schema auto-created via `Base.metadata.create_all()` at runtime. Alembic may be activated for future production schema versioning.
- **setup.sh**: `ensure_db()` starts PostgreSQL container and waits for readiness before launching db_node
- **environment.sh**: Section 10 installs Docker, starts PostgreSQL; verification checks added
- **9 passing tests** in `test_db_node.py` (in-memory SQLite, no Docker needed)

### 3.11 Test Suite — ✅ ALL TESTS HAVE REAL ASSERTIONS

12 test files with 68 total test cases (1,417 lines):

| Test File | Type | Lines | Tests | Content |
|---|---|---|---|---|
| `test_bt_sim_node.cpp` | GTest | 84 | 4 TEST_F | Deadzone, waveforms, dimensions |
| `test_draw_node.cpp` | GTest | 103 | 4 TEST_F | Grid layout, paint, clear, cursor clamping |
| `test_esp32_sensor_node.cpp` | GTest | 104 | 4 TEST_F | *(No longer built — ESP32 removed in v2.4)* |
| `test_joy_control_node.cpp` | GTest | 80 | 4 TEST_F | Velocity scaling, bounds checking |
| `test_motor_controller.cpp` | GTest | 84 | 4 TEST_F | Differential drive kinematics |
| `test_recon_node.cpp` | GTest | 97 | 3 TEST_F | Frontier detection, radius constraints |
| `test_sim_goal_follower.cpp` | GTest | 56 | 4 TEST_F | P-control, fuzzy avoidance, stuck timeout |
| `test_sim_motor_node.cpp` | GTest | 160 | 4 TEST_F | Straight-line kinematics, rotation, collision, theta normalisation |
| `test_sim_sensor_node.cpp` | GTest | 192 | 4 TEST_F | Raycast wall hit, max range, room connectivity (BFS), obstacle blocking |
| `test_slam_bridge_node.cpp` | GTest | 67 | 3 TEST_F | *(No longer built — slam_bridge removed in v2.4)* |
| `test_db_node.py` | pytest | 179 | 9 tests | Models, CRUD, relationships, cascade |
| `test_roomba_webui.py` | pytest | 211 | 21 tests | DataChannel fallback, mock data, Flask routes, WebSocket events, BT manager |

### 3.12 bt_sim_node — ✅ SCAFFOLDED

- 3 profiles: idle, sine, patrol (all working in simulation)
- Publishes `/joy` at 50 Hz, `/cmd_vel` at 20 Hz, `/bt_sim/status` at 1 Hz
- **Not implemented:** `BtSimInject.srv` custom service for test injection

### 3.13 draw_node — ✅ TESTED

Interactive OccupancyGrid drawing canvas for testing the DB persistence pipeline end-to-end.

- Subscribes to `/joy`, publishes OccupancyGrid on `/map` (5 Hz default), publishes `SAVE_MAP` on `/robot/events`
- Controls: left stick = move cursor, RT = draw (100), LT = erase (0), A = stamp, X = save, Y = clear, D-pad up/down = brush size (1–10), Start = center cursor
- Grid: 100×100 @ 0.05 m (5 m × 5 m canvas), all cells start as −1 (unknown)
- Cursor crosshair rendered as value 50 in published grid
- **X-axis inversion fix:** Both axes negated (`joy_lx_ = -axis(ax_lx_)`, `joy_ly_ = -axis(ax_ly_)`) — xpadneo reports right-as-negative for axis 0
- Parameters loaded from `controller.yaml` `draw:` section (wrapped in `/**:/ros__parameters:` for ROS2 `--params-file`)
- `setup.sh draw-test` mode: joy_linux_node → draw_node → db_node → webui
- 4 GTest unit tests in `test_draw_node.cpp`

### 3.15 Simulation Nodes — ✅ IMPLEMENTED

Drop-in replacements for `esp32_sensor_node` and `motor_controller` that simulate a random room with obstacles, enabling closed-loop testing of pathfinding, SLAM, and recon without physical hardware.

#### `sim_motor_node` (C++17)
- Subscribes `/cmd_vel` → integrates differential-drive kinematics → publishes `/odom` (Odometry @ 50 Hz) + `/roomba/pose` (PoseStamped) + TF `odom→base_link`
- Collision detection against ground-truth map prevents driving through walls (rotation still allowed), with collision counter
- Configurable Gaussian noise on odometry (simulates encoder drift)
- Hardware watchdog: stops motors if no `/cmd_vel` within timeout, publishes `WATCHDOG_STOP` event
- **Sim commands** via `/sim/command`: `reset_pose` (teleport to spawn), `pause`, `resume`
- 4 GTest unit tests: straight-line kinematics, rotation, collision, theta normalisation

#### `sim_sensor_node` (C++17)
- Generates random room at startup: outer walls + 1–3 internal partitions with doorways + 4–8 rectangular obstacles
- BFS flood-fill validates reachability from spawn; clears unreachable obstacles
- Subscribes `/odom` → raycasts sensor readings from robot pose
- Publishes `/sensors/{front,left,right}` (Range @ 10 Hz) with Gaussian noise — sim retains 3 ultrasonic topics for `sim_goal_follower` fuzzy avoidance
- Publishes `/scan` (LaserScan, 360 beams @ 10 Hz) — simulated LIDAR for `slam_toolbox`
- Publishes `/sim/ground_truth` (OccupancyGrid @ 0.5 Hz) — full revealed room for debug display
- Publishes `/sim/discovered` (OccupancyGrid @ 2 Hz) — fog-of-war debug view
- Publishes `/sim/status` (String/JSON @ 1 Hz) — room dimensions, grid size, obstacle/partition counts, free/wall cells, seed
- Publishes `/sensors/health` — all OK in simulation
- **Sim commands** via `/sim/command`: `regenerate` (new random room + reset fog-of-war), `regenerate:seed=<N>` (reproducible room)
- 4 GTest unit tests: raycast wall hit, max range, room connectivity, obstacle blocking

#### Web UI Simulation Features
- **Map page — Simulation Controls card** (auto-shown when sim_status received):
  - New Map button — regenerates random room
  - Reset Pose button — teleports robot to spawn
  - Pause/Resume button — freezes/unfreezes physics
  - Seed input — enter seed for reproducible room generation
- **Map page — Simulation Info card**: room size, grid dimensions, partition count, obstacle count, free/wall cell counts, current seed
- **Map page — Ground Truth toggle**: switch between fog-of-war discovered map and full revealed ground truth map for comparison
- **API endpoints**: `GET /api/sim/status`, `POST /api/sim/command`, `GET /api/sim/ground_truth`
- **WebSocket events**: `sim_status` (1 Hz), `sim_command` (from client)
- **Event toasts**: SIM_MAP_REGENERATED, SIM_POSE_RESET events shown as notifications

#### `config/simulation.yaml`
- Flat ROS2 parameter namespace (`/**:/ros__parameters:`) — compatible with `--params-file`
- Room generation: dimensions, resolution, wall thickness, seed (reproducible runs), partition/obstacle counts and sizes
- Spawn position and orientation
- Wheel geometry, max velocities, odom noise, collision radius
- Ultrasonic and LIDAR parameters (beam count, ranges, noise)
- **Fuzzy MF parameters**: 9 trapezoidal membership functions for recon frontier selection (distance/size/alignment) + 6 for obstacle avoidance (front/side distances) — all tunable via YAML

#### `setup.sh simulate-hw` mode
- Launches: `sim_motor_node` → `sim_sensor_node` → `slam_toolbox` → `sim_goal_follower` → `recon_node` → `joy_linux_node` → `joy_control_node` → `db_node` → `webui`
- **SLAM pipeline:** `sim_sensor_node` → `/scan` → `slam_toolbox` → `/map` + TF `map→odom` — identical path used on real hardware
- **Fuzzy recon:** `recon_node` clusters frontiers and uses Mamdani fuzzy inference (distance × size × heading alignment → desirability) to select the best exploration target
- **Fuzzy obstacle avoidance:** `sim_goal_follower` subscribes `/sensors/{front,left,right}` and applies fuzzy speed/turn modifiers on top of proportional goal tracking
- Full closed-loop: controller → cmd_vel → sim kinematics → odom → raycasting → sensors/scan → slam_toolbox → /map → fuzzy recon → goal → fuzzy goal_follower → cmd_vel
- Compatible with DB saves, map events, web UI visualisation

### 3.16 WiFi Access Point & Networking — ✅ COMPLETE

| Component | Status |
|---|---|
| hostapd (WiFi AP daemon) | ✅ Active, config at `/etc/hostapd/hostapd.conf` (mode 600) |
| Virtual interface `ap0` | ✅ Created from wlan0 via `iw`, concurrent AP+STA |
| SSID: `Roomba`, WPA2, password: `roomba123` | ✅ Configured, channel 1 |
| dnsmasq (DNS + DHCP) | ✅ Active, config at `/etc/dnsmasq.d/roomba.conf` |
| DHCP range: 10.0.0.10–10.0.0.50 (ap0 only) | ✅ Working |
| DNS: `roomba.local` → 10.0.0.1 / Pi LAN IP | ✅ Both interfaces |
| `roomba-ap.service` (systemd) | ✅ Enabled, starts on boot |
| Concurrent AP+STA (ap0 + wlan0) | ✅ Verified — internet access preserved on wlan0 |
| Web UI on port 80 | ✅ Accessible from both networks |

**Access paths (all verified 200):**

| Client connects via | URL |
|---|---|
| Roomba hotspot | `http://10.0.0.1/` or `http://roomba.local/` |
| Existing WiFi (LAN) | `http://<Pi-LAN-IP>/` or `http://gabi.local/` (mDNS, non-Android) |

---

## 4. CRITICAL BUGS FIXED DURING DEVELOPMENT

| # | Bug | Root Cause | Fix |
|---|---|---|---|
| 1 | **WebSocket deadlock** — A/B/X/Y buttons froze entire webui | `socketio.emit()` called from rclpy OS thread into eventlet | `queue.Queue` bridge + `drain_events()` in `emit_loop` |
| 2 | **Sticks mirrored** — left↑ showed right↑ | SDL2 axis indices used instead of joy_linux+xpadneo | Corrected controller.yaml indices |
| 3 | **Button rapid-fire** — single press fired multiple actions | No edge detection on button callbacks | `buttonJustPressed()` with `prev_buttons_` vector |
| 4 | **Duplicate webui processes** on port 5000 | Manual starts without cleanup | `setup.sh kill` mode + auto-kill on every start |
| 5 | **joy_linux_node "No executable found"** | venv Python path pollutes ros2 run discovery | `source_ros2_cmd()` deactivates venv for C++ nodes |
| 6 | **webui "No module named eventlet"** | `ros2 run` uses system Python, ignoring venv | Use `python3 -m roomba_webui.app` directly |
| 7 | **Wrong package name** `ros2 run joy joy_linux_node` | Package is `joy_linux`, not `joy` | `ros2 run joy_linux joy_linux_node` |
| 8 | **tmux `source` not found** | tmux uses `/bin/sh` by default | `bash -c` wrapper in `start_in_tmux()` → then switched to `send-keys` |
| 9 | **`((killed++))` exit code 1** with `set -e` | Bash arithmetic returns 1 when result is 0→1 | `killed=$((killed + 1))` |
| 10 | **SDL2 haptic error** on joy_node startup | SDL2 joy_node tries force feedback on xpadneo | Switched to `joy_linux_node` (evdev-based, no haptic) |
| 11 | **BT scan used two processes** — discovered devices lost | `subprocess.run` followed by separate `devices` session | Single `Popen` session: scan on → wait → devices → exit |
| 12 | **BT pairing needed agent** — Xbox controllers rejected | No `agent on`/`default-agent` before pair | Added agent commands to pair and setup_controller |
| 13 | **MAC command injection** via BT API | No validation on MAC address input | `_MAC_RE` regex validator on all BT endpoints |
| 14 | **bt_status_loop bare `except: pass`** — errors invisible | Swallowed all exceptions silently | Replaced with `logger.debug()` for non-critical error |
| 15 | **setup_logging() never called** — file logging dead code | `logging_config.py` existed but was never imported/called | Wired `setup_logging()` into `app.py main()` |
| 16 | **Draw mode not detected** — `data.resolution === 0.05` always false | ROS2 float32→float64 conversion produces `~0.05000000074505806`; strict equality fails | `Math.abs(data.resolution - 0.05) < 0.001` on both index.html and map.html |
| 17 | **Draw cursor X-axis inverted** — moving right went left on map | xpadneo reports right-as-negative for axis 0; `joy_lx_` was not negated | `joy_lx_ = -axis(ax_lx_)` in draw_node.cpp (both axes now negated) |
| 18 | **db_node crash on launch** — `ModuleNotFoundError: sqlalchemy` | `ros2 run` uses shebang `#!/usr/bin/python3` (system Python), bypassing venv | `python3 -m roomba_db.db_node` in setup.sh launch_db_node |
| 19 | **controller.yaml not loading** for draw_node | ROS2 `--params-file` requires `/**:/ros__parameters:` wrapper; file had bare top-level keys | Wrapped YAML in `/**:\n  ros__parameters:`, updated ros_bridge.py to navigate into wrapper |
| 20 | **Saved map overwritten by live data** — canvas flicked back to live | `robot_pose` handler unconditionally called `drawMap(lastMapData)` | Added `if (viewingLive)` guard on robot_pose and map_update handlers |
| 21 | **Demo mode launch crash** — `python3 -m roomba_webui` not a runnable module | setup.sh used `python3 -m roomba_webui` instead of `python3 -m roomba_webui.app` | Fixed to `python3 -m roomba_webui.app` in `launch_webui_demo()` |
| 22 | **db_node crash on first /map message** — `RcutilsLogger.debug() takes 2 positional arguments but 5 were given` | ROS2 `RcutilsLogger` does not support Python %-style format strings (`self.get_logger().debug("msg %d", val)`) — crashes at runtime | Converted all `self.get_logger()` calls in db_node.py from %-style to f-strings |
| 23 | **ros_bridge missing sensor subscriptions** — dashboard always showed mock sensor data | `ros_bridge.py` subscribed to `/joy`, `/robot/mode`, `/roomba/pose`, `/map`, `/robot/events` but **not** `/sensors/{front,left,right}` or `/sensors/health` — the "sensors" DataChannel never received real data from sim or hardware nodes | Added 4 subscriptions (`/sensors/front`, `/sensors/left`, `/sensors/right`, `/sensors/health`) with callbacks that update `channels["sensors"]` and a `_sensor_health` dict |
| 24 | **emit_loop hardcoded mock sensor health** — health status never reflected real data | `emit_loop` in `app.py` always called `mock_data.mock_sensor_health()` directly instead of checking for real data from `/sensors/health` | Changed to `ros_bridge.get_sensor_health()` with mock fallback when None |
| 25 | **3 nodes launched without --params-file** — violated config-over-constants rule | `launch_recon_node()`, `launch_sim_goal_follower()`, `launch_joy_control_node()` in setup.sh launched nodes without `--params-file`, so all parameters fell back to code defaults | Added `--params-file` with correct YAML paths to all 3 launch functions |
| 26 | **No slam_toolbox pre-flight check** — simulate-hw failed silently without SLAM | If `slam_toolbox` package is not installed, `ros2 run slam_toolbox` fails silently inside tmux — no `/map` published, web UI shows mock data with no error | Added `check_slam_toolbox()` function and wired into `simulate-hw` and `full` mode pre-flight checks |
| 27 | **slam_toolbox params never loaded** — wrong launch argument name | `setup.sh` passed `params_file:=` but `online_async_launch.py` expects `slam_params_file:=` — slam_toolbox used default config with `base_frame: base_footprint` instead of `base_link`, causing "Failed to compute odom pose" on every scan | Changed to `slam_params_file:=` in both `launch_slam_toolbox()` and `launch_slam()` |
| 28 | **Mode flashes IDLE→RECON on page refresh** | `on_connect()` sent no initial state; HTML had hardcoded `active` class on IDLE button — browser showed IDLE until 1Hz emit_loop sync arrived | `on_connect()` now sends `robot_mode` + `map_update` immediately; removed hardcoded `active` from IDLE button |
| 29 | **Robot position wrong on SLAM map** — coordinate frame mismatch | `sim_motor_node` published `/roomba/pose` with `frame_id: map` but coordinates were in odom frame; `drawRobot()` subtracted SLAM map origin from odom-frame coords | ros_bridge now subscribes to `/tf`, tracks `map→odom` transform, and applies it to `/roomba/pose` before sending to UI; sim_motor frame_id fixed to `odom` |
| 30 | **drawRobot() used wrong map data for ground truth** | `drawRobot()` always used `lastMapData` (SLAM map) for coordinate conversion, even when ground truth view active — origin mismatch | `drawRobot()` now uses `groundTruthData` when ground truth toggle is active |
| 31 | **Robot icon Y-axis inverted on map** | `drawMap()` rendered row y=0 at canvas top (no Y-flip) but `drawRobot()` applied `canvas.height - ...` Y-flip — robot appeared vertically mirrored relative to map features | `drawMap()` now flips Y: `ctx.fillRect(x*cw, canvas.height-(y+1)*ch, ...)` so map origin is at canvas bottom-left, matching `drawRobot()` |
| 32 | **Exploration stops too early** — robot maps only part of room | `recon_radius: 5.0` in simulation.yaml too small for 10×10 room; frontiers beyond 5m from origin filtered out | Increased `recon_radius` to 15.0m (covers full room diagonal) |
| 33 | **Robot gets stuck at walls indefinitely** | `sim_goal_follower` reset `goal_time_` on every `/goal_pose` message (even duplicate goals), preventing stuck timeout; no recovery behavior | Only reset timeout on changed goals (>0.1m); added 2s back-up recovery + `GOAL_FAILED` event; recon_node blacklists failed goals for 30s |
| 34 | **SLAM toolbox crash — symbol lookup error then segfault** | `libfastcdr.so` version mismatch after partial ROS2 updates; 335+ packages outdated with incompatible library versions | `sudo dpkg --configure -a` (with `DEBIAN_FRONTEND=noninteractive` to avoid docker debconf blocking) followed by full `apt upgrade` of all `ros-jazzy-*` packages |
| 35 | **SLAM map never updates while robot stationary** | `slam_params.yaml` had `minimum_travel_distance: 0.3` and `minimum_travel_heading: 0.5` — SLAM ignores new scans until robot moves | Set both to `0.0` for `sensor-test` mode (stationary LIDAR bench testing). Should be bumped to small values (0.1/0.2) once driving |

---

## 5. KNOWN LIMITATIONS & TECH DEBT

| # | Item | Severity | Notes |
|---|---|---|---|
| 1 | ~~All C++ node implementations are scaffold-level~~ | ~~Medium~~ | ✅ **PARTIALLY RESOLVED** — sim nodes (`sim_sensor_node`, `sim_motor_node`, `sim_goal_follower`) and `recon_node` are fully implemented and tested in simulation. `motor_controller` remains scaffolded pending rewrite for ESP32 I2C communication (currently uses pigpio GPIO which is wrong architecture). ESP32 sensor role removed in v2.4 (LIDAR direct to Pi); ESP32 retained for motors. |
| 2 | ~~Test skeletons contain no real assertions~~ | ~~High~~ | ✅ **RESOLVED** — All 11 test files now have real assertions (49 total test cases across C++ GTest and Python pytest) |
| 3 | ~~DB services not wired to webui API~~ | ~~Medium~~ | ✅ **RESOLVED** — Map CRUD endpoints (GET/POST/DELETE) fully working via web UI. Save pipeline: controller X → SAVE_MAP event → web UI name prompt → POST /api/maps → PostgreSQL |
| 4 | recon_node uses hardcoded origin (0,0) | Medium | Should read from /odom or /amcl_pose |
| 5 | ~~motor_controller uses `gpioInitialise()` directly~~ | ~~Low~~ | ✅ **SUPERSEDED** — pigpio removed; motor_controller must be rewritten to use I2C communication with ESP32 motor coprocessor (see #16) |
| 6 | BtSimInject.srv not generated | Low | Custom service for test injection not built yet |
| 7 | No cmd_vel_mux implemented | Medium | Requirements say only one source of cmd_vel should be active |
| 8 | Wheel measurements are placeholder values | Medium | `wheel_separation_m: 0.20`, `wheel_radius_m: 0.033` need real measurements |
| 9 | ~~requirements.md still references "ROS2 `joy` node" generically~~ | ~~Low~~ | ✅ **RESOLVED** — requirements.md v1.7 now mandates `joy_linux` throughout |
| 10 | ~~environment.sh pigpio section says `libpigpio-dev` in requirements but builds from source in script~~ | ~~Low~~ | ✅ **RESOLVED** — pigpio removed entirely; environment.sh Section 3 now sets up I2C for ESP32 motor coprocessor |
| 11 | AP hotspot IP (10.0.0.1) is hardcoded in start script | Low | Works fine for single-robot use case |
| 12 | ~~dnsmasq upstream DNS (router 172.31.225.213) is hardcoded~~ | ~~Low~~ | ✅ **RESOLVED** — environment.sh now auto-detects gateway via `ip route` with fallback |
| 13 | Android mDNS limitation: `gabi.local` via avahi not resolved by Android Chrome | Info | Hotspot with dnsmasq is the workaround — `roomba.local` works on hotspot for all devices |
| 14 | Simulation raycasting is 2D only | Low | No multi-floor or ramp support — adequate for single-storey room mapping |
| 15 | Sim room generation is rectangular only | Low | Real rooms may have L-shapes; partitions approximate room complexity |
| 16 | motor_controller.cpp needs rewrite for ESP32 I2C | High | Currently uses pigpio (direct Pi GPIO PWM). Must be rewritten to send motor commands to ESP32 coprocessor over I2C (`/dev/i2c-1`, address `0x42`). CMakeLists.txt link against pigpio must be replaced with I2C library. |

---

## 6. REQUIREMENTS.MD — ✅ ALL UPDATES APPLIED

All items identified below were applied to `project_requirements.md` (now v2.1). No further requirements updates are pending.

| # | Item | Status |
|---|---|---|
| 6.1 | Joy package name → `joy_linux` throughout | ✅ Applied |
| 6.2 | D-Pad mapping → axes 6,7 (not buttons) | ✅ Applied |
| 6.3 | Trigger rest values → 1.0 released, -1.0 pressed, normalised | ✅ Applied |
| 6.4 | Axis count → 8 axes, 11 buttons, full mapping tables | ✅ Applied |
| 6.5 | setup.sh modes → 7 modes (kill/bt-test/controller added) | ✅ Applied |
| 6.6 | Venv/ROS2 interaction → documented in Section 10 | ✅ Applied |
| 6.7 | pigpio installation → build from source (not apt) | ✅ Applied |
| 6.8 | eventlet threading model → binding constraint in Section 3.5 | ✅ Applied |
| 6.9 | Revision table → v1.7 entry added | ✅ Applied |
| 6.10 | WiFi AP + networking → v1.8 (port 80, hostapd, dnsmasq, socket.io local bundle) | ✅ Applied |
| 6.11 | environment.sh hardening + `--check` mode → v1.9 (100 checks, security fixes, idempotency fixes) | ✅ Applied |
| 6.12 | `cap_net_bind_service` + ROS2 ldconfig fix → v2.0 (3 new checks, 103 total) | ✅ Applied |
| 6.13 | Headless save pipeline, map_events schema, /api/maps/events, debug endpoints, BT remove/setup, controller_state xbox button, bluetooth_status field name, Alembic status, RcutilsLogger constraint → v2.1 | ✅ Applied |
| 6.14 | LD14P wire colours corrected (non-standard), `sensor-test` mode added to component map and mode tables, development stages updated, `odom→base_link` static TF row added → v2.5 | ✅ Applied |

---

## 7. DEVELOPMENT ROADMAP (NEXT STEPS)

### Stage 4c — Simulated Hardware Testing ← ✅ COMPLETE
1. ~~Run `./setup.sh simulate-hw` and verify all 8 nodes launch~~ ✅ Done
2. ~~Verify `/sim/ground_truth` publishes random room OccupancyGrid~~ ✅ Done
3. ~~Verify `/sensors/*` and `/scan` produce simulated readings~~ ✅ Done
4. ~~Test slam_toolbox builds map from simulated LaserScan~~ ✅ Done
5. ~~Test recon_node frontier exploration in simulated environment~~ ✅ Done — fuzzy frontier selection + blacklist working
6. Develop and tune pathfinding algorithm using sim data
7. ~~Test fuzzy logic recon behaviour with simulated obstacles~~ ✅ Done — stuck recovery + goal blacklisting

### Stage 5 — LIDAR Bench Test ← ✅ COMPLETE
1. ~~Wire LD14P to Pi 5~~ ✅ Done — **wire colours are non-standard:** Black=VCC(5V)→Pin 4, Green=GND→Pin 6, White=TX(data)→Pin 10 (GPIO15 UART0 RXD), Red=RX(unused, leave disconnected)
2. ~~Run `environment.sh` to enable UART and disable serial console~~ ✅ Done
3. ~~Clone `ldlidar_stl_ros2` into `roomba_ws/src/` and rebuild~~ ✅ Done
4. ~~Test LIDAR data on serial port~~ ✅ Done — `stty -F /dev/ttyAMA0 230400 raw` confirms data flow
5. ~~Verify `/scan` topic publishing real LaserScan data~~ ✅ Done — ~6 Hz, ~660-670 range readings per scan
6. ~~Test SLAM toolbox with real LIDAR data~~ ✅ Done — Ceres solver, online async mode, produces 86×57 occupancy grid at 0.05m resolution
7. ~~Verify web UI displays live map~~ ✅ Done — `/map` channel switches to LIVE, map rendered in browser

**Key finding:** LD14P wire colours are counterintuitive and do NOT follow typical conventions (Black≠GND, Red≠VCC). The correct mapping was confirmed by hardware testing.

**ROS2 package upgrade required:** SLAM toolbox initially crashed with `symbol lookup error` (missing `serialize(unsigned int)` in `libfastcdr.so`) then segfaulted after partial fix. Root cause was library version mismatches across 335+ outdated packages. Fixed by `sudo dpkg --configure -a` and full `apt upgrade` of all `ros-jazzy-*` packages.

**`sensor-test` mode added to `setup.sh`:** Launches `ldlidar_stl_ros2` → static odom→base_link TF → `slam_toolbox` → `db_node` → web UI. Minimal pipeline for LIDAR-only testing without motors.

### Stage 6 — Motor Integration ← **NEXT**
1. Wire motor driver (L298N/DRV8833) to Pi GPIO
2. Measure actual wheel separation and radius → update hardware.yaml
3. Test `motor_controller` with manual cmd_vel
4. Verify watchdog timeout stops motors

### Stage 7 — SLAM & Navigation
1. Test slam_toolbox with real LIDAR data
2. Test recon_node frontier exploration in real environment
3. Validate radius constraint
4. End-to-end full mode test

### Stage 8 — Full Integration
1. Test `./setup.sh full` mode
2. Implement cmd_vel_mux (prevent dual-source publishing)
3. Write additional test assertions where needed
4. Activate Alembic migrations for production schema versioning

### Stage 9 — Hardening
1. Add respawn logic to launch files
2. Implement battery monitoring (if ESP32 ADC available)
3. Performance profiling under sustained operation
4. Error recovery and fault tolerance testing

---

*Generated from project analysis on 2026-03-04 · Last updated 2026-04-20 (v3.9 — Stage 5 LIDAR bench test complete: real LD14P data → SLAM → map → web UI; sensor-test mode added; LD14P wire colour documentation corrected; ROS2 package upgrade resolved SLAM crashes; slam_params.yaml tuned for stationary testing)*
