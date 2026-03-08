# PROJECT STATUS — AUTONOMOUS MAPPING ROBOT
### Codename: `roomba`
**Date:** 2026-03-04 (last updated)  
**Platform:** Raspberry Pi 5 · Ubuntu Server 24.04 LTS (ARM64) · Kernel 6.8.0-1047-raspi  
**ROS2:** Jazzy Jalisco · Python 3.12 · C++17  

---

## 1. EXECUTIVE SUMMARY

The project has completed **environment setup**, **full workspace scaffolding**, **web UI implementation**, **real Xbox controller pipeline testing**, a **full documentation audit**, and **WiFi Access Point networking**. All 6 ROS2 packages build successfully. The controller pipeline (joy_linux_node → joy_control_node → webui) has been tested live with a real Xbox controller connected via Bluetooth. The web UI serves all 4 pages on port 80 with mock/real data fallback working. The Pi runs a WiFi hotspot (SSID `Roomba`) via hostapd+dnsmasq concurrently with its existing WiFi connection, so clients can access the UI at `http://10.0.0.1/` or `http://roomba.local/` without any client-side configuration.

Since the initial status snapshot, the following documentation work has been completed:
- `environment.sh` updated to current standard (arithmetic bug fixed, joy_linux/eventlet/pigpio verification checks added)
- `environment.sh` hardened: `--check` mode added (103-check verify-only run), pigpiod ExecStop fixed, hostapd.conf mode 600, dnsmasq.conf idempotent, socket.io sha256 check, roomba-ap-start.sh error handling
- Fixed `cap_net_bind_service` + `LD_LIBRARY_PATH` conflict: capability on python3.12 (for port 80) caused linker to ignore `LD_LIBRARY_PATH`, breaking all ROS2 C extensions — fixed via `/etc/ld.so.conf.d/ros2-jazzy.conf` ldconfig entry
- `project_requirements.md` updated from v1.6 → v2.0 with all implementation corrections from live testing
- All requirements-update items identified in Section 6 of this document have been applied

**Current stage:** Stage 4 Complete + Networking Complete — Ready for Stage 5 (Sensor Bench Test).

---

## 2. CODEBASE METRICS

| Metric | Value |
|---|---|
| Total source lines (src + tests + config + templates) | 4,747 |
| C++ source files | 6 |
| Python source files | 13 |
| HTML templates | 5 (base + 4 pages) |
| YAML config files | 5 |
| Test skeletons | 8 |
| ROS2 packages | 6 |

### Package Breakdown

| Package | Language | Nodes | Status |
|---|---|---|---|
| `roomba_hardware` | C++17 | `esp32_sensor_node`, `motor_controller` | Scaffolded, builds, untested on hardware |
| `roomba_control` | C++17 | `joy_control_node`, `bt_sim_node` | **joy_control_node tested live** with real controller |
| `roomba_navigation` | C++17 | `slam_bridge_node`, `recon_node` | Scaffolded, builds, untested |
| `roomba_db` | Python | `db_node` | Scaffolded, builds, DB services not yet wired to webui |
| `roomba_webui` | Python | Flask+SocketIO server | **Tested live** — all routes 200, WebSocket bridge working |
| `roomba_bringup` | Python | Launch files | Scaffolded |

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

Tiered startup script with 7 modes:

| Mode | Status | Tested |
|---|---|---|
| `kill` | ✅ Working | Yes — kills tmux session + processes + port 80/5000 |
| `demo` | ✅ Working | Yes |
| `web` | ✅ Working | Partial |
| `bt-test` | ✅ Working | Partial |
| `controller` | ✅ **Working** | **Yes — full live test passed** |
| `hardware` | ✅ Scaffolded | No — requires hardware |
| `full` | ✅ Scaffolded | No — requires hardware |

Key features implemented:
- Auto-kills stale processes before every start
- Split `source_ros2_cmd()` vs `venv_ros2_cmd()` for C++ vs Python nodes
- Uses `python3 -m roomba_webui.app` (not `ros2 run`) for webui — avoids venv/system Python conflict
- Uses `ros2 run joy_linux joy_linux_node` (not `ros2 run joy ...`)
- `--dry-run` and `--no-kill` flags
- SIGINT/SIGTERM trap for clean shutdown
- tmux session management

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
| Dashboard | `/` | ✅ Renders, data channels functional |
| Map Viewer | `/map` | ✅ Renders, canvas + mock OccupancyGrid |
| Controller Monitor | `/controller` | ✅ Renders, live axis/button display, connected banner |
| Bluetooth Manager | `/bluetooth` | ✅ Renders, scan/pair/connect/disconnect buttons |

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

**API endpoints:** All return 200 for GET, some POST endpoints return 501 (not yet connected to hardware/DB)

### 3.5 ROS2 Bridge (ros_bridge.py) — ✅ COMPLETE

- Subscribes: `/joy`, `/robot/mode`, `/roomba/pose`, `/map`, `/robot/events`
- Publishes: `/robot/mode`
- Thread-safe queue.Queue event passing (maxsize=64)
- Per-axis inversion from config
- Trigger normalization: `1.0 → -1.0` mapped to `0.0 → 1.0`
- D-pad axes synthesized as virtual buttons (dpad_up, dpad_down, dpad_left, dpad_right)

### 3.6 DataChannel System — ✅ COMPLETE

Universal fallback-to-mock pattern:
- Each channel independently tracks `last_real_timestamp`
- `is_live()` returns True if real data received within `timeout_s`
- `get()` returns real data if live, mock data otherwise
- No mode flags, no environment variables
- Configurable timeouts per channel in `config/webui.yaml`

### 3.7 Hardware Nodes — ⚠️ SCAFFOLDED ONLY

| Node | Lines | Key TODOs |
|---|---|---|
| `esp32_sensor_node.cpp` | 267 | Needs real ESP32 + I2C bus. Firmware handshake implemented. |
| `motor_controller.cpp` | 233 | Uses pigpio directly — TODO comment: consider pigpiod_if2 for multi-process. Wheel measurements TBD. |
| `slam_bridge_node.cpp` | 146 | Converts 3× Range → synthesized LaserScan. SLAM quality warning logged. |
| `recon_node.cpp` | 281 | Frontier-based exploration. Origin pose is hardcoded 0,0 — TODO: read from /odom. |

### 3.8 Database Layer — ⚠️ PARTIAL

- `models.py` (108 lines): SQLAlchemy ORM with `MapRecord` and `SessionRecord` tables
- `db_node.py` (121 lines): Subscribes to `/robot/events` and `/robot/mode`, records sessions
- **Not yet implemented:** `/db/save_map`, `/db/load_map`, `/db/list_maps`, `/db/delete_map` services (currently topic-based with placeholder logic)
- **Not wired to webui:** Map API endpoints (`/api/maps/*`) return 501

### 3.9 Test Suite — ⚠️ SKELETONS ONLY

8 test files exist as skeletons. None contain real test logic:

| Test File | Type |
|---|---|
| `test_bt_sim_node.cpp` | GTest skeleton |
| `test_joy_control_node.cpp` | GTest skeleton |
| `test_motor_controller.cpp` | GTest skeleton |
| `test_esp32_sensor_node.cpp` | GTest skeleton |
| `test_slam_bridge_node.cpp` | GTest skeleton |
| `test_recon_node.cpp` | GTest skeleton |
| `test_db_node.py` | pytest skeleton |
| `test_roomba_webui.py` | pytest skeleton |

### 3.10 bt_sim_node — ✅ SCAFFOLDED

- 3 profiles: idle, sine, patrol (all working in simulation)
- Publishes `/joy` at 50 Hz, `/cmd_vel` at 20 Hz, `/bt_sim/status` at 1 Hz
- **Not implemented:** `BtSimInject.srv` custom service for test injection

### 3.11 WiFi Access Point & Networking — ✅ COMPLETE

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

---

## 5. KNOWN LIMITATIONS & TECH DEBT

| # | Item | Severity | Notes |
|---|---|---|---|
| 1 | All C++ node implementations are scaffold-level | Medium | Code compiles and structure follows spec, but no real hardware testing |
| 2 | Test skeletons contain no real assertions | High | Tests exist but have zero coverage |
| 3 | DB services not wired to webui API | Medium | Map CRUD endpoints return 501 |
| 4 | recon_node uses hardcoded origin (0,0) | Medium | Should read from /odom or /amcl_pose |
| 5 | motor_controller uses `gpioInitialise()` directly | Low | Should use pigpiod daemon interface for multi-process safety |
| 6 | BtSimInject.srv not generated | Low | Custom service for test injection not built yet |
| 7 | No cmd_vel_mux implemented | Medium | Requirements say only one source of cmd_vel should be active |
| 8 | Wheel measurements are placeholder values | Medium | `wheel_separation_m: 0.20`, `wheel_radius_m: 0.033` need real measurements |
| 9 | ~~requirements.md still references "ROS2 `joy` node" generically~~ | ~~Low~~ | ✅ **RESOLVED** — requirements.md v1.7 now mandates `joy_linux` throughout |
| 10 | ~~environment.sh pigpio section says `libpigpio-dev` in requirements but builds from source in script~~ | ~~Low~~ | ✅ **RESOLVED** — both environment.sh and requirements.md now document build-from-source |
| 11 | AP hotspot IP (10.0.0.1) is hardcoded in start script | Low | Works fine for single-robot use case |
| 12 | ~~dnsmasq upstream DNS (router 172.31.225.213) is hardcoded~~ | ~~Low~~ | ✅ **RESOLVED** — environment.sh now auto-detects gateway via `ip route` with fallback |
| 13 | Android mDNS limitation: `gabi.local` via avahi not resolved by Android Chrome | Info | Hotspot with dnsmasq is the workaround — `roomba.local` works on hotspot for all devices |

---

## 6. REQUIREMENTS.MD — ✅ ALL UPDATES APPLIED

All items identified below were applied to `project_requirements.md` (now v1.9). No further requirements updates are pending.

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

---

## 7. DEVELOPMENT ROADMAP (NEXT STEPS)

### Stage 5 — Sensor Bench Test
1. Flash ESP32 with I2C slave firmware (docs/esp32_firmware.md)
2. Wire HC-SR04 sensors to ESP32
3. Connect ESP32 to Pi 5 via I2C
4. Test `./setup.sh hardware` mode
5. Verify `/sensors/*` topics publishing real data

### Stage 6 — Motor Integration
1. Wire motor driver (L298N/DRV8833) to Pi GPIO
2. Measure actual wheel separation and radius → update hardware.yaml
3. Test `motor_controller` with manual cmd_vel
4. Verify watchdog timeout stops motors

### Stage 7 — SLAM & Navigation
1. Verify slam_bridge_node produces valid LaserScan
2. Test slam_toolbox with ultrasonic data
3. Test recon_node frontier exploration
4. Validate radius constraint

### Stage 8 — Full Integration
1. Test `./setup.sh full` mode
2. Implement cmd_vel_mux (prevent dual-source publishing)
3. Wire DB services to webui API endpoints
4. Write real test assertions in test skeletons

### Stage 9 — Hardening
1. Add respawn logic to launch files
2. Implement battery monitoring (if ESP32 ADC available)
3. Performance profiling under sustained operation
4. Error recovery and fault tolerance testing

---

*Generated from project analysis on 2026-03-04 · Last updated 2026-03-04 (v2.0)*
