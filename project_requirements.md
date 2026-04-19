# PROJECT REQUIREMENTS — AUTONOMOUS MAPPING ROBOT
### Codename: `roomba`
**Revision:** 2.5  
**Status:** DRAFT — For LLM-assisted development  
**Platform:** Raspberry Pi 5 · Ubuntu Server 24.04 LTS · ROS2 Jazzy Jalisco  

| Rev | Change |
|---|---|
| 1.0 | Initial specification |
| 1.1 | Sensor layer migrated from direct GPIO to ESP32 coprocessor over I2C |
| 1.2 | All real-time nodes (hardware, control, movement) mandated in C++17; Python retained for non-RT layers only |
| 1.3 | xpadneo mandated as Bluetooth controller driver; web UI made standalone with mock/test data mode; controller monitor and Bluetooth management pages added |
| 1.4 | Added `setup.sh` and `environment.sh` as required deliverables — see Section 10 |
| 1.5 | Added `bt_sim_node` for pre-hardware controller pipeline testing |
| 1.6 | **Architecture overhaul:** removed LIVE/DEMO mode split entirely. System now uses a universal fallback-to-mock pattern — real data from ROS2 topics replaces mock data automatically the moment it becomes available, with no code changes or restarts required |
| 1.7 | **Implementation corrections from live testing:** `joy_linux_node` mandated over SDL2 `joy_node`; D-pad reported as axes (not buttons) with xpadneo; trigger rest-value normalisation documented; `setup.sh` expanded with `kill`, `controller`, `bt-test` modes; `eventlet` monkey-patch threading constraint added; venv / ROS2 interaction caveats documented |
| 1.8 | **Networking & WiFi Access Point:** Web UI port changed from 5000 to 80; socket.io client library bundled locally (no CDN); Pi runs a WiFi hotspot (SSID `Roomba`) via `hostapd`+`dnsmasq` as a concurrent AP (ap0) alongside existing WiFi (wlan0); `dnsmasq` provides DHCP on the hotspot and DNS on both interfaces (`roomba.local`→Pi); `environment.sh` updated with new Section 9 for AP/networking setup; `setup.sh` auto-kill updated for port 80 |
| 1.9 | **environment.sh hardening & `--check` mode:** Added `--check` flag for verify-only runs (skips all installs, runs 100-check verification suite against project spec); `hostapd.conf` set to mode 600 (WPA passphrase not world-readable); `dnsmasq.conf` made idempotent (guard before overwrite); `roomba-ap-start.sh` now validates wlan0 exists before creating ap0; `socket.io.min.js` download verified via sha256 checksum; standalone dnsmasq service explicitly disabled; dead `screen` package removed from install list; verification expanded to 100 checks across 12 categories |
| 2.0 | **`cap_net_bind_service` + ROS2 ldconfig fix:** `python3.12` given `cap_net_bind_service` capability so the web UI can bind to port 80 without root; Linux capabilities cause the dynamic linker to **ignore `LD_LIBRARY_PATH`**, breaking all ROS2 C extension imports — fixed by registering `/opt/ros/jazzy/lib` in `/etc/ld.so.conf.d/ros2-jazzy.conf` via `ldconfig`; 3 new verification checks added (capability set, ldconfig entry, `librcl_action.so` in cache); total checks now **103** |
| 2.1 | **Headless save pipeline & map events:** Map saves now happen headlessly from `db_node` on SAVE_MAP events (controller X button) without requiring the web UI. New `map_events` DB table tracks SAVED/DELETED events. Web UI polls `GET /api/maps/events` every 3 s to detect headless saves and auto-refresh the saved maps list. `POST /api/maps` and `DELETE /api/maps/<id>` also write `MapEvent` rows. Added debug endpoints (`/api/debug/channels`, `/api/debug/controller`), Bluetooth `remove` and `setup` endpoints. ROS2 `RcutilsLogger` calls in `db_node.py` converted from %-style format strings to f-strings (RcutilsLogger does not support positional format args). Alembic scaffolded but not active — schema auto-created via `Base.metadata.create_all()` |
| 2.5 | **LIDAR bench test & sensor-test mode:** LD14P wire colours corrected (non-standard: Black=VCC, Green=GND, White=TX, Red=RX/unused). New `sensor-test` startup mode added — launches real LIDAR + static odom→base_link TF + SLAM toolbox + DB + web UI for stationary LIDAR testing without motors. `slam_params.yaml` minimum_travel_distance/heading set to 0.0 for stationary testing. Component map updated with `sensor-test` column and `odom→base_link` static TF row |
| 2.2 | **Stage 4c simulation & web UI streamlining:** Added `sim_goal_follower` to language assignment table (C++17, `roomba_navigation`). `recon_node` spec updated with Mamdani fuzzy inference (15 rules, 3 inputs, centroid defuzz), goal blacklisting on GOAL_FAILED events, and 30s blacklist timeout. `sim_goal_follower` spec added: P-control + fuzzy obstacle avoidance (10 rules) + stuck recovery (2s back-up + GOAL_FAILED event). Added sim API endpoints (`GET /api/sim/status`, `POST /api/sim/command`, `GET /api/sim/ground_truth`) and WebSocket events (`sim_status`, `sim_command`, `robot_mode`). Map page updated: side-by-side layout, saved maps as table (not dropdown), inline mode controls for IDLE/MANUAL/RECON. `simulate-hw` mode added to component map with `sim_goal_follower`. Environment.sh test skeleton checks updated from 8 to 12. Total test files: 12 with 68 test cases |
| 2.3 | **Sensor architecture change (superseded by 2.4):** Intermediate design with ESP32 bridging LIDAR + ultrasonic over I2C. Replaced by direct UART connection in v2.4 |
| 2.4 | **Direct LIDAR architecture:** Ultrasonic sensors **removed**. LD14P (LD-D200) LIDAR now connects **directly to Raspberry Pi 5 via UART** (`/dev/ttyAMA0`, 230400 baud), bypassing the ESP32 for sensor data. LIDAR driver provided by `ldlidar_stl_ros2` package (cloned into workspace). `esp32_sensor_node` and `slam_bridge_node` removed from codebase. **ESP32 retained as motor coprocessor** — receives motor commands from Pi over I2C and drives the motor drivers. `pigpio` removed (motors not on Pi GPIO). I2C bus enabled for ESP32 communication. Component map, launch architecture, configs, and setup.sh updated accordingly |

---

## 0. PURPOSE OF THIS DOCUMENT

This document defines the authoritative technical specification for an autonomous mapping robot. It is written to be consumed directly by an LLM coding agent. Every section constitutes a binding requirement unless explicitly marked `[OPTIONAL]`. When generating code, the agent **must**:

- Follow these requirements exactly. Do not infer intent — ask for clarification if ambiguous.
- Never introduce dependencies not listed here without flagging them explicitly.
- Produce modular, independently testable ROS2 nodes. One concern per node.
- **Real-time nodes must be written in C++17.** Non-real-time nodes may be written in Python 3.11+. See Section 2.1 for the definitive language assignment per node.
- C++ code must follow the ROS2 C++ style guide. Python code must follow PEP8. Maximum line length: 100 characters in both languages.
- Type annotations are mandatory on all public Python functions. All public C++ functions must have Doxygen-style doc comments.
- Every node must have a corresponding unit test skeleton in `/tests/`.

---

## 1. HARDWARE PLATFORM

| Component | Specification |
|---|---|
| SBC | Raspberry Pi 5 (8 GB RAM recommended) |
| OS | Ubuntu Server 24.04 LTS (64-bit ARM) |
| Motor Driver | ESP32 coprocessor (I2C slave at `0x42` on `/dev/i2c-1`) drives motor H-bridges (L298N or DRV8833). Pi sends motor commands over I2C. See `esp32_firmware.md` for register map |
| LIDAR | LD14P (LD-D200) 360° laser scanner — connected **directly to Pi 5 via UART** (`/dev/ttyAMA0`, 230400 baud). ROS2 driver: `ldlidar_stl_ros2` |
| Remote Control | Xbox controller (Bluetooth) — driver: `xpadneo` + `bluez` (see Section 1.2) |
| Power | LiPo battery pack — logic rail 5V/5A, motor rail 12V (or 7.4V 2S LiPo) |
| Connectivity | WiFi (onboard Pi 5) — concurrent AP+STA: Pi creates its own hotspot (SSID `Roomba`, WPA2) on virtual interface `ap0` while remaining connected to an existing WiFi network on `wlan0`. Web UI accessible from hotspot clients at `http://10.0.0.1/` or `http://roomba.local/`, and from LAN clients via the Pi's DHCP-assigned IP. No internet dependency at runtime |

### 1.1 LIDAR — Direct UART Connection to Pi 5

The LD14P (LD-D200) LIDAR connects **directly** to the Raspberry Pi 5 via UART. No ESP32 coprocessor or I2C bridge is used. The LIDAR driver is provided by the `ldlidar_stl_ros2` ROS2 package (C++, cloned into `roomba_ws/src/`).

#### Wiring (LD14P → Pi 5 GPIO header)

| LD14P Pin | Wire Colour (confirmed) | Pi 5 Pin | Pi 5 Function |
|---|---|---|---|
| TX | White | Pin 10 (GPIO 15) | UART0 RX (`/dev/ttyAMA0`) |
| RX | Red | — (leave disconnected) | Not used |
| VCC | Black | Pin 4 | 5V power |
| GND | Green | Pin 6 | Ground |

> **⚠️ WARNING:** The LD14P wire colours are **non-standard and counterintuitive** (Black≠GND, Red≠VCC). The mapping above was confirmed by hardware testing. Do not assume standard colour conventions.

> **Note:** The LD14P motor draws ~350 mA at 5V. The Pi 5 can supply this from its 5V pins, but ensure the power supply can deliver at least 5A total (Pi + LIDAR + motors).

#### Pi 5 UART Configuration

The Pi 5's PL011 UART must be freed from the serial console:

```bash
# Disable serial console (frees /dev/ttyAMA0 for LIDAR)
sudo systemctl stop serial-getty@ttyAMA0.service
sudo systemctl disable serial-getty@ttyAMA0.service

# Enable UART0 hardware in boot config
# Add to /boot/firmware/config.txt:
enable_uart=1
dtoverlay=uart0
```

After reboot, `/dev/ttyAMA0` is available for the LIDAR at **230400 baud**.

#### LIDAR Driver — `ldlidar_stl_ros2`

The driver is the official LDRobot ROS2 package, cloned into `roomba_ws/src/ldlidar_stl_ros2/` and built with colcon alongside our packages.

- **Publishes:** `sensor_msgs/msg/LaserScan` on `/scan` (frame ID: `laser_frame`)
- **Parameters:** serial port, baud rate, LIDAR model, frame ID, topic name — all configurable via launch file or `--params-file`
- **LD14P specs:** 360° scan, ~2300 points/revolution, 5–10 Hz rotation, range 0.02–8.0 m

```bash
# Clone into workspace (one-time setup)
cd roomba_ws/src
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
cd .. && colcon build --symlink-install
```

#### Motor Driver Wiring

- Motors are driven by an **ESP32 coprocessor** connected to the Pi 5 over **I2C** (`/dev/i2c-1`, address `0x42`).
- The ESP32 receives motor speed/direction commands from the Pi and drives the H-bridge motor drivers (L298N or DRV8833) via PWM GPIO outputs.
- Motor parameters are declared in `config/hardware.yaml`. No values shall be hardcoded in source files.
- See `docs/esp32_firmware.md` for the I2C register map and wiring.

---

### 1.2 Xbox Controller — Bluetooth Driver Stack

The mandated driver stack for the Xbox controller is **`xpadneo` + `bluez`**. Do not use the in-kernel `xpad` module for Bluetooth connections — it has known axis misreporting and missing feature support for Xbox Series controllers.

#### Driver Stack

| Layer | Component | Role |
|---|---|---|
| Bluetooth stack | `bluez` (system) | HCI management, pairing, connection |
| HID driver | `xpadneo` (DKMS) | Xbox-specific HID mapping, rumble, battery |
| Input device | `/dev/input/js0` or `/dev/input/eventX` | Exposed to ROS2 joy node |
| ROS2 | `joy_linux_node` (from `joy_linux` package) | Reads input events via evdev, publishes `/joy` |

> **IMPORTANT — `joy_linux_node`, not `joy_node`:** The SDL2-based `joy_node` (from the `joy` package) causes haptic/force-feedback errors with xpadneo on Xbox controllers. Use the evdev-based `joy_linux_node` from the `joy_linux` package instead. The `ros-jazzy-joy` apt package installs both sub-packages. Launch command: `ros2 run joy_linux joy_linux_node --ros-args -p dev:=/dev/input/js0`

#### Installation (must be documented in `README.md`)

```bash
# Dependencies
sudo apt install dkms linux-headers-$(uname -r) bluez

# xpadneo — install from source (no apt package available)
git clone https://github.com/atar-axis/xpadneo.git
cd xpadneo
sudo ./install.sh

# Verify driver loaded
dkms status | grep xpadneo
```

#### Pairing Procedure (must be documented in `README.md`)

```bash
sudo bluetoothctl
  power on
  agent on
  default-agent
  scan on
  # Press Xbox button + Share button on controller until LED flashes rapidly
  pair <MAC_ADDRESS>
  trust <MAC_ADDRESS>
  connect <MAC_ADDRESS>
  scan off
  exit
```

After pairing, the controller MAC must be persisted so it auto-reconnects on boot. Use `bluetoothctl trust <MAC>` as shown above — `bluez` handles reconnection automatically.

#### xpadneo Configuration

- xpadneo configuration file: `/etc/modprobe.d/xpadneo.conf`
- Relevant options to document in `README.md`:
  - `options xpadneo trigger_rumble_damping=4` — reduces trigger rumble intensity
  - `options xpadneo disable_ff=0` — keep force feedback enabled (used for future haptic alerts)
- Button and axis indices exposed by xpadneo to the `joy_linux_node` are defined in `config/controller.yaml`. Do not hardcode indices.

#### Axis & Button Mapping (joy_linux + xpadneo) — Verified

The following mapping has been verified on real hardware (Xbox controller paired via Bluetooth with xpadneo v0.10+). **These indices differ from the SDL2-based `joy_node`.**

**Axes (8 total):**

| Index | Axis | Rest Value | Notes |
|---|---|---|---|
| 0 | Left Stick X | 0.0 | |
| 1 | Left Stick Y | 0.0 | Up = negative (hardware convention) |
| 2 | Left Trigger | 1.0 | Released = 1.0, fully pressed = -1.0 |
| 3 | Right Stick X | 0.0 | |
| 4 | Right Stick Y | 0.0 | Up = negative |
| 5 | Right Trigger | 1.0 | Released = 1.0, fully pressed = -1.0 |
| 6 | D-Pad X | 0.0 | Left = -1.0, Right = 1.0 |
| 7 | D-Pad Y | 0.0 | Up = -1.0, Down = 1.0 |

> **Trigger normalisation:** The web bridge (`ros_bridge.py`) normalises triggers from the hardware range `[1.0, -1.0]` to the logical range `[0.0, 1.0]` before emitting to the frontend.

> **D-Pad as axes:** With xpadneo + joy_linux, the D-Pad is reported as **axes 6 and 7**, not as buttons. The web bridge synthesises virtual D-Pad button events (`dpad_up`, `dpad_down`, `dpad_left`, `dpad_right`) from these axis values.

**Buttons (11 total):**

| Index | Button |
|---|---|
| 0 | A |
| 1 | B |
| 2 | X |
| 3 | Y |
| 4 | LB |
| 5 | RB |
| 6 | Select (Back/View) |
| 7 | Start (Menu) |
| 8 | Xbox |
| 9 | Left Stick Click |
| 10 | Right Stick Click |

> **Per-axis inversion:** `config/controller.yaml` includes an `invert:` section with boolean flags per axis. The `joy_control_node` (C++) already negates `left_y` for the forward=positive convention, so `left_y` inversion in the webui config is set to `false`.

#### Bluetooth Management from Web UI

The web interface must include a **Bluetooth management page** (see Section 3.5) that wraps `bluetoothctl` via Python `subprocess` calls. This allows the operator to pair, connect, and disconnect the controller from the browser without SSH access.

---

## 2. SOFTWARE STACK

### 2.1 Core Framework & Language Policy

| Layer | Technology | Version |
|---|---|---|
| Robotics middleware | ROS2 | Jazzy Jalisco |
| Real-time language | **C++** | **C++17** (mandatory for RT nodes) |
| Non-real-time language | Python | 3.11+ |
| Build system | colcon | latest compatible |
| Package manager | apt + pip (venv per non-RT package) | — |

#### Language Assignment — Authoritative Table

This table is **binding**. Do not deviate without explicit approval.

| Node | Package | Language | Rationale |
|---|---|---|---|
| `motor_controller` | `roomba_hardware` | **C++17** | PWM generation and watchdog are hard real-time |
| `joy_control_node` | `roomba_control` | **C++17** | Controller input must have minimal latency to cmd_vel |
| `bt_sim_node` | `roomba_control` | **C++17** | Simulated controller — must use identical topic/msg types as real joy path |
| `draw_node` | `roomba_control` | **C++17** | Testing — controller-driven OccupancyGrid drawing for DB persistence testing |
| `recon_node` | `roomba_navigation` | **C++17** | Real-time navigation decisions and frontier evaluation |
| `sim_sensor_node` | `roomba_hardware` | **C++17** | Simulated ultrasonic + LIDAR via raycasting — drop-in for simulation mode |
| `sim_motor_node` | `roomba_hardware` | **C++17** | Simulated diff-drive kinematics + collision — drop-in for `motor_controller` |
| `sim_goal_follower` | `roomba_navigation` | **C++17** | Simulated goal tracking with fuzzy obstacle avoidance — used in `simulate-hw` mode |
| `db_node` | `roomba_db` | Python 3.11+ | Non-RT — database I/O, SQLAlchemy ORM |
| `roomba_webui` | `roomba_webui` | Python 3.11+ | Non-RT — Flask/SocketIO server |

> **External driver nodes** (not in our codebase, launched as-is): `ldlidar_stl_ros2` (LD14P LIDAR C++ driver, publishes `/scan`), `slam_toolbox`, `nav2`, `joy_linux_node`.

> **Rule:** If a node subscribes to `/cmd_vel`, publishes to `/cmd_vel`, reads sensors, or drives actuators — it is real-time and **must** be C++17. If a node only handles persistence, web serving, or operator-facing UI — it may be Python.

#### C++17 Standards for Real-Time Nodes

- Use `rclcpp` (not `rclpy`) for all C++ nodes.
- Use `rclcpp::TimerBase` with `rclcpp::WallTimer` for periodic callbacks. Never use `std::this_thread::sleep_for` in a callback.
- Prefer `std::array` and stack allocation over heap allocation in hot paths.
- Use `RCLCPP_DEBUG`, `RCLCPP_WARN`, `RCLCPP_ERROR` macros — never `std::cout` or `printf`.
- All parameters must be declared with `declare_parameter<T>()` and retrieved via `get_parameter()`.
- C++ packages must have a `CMakeLists.txt` using `ament_cmake`. Python packages use `ament_python`.
- Unit tests for C++ nodes use `ament_cmake_gtest` with Google Test.

### 2.2 ROS2 Package Layout

```
roomba_ws/
├── src/
│   ├── roomba_bringup/          # Launch files only (Python)
│   ├── roomba_hardware/         # C++17 — Motor controller, simulation sensor/motor nodes
│   ├── roomba_navigation/       # C++17 — Pathfinding, autonomy
│   ├── roomba_control/          # C++17 — Xbox controller to cmd_vel bridge
│   ├── roomba_webui/            # Python — Flask web server + WebSocket bridge
│   │   ├── data_channels.py     # DataChannel class — per-channel real/mock fallback logic
│   │   ├── mock_data.py         # All mock data generators (isolated here only)
│   │   └── bluetooth_manager.py # bluetoothctl subprocess wrapper
│   ├── roomba_db/               # Python — Database abstraction layer
│   └── ldlidar_stl_ros2/        # External — LDRobot LD14P UART driver (cloned from GitHub)
├── config/
│   └── hardware.yaml
└── tests/
```

### 2.3 Required ROS2 Packages

- `slam_toolbox` — SLAM implementation (2D, online async mode)
- `nav2` — Navigation stack (path planning, costmap)
- `sensor_msgs` — Range, LaserScan message types
- `geometry_msgs` — Twist, Pose, PoseStamped
- `nav_msgs` — OccupancyGrid, Odometry
- `joy` — Joystick ROS2 meta-package; installs both `joy` (SDL2) and `joy_linux` (evdev) sub-packages. **Use `joy_linux_node` from the `joy_linux` package** — see Section 1.2
- `teleop_twist_joy` — Joystick to cmd_vel (used as base, may be customised)

### 2.4 Python Dependencies (non-RT nodes only)

```
flask>=3.0
flask-socketio>=5.3
flask-sqlalchemy>=3.1
sqlalchemy>=2.0
psycopg2-binary>=2.9
alembic>=1.13
numpy>=1.26
opencv-python-headless>=4.9   # [OPTIONAL] future camera support
pytest>=8.0
pyyaml>=6.0
eventlet>=0.35
```

> **Note:** `smbus2` and `RPi.GPIO` are **removed from Python dependencies**. Motor control is handled in C++ via ESP32 I2C communication.
>
> **Note:** `eventlet` is **mandatory** for Flask-SocketIO async mode. See the eventlet threading constraint in Section 3.5.

### 2.5 C++ Dependencies (real-time nodes)

Add to each real-time package's `CMakeLists.txt`:

```cmake
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)         # roomba_navigation only

# For roomba_hardware (ESP32 motor coprocessor):
# I2C communication: uses Linux i2c-dev (no extra library needed)
# sudo apt install i2c-tools  (for diagnostics only)
```

| Library | Purpose | Install |
|---|---|---|
| `i2c-dev` (kernel) | I2C communication with ESP32 motor coprocessor | Built into kernel; `i2c-tools` for diagnostics (`sudo apt install i2c-tools`). See `environment.sh` Section 3 |
| `Google Test` | C++ unit tests | `sudo apt install libgtest-dev` |

---

## 3. MODULE SPECIFICATIONS

### 3.1 MODULE: `roomba_hardware` — Actuator Layer & Simulation

**Nodes to implement:**

> **LIDAR driver:** The LD14P LIDAR is driven by the external `ldlidar_stl_ros2` package (see Section 1.1). It publishes `/scan` directly. No sensor node is needed in `roomba_hardware` for real hardware — only `motor_controller` and the simulation drop-in nodes.

#### `motor_controller` — **C++17**
- Implemented in C++17 using `rclcpp` and Linux `i2c-dev` for ESP32 communication.
- Subscribes to `/cmd_vel` (`geometry_msgs/msg/Twist`).
- Translates linear.x and angular.z into differential drive motor commands, sent to ESP32 coprocessor over I2C (`/dev/i2c-1`, address `0x42`).
- Wheel parameters declared in `hardware.yaml`: `wheel_separation`, `wheel_radius`.
- Must implement a **hardware watchdog**: if `/cmd_vel` is not received within 500ms, set motor PWM to zero and publish a `std_msgs/msg/String` STOP event to `/robot/events`. Use `rclcpp::WallTimer` for the watchdog, reset on every received message.
- PWM frequency: 1000 Hz.

---

### 3.2 MODULE: `roomba_control` — Xbox Controller Interface

#### `joy_control_node` — **C++17**
- Implemented in C++17 using `rclcpp`.
- Depends on ROS2 `joy_linux_node` (from `joy_linux` package, launched alongside).
- Subscribes to `/joy` (`sensor_msgs/msg/Joy`).
- Publishes to `/cmd_vel` only when **manual mode** is active. Must read current mode from `/robot/mode` parameter server or topic before publishing.
- Button mapping (configurable in `config/controller.yaml`, not hardcoded):

| Xbox Button | Action |
|---|---|
| Left Stick Y-axis | Linear velocity (forward/back) |
| Right Stick X-axis | Angular velocity (turn) |
| Start | Toggle Manual / Autonomous mode |
| B | Emergency stop (publishes to `/robot/events`) |
| Y | Trigger Recon mode |
| X | Save current map to DB |

- Velocity scaling factors must be parameters (`max_linear_vel`, `max_angular_vel`).
- Bluetooth pairing is handled at OS level (not within ROS). Document the pairing procedure in `README.md`.

#### `bt_sim_node` — **C++17** — Bluetooth / Controller Pre-Hardware Test Node

**Purpose:** Allows the full Bluetooth → ROS2 → web UI pipeline to be developed and validated before any physical Xbox controller exists. When `bt_sim_node` is running, it publishes to `/joy` just like a real `joy_node` would — so the web UI's `/joy` `DataChannel` receives real ROS2 data, promotes out of mock mode, and shows the `connected: true` state. The moment a real controller is paired and `joy_node` is started instead, `bt_sim_node` is simply not launched — the rest of the stack is completely unchanged.

**Design constraint:** `bt_sim_node` must publish **identical message types on identical topics** to what the real `joy_node` and `joy_control_node` produce. No downstream node (`motor_controller`, `roomba_webui`, etc.) may contain any awareness of whether data comes from `bt_sim_node` or the real controller path.

**Node behaviour:**
- Publishes `sensor_msgs/msg/Joy` on `/joy` at **50 Hz**.
- Publishes `geometry_msgs/msg/Twist` on `/cmd_vel` at **20 Hz**.
- Publishes `std_msgs/msg/String` on `/bt_sim/status` — value `"BT_SIM_ACTIVE"` — so other nodes and the web UI can detect simulation mode.
- Simulates controller input using one of three **waveform profiles**, selectable via ROS2 parameter `bt_sim_profile`:

| Profile | Behaviour | Use case |
|---|---|---|
| `idle` | All axes 0.0, all buttons false | Verify idle/watchdog behaviour |
| `sine` | Left Y and Right X as slow sine waves (period 8 s), no buttons | Verify smooth cmd_vel generation and web UI visualisation |
| `patrol` | Left Y = 0.5 constant (forward), Right X = 0.0 for 3 s then 0.5 for 1 s (turn), repeating | Simulate a simple patrol loop |

- All waveform parameters (amplitude, period, patrol durations) must be ROS2 parameters — not hardcoded.
- The node must also accept **manual override via a ROS2 service** `/bt_sim/inject` (`std_srvs/srv/SetBool`-style, but with a custom `BtSimInject.srv` carrying axes and button arrays) so that integration tests can inject precise input values programmatically.
- On startup, logs at `INFO`: `"bt_sim_node active — real controller NOT required. Profile: <profile>"`
- Must be **mutually exclusive** with `joy_node`: the launch system must ensure only one of `{joy_node, bt_sim_node}` is active at a time. This is enforced by the `bt-test` mode in `setup.sh` (see Section 10) which launches `bt_sim_node` instead of `joy_node`.

**Custom service definition** (`roomba_control/srv/BtSimInject.srv`):
```
# Request
float32[] axes        # must be length 6: [left_x, left_y, right_x, right_y, lt, rt]
bool[]    buttons     # must be length 14: [a, b, x, y, lb, rb, start, select, ls, rs, dup, ddown, dleft, dright]
float32   duration_s  # how long to hold this input before returning to profile (0 = indefinite)
---
# Response
bool success
string message
```

---

### 3.3 MODULE: `roomba_navigation` — SLAM & Autonomy

#### `recon_node` — **C++17** — Reconnaissance / Autonomous Mapping Mode
- Activated when `/robot/mode` = `RECON`.
- Implements a **frontier-based exploration** algorithm with **Mamdani fuzzy inference** for frontier selection:
  1. Read current OccupancyGrid from `/map`.
  2. Identify frontier cells (known-free adjacent to unknown).
  3. Cluster adjacent frontier cells into frontier groups.
  4. Evaluate each frontier using fuzzy inference with 3 inputs (distance, size, heading alignment), 15 rules, and centroid defuzzification to produce a desirability score.
  5. Select the most desirable frontier as the navigation goal.
  6. Publish goal as `PoseStamped` to `/goal_pose`.
  7. On goal completion or failure, repeat.
- **Goal blacklisting:** When a `GOAL_FAILED` event is received on `/robot/events`, the current goal is blacklisted for 30 seconds. The node selects the next best frontier, preventing repeated attempts at unreachable goals.
- **Fuzzy membership function parameters** (trapezoidal) are loaded from `config/simulation.yaml` — not hardcoded.
- **Boundary constraint:** Must accept a `recon_radius` parameter (metres) defining maximum travel distance from starting pose. Refuse to navigate to frontiers outside this radius.
- On entering RECON mode, snapshot the current pose as `origin`. All distance checks are relative to this `origin`.
- On exiting RECON mode (mode change or B button), publish the final map save trigger to `/robot/events`.
- On completion (no more frontiers within radius), publish `RECON_COMPLETE` to `/robot/events`.

#### `sim_goal_follower` — **C++17** — Simulated Goal Tracker
- Drop-in replacement for nav2 goal tracking used in `simulate-hw` mode.
- Subscribes to `/goal_pose` (PoseStamped) and `/sensors/{front,left,right}` (Range).
- Publishes `/cmd_vel` (Twist) using **P-control** for goal approach with **fuzzy obstacle avoidance** overlay (10 rules, 2 outputs: speed modifier and turn modifier).
- **Stuck recovery:** If the robot hasn't moved >0.1m toward the goal in the stuck timeout period, publishes a 2-second backward motion, then publishes `GOAL_FAILED` on `/robot/events` so `recon_node` can blacklist the goal and try an alternative.
- Only resets goal timeout when a genuinely new goal arrives (>0.1m from previous goal position) — prevents duplicate goal messages from resetting the timer.
- Publishes `GOAL_REACHED` on `/robot/events` when within tolerance of the goal.
- Parameters loaded from `config/simulation.yaml`.

---

### 3.4 MODULE: `roomba_db` — Persistence Layer

#### Database: PostgreSQL 16 in Docker (production) / SQLite in-memory (tests)
- **Production:** PostgreSQL 16-alpine container managed by `docker/docker-compose.yaml`. Connection: `postgresql://roomba:<password>@localhost:5432/roomba`.
- **Tests:** In-memory SQLite via `ROOMBA_DB_URL=sqlite:///:memory:` — no Docker needed.
- Connection string must come from environment variable `ROOMBA_DB_URL`.
- SQLAlchemy ORM is mandatory. No raw SQL strings in application code.
- **Alembic** is scaffolded (`alembic.ini` present) but **not active**. Schema is auto-created via `Base.metadata.create_all(engine)` in `get_session_factory()`. Alembic migrations may be introduced later for production schema versioning.

#### Docker Setup
- `docker/docker-compose.yaml`: postgres:16-alpine, 256MB memory limit, health check, named volume `roomba_pgdata`, port `127.0.0.1:5432:5432`
- `docker/.env`: DB credentials (gitignored). Copy `docker/.env.example` to create.
- `setup.sh` calls `ensure_db()` which runs `docker compose up -d` and waits for `pg_isready` before launching `db_node`
- `environment.sh` Section 10 installs `docker.io` + `docker-compose-v2` and starts the container

#### Schema

```
Table: maps
  id          INTEGER PRIMARY KEY AUTOINCREMENT
  name        TEXT NOT NULL
  created_at  DATETIME DEFAULT CURRENT_TIMESTAMP
  updated_at  DATETIME
  map_data    BLOB NOT NULL        -- serialised OccupancyGrid (JSON-encoded)
  origin_x    FLOAT
  origin_y    FLOAT
  resolution  FLOAT                -- metres/cell
  width       INTEGER              -- cells
  height      INTEGER              -- cells
  thumbnail   BLOB                 -- [OPTIONAL] PNG preview for web UI

Table: sessions
  id          INTEGER PRIMARY KEY AUTOINCREMENT
  started_at  DATETIME DEFAULT CURRENT_TIMESTAMP
  ended_at    DATETIME
  mode        TEXT                 -- MANUAL | RECON | IDLE
  map_id      INTEGER REFERENCES maps(id)

Table: map_events
  id          INTEGER PRIMARY KEY AUTOINCREMENT
  event_type  TEXT NOT NULL         -- SAVED | DELETED
  map_id      INTEGER               -- associated map (nullable for deletes)
  map_name    TEXT                   -- snapshot of map name at event time
  created_at  DATETIME DEFAULT CURRENT_TIMESTAMP
```

The `map_events` table enables the web UI to detect headless saves from `db_node` (controller X button) via polling, without requiring WebSocket push from the saver.

#### `db_node`
- ROS2 node that subscribes to:
  - `/robot/events` (String) — handles SAVE_MAP trigger (from controller X button or web UI)
  - `/robot/mode` (String) — records session changes
  - `/map` (OccupancyGrid) — caches latest map for saving on demand
- Map saves run in a ThreadPoolExecutor to avoid blocking the ROS2 callback thread.
- **Headless save pipeline:** On `SAVE_MAP` event, `db_node` saves the latest cached map to the `maps` table with a date-based auto-generated name, then writes a `MapEvent(event_type="SAVED")` row so the web UI can detect the new save via polling. This works even when the web UI is not running.
- **ROS2 logger constraint:** `RcutilsLogger` does **not** support Python %-style format strings (`logger.debug("x=%d", val)`). Use f-strings for all `self.get_logger()` calls in ROS2 Python nodes.

#### Web API Endpoints (in `app.py`)
- `GET /api/maps` — list all maps (id, name, created_at, resolution, width, height)
- `GET /api/maps/<id>` — single map metadata
- `GET /api/maps/<id>/data` — full grid data (JSON)
- `POST /api/maps` — save current live map (optional `name` field); writes a `MapEvent(SAVED)` row
- `PUT /api/maps/<id>` — rename map
- `DELETE /api/maps/<id>` — delete map; writes a `MapEvent(DELETED)` row
- `GET /api/maps/events?since=<id>` — return `MapEvent` rows with id > `since` (default 0 = all). Polled by the map page every 3 s to detect headless saves from `db_node`
- All DB calls wrapped in `tpool.execute()` (eventlet threading constraint).

---

### 3.5 MODULE: `roomba_webui` — Web Interface

#### Framework: Flask + Flask-SocketIO

- Server binds to `0.0.0.0:80` (LAN and hotspot accessible). Port configurable via `config/webui.yaml` or env var `WEBUI_PORT`.
- **No authentication required** (LAN-only / hotspot assumption). Add a `TODO` comment to add auth if exposed beyond LAN.
- **Static assets:** The `socket.io` client library is bundled locally at `static/js/socket.io.min.js` — no CDN dependency. The base template references it via `url_for('static', ...)`.
- The web server runs as a ROS2 node at all times. It subscribes to all relevant topics passively.

---

#### eventlet Threading Constraint — CRITICAL

> **`eventlet.monkey_patch()` must be the very first import in `app.py`**, before importing Flask, Flask-SocketIO, rclpy, or any other module. Failure to do so causes silent deadlocks.
>
> Flask-SocketIO with eventlet uses cooperative green threads. **`socketio.emit()` can only be called from an eventlet green thread.** Calling it from a real OS thread (e.g., an `rclpy.spin()` callback) will deadlock the entire server.
>
> The mandated solution is a **`queue.Queue` bridge**: rclpy subscriber callbacks enqueue events via `queue.Queue.put_nowait()`, and an eventlet background loop (`emit_loop`) calls `queue.Queue.get()` to drain events and emit them safely on the green thread. This pattern is implemented in `ros_bridge.py`.
>
> This is a binding architectural constraint — do not attempt to call `socketio.emit()` from any rclpy callback directly.

---

#### Core Architecture Principle — Universal Fallback-to-Mock

> **The system has no "demo mode" or "live mode" switch. There is only one mode.**
>
> Every data channel used by the web UI operates independently on the same rule:
> **emit mock data by default; the moment a real ROS2 topic produces data, switch to it transparently and stay on it. If the topic goes silent, fall back to mock automatically.**
>
> No restart, no config change, no environment variable, and no code path difference is required. The frontend receives the same WebSocket events regardless of data source. The browser never knows the difference.

This principle applies to **every** data channel individually and independently. Sensors, controller, map, pose, and Bluetooth status each manage their own fallback state in isolation. Adding a new hardware component means its ROS2 topic starts publishing — the UI promotes automatically.

---

#### Per-Channel Fallback Behaviour

Each data channel is managed by a `DataChannel` class in `roomba_webui/data_channels.py`. Every channel must implement this interface:

```python
class DataChannel:
    """Manages one stream of data with automatic real/mock fallback.

    Args:
        topic: ROS2 topic name this channel subscribes to.
        timeout_s: Seconds of silence before falling back to mock.
        mock_fn: Callable that returns the next mock value for this channel.
    """
    def get(self) -> Any:
        """Return the most recent real value, or mock value if topic is silent."""

    def is_live(self) -> bool:
        """Return True if real topic data has been received within timeout_s."""

    def on_ros_message(self, msg) -> None:
        """Called by the ROS2 subscriber callback. Updates internal state."""
```

Each channel tracks a `last_real_timestamp`. If `now - last_real_timestamp > timeout_s`, `is_live()` returns `False` and `get()` returns `mock_fn()`. The moment a real message arrives, `last_real_timestamp` is updated and `is_live()` returns `True`.

The `timeout_s` per channel is configurable in `config/webui.yaml`. Defaults:

| Channel | Default timeout |
|---|---|
| Sensor distances | 2.0 s |
| Controller (`/joy`) | 3.0 s |
| Robot pose | 2.0 s |
| Map (OccupancyGrid) | 10.0 s |
| Bluetooth status | 5.0 s |

---

#### Mock Data Specification

All mock generators live in `roomba_webui/mock_data.py`. No mock logic anywhere else.

| Channel | Mock behaviour |
|---|---|
| Sensor distances | Sine-wave oscillation 0.2–3.5 m per sensor, each with a different phase offset |
| Robot pose | Slow circular path, radius 1.5 m, period 30 s |
| Map (OccupancyGrid) | Static pre-baked 20×20 grid: walls on edges, free space inside |
| Sensor health | All sensors OK |
| Controller axes | Gentle sine wave on left-Y, all other axes 0.0 |
| Controller buttons | All unpressed |
| Controller connected | `false` (mock always reports disconnected — real data reports true) |
| Bluetooth device | `{name: "No device (mock)", mac: "—", connected: false, battery_pct: null}` |

> **Key rule on controller connected state:** mock data always reports `connected: false`. This means the web UI will naturally show "no controller" until a real `/joy` message arrives — giving a clear visual signal that hardware is not yet present, with no special logic required.

---

#### Status Indicators on All Pages

Every page in the web UI must display a **data source indicator strip** — a small persistent bar showing the live/mock state of each channel in real time:

```
● SENSORS  ○ CONTROLLER  ● MAP  ● POSE  ○ BLUETOOTH
```

- Filled circle (●) = receiving real ROS2 data
- Empty circle (○) = using mock data (hardware absent or silent)
- The strip updates every 2 seconds via a dedicated `channel_status` WebSocket event.
- This is the **only** place where the real/mock distinction is surfaced to the user. All other UI elements simply display whatever data they receive.

`channel_status` WebSocket event payload:
```json
{
  "sensors":     { "live": true,  "last_seen_s": 0.3 },
  "controller":  { "live": false, "last_seen_s": 999  },
  "map":         { "live": true,  "last_seen_s": 1.1  },
  "pose":        { "live": true,  "last_seen_s": 0.1  },
  "bluetooth":   { "live": false, "last_seen_s": 999  }
}
```

---

#### Pages / Endpoints

| Route | Method | Description |
|---|---|---|
| `/` | GET | Main dashboard |
| `/map` | GET | Live map viewer page |
| `/controller` | GET | Controller input monitor page |
| `/bluetooth` | GET | Bluetooth device management page |
| `/api/maps` | GET | JSON list of saved maps |
| `/api/maps/<id>` | GET | JSON of single map metadata |
| `/api/maps/<id>/data` | GET | Map grid data as JSON |
| `/api/maps/<id>` | DELETE | Delete a map (writes MapEvent) |
| `/api/maps/events` | GET | Poll for map events since a given id (`?since=<id>`) |
| `/api/maps` | POST | Save current live map to DB (writes MapEvent) |
| `/api/maps/<id>` | PUT | Rename a saved map |
| `/api/robot/mode` | POST | Set robot mode (`MANUAL`, `RECON`, `IDLE`) |
| `/api/robot/status` | GET | Current mode, battery estimate, sensor readings |
| `/api/bluetooth/devices` | GET | List known/paired Bluetooth devices |
| `/api/bluetooth/scan` | POST | Start a 10-second BT scan, return discovered devices |
| `/api/bluetooth/pair` | POST | Pair a device by MAC address |
| `/api/bluetooth/connect` | POST | Connect a paired device by MAC address |
| `/api/bluetooth/disconnect` | POST | Disconnect a device by MAC address |
| `/api/bluetooth/trust` | POST | Trust a device by MAC address |
| `/api/bluetooth/remove` | POST | Remove a paired device by MAC address |
| `/api/bluetooth/setup` | POST | One-shot pair + trust + connect by MAC address |
| `/api/debug/channels` | GET | Debug — data channel liveness and bridge status |
| `/api/debug/controller` | GET | Debug — current controller channel data as JSON |
| `/api/sim/status` | GET | Current simulation room info (dimensions, seed, cell counts) |
| `/api/sim/command` | POST | Send sim command (regenerate, reset_pose, pause, resume) — whitelist-validated |
| `/api/sim/ground_truth` | GET | Ground truth OccupancyGrid for debug overlay |

#### WebSocket Events (Flask-SocketIO)

| Event | Direction | Payload | Description |
|---|---|---|---|
| `map_update` | Server → Client | OccupancyGrid as JSON | Emitted at 2 Hz — real or mock |
| `robot_pose` | Server → Client | `{x, y, theta}` | Emitted at 5 Hz — real or mock |
| `sensor_data` | Server → Client | `{front, left, right}` in metres | Emitted at 5 Hz — real or mock |
| `sensor_health` | Server → Client | `{front_ok, left_ok, right_ok}` | Emitted at 1 Hz — real or mock |
| `controller_state` | Server → Client | Full axes + buttons payload | Emitted at 20 Hz — real or mock |
| `bluetooth_status` | Server → Client | `{connected, name, mac, battery_pct}` | Emitted at 2 Hz — real or mock |
| `channel_status` | Server → Client | Per-channel live/mock state | Emitted at 0.5 Hz always |
| `robot_event` | Server → Client | `{type, message}` | Forwarded from `/robot/events` |
| `robot_mode` | Server → Client | `{mode}` | Current mode broadcast (on connect + on change) |
| `sim_status` | Server → Client | Room info JSON | Emitted at 1 Hz when sim nodes active |
| `set_mode` | Client → Server | `{mode}` | Triggers mode change |
| `sim_command` | Client → Server | `{command}` | Send sim command (regenerate, reset_pose, etc.) |

#### Controller Monitor Page (`/controller`)

- Displays real-time Xbox controller input. Source: `/joy` topic when live, mock sine data when not.
- Page renders a **visual gamepad diagram** (SVG or Canvas) with live axis and button states.
- When `controller_state.connected == false`, overlay a banner: **"Controller not connected — showing mock data"**.
- When `controller_state.connected == true`, the banner disappears automatically. No page reload.
- `controller_state` payload:

```json
{
  "axes": { "left_x": 0.0, "left_y": 0.0, "right_x": 0.0, "right_y": 0.0,
            "left_trigger": 0.0, "right_trigger": 0.0 },
  "buttons": { "a": false, "b": false, "x": false, "y": false,
               "lb": false, "rb": false, "start": false, "select": false,
               "xbox": false, "left_stick": false, "right_stick": false,
               "dpad_up": false, "dpad_down": false, "dpad_left": false, "dpad_right": false },
  "connected": false,
  "battery_pct": null
}
```

#### Bluetooth Management Page (`/bluetooth`)

- Allows pairing, connecting, and disconnecting the Xbox controller from the browser.
- Backend wraps `bluetoothctl` via `roomba_webui/bluetooth_manager.py`. No subprocess calls in route handlers.
- When no real Bluetooth data is available, the device list shows the mock entry (name: `"No device (mock)"`) with a clear `[MOCK]` badge. The scan, pair, and connect buttons remain functional — they call real `bluetoothctl` commands regardless.
- `BluetoothManager` interface (must be mockable for tests):

```python
class BluetoothManager:
    def scan(self, duration_seconds: int = 10) -> list[dict]: ...
    def pair(self, mac: str) -> bool: ...
    def connect(self, mac: str) -> bool: ...
    def disconnect(self, mac: str) -> bool: ...
    def trust(self, mac: str) -> bool: ...
    def list_devices(self) -> list[dict]: ...
    def get_connection_status(self, mac: str) -> dict: ...
```

#### Map Visualisation (`/map`)
- Side-by-side layout: live map canvas on the left, controls panel on the right — no scrolling required.
- Renders OccupancyGrid as a `<canvas>` element.
- Colour coding: free = white, occupied = black, unknown = grey (`#888`), cursor (draw mode) = blue (`#58a6ff`).
- Robot pose overlaid as a directional arrow. Y-axis flipped so map origin is at canvas bottom-left.
- Auto-updates via `map_update` WebSocket event.
- When map channel is mock, a subtle `[MOCK MAP]` watermark is shown on the canvas.
- **Saved maps table** in right panel showing Name, Size, View/Delete buttons. Permanent "⚡ Live" row always first. "Back to Live" button shown when viewing a saved map.
- **Mode controls** (IDLE/MANUAL/RECON buttons) integrated in the map controls bar for switching modes while watching the map. RECON mode has distinct amber styling.
- **Simulation controls** (auto-shown when sim data detected): New Map, Reset Pose, Pause/Resume, Seed input. Ground Truth toggle for overlay comparison.
- **Event toasts:** RECON_COMPLETE, GOAL_FAILED, SIM_MAP_REGENERATED, SIM_POSE_RESET shown as notifications.

---

## 4. ROBOT OPERATING MODES

| Mode | Description | cmd_vel source | SLAM active | Recon active |
|---|---|---|---|---|
| `IDLE` | Stationary, all nodes running | None | Yes | No |
| `MANUAL` | Operator-driven via Xbox controller | Xbox node | Yes | No |
| `RECON` | Autonomous frontier exploration | recon_node | Yes | Yes |

- Mode is stored as a ROS2 parameter on the parameter server: `/robot/mode`.
- Mode transitions must be logged with timestamp to the `sessions` DB table.
- **Only one source of cmd_vel must be active at any time.** Implement a `cmd_vel_mux` or use topic remapping to enforce this. A safety interlock must prevent RECON node from publishing while in MANUAL mode and vice versa.

---

## 5. LAUNCH ARCHITECTURE

#### `roomba_bringup/launch/full_system.launch.py`
Must launch, in order:
1. `joy_linux_node` (from `joy_linux` package — **not** SDL2 `joy_node`)
2. `ldlidar_stl_ros2` LIDAR driver — publishes `/scan` from `/dev/ttyAMA0`
3. `motor_controller`
4. `joy_control_node`
5. `slam_toolbox` (online async, config from `config/slam_params.yaml`)
6. `nav2` stack (config from `config/nav2_params.yaml`)
7. `recon_node`
8. `db_node`
9. `roomba_webui`

> **Note:** The LIDAR driver (step 2) should begin publishing `/scan` within 1–2 seconds of launch. `slam_toolbox` (step 5) starts after a brief delay to allow the first scans to arrive.

Each node must be wrapped in a `ComposableNode` where possible, otherwise standalone. All nodes must have `respawn=True` and `respawn_delay=2.0` to handle transient crashes.

---

## 6. CONFIGURATION FILES

All tuneable parameters must live in YAML files under `config/`. Source files must never contain magic numbers. Required config files:

- `config/hardware.yaml` — LIDAR serial port, motor GPIO pins, wheel geometry, PWM frequency
- `config/controller.yaml` — button mapping, velocity scaling
- `config/slam_params.yaml` — slam_toolbox parameters
- `config/nav2_params.yaml` — nav2 planner, controller, costmap params
- `config/webui.yaml` — host, port, update rates

**Minimum required entries for `hardware.yaml`:**
```yaml
lidar:
  serial_port: "/dev/ttyAMA0"    # Pi 5 PL011 UART
  baud_rate: 230400              # LD14P protocol baud rate
  frame_id: "laser_frame"
  topic: "/scan"
  range_min: 0.02                # metres (LD14P minimum)
  range_max: 8.0                 # metres (LD14P maximum)

motors:
  pwm_frequency_hz: 1000
  wheel_separation_m: 0.20      # TODO: measure actual robot
  wheel_radius_m: 0.033         # TODO: measure actual robot
  left_pwm_pin: 12              # TODO: confirm wiring
  left_dir_pin: 16
  right_pwm_pin: 13
  right_dir_pin: 20
  watchdog_timeout_ms: 500
```

---

## 7. TESTING REQUIREMENTS

- Every ROS2 node must have a corresponding test file in `/tests/`.
- **C++ nodes** use Google Test via `ament_cmake_gtest`. Test files: `/tests/test_<node_name>.cpp`.
- **Python nodes** use `pytest` + `rclpy` mock patterns. Test files: `/tests/test_<node_name>.py`.
- `esp32_sensor_node` C++ tests must mock the I2C file descriptor (`open`/`read`/`write`) using a test double or dependency injection — never require real hardware for unit tests. **[REMOVED — esp32_sensor_node no longer exists]**
- Minimum test coverage: happy path + one fault injection per node (e.g., sensor timeout, DB unavailable, controller disconnect).
- Hardware-dependent tests (GPIO, motor) must be skippable via a `RUN_HW_TESTS=1` environment flag. In C++ tests, use `GTEST_SKIP()` when the flag is not set.

---

## 8. SUGGESTED FUTURE ENHANCEMENTS `[OPTIONAL / NOT IN SCOPE v1.0]`

These are out of scope for the initial implementation but the architecture must not preclude them:

- Camera module (Pi Camera v3) integration for visual SLAM or object detection.
- LIDAR sensor upgrade (e.g. RP LIDAR A1) if higher range or resolution is needed.
- Additional sensors (e.g. IMU, IR cliff sensors, bumper switches) via ESP32 coprocessor or direct Pi GPIO.
- Battery voltage monitoring via ESP32 ADC or INA219 I2C sensor, with low-battery alerts in the web UI.
- Multi-room map stitching and named zone support.
- REST API authentication (JWT) for web UI when exposing beyond LAN.
- OTA firmware update for Pi software via the web interface.
- Scheduled cleaning / patrol routes from stored maps.

---

## 9. CODING RULES FOR LLM AGENT — NON-NEGOTIABLE

1. **One node, one file.** Never combine two ROS2 nodes in a single source file, regardless of language.
2. **Language is not a choice.** Follow the language assignment table in Section 2.1 exactly. Do not implement a real-time node in Python and do not implement DB or web nodes in C++ without explicit approval.
3. **Config over constants.** All pin numbers, speeds, thresholds, serial ports, and topic names must be ROS2 parameters or loaded from YAML. No hardcoded values in any language.
4. **Fail loudly.** On startup, each node must validate its required parameters and abort with a human-readable error if any are missing or out of range. C++ nodes throw `std::runtime_error`; Python nodes raise `RuntimeError`.
5. **Log levels matter.** C++: use `RCLCPP_DEBUG/INFO/WARN/ERROR` macros. Python: use `self.get_logger().debug/info/warn/error`. Use `DEBUG` for telemetry, `INFO` for state changes, `WARN` for recoverable issues, `ERROR` for failures. Never use `std::cout`, `printf`, or `print()` for logging.
6. **No blocking calls on the main thread.** Use timers, callbacks, and executors. C++: never call `std::this_thread::sleep_for` in a callback. Python: never call `time.sleep()` in a callback. I2C reads must be in a `WallTimer` callback, not in the constructor.
7. **Thread safety in C++.** Shared state between callbacks and any async handlers must be protected by `std::mutex`. In Python: use `threading.Lock` for shared state between ROS callbacks and Flask/SocketIO handlers.
8. **Explicit over implicit.** Prefer verbose, readable code over clever one-liners in both languages. This codebase will be maintained by non-experts.
9. **Document every public symbol.** C++ public functions and classes: Doxygen `/** @brief ... */` comments. Python public functions and classes: Google-style docstrings. All ROS2 service and topic interfaces must be documented at the point of declaration.
10. **Commit to types.** C++: use strongly typed structs, avoid `void*` and raw arrays in interfaces. Python: use `dataclasses` or `TypedDict` for structured data, avoid raw `dict` passing between modules.
11. **Ask, don't assume.** If a requirement is underspecified (e.g. motor driver model not yet confirmed, I2C register layout for new ESP32 commands), generate a stub with a clearly marked `// TODO:` (C++) or `# TODO:` (Python) comment describing what needs to be resolved.
12. **No mode flags for data source.** The web UI must never check an environment variable, config flag, or any boolean to decide whether to use real or mock data. Data source is determined solely by whether a ROS2 topic has been heard within the channel timeout. All mode-switching logic lives in `DataChannel.is_live()` and nowhere else.

---

---

## 10. STARTUP & ENVIRONMENT SCRIPTS

Two shell scripts are required deliverables. They live at the root of the workspace (`roomba_ws/`) and must be kept up to date whenever dependencies or startup procedures change.

### 10.1 `setup.sh` — Tiered Startup Script

`setup.sh` is the single entry point for starting the robot. It supports **partial operation modes** so the system can be brought up incrementally (e.g. web UI only for development, or hardware only for sensor testing).

#### Usage

```
./setup.sh [MODE] [--dry-run] [--no-kill]

Modes:
  kill        Kill all roomba processes and tmux session, then exit
  demo        Web UI only, no ROS2, no hardware — uses mock data (default if no arg given)
  web         Web UI + DB node only — ROS2 running but no hardware nodes
  bt-test     Web + DB + bt_sim_node + joy_control_node — simulated controller
  controller  Web + joy_linux_node + joy_control_node — real Xbox controller
  draw-test   Web + DB + draw_node + joy_linux_node — draw on map with controller, save to DB
  sensor-test LIDAR + static TF + SLAM + DB + Web UI — real LIDAR data, no motors
  hardware    Hardware nodes only (sensors + motors) — no UI
  full        Full system — all nodes + web UI
  help        Print this message

Options:
  --dry-run   Print what would be started without actually starting anything
  --no-kill   Skip the auto-kill step (use to layer on top of running components)

Development Stages (recommended progression):
  Stage 1 – Web UI dev          →  demo
  Stage 2 – Web + ROS2 bridge   →  web
  Stage 3 – Simulated gamepad   →  bt-test
  Stage 4 – Real Xbox testing   →  controller
  Stage 4b – Draw/DB testing    →  draw-test
  Stage 4c – LIDAR bench test   →  sensor-test
  Stage 5 – Sensor bench test   →  hardware
  Stage 6 – Full integration    →  full
```

#### Behaviour Requirements

- Each mode must **check prerequisites** before launching and print a clear human-readable error and exit with code 1 if any prerequisite is missing. Prerequisites checked per mode:

| Mode | Prerequisites checked |
|---|---|
| `kill` | None |
| `demo` | Python 3.11+, Flask, Flask-SocketIO, eventlet installed |
| `web` | All `demo` checks + ROS2 sourced, workspace built |
| `bt-test` | All `web` checks |
| `controller` | All `web` checks + `xpadneo` loaded (warn), `joy_linux` package available |
| `draw-test` | All `web` checks + Docker running, `xpadneo` loaded (warn), `joy_linux` package available |
| `sensor-test` | All `web` checks + LIDAR serial port accessible (`/dev/ttyAMA0` exists), `slam_toolbox` package available |
| `hardware` | All `web` checks + LIDAR serial port accessible (`/dev/ttyAMA0` exists), I2C bus accessible (`/dev/i2c-1` exists) |
| `full` | All `hardware` checks + `xpadneo` loaded (`lsmod | grep xpadneo`), `joy_linux` package available, `slam_toolbox` package available |

- **Auto-kill:** Before every start (unless `--no-kill` is passed), the script kills all stale roomba processes (by tmux session, by process name patterns, and by port 80 or 5000) to ensure a clean slate. The `kill` mode does only this, then exits.
- After prerequisite checks pass, the script must print a startup summary showing which components will be started.
- Each component is started in a **separate `tmux` window** so logs are always visible and components can be individually restarted. Uses `tmux send-keys` (not shell arguments) to avoid `/bin/sh` `source` compatibility issues.
- The script must trap `SIGINT` and `SIGTERM` and cleanly shut down all started components on exit.
- Startup order within `full` mode must match Section 5 (Launch Architecture).
- A `--dry-run` flag must print what would be started without actually starting anything.

#### Venv / ROS2 Interaction — CRITICAL

The Python venv and ROS2 environment interact in ways that require careful handling:

1. **C++ nodes via `ros2 run`:** The venv must be **deactivated** before running C++ nodes. If the venv is active, `ros2 run` may fail to find C++ executables due to Python path pollution. The script uses `source_ros2_cmd()` which explicitly runs `deactivate` and `unset VIRTUAL_ENV` before sourcing ROS2.
2. **Python nodes needing venv packages (Flask, eventlet, SQLAlchemy):** These cannot be launched via `ros2 run` because `ros2 run` uses the system Python, which does not have venv packages. Instead, launch directly with `python3 -m <module>` after activating both ROS2 and the venv. The script uses `venv_ros2_cmd()` for this.
3. **Package verification:** Do not use `ros2 pkg list` inside a venv — it may fail. Use filesystem checks (e.g., `[[ -d /opt/ros/jazzy/share/joy_linux ]]`) instead.

#### Partial Mode Component Map

| Component | demo | web | bt-test | controller | draw-test | sensor-test | simulate-hw | hardware | full |
|---|---|---|---|---|---|---|---|---|---|
| Flask web server (DEMO mode) | ✓ | — | — | — | — | — | — | — | — |
| Flask web server (ROS2 mode) | — | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | — | ✓ |
| `db_node` | — | ✓ | ✓ | — | ✓ | ✓ | ✓ | — | ✓ |
| `bt_sim_node` | — | — | ✓ | — | — | — | — | — | — |
| `draw_node` | — | — | — | — | ✓ | — | — | — | — |
| `sim_sensor_node` | — | — | — | — | — | — | ✓ | — | — |
| `sim_motor_node` | — | — | — | — | — | — | ✓ | — | — |
| `sim_goal_follower` | — | — | — | — | — | — | ✓ | — | — |
| `joy_control_node` | — | — | ✓ | ✓ | — | — | ✓ | — | ✓ |
| `joy_linux_node` (real Xbox) | — | — | — | ✓ | ✓ | — | ✓ | — | ✓ |
| `ldlidar_stl_ros2` (LIDAR) | — | — | — | — | — | ✓ | — | ✓ | ✓ |
| `odom→base_link` static TF | — | — | — | — | — | ✓ | — | — | — |
| `motor_controller` | — | — | — | — | — | — | — | ✓ | ✓ |
| `slam_toolbox` | — | — | — | — | — | ✓ | ✓ | — | ✓ |
| `nav2` stack | — | — | — | — | — | — | — | — | ✓ |
| `recon_node` | — | — | — | — | — | — | ✓ | — | ✓ |
| ESP32 I2C (motor coprocessor) | — | — | — | — | — | — | — | ✓ | ✓ |

### 10.2 `environment.sh` — Environment Reproduction Script

`environment.sh` is a **fully idempotent** script that installs every dependency needed to build and run the project from a clean Ubuntu Server 24.04 LTS install. Running it twice must produce the same result without errors. Every command must be non-interactive (`-y` flags, `DEBIAN_FRONTEND=noninteractive`).

The script is divided into clearly labelled sections, each independently re-runnable:

The script supports two modes:
- **`./environment.sh`** — Full install + verification (default)
- **`./environment.sh --check`** — Verify-only: skip all installs, run 103-check verification suite to confirm the environment matches the project spec. Use this on a device that was already set up.

The script is divided into clearly labelled sections, each independently re-runnable:

1. **System packages** — apt update, base tools (`git`, `curl`, `wget`, `tmux`, `build-essential`, `cmake`, `python3-pip`, `python3-venv`)
2. **UART / Serial** — Enable UART0 on Pi 5, disable serial console on `/dev/ttyAMA0`, install `python3-serial` (for LIDAR driver diagnostics)
3. **I2C for ESP32** — enable I2C bus (`dtparam=i2c_arm=on`), install `i2c-tools`, load `i2c-dev` module, add user to `i2c` group
4. **ROS2 Jazzy** — full install via official apt repo (locale, keys, sources, `ros-jazzy-ros-base`)
5. **ROS2 packages** — `ros-jazzy-slam-toolbox`, `ros-jazzy-nav2-*`, `ros-jazzy-joy`, `ros-jazzy-teleop-twist-joy`
6. **Python venv** — create `roomba_ws/.venv`, install all Python deps from `requirements.txt`
7. **xpadneo** — install `dkms`, `linux-headers`, clone and install xpadneo
8. **bluez** — install `bluez`, enable `bluetooth` systemd service
9. **WiFi Access Point & Networking** — install `hostapd`, `dnsmasq`, `iw`; unmask hostapd (Ubuntu masks it after install); disable standalone dnsmasq (conflicts with systemd-resolved); create virtual AP interface `ap0`; write `hostapd.conf` (SSID `Roomba`, WPA2, **mode 600** to protect passphrase); write `dnsmasq.conf` idempotently (DHCP on ap0 10.0.0.10–50, DNS on both interfaces resolving `roomba.local`); `roomba-ap-start.sh` validates wlan0 before creating ap0; create and enable `roomba-ap.service` systemd unit; download `socket.io.min.js` with **sha256 integrity check**; set `cap_net_bind_service` on `python3.12` (bind port 80 without root); register ROS2 library paths in `/etc/ld.so.conf.d/ros2-jazzy.conf` via `ldconfig` (required because Linux capabilities cause the dynamic linker to ignore `LD_LIBRARY_PATH`)
10. **Workspace build** — `source /opt/ros/jazzy/setup.bash`, `colcon build --symlink-install`
11. **Environment file** — append ROS2 source and workspace overlay to `~/.bashrc` (idempotent, check before appending)
12. **Post-install verification** — run a check for each critical component and print a pass/fail summary table

#### Verification Summary (printed at end of `environment.sh`)

```
=== ROOMBA ENVIRONMENT VERIFICATION ===

━━━ 1. Core Software ━━━        (8 checks: ROS2, colcon, tmux, cmake, git, curl, Python 3.11+)
━━━ 2. ROS2 Packages ━━━        (14 checks: slam_toolbox, nav2 suite, joy, joy_linux, gtest)
━━━ 3. Python Virtual Env ━━━   (9 checks: venv, system-site-packages, all pip packages)
━━━ 4. pigpio ━━━               (3 checks: library, systemd unit, service active)
━━━ 5. LIDAR UART ━━━           (3 checks: serial port exists, UART enabled, console disabled)
━━━ 6. Xbox Controller ━━━      (6 checks: bluez, bluetooth, dkms, xpadneo reg/module/config)
━━━ 7. WiFi AP & Networking ━━━ (24 checks: hostapd/dnsmasq install+config, AP service, ap0 IP,
                                  hostapd.conf mode 600, dnsmasq standalone disabled,
                                  cap_net_bind_service on python3.12, ROS2 ldconfig)
━━━ 8. Workspace & Build ━━━    (9 checks: src/, 6 packages, install/)
━━━ 9. Config Files ━━━         (7 checks: all 5 YAMLs, port 80, host 0.0.0.0)
━━━ 10. Web UI Assets ━━━       (7 checks: socket.io bundled, no CDN, core .py files)
━━━ 11. Shell Environment ━━━   (4 checks: ~/.bashrc entries)
━━━ 12. Test Skeletons ━━━      (12 checks: all 12 test files)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Total checks: 103
  PASS: 101  │  FAIL: 0  │  WARN: 2
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
===
```

Hardware-dependent checks (ESP32, controller) print `[WARN]` not `[FAIL]` since they require physical hardware to be connected.

---

*End of requirements document — v2.3*