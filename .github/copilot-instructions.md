# Roomba Project — Agent Instructions

Autonomous mapping robot: Pi 5, ROS2 Jazzy, C++17/Python 3.12, Flask+SocketIO web UI.

## Mandatory reading — consult before every task

Read the relevant project files before starting any work. These are the source of truth:

| File | What it contains | When to read |
|------|-----------------|--------------|
| `rules.md` | Binding constraints for all agent work (logging, docs, security, threading, testing) | **Every task** — this is non-negotiable |
| `project_requirements.md` | Full technical spec: hardware, software stack, module specs, language table, coding rules (Section 9), script requirements (Section 10) | When implementing features, adding nodes, or changing architecture |
| `project_status.md` | Current progress, completed stages, known issues, tech debt, roadmap | Before starting work (know what's done) and after finishing (update it) |
| `roomba_ws/docs/esp32_firmware.md` | ESP32 I2C register map, wiring, firmware contract | When working on `roomba_hardware` or sensor code |
| `roomba_ws/config/*.yaml` | All runtime configuration (controller, hardware, webui, slam, nav2) | When adding parameters, changing behaviour, or debugging config |

## Scripts — use them, update them

| Script | Purpose | When to use |
|--------|---------|-------------|
| `roomba_ws/setup.sh` | Tiered startup (`demo`, `web`, `bt-test`, `controller`, `hardware`, `full`) | To run/test the system. If you add a node or dependency, update the relevant mode |
| `roomba_ws/environment.sh` | Idempotent provisioning (system packages, ROS2, pigpio, xpadneo, bluez, WiFi AP). Has `--check` mode with 103 verification checks | If you add a system dependency, add it here. Run `--check` to verify the environment |

When adding a new ROS2 node, startup mode, or system dependency:
1. Add the node to the language table in `project_requirements.md` Section 2.1
2. Add a launcher function in `setup.sh` and wire it into the appropriate mode(s)
3. Add install steps to `environment.sh` if new system packages are needed
4. Add verification checks to `environment.sh --check` for the new dependency

## Key constraints (full details in rules.md)

- **Documentation:** Update `project_status.md` and docstrings on every change
- **Logging:** All I/O, subprocess, and state transitions must log using `get_logger(__name__)` (Python) or `RCLCPP_*` macros (C++). Never `print()`/`std::cout`
- **Language:** C++17 for real-time nodes, Python for web/DB — follow `project_requirements.md` Section 2.1 exactly
- **Config:** All parameters in YAML — zero hardcoded values
- **Security:** Validate external input at system boundaries (MAC regex, API payload types, HTML escaping)
- **Threading:** `eventlet.monkey_patch()` first in `app.py`; never `socketio.emit()` from rclpy threads; `tpool.execute()` for blocking calls
- **Testing:** Every node needs a test skeleton in `roomba_ws/tests/`
- **Web UI:** No CDN — all assets bundled locally. Bluetooth page reflects live `bluetoothctl` state
- **Process:** Read code before modifying. Verify syntax after editing. Scope to the request — don't over-engineer
