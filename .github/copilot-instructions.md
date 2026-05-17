# Recon-Platform-R2 — Agent Instructions

Handheld LIDAR mapping device. Raspberry Pi 5 + ESP32 I/O hub.
ROS2 Jazzy, C++17 + Python 3.12, Flask + SocketIO web UI, PostgreSQL in Docker.

The repo pivoted from an autonomous mobile robot to a handheld scanner
on 2026-05-09. Anything that refers to motors, Bluetooth controllers,
joy_linux, fuzzy frontier exploration, or `roomba_*` package names is
historical — see [`docs/archive/2026-05-10_pre-handheld/`](../docs/archive/2026-05-10_pre-handheld/).

## Mandatory reading

Read these in order before any non-trivial task:

| File                                                  | What's in it                                       | When to read                                |
| ----------------------------------------------------- | -------------------------------------------------- | ------------------------------------------- |
| [`docs/STATUS.md`](../docs/STATUS.md)                 | Per-package state, what's working, known limits    | Every task — know the current baseline      |
| [`docs/ROADMAP.md`](../docs/ROADMAP.md)               | H1 → H6 stages with goals and acceptance criteria  | Every task — know which stage you're in     |
| [`docs/AGENT_RULES.md`](../docs/AGENT_RULES.md)       | Binding constraints (logging, threading, etc.)     | **Every task** — non-negotiable             |
| [`docs/SPEC.md`](../docs/SPEC.md)                     | Hardware + software contract at every layer        | When changing a topic, schema, or wire format |
| [`docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md)     | Data flow, TF tree, threading model                | When changing how data moves                |
| [`docs/UART_PROTOCOL.md`](../docs/UART_PROTOCOL.md)   | ESP32 ↔ Pi binary framing reference                | ESP32 firmware or Pi-side bridge work only  |
| [`firmware/esp32/README.md`](../firmware/esp32/README.md) | Wiring + PlatformIO build/flash workflow       | ESP32 firmware work only                    |
| [`roomba_ws/config/*.yaml`](../roomba_ws/config/)     | All runtime configuration                          | When adding parameters or debugging config  |

## Scripts — use them, update them

| Script                                       | Purpose                                                                                  |
| -------------------------------------------- | ---------------------------------------------------------------------------------------- |
| [`roomba_ws/setup.sh`](../roomba_ws/setup.sh) | **Canonical entry point.** 4 modes: `kill`, `demo`, `web`, `sensor-test`. Never bypass it. |
| [`roomba_ws/environment.sh`](../roomba_ws/environment.sh) | Idempotent provisioning. Run with `--check` to verify an already-set-up device. |

`setup.sh` activates the Python venv per tmux pane before launching
each node. **Calling `ros2 launch` / `ros2 run` directly will fail** with
`ModuleNotFoundError: sqlalchemy/eventlet` because nodes spawn in
subprocesses that don't inherit the venv. Confirmed empirically; the
rule is in [`AGENT_RULES.md`](../docs/AGENT_RULES.md) §9.

When adding a new ROS2 node, mode, or system dependency:

1. Add the node to [`docs/SPEC.md`](../docs/SPEC.md) §3.2 (package table)
   and §4 (topic graph).
2. Add a launcher function in [`setup.sh`](../roomba_ws/setup.sh)
   and wire it into the appropriate mode(s).
3. Add install steps to [`environment.sh`](../roomba_ws/environment.sh)
   if new system packages are needed.
4. Add verification checks to `environment.sh --check`.
5. Update [`docs/STATUS.md`](../docs/STATUS.md) to reflect the new state.

## Key constraints (summary; full rules in `docs/AGENT_RULES.md`)

- **Documentation:** Update `docs/STATUS.md` on every behaviour change. Inline docstrings on every public symbol.
- **Logging:** All I/O, subprocess, and state transitions log via `get_logger(__name__)` (Python) or `RCLCPP_*` (C++). Never `print()` / `std::cout`.
- **`db_node` uses f-strings** for logger calls — `RcutilsLogger` does not support %-style format args.
- **Language:** C++17 for real-time / hardware (`recon_hardware`, `recon_control`). Python 3.12 for non-RT (`recon_db`, `recon_webui`).
- **One node, one source file.**
- **Config over constants:** All tunables in YAML or ROS2 params. Zero hardcoded values. ESP32 pinout lives in `firmware/esp32/src/config.h`; LIDAR pinout in `roomba_ws/config/hardware.yaml`.
- **Security:** Validate all external input. HTML-escape values from outside sources. No secrets in code.
- **Threading (web UI):** `eventlet.monkey_patch()` first in `app.py`. Never `socketio.emit()` from rclpy threads — use the `queue.Queue` bridge. `tpool.execute()` for blocking calls in Flask routes.
- **Pose topic is `/scanner/pose`** (not `/roomba/pose` — that's gone).
- **DB env var is `RECON_DB_URL`** (not `ROOMBA_DB_URL`). Container is still `roomba_postgres` to preserve historical scans.
- **Testing:** Every node has at least one test. Run `pytest` + `colcon test` before claiming done.
- **Web UI:** No CDN. All assets bundled locally under `static/`.
- **ESP32 firmware:** No external libraries beyond `Arduino.h` + `Wire.h`. Frame format in `docs/UART_PROTOCOL.md` is canonical.
- **Process:** Scope to the request. Read before writing. Verify after editing. One stage = one commit (`Stage Hx.y: <subject>`).
