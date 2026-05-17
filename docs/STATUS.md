# Recon-Platform-R2 — Project Status

> Current state as of 2026-05-10. For roadmap → [`ROADMAP.md`](ROADMAP.md);
> for the spec → [`SPEC.md`](SPEC.md).

---

## 1. Executive summary

The handheld pivot is complete on the `handheld` branch. Four stages
have landed since 2026-05-10 (`5088514` → `2bcacca`); the codebase is
clean, builds, and has been smoke-tested end-to-end through the
`setup.sh demo` and `setup.sh web` modes. `setup.sh sensor-test`
(real LIDAR + SLAM + web UI) is wired and ready — the user has run it
on bench hardware. The ESP32 firmware skeleton is committed but not
yet flashed; the Pi-side bridge that consumes its frames lands in H2.1.

Branch state:

```
handheld   2bcacca  Stage H2.0: ESP32 I/O hub firmware skeleton
           f20d4f3  Stage H1.6: live-pose viewport, faster rates, dynamic-obstacle SLAM tune
           6c315cb  Stage H1.5: cleanup pass — strip vestigial autonomous-robot code
           5088514  Stage H1: pivot to handheld — delete autonomy/motors/BT, rename roomba_* → recon_*
main       c4f4c0a  Stage 5: LD14P LIDAR bench test complete  (frozen pre-pivot)
```

`handheld` is pushed to `origin/handheld`.

---

## 2. Codebase metrics

| Metric                                       | Value          |
| -------------------------------------------- | -------------- |
| ROS2 packages (`recon_*` + vendored ldlidar) | 5 + 1          |
| C++ source files (own code)                  | 2 (`sim_sensor_node.cpp`, `draw_node.cpp`) |
| Python source files (recon_db + recon_webui) | 9              |
| ESP32 firmware files                         | 9 (5 own + main + 3 build files) |
| HTML templates                               | 3 (`base.html`, `index.html`, `map.html`) |
| YAML config files                            | 4 (`webui`, `slam_params`, `hardware`, `simulation`) |
| Test files                                   | 4 (2 Python, 2 C++) |
| Test cases                                   | 19 (15 Python pytest + 4 C++ gtest) |

---

## 3. Per-package status (Pi-side)

| Package           | State                                                                                    |
| ----------------- | ---------------------------------------------------------------------------------------- |
| `recon_hardware`  | ✅ Builds. `sim_sensor_node` trimmed to LIDAR-only (397 lines, was 952). Publishes `/scan` and `/sim/ground_truth` from a static spawn pose. |
| `recon_control`   | ✅ Builds. `draw_node` rewritten for `/draw/command` text protocol (no /joy). Awaits H5 web UI to drive it. |
| `recon_db`        | ✅ Builds + tested. `db_node` saves on `/robot/events:"SAVE_MAP"`, emits `MapEvent` rows. `RECON_DB_URL` env var. 9/9 pytest. |
| `recon_webui`     | ✅ Builds + tested. Flask + SocketIO + embedded `RosBridge`. Pose-from-TF wired (no /scanner/pose publisher needed yet). 6/6 pytest. |
| `recon_bringup`   | ✅ Builds. `full_system.launch.py` is a placeholder (db_node + recon_webui only) — not the canonical entry point; setup.sh is. |
| `ldlidar_stl_ros2`| ✅ Builds (vendored submodule). Pre-existing test failures in the submodule — not in our scope. |

---

## 4. Per-component status

### 4.1 Hardware

| Component                    | Hardware on hand | Wired & verified |
| ---------------------------- | ---------------- | ---------------- |
| Pi 5 (4 GB)                  | ✅                | ✅                |
| LD14P LIDAR                  | ✅                | ✅ (Stage 5)      |
| ESP32 DevKit V1              | ✅                | ⏳ Awaits flashing of H2.0 firmware |
| MPU-6050 IMU                 | ✅ (assumed)      | ⏳                |
| 3 buttons + enclosure        | ⏳ TBD            | ⏳                |
| Battery + SPST + buck        | ⏳ TBD            | ⏳                |

### 4.2 Pi software

| Layer            | State                                                                |
| ---------------- | -------------------------------------------------------------------- |
| `environment.sh` | ✅ Idempotent install + `--check` mode, post-pivot deps only          |
| `setup.sh`       | ✅ 4 modes: kill / demo / web / sensor-test                            |
| ROS2 Jazzy       | ✅ Installed at `/opt/ros/jazzy/`                                      |
| Python venv      | ✅ `roomba_ws/.venv` with `--system-site-packages`                     |
| Docker + Postgres | ✅ Container `roomba_postgres`, healthy, schema auto-created           |
| WiFi AP "Recon"  | ✅ `recon-ap.service` enabled (legacy `roomba-ap` cleanup automatic)  |
| `cap_net_bind_service` + ldconfig | ✅ Web UI binds port 80 without root                |

### 4.3 ROS2 layer

| Topic / TF              | Producer state                                                 |
| ----------------------- | -------------------------------------------------------------- |
| `/scan`                 | ✅ `ldlidar_node` — verified ~10 Hz on real LD14P               |
| `/map`                  | ✅ `slam_toolbox` — `map_update_interval: 1.0` (was 5.0)         |
| `/tf` (`map → odom`)    | ✅ `slam_toolbox` — verified live                                |
| `/tf` (`odom → base_link`) | ✅ static identity from `setup.sh sensor-test`                |
| `/tf` (`base_link → laser_frame`) | ✅ static, from ldlidar launch                          |
| `/imu/data_raw`         | ⏳ H2.1 — Pi-side bridge consumes ESP32 frames                  |
| `/imu/data`             | ⏳ H3 — `imu_filter_madgwick`                                   |
| `/odom` (real)          | ⏳ H3 — `robot_localization` ekf_node                            |
| `/scanner/pose`         | ⏳ H3 — small republisher of `/odom.pose`. Until then, web bridge derives from TF. |
| `/robot/events`         | ✅ Live channel: SAVE_MAP and DELETED events                     |
| `/robot/mode`           | ✅ Live channel: IDLE / SCAN toggle                              |
| `/draw/command`         | ⏳ H5 — web UI publisher                                         |

### 4.4 Web UI

| Page / route                  | State                                                       |
| ----------------------------- | ----------------------------------------------------------- |
| `/` (Dashboard)               | ✅ Live pose, mode toggle, event log                         |
| `/map` (Live Map)             | ✅ Centered viewport, pose-anchored panning, save/list maps  |
| `/api/robot/status`           | ✅                                                            |
| `/api/robot/mode` (POST)      | ✅                                                            |
| `/api/maps` (GET/POST)        | ✅                                                            |
| `/api/maps/<id>` (GET/PUT/DELETE) | ✅                                                       |
| `/api/maps/<id>/data` (GET)   | ✅                                                            |
| `/api/maps/events` (GET)      | ✅                                                            |
| `/api/debug/channels`         | ✅                                                            |
| WebSocket `robot_pose`        | ✅ 10 Hz (was 5)                                              |
| WebSocket `map_update`        | ✅ 5 Hz (was 2)                                               |
| WebSocket `channel_status`    | ✅ 0.5 Hz                                                     |

### 4.5 ESP32 firmware

| Component                     | State                                                       |
| ----------------------------- | ----------------------------------------------------------- |
| PlatformIO project            | ✅ `firmware/esp32/platformio.ini` (`board=esp32dev`, `framework=arduino`) |
| `main.cpp` + scheduler        | ✅ Cooperative `millis()` deadlines                          |
| MPU-6050 driver (Wire.h only) | ✅ Register-level, no external libs, ±4 g / ±500 °/s @ 100 Hz |
| Button handler (debounce + long-press) | ✅                                                  |
| UART framing + CRC8           | ✅ `[0xA5 0x5A][TYPE][LEN][PAYLOAD][CRC8]`, Dallas/Maxim     |
| README + flash workflow       | ✅ [`firmware/esp32/README.md`](../firmware/esp32/README.md) |
| Bench-flashed                 | ⏳ Not yet — awaits H2.1 Pi-side decoder for round-trip verification |
| Unit tests                    | ❌ None yet (CRC8 + framing logic worth covering in H2.1)    |

---

## 5. Test coverage

### 5.1 Python (pytest, 15 cases)

| File                       | Cases | Covers                                          |
| -------------------------- | ----- | ----------------------------------------------- |
| `tests/test_db_node.py`    | 9     | Models, CRUD, session-map relationship, cascade |
| `tests/test_recon_webui.py`| 6     | DataChannel fallback timing, mock data shape    |

Run: `cd roomba_ws && .venv/bin/python -m pytest tests/`.

### 5.2 C++ (gtest via colcon, 4+ cases)

| File                           | Cases | Covers                                  |
| ------------------------------ | ----- | --------------------------------------- |
| `tests/test_draw_node.cpp`     | 4     | Grid layout, paint, clear, brush clamp  |
| `tests/test_sim_sensor_node.cpp`| 4    | Raycast wall hit, max range, room connectivity (BFS), obstacle blocking |

Run: `cd roomba_ws && colcon test`.

### 5.3 ESP32

❌ No unit tests yet. **Action item for H2.1:** add a host-side gtest
suite for the `framing::crc8()` and parser logic (parser is on the Pi
side anyway, so this lives in `recon_hardware` once H2.1 lands).

---

## 6. Smoke-test results (last run 2026-05-10)

| Mode                  | Result                                                                            |
| --------------------- | --------------------------------------------------------------------------------- |
| `setup.sh kill`       | ✅ Tears down tmux + processes cleanly                                              |
| `setup.sh demo`       | ✅ All routes 200; `pose` channel mock; `/scanner/pose` topic name visible          |
| `setup.sh web`        | ✅ db_node + recon_webui_bridge spawn; manual `/tf` publish flips pose channel live |
| `setup.sh sensor-test`| ⏳ User-verified on real hardware in earlier sessions; not re-run since H1.6        |

---

## 7. Known limitations & tech debt

| #  | Item                                                          | Severity | Notes                                                                                            |
| -- | ------------------------------------------------------------- | -------- | ------------------------------------------------------------------------------------------------ |
| 1  | No EKF / IMU fusion yet                                       | High     | `/odom` is a static identity TF. Stationary scanning works; walking will drift until H3.        |
| 2  | ESP32 firmware not flashed yet                                | High     | Code committed but no round-trip test. Blocks H2.1 bridge bring-up.                              |
| 3  | `/draw/command` has no publisher                              | Low      | `draw_node` runs idle; web UI driver lands in H5.                                                |
| 4  | Vendored `ldlidar_stl_ros2` test failures (pre-existing)      | Low      | Not in our scope; submodule is upstream code.                                                    |
| 5  | Postgres credentials still `roomba`/`roomba` in the container | Low      | `RECON_DB_URL` points at it. Renaming would lose existing scan data; left as deliberate carry-over. |
| 6  | No authentication on web UI                                   | Medium   | Designed for AP-only operation. Add basic auth before exposing on a LAN.                          |
| 7  | Scan-session state machine (IDLE → SCAN → SAVE) is just text  | Medium   | H5 introduces a real state machine that gates DB writes.                                          |
| 8  | No firmware unit tests                                        | Medium   | CRC8 + framing serialiser are pure functions — easy host-side gtest.                              |
| 9  | `full_system.launch.py` is a placeholder                      | Low      | setup.sh is the canonical entry point; the launch file just covers db_node + recon_webui.        |
| 10 | Pi-side bridge for ESP32 not built                            | High     | First task of H2.1.                                                                              |

---

## 8. Documentation

Active docs live in `docs/`:

| Doc                                    | Purpose                                       |
| -------------------------------------- | --------------------------------------------- |
| [`SPEC.md`](SPEC.md)                   | What the system is at every layer              |
| [`ARCHITECTURE.md`](ARCHITECTURE.md)   | How data moves through it                      |
| [`STATUS.md`](STATUS.md)               | This file                                      |
| [`ROADMAP.md`](ROADMAP.md)             | Forward plan, H2.1 → H6                        |
| [`AGENT_RULES.md`](AGENT_RULES.md)     | Binding rules for LLM/agent contributors       |
| [`UART_PROTOCOL.md`](UART_PROTOCOL.md) | ESP32 ↔ Pi binary framing canonical reference  |

Pre-pivot history is preserved in
[`docs/archive/2026-05-10_pre-handheld/`](archive/2026-05-10_pre-handheld/).
