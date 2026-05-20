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
handheld   <H3a>    Stage H3 (partial): imu_yaw_integrator + slam_toolbox imu_topic (this commit)
           2d7d444  setup.sh polish: WEBUI_PORT-aware kill, robust TF teardown, mode-specific attach hint
           168947d  .gitignore expanded
           fb3a93f  Stage H2.1: Pi-side esp32_uart_bridge + /stats page
           584f8e2  firmware/esp32/environment.sh one-shot provisioner
           a26089c  firmware: bench-validated end-to-end on real hardware
           363e3ea  ESP32 firmware: 4-pin button wiring guidance + bench-test decoder
           6364494  Docs overhaul — fresh post-pivot documentation set
           2bcacca  Stage H2.0: ESP32 I/O hub firmware skeleton
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
| `recon_hardware`  | ✅ Hybrid C++/Python (ament_cmake + ament_python). `sim_sensor_node` (C++) + `esp32_uart_bridge.py` (H2.1) + `imu_yaw_integrator.py` (H3) + `framing.py` parser. Publishes `/scan`, `/imu/data_raw`, `/imu/data`, `/buttons/*`, `/esp32/diagnostics`. |
| `recon_control`   | ✅ Builds. `draw_node` rewritten for `/draw/command` text protocol (no /joy). Awaits H5 web UI to drive it. |
| `recon_db`        | ✅ Builds + tested. `db_node` saves on `/robot/events:"SAVE_MAP"`, emits `MapEvent` rows. `RECON_DB_URL` env var. 9/9 pytest. |
| `recon_webui`     | ✅ Builds + tested. Flask + SocketIO + embedded `RosBridge`. New `imu`/`bridge_health` DataChannels; new `/stats` page + `/api/stats`. 6/6 pytest. |
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
| `/imu/data_raw`         | ✅ H2.1 — `esp32_uart_bridge` publishes at ~100 Hz, BEST_EFFORT QoS |
| `/imu/data`             | ✅ H3 (partial) — `imu_yaw_integrator` republishes with orientation quaternion populated from integrated gyro Z (yaw only; roll/pitch=0). 100 Hz, RELIABLE QoS for slam_toolbox compatibility. |
| `/esp32/diagnostics`    | ✅ H2.1 — JSON link health (frame counts, port, ESP32 uptime, boot STATUS) @ 1 Hz |
| `/buttons/save`, `/buttons/reset`, `/buttons/shutdown_request`, `/buttons/shutdown_longpress` | ✅ H2.1 — std_msgs/Empty edges from the ESP32 |
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
| `/stats` (Telemetry, H2.1)    | ✅ ESP32 link health + IMU live values w/ sparklines + SLAM stats |
| `/api/robot/status`           | ✅                                                            |
| `/api/robot/mode` (POST)      | ✅                                                            |
| `/api/maps` (GET/POST)        | ✅                                                            |
| `/api/maps/<id>` (GET/PUT/DELETE) | ✅                                                       |
| `/api/maps/<id>/data` (GET)   | ✅                                                            |
| `/api/maps/events` (GET)      | ✅                                                            |
| `/api/stats` (H2.1)           | ✅ Combined IMU + bridge_health + SLAM + pose snapshot       |
| `/api/debug/channels`         | ✅                                                            |
| WebSocket `robot_pose`        | ✅ 10 Hz                                                      |
| WebSocket `map_update`        | ✅ 5 Hz                                                       |
| WebSocket `imu_data` (H2.1)   | ✅ 20 Hz (decimated from 100 Hz on the wire)                  |
| WebSocket `bridge_health` (H2.1) | ✅ 1 Hz JSON                                              |
| WebSocket `stats_update` (H2.1)  | ✅ 2 Hz consolidated snapshot                              |
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
| Bench-flashed                 | ✅ Round-trip-validated 2026-05-17 via `decode_serial.py` + 2026-05-20 via the Pi-side bridge |
| Unit tests                    | ✅ H2.1 — 12 pytest cases in `tests/test_esp32_uart_bridge.py` covering CRC8, all 4 frame types, resync, bad CRC, oversized LEN, chunked input |

---

## 5. Test coverage

### 5.1 Python (pytest, 27 cases)

| File                          | Cases | Covers                                              |
| ----------------------------- | ----- | --------------------------------------------------- |
| `tests/test_db_node.py`       | 9     | Models, CRUD, session-map relationship, cascade     |
| `tests/test_recon_webui.py`   | 6     | DataChannel fallback timing, mock data shape        |
| `tests/test_esp32_uart_bridge.py` (H2.1) | 12 | CRC8 + 4 frame round-trips + resync + bad CRC + oversized LEN + chunked input |
| `tests/test_imu_yaw_integrator.py` (H3) | 12 | Yaw integration step (dt clamp, wrap), quaternion form, unit-norm |

Run: `bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && source .venv/bin/activate && pytest tests/'`.

### 5.2 C++ (gtest via colcon, 4+ cases)

| File                           | Cases | Covers                                  |
| ------------------------------ | ----- | --------------------------------------- |
| `tests/test_draw_node.cpp`     | 4     | Grid layout, paint, clear, brush clamp  |
| `tests/test_sim_sensor_node.cpp`| 4    | Raycast wall hit, max range, room connectivity (BFS), obstacle blocking |

Run: `cd roomba_ws && colcon test`.

---

## 6. Smoke-test results (last run 2026-05-10)

| Mode                  | Result                                                                            |
| --------------------- | --------------------------------------------------------------------------------- |
| `setup.sh kill`       | ✅ Tears down tmux + processes cleanly                                              |
| `setup.sh demo`       | ✅ All routes 200; `/stats` renders with mock IMU + mock bridge_health              |
| `setup.sh web`        | ✅ db_node + recon_webui_bridge spawn; manual `/tf` publish flips pose channel live |
| `setup.sh imu-test`   | ✅ H2.1 + H3 — ESP32 bridge + yaw integrator + Web UI. `/imu/data` @ 100.5 Hz with orientation populated (yaw integrated from gyro Z). Drift ~0.5 °/s with the chip's uncalibrated bias — acceptable; scan matching corrects it. |
| `setup.sh sensor-test --no-esp32` | ✅ User-verified on real hardware (Stage 5); existing behaviour                |
| `setup.sh sensor-test`| ⏳ LIDAR + ESP32 bridge layered together — to be verified once both are wired      |

---

## 7. Known limitations & tech debt

| #  | Item                                                          | Severity | Notes                                                                                            |
| -- | ------------------------------------------------------------- | -------- | ------------------------------------------------------------------------------------------------ |
| 1  | No EKF + no /odom from IMU yet                                | Medium   | H3 partial: `imu_yaw_integrator` publishes `/imu/data` (yaw quaternion) which slam_toolbox uses as a scan-match prior via its `imu_topic` param. Translation still comes entirely from scan matching; there's no `/odom` topic and the static odom→base_link is still in setup.sh. EKF + `/odom` from IMU lands in H3.1. |
| 2  | ESP32 accel Z reads ~15.3 m/s² with chip flat                 | Low      | Factory `ZA_OFFSET_USR = 1544` baked-in bias. Direction-of-gravity is correct; magnitude is off. Madgwick + EKF in H3 estimates and removes the bias online. **Do NOT zero the offset registers in firmware** — bit 0 of each L byte is a reserved temp-comp gate and clobbering it makes the calibration worse. |
| 3  | Boot STATUS diag may be missed by webui                        | Low      | The bridge publishes its boot STATUS frame ~0 s after open(); if the webui RosBridge subscribes after that point the frame is lost. Frame counts are still accurate; only the chip-identity dump is unavailable until next reset. |
| 4  | `/draw/command` has no publisher                              | Low      | `draw_node` runs idle; web UI driver lands in H5.                                                |
| 5  | Vendored `ldlidar_stl_ros2` test failures (pre-existing)      | Low      | Not in our scope; submodule is upstream code.                                                    |
| 6  | Postgres credentials still `roomba`/`roomba` in the container | Low      | `RECON_DB_URL` points at it. Renaming would lose existing scan data; left as deliberate carry-over. |
| 7  | No authentication on web UI                                   | Medium   | Designed for AP-only operation. Add basic auth before exposing on a LAN.                          |
| 8  | Scan-session state machine (IDLE → SCAN → SAVE) is just text  | Medium   | H5 introduces a real state machine that gates DB writes.                                          |
| 9  | `full_system.launch.py` is a placeholder                      | Low      | setup.sh is the canonical entry point; the launch file just covers db_node + recon_webui.        |

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
