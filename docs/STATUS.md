# Recon-Platform-R2 — Project Status

> Current state as of 2026-07-04. For roadmap → [`ROADMAP.md`](ROADMAP.md);
> for the spec → [`SPEC.md`](SPEC.md);
> for outstanding work → [`REMAINING_ISSUES.md`](REMAINING_ISSUES.md).

---

## 1. Executive summary

The handheld pivot has landed through **H4 + LIDAR-through-ESP32 integration**
on the `handheld` branch. Full hardware stack walks end-to-end: LD14P
LIDAR → ESP32 → Pi (via USB-CDC at 460 800 baud) → pty → slam_toolbox,
with motor power gated by a Pi-controlled enable line so the LIDAR only
spins when the user presses Start scan. Tier-2 post-processing (median +
morphology + connected-component clustering, pure NumPy) cleans saved
maps and stores the result as per-cluster labels in `processed_maps`.

**2026-07-04 — map-quality fix set (cloudy maps + broken post-processing).**
Field maps saved through June rendered as fuzzy white clouds and the
Processed tab made them worse. Root causes found and fixed:

* *Map generation* — `slam_params.yaml`: `max_laser_range` 12 → 8 m (LD14P
  spec maximum; >8 m junk returns each carved a long radial free-space
  streak) and `minimum_travel_distance/_heading` 0.0 → 0.1 (a pose-graph
  node per scan layered duplicate wall estimates → smeared/doubled walls;
  verify on the next walk that the pose arrow still updates while standing
  still). `set_scanning(True)` now waits `webui.scan.lidar_spinup_s`
  (2.5 s, webui.yaml) between enabling the LIDAR motor and unpausing SLAM
  so spin-up revolutions never reach the pose graph.
* *Rendering* — walls drew as `#1a1a1a` on `#0a0a0a` unknown, invisible at
  the free/unknown boundary, so every map read as a white blob. New
  palette on both `/` and `/maps`: wall `#000`, unknown `#333` fog.
* *Post-processing* (`recon_db/postprocess.py`) — (1) new free-space
  opening stage dissolves the 1–2-cell "free" ray fans (10–17 % of free
  cells on field maps); removed cells revert to unknown. (2) Manhattan
  deskew tilt is now estimated from the merged Hough segments instead of
  the raw accumulator — accumulator concentration on real maps was
  0.16–0.21 against the 0.2 gate, so the deskew never fired; the segment
  resultant scores 0.75–0.99 on the same maps (gate 0.55 + 40-cell
  minimum wall length). Field map "1" now deskews −20.1° and reads
  *enclosed*. (3) Wall baking is conservative: only occupied cells within
  `wall_absorb` (4) dilations of a detected wall are absorbed; the old
  bake cleared *every* occupied cell to free, erasing all undetected
  structure and fabricating open floor. 68/68 postprocess tests (7 new),
  112/112 suite-wide.

Branch state (latest first):

```
handheld  HEAD     Auto-start at boot via recon-stack.service + env.sh Section 9.5 + check_sudo helper closes REMAINING_ISSUES M5; setup.sh stty pre-flight on /dev/ttyUSB0 unwedges the systemd-driven bridge boot
          a3dafcd  Map-processing overhaul — Hough Line Transform overlay + modal on /maps, no rainbow
          749535e  Web UI overhaul — instrument-panel theme, /=map, /maps + /diagnostics, hero status pill
          dc4c501  Docs + audit refresh (52/52 pytest, 8/8 gtest, 25/25 REST, REMAINING_ISSUES.md)
          81a7ee8  Clear-map UI now wipes the canvas locally on success (works while scan is paused)
          20d0f3a  Add 'Clear map' button + POST /api/map/clear → /slam_toolbox/reset
          5d5fea8  Fix silently-stalling map — LD14P fixed-beam geometry + auto-pause moved off rclpy spin thread
          e0a0a8a  Docs refresh — STATUS / ARCHITECTURE / SPEC / ROADMAP catch up through H4-prep + LIDAR-through-ESP32
          b8746e9  Inc 2 — LIDAR-through-ESP32 data path (Serial2 relay + pty + set_scanning lockstep)
          ddc66ea  Inc 1 — LIDAR motor power control (GPIO4 + S8050 + 3 s watchdog + /lidar_enable)
          49189dd  ros_bridge fix — Start/Pause uses slam_toolbox set_parameters (real type) instead of the misleading Pause toggle
          3ced924  environment.sh — drop unused madgwick, add std_srvs + python3-serial, document cap-strip
          7978580  setup.sh preflight — fall back to :8080 if cap_net_bind_service was stripped by apt upgrade
          2a29da6  Tier-2 post-processing for saved maps (median + morphology + connected components, NumPy-only)
          8a16c01  Start/Pause scan buttons on /map (slam_toolbox.paused_new_measurements via ros_bridge)
          f744e1b  setup.sh full mode + walking-trail polyline on /map
          2cd836b  Stage H3 — robot_localization ekf_node + ros_bridge TF composition
          6a372a9  Stage H3 partial — imu_yaw_integrator @ 100 Hz + slam imu_topic prior
          2d7d444  setup.sh polish — kill-mode, mode-specific attach hint
          168947d  .gitignore expansion
          fb3a93f  Stage H2.1 — Pi-side esp32_uart_bridge + /stats page
          584f8e2  firmware/esp32/environment.sh one-shot provisioner
          a26089c  firmware: bench-validated end-to-end on real hardware
          363e3ea  ESP32 firmware — 4-pin button wiring guidance + bench-test decoder
          6364494  Docs overhaul — fresh post-pivot documentation set
          2bcacca  Stage H2.0 — ESP32 I/O hub firmware skeleton
          f20d4f3  Stage H1.6 — live-pose viewport, faster rates, dynamic-obstacle SLAM tune
          6c315cb  Stage H1.5 — cleanup pass
          5088514  Stage H1 — pivot to handheld (roomba_* → recon_*)
main      c4f4c0a  Stage 5 — LD14P LIDAR bench test complete  (frozen pre-pivot)
```

`handheld` is ahead of `origin/handheld` by 3 commits at last sync (`5d5fea8`,
`20d0f3a`, `81a7ee8` — see Phase 4 push in this update cycle).

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
| `recon_hardware`  | ✅ Hybrid C++/Python. `sim_sensor_node` (C++) + `esp32_uart_bridge.py` (H2.1) + `imu_yaw_integrator.py` (H3) + `framing.py` parser. Publishes `/scan`, `/imu/data_raw`, `/imu/data`, `/buttons/*`, `/esp32/diagnostics`. Plus the H3 `robot_localization` ekf_node (run via `setup.sh`, configured by `config/ekf.yaml`) publishes `/odom` + `odom→base_link` TF. |
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
| LD14P LIDAR                  | ✅                | ✅ Now routed via ESP32 (Serial2 @ 230 400, Vcc switched by S8050 on GPIO4) — Pi UART0 freed |
| ESP32 DevKit V1              | ✅                | ✅ Flashed (Inc 1+2), USB-CDC link @ 460 800 baud, drives LIDAR enable line |
| MPU-6050 IMU                 | ✅                | ✅                |
| S8050 NPN + 1 kΩ on GPIO4    | ✅                | ✅ Low-side switch on LD14P GND/RX; motor stops cleanly on `LIDAR_EN=0` |
| 3 buttons + enclosure        | ⏳ TBD            | ⏳                |
| 22.5 W USB-C power bank → Pi 5 | ✅              | ✅ Powers Pi over USB-C; Pi feeds ESP32 (USB) → LD14P (5 V rail) — no buck/SPST |

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
| `/imu/data`             | ✅ H3 — `imu_yaw_integrator` republishes with orientation quaternion populated from integrated gyro Z (yaw only; roll/pitch=0). 100 Hz, RELIABLE QoS for slam_toolbox compatibility. |
| `/odom`                 | ✅ H3 — `robot_localization` ekf_node fuses `/imu/data` yaw + yaw-rate into a 2-D pose at ~30 Hz. Position stays at origin (no translation source in IMU); slam_toolbox's `map→odom` supplies the translation correction. |
| `/tf` (`odom → base_link`) | ✅ H3 — published by ekf_node. Replaces the static identity TF in `imu-test` and `sensor-test` modes; the static TF is only kept as the fallback for `sensor-test --no-esp32`. |
| `/esp32/diagnostics`    | ✅ H2.1 — JSON link health (frame counts, port, ESP32 uptime, boot STATUS) @ 1 Hz |
| `/buttons/save`, `/buttons/startstop`, `/buttons/shutdown_request`, `/buttons/shutdown_longpress` | ✅ H2.1 — std_msgs/Empty edges from the ESP32. **`recon_webui` now acts on them** (`/buttons/reset` was renamed to `/buttons/startstop`): SAVE = state toggle (scanning → save map + clear + stop LIDAR; stopped → start LIDAR, keep cached map); START/STOP = restart `recon-stack.service`; SHUTDOWN long-press = stop LIDAR + power off the Pi. Config + safety switch in `webui.yaml: webui.buttons`; sudoers rule in `environment.sh` §9.6. See `SPEC.md` §5.6. |
| `/robot/events`         | ✅ Live channel: SAVE_MAP and DELETED events                     |
| `/robot/mode`           | ✅ Live channel: IDLE / SCAN toggle (now also drives `/lidar_enable` lockstep) |
| `/lidar_enable` (svc)   | ✅ `std_srvs/SetBool` exposed by `esp32_uart_bridge`; flips PIN_LIDAR_EN on the ESP32. Refreshed @ 1 Hz while motor on so the firmware's 3 s watchdog drops the motor on Pi crash. |
| `/slam_toolbox/reset` (svc, consumed) | ✅ `slam_toolbox/srv/Reset` — called by `ros_bridge.clear_map()` (driven by the Clear-map button + `POST /api/map/clear`) to wipe the SLAM pose graph + occupancy grid in place, preserving the current Start/Pause state. |
| `/scanner/pose`         | ⏳ Not directly published. Web bridge composes `map→odom ∘ odom→base_link` instead. |
| `/draw/command`         | ⏳ H5 — web UI publisher                                         |

### 4.4 Web UI

| Page / route                  | State                                                       |
| ----------------------------- | ----------------------------------------------------------- |
| `/` (Dashboard)               | ✅ Live pose, mode badge (read-only — Start/Pause moved to `/map`), event log |
| `/map` (Live Map)             | ✅ Centred viewport, pose-anchored panning, save/list maps, **Start/Pause scan**, **Clear map** (resets live SLAM in place), **Clear trail**, **walking-trail polyline** (last 600 poses), per-map **Process** button (Tier-2 cleanup → coloured clusters) |
| `/stats` (Telemetry, H2.1)    | ✅ ESP32 link health + IMU live values w/ sparklines + SLAM stats |
| `/api/robot/status`           | ✅                                                            |
| `/api/robot/mode` (POST)      | ✅                                                            |
| `/api/scan/state` (GET)       | ✅ `{active}`                                                  |
| `/api/scan/start` (POST)      | ✅ Returns `{slam_responded, lidar_responded, mode}` — drives both `paused_new_measurements=false` and `/lidar_enable=true` in order (LIDAR on first, then SLAM unpause) |
| `/api/scan/pause` (POST)      | ✅ Same as above in reverse order (SLAM pause first, then LIDAR off) |
| `/api/map/clear` (POST)       | ✅ Returns `{success, slam_responded, message}` — calls `/slam_toolbox/reset` with `pause_new_measurements=false`. ~40 ms while paused, ~300 ms while active. UI wipes the canvas locally on success so the empty state is visible even while SLAM is paused (no fresh `/map` would arrive otherwise). |
| `/api/maps` (GET/POST)        | ✅                                                            |
| `/api/maps/<id>` (GET/PUT/DELETE) | ✅                                                       |
| `/api/maps/<id>/data` (GET)   | ✅                                                            |
| `/api/maps/<id>/process` (POST) | ✅ Run Tier-2 pipeline on saved map, persist as ProcessedMap row |
| `/api/maps/<id>/processed` (GET) | ✅ List processed runs for a map                          |
| `/api/processed/<id>/data` (GET) | ✅ Cleaned grid + cluster labels + Hough `line_segments` + `deskew_deg` (Manhattan alignment) |
| `/api/maps/events` (GET)      | ✅                                                            |
| `/api/stats` (H2.1)           | ✅ Combined IMU + bridge_health + SLAM + pose snapshot       |
| `/api/debug/channels`         | ✅                                                            |
| WebSocket `robot_pose`        | ✅ 10 Hz                                                      |
| WebSocket `map_update`        | ✅ 5 Hz                                                       |
| WebSocket `imu_data` (H2.1)   | ✅ 20 Hz (decimated from 100 Hz on the wire)                  |
| WebSocket `bridge_health` (H2.1) | ✅ 1 Hz JSON (now includes `lidar.desired_on / acked_on / pty_overflows`) |
| WebSocket `stats_update` (H2.1)  | ✅ 2 Hz consolidated snapshot                              |
| WebSocket `channel_status`    | ✅ 0.5 Hz                                                     |

### 4.5 ESP32 firmware

| Component                     | State                                                       |
| ----------------------------- | ----------------------------------------------------------- |
| PlatformIO project            | ✅ `firmware/esp32/platformio.ini` (`board=esp32dev`, `framework=arduino`) |
| `main.cpp` + scheduler        | ✅ Cooperative `millis()` deadlines                          |
| MPU-6050 driver (Wire.h only) | ✅ Register-level, no external libs, ±4 g / ±500 °/s @ 100 Hz |
| Button handler (debounce + long-press) | ✅                                                  |
| UART framing + CRC8           | ✅ `[0xA5 0x5A][TYPE][LEN][PAYLOAD][CRC8]`, Dallas/Maxim. **Bidirectional** since Inc 1; MAX_PAYLOAD = 64 |
| **LIDAR motor control (Inc 1)** | ✅ `PIN_LIDAR_EN = 4` drives S8050 base; first line of `setup()` forces LOW so the motor is off through the boot window. Opcodes `LIDAR_EN` (Pi→ESP) + `LIDAR_ACK` (ESP→Pi); 3 s watchdog forces motor off on missing Pi refresh. |
| **LIDAR data relay (Inc 2)**  | ✅ `Serial2` @ 230 400 on GPIO16 RX drains LD14P bytes, forwards as `LIDAR_FRAME` envelopes (max 64 B/chunk) only while motor is on. RX buffer bumped to 1 KB. |
| USB-CDC link to Pi            | ✅ Bumped 115 200 → **460 800 baud** in Inc 1 to carry LIDAR data + IMU + heartbeats with overhead |
| README + flash workflow       | ✅ [`firmware/esp32/README.md`](../firmware/esp32/README.md) |
| Bench-flashed                 | ✅ Round-trip-validated 2026-05-17 via `decode_serial.py`; full Inc 1+2 path verified 2026-05-23 (motor toggles cleanly, /scan published at 6 Hz through the pty) |
| Unit tests                    | ✅ H2.1 — 12 pytest cases in `tests/test_esp32_uart_bridge.py` covering CRC8, all 4 frame types, resync, bad CRC, oversized LEN, chunked input. Inc 1/2 opcodes (LIDAR_FRAME/EN/ACK) NOT yet covered by tests — to add. |

---

## 5. Test coverage

### 5.1 Python (pytest, 77 cases)

| File                          | Cases | Covers                                              |
| ----------------------------- | ----- | --------------------------------------------------- |
| `tests/test_db_node.py`       | 9     | Models, CRUD, session-map relationship, cascade     |
| `tests/test_recon_webui.py`   | 6     | DataChannel fallback timing, mock data shape        |
| `tests/test_esp32_uart_bridge.py` (H2.1) | 12 | CRC8 + 4 frame round-trips + resync + bad CRC + oversized LEN + chunked input |
| `tests/test_imu_yaw_integrator.py` (H3) | 12 | Yaw integration step (dt clamp, wrap), quaternion form, unit-norm |
| `tests/test_imu_calib.py`     | 5     | Gyro-bias autocal (still vs moving), static accel-bias subtraction |
| `tests/test_postprocess.py`   | 33    | Median, morphology, connected components, Hough walls, Manhattan deskew + snap + concentration guard |

Run: `bash -c 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && source .venv/bin/activate && pytest tests/'`.

### 5.2 C++ (gtest via colcon, 4+ cases)

| File                           | Cases | Covers                                  |
| ------------------------------ | ----- | --------------------------------------- |
| `tests/test_draw_node.cpp`     | 4     | Grid layout, paint, clear, brush clamp  |
| `tests/test_sim_sensor_node.cpp`| 4    | Raycast wall hit, max range, room connectivity (BFS), obstacle blocking |

Run: `cd roomba_ws && colcon test`.

---

## 6. Smoke-test results (last run 2026-05-25)

| Mode                  | Result                                                                            |
| --------------------- | --------------------------------------------------------------------------------- |
| `setup.sh kill`       | ✅ Tears down tmux + processes cleanly                                              |
| `setup.sh demo`       | ✅ All routes 200; `/stats` renders with mock IMU + mock bridge_health              |
| `setup.sh web`        | ✅ db_node + recon_webui_bridge spawn; manual `/tf` publish flips pose channel live |
| `setup.sh imu-test`   | ✅ ESP32 bridge + yaw integrator + EKF + Web UI. `/imu/data` @ 100 Hz, `/odom` @ 25 Hz, `odom→base_link` TF published by ekf_node, webui `pose.live=True` with theta tracking gyro drift at ~0.7 °/s |
| `setup.sh sensor-test --no-esp32` | ✅ Pre-LIDAR-through-ESP32 fallback — LIDAR driver opens `/dev/ttyAMA0` directly. Still works if you re-route the LIDAR wires back to the Pi |
| `setup.sh sensor-test`| ✅ LIDAR + ESP32 bridge through the pty path — drives Start/Pause via `/lidar_enable` |
| `setup.sh full`       | ✅ LIDAR + ESP32 + yaw integrator + EKF + SLAM + DB + Web UI. Bench-walked end-to-end: Start scan → motor spins + `/scan @ 6 Hz` + `/map @ 1 Hz` sustained; Pause → motor stops + `/scan` quiet + SLAM frozen; Clear map → reset returns in ~40 ms (paused) / ~300 ms (active), canvas wipes immediately, `/map` resumes ~20 s after re-Start |
| `recon-stack.service` (boot)   | ✅ systemd unit at `/etc/systemd/system/recon-stack.service` (template tracked in `roomba_ws/systemd/`). Runs `setup.sh full` as user `gabi` after `docker.service` + `recon-ap.service` + `network-online.target` + a 30-s `/dev/ttyUSB0` wait. `Restart=on-failure` with 15-s backoff, capped at 5 attempts in 5 min. SIGTERM on stop triggers setup.sh's existing trap → tmux teardown. Auto-installed + enabled by `environment.sh` Section 9.5. |

---

## 7. Known limitations & tech debt

| #  | Item                                                          | Severity | Notes                                                                                            |
| -- | ------------------------------------------------------------- | -------- | ------------------------------------------------------------------------------------------------ |
| 1  | EKF tracks only orientation, not translation                  | Low      | H3: `/odom` exposes only yaw + yaw-rate; the EKF ignores `linear_acceleration` because the chip's factory ZA_OFFSET biases it. Translation comes from slam_toolbox's `map→odom` correction at scan rate. Acceptable for handheld walking. |
| 2  | ESP32 accel Z reads ~15.3 m/s² with chip flat                 | Low      | Factory `ZA_OFFSET_USR = 1544` baked-in bias. Direction-of-gravity correct, magnitude is off. EKF removes the bias online. **Do NOT zero the offset registers in firmware** — bit 0 is a reserved temp-comp gate. |
| 3  | Boot STATUS diag may be missed by webui                        | Low      | If the bridge boots before the ESP32 sends STATUS, the frame is lost. Frame counts are still accurate; only the chip-identity dump is unavailable. |
| 4  | Initial pty overflows during startup race                      | Low      | The bridge writes LIDAR_FRAME bytes to the pty as soon as the ESP32 starts streaming. Before the ldlidar driver opens the slave (and starts draining), writes get EAGAIN and the chunk is dropped. Diagnostic counter `lidar.pty_overflows` grows in this window, then stabilises to 0 once the driver is up. Not data we care about (motor hasn't been commanded ON yet at boot). |
| 5  | LIDAR motor takes ~1 s to spin up after `LIDAR_EN=1`           | Low      | Mechanical spin-up + first valid LD14P packet. Start scan responds instantly; first /scan publish lags ~1 s. UI shows the transition immediately via the mode badge. |
| 6  | `/draw/command` has no publisher                              | Low      | `draw_node` runs idle; web UI driver lands in H5.                                                |
| 7  | LD14P driver (`ldlidar_stl_ros2`) is a locally-patched fork    | Low      | We patched `demo.cpp` to remove the 3-second initial-data timeout (so it can launch while the motor is off) and added a project-specific `ld14p.launch.py`. Tracked in a nested git repo with `master @ 35b3c8c`. Upstream behaviour is preserved in the embedded git history. |
| 8  | Postgres credentials still `roomba`/`roomba` in the container | Low      | `RECON_DB_URL` points at it. Renaming would lose existing scan data; deliberate carry-over. |
| 9  | No authentication on web UI                                   | Medium   | Designed for AP-only operation. Add basic auth before exposing on a LAN.                          |
| 10 | Scan-session state machine (IDLE → SCAN → SAVE) is just text  | Medium   | H5 introduces a real state machine that gates DB writes.                                          |
| 11 | `full_system.launch.py` is a placeholder                      | Low      | setup.sh is the canonical entry point.                                                            |
| 12 | LIDAR_FRAME/EN/ACK opcodes not in pytest coverage              | Low      | The 12 existing framing pytests cover IMU/BUTTON/HEARTBEAT/STATUS. New opcodes are exercised live; add round-trip pytests next pass. See [`REMAINING_ISSUES.md`](REMAINING_ISSUES.md). |
| 13 | `cap_net_bind_service` stripped by every `apt upgrade`        | Low      | Setup.sh preflight (commit 7978580) falls back to port 8080 with a WARN if the cap is missing. Re-run `environment.sh` to restore port 80. |
| 14 | LD14P driver patched to emit a fixed 720-beam scan             | Medium   | The stock driver computes `angle_increment = 2π/src.size()` per rotation, but the LD14P's actual point count varies ±5 between rotations (motor-speed jitter). Karto/slam_toolbox locks the count from the first scan and rejects every later one with a different count → **map silently stops updating after ~1 scan**. Workaround in nested commit `42688f6`: re-bucket variable points into a fixed 720-bin (0.5°) grid. Upstream LD14P drivers ship this bug; consider opening an issue. |
| 15 | Sync ROS2 service helpers in `ros_bridge` must run on an eventlet greenlet | Medium   | `_call_slam_pause` / `_call_lidar_enable` / `clear_map()` poll a Future with `time.sleep`, which is eventlet-greened. Calling them from the rclpy spin thread acquires a greened semaphore on a real OS thread and intermittently crashes the eventlet hub with `greenlet.error: Cannot switch to a different thread`. Auto-pause was rewired through `eventlet.spawn_after` (commit 5d5fea8) to avoid this. New code paths in `ros_bridge` need to honour the same constraint. |
| 16 | Walls smear / double across multiple loops of a room                    | Medium   | First full-stack chassis walk (map `b020+hol`, 617×432) showed the same wall laid down at 88° and 106° (should be 90° apart) → **yaw drift between passes**. Mitigations landed: (a) Pi-side gyro-bias autocal in `esp32_uart_bridge` removes the steady drift at the source; (b) `slam_params.yaml` bumped `scan_buffer_size` 10→20 and `loop_search_maximum_distance` 3.0→4.0. **Still pending a field walk:** enable `minimum_travel_distance/heading: 0.1` (see the comment in `slam_params.yaml`) to stop per-scan wall replay. |
| 17 | Manhattan deskew only fires on rectilinear maps                          | Low      | `recon_db.postprocess` Stage 6 rotates a processed map so its dominant walls are axis-aligned (`deskew_deg` in the payload), but is gated on angular concentration R ≥ 0.2 — a drift-smeared scan like `b020+hol` (R≈0.10) is intentionally left unrotated so the deskew never makes a messy map worse. Cleanly-tilted single rooms (R≈0.33) are straightened. |

---

## 8. Documentation

Active docs live in `docs/`:

| Doc                                    | Purpose                                       |
| -------------------------------------- | --------------------------------------------- |
| [`SPEC.md`](SPEC.md)                            | What the system is at every layer              |
| [`ARCHITECTURE.md`](ARCHITECTURE.md)            | How data moves through it                      |
| [`STATUS.md`](STATUS.md)                        | This file                                      |
| [`ROADMAP.md`](ROADMAP.md)                      | Forward plan, H3.1 → H6                        |
| [`REMAINING_ISSUES.md`](REMAINING_ISSUES.md)    | Open issues uncovered in the 2026-05-25 hardcore-test pass |
| [`AGENT_RULES.md`](AGENT_RULES.md)              | Binding rules for LLM/agent contributors       |
| [`UART_PROTOCOL.md`](UART_PROTOCOL.md)          | ESP32 ↔ Pi binary framing canonical reference  |

Pre-pivot history is preserved in
[`docs/archive/2026-05-10_pre-handheld/`](archive/2026-05-10_pre-handheld/).
