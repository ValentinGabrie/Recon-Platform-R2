# Recon-Platform-R2 — Roadmap

> Stage-gated plan from the handheld pivot (2026-05-09) through to a
> finished enclosure. Each stage is a small, mergeable chunk — never
> "do everything at once."
>
> Companion docs: [`SPEC.md`](SPEC.md) for what's being built,
> [`STATUS.md`](STATUS.md) for what's actually working today.

---

## Convention

Each stage has:
- A **Goal**: one sentence, the success criterion.
- A **Scope**: what's IN.
- An **Out of scope**: what's deferred to a later stage (so we don't sprawl).
- An **Acceptance** list: bullet items that must be true to declare the stage done.
- An indication of **Dependencies** on prior stages.

Stages are not parallelisable — H3 needs H2.1, H4 needs H3, etc. The
exception is documentation, which can chase any stage.

---

## ✅ H1 — Mechanical refactor (commit `5088514`, 2026-05-10)

**Goal:** Smallest compiling diff that turns the autonomous-robot
codebase into a handheld-scanner codebase.

**Done.** Five `roomba_*` packages renamed `recon_*`, `roomba_navigation`
deleted, autonomy / motors / Bluetooth / joy code stripped, web UI
reduced to two pages, configs and setup.sh trimmed. See
[archive snapshot](archive/2026-05-10_pre-handheld/) for the before
state.

---

## ✅ H1.5 — Cleanup pass (commit `6c315cb`, 2026-05-10)

**Goal:** Remove vestigial code that compiled in H1 but couldn't function.

**Done.** `sim_sensor_node` trimmed 952→397 lines (LIDAR-only).
`draw_node` rewritten to consume `/draw/command` text protocol instead
of `/joy`. Pose topic `/roomba/pose` → `/scanner/pose`. Stale
emit_rates / channel_timeouts removed from `webui.yaml`. Final
roomba/ROOMBA brand sweep across docs, comments, env vars.

---

## ✅ H1.6 — Live-pose viewport, faster rates, dynamic-obstacle SLAM tune (commit `f20d4f3`, 2026-05-10)

**Goal:** First handheld-friendly UX pass on the map page + SLAM tuning
for transient obstacles.

**Done.** `ros_bridge` derives scanner pose from `map → odom` TF (no
EKF needed yet). `map.html` re-anchors the canvas so the device is
always at viewport centre. Web emit rates bumped 2 → 5 Hz (map) and
5 → 10 Hz (pose). `slam_toolbox.map_update_interval` 5.0 → 1.0 s for
faster ray-trace clearing of transient obstacles.

---

## ✅ H2.0 — ESP32 firmware skeleton (commit `2bcacca`, 2026-05-10)

**Goal:** Code that, when flashed, will read the IMU + 3 buttons and
ship frames over USB-Serial.

**Done.** PlatformIO project at `firmware/esp32/`. Cooperative scheduler
(`millis()` deadlines), MPU-6050 register-level driver, button debounce
+ long-press, binary frame serialiser with Dallas/Maxim CRC8. **Not yet
flashed** — round-trip verification waits for H2.1.

---

## ✅ H2.1 — Pi-side ESP32 bridge + `/stats` page (2026-05-20)

**Done.** New Python ROS2 node `recon_hardware.esp32_uart_bridge` reads
the binary frame stream from the ESP32 over USB-Serial and republishes
it as: `sensor_msgs/Imu` on `/imu/data_raw` (BEST_EFFORT, ~100 Hz),
`std_msgs/Empty` edges on `/buttons/{save,reset,shutdown_request,shutdown_longpress}`,
and a JSON `std_msgs/String` link-health blob on `/esp32/diagnostics`
(1 Hz). The webui gained two new DataChannels (`imu`, `bridge_health`),
a `/stats` page with ESP32 link health + IMU sparklines + SLAM stats
cards, a `/api/stats` aggregator, and three new WebSocket events
(`imu_data`, `bridge_health`, `stats_update`).

Two real-world surprises captured for posterity:

1. `DataChannel` used `threading.Lock`, which after `eventlet.monkey_patch()`
   becomes a greenlet semaphore — and crashes with "Cannot switch to a
   different thread" when the rclpy real-OS spin thread tries to acquire
   it. Fix: `data_channels.py` now pulls the un-greened `threading`
   module via `eventlet.patcher.original("threading")` and uses that
   `Lock`. Falls back to stdlib threading cleanly when eventlet isn't
   imported (pytest).
2. `--params-file` rejected a YAML that had a top-level non-`/**:` key
   ("value before ros__parameters"). Split into `config/esp32_bridge.yaml`
   (pure ros-params, loaded via `--params-file`) and `config/hardware.yaml`
   (LIDAR-only static doc).

Validated live: bridge running, `imu.live=True (last_seen 0.03 s)`,
`bridge_health.live=True (0.37 s)`, 3500+ IMU frames + 33 heartbeats
captured over a 33-second window with zero CRC failures.

---

## ✅ H3 — IMU fusion (gyro-yaw → EKF → /odom + scan-match prior) (2026-05-20)

**Done.** Three layers landed in one commit:

1. `recon_hardware.imu_yaw_integrator` (Python) subscribes
   `/imu/data_raw`, integrates `angular_velocity.z * dt` into a running
   yaw, republishes on `/imu/data` (RELIABLE QoS, ~100 Hz) with the
   orientation quaternion populated (yaw only — accel-based roll/pitch
   would be biased by the chip's factory ZA_OFFSET).
2. `robot_localization` ekf_node (configured via `config/ekf.yaml`)
   fuses `/imu/data` yaw + yaw-rate into a 2-D pose, publishes `/odom`
   at ~30 Hz and the `odom → base_link` TF that replaces the static
   identity TF. The EKF ignores `linear_acceleration` entirely
   (bias-corrupted on this chip) — translation comes from
   slam_toolbox's `map→odom` correction at scan rate.
3. `slam_params.yaml` gained `imu_topic: /imu/data`, so slam_toolbox
   uses the gyro yaw as a scan-matching prior; scan matching
   reciprocates by erasing the gyro drift each successful match.

`ros_bridge.py` updated to compose `map→odom ∘ odom→base_link` for the
web UI pose channel (the old code assumed an identity `odom→base_link`).
A `/odom` fallback subscription gives the bridge a pose even when
slam_toolbox isn't running (e.g. imu-test mode).

Verified live in `setup.sh imu-test`: `/imu/data` @ 100 Hz, `/odom`
@ 25 Hz, web UI `pose.live=True`, theta drifting at ~0.7 °/s with the
chip's uncalibrated gyro bias (well within the "scan matcher will erase
it" envelope). 12 new pytest cases for the integration math; 39 total
tests passing.

## ✅ H4-prep — Start/Pause scan + walking trail + Tier-2 post-processing (2026-05-23)

> Pre-work that the original H4 (walking-test tuning) depends on. Lands
> independently as the UX layer the walking test will exercise.

**Done.** Three landings that converted the live SLAM stack into a
hand-driven scanning UX:

1. **`setup.sh full` mode** (`f744e1b`) — the canonical scanning mode.
   LIDAR + ESP32 + yaw integrator + EKF + slam_toolbox + DB + Web UI,
   no optional fallbacks; every prereq must be present. Added a
   walking-trail polyline on `/map` (last 600 poses drawn as a soft-blue
   line with a Clear-trail button) so a walk-around scan visibly shows
   where you've been.

2. **Start/Pause scan control** (`8a16c01`, fixed by `49189dd`) —
   `/map` page gained Start/Pause buttons. `ros_bridge.set_scanning()`
   drives the `slam_toolbox.paused_new_measurements` parameter via
   `set_parameters` (the misleadingly-named `Pause` service in
   slam_toolbox is actually a one-way set-to-paused and never resumes —
   our first attempt used it and ended up stuck in pause mode every
   time). The bridge auto-pauses 5 s after boot so a fresh launch waits
   for the user to press Start scan, and the dashboard's Scanner Mode
   card is now a read-only status badge that points at `/map` for the
   real control.

3. **Tier-2 post-processing pipeline** (`2a29da6`) — pure-numpy
   `recon_db.postprocess` module that cleans saved maps with a configurable
   pipeline: 3×3 median (off by default), morphological opening + closing,
   8-connected component labelling with a `min_cluster_size` noise filter.
   Persisted to a new `processed_maps` table; surfaced through
   `POST /api/maps/<id>/process` + `GET /api/maps/<id>/processed` +
   `GET /api/processed/<id>/data`. Each saved map row on `/map` got a
   **Process** button that renders the cleaned grid with per-cluster
   colours. 13 new pytest cases in `tests/test_postprocess.py`.

Reliability work in the same window: `setup.sh` preflight (`7978580`)
falls back to port 8080 when `cap_net_bind_service` has been stripped by
an `apt upgrade`; `environment.sh` (`3ced924`) dropped the unused
`imu-filter-madgwick` apt package and added `std_srvs` + `python3-serial`.

---

## ✅ LIDAR-through-ESP32 integration (Inc 1 + Inc 2, 2026-05-23)

**Done.** Moved the LD14P from a Pi-direct UART connection to a
fully-mediated path through the ESP32. Two increments:

### Inc 1 — motor power control (`ddc66ea`)

- All 4 LD14P wires re-terminated at the ESP32; Vcc shared, GND + RX
  switched through an S8050 NPN low-side switch on ESP32 GPIO 4.
- Firmware sets `PIN_LIDAR_EN` LOW as the very first instruction in
  `setup()` so the motor stays off through the boot window even without
  an external pull-down.
- Bidirectional UART protocol — new `FrameParser` in firmware mirrors the
  Pi-side parser. Opcodes `LIDAR_EN` (Pi → ESP, 1 B set) and
  `LIDAR_ACK` (ESP → Pi, 1 B current state).
- Pi bridge exposes `/lidar_enable` (`std_srvs/SetBool`) + refreshes
  `LIDAR_EN=1` every 1 s while the motor is on. Firmware watchdog (3 s)
  forces motor off if refreshes stop — Pi crash / disconnect can't leave
  the LIDAR spinning.
- Bumped the USB-CDC link from 115 200 to **460 800 baud** to fit the
  LD14P data stream that follows in Inc 2.

### Inc 2 — data path (`b8746e9`)

- Firmware listens on Serial2 (GPIO 16 RX, 230 400 baud) for raw LD14P
  bytes and forwards them as `LIDAR_FRAME` envelopes (max 64 B/chunk,
  MAX_PAYLOAD bumped from 24 to 64). RX buffer sized to 1 KB so a slow
  loop iteration can't drop a scan packet.
- Pi bridge creates a pty (`os.openpty()` + non-blocking master) and
  symlinks `/tmp/lidar_pty` to the slave. `LIDAR_FRAME` bytes are
  written to the master; pty overflows are counted but never block.
- `ld14p.launch.py` port changed from `/dev/ttyAMA0` to `/tmp/lidar_pty`.
- Locally patched `ldlidar_stl_ros2/src/demo.cpp` (commit `35b3c8c` in
  the embedded git): removed the 3-second `WaitLidarCommConnect`
  timeout-and-exit so the driver can launch while the motor is still
  off and wait indefinitely for the first packet.
- `ros_bridge.set_scanning()` now flips `/lidar_enable` in lockstep with
  `paused_new_measurements`. Ordering matters:
  - **Start**: enable LIDAR first → unpause SLAM.
  - **Pause**: pause SLAM first → disable LIDAR.

Verified live end-to-end: `POST /api/scan/start` returns
`slam_responded:true, lidar_responded:true`; `/scan` published at 6 Hz;
`/map` updated at 1 Hz; zero pty overflows in steady state. Pause path
stops the motor and freezes the map cleanly.

---

## ⏳ H3.1 — Madgwick + accel-fused roll/pitch, bench-rotation calibration

**Goal:** Real `/odom` and `/scanner/pose` from sensor data; SLAM no
longer relies on a static identity TF.

**Scope:**
- Add `imu_filter_madgwick` to `recon_bringup` launch path. Subscribes
  `/imu/data_raw` → publishes `/imu/data` (orientation-fused).
- Add `robot_localization` ekf_node. Consumes `/imu/data` only (no
  wheel odometry — handheld). Publishes `/odom` and `odom → base_link`
  TF.
- Drop the static identity `odom → base_link` from `setup.sh`.
- Small republisher (or directly in `ros_bridge`) that emits
  `/scanner/pose` (`PoseStamped`) from `/odom.pose` so the existing web
  UI subscriber path keeps working.
- Calibrate IMU bias on bench rotations: turn device 360° on a flat
  surface, expect `theta` integration error within ± 5° per minute.
  Document calibration procedure.
- Update `slam_params.yaml` to consume `/odom` instead of relying on
  scan-matching as the only pose source.

**Out of scope:**
- Walking-tested SLAM — H4.
- IMU magnetometer (we don't have one; MPU-6050 is 6-axis).
- Anything that pretends to do absolute heading; we have drift, deal with it.

**Acceptance:**
- `ros2 topic echo /odom` produces a stream that integrates IMU rotations
  visibly when the device is rotated by hand.
- `setup.sh sensor-test` starts the EKF + LIDAR + SLAM stack with no
  TF errors in the log (no "Failed to compute odom pose" warnings).
- The red arrow on the web UI tracks heading changes when the device is
  rotated, even at zero translation.
- Drift over a 1-minute stationary period < 5° / 5 cm.

**Dependencies:** H2.1.

---

## ⏳ H4 — IMU-aided SLAM, first walking test

**Goal:** Walk through a small room, end up with a recognisable map.

**Scope:**
- Tune `slam_toolbox` for handheld motion (faster `minimum_time_interval`,
  smaller `minimum_travel_distance`, adjusted `correlation_search_space_*`).
- Tune Madgwick `gain` parameter against the bench data from H3.
- Tune EKF process / measurement covariances to balance gyro drift vs
  scan-matching corrections.
- First field test: 5 m × 5 m room, walk perimeter twice, verify the
  map closes and walls are roughly straight.
- Document the calibration / tuning workflow in `docs/CALIBRATION.md`
  (new doc).

**Out of scope:**
- Multi-floor — won't work with 2-D SLAM.
- Long sessions / multi-room — first acceptance is one room.
- Scan-session state machine — that's H5.

**Acceptance:**
- Walking a 5 m × 5 m room at human pace produces a closed-loop map
  whose long walls are within ± 10 cm of true.
- No "scan match failed" warnings during normal walking.
- Map persists on save and reloads correctly through `/api/maps/<id>/data`.

**Dependencies:** H3.

---

## ⏳ H5 — Scan-session state machine + UI rework

**Goal:** Operating modes are real, with persistent state and visible
session identity.

**Scope:**
- New ROS2 node (or extend `db_node`): `session_manager`. Tracks the
  current scan session (`SessionRecord` rows in DB). State machine:
  `IDLE → SCAN → PAUSED → SCAN → SAVED → IDLE`.
- `/buttons/save` → finalise current session, write `SessionRecord` with
  the latest `MapRecord.id`. Auto-generate session name.
- `/buttons/reset` → discard current session, clear slam_toolbox map
  (this requires the slam_toolbox lifecycle pattern).
- Web UI rework:
  - Replace text mode toggle with a session widget showing
    elapsed time + cell count.
  - "Start new scan" / "Save & finish" / "Discard" buttons drive
    `/session/command` topic.
  - Saved Maps table now shows session metadata (start time, duration).
  - Publish to `/draw/command` from the dashboard's "annotation mode"
    (long-deferred draw_node finally has a driver).

**Out of scope:**
- Cloud sync.
- Multi-user sessions / auth.

**Acceptance:**
- Pressing SAVE on the device while in SCAN mode produces a finalised
  session row + map row, all visible in the web UI within 1 s.
- Pressing RESET while in SCAN mode clears the live map AND drops the
  in-progress session without saving.
- Dashboard shows "SCAN — 00:42 — 1247 cells" (or similar) live.

**Dependencies:** H4.

---

## ⏳ H6 — Enclosure + battery + final integration

**Goal:** A box you can hold, with a power switch and a battery.

**Scope:**
- 3D-printed enclosure for Pi 5 + ESP32 + LIDAR + 3 buttons.
- Battery selection (likely a USB-PD bank or 18650 + buck).
- Mechanical SPST switch on the battery rail for hard power.
- ESP32 SHUTDOWN button → soft Pi shutdown via the bridge running
  `sudo shutdown -h now`. Add corresponding `setup.sh` integration so
  the device powers down cleanly even without the SPST.
- Update `base_link → laser_frame` static TF to reflect the actual
  mechanical offset between the device's mounting reference and the
  LIDAR's optical centre.
- Status LED on the enclosure surface (mirrors ESP32 LED) so the user
  has external feedback.
- Field-readiness checklist in `docs/FIELD_GUIDE.md` (new doc):
  charging, power-on sequence, AP credentials, scan workflow.

**Out of scope:**
- Anything not strictly needed to scan a real room outside a workshop.

**Acceptance:**
- Device runs untethered for ≥ 1 hour from full charge.
- Single-button-press scanning workflow: SHUTDOWN-hold = power-off,
  SAVE = save+finalise, RESET = discard.
- A non-developer can scan a room with only the field guide.

**Dependencies:** H5.

---

## Beyond H6 — speculative

These are explicitly **not** on the active roadmap. Capturing them so
the path back is short if they become priority.

| Idea                                | Why it's not H1–H6                                      |
| ----------------------------------- | ------------------------------------------------------- |
| 3-D mapping (e.g. add a tilt servo) | Handheld 2-D scanning is the validated product idea     |
| Cloud sync of saved maps            | Adds a multi-machine concern; local-first is the bar    |
| Multi-floor / elevator support      | Hard. Probably needs a different SLAM front-end         |
| iOS / Android companion app         | Web UI is good enough; a native app is a separate project |
| Active LIDAR scanner (rotating mirror) | LD14P already gives 360°; mechanical complexity not justified |

---

## Branch and commit hygiene

- All H-stage work happens on the `handheld` branch.
- One commit per stage, descriptive title prefixed `Stage Hx.y:`.
- `main` stays at `c4f4c0a` (Stage 5) until H6 is field-validated; at
  that point we'll consider an `handheld → main` merge or a fresh
  `recon` branch as the new mainline.
- Every committed stage must leave the workspace in a buildable,
  smoke-tested state — the H1 / H1.5 / H1.6 / H2.0 commits all do.

For day-to-day rules see [`AGENT_RULES.md`](AGENT_RULES.md).
