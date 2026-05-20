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

## 🟡 H3 partial — gyro-yaw → slam_toolbox imu_topic (2026-05-20)

**Done (minimum-viable slice):** New Python ROS2 node
`recon_hardware.imu_yaw_integrator` subscribes `/imu/data_raw`,
integrates `angular_velocity.z * dt` into a running yaw, and
republishes on `/imu/data` (RELIABLE QoS, ~100 Hz) with the orientation
quaternion populated (yaw only; roll/pitch=0 because the chip's
factory ZA_OFFSET makes accel-derived tilt unreliable). `slam_params.yaml`
gained `imu_topic: /imu/data`, so slam_toolbox now uses the gyro yaw as
a prior for scan matching — and scan matching reciprocates by
continuously correcting the gyro drift.

Verified live in `setup.sh imu-test`: `/imu/data` flowing at 100 Hz with
orientation populated (q.z ≈ 0.037 after ~8 s of stationary integration,
i.e. ~0.5 °/s of gyro bias drift — within the "acceptable, scan matcher
will erase it" envelope). 12 new pytest cases for the integration math.

**Not yet done — defer to H3.1:**

- Full `robot_localization` EKF + `/odom` publication + dynamic
  `odom → base_link` TF. Static identity TF is still what `setup.sh`
  launches; translation comes entirely from scan matching.
- Madgwick / accel-fused orientation (roll, pitch).
- Bench-rotation calibration test that quantifies yaw drift over a
  controlled 360° turn.

## ⏳ H3.1 — full EKF, `/odom`, bench-rotation calibration

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
