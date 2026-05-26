# Recon-Platform-R2 — Remaining Issues

> Curated list of open problems as of **2026-05-25**, produced from a
> full file-by-file repo audit + stationary hardcore test pass
> (build / unit / REST / WS / ROS-inventory). See [`STATUS.md`](STATUS.md)
> for what's working and [`ROADMAP.md`](ROADMAP.md) for the
> still-pending feature stages.
>
> **Format.** Each entry has a severity (Critical / Medium / Low /
> Cosmetic), a **what** (the symptom), a **why** (root cause if known),
> and a **fix** (concrete next step). Closed issues are removed; this
> file is a *current state* document, not a changelog.

---

## Audit summary

| Suite                           | Result                                       |
| ------------------------------- | -------------------------------------------- |
| `colcon build --symlink-install` (6 pkgs) | ✅ 5.0 s                              |
| `pytest tests/`                  | ✅ 52 / 52 in 1.3 s                          |
| `colcon test` (gtest)            | ✅ 4 DrawNode + 4 SimSensor                  |
| `firmware/esp32 pio run`         | ✅ 3.3 s, 22 % flash                         |
| `bash -n` on `setup.sh` / `environment.sh` / `rebuild.sh` / firmware `environment.sh` | ✅ all clean |
| `./environment.sh --check`       | ⚠️ 97 PASS / 4 FAIL — all FAILs are `sudo grep` calls that fail because sudo isn't cached (see issue #1) |
| REST endpoint smoke (25 routes)  | ✅ 25 / 25                                  |
| WebSocket event smoke (8 events) | ✅ 8 / 8 (`robot_pose`, `map_update`, `robot_mode`, `imu_data`, `bridge_health`, `stats_update`, `channel_status`, `scan_state`) |
| ROS topic inventory vs docs      | ✅ all documented topics/services exist at runtime; rates as expected (`/scan` 5.7 Hz, `/map` 1.0 Hz, `/tf` 74 Hz, `/odom` 24 Hz, `/imu/data` 100 Hz, `/imu/data_raw` 115 Hz, `/esp32/diagnostics` 1 Hz) |

Nothing functionally broken. The issues below are quality-of-life,
deprecation-related, or coverage gaps surfaced by the audit.

---

## Critical

*(none — every documented contract works end-to-end)*

---

## Medium

### M1 — LIDAR_FRAME / LIDAR_EN / LIDAR_ACK opcodes lack pytest coverage

**What:** `tests/test_esp32_uart_bridge.py` covers IMU / BUTTON /
HEARTBEAT / STATUS round-trips (12 cases). The three opcodes added in
the LIDAR-through-ESP32 increments (`0x05` / `0x06` / `0x07`) are
exercised live but have no unit tests.

**Why:** Inc 1 + Inc 2 were validated end-to-end on the physical device
(motor toggles, /scan flowing) without adding regression coverage.

**Fix:** Add pytest cases in `test_esp32_uart_bridge.py`:
- LIDAR_FRAME byte payload round-trip (varying lengths 1..MAX_PAYLOAD)
- LIDAR_EN encode (Pi → ESP) via `encode_lidar_en(True)` / `encode_lidar_en(False)` — verify exact wire bytes incl. CRC
- LIDAR_ACK decode → `(FrameType.LIDAR_ACK, bool)` tuple
- BAD_LEN handling for LIDAR_FRAME with length > MAX_PAYLOAD
- The /lidar_enable service handler in bridge: motor desired-state flip + diagnostics counters

Effort: ~2 hours.

### M2 — Sync ROS2 service helpers must run on eventlet greenlet (load-bearing, fragile)

**What:** `ros_bridge._call_slam_pause` / `_call_lidar_enable` /
`clear_map()` poll a `Future` with `time.sleep`. Eventlet has
monkey-patched the threading primitives those helpers transitively
acquire (rclpy client locks), so calling them from a real OS thread —
e.g. the rclpy spin thread — crashes the eventlet hub with
`greenlet.error: Cannot switch to a different thread`. Auto-pause was
rewired through `eventlet.spawn_after` to avoid this (commit `5d5fea8`),
but the rule is implicit: any new helper added next to these or any new
rclpy timer/subscriber callback that wants to use them will re-introduce
the same crash.

**Why:** Mixing eventlet + rclpy in one Python process. Both manage
their own concurrency model.

**Fix (in priority order):**
1. **Now**: tighten [`AGENT_RULES.md`](AGENT_RULES.md) §6 rule 5 (done) and
   [`ARCHITECTURE.md`](ARCHITECTURE.md) §8 invariant (done).
2. **Soon**: add an `assert_greenlet()` helper at the top of each sync
   service helper that raises if it sees `threading.current_thread() is
   not the eventlet hub`. Fail loudly rather than crash the hub.
3. **Later**: write a pytest that simulates calling a helper from an
   rclpy timer callback and asserts it either succeeds-or-rejects
   cleanly.

### M3 — Slam_toolbox occasionally drops messages: "queue is full"

**What:** ~once every 20–30 s during active scanning, slam_toolbox
logs:
```
[slam_toolbox]: Message Filter dropping message: frame 'laser_frame'
at time XXX for reason 'discarding message because the queue is full'
```

**Why:** slam_toolbox uses a `tf2_ros::MessageFilter` to gate
`/scan` until the corresponding `map → odom` (and through it
`odom → base_link`) TFs are available. The default queue size is small
(10) and the EKF/static-TF chain occasionally has a moment of latency.

**Impact:** A dropped scan per ~5 minutes. Map quality probably
slightly worse during transient stalls but no visible breakage.

**Fix:** Bump the slam_toolbox transform queue depth in
[`config/slam_params.yaml`](../roomba_ws/config/slam_params.yaml) (param
name varies between Jazzy point releases — try
`transform_publish_period` first). Alternatively, increase EKF
publication rate to reduce TF latency.

### M4 — LD14P driver is a locally-patched fork — upstream divergence

**What:** `roomba_ws/src/ldlidar_stl_ros2` is an embedded git repo, not
a submodule (no `.gitmodules`). Local commits:
- `35b3c8c` — `ld14p.launch.py` port → `/tmp/lidar_pty`, indefinite-wait demo.cpp patch
- `42688f6` — fixed 720-beam geometry

Without `.gitmodules`, anyone who clones the parent and resets the
nested directory loses both patches and the map will silently stop
updating after one rotation.

**Fix (one of):**
1. Convert to a proper git submodule (`git submodule add` with the
   nested SHA pinned) so checkout actually carries the patches.
2. Vendor the directory in-tree (delete the nested `.git`, track all
   files in the parent). Simplest but loses upstream history.
3. Open a PR against upstream `ldlidar_stl_ros2` for the fixed-beam
   geometry (it's a real bug their other consumers will hit too) and
   keep our local thin patch for the pty path.

### M5 — `environment.sh --check` returns false FAILs for hostapd content when sudo isn't cached  ✅ CLOSED 2026-05-26

**Fixed.** Added a `check_sudo` helper in `environment.sh` that:
1. Tries `sudo -n true` first; if no cached sudo, emits **WARN**
   (not FAIL) with the hint "run `sudo -v` first to verify".
2. Otherwise runs the underlying `sudo -n grep` / `sudo -n stat`
   normally.

The 4 hostapd checks were rewired to use `check_sudo`. From a fresh
shell `environment.sh --check` now shows clean PASS / WARN with no
spurious failures.

### M6 — Eventlet is deprecated upstream (bugfix-only)

**What:** `pytest` emits this on every run:
```
EventletDeprecationWarning: Eventlet is deprecated. It is currently
being maintained in bugfix mode, and we strongly recommend against
using it for new projects.
```
The recon_webui stack relies heavily on eventlet for greenlet-based
Flask-SocketIO + tpool for blocking DB calls. Migration is a real
project.

**Fix (planned, not urgent):** evaluate `asyncio`-based Flask-SocketIO
(via `asgiref` + `quart`) or the newer
`python-socketio[asyncio_client]`. The threading invariants in §6 will
all need to be revisited. **Don't** start until H6 is closer, because
this is a stack-wide rewrite.

---

## Low

### L1 — `tests/test_db_node.py` uses deprecated `datetime.utcnow()`

**What:** 13 deprecation warnings during `pytest tests/test_db_node.py`:
```
DeprecationWarning: datetime.datetime.utcnow() is deprecated and
scheduled for removal in a future version.
```

**Fix:** s/`datetime.utcnow()`/`datetime.now(datetime.UTC)`/ in
`tests/test_db_node.py` (and check `recon_db/models.py` for the same
pattern). Trivial.

### L2 — `websocket-client` not in venv requirements

**What:** `python-socketio` is installed but its `websocket-client`
extra isn't. In-venv socket.io clients (like the WebSocket smoke
script) fall back to long-polling. Production webui serves websockets
fine — Flask-SocketIO has its own server-side WS implementation that
doesn't need this package.

**Fix:** Add `websocket-client` to
[`requirements.txt`](../roomba_ws/requirements.txt) only if we ship an
in-process WS client (we don't today). Otherwise close as wontfix.

### L3 — `/diagnostics` topic clutter

**What:** `ros2 topic list` shows a `/diagnostics`
(`diagnostic_msgs/msg/DiagnosticArray`) topic with 1 publisher and 0
subscribers. Published by `robot_localization`'s ekf_node by default;
not consumed by anything in our stack.

**Fix:** Set `print_diagnostics: false` in
[`config/ekf.yaml`](../roomba_ws/config/ekf.yaml) to suppress it.
Cosmetic.

### L4 — Boot STATUS frame may be missed by webui

**What:** `esp32_uart_bridge` publishes its boot `STATUS` frame ~0 s
after the serial port opens. If the webui's `RosBridge` subscribes a
few hundred ms later, the chip-identity dump is lost until next ESP32
reset. Frame counts are still accurate.

**Fix:** Have the bridge cache the last STATUS frame and re-publish on
any new `/esp32/diagnostics` subscriber. Or just have the bridge node
re-emit the diagnostics payload with `status` populated every minute.

### L5 — `pty_overflows` counter is cumulative and includes startup race

**What:** In the `setup.sh full` boot sequence, the esp32_uart_bridge
opens the pty master and starts receiving `LIDAR_FRAME` packets as
soon as the motor is enabled — but the LD14P driver may not have
opened the slave yet (typical race is ~1–2 s). During that window,
non-blocking writes to the pty master return `EAGAIN` and the bridge
counts them as `pty_overflows`. The counter never decreases, so the
diagnostics card shows a "drift" that's actually all startup-only.

**Fix:** Reset `pty_overflows` to 0 on the first successful pty write
after at least one `LIDAR_ACK` has been received. Or expose
`pty_overflows_steady_state` as a separate counter that starts when
the slave is first observed open.

### L6 — `/imu/data_raw` reports ~115 Hz on `ros2 topic hz`

**What:** Spec says 100 Hz, firmware emits at 100 Hz, but the measured
rate over a short window is ~115 Hz. Likely an averaging artefact of
`ros2 topic hz`'s warm-up window (counts boot-time burst when the
bridge first drains its serial buffer). `/imu/data` (yaw integrator
output) measures at 100 Hz exactly.

**Fix:** Measure over a longer window (`ros2 topic hz /imu/data_raw
--window 200`) and confirm steady-state is 100 Hz. If still off, check
the bridge's clock skew. Probably not a real issue.

### L7 — `/draw/command` has no publisher

**What:** `draw_node` (recon_control) subscribes to
`/draw/command` for a text-based drawing protocol used for end-to-end
DB pipeline testing. No producer exists today — the node sits idle.

**Fix:** Either build the web UI driver (planned in H5), document the
node as "test fixture, manual `ros2 topic pub` only", or remove the
node if it's not earning its keep.

---

## Cosmetic

### C1 — `static_transform_publisher` node name has a random suffix

**What:** Each `setup.sh` run gives the static TF publisher a fresh
nonce in its node name (e.g.
`/static_transform_publisher_gC0UGQQboOEu85wO`). `setup.sh kill`
relies on a pattern match in `RECON_PROC_PATTERNS` which works today
but is brittle if upstream changes the naming scheme.

**Fix:** Pass `__node:=static_tf_base_link_to_imu_link` explicitly in
setup.sh.

### C2 — `full_system.launch.py` is a placeholder

**What:** [`recon_bringup`](../roomba_ws/src/recon_bringup/) ships a
launch file that only covers `db_node + recon_webui`. The canonical
entry point is `setup.sh`.

**Fix:** Either flesh out the launch file to mirror `setup.sh full`,
or delete the placeholder + drop `recon_bringup` to a pure README
package.

### C3 — Postgres credentials still `roomba`/`roomba`

**What:** Container name and credentials carry over from the pre-pivot
codebase. `RECON_DB_URL` points at it. Renaming would orphan existing
scan data on dev devices.

**Fix:** Migrate at H6 when the user is OK losing the bench history,
or write a `pg_dump` → recreate → `pg_restore` one-shot script.

### C4 — `setup.sh imu-test --no-esp32` mode

**What:** There's a `sensor-test --no-esp32` flag for the pre-Inc-2
LIDAR-direct path. Now that LIDAR routes through the ESP32, this flag
documents a fallback that requires manual rewiring to use.

**Fix:** Add a clear note in `setup.sh --help` and STATUS smoke-test
row that this flag assumes the LD14P pigtail goes to `/dev/ttyAMA0`,
not through the ESP32.

---

## Known limitations (carried from STATUS.md tech-debt, repeated here for completeness)

- **EKF tracks only orientation, not translation.** `/odom` exposes
  yaw + yaw-rate only. Position stays at origin until slam_toolbox
  supplies translation via `map→odom`. Acceptable for handheld walking.
- **ESP32 accel Z reads ~15.3 m/s² with chip flat** — factory
  ZA_OFFSET bias. Direction-of-gravity correct, magnitude is off. EKF
  removes the bias online. Do NOT zero the offset registers in firmware.
- **LIDAR motor takes ~1 s to spin up after `LIDAR_EN=1`.**
  Mechanical spin-up + first valid LD14P packet.
- **No authentication on web UI.** Designed for AP-only operation.
  Add basic auth before exposing on a LAN.
- **Scan-session state machine (IDLE → SCAN → SAVE) is text-only** —
  H5 introduces a real state machine.
- **`cap_net_bind_service` stripped by every `apt upgrade`.**
  Setup.sh preflight falls back to port 8080 with a WARN; re-run
  `environment.sh` to restore port 80.

---

## Process

When closing an issue here:
1. Open a commit that fixes it.
2. Remove the entry from this file in the same commit (or the next
   doc-refresh commit).
3. If the fix surfaces a new lesson, add it to
   [`AGENT_RULES.md`](AGENT_RULES.md) or
   [`ARCHITECTURE.md`](ARCHITECTURE.md) §8 as an invariant.

When adding a new issue:
1. Pick the lowest severity that still motivates fixing it.
2. State a concrete **Fix** — "investigate" is not a fix, "set
   `print_diagnostics: false` in ekf.yaml" is.
3. If it's a real surprise that future-you would want to know about
   before reading code, add a one-liner to [`STATUS.md`](STATUS.md)
   §7 Known limitations too.
