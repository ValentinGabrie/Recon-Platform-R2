# Recon-Platform-R2 — Agent Rules

> Binding constraints for any LLM/agent contributing to this repo.
> Every rule is non-negotiable unless the maintainer explicitly overrides
> it in the prompt. Replaces the pre-pivot
> [`rules.md`](archive/2026-05-10_pre-handheld/rules.md).

What changed from the old `rules.md` (autonomous robot → handheld scanner):

- Bluetooth, Xbox controller, motor / pigpio rules dropped — those
  subsystems were deleted in H1.
- Web UI scope narrowed: only Dashboard + Map pages exist.
- Added: **`setup.sh` is the only entry point** (never `ros2 launch` directly).
- Added: pose topic is `/scanner/pose`, **not** `/roomba/pose` (gone).
- ROS2 packages are `recon_*`, **not** `roomba_*`.
- The eventlet, ros_bridge thread-safety, config-over-constants, and
  fail-loudly rules carry forward verbatim.

---

## 1. Documentation

- **Update docs on every behaviour change.** When a code change
  modifies behaviour, adds a feature, or fixes a bug, the relevant
  Markdown in `docs/` is updated **in the same commit**. Targets:
  - `docs/STATUS.md` — what changed, what's now working.
  - `docs/SPEC.md` — only if the contract (topics, frame format, API)
    changes.
  - `docs/ROADMAP.md` — when a stage moves from "in progress" to "done".
- **Inline doc comments are not optional.** C++: `/** @brief */` on
  every public function/class. Python: Google-style docstrings on every
  public function/class.
- **TODOs are commitments.** A `// TODO:` or `# TODO:` must say
  *what's missing* and *why it was deferred*. Bare `TODO` is forbidden.
- **Never delete content from the archive.** `docs/archive/` is a
  frozen snapshot. If history is misleading, add a note alongside it,
  don't rewrite the past.

## 2. Logging

- **Every I/O, subprocess call, and state transition logs.** Silent
  operations are not allowed.
- **Use the project logging stack.**
  - Python: `get_logger(__name__)` from `recon_webui.logging_config`.
  - C++ (rclcpp nodes): `RCLCPP_DEBUG/INFO/WARN/ERROR` macros.
  - ESP32: `Serial.println()` is acceptable for diagnostics — but only
    *outside* the `setup()`-after-Serial.begin window, and never in the
    100 Hz hot path. Frames are the canonical channel.
  - **Never** `print()` / `std::cout` / `printf` for operational output.
- **Log levels are semantic.**
  - `DEBUG` — telemetry useful only during development.
  - `INFO` — state transitions, startup, configuration loaded.
  - `WARN` — recoverable issues (timeout, retry, fallback activated).
  - `ERROR` — failures requiring attention.
- **Include context.** Topic names, file paths, error codes, MAC
  addresses. `logger.info("connected")` is wrong;
  `logger.info(f"db_node connected — url={db_url}")` is right.
- **`db_node` (ROS2 Python) uses f-strings**, never %-style format
  args. `RcutilsLogger` does not support positional formatting and
  crashes at runtime. Documented in
  [archive/.../project_status.md §4 #22](archive/2026-05-10_pre-handheld/project_status.md).

## 3. Architecture

- **Language assignment.** C++17 for real-time / hardware paths
  (`recon_hardware`, `recon_control`). Python 3.12 for non-RT
  (`recon_db`, `recon_webui`). No deviations.
- **One node, one source file.** Never combine two ROS2 nodes in one
  source file.
- **Config over constants.** All addresses, pins, baud rates, topic
  names, thresholds, timeouts come from YAML config or ROS2 parameters.
  Zero hardcoded values in source. Pinout for the ESP32 lives in
  `firmware/esp32/src/config.h`; pinout for the LIDAR lives in
  `roomba_ws/config/hardware.yaml`.
- **No mode flags for data source.** The web UI determines real vs
  mock data solely via `DataChannel.is_live()` timeouts. No env vars,
  no boolean flags, no branching logic.

## 4. Error handling

- **Fail loudly on startup.** Validate all required parameters in node
  init. C++: throw `std::runtime_error`. Python: raise `RuntimeError`.
  The exception message says what's missing and what was expected.
- **Never swallow exceptions.** Every `except` / `catch` must at
  minimum log. Bare `except: pass` is forbidden unless a comment
  explains exactly why (and that "why" is reviewed in the PR).
- **Structured errors from APIs.** REST endpoints return
  `{"success": bool, "message": str, ...}`. Never bare booleans.
- **Validate at system boundaries.** Web UI input, REST payloads,
  filenames, IDs from URL params: validate before use. Internal
  function-to-function calls within the same module can trust their
  inputs.

## 5. Security

- **Validate external input.** Numeric IDs in URL paths use Flask's
  `<int:>` converter. Map names are length-limited and HTML-escaped.
  Never pass unsanitised strings to subprocess, SQL, or shell.
- **HTML-escape everything from external sources.** Use `esc()` in
  templates for any value originating outside the server.
- **No secrets in code.** DB credentials, WiFi passwords go in
  config files (mode 600) outside version control. The Postgres
  password lives in `roomba_ws/docker/.env` (gitignored).

## 6. Threading & concurrency (web UI)

These rules are **load-bearing**. Breaking them caused documented
deadlocks pre-pivot.

- **`eventlet.monkey_patch()` is the first executable line of `app.py`.**
  Must come before any other import that touches stdlib threading
  primitives.
- **Never call `socketio.emit()` from the rclpy thread.** Use the
  `queue.Queue(maxsize=64)` bridge in `RosBridge`. The eventlet
  `emit_loop` drains it.
- **`rclpy.spin()` runs on a real OS thread**, not an eventlet green
  thread. Get the un-greened threading module via
  `eventlet.patcher.original("threading")`.
- **Use `tpool.execute()` for blocking calls inside Flask routes.** DB
  queries, subprocess calls, file I/O. Otherwise they freeze the
  green-thread loop.
- **Sync ROS2 service helpers (`_call_slam_pause`, `_call_lidar_enable`,
  `clear_map`, future similar) run ONLY from an eventlet greenlet** —
  never from the rclpy spin thread. They poll a `Future` with
  `time.sleep`, which after monkey-patch is eventlet-greened; acquiring
  the underlying semaphore from a real OS thread crashes the hub with
  `greenlet.error: Cannot switch to a different thread`. Schedule from
  `app.py` via `eventlet.spawn_after(...)` if you need a timer; Flask
  routes already inherit the greenlet context.
- **Protect shared state.** Python: `threading.Lock` (un-greened via
  `eventlet.patcher.original` if accessed from both the rclpy thread
  and the eventlet hub). C++: `std::mutex`.

## 7. Testing

- **Every node has at least one test.** Minimum: a "loads without
  crashing" smoke test. Real assertions are mandatory; a test that
  always passes is not a test.
- **Test the contract, not the implementation.** Validate that nodes
  publish expected messages on expected topics with expected types.
  Don't introspect private members.
- **Update tests when behaviour changes.** If a function's return type
  changes, every test that touches it is updated in the same commit.
- **Run before committing.**
  - Python: `cd roomba_ws && .venv/bin/python -m pytest tests/`
  - C++: `cd roomba_ws && colcon test`
  - Both must be green before a stage is declared complete.

## 8. Web UI

- **No CDN dependencies.** All JS/CSS bundled locally under
  `static/`. The device operates on an isolated AP.
- **Toast every user action.** Every button click that triggers a
  backend operation shows immediate loading feedback and a result
  toast (success/error with message).
- **Mock fallback is automatic.** If a `DataChannel` goes stale,
  `is_live()` returns false and `get()` returns the mock value. The
  client renders a "MOCK" watermark when applicable. No client-side
  branching.

## 9. Shell scripts

- **`setup.sh` is the canonical entry point.** Smoke tests, demos, dev
  startup → always go through `setup.sh <mode>`. Never `ros2 launch` /
  `ros2 run` / `python3 -m recon_*` directly. Bare launches don't
  activate the venv → `ModuleNotFoundError: sqlalchemy/eventlet`.
  Confirmed empirically; don't relearn.
- **`setup.sh` and `environment.sh` are maintained deliverables.** New
  node, dependency, or startup mode → update the relevant script in
  the same commit.
- **Pre-flight checks before launch.** Every mode in `setup.sh`
  validates its dependencies and fails with a clear message rather
  than letting nodes crash with cryptic errors.
- **Idempotent installs.** `environment.sh` is safe to run multiple
  times. Guard every write with an existence check. `--check` is the
  verify-only mode for already-set-up devices.

## 10. ESP32 firmware

- **No external libraries beyond the framework.** `Arduino.h` and
  `Wire.h` only. If you reach for `Adafruit_*` or similar, justify it
  in the PR description; the project preference is owning the I²C
  reads.
- **Conserve UART budget.** 100 Hz IMU streaming is ~3 KB/s. The link
  now runs at **460 800 baud** (~46 KB/s) since Inc 1 to carry LIDAR
  data, but the budget is still finite; don't fill it with diagnostics.
  STATUS frames at boot + on error only.
- **`PIN_LIDAR_EN` is driven LOW as the very first statement in
  `setup()`** — before any `delay` or `Serial.begin`. The motor must
  stay off through the ~200 ms boot window even without an external
  pull-down. The 3 s watchdog enforces motor-off on Pi-side silence.
- **Signed `(int32_t)` math for every `millis()`-based comparison.**
  Unsigned subtraction underflows when the captured `now` is slightly
  older than a value set by a concurrent code path (e.g. a refresh
  arriving mid-iteration); the watchdog initially shipped with this bug
  and toggled the motor every millisecond.
- **Cooperative scheduling.** No FreeRTOS tasks for the current
  workload. `loop()` runs as fast as possible, each task gated by a
  `millis()` deadline. If you need true preemption, justify it.
- **Frame format is canonical** — see [`UART_PROTOCOL.md`](UART_PROTOCOL.md).
  Changing the wire format breaks the Pi-side bridge; coordinate the
  two sides in one commit.

## 11. Process

- **Scope to the request.** Do exactly what's asked. Don't refactor
  surrounding code, don't add unrequested features, don't "improve"
  things that work. The H-stage roadmap exists to keep changes small.
- **Read before writing.** Read the file (or the relevant portion)
  before modifying it. The harness enforces this for `Edit`/`Write`,
  but the principle applies to architecture changes too: read
  `STATUS.md` and `ARCHITECTURE.md` before proposing a change.
- **Verify after editing.** After Python edits, `pytest`. After C++
  edits, `colcon build`. After template edits, smoke-test
  `setup.sh demo`. After firmware edits, `pio run`.
- **Commit-ready.** Every change leaves the project buildable and
  smoke-tested. No half-done modifications. If you can't finish in
  the current session, leave the working tree clean and document the
  partial state.
- **One stage = one commit.** Stage commits use the format
  `Stage Hx.y: <subject>`. Multi-line bodies explain the *why*.

---

## Quick reference: forbidden patterns

If you find yourself typing any of these, stop and reconsider:

```python
# Forbidden
print("debug:", x)                       # use logger.debug
import time; time.sleep(0.1)             # in eventlet context — use eventlet.sleep
socketio.emit(...)                       # from rclpy thread — use the queue
except Exception: pass                   # silent swallow
"/roomba/pose"                            # post-pivot the topic is "/scanner/pose"
"ROOMBA_DB_URL"                           # post-pivot it's "RECON_DB_URL"
ros2 launch recon_bringup full_system.launch.py   # use setup.sh
```

```cpp
// Forbidden
std::cout << "...";                       // use RCLCPP_*
auto v = config_value;  // hardcoded     // pull from a parameter
```

```bash
# Forbidden
ros2 run recon_webui recon_webui          # bypasses venv → ModuleNotFoundError
```

---

## Slim mandatory-reading list for a new task

When picking up work, read these in order:

1. [`docs/STATUS.md`](STATUS.md) — what's the current state?
2. [`docs/ROADMAP.md`](ROADMAP.md) — which stage am I in?
3. This file (rules) — what are the binding constraints?
4. [`docs/SPEC.md`](SPEC.md) §<relevant section> — what's the contract for the layer I'm touching?
5. [`docs/ARCHITECTURE.md`](ARCHITECTURE.md) — only if changing data flow.

`docs/UART_PROTOCOL.md` is mandatory only for ESP32 firmware or Pi-side
bridge work.
