# Rules — Binding Constraints for LLM/Agent Development

This file defines mandatory constraints for any LLM or agent working on the roomba project.
Every rule is non-negotiable unless the maintainer explicitly overrides it in the prompt.

---

## 1. Documentation

- **Update docs on every change.** Any code change that alters behaviour, adds a feature, or fixes a bug must be reflected in the relevant `.md` file(s) before the task is considered complete. This includes `project_status.md` (progress tracking), `project_requirements.md` (if the spec evolves), and inline docstrings.
- **Keep `project_status.md` current.** After completing work, update the status file: mark completed items, add new known issues, and adjust the roadmap if scope changed.
- **Document public symbols.** C++: Doxygen `/** @brief */` on every public function/class. Python: Google-style docstrings on every public function/class. No exceptions.
- **TODOs are commitments.** Every `// TODO:` (C++) or `# TODO:` (Python) must describe *what* needs to be resolved and *why* it's deferred. Never leave a bare TODO.

## 2. Logging

- **Always add logging.** Every new function that performs I/O, subprocess calls, network requests, or state transitions must log at the appropriate level. No silent operations.
- **Use the project's logging stack.** Python nodes: use `get_logger(__name__)` from `logging_config.py`. C++ nodes: use `RCLCPP_DEBUG/INFO/WARN/ERROR` macros. Never use `print()`, `std::cout`, or `printf` for operational output.
- **Log levels are semantic:**
  - `DEBUG` — telemetry, internal state, values useful only during development.
  - `INFO` — state transitions, startup/shutdown, configuration loaded, connections established.
  - `WARN` — recoverable issues (timeout, retry, fallback activated).
  - `ERROR` — failures requiring attention (I/O error, subprocess crash, missing resource).
- **Log on entry and exit of critical paths.** Bluetooth operations, ROS bridge lifecycle, database writes, and subprocess calls must log both initiation and outcome.
- **Include context in log messages.** Always include relevant identifiers: MAC addresses, topic names, file paths, error codes. Example: `logger.info(f"BT connect success: {mac}")`, not `logger.info("connected")`.

## 3. Language & Architecture

- **Follow the language assignment table.** C++17 for real-time nodes (hardware, control, navigation). Python 3.11+ for non-RT (web UI, database). No deviations.
- **One node, one file.** Never combine two ROS2 nodes in a single source file.
- **Config over constants.** All addresses, pins, thresholds, topic names, and tuning parameters must come from YAML config or ROS2 parameters. Zero hardcoded values in source.
- **No mode flags for data source.** The web UI determines data source solely via `DataChannel.is_live()` timeout. No environment variables, no boolean flags, no if/else for mock vs real.

## 4. Error Handling

- **Fail loudly on startup.** Validate all required parameters on node init. C++: throw `std::runtime_error`. Python: raise `RuntimeError`. Include what's missing and what was expected.
- **Never swallow exceptions silently.** Every `except` or `catch` block must at minimum log the error. Bare `except: pass` is forbidden unless the comment explains exactly why.
- **Return structured errors from APIs.** All REST/WebSocket endpoints return `{"success": bool, "message": str}` or equivalent. Never return bare booleans without diagnostic context.
- **Validate at system boundaries.** User input from web UI, REST API payloads, MAC addresses, file paths — validate and sanitise before use. Internal function-to-function calls within the same module can trust their inputs.

## 5. Security

- **Validate all external input.** MAC addresses must match `^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$`. API payloads must be type-checked. Never pass unsanitised strings to subprocess, SQL, or shell commands.
- **Escape output for HTML.** Any value originating from external sources (Bluetooth device names, user input, ROS messages) must be escaped before injection into HTML/innerHTML. Use the `esc()` function in templates.
- **No secrets in code.** Passwords, API keys, and WPA passphrases go in config files with restricted permissions (600), not in source code or version control.

## 6. Threading & Concurrency

- **eventlet monkey-patch is first.** `import eventlet; eventlet.monkey_patch()` must be the first two lines in `app.py`. No imports before it.
- **Never call `socketio.emit()` from the rclpy thread.** Use `queue.Queue` to bridge rclpy OS threads to eventlet green threads. This is a binding architectural constraint.
- **Use `tpool.execute()` for blocking calls in eventlet context.** All subprocess calls (bluetoothctl, etc.) from Flask routes must go through `tpool.execute()`.
- **Protect shared state.** Python: `threading.Lock`. C++: `std::mutex`. No unprotected shared mutable state between callbacks.

## 7. Testing

- **Every node has a test skeleton.** All tests live in `/tests/`. C++: Google Test via `ament_cmake_gtest`. Python: pytest.
- **Test the contract, not the implementation.** Test that nodes publish correct messages on correct topics with correct types. Don't test private internals.
- **Update tests when behaviour changes.** If a function's return type changes (e.g., `bool` → `dict`), update every test that touches it in the same commit.

## 8. Web UI

- **No CDN dependencies.** All JS/CSS libraries must be bundled locally under `static/`. The robot operates on an isolated network.
- **Bluetooth page shows real state.** The paired devices list and controller status card must reflect live `bluetoothctl` output, not cached or assumed state. Poll periodically.
- **Toast notifications for all user actions.** Every button click that triggers a backend operation must show immediate feedback (loading state) and a result toast (success/failure with message).

## 9. Shell Scripts

- **`setup.sh` and `environment.sh` are maintained deliverables.** When adding a new node, dependency, or startup mode, update the relevant script.
- **Prerequisite checks before launch.** Every mode in `setup.sh` must validate its dependencies and fail with a clear message. Don't let nodes crash with cryptic errors.
- **Idempotent installs.** `environment.sh` must be safe to run multiple times. Guard every write with existence checks.

## 10. Process

- **Scope to the request.** Do exactly what's asked. Don't refactor surrounding code, add unrequested features, or "improve" things that work.
- **Read before writing.** Always read existing code before modifying it. Understand the current state, then change it.
- **Verify after editing.** After modifying Python files, check syntax. After modifying C++ files, confirm the build compiles. After modifying templates, verify Jinja2 parsing.
- **Commit-ready changes.** Every change should leave the project in a buildable, runnable state. No half-done modifications.
