# Thesis Deep-Analysis Prompt
# Paste this entire file as your opening message in a new session.
# The agent will read every source file and write the output document.

---

You are writing technical documentation for a university thesis. The subject is the
**Recon-Platform-R2** project: a handheld 2-D LIDAR scanning device built on a
Raspberry Pi 5 + ESP32 coprocessor, running ROS2 Jazzy, with a Flask/SocketIO web UI
and a PostgreSQL database. The repo lives at `/home/gabi/Recon-Platform-R2`.

Your job is to produce ONE comprehensive output document at:
  `/home/gabi/Recon-Platform-R2/docs/THESIS_TECHNICAL_DOCUMENTATION.md`

The document must be written in thesis-quality English (formal, third person, precise).
It must cover every design decision, every line of non-trivial code, every protocol,
every algorithm, and every architectural trade-off found in the repository.
Cross-reference the `.md` docs (STATUS, SPEC, ARCHITECTURE, AGENT_RULES, ROADMAP,
UART_PROTOCOL) against the actual source code — if the docs claim X but the code does
Y, note the discrepancy explicitly. The docs are a starting reference, NOT the truth;
the code is the truth.

USE ALL YOUR AVAILABLE TOKENS. Be exhaustive. Do not summarise where you can elaborate.
Prefer depth over brevity on every topic.

---

## MANDATORY READING ORDER — read every file listed before writing a single word

Work through the files in this exact sequence. Use `read_file` for every file.
Do not skip any file. Do not rely on memory or inference — read the raw bytes.

### 1. Top-level documentation
```
/home/gabi/Recon-Platform-R2/docs/AGENT_RULES.md
/home/gabi/Recon-Platform-R2/docs/SPEC.md
/home/gabi/Recon-Platform-R2/docs/ARCHITECTURE.md
/home/gabi/Recon-Platform-R2/docs/ROADMAP.md
/home/gabi/Recon-Platform-R2/docs/STATUS.md
/home/gabi/Recon-Platform-R2/docs/UART_PROTOCOL.md
/home/gabi/Recon-Platform-R2/docs/REMAINING_ISSUES.md
/home/gabi/Recon-Platform-R2/README.md
/home/gabi/Recon-Platform-R2/.github/copilot-instructions.md
```

### 2. ESP32 firmware (read every file)
```
/home/gabi/Recon-Platform-R2/firmware/esp32/README.md
/home/gabi/Recon-Platform-R2/firmware/esp32/platformio.ini
/home/gabi/Recon-Platform-R2/firmware/esp32/src/config.h
/home/gabi/Recon-Platform-R2/firmware/esp32/src/main.cpp
/home/gabi/Recon-Platform-R2/firmware/esp32/src/framing.h
/home/gabi/Recon-Platform-R2/firmware/esp32/src/framing.cpp
/home/gabi/Recon-Platform-R2/firmware/esp32/src/imu.h
/home/gabi/Recon-Platform-R2/firmware/esp32/src/imu.cpp
/home/gabi/Recon-Platform-R2/firmware/esp32/src/buttons.h
/home/gabi/Recon-Platform-R2/firmware/esp32/src/buttons.cpp
/home/gabi/Recon-Platform-R2/firmware/esp32/environment.sh
```

### 3. ROS2 workspace — configuration
```
/home/gabi/Recon-Platform-R2/roomba_ws/config/hardware.yaml
/home/gabi/Recon-Platform-R2/roomba_ws/config/ekf.yaml
/home/gabi/Recon-Platform-R2/roomba_ws/config/esp32_bridge.yaml
/home/gabi/Recon-Platform-R2/roomba_ws/config/slam_params.yaml
/home/gabi/Recon-Platform-R2/roomba_ws/config/webui.yaml
/home/gabi/Recon-Platform-R2/roomba_ws/config/simulation.yaml
```

### 4. recon_hardware package (C++ + Python)
```
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_hardware/package.xml
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_hardware/CMakeLists.txt
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_hardware/src/sim_sensor_node.cpp
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_hardware/recon_hardware/__init__.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_hardware/recon_hardware/esp32_uart_bridge.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_hardware/recon_hardware/framing.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_hardware/recon_hardware/imu_yaw_integrator.py
```

### 5. recon_control package
```
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_control/package.xml
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_control/CMakeLists.txt
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_control/src/
```
(list the src directory and read every file found)

### 6. recon_db package
```
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_db/package.xml
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_db/setup.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_db/recon_db/__init__.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_db/recon_db/db_node.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_db/recon_db/models.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_db/recon_db/postprocess.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_db/recon_db/migrations/env.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_db/recon_db/migrations/script.py.mako
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_db/recon_db/migrations/versions/
```
(list the versions directory and read every migration file)

### 7. recon_webui package
```
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/package.xml
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/setup.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/recon_webui/__init__.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/recon_webui/app.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/recon_webui/ros_bridge.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/recon_webui/data_channels.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/recon_webui/logging_config.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/recon_webui/mock_data.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/recon_webui/templates/base.html
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/recon_webui/templates/map.html
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/recon_webui/templates/maps.html
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/recon_webui/templates/diagnostics.html
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_webui/recon_webui/static/js/common.js
```

### 8. recon_bringup package
```
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_bringup/package.xml
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_bringup/setup.py
/home/gabi/Recon-Platform-R2/roomba_ws/src/recon_bringup/launch/
```
(list the launch directory and read every .launch.py file)

### 9. Vendored LIDAR driver (key files only — it is third-party)
```
/home/gabi/Recon-Platform-R2/roomba_ws/src/ldlidar_stl_ros2/README.md
/home/gabi/Recon-Platform-R2/roomba_ws/src/ldlidar_stl_ros2/package.xml
/home/gabi/Recon-Platform-R2/roomba_ws/src/ldlidar_stl_ros2/launch/
/home/gabi/Recon-Platform-R2/roomba_ws/src/ldlidar_stl_ros2/src/
/home/gabi/Recon-Platform-R2/roomba_ws/src/ldlidar_stl_ros2/include/
```
(list each directory and read every file)

### 10. Tests
```
/home/gabi/Recon-Platform-R2/roomba_ws/tests/test_esp32_uart_bridge.py
/home/gabi/Recon-Platform-R2/roomba_ws/tests/test_imu_yaw_integrator.py
/home/gabi/Recon-Platform-R2/roomba_ws/tests/test_db_node.py
/home/gabi/Recon-Platform-R2/roomba_ws/tests/test_postprocess.py
/home/gabi/Recon-Platform-R2/roomba_ws/tests/test_recon_webui.py
/home/gabi/Recon-Platform-R2/roomba_ws/tests/test_draw_node.cpp
/home/gabi/Recon-Platform-R2/roomba_ws/tests/test_sim_sensor_node.cpp
```

### 11. Infrastructure / scripts
```
/home/gabi/Recon-Platform-R2/roomba_ws/setup.sh
/home/gabi/Recon-Platform-R2/roomba_ws/environment.sh
/home/gabi/Recon-Platform-R2/roomba_ws/rebuild.sh
/home/gabi/Recon-Platform-R2/roomba_ws/requirements.txt
/home/gabi/Recon-Platform-R2/roomba_ws/docker/docker-compose.yaml
/home/gabi/Recon-Platform-R2/roomba_ws/systemd/
```
(list the systemd directory and read every file)

### 12. Archive (for historical context — note what changed and why)
```
/home/gabi/Recon-Platform-R2/docs/archive/2026-05-10_pre-handheld/
```
(list the directory and read every file found)

---

## OUTPUT DOCUMENT STRUCTURE

Write the output to `/home/gabi/Recon-Platform-R2/docs/THESIS_TECHNICAL_DOCUMENTATION.md`.
Use `create_file` to write it. If the file already exists, overwrite it.
Structure the document EXACTLY as follows. Each section must be as long as it needs to be
to be complete — there is no length limit.

```
# Recon-Platform-R2 — Complete Technical Documentation
## For thesis use — generated by exhaustive file-by-file analysis

---

## Chapter 1: Project Overview and Motivation
  1.1  Problem statement — what does indoor LIDAR mapping solve?
  1.2  System goals and acceptance criteria (from ROADMAP.md, verified against code)
  1.3  Project evolution: from autonomous robot to handheld scanner (archive comparison)
  1.4  Scope limitations and deliberate design exclusions

## Chapter 2: Hardware Architecture
  2.1  System-level block diagram (described in text + ASCII art)
  2.2  Raspberry Pi 5 — role, OS, kernel configuration
  2.3  ESP32 DevKit V1 — role as I/O coprocessor
  2.4  LD14P (LD-D200) LIDAR sensor — operating principle, electrical interface,
       non-standard wire colour issue (documented discrepancy with datasheet)
  2.5  MPU-6050 IMU — register-level configuration, sampling rate, I²C addressing
  2.6  S8050 NPN transistor motor-gate circuit — schematic analysis, why low-side switching
  2.7  Button debounce hardware vs software responsibility split
  2.8  Power budget: battery rail, SPST switch, 5V buck converter, USB current limits
  2.9  Wiring table cross-reference: SPEC.md vs config.h vs actual firmware pin usage

## Chapter 3: ESP32 Firmware
  3.1  Build system: PlatformIO, arduino-esp32 framework, flash/RAM usage
  3.2  config.h — every constant explained (pin assignments, baud rates, frame types,
       timing constants). Note any magic numbers.
  3.3  Cooperative scheduler pattern in main.cpp — loop() structure, task interleaving,
       timing guarantees and their limits
  3.4  framing.h / framing.cpp — binary frame format byte-by-byte:
         SOF byte, frame type, length, payload, CRC8 (Dallas/Maxim polynomial).
         Encode and decode paths. Error handling.
  3.5  imu.h / imu.cpp — register-level MPU-6050 driver:
         initialisation sequence, register addresses, raw→float conversion,
         100 Hz sampling implementation, Wire.h I²C timing
  3.6  buttons.h / buttons.cpp — debounce algorithm, long-press state machine,
         event generation, GPIO pull-up configuration
  3.7  LIDAR relay path: Serial2 RX → USB-CDC forwarding, LIDAR_EN watchdog logic,
         3-second timeout, why the motor must default OFF
  3.8  Bidirectional protocol: all 7 opcode types, direction, payload schemas
  3.9  Known firmware limitations and TODOs (from REMAINING_ISSUES.md + code comments)

## Chapter 4: Pi-Side Hardware Abstraction (recon_hardware)
  4.1  Package structure: mixed C++17 + Python 3.12 in one ROS2 package — rationale
  4.2  framing.py — Python mirror of the C++ framing layer:
         frame parser state machine, CRC8 verification, edge cases
  4.3  esp32_uart_bridge.py — complete analysis:
         Serial port discovery and opening, read loop threading model,
         frame demuxing to ROS2 topics, PTY master for LIDAR relay,
         /lidar_enable service server, diagnostics publisher, watchdog logic
  4.4  imu_yaw_integrator.py — Madgwick-free yaw integration:
         why gyro integration instead of full AHRS, drift characteristics,
         quaternion output for EKF, 100 Hz subscription
  4.5  sim_sensor_node.cpp — simulated LIDAR for bench testing:
         synthetic scan geometry, publish rate, how it substitutes for real hardware
  4.6  robot_localization EKF node — configuration in ekf.yaml,
         sensor fusion: IMU + wheel odometry (none) → odom frame,
         TF tree: map → odom → base_link → laser

## Chapter 5: SLAM and Mapping Pipeline
  5.1  slam_toolbox (online_async mode) — algorithm overview, configuration in
       slam_params.yaml, every parameter explained and justified
  5.2  /scan topic: LaserScan message format, 720-beam fixed geometry patch,
       why the beam-count lock required the vendored driver patch
  5.3  LIDAR → ESP32 → pty → ldlidar_stl_ros2 data path:
       latency budget, buffer sizes, why a PTY was chosen over a named pipe or socket
  5.4  The LD14P vendored driver patches (commits 35b3c8c + 42688f6):
       ld14p.launch.py change (opens /tmp/lidar_pty),
       demo.cpp change (wait-for-first-packet + fixed-beam geometry) — code diff analysis
  5.5  Map coordinate frame and occupancy grid encoding (OccupancyGrid message)
  5.6  Dynamic obstacle handling: map_update_interval tuning, raytracing behaviour
  5.7  Start/Pause scan: slam_toolbox set_parameters RPC, why the Pause toggle API
       was rejected, lockstep with LIDAR motor enable

## Chapter 6: State Estimation
  6.1  IMU data flow: raw accelerometer + gyroscope → /imu/data_raw → yaw integrator
       → /imu/data (quaternion) → EKF
  6.2  robot_localization ekf_node: configuration, input topics, output /odom +
       odom→base_link TF, covariance tuning
  6.3  TF tree in full: every frame, every publisher, update rates
  6.4  Why no wheel odometry: design choice consequences for drift accumulation
  6.5  Accuracy limits of gyro-integration-only heading in a handheld device

## Chapter 7: Database Layer (recon_db)
  7.1  PostgreSQL in Docker: docker-compose.yaml analysis, container name, port,
       volume persistence, RECON_DB_URL env var convention
  7.2  SQLAlchemy models.py — every table, every column, types, constraints, indices
  7.3  Alembic migrations: migration chain, versioning strategy, env.py configuration
  7.4  db_node.py — ROS2 node analysis:
         event subscription (/robot/events), SAVE_MAP trigger,
         OccupancyGrid → numpy → BLOB serialisation,
         SQLAlchemy session lifecycle, f-string logger constraint
  7.5  postprocess.py — Tier-2 map processing pipeline:
         median filter (why median over Gaussian), morphological operations
         (opening/closing, structuring element sizes), connected-component
         clustering, per-cluster label storage, NumPy-only constraint rationale
  7.6  REST API surface (/api/maps/*, /api/map/clear) — routes, request/response
       schemas, error handling

## Chapter 8: Web UI (recon_webui)
  8.1  Threading model: eventlet.monkey_patch() placement, why it must be first,
       consequences of getting it wrong
  8.2  app.py — Flask application factory, route registration, SocketIO setup,
       eventlet WSGI server, port 80 vs 8080 fallback (cap_net_bind_service)
  8.3  ros_bridge.py — ROS2 ↔ SocketIO bridge:
         rclpy spin thread, queue.Queue bridge pattern (why not direct emit),
         tpool.execute() for blocking Flask routes, subscription list,
         _call_slam_pause / _call_lidar_enable lockstep
  8.4  data_channels.py — SocketIO event namespace and channel definitions
  8.5  Templates analysis:
         base.html — instrument-panel theme, CSS architecture, JS includes
         map.html — live OccupancyGrid canvas rendering, walking-trail polyline,
                    Start/Pause/Clear/Save button wiring to SocketIO events
         maps.html — saved map gallery, Hough Line Transform overlay,
                     modal viewer, POST /api/map/clear handler
         diagnostics.html — /esp32/diagnostics live feed
  8.6  common.js — shared SocketIO client logic, reconnect handling,
       canvas rendering algorithm, coordinate transform (ROS grid → canvas pixels)
  8.7  WiFi AP setup: hostapd + dnsmasq configuration (from environment.sh),
       ap0 virtual interface, SSID "Recon", IP 10.0.0.1, DNS recon.local

## Chapter 9: System Integration and Deployment
  9.1  setup.sh — complete analysis of all four modes (kill / demo / web / sensor-test):
         tmux session structure, per-pane venv activation, node launch order,
         stty pre-flight on /dev/ttyUSB0, why direct ros2 run fails (ModuleNotFoundError)
  9.2  environment.sh — idempotent provisioning script:
         every apt package installed, every pip package, venv creation,
         kernel serial console configuration, cap_net_bind_service,
         --check verification mode
  9.3  systemd service(s) — unit file analysis, dependency ordering, restart policy,
       how auto-start at boot works (REMAINING_ISSUES M5 closure)
  9.4  rebuild.sh — colcon build invocation, workspace overlay structure
  9.5  Docker integration: how the ROS2 nodes reach the PostgreSQL container,
       network mode, container lifecycle relative to the recon-stack service
  9.6  requirements.txt — every Python dependency, why each is needed

## Chapter 10: Testing Strategy
  10.1  Test philosophy: what is tested, what is explicitly not tested, rationale
  10.2  test_esp32_uart_bridge.py — test case analysis:
          frame encode/decode round-trips, demux routing, error injection
  10.3  test_imu_yaw_integrator.py — gyro integration accuracy tests
  10.4  test_db_node.py — database node unit tests, mock ROS2 infrastructure
  10.5  test_postprocess.py — map post-processing tests, synthetic OccupancyGrid fixtures
  10.6  test_recon_webui.py — Flask test client, SocketIO mock, route coverage
  10.7  test_draw_node.cpp + test_sim_sensor_node.cpp — gtest cases analysis
  10.8  Coverage gaps: LIDAR_FRAME/LIDAR_EN/LIDAR_ACK missing tests
        (REMAINING_ISSUES M1), ros_bridge async methods (M2)
  10.9  CI/CD: colcon test invocation, pytest integration, result reporting

## Chapter 11: Security Analysis
  11.1  Network attack surface: Flask on :80, no authentication, AP-only network
  11.2  Input validation: HTML escaping in templates, SQL injection via SQLAlchemy ORM
  11.3  Serial port security: /dev/ttyUSB0 permissions, udev rules
  11.4  Docker container isolation: PostgreSQL credentials, env var handling
  11.5  Known security limitations and acceptable-risk decisions

## Chapter 12: Design Trade-offs and Lessons Learned
  12.1  Why ROS2 Jazzy over ROS1 or bare Python
  12.2  ESP32 as coprocessor vs direct Pi UART — latency, reliability, extensibility
  12.3  PTY relay vs direct serial vs Unix socket for LIDAR data path
  12.4  eventlet vs asyncio vs threading in the web UI
  12.5  NumPy-only post-processing vs OpenCV — dependency weight, ARM64 compatibility
  12.6  Cooperative scheduler in ESP32 firmware vs FreeRTOS tasks
  12.7  The pre-handheld pivot: what was discarded, what survived, architectural debt

## Appendix A: Complete Topic and Service Inventory
  (table of every ROS2 topic, type, publisher, subscriber, QoS, rate)

## Appendix B: Complete File Tree with One-Line Purpose Annotations

## Appendix C: Binary Frame Format Reference
  (complete wire-format tables for all 7 frame types, CRC polynomial, example bytes)

## Appendix D: YAML Configuration Reference
  (every key in every config file, its type, default, and effect on behaviour)

## Appendix E: Bibliography and Further Reading
  (see bibliography instructions below)
```

---

## BIBLIOGRAPHY INSTRUCTIONS

After writing the main document, fetch the following URLs and extract titles, authors,
publication years, publishers, and DOIs/URLs for the bibliography. Include ALL that are
accessible. Add any additional relevant sources you find.

### ROS2 / Middleware
- https://docs.ros.org/en/jazzy/index.html
- https://design.ros2.org/articles/why_ros2.html
- https://arxiv.org/abs/2211.07752   (ROS2 real-time paper)
- https://ieeexplore.ieee.org/document/9196654  (ROS2 evaluation)

### SLAM / Mapping
- https://arxiv.org/abs/1605.07797   (slam_toolbox / Karto SLAM paper)
- https://arxiv.org/abs/1610.06462   (Google Cartographer)
- https://ieeexplore.ieee.org/document/9561657  (LiDAR SLAM survey)
- https://www.roboticsproceedings.org/rss05/p67.pdf  (Hector SLAM)

### Sensor Fusion / EKF
- https://arxiv.org/abs/1107.0442   (Madgwick filter paper)
- https://ieeexplore.ieee.org/document/1271397  (robot_localization / EKF)
- https://www.cs.unc.edu/~welch/kalman/  (Kalman filter tutorial)

### IMU / MPU-6050
- https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf

### LIDAR hardware
- https://www.ldrobot.com/ProductDetails?productId=10  (LD14P product page)

### ESP32 / embedded
- https://docs.espressif.com/projects/esp-idf/en/latest/esp32/
- https://docs.platformio.org/en/latest/

### Flask / SocketIO / eventlet
- https://flask.palletsprojects.com/en/latest/
- https://flask-socketio.readthedocs.io/en/latest/
- https://eventlet.readthedocs.io/en/latest/

### PostgreSQL / SQLAlchemy / Alembic
- https://www.postgresql.org/docs/current/
- https://docs.sqlalchemy.org/en/20/
- https://alembic.sqlalchemy.org/en/latest/

### Raspberry Pi 5
- https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-product-brief.pdf

### Software engineering / embedded systems books to cite
Fetch the following and note full bibliographic details:
- "Programming Robots with ROS" — Quigley, Gerkey, Smart (O'Reilly)
- "ROS Robot Programming" — ROBOTIS (free PDF)
- "Probabilistic Robotics" — Thrun, Burgard, Fox (MIT Press) — chapters on EKF-SLAM and occupancy grids
- "Embedded Systems Design with the Atmel AVR Microcontroller" — not directly relevant but cite for embedded context
- "Real-Time Concepts for Embedded Systems" — Qing Li, Caroline Yao

---

## FINAL INSTRUCTIONS

1. Read ALL files listed. Use `read_file` with large line ranges — read full files,
   not excerpts. If a file is longer than your read range, read it in multiple chunks.

2. List every directory you are told to list before deciding which files to read.

3. Cross-check every claim in the .md files against the actual code. Note any
   discrepancy as: **[DISCREPANCY: docs say X, code does Y]**

4. When you encounter a non-obvious algorithm or design pattern, explain it fully —
   assume the reader is a final-year CS/EE student who knows Python and C++ but
   has not seen ROS2 before.

5. Write the output file using `create_file`. If the file is too large for one call,
   use multiple `replace_string_in_file` calls to append sections.

6. After the file is written, print a one-paragraph summary of what was written
   and flag any files you were unable to read.

7. Do not stop until every chapter, every appendix, and the bibliography are written.
   Do not summarise. Do not skip. Use every available token.
