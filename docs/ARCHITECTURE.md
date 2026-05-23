# Recon-Platform-R2 — Architecture

> Companion to [`SPEC.md`](SPEC.md). Where SPEC says **what** the system
> is, this doc shows **how data moves through it**.

---

## 1. Hardware block diagram

All sensor data and motor power for the LIDAR now flow through the ESP32
— the Pi has a single serial peripheral (the ESP32 USB-CDC link) instead
of the previous Pi-direct UART for the LIDAR.

```
   ┌──────────┐                 ┌──────────────────────────────────────────┐
   │  LD14P   │  Vcc + GND ◄── S8050 (low-side, gated by ESP32 GPIO 4)     │
   │  LIDAR   │  ──────────────────────────────────────────────────────────┤
   │ (4 wire) │  TX (white) ──► ESP32 Serial2 RX (GPIO 16) @ 230400 8N1    │
   │          │  RX (green) ──► tied to LIDAR GND (default speed)          │
   └──────────┘                                                            │
                                                                           │
                ┌──────────────────────────────────────────────────────────┤
                │                       ESP32 DevKit V1                     │
                │  • MPU-6050 @ 100 Hz over I²C 0x68                         │
                │  • 3 buttons (SHUTDOWN/RESET/SAVE) on GPIO 25/26/27         │
                │  • LD14P UART relay (Serial2 → USB-CDC LIDAR_FRAME chunks)  │
                │  • LIDAR_EN (GPIO 4) — 3 s watchdog on Pi refresh          │
                │  • USB-CDC link to Pi @ 460 800 8N1  ───────────────────┐ │
                └────────────────────────────────────────────────────────┐│ │
                                                                         ││ │
   ┌─────────────────────────────────────────────────────────────────────┴┴─┴─┐
   │                            Raspberry Pi 5                                 │
   │                       Ubuntu 24.04 · ROS2 Jazzy                            │
   │                                                                            │
   │  /dev/ttyUSB0  ─►  esp32_uart_bridge (Python, rclpy)                       │
   │                          │ demux frames                                    │
   │                          ├─► /imu/data_raw         (100 Hz, BEST_EFFORT)   │
   │                          ├─► /buttons/{save,reset,shutdown_*}              │
   │                          ├─► /esp32/diagnostics    (1 Hz JSON)              │
   │                          ├─► /lidar_enable  service (std_srvs/SetBool)     │
   │                          └─► LIDAR_FRAME bytes ► pty master ───┐           │
   │                                                                 │           │
   │  /tmp/lidar_pty (symlink ► /dev/pts/N) ◄────────────────────────┘           │
   │       │                                                                     │
   │       ▼                                                                     │
   │  ldlidar_stl_ros2  ──►  /scan                                                │
   │       │                  │                                                    │
   │       │                  ▼                                                    │
   │       │              slam_toolbox (online_async)  ───►  /map, /tf (map→odom)  │
   │       │                                                                       │
   │       │                /imu/data (yaw quaternion)                              │
   │       │                  ▲                                                    │
   │       │           imu_yaw_integrator ◄── /imu/data_raw                        │
   │       │                  │                                                    │
   │       │              robot_localization ekf_node  ──► /odom + odom→base_link  │
   │       │                                                                       │
   │       └──────────────────────────────────────────────┐                       │
   │                                                       ▼                       │
   │                                            recon_webui_bridge                  │
   │                                            (rclpy on real OS thread)           │
   │                                                       │                       │
   │                                                       ▼                       │
   │                                            recon_webui (Flask + SocketIO       │
   │                                            on :80, AP "Recon" + wlan0)         │
   │                                                       │                       │
   │                                              ┌────────┴──────────┐            │
   │                                              ▼                   ▼            │
   │                                         db_node             /api/maps/*       │
   │                                              │                                │
   │                                              ▼                                │
   │                                       PostgreSQL (roomba_postgres container)  │
   └────────────────────────────────────────────────────────────────────────────────┘
                                              │ ap0 10.0.0.1
                                              │ wlan0 LAN
                                              ▼
                                          ┌────────────────────┐
                                          │  Browser           │
                                          │  http://recon.local │
                                          └────────────────────┘
```

**Key change from the pre-Inc-2 layout:**
- The Pi no longer talks to `/dev/ttyAMA0`. Both UART0 (GPIO 14/15) and
  the on-chip PL011 are unused — available for other peripherals later.
- All Pi-side LIDAR access goes through `/tmp/lidar_pty`, which is the
  slave end of a pty whose master is held by `esp32_uart_bridge`. The
  ldlidar driver treats it as a normal serial port; the byte stream is
  the LD14P's native UART output, relayed unmodified by the ESP32.

---

## 2. ROS2 node graph (today, sensor-test mode)

```
       ldlidar_node
            │ /scan
            ▼
       slam_toolbox ────► /map ──┐
            │                    │
            │ /tf (map→odom)     │
            ▼                    │
                                 │
   tf2_ros static_tf ────► odom→base_link (identity)
                                 │
                                 │
                       ┌─────────┴───────────┐
                       ▼                     ▼
              recon_webui_bridge        db_node
                       │
                       │ pose (from map→odom TF)
                       │ map (from /map)
                       │ events (from /robot/events)
                       ▼
                   recon_webui (Flask + SocketIO)
                       │
                       ▼
                   Browser
```

The currently-stubbed-out arrows for H2.1 / H3:

```
   esp32_uart_bridge (H2.1) ──► /imu/data_raw ──► imu_filter_madgwick ──► /imu/data
                            └─► /buttons/{save,reset,shutdown_request}

   imu_filter_madgwick + /odom (wheel placeholder = none) ──► ekf_node (H3) ──► /odom
                                                                           └─► /scanner/pose
```

---

## 3. Data flows in detail

### 3.1 LIDAR → SLAM → web UI

1. **LIDAR motor**: off by default. The user presses **Start scan** in
   the web UI → `set_scanning(True)` in `ros_bridge.py` calls
   `/lidar_enable` with `data=true`. The ESP32 pulls GPIO4 HIGH which
   saturates the S8050, completing the LIDAR's GND/RX path; the motor
   spins up within ~1 s.
2. **ESP32 relay**: the LD14P streams its native UART output at 230 400
   baud into ESP32 Serial2 (GPIO16 RX). The firmware drains up to 64 B
   per `loop()` iteration and forwards each chunk to the Pi as a
   `LIDAR_FRAME` envelope inside the binary framing protocol.
3. **Pi bridge unwrap**: `esp32_uart_bridge` reads the USB-CDC stream
   at 460 800 baud, demuxes by frame type, and writes the `LIDAR_FRAME`
   payload bytes to a pty master held by the bridge process. The
   matching slave path is symlinked to `/tmp/lidar_pty`.
4. **LD14P driver**: `ldlidar_stl_ros2_node` opens `/tmp/lidar_pty` as if
   it were `/dev/ttyAMA0`. Same parser code as before — the bytes on the
   wire are identical to what the LIDAR would have sent directly to the
   Pi UART. Publishes `sensor_msgs/LaserScan` on `/scan` at ~6 Hz.
   (Driver patched to wait indefinitely for first packet; see
   [`STATUS.md`](STATUS.md#7-known-limitations--tech-debt) #7.)
5. **SLAM**: `slam_toolbox` (online_async, Ceres solver) consumes
   `/scan`, maintains a pose graph, and publishes:
   - `nav_msgs/OccupancyGrid` on `/map` every `map_update_interval`
     seconds (currently 1.0 s).
   - `tf2_msgs/TFMessage` on `/tf` carrying `map → odom` at ~50 Hz.
   SLAM is **paused at boot** (`paused_new_measurements: true`); the
   same Start scan service-call resumes it in lockstep with the motor
   enable.
6. **Web bridge**: `recon_webui_bridge` composes
   `map→odom ∘ odom→base_link` for the scanner pose and pushes pose +
   map into per-channel buffers.
7. **WebSocket emit loop / client renderer**: unchanged from the pre-
   ESP32-relay layout. The map page also draws a walking-trail polyline
   from the last 600 pose samples.

### 3.1a Stop sequence (Pause scan)

Reverse order so SLAM never integrates a half-scan from a spinning-down
motor:

1. `set_scanning(False)` → `paused_new_measurements: true` (SLAM stops
   integrating any new `/scan` messages).
2. `/lidar_enable` with `data=false` → ESP32 pulls GPIO4 LOW → motor
   stops within a rotation period.
3. The ESP32 firmware ACKs the new state via `LIDAR_ACK`, which the
   bridge surfaces in `/esp32/diagnostics.lidar.acked_on`.

### 3.2 Save flow (today)

Two entry points, one DB write:

```
  Web UI POST /api/maps                          ╲
                                                  ╲
  /robot/events: "SAVE_MAP"  (from db_node       ╱── INSERT INTO maps
                              future hardware    ╱       + INSERT INTO map_events
                              SAVE button)      ╱
```

Either path ends at `db_node`'s `/robot/events` subscriber, which
inserts a row into `maps` and a `MapEvent(SAVED)` into `map_events`.
The web UI polls `/api/maps/events?since=<id>` every 3 s to pick up
saves that originated headlessly.

### 3.3 Mode toggle

```
  Browser click on IDLE / SCAN button
              │
              ▼
  socket.emit("set_mode", {mode})
              │
              ▼
  app.py on_set_mode() → ros_bridge.publish_mode()
              │
              ▼
  /robot/mode (std_msgs/String)
              │
              ├──► db_node     (logs mode in MapEvent? — TBD H5)
              └──► back into recon_webui_bridge → emit "robot_mode" to all clients
```

Mode is currently free-text `"IDLE" | "SCAN"`. H5 introduces a proper
session state machine (IDLE → SCAN → SAVE → IDLE) that gates DB writes.

### 3.4 ESP32 ⇄ Pi (bidirectional since Inc 1)

```
  ESP32 sketch (firmware/esp32/src/main.cpp)
        │ Serial.write(framing::send_imu / send_button / send_heartbeat /
        │              send_status / send_lidar_frame / send_lidar_ack)
        │ binary: [0xA5 0x5A][TYPE][LEN][PAYLOAD][CRC8]  (MAX_PAYLOAD = 64)
        │
        ├──► Pi /dev/ttyUSB0 (460 800 8N1)
        │       │
        │       ▼
        │  esp32_uart_bridge.py (recon_hardware)
        │       │ pyserial reader thread
        │       │ frame parser (sync hunt → length → CRC verify)
        │       ├─► /imu/data_raw      (FrameType.IMU)
        │       ├─► /buttons/*         (FrameType.BUTTON)
        │       ├─► /esp32/diagnostics (heartbeats + ack state)
        │       ├─► LIDAR_FRAME bytes → pty master → /tmp/lidar_pty
        │       └─► LIDAR_ACK         (diag.lidar.acked_on)
        │
        └◄── Pi /dev/ttyUSB0 (same link, reverse direction)
                  ▲
                  │ esp32_uart_bridge writes:
                  │   - LIDAR_EN frames on /lidar_enable service call
                  │   - LIDAR_EN refresh every 1 s while motor is on
                  │
                ESP32 firmware parses incoming frames in loop():
                  - LIDAR_EN=1 → set GPIO4 HIGH, update watchdog deadline
                  - LIDAR_EN=0 → set GPIO4 LOW immediately
                  - watchdog: 3 s with no refresh → force LOW
```

The frame parser must handle byte loss: lost a sync byte → drop until
next 0xA5 0x5A; bad CRC → drop frame, log at debug. See
[`UART_PROTOCOL.md`](UART_PROTOCOL.md) §4 for the resync algorithm.

The pty path is held open by the bridge process: `os.openpty()` returns
master + slave fds, the slave fd is `os.close()`'d immediately, and the
master is set non-blocking. Writes to the master that race ahead of the
LD14P driver's reads return `EAGAIN`; the bridge counts those as
`pty_overflows` rather than blocking the reader thread. Once the driver
opens the slave and starts draining, the kernel pty buffer keeps both
sides in lockstep with zero data loss in steady state.

---

## 4. TF tree (today and target)

### Today (H1.6, sensor-test mode)

```
map ──► odom ──► base_link ──► laser_frame
       (slam) (static, identity)  (static, from ldlidar launch)
```

The `odom → base_link` transform is the **identity** — published by a
static `tf2_ros static_transform_publisher` because we have no source
of motion estimate. Result: the scanner's pose in the map frame is
literally `map → odom`, so we read it straight off `/tf`.

### After H3 (IMU EKF lands)

```
map ──► odom ──► base_link ──► laser_frame
        (ekf_node from /imu/data,        (static)
         no wheel odom)
```

`robot_localization` ekf_node will consume `/imu/data` (orientation +
filtered angular velocity) and publish a real `odom → base_link` TF +
`/odom`. SLAM still publishes `map → odom`. The web UI keeps deriving
pose from TF — no client change needed.

### After H6 (enclosure)

The static `base_link → laser_frame` transform will reflect the actual
mechanical offset between the scanner's mounting point and the LIDAR
optics, so map coordinates correspond to a physical reference on the
device.

---

## 5. Threading model (Pi-side webui)

Three concurrent execution contexts in `recon_webui` Python process:

```
  ┌─────────────────────────────────┐
  │ eventlet green thread loop      │  ← Flask + SocketIO live here
  │  - HTTP request handlers        │  ← `tpool.execute()` for blocking calls
  │  - WebSocket emit loop          │
  │  - DB queries (via tpool)       │
  └────────────┬────────────────────┘
               │  queue.Queue (events out of rclpy)
  ┌────────────▼────────────────────┐
  │ Real OS thread (`ros2_bridge_spin`) │  ← rclpy.spin(node)
  │  - Subscriber callbacks         │
  │  - Pushes events to queue       │
  └─────────────────────────────────┘
               ▲
               │  socketio.emit() is FORBIDDEN here.
               │  Use the queue.
```

Hard rules (see [`AGENT_RULES.md`](AGENT_RULES.md) §6):

1. **`eventlet.monkey_patch()` is the first executable line** of `app.py`.
   Imports come AFTER monkey-patch.
2. **`rclpy.spin()` runs on a real OS thread**, not an eventlet green
   thread. Use `eventlet.patcher.original("threading")` to escape the
   greened threading module.
3. **Never call `socketio.emit()` from the rclpy thread.** Use a
   `queue.Queue(maxsize=64)` to hand events to the eventlet loop, and
   have `emit_loop()` drain it.
4. **Use `tpool.execute()` for blocking calls** (DB queries, subprocess)
   inside Flask routes — otherwise they freeze the green-thread loop.

---

## 6. Mock-fallback architecture

The web UI doesn't have separate "demo" and "live" code paths. Every
data source is wrapped in a `DataChannel`:

```python
DataChannel(
    topic="/scanner/pose",
    timeout_s=2.0,
    mock_fn=mock_data.mock_robot_pose,
)
```

- `is_live()` returns True iff a real ROS2 message arrived within
  `timeout_s`.
- `get()` returns the most recent real value when live, otherwise the
  result of `mock_fn()`.

Switching from demo (`setup.sh demo`, no ROS2) to sensor-test
(`setup.sh sensor-test`, real LIDAR) requires **zero code changes** —
the channels self-detect.

---

## 7. ESP32 firmware structure (single sketch, no FreeRTOS)

```
   setup()
     ├─ pinMode(PIN_LIDAR_EN); digitalWrite(LOW)   ← FIRST line — motor off
     ├─ pinMode + Serial.begin(460800)
     ├─ Serial2.setRxBufferSize(1024); Serial2.begin(230400, …, GPIO16)
     ├─ Button.begin × 3
     ├─ imu::begin()
     ├─ framing::send_status_diag(boot flags + IMU identity)
     └─ schedule next IMU/heartbeat/LED deadlines

   loop() — runs as fast as possible (~kHz)
     ├─ Drain Serial (incoming from Pi):
     │     parse frames; on LIDAR_EN: set GPIO4, update refresh deadline
     ├─ Drain Serial2 (incoming from LD14P, only when motor on):
     │     forward up to 64 B as LIDAR_FRAME
     ├─ Watchdog: if motor on and no refresh in 3 s → force motor off
     ├─ Button.update × 3      (debounce + edge → framing::send_button)
     ├─ if (now ≥ next_imu_ms)
     │     imu::read() → framing::send_imu()
     │     on failure: imu::begin() retry, framing::send_status()
     ├─ if (now ≥ next_heartbeat_ms)
     │     framing::send_heartbeat(uptime)
     └─ update_status_led()
```

No FreeRTOS tasks. Cooperative scheduling on `millis()` deadlines is
sufficient for this workload. With the motor on, total UART traffic is:
- IMU @ 100 Hz × 29 B = ~3 KB/s
- LIDAR_FRAME relay (LD14P at 230 400 baud) = ~23 KB/s
- Heartbeats + acks + Pi → ESP refreshes = negligible
**Total ~26 KB/s**, well under the 460 800 baud budget of ~46 KB/s.

Signed `(int32_t)` math is used throughout for all `millis()`-based
comparisons — see the watchdog block in `loop()` and the IMU/heartbeat
deadline checks. Unsigned subtraction was a real bug in the first
version of the watchdog (refresh frames received the same loop iteration
as the comparison caused `now - last_refresh` to underflow, leading to
spurious motor-OFF events every ~1 ms after each refresh).

---

## 8. Key invariants

These are properties the system relies on. Breaking them silently
breaks downstream stages.

| Invariant | Why it matters | Enforced by |
| --------- | -------------- | ----------- |
| `slam_toolbox` always publishes `map → odom` TF, even when the device is stationary | The web UI derives pose from this TF; no TF = no red arrow | `slam_params.yaml: minimum_travel_distance: 0.0` |
| Eventlet monkey-patch runs before any other import in `app.py` | Prevents stdlib `threading.RLock` instances from being non-greened | Code review + `# noqa` comment marker |
| `socketio.emit()` only from eventlet thread | Calling from rclpy thread caused a documented deadlock | `RosBridge` uses `queue.Queue` exclusively |
| ESP32 frame CRC covers TYPE+LEN+PAYLOAD (not the sync bytes) | Pi-side parser must match | [`UART_PROTOCOL.md`](UART_PROTOCOL.md) is canonical |
| `PIN_LIDAR_EN` (GPIO4) is set LOW as the very first line of `setup()` | The motor must be off through the ESP32 boot window — even a brief HIGH would spin the LIDAR before firmware is ready | `firmware/esp32/src/main.cpp:setup()` |
| LIDAR_EN refreshes from the Pi while motor is on | The firmware's 3 s watchdog kills the motor if refreshes stop (handles Pi crash / disconnected USB) | `esp32_uart_bridge` `_refresh_lidar` timer @ 1 Hz |
| Start scan enables LIDAR _before_ unpausing SLAM; Pause scan does the opposite | Prevents SLAM from integrating a partial scan during motor spin-up/down | `ros_bridge.set_scanning()` ordering, lines 471-475 |
| `set_parameters` (not the `Pause` service) drives SLAM paused state | The `slam_toolbox/srv/Pause` service is mislabeled as a toggle but actually only ever sets to paused — never resumes | Documented in `ros_bridge.py` `_call_slam_pause` |
| Signed `(int32_t)` math for all `millis()`-based comparisons in firmware | Unsigned subtraction underflows when one millis read happens slightly later than the captured `now`, causing immediate-trigger bugs | Convention in `main.cpp`; the watchdog was a real bug fix for this |
| The Postgres container stays named `roomba_postgres` | Renaming would orphan the existing scan data on dev devices | `docker/docker-compose.yaml` left alone in H1 |
| All ROS2 nodes launched via `setup.sh` (never raw `ros2 launch`) | Bare launch doesn't activate the venv → SQLAlchemy/eventlet missing | Documented in `AGENT_RULES.md` |
