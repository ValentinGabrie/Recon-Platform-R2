==================================================
Recon-Platform-R2 — Data Flow and Code-Path Atlas
==================================================

:Project: Recon-Platform-R2 (handheld LIDAR scanner)
:Branch: ``handheld``
:Companion docs: :doc:`SPEC <SPEC>` · :doc:`ARCHITECTURE <ARCHITECTURE>` · :doc:`STATUS <STATUS>`

.. contents:: Table of contents
   :depth: 2
   :local:

Purpose
=======

This document traces every major data flow in the system byte-by-byte
from its origin to its consumer, with file paths and function names at
each step. Where SPEC.md or ARCHITECTURE.md describes *what* moves,
this document describes *where in the code* it moves.

The intent is a developer-oriented atlas: a contributor who needs to
modify a flow can find the entry, exit, and every transformation in
between without grep'ing the workspace.

For high-level component description and design rationale see
:doc:`SPEC` and :doc:`ARCHITECTURE`. For the binding contributor
constraints see :doc:`AGENT_RULES`. For exhaustive prose see
:doc:`THESIS_TECHNICAL_DOCUMENTATION`.

.. note::

   Every "code-path" subsection in this document lists the
   sequence of files and functions a data item visits. File
   paths are relative to the repository root. Functions are
   given as ``filename:function_name`` so they double as
   navigation hints in any editor that resolves the syntax.

System map
==========

Top-level components and the physical and logical channels between
them::

   ┌──────────┐    UART 230400 8N1                  ┌─────────────────────────┐
   │  LD14P   │ ─────────────────────────────────►  │ ESP32 Serial2 (GPIO16)  │
   │ LIDAR    │      (white TX wire)                │                          │
   │          │ ◄── GPIO4 → S8050 → GND/RX ────────┤ GPIO4 = LIDAR_EN          │
   └──────────┘                                     │ MPU-6050 on I²C 0x68     │
                                                    │ 3 buttons on GPIO 25/26/27│
                                                    └────────────┬──────────────┘
                                                                 │
                                                       USB-CDC 460 800 8N1
                                                                 │
                                                                 ▼
   ┌──────────────────────────────────────────────────────────────────────────┐
   │  Raspberry Pi 5 — Ubuntu 24.04 — ROS2 Jazzy                              │
   │                                                                          │
   │  ┌───────────────────────┐    ┌─────────────────────┐                    │
   │  │ esp32_uart_bridge.py  │    │ ldlidar_stl_ros2    │                    │
   │  │  /dev/ttyUSB0 ─► pty  │ ─► │ opens /tmp/lidar_pty│ ─► /scan           │
   │  │   IMU, BUTTON,        │    └──────────┬──────────┘                    │
   │  │   HEARTBEAT, STATUS   │               ▼                                │
   │  │   ► /imu/data_raw     │           slam_toolbox ─► /map, /tf           │
   │  │   ► /buttons/*        │                                                │
   │  │   ► /esp32/diagnostics│   imu_yaw_integrator ─► /imu/data ─► ekf_node │
   │  │   /lidar_enable srv   │                              ─► /odom, /tf    │
   │  └───────────────────────┘                                                │
   │                                                                          │
   │  recon_webui (Flask + SocketIO + rclpy on real OS thread)                │
   │       │                                                                  │
   │       ├──► db_node ──► PostgreSQL (Docker container roomba_postgres)     │
   │       │                                                                  │
   │       └──► WebSocket clients on :80                                       │
   └──────────────────────────────────────────────────────────────────────────┘
                                          │ ap0 10.0.0.1  /  wlan0 LAN
                                          ▼
                                  Browser at http://recon.local/

Process layout in ``setup.sh full`` mode:

.. list-table::
   :widths: 18 35 47
   :header-rows: 1

   * - tmux window
     - Executable / module
     - Subscribed / published topics
   * - ``esp32``
     - ``recon_hardware.esp32_uart_bridge``
     - Pub: ``/imu/data_raw``, ``/buttons/*``, ``/esp32/diagnostics``; Srv: ``/lidar_enable``
   * - ``imu_yaw``
     - ``recon_hardware.imu_yaw_integrator``
     - Sub: ``/imu/data_raw``; Pub: ``/imu/data``
   * - ``imu_tf``
     - ``tf2_ros static_transform_publisher``
     - Static ``base_link → imu_link``
   * - ``ekf``
     - ``robot_localization ekf_node``
     - Sub: ``/imu/data``; Pub: ``/odom``, ``odom → base_link`` TF
   * - ``lidar``
     - ``ldlidar_stl_ros2_node`` + static TF
     - Pub: ``/scan``, ``base_link → laser_frame``
   * - ``slam_tb``
     - ``slam_toolbox async_slam_toolbox_node``
     - Sub: ``/scan``, ``/imu/data``; Pub: ``/map``, ``map → odom`` TF; Srv: ``/slam_toolbox/{set_parameters,reset}``
   * - ``db_node``
     - ``recon_db.db_node``
     - Sub: ``/robot/events``, ``/robot/mode``, ``/map``
   * - ``webui``
     - ``recon_webui.app`` (Flask + SocketIO + ``RosBridge``)
     - Sub: everything; serves HTTP+WS on :80

Concurrency boundaries
======================

Three execution contexts live in the same ``recon_webui`` Python process:

.. code-block:: text

   ┌─────────────────────────────────────────────────────┐
   │  eventlet hub (green threads)                       │
   │   • Flask routes (every HTTP request)               │
   │   • SocketIO emit_loop                              │
   │   • DB queries via eventlet.tpool.execute(...)      │
   │   • Sync ROS service helpers:                       │
   │       _call_slam_pause, _call_lidar_enable,         │
   │       clear_map, auto_pause                         │
   └────────────┬────────────────────────────────────────┘
                │ queue.Queue(maxsize=64) — events out of rclpy
   ┌────────────▼────────────────────────────────────────┐
   │  Real OS thread "ros2_bridge_spin"                  │
   │   • rclpy.spin(node) — subscription callbacks       │
   │   • Writes DataChannel.on_ros_message(...)          │
   │   • Pushes events into queue.Queue                  │
   │   • NEVER calls socketio.emit()                     │
   └─────────────────────────────────────────────────────┘
   ┌─────────────────────────────────────────────────────┐
   │  Separate OS thread "esp32_serial_reader"           │
   │   • pyserial blocking reads from /dev/ttyUSB0       │
   │   • FrameParser.feed_bytes(chunk)                   │
   │   • Publishes Imu / Empty / String messages         │
   └─────────────────────────────────────────────────────┘

Binding invariants (:doc:`AGENT_RULES` §6):

#. ``eventlet.monkey_patch()`` is the **first executable line** of
   ``recon_webui/app.py``. Every other import comes after.
#. ``rclpy.spin()`` runs on a real OS thread obtained via
   ``eventlet.patcher.original("threading").Thread(...)``.
#. ``socketio.emit()`` is **never** called from the rclpy thread.
   All emits go through the ``queue.Queue`` bridge.
#. Flask routes that touch the DB go through
   ``eventlet.tpool.execute(callable)``.
#. Sync ROS2 service helpers (``_call_slam_pause``,
   ``_call_lidar_enable``, ``clear_map``) **only** run on an
   eventlet greenlet. Auto-pause uses
   ``eventlet.spawn_after(5.0, ros_bridge.auto_pause)`` instead of
   a rclpy timer for this reason.

.. _flow-lidar:

Flow 1 — LIDAR scan → /scan → /map
==================================

The full sensor-to-SLAM data path. This is the highest-bandwidth
flow in the system (~23 KB/s sustained when the motor is on) and
the longest code path.

Sequence
--------

.. code-block:: text

   LD14P (motor spinning)
       │ 230 400 baud 8N1 on TX (white wire)
       ▼
   ESP32 Serial2 RX (GPIO 16)
       │ Arduino-core driver buffers up to 1024 bytes
       ▼
   firmware/esp32/src/main.cpp:loop()
       │ each iteration: drain up to 64 bytes
       │ framing::send_lidar_frame(buf, len)
       ▼
   firmware/esp32/src/framing.cpp:send_frame()
       │ adds [0xA5][0x5A][0x05][LEN][PAYLOAD][CRC8]
       │ writes to Serial (USB-CDC, 460 800 baud)
       ▼
   USB-Serial bridge (CP2102 or CH340)
       │ enumerates as /dev/ttyUSB0 on the Pi
       ▼
   Pi-side reader thread
       │ recon_hardware/esp32_uart_bridge.py:_reader_loop()
       │ ser.read(128) → FrameParser.feed_bytes(chunk)
       ▼
   recon_hardware/framing.py:FrameParser.feed()
       │ HUNT0 → HUNT1 → TYPE → LEN → PAY → CRC
       │ on valid CRC: returns (FrameType.LIDAR_FRAME, payload_bytes)
       ▼
   esp32_uart_bridge.py:_handle_frame()
       │ LIDAR_FRAME branch:
       │   counters["lidar_frame"] += 1
       │   counters["lidar_bytes"] += len(payload)
       │   _write_lidar_bytes(payload)
       ▼
   esp32_uart_bridge.py:_write_lidar_bytes()
       │ os.write(pty_master_fd, data)
       │ on EAGAIN: pty_overflow_count += 1 (drop)
       ▼
   Linux kernel pty buffer
       │ /dev/pts/N — slave-side reader sees the bytes
       │ symlinked as /tmp/lidar_pty
       ▼
   ldlidar_stl_ros2_node (vendored, C++)
       │ opens /tmp/lidar_pty as if it were /dev/ttyAMA0
       │ LDLidar SDK parses LD14P packets
       ▼
   ldlidar_stl_ros2/src/demo.cpp:ToLaserscanMessagePublish()
       │ buckets variable-count input points into 720 fixed bins
       │ "take nearer hit" merge per bin
       │ builds sensor_msgs/LaserScan
       ▼
   /scan topic (RELIABLE, ~6 Hz)
       │
       ▼
   slam_toolbox subscriber
       │ runs scan matching against the pose graph
       │ updates internal occupancy grid
       │ at every map_update_interval (1.0 s):
       ▼
   /map topic (RELIABLE TRANSIENT_LOCAL, 1 Hz)
       │
       ▼
   recon_webui_bridge:_map_callback()
       │ converts to dict {width, height, resolution, origin_x, origin_y, data}
       │ DataChannel("map").on_ros_message(grid_dict)
       ▼
   emit_loop in recon_webui/app.py:emit_loop()
       │ every 1/rates["map_update"] = 200 ms:
       │   socketio.emit("map_update", channels["map"].get())
       ▼
   Browser, map.html:socket.on("map_update", ...)
       │ drawMap(d) → drawTrail(d) → drawRobot()
       ▼
   <canvas id="mapCanvas"> repaints

Key files
---------

==================================================================== ==============================================
File                                                                  Purpose
==================================================================== ==============================================
``firmware/esp32/src/main.cpp:loop()``                                 Drain Serial2, gate on ``g_lidar_enabled``
``firmware/esp32/src/framing.cpp:send_frame()``                        Build wire frame + CRC
``firmware/esp32/src/framing.cpp:send_lidar_frame()``                  ESP→Pi LIDAR_FRAME envelope
``recon_hardware/esp32_uart_bridge.py:_reader_loop()``                 OS thread reading /dev/ttyUSB0
``recon_hardware/framing.py:FrameParser.feed()``                       6-state byte parser
``recon_hardware/esp32_uart_bridge.py:_handle_frame()``                Demux by FrameType
``recon_hardware/esp32_uart_bridge.py:_setup_lidar_pty()``             ``pty.openpty()`` + symlink
``recon_hardware/esp32_uart_bridge.py:_write_lidar_bytes()``           Non-blocking ``os.write`` to pty master
``roomba_ws/src/ldlidar_stl_ros2/launch/ld14p.launch.py``              Opens ``/tmp/lidar_pty`` at 230 400
``roomba_ws/src/ldlidar_stl_ros2/src/demo.cpp:ToLaserscanMessagePublish()`` Fixed 720-beam re-bucketing
``recon_webui/ros_bridge.py:_map_callback()``                          Pi-side webui consumer
``recon_webui/app.py:emit_loop()``                                     Periodic SocketIO emit
``recon_webui/templates/map.html`` (drawMap/drawTrail/drawRobot)       Canvas render
==================================================================== ==============================================

Critical invariants
-------------------

.. warning::

   **720-beam fixed geometry.** ``demo.cpp`` is locally patched
   (nested commit ``42688f6``) so ``beam_size = 720``. Karto/
   slam_toolbox locks the beam count from the first scan and
   rejects every later scan whose count differs. If this patch is
   reverted, the map silently stops updating after one rotation.
   See :doc:`STATUS` §7 #14.

.. warning::

   **PTY non-blocking.** The pty master fd is set ``O_NONBLOCK``.
   Writes that race ahead of the LD14P driver opening the slave
   return ``EAGAIN`` and the chunk is dropped, incrementing
   ``lidar.pty_overflows``. Initial overflows during the startup
   race are expected; steady-state overflows indicate a real
   problem.

.. note::

   **Latency budget**: ~16 ms from photon to ``/scan`` (LIDAR
   serialisation ~10 ms, ESP32 hop ~3 ms, Pi-side parsing + pty +
   driver parse ~3 ms). Negligible at walking speed.

.. _flow-imu:

Flow 2 — IMU → /imu/data_raw → /imu/data → /odom
================================================

The 100 Hz inertial fusion path. Drives the device's heading
estimate and provides scan_matcher with a prior.

Sequence
--------

.. code-block:: text

   MPU-6050 (I²C 0x68 on ESP32)
       │ 14-byte burst from REG_ACCEL_XOUT_H (0x3B)
       ▼
   firmware/esp32/src/imu.cpp:imu::read()
       │ raw int16 × ACCEL_LSB_TO_MS2 / GYRO_LSB_TO_RADS
       │ → ImuSample {ax, ay, az, gx, gy, gz} in SI units
       ▼
   firmware/esp32/src/main.cpp:loop() — IMU branch
       │ scheduled every 10 ms (IMU_PERIOD_MS)
       │ framing::send_imu(ax, ay, az, gx, gy, gz)
       ▼
   firmware/esp32/src/framing.cpp:send_imu()
       │ pack 6 × float32 LE into 24 B payload
       │ send_frame(FRAME_IMU, payload, 24)
       ▼
   USB-CDC 460 800 baud → /dev/ttyUSB0
       ▼
   esp32_uart_bridge.py:_handle_frame() — IMU branch
       │ counters["imu"] += 1
       │ _publish_imu(sample)
       ▼
   esp32_uart_bridge.py:_publish_imu()
       │ sensor_msgs/Imu — stamp = self.get_clock().now()
       │ linear_acceleration + angular_velocity filled
       │ orientation left at zero
       │ QoS: BEST_EFFORT, depth=10
       ▼
   /imu/data_raw (100 Hz, BEST_EFFORT)
       ▼
   ┌──────────────────────────────┬─────────────────────────────┐
   │                              │                             │
   ▼                              ▼                             ▼
   imu_yaw_integrator.py     recon_webui_bridge      (debug/recording)
   :_on_imu()                :_imu_callback()
       │                          │
       │                          │ DataChannel("imu").on_ros_message
       ▼                          ▼
   integrate_yaw(prev, gyro_z, dt)
       │ wrap to (-π, π]
       │
       ▼
   yaw_to_quaternion_xyzw(yaw)
       │ (0, 0, sin(yaw/2), cos(yaw/2))
       │
       ▼
   Output Imu message
       │ same accel/gyro values
       │ orientation = quaternion
       │ orientation_covariance diag = [0.05, 0.05, 0.10]
       ▼
   /imu/data (100 Hz, RELIABLE)
       ▼
   ┌──────────────────────────────┐
   │                              │
   ▼                              ▼
   ekf_node                  slam_toolbox
   (robot_localization)       (uses orientation as
       │                       scan-match prior)
       │ imu0_config: only yaw + vyaw
       │ frequency: 30.0
       │
       ▼
   /odom (30 Hz)
       │ position = (0, 0) always
       │ orientation = integrated yaw
       │
       ▼
   /tf: odom → base_link (30 Hz)

Key files
---------

================================================================ ===========================================
File                                                              Purpose
================================================================ ===========================================
``firmware/esp32/src/imu.cpp:imu::read()``                        Raw-bytes → SI units on the ESP32
``firmware/esp32/src/main.cpp:loop()`` (IMU branch)               100 Hz scheduling
``firmware/esp32/src/framing.cpp:send_imu()``                     Pack 6 floats + CRC
``recon_hardware/esp32_uart_bridge.py:_publish_imu()``            Build sensor_msgs/Imu
``recon_hardware/imu_yaw_integrator.py:integrate_yaw()``          Pure-function integration step
``recon_hardware/imu_yaw_integrator.py:yaw_to_quaternion_xyzw()``  2-D yaw → quaternion
``recon_hardware/imu_yaw_integrator.py:ImuYawIntegrator._on_imu()`` Subscriber callback
``roomba_ws/config/ekf.yaml``                                      EKF configuration
``recon_webui/ros_bridge.py:_imu_callback()``                     Web UI consumer (for diagnostics)
================================================================ ===========================================

QoS note
--------

.. note::

   The bridge publishes ``/imu/data_raw`` with **BEST_EFFORT** QoS
   to match the integrator's subscription. A QoS mismatch
   silently fails to connect: ROS2 DDS does not connect a
   BEST_EFFORT publisher to a RELIABLE subscriber. The yaw
   integrator re-publishes on ``/imu/data`` with the default
   **RELIABLE** QoS because slam_toolbox subscribes with reliable
   QoS.

.. _flow-button:

Flow 3 — Button press → ROS event
=================================

Hardware button → debounce → framing → ROS topic → event log /
DB.

Sequence
--------

.. code-block:: text

   User presses SAVE button (GPIO 27 active-low)
       │
       ▼
   firmware/esp32/src/buttons.cpp:Button::update()
       │ digitalRead == LOW
       │ wait for BUTTON_DEBOUNCE_MS = 25 ms of stable LOW
       │ emit on_event(BTN_SAVE, BTN_PRESSED)
       ▼
   firmware/esp32/src/main.cpp:on_button_event()
       │ framing::send_button(id=2, state=1)
       ▼
   USB-CDC → /dev/ttyUSB0
       ▼
   esp32_uart_bridge.py:_handle_frame() — BUTTON branch
       │ _publish_button(evt)
       ▼
   esp32_uart_bridge.py:_publish_button()
       │ append to _last_button_events ring (8 entries)
       │ if state == PRESSED and id == SAVE:
       │   _btn_save_pub.publish(Empty())
       ▼
   /buttons/save (event)
       │
       ▼
   ┌───────────────────────┬────────────────────────────┐
   │                       │                            │
   ▼                       ▼                            ▼
   recon_webui_bridge      (db_node — H5: subscribes here for save trigger)
   :_button_callback("SAVE button")
       │
       │ event_queue.put_nowait({type: "BUTTON", message: "SAVE button pressed"})
       ▼
   emit_loop drains queue → socketio.emit("robot_event", ...)
       ▼
   Browser, map.html:document.addEventListener("robotEvent", ...)
       │ prepends entry to <div id="eventTailList">

Long-press variant
------------------

A button held for ≥ ``BUTTON_LONGPRESS_MS`` (2 000 ms) emits one
extra ``LONGPRESS`` event between ``PRESSED`` and ``RELEASED``.

For SHUTDOWN long-press specifically, the bridge publishes on
``/buttons/shutdown_longpress`` rather than the regular
``/buttons/shutdown_request``. Future H6 work will hook a
``shutdown_handler`` node to this topic that invokes
``sudo shutdown -h now``.

.. _flow-start-scan:

Flow 4 — Start scan (LIDAR motor + SLAM unpause)
================================================

Triggered by user clicking the **Start scan** button on
``map.html`` (or pressing ``SPACE``, or calling
``POST /api/scan/start`` directly).

Sequence
--------

.. code-block:: text

   User clicks "Start scan"
       │
       ▼
   map.html:toggleScan()
       │ fetch("/api/scan/start", { method: "POST" })
       ▼
   recon_webui/app.py:api_scan_start()
       │ ros_bridge.set_scanning(True)
       ▼
   recon_webui/ros_bridge.py:set_scanning(active=True)
       │
       │ # ORDER MATTERS: enable LIDAR FIRST, then unpause SLAM
       │ lidar_responded = self._call_lidar_enable(True)
       │
       ▼
   ros_bridge.py:_call_lidar_enable(True)
       │ req = SetBool.Request(); req.data = True
       │ future = self._lidar_enable_client.call_async(req)
       │ while not future.done(): time.sleep(0.02)   ← greened sleep
       ▼
   /lidar_enable service → esp32_uart_bridge
       ▼
   esp32_uart_bridge.py:_on_lidar_enable(req, resp)
       │ self._lidar_desired = True
       │ _send_lidar_en(True) → encode_lidar_en(True) → ser.write(frame)
       ▼
   USB-CDC → ESP32 Serial.read()
       ▼
   firmware/esp32/src/main.cpp:loop() — incoming-frame branch
       │ g_pi_parser.feed(byte, frame)
       │ handle_incoming_frame(frame)
       ▼
   main.cpp:handle_incoming_frame() — LIDAR_EN branch
       │ if payload[0] != 0:
       │   g_lidar_last_refresh_ms = millis()  ← reset watchdog
       │ set_lidar_enabled(true)
       ▼
   main.cpp:set_lidar_enabled(true)
       │ digitalWrite(PIN_LIDAR_EN, HIGH)
       │ S8050 saturates → LIDAR motor spins up
       │ framing::send_lidar_ack(1)  ← ESP→Pi confirmation
       ▼
   USB-CDC → /dev/ttyUSB0 (back to the Pi)
       ▼
   esp32_uart_bridge.py:_handle_frame() — LIDAR_ACK branch
       │ self._lidar_acked = True
       │ logs "LIDAR motor → ON"
       ▼
   (back in set_scanning, now SLAM)
       │ responded = self._call_slam_pause(not active)
       │              = self._call_slam_pause(False)  ← unpause
       ▼
   ros_bridge.py:_call_slam_pause(paused=False)
       │ build SetParameters.Request:
       │   parameters = [{ name: "paused_new_measurements",
       │                   value.type: BOOL, value.bool_value: False }]
       │ future = self._slam_pause_client.call_async(req)
       │ while not future.done(): time.sleep(0.02)
       ▼
   /slam_toolbox/set_parameters service
       │
       ▼
   slam_toolbox internal — flips paused_new_measurements = False
       │
       ▼
   slam_toolbox starts integrating /scan messages
       │
       ▼
   /map topic resumes publishing

After the round-trip
--------------------

After both service calls complete, ``set_scanning`` does:

.. code-block:: python

   self._scan_active = True
   new_mode = "SCAN"
   self.publish_mode(new_mode)   # publishes std_msgs/String on /robot/mode
   self._event_queue.put_nowait({
       "type": "MODE_CHANGE",
       "message": "Scan started"
       + (... if not responded else "")
       + (... if not lidar_responded else ""),
   })
   return {"active": True, "slam_responded": ..., "lidar_responded": ..., "mode": "SCAN"}

The Flask route returns this dict to the browser. The JS handler
flips ``scanActive``, repaints the hero pill, and emits a toast
on partial failure.

Watchdog refresh
----------------

While the motor is on, the bridge refreshes ``LIDAR_EN=1`` every
1.0 s (``config/esp32_bridge.yaml: lidar_refresh_s``) via
``_refresh_lidar`` — a rclpy timer callback. The firmware's
3-second watchdog forces the motor off if no refresh arrives
within that window; the 1 s refresh interval leaves comfortable
headroom.

.. _flow-pause-scan:

Flow 5 — Pause scan (reverse order)
===================================

Mirror of :ref:`flow-start-scan` with **reversed ordering**.

Sequence
--------

.. code-block:: text

   User clicks "Pause scan" (or presses SPACE while scanning)
       │
       ▼
   map.html:toggleScan() → fetch("/api/scan/pause", { method: "POST" })
       ▼
   recon_webui/app.py:api_scan_pause()
       │ ros_bridge.set_scanning(False)
       ▼
   ros_bridge.py:set_scanning(active=False)
       │
       │ # ORDER MATTERS: pause SLAM FIRST, then disable LIDAR
       │ responded       = self._call_slam_pause(True)
       │ lidar_responded = self._call_lidar_enable(False)
       │
       ▼
   slam_toolbox flips paused_new_measurements = True
       │ stops integrating /scan
       ▼
   /lidar_enable(False) → ESP32 LIDAR_EN=0 frame
       ▼
   firmware/esp32/src/main.cpp:handle_incoming_frame()
       │ payload[0] == 0 → set_lidar_enabled(false)
       ▼
   digitalWrite(PIN_LIDAR_EN, LOW)
       │ S8050 cuts off → LIDAR motor coasts to a stop
       │ framing::send_lidar_ack(0)
       ▼
   bridge — LIDAR_ACK → _lidar_acked = False
       ▼
   browser hero pill flips to "PAUSED"

Why reverse order
-----------------

.. warning::

   **Start order: LIDAR on first, then SLAM unpause.** If SLAM
   unpaused first, it would integrate empty/partial scans during
   the motor's ~1 s spin-up.

   **Pause order: SLAM pause first, then LIDAR off.** If the
   motor cut first, SLAM would integrate the partial scan
   produced as the motor spins down. Either ordering bug would
   corrupt the map momentarily.

.. _flow-clear-map:

Flow 6 — Clear map
==================

Resets slam_toolbox's pose graph and occupancy grid in place,
without restarting the node or touching saved maps. Triggered by
the **Clear map** button on ``map.html`` (or ``C`` shortcut).

Sequence
--------

.. code-block:: text

   User clicks "Clear map" → confirm() dialog → OK
       │
       ▼
   map.html:clearMap()
       │ fetch("/api/map/clear", { method: "POST" })
       ▼
   recon_webui/app.py:api_map_clear()
       │ ros_bridge.clear_map()
       ▼
   ros_bridge.py:clear_map(timeout_s=3.0)
       │ req = SlamReset.Request()
       │ req.pause_new_measurements = False   ← preserve scan state
       │ future = self._slam_reset_client.call_async(req)
       │ poll future with greened time.sleep(0.02)
       ▼
   /slam_toolbox/reset (slam_toolbox/srv/Reset)
       │
       ▼
   slam_toolbox: empties pose graph + occupancy grid in place
       │ keeps current paused_new_measurements state
       │ returns RESULT_SUCCESS (~40 ms paused, ~300 ms active)
       ▼
   ros_bridge.py:clear_map()
       │ if result == RESULT_SUCCESS:
       │   event_queue.put_nowait({type: "MAP_CLEAR", message: "Map cleared"})
       │ returns {"success": True/False, "slam_responded": True,
       │          "message": "reset result_code=..."}
       ▼
   app.py returns JSON to browser
       ▼
   map.html:clearMap() success handler
       │ poseTrail.length = 0
       │ ctx.fillStyle = "#000"; ctx.fillRect(0, 0, w, h)
       │ drawRobot()
       │ document.getElementById("mapInfo").textContent = "cleared · ..."
       │ pillFlash("CLEARED", "cleared")
       │ showToast("Map cleared", "ok")

Why the local canvas wipe?
--------------------------

.. note::

   slam_toolbox does not publish ``/map`` while paused. If the
   user clears the map while paused (a common case — "pause,
   then clear, then start a fresh scan"), no fresh ``/map``
   message would arrive to repaint the canvas. The client-side
   wipe is what makes the empty state immediately visible.

.. _flow-save-rest:

Flow 7 — Save map (web UI button)
=================================

User-driven map save through the REST endpoint.

Sequence
--------

.. code-block:: text

   User clicks "Save map" (or presses S)
       │
       ▼
   map.html:saveMap()
       │ prompt("Map name (blank for auto):")
       │ fetch("/api/maps", { method: "POST", body: JSON.stringify({name}) })
       ▼
   recon_webui/app.py:api_save_map()
       │ map_data = channels["map"].get()  ← current live grid
       │ if not name: name = "map_YYYYMMDD_HHMMSS"
       │ tpool.execute(_save)              ← off the eventlet hub
       ▼
   _save() inner function (runs on a thread from eventlet's tpool)
       │ session = db_factory()
       │ record = MapRecord(name=..., map_data=json.dumps(data).encode("utf-8"),
       │                     origin_x=..., origin_y=..., resolution=...,
       │                     width=..., height=...)
       │ session.add(record); session.flush()  ← get record.id
       │ event = MapEvent(event_type="SAVED", map_id=record.id, map_name=name)
       │ session.add(event); session.commit()
       │ return record.id
       ▼
   PostgreSQL roomba_postgres container (port 5432, localhost only)
       │ INSERT INTO maps (...)
       │ INSERT INTO map_events (...)
       │
       ▼
   app.py returns {"success": True, "id": map_id, "message": ...}
       ▼
   map.html:saveMap() → showToast(message, "ok")

.. _flow-save-button:

Flow 8 — Save map (hardware SAVE button)
========================================

Headless save path — works even with no browser open.

Sequence
--------

.. code-block:: text

   User presses SAVE button on the device
       │
       ▼  (see Flow 3 for the button → /buttons/save plumbing)
   /buttons/save (std_msgs/Empty)
       │
       ▼
   [planned H5 wiring: shutdown_handler / session manager
    subscribes to /buttons/save and publishes "SAVE_MAP" on
    /robot/events. Today the wiring is partial — db_node
    listens to /robot/events directly and there's no node
    that converts button presses to events automatically.]
       │
       ▼
   recon_db/db_node.py:_on_robot_event(msg)
       │ if event == "SAVE_MAP":
       │   self._executor.submit(self._save_map)
       ▼
   db_node.py:_save_map() (runs on db_node's ThreadPoolExecutor)
       │ session = self._session_factory()
       │ record = MapRecord(name="map_YYYYMMDD_HHMMSS_<counter>",
       │                     map_data=json.dumps(self._latest_map["data"]).encode(),
       │                     ...)
       │ session.add(record); session.flush()
       │ event = MapEvent(event_type="SAVED", map_id=record.id, map_name=name)
       │ session.add(event); session.commit()
       ▼
   PostgreSQL
       ▼
   Web UI polls GET /api/maps/events?since=<last_id> every 3 s
       ▼
   recon_webui/app.py:api_map_events()
       │ session.query(MapEvent).filter(MapEvent.id > since_id).all()
       ▼
   New event detected → maps.html:loadMaps() refreshes the gallery
       │ map.html:pollMapEvents() shows toast "Map saved: <name>"

Both flows write to the same tables (``maps``, ``map_events``)
and produce identical schema. The web UI gallery refreshes
automatically regardless of which path the save came through.

.. _flow-process-map:

Flow 9 — Process map (Tier-2 pipeline)
======================================

Triggered by opening a saved-map modal on ``/maps`` and clicking
the **Processed** tab.

Sequence
--------

.. code-block:: text

   User clicks card on /maps → modal opens (maps.html:openModal)
   User clicks "Processed" tab → maps.html:switchTab("processed")
       │
       ▼
   maps.html:ensureProcessed()
       │ try: GET /api/maps/<id>/processed   ← check for existing run
       │ if existing.length: processed_id = existing[0].id
       │ else: POST /api/maps/<id>/process    ← run pipeline now
       ▼
   recon_webui/app.py:api_process_map(map_id)
       │ overrides = request.get_json(silent=True) or {}
       │ tpool.execute(_do)                   ← off the eventlet hub
       ▼
   _do() — opens DB session
       │ src = session.query(MapRecord).get(map_id)
       │ grid = json.loads(src.map_data)
       │ result = process_grid(grid, src.width, src.height, params=overrides)
       ▼
   recon_db/postprocess.py:process_grid()
       │
       │ Stage 1: median_3x3(arr)              (optional, off by default)
       │ Stage 2: morphological_opening(mask, iterations=0)
       │ Stage 3: morphological_closing(mask, iterations=1)
       │ Stage 4: label_connected_components(mask) — 8-connected BFS
       │          filter clusters with size < min_cluster_size
       │ Stage 5: hough_line_segments(final_occupied, ...)
       │
       │ returns ProcessResult(cleaned, cluster_labels,
       │                       n_clusters, n_noise_cells,
       │                       line_segments, parameters)
       ▼
   back in _do():
       │ row = ProcessedMap(source_map_id=src.id,
       │                     algorithm="tier2_dbscan_v1",
       │                     parameters=json.dumps(result.parameters),
       │                     map_data=result.to_json_bytes(),
       │                     n_clusters=..., n_noise_cells=...)
       │ session.add(row); session.commit()
       │ return {"id": row.id, "n_clusters": ..., ...}
       ▼
   app.py returns JSON
       ▼
   maps.html:ensureProcessed()
       │ fetch GET /api/processed/<processed_id>/data
       ▼
   app.py:api_get_processed_data()
       │ payload = json.loads(row.map_data)
       │ payload["resolution"] = src.resolution  ← inherit geometry
       │ payload["origin_x"]   = src.origin_x
       │ payload["origin_y"]   = src.origin_y
       │ return payload
       ▼
   maps.html:renderOccupancyToCanvas(d, mCanvas, d.line_segments)
       │ paints cleaned 3-colour grid
       │ overlays Hough segments as cyan polylines

Algorithm references
--------------------

For algorithm internals see :doc:`THESIS_TECHNICAL_DOCUMENTATION`
§7.5. The pipeline is **pure NumPy** — no scipy, scikit-image, or
OpenCV — to keep the AP-isolated Pi install lean.

.. _flow-emit-loop:

Flow 10 — WebSocket emit loop
=============================

The emit_loop is the recurring "push everything to the browser"
heartbeat of the web UI. Runs as an eventlet green thread.

Sequence
--------

.. code-block:: text

   recon_webui/app.py:main()
       │ socketio.start_background_task(emit_loop)
       ▼
   app.py:emit_loop()  ← runs forever on the eventlet hub
       │ while True:
       │   ┌─ for event in ros_bridge.drain_events():
       │   │      socketio.emit("robot_event", event)   ← drains queue.Queue
       │   ├─ if should_emit("robot_pose", 10 Hz):
       │   │      socketio.emit("robot_pose", channels["pose"].get())
       │   ├─ if should_emit("map_update", 5 Hz):
       │   │      socketio.emit("map_update", channels["map"].get())
       │   ├─ if should_emit("imu_data", 20 Hz):
       │   │      sample = channels["imu"].get()
       │   │      _imu_history.append(sample); trim to 120
       │   │      socketio.emit("imu_data", sample)
       │   ├─ if should_emit("bridge_health", 1 Hz):
       │   │      socketio.emit("bridge_health", channels["bridge_health"].get())
       │   ├─ if should_emit("stats_update", 2 Hz):
       │   │      socketio.emit("stats_update", _build_stats_snapshot())
       │   ├─ if should_emit("channel_status", 0.5 Hz):
       │   │      status = {name: {live, last_seen_s} for ...}
       │   │      socketio.emit("channel_status", status)
       │   └─ if should_emit("robot_mode", 1 Hz):
       │          socketio.emit("robot_mode", {"mode": ros_bridge.get_mode()})
       │   eventlet.sleep(0.01)  ← yield to hub for 10 ms
       ▼
   Each emit() → Flask-SocketIO serialises to JSON, broadcasts to all clients
       ▼
   Browser:
     common.js:socket.on("channel_status", ...) → updates health-bar pills
     common.js:socket.on("scan_state", ...)     → updates SCAN pill
     map.html:socket.on("robot_pose", ...)      → repaint pose overlay + trail buffer
     map.html:socket.on("map_update", ...)      → drawMap + drawTrail + drawRobot
     diagnostics.html:socket.on("imu_data", ...) → pushSample + drawSpark × 6
     diagnostics.html:socket.on("bridge_health", ...) → renderBridgeHealth

The ``should_emit`` helper rate-limits each event by tracking the
last emit time per event name. Rates come from
``config/webui.yaml: emit_rates``.

.. _flow-startup:

Flow 11 — System startup (boot → /map filling)
==============================================

End-to-end view of what happens between power-on and the
operator seeing a populated map.

Sequence
--------

.. code-block:: text

   Power applied to Pi 5 (mechanical SPST switch on battery rail)
       │
       ▼
   Ubuntu 24.04 boots → multi-user.target
       │
       ▼
   systemd: docker.service + recon-ap.service start
       │ Postgres container starts (healthcheck every 5 s, ready in ~10 s)
       │ recon-ap-start.sh creates ap0 + IP 10.0.0.1 + restarts hostapd/dnsmasq
       │
       ▼
   systemd: recon-stack.service triggers (After=docker, recon-ap, network-online)
       │ ExecStartPre: wait up to 30 s for /dev/ttyUSB0
       │ ExecStart: /bin/bash setup.sh full
       ▼
   roomba_ws/setup.sh full
       │ kill_stale_processes  (clean slate)
       │ check_python, check_flask, check_ros2, check_workspace_built,
       │   check_docker, check_lidar_serial, check_slam_toolbox,
       │   check_esp32_serial, check_pyserial
       │ ensure_db  (wait for pg_isready)
       │
       │ tmux: launch_esp32_bridge ───► esp32_uart_bridge starts, opens
       │   sleep 1                       /dev/ttyUSB0, creates pty,
       │   ┌── pre-flight: stty raw 460800 8N1 ──┐  symlinks /tmp/lidar_pty
       │
       │ launch_imu_yaw_integrator      ► subscribes /imu/data_raw
       │   sleep 1
       │ launch_imu_link_tf             ► static base_link → imu_link
       │   sleep 1
       │ launch_ekf                     ► robot_localization ekf_node
       │   sleep 1                       /odometry/filtered remapped to /odom
       │ launch_lidar_node              ► ldlidar_stl_ros2 opens /tmp/lidar_pty
       │   sleep 2                       blocks in WaitLidarCommConnect loop
       │ launch_slam_toolbox            ► async_slam_toolbox_node, paused
       │   sleep 2                       paused_new_measurements: true
       │ launch_db_node                 ► subscribes /robot/events, /robot/mode, /map
       │   sleep 1
       │ launch_webui_ros               ► recon_webui Flask + SocketIO
       │
       ▼
   recon_webui/app.py:main()
       │ eventlet.monkey_patch() (already run)
       │ setup_logging()
       │ setup_channels(config)   ← 4 DataChannels: map, pose, imu, bridge_health
       │ db_factory = get_session_factory()  ← creates tables if missing
       │ ros_bridge = RosBridge(channels); ros_bridge.start()  ← OS thread
       │ eventlet.spawn_after(5.0, ros_bridge.auto_pause)
       │ socketio.start_background_task(emit_loop)
       │ socketio.run(app, host="0.0.0.0", port=80)
       │
       ▼
   T+5 s: ros_bridge.auto_pause() fires on eventlet hub
       │ _call_slam_pause(True)         ← belt-and-braces (SLAM already paused at boot)
       │ _call_lidar_enable(False)      ← LIDAR motor already off
       │ logs "Auto-paused at startup — press Start scan to begin"
       │
       ▼
   User opens browser → http://recon.local/  → loads map.html
       │ JS connects WebSocket → on("connect") → emits initial state
       │ User clicks "Start scan" (or presses SPACE)
       │
       ▼  (see Flow 4)
   LIDAR motor enabled → /scan starts flowing → slam_toolbox unpaused
       │
       ▼
   ~1 s later: first valid /scan → ldlidar driver leaves wait loop
       │ ToLaserscanMessagePublish builds 720-bin LaserScan
       │ slam_toolbox runs first scan match
       │
       ▼
   1.0 s later: slam_toolbox publishes first /map
       │ webui_bridge:_map_callback → channels["map"].on_ros_message
       │ emit_loop next iteration → socketio.emit("map_update", grid)
       │
       ▼
   Browser: map.html:socket.on("map_update", ...) → first canvas paint
       │ map watermark "MOCK MAP" disappears as channel_status reports live
       │ pose channel goes live as /tf publishes map → odom
       │
       ▼
   User walks → trail polyline accumulates → walls fill in as scan matcher
   builds the pose graph

ROS topic and service inventory
===============================

A compact reference for every topic and service the running stack
exposes. Detailed types, QoS, and rates are in
:doc:`THESIS_TECHNICAL_DOCUMENTATION` Appendix A.

Topics
------

.. list-table::
   :widths: 30 20 25 25
   :header-rows: 1

   * - Topic
     - Type
     - Publisher
     - Subscriber(s)
   * - ``/scan``
     - LaserScan
     - ``ldlidar_stl_ros2_node``
     - ``slam_toolbox``
   * - ``/map``
     - OccupancyGrid
     - ``slam_toolbox``
     - ``recon_webui_bridge``, ``db_node``
   * - ``/tf``
     - TFMessage
     - ``slam_toolbox``, ``ekf_node``, static publishers ×2
     - ``recon_webui_bridge``, ``slam_toolbox``
   * - ``/imu/data_raw``
     - Imu (BEST_EFFORT)
     - ``esp32_uart_bridge``
     - ``imu_yaw_integrator``, ``recon_webui_bridge``
   * - ``/imu/data``
     - Imu (RELIABLE)
     - ``imu_yaw_integrator``
     - ``ekf_node``, ``slam_toolbox``
   * - ``/odom``
     - Odometry
     - ``ekf_node`` (remapped from ``/odometry/filtered``)
     - ``recon_webui_bridge``
   * - ``/buttons/save``
     - Empty
     - ``esp32_uart_bridge``
     - ``recon_webui_bridge``
   * - ``/buttons/reset``
     - Empty
     - ``esp32_uart_bridge``
     - ``recon_webui_bridge``
   * - ``/buttons/shutdown_request``
     - Empty
     - ``esp32_uart_bridge``
     - ``recon_webui_bridge``
   * - ``/buttons/shutdown_longpress``
     - Empty
     - ``esp32_uart_bridge``
     - ``recon_webui_bridge``
   * - ``/esp32/diagnostics``
     - String (JSON)
     - ``esp32_uart_bridge``
     - ``recon_webui_bridge``
   * - ``/robot/mode``
     - String
     - ``recon_webui_bridge``
     - ``recon_webui_bridge``, ``db_node``
   * - ``/robot/events``
     - String
     - ``db_node``, ``draw_node``
     - ``db_node``, ``recon_webui_bridge``
   * - ``/draw/command``
     - String
     - (none — H5)
     - ``draw_node``

Services
--------

.. list-table::
   :widths: 35 25 25 15
   :header-rows: 1

   * - Service
     - Type
     - Server
     - Client
   * - ``/lidar_enable``
     - std_srvs/SetBool
     - ``esp32_uart_bridge``
     - ``recon_webui_bridge``
   * - ``/slam_toolbox/set_parameters``
     - rcl_interfaces/SetParameters
     - ``slam_toolbox``
     - ``recon_webui_bridge``
   * - ``/slam_toolbox/reset``
     - slam_toolbox/srv/Reset
     - ``slam_toolbox``
     - ``recon_webui_bridge``

TF transforms
-------------

.. list-table::
   :widths: 15 15 40 30
   :header-rows: 1

   * - Parent
     - Child
     - Publisher
     - Notes
   * - ``map``
     - ``odom``
     - ``slam_toolbox``
     - Dynamic, scan-matched (~50 Hz)
   * - ``odom``
     - ``base_link``
     - ``ekf_node``
     - Dynamic, gyro-yaw (30 Hz)
   * - ``base_link``
     - ``imu_link``
     - static_transform_publisher
     - Identity (H6 replaces with real offset)
   * - ``base_link``
     - ``laser_frame``
     - static_transform_publisher
     - (0, 0, 0.10) from ``ld14p.launch.py``

REST endpoint map
=================

For full request/response schemas see
:doc:`THESIS_TECHNICAL_DOCUMENTATION` §7.6 and :doc:`SPEC` §5.2.

.. list-table::
   :widths: 35 10 55
   :header-rows: 1

   * - Route
     - Method
     - Code path
   * - ``/api/robot/status``
     - GET
     - ``app.py:api_robot_status`` → ``ros_bridge.get_mode()``, ``ros_bridge.is_scanning()``
   * - ``/api/scan/state``
     - GET
     - ``app.py:api_scan_state`` → ``ros_bridge.is_scanning()``
   * - ``/api/scan/start``
     - POST
     - ``app.py:api_scan_start`` → ``ros_bridge.set_scanning(True)``  (see :ref:`flow-start-scan`)
   * - ``/api/scan/pause``
     - POST
     - ``app.py:api_scan_pause`` → ``ros_bridge.set_scanning(False)`` (see :ref:`flow-pause-scan`)
   * - ``/api/map/clear``
     - POST
     - ``app.py:api_map_clear`` → ``ros_bridge.clear_map()`` (see :ref:`flow-clear-map`)
   * - ``/api/maps``
     - GET
     - ``app.py:api_list_maps`` → tpool → ``MapRecord`` query
   * - ``/api/maps``
     - POST
     - ``app.py:api_save_map`` → tpool → INSERT maps + map_events (see :ref:`flow-save-rest`)
   * - ``/api/maps/<id>``
     - GET
     - ``app.py:api_get_map`` → tpool → ``MapRecord.get(id)``
   * - ``/api/maps/<id>``
     - PUT
     - ``app.py:api_rename_map`` → tpool → ``UPDATE maps SET name``
   * - ``/api/maps/<id>``
     - DELETE
     - ``app.py:api_delete_map`` → tpool → DELETE + ``MapEvent(DELETED)``
   * - ``/api/maps/<id>/data``
     - GET
     - ``app.py:api_get_map_data`` → tpool → returns full grid
   * - ``/api/maps/<id>/process``
     - POST
     - ``app.py:api_process_map`` → tpool → ``process_grid()`` + INSERT processed_maps (see :ref:`flow-process-map`)
   * - ``/api/maps/<id>/processed``
     - GET
     - ``app.py:api_list_processed`` → tpool → ``ProcessedMap`` query
   * - ``/api/processed/<id>/data``
     - GET
     - ``app.py:api_get_processed_data`` → tpool → returns cleaned + segments
   * - ``/api/maps/events``
     - GET
     - ``app.py:api_map_events`` → tpool → ``MapEvent.id > since``
   * - ``/api/stats``
     - GET
     - ``app.py:api_stats`` → ``_build_stats_snapshot()``
   * - ``/api/debug/channels``
     - GET
     - ``app.py:api_debug_channels`` → per-channel liveness
   * - ``/api/robot/mode``
     - POST
     - ``app.py:api_set_mode`` → ``ros_bridge.publish_mode(mode)``

WebSocket event map
===================

Server → client
---------------

.. list-table::
   :widths: 20 15 65
   :header-rows: 1

   * - Event
     - Rate
     - Source / handler
   * - ``robot_pose``
     - 10 Hz
     - ``channels["pose"].get()``; consumed by ``map.html:socket.on("robot_pose")``
   * - ``map_update``
     - 5 Hz
     - ``channels["map"].get()``; consumed by ``map.html:socket.on("map_update")``
   * - ``imu_data``
     - 20 Hz
     - ``channels["imu"].get()``; consumed by ``diagnostics.html`` sparklines
   * - ``bridge_health``
     - 1 Hz
     - ``channels["bridge_health"].get()``; consumed by ``common.js`` (LIDAR pill) + ``diagnostics.html``
   * - ``stats_update``
     - 2 Hz
     - ``_build_stats_snapshot()``; ``diagnostics.html``
   * - ``channel_status``
     - 0.5 Hz
     - ``{name: {live, last_seen_s}}``; ``common.js`` health bar
   * - ``robot_mode``
     - 1 Hz
     - ``ros_bridge.get_mode()``; ``common.js``
   * - ``robot_event``
     - event
     - drained from ``ros_bridge.event_queue``; ``map.html`` event tail
   * - ``scan_state``
     - event
     - emitted by ``api_scan_start/pause``; ``common.js`` SCAN pill

Client → server
---------------

.. list-table::
   :widths: 20 80
   :header-rows: 1

   * - Event
     - Handler
   * - ``connect``
     - ``app.py:on_connect`` — emits initial ``robot_mode``, ``scan_state``, ``map_update``
   * - ``set_mode``
     - ``app.py:on_set_mode`` — calls ``ros_bridge.publish_mode(mode)``

Cross-cutting code-path glossary
================================

Where to look when you need to modify or debug a class of behaviour:

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Concern
     - Files
   * - **ESP32 firmware**
     - ``firmware/esp32/src/{main,config.h,framing,imu,buttons}.{h,cpp}``
   * - **Wire-format spec**
     - :doc:`UART_PROTOCOL`, ``firmware/esp32/src/framing.*``, ``recon_hardware/framing.py``
   * - **Pi-side bridge / IMU + buttons + LIDAR relay**
     - ``recon_hardware/esp32_uart_bridge.py``
   * - **Yaw integration**
     - ``recon_hardware/imu_yaw_integrator.py``, ``config/ekf.yaml``
   * - **SLAM configuration**
     - ``config/slam_params.yaml``
   * - **LIDAR driver patches**
     - ``roomba_ws/src/ldlidar_stl_ros2/launch/ld14p.launch.py``, ``roomba_ws/src/ldlidar_stl_ros2/src/demo.cpp``
   * - **Database schema**
     - ``recon_db/models.py``
   * - **Map save / event pipeline**
     - ``recon_db/db_node.py`` (button-driven), ``recon_webui/app.py:api_save_map`` (UI-driven)
   * - **Tier-2 post-processing**
     - ``recon_db/postprocess.py``
   * - **Flask routes**
     - ``recon_webui/app.py``
   * - **ROS bridge / pause / clear**
     - ``recon_webui/ros_bridge.py``
   * - **Mock fallback**
     - ``recon_webui/data_channels.py``, ``recon_webui/mock_data.py``
   * - **Templates / canvas rendering**
     - ``recon_webui/templates/{map,maps,diagnostics,base}.html``
   * - **Shared client JS**
     - ``recon_webui/static/js/common.js``
   * - **Startup orchestration**
     - ``roomba_ws/setup.sh``
   * - **Provisioning + verification**
     - ``roomba_ws/environment.sh``
   * - **Boot auto-start**
     - ``roomba_ws/systemd/recon-stack.service``
   * - **Postgres container**
     - ``roomba_ws/docker/docker-compose.yaml``

See also
========

* :doc:`AGENT_RULES` — Binding rules for contributors (threading,
  logging, language assignment).
* :doc:`SPEC` — Layer-by-layer contract (hardware, software,
  topics, REST, WebSocket).
* :doc:`ARCHITECTURE` — Diagrammatic data-flow narrative and the
  list of binding invariants.
* :doc:`STATUS` — Per-package status, smoke-test results, known
  limitations.
* :doc:`UART_PROTOCOL` — Canonical wire-format reference.
* :doc:`REMAINING_ISSUES` — Open issues with severity and fix
  recommendations.
* :doc:`THESIS_TECHNICAL_DOCUMENTATION` — Exhaustive prose
  treatment of every chapter referenced above.

— End of document —
