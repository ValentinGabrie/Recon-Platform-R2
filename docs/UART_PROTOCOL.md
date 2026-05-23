# Recon-Platform-R2 — UART Protocol (ESP32 ⇄ Pi)

> Canonical reference for the binary wire format used between the ESP32
> I/O hub and the Pi-side bridge. The ESP32 firmware emits this format
> (see [`firmware/esp32/src/framing.cpp`](../firmware/esp32/src/framing.cpp));
> the Pi-side bridge node (H2.1) decodes it. **Either side breaking
> this contract breaks the other.** Changes here are coordinated commits
> on both sides.

---

## 1. Transport

- **Physical:** USB-Serial via the ESP32's on-board CP2102 / CH340 bridge.
- **Pi-side device:** `/dev/ttyUSB0` (CP2102) or `/dev/ttyACM0` (some
  CH340 / native USB variants).
- **Baud:** **460 800**, **8N1**, no flow control.
- **Direction:** **Bidirectional.** ESP32 → Pi for sensor data (IMU, button
  events, heartbeats, status, LIDAR frames); Pi → ESP32 for actuator
  commands (LIDAR motor enable). The framing is symmetric — both sides
  emit and parse the same `[A5][5A][TYPE][LEN][PAYLOAD][CRC8]` envelope.

Sustained throughput budget at 460 800 8N1: ~46 KB/s. Steady-state
traffic from the firmware with motor on:
  * IMU @ 100 Hz × 29 B = ~3 KB/s
  * Heartbeat @ 1 Hz = negligible
  * LIDAR_FRAME forwarding the LD14P's 230 400 baud raw stream = ~23 KB/s
  * **Total ~26 KB/s**, leaving comfortable headroom for the few-byte
    Pi→ESP32 LIDAR_EN refreshes that travel in the reverse direction.

> The baud was bumped from 115 200 to 460 800 in [`config.h`](../firmware/esp32/src/config.h#L48)
> on the same commit that introduced the LIDAR relay. 115 200 cannot carry
> the LD14P stream — at 11.5 KB/s it's below the LD14P's own 23 KB/s rate.

---

## 2. Frame layout

```
0       1       2       3       4 .. 4+LEN-1   4+LEN
┌───────┬───────┬───────┬───────┬─────────────┬───────┐
│ 0xA5  │ 0x5A  │ TYPE  │ LEN   │ PAYLOAD ... │ CRC8  │
└───────┴───────┴───────┴───────┴─────────────┴───────┘
                          \_______ CRC covers TYPE..end of PAYLOAD _______/
```

| Field    | Size | Description                                                     |
| -------- | ---- | --------------------------------------------------------------- |
| SYNC[0]  | 1 B  | Constant `0xA5`                                                 |
| SYNC[1]  | 1 B  | Constant `0x5A`                                                 |
| TYPE     | 1 B  | Frame type — see §3                                              |
| LEN      | 1 B  | Payload length in bytes (0 ≤ LEN ≤ **64**)                       |
| PAYLOAD  | LEN B| Type-specific (see §3)                                           |
| CRC8     | 1 B  | Dallas/Maxim CRC-8 over `[TYPE, LEN, PAYLOAD]`. **Sync bytes are NOT covered.** |

Total frame size = `5 + LEN` bytes.

**Endianness.** All multi-byte integer and float payload fields are
**little-endian**. Both ESP32 (Xtensa LX6) and Pi 5 (ARM64 in LE mode)
are little-endian, so `memcpy` works on both sides.

---

## 3. Frame types

The canonical enum lives in
[`firmware/esp32/src/config.h`](../firmware/esp32/src/config.h). This
table is the contract.

| TYPE | Name          | Dir   | LEN   | Rate                 | Payload format                                              |
| ---- | ------------- | ----- | ----- | -------------------- | ----------------------------------------------------------- |
| 0x01 | `IMU`         | ESP→Pi| 24    | 100 Hz               | 6 × `float32`: `ax, ay, az, gx, gy, gz`                     |
| 0x02 | `BUTTON`      | ESP→Pi|  2    | event-driven         | `uint8 id, uint8 state`                                     |
| 0x03 | `HEARTBEAT`   | ESP→Pi|  4    | 1 Hz                 | `uint32 uptime_ms`                                          |
| 0x04 | `STATUS`      | ESP→Pi|  2/8  | boot + on IMU error  | `uint8 flags, …` (see §3.4)                                  |
| 0x05 | `LIDAR_FRAME` | ESP→Pi| 1..64 | as bytes arrive      | Raw LD14P UART bytes — the bridge writes them to a pty so the existing ldlidar_stl_ros2 driver consumes them as if they came from a real serial port. |
| 0x06 | `LIDAR_EN`    | Pi→ESP|  1    | event + 1 Hz refresh | `uint8 enable` (0=motor off, 1=on). The Pi re-sends `1` once a second so the firmware's 3-second watchdog drops the motor if the Pi crashes. |
| 0x07 | `LIDAR_ACK`   | ESP→Pi|  1    | on state change      | `uint8 enabled` — current motor state after the firmware applied a LIDAR_EN request. Used by the Pi-side bridge to surface the real hardware state in its `/esp32/diagnostics` topic. |

### 3.1 IMU payload

```
offset  field  type     unit
  0..3  ax     float32  m/s²    (linear accel, sensor frame)
  4..7  ay     float32  m/s²
  8..11 az     float32  m/s²
 12..15 gx     float32  rad/s   (angular rate, sensor frame)
 16..19 gy     float32  rad/s
 20..23 gz     float32  rad/s
```

The MPU-6050 is configured at **±4 g** and **±500 °/s** with the
on-chip DLPF at ~44 Hz; conversion to SI units happens on the ESP32.
The Pi-side bridge republishes these as `sensor_msgs/Imu` on
`/imu/data_raw` with no further scaling (orientation field left
unfilled — that's Madgwick's job in H3).

### 3.2 BUTTON payload

```
offset  field  type    values
  0     id     uint8   0=SHUTDOWN, 1=RESET, 2=SAVE
  1     state  uint8   0=RELEASED, 1=PRESSED, 2=LONGPRESS
```

A normal press generates two frames: `PRESSED` on the falling edge
(after 25 ms debounce), `RELEASED` on the rising edge. A press held
for ≥ 2 000 ms generates an additional `LONGPRESS` frame between them.

### 3.3 HEARTBEAT payload

```
offset  field      type     unit
  0..3  uptime_ms  uint32   milliseconds since ESP32 boot (LE)
```

Used by the Pi-side bridge to detect a hung or absent ESP32. If no
HEARTBEAT (or any other frame) arrives within ~3 s, the bridge logs
WARN and stops publishing `/imu/data_raw`.

### 3.4 STATUS payload

```
offset  field     type    bits
  0     flags     uint8   bit 0: STATUS_BOOT      (first frame after boot)
                          bit 1: STATUS_IMU_OK    (0 ⇒ IMU not on I²C bus)
                          bit 2: STATUS_IMU_DATA  (1 ⇒ first valid sample seen)
                          bits 3..7: reserved (set to 0)
  1     reserved  uint8   always 0
```

Emitted once at boot, and again whenever `imu::read()` fails and the
firmware retries `imu::begin()`. The Pi-side bridge republishes
flag changes as `diagnostic_msgs/DiagnosticStatus`.

---

## 4. CRC8

- **Polynomial:** `0x07` (Dallas/Maxim / "1-Wire CRC")
- **Initial value:** `0x00`
- **Reflect input:** No
- **Reflect output:** No
- **XOR-out:** `0x00`

This is identical to the Linux kernel's `crc8` helper with `polynomial
= 0x07`, init `0x00`. Reference Python decoder:

```python
def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc
```

Reference C implementation lives in
[`firmware/esp32/src/framing.cpp`](../firmware/esp32/src/framing.cpp)
(`framing::crc8`).

The CRC byte is computed over **TYPE + LEN + PAYLOAD** (not the sync
bytes). Sync bytes are excluded so resync logic can scan for them
independently of any CRC machinery.

---

## 5. Resync algorithm (Pi-side parser)

Bytes can be dropped or corrupted; the parser must recover without
external prodding. Recommended algorithm (matches the planned H2.1
implementation):

```
state = HUNT_SYNC0
loop:
    b = serial.read(1)
    match state:
        case HUNT_SYNC0:
            if b == 0xA5: state = HUNT_SYNC1
        case HUNT_SYNC1:
            if b == 0x5A: state = READ_TYPE
            elif b == 0xA5: pass  # stay in HUNT_SYNC1 (handle 0xA5 0xA5 0x5A)
            else: state = HUNT_SYNC0
        case READ_TYPE:
            type = b; state = READ_LEN
        case READ_LEN:
            len = b
            if len > MAX_PAYLOAD: state = HUNT_SYNC0; continue
            buf = []; state = READ_PAYLOAD
        case READ_PAYLOAD:
            buf.append(b)
            if len(buf) == len: state = READ_CRC
        case READ_CRC:
            expected = crc8([type, len] + buf)
            if b == expected:
                handle_frame(type, buf)
            else:
                log.debug("CRC fail — dropping frame")
            state = HUNT_SYNC0
```

`MAX_PAYLOAD = 24` (current ceiling). The check `if len > MAX_PAYLOAD`
catches the case where a stray byte was misinterpreted as the LEN field.

---

## 6. Frame examples

### 6.1 IMU sample (device flat, +Z up, stationary)

Sensor reads roughly `ax ≈ 0, ay ≈ 0, az ≈ +g`, gyro ≈ 0 → bytes:

```
A5 5A 01 18
00 00 00 00         # ax = 0.0
00 00 00 00         # ay = 0.0
DB 0F 1D 41         # az ≈ 9.80665 (float32 LE)
00 00 00 00         # gx = 0.0
00 00 00 00         # gy = 0.0
00 00 00 00         # gz = 0.0
xx                  # CRC8([01, 18, 24 payload bytes])
```

Total: 30 bytes per IMU frame at 100 Hz = 3 000 B/s.

### 6.2 SAVE button pressed (id=2, state=PRESSED)

```
A5 5A 02 02
02 01
xx
```

Total: 7 bytes per button event.

### 6.3 Heartbeat at uptime 12 345 ms

```
A5 5A 03 04
39 30 00 00         # 0x00003039 = 12345 (LE)
xx
```

### 6.4 Boot STATUS (IMU healthy)

```
A5 5A 04 02
03 00               # flags = STATUS_BOOT | STATUS_IMU_OK
xx
```

---

## 7. Versioning

There is currently **no version field** in the frame format. The
firmware and Pi-side bridge are versioned together via git — a
`firmware/esp32/src/config.h` change that alters the frame layout is
expected to land in the same commit as the corresponding bridge
update.

If the format ever needs to change incompatibly, the recommended
migration is:

1. Add a `VERSION` byte after `LEN` (would be a breaking change).
2. Bump the second sync byte from `0x5A` to a new value to make old
   and new parsers mutually exclusive.

Until that day, the rule is: **bridge and firmware ship as a pair.**

---

## 8. Quick sanity check (no parser required)

After flashing the ESP32 and connecting it to the Pi, you can confirm
the frame stream is alive without writing any code:

```bash
xxd -c 30 < /dev/ttyUSB0 | head
```

You'll see groups of 30 bytes starting with `a5 5a 01 18 ...` (IMU
frames, every 10 ms), interleaved with shorter frames:

- `a5 5a 03 04 ...` (heartbeat, every 1 000 ms)
- `a5 5a 04 02 03 00 ..` once at boot
- `a5 5a 02 02 ..` whenever you press a button

If you only see the first pattern, the IMU is healthy. If `a5 5a 04 02`
frames appear with the second byte `00` (no STATUS_IMU_OK bit) and the
ESP32 LED is slow-blinking, the IMU is not on the I²C bus.

---

## 9. Implementation cross-references

| Concern                  | Where it lives                                                                  |
| ------------------------ | ------------------------------------------------------------------------------- |
| Sync bytes / type enum   | [`firmware/esp32/src/config.h`](../firmware/esp32/src/config.h)                  |
| Encoder + CRC8           | [`firmware/esp32/src/framing.cpp`](../firmware/esp32/src/framing.cpp)            |
| Pi-side decoder (planned)| H2.1: `roomba_ws/src/recon_hardware/recon_hardware/esp32_uart_bridge.py`         |
| Wire format walkthrough  | This file                                                                       |
| Frame field meanings     | [`SPEC.md`](SPEC.md) §3.3                                                       |
| Bridge → ROS2 topic mapping | [`SPEC.md`](SPEC.md) §4.2                                                    |
