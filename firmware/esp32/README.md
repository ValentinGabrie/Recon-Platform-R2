# Recon-Platform-R2 — ESP32 I/O Hub Firmware

Firmware for the ESP32-D coprocessor that reads the MPU-6050 IMU and three
front-panel buttons, then ships the data to the Pi 5 over USB-Serial using
the binary framing in [`src/framing.h`](src/framing.h).

The Pi-side bridge that consumes these frames lands in **Stage H2.1**.

Build system: **PlatformIO** with the Arduino-ESP32 framework
(`platform = espressif32`, `board = esp32dev`, `framework = arduino`).

---

## Hardware

| Function       | ESP32 pin   | Notes                                          |
| -------------- | ----------- | ---------------------------------------------- |
| MPU-6050 SDA   | `GPIO 21`   | I²C, 400 kHz                                   |
| MPU-6050 SCL   | `GPIO 22`   |                                                |
| MPU-6050 VCC   | `3V3`       | **Not 5 V** — module is 3.3 V tolerant only    |
| MPU-6050 GND   | `GND`       |                                                |
| MPU-6050 AD0   | `GND`       | I²C address `0x68`                             |
| Button SHUTDOWN| `GPIO 25`   | See "Buttons" below — wire one terminal to GPIO, one to GND. Internal pull-up enabled. |
| Button START/STOP | `GPIO 26` | " (formerly "RESET"; `PIN_BTN_STARTSTOP`)      |
| Button SAVE    | `GPIO 27`   | "                                              |
| Status LED     | `GPIO 2`    | Onboard blue LED on most ESP32 DevKit V1s      |
| UART to Pi     | USB         | Use the ESP32's micro-USB port → Pi USB        |

The ESP32 is powered from the Pi's USB port, so no separate supply is needed
on the bench. In the final enclosure the same holds: a 22.5 W USB-C power
bank feeds the Pi, and the ESP32 keeps drawing its power from the Pi's USB
port.

To pin all values, see [`src/config.h`](src/config.h).

### Buttons — 4-pin tactile switch wiring

The 4 terminals on a standard tactile switch are only **2 electrical nodes**.
Internally:

```
   1 ●━━━━━━━●  3        ← pins 1 and 3 are permanently shorted
            │ │
            │ ●─push      ← pressing closes (1,3) to (2,4)
            │ │
   2 ●━━━━━━━●  4        ← pins 2 and 4 are permanently shorted
```

Wire **one pin from each pair**. The foolproof choices are diagonal:

| Wire to GPIO | Wire to GND | Why                                 |
| ------------ | ----------- | ----------------------------------- |
| Pin **1**    | Pin **4**   | Diagonal — guaranteed opposite pairs |
| Pin **2**    | Pin **3**   | The other diagonal — also guaranteed |

Wiring two pins on the **same** side (1↔3 or 2↔4) shorts the GPIO straight
to GND — the firmware will see "always pressed" and never report an edge.

If unsure, use a multimeter in continuity mode:
- Probing pins **1 and 3** beeps unconditionally.
- Probing pins **2 and 4** beeps unconditionally.
- Probing any other pair is silent until you push the button, then beeps.
  Any pair from the second bullet is a valid (GPIO, GND) pair.

The other two terminals on each button stay unconnected; they exist only
to give the switch four mechanical anchors on a PCB.

No external pull-ups or pull-downs are needed — the firmware sets each
pin to `INPUT_PULLUP` so a press reads LOW.

---

## Wire format (UART)

```
0       1       2       3       4..4+LEN-1     4+LEN
┌───────┬───────┬───────┬───────┬─────────────┬───────┐
│ 0xA5  │ 0x5A  │ TYPE  │ LEN   │ PAYLOAD ... │ CRC8  │
└───────┴───────┴───────┴───────┴─────────────┴───────┘
                          \_______ CRC covers TYPE..end of PAYLOAD ______/
```

- **Sync**: `0xA5 0x5A` — used by the Pi parser to re-sync after byte loss.
- **Type**: see `FrameType` in `src/config.h`.
- **Len**: payload length in bytes (0–24).
- **CRC8**: Dallas/Maxim, polynomial `0x07`, init `0x00`, **covers TYPE+LEN+PAYLOAD**.
- **Endianness**: little-endian throughout (ESP32 + Pi agree).

Frame types currently emitted by the firmware:

| Type | Name        | Payload                                     | Rate   |
| ---- | ----------- | ------------------------------------------- | ------ |
| 0x01 | IMU         | 6× `float32`: ax, ay, az, gx, gy, gz        | 100 Hz |
| 0x02 | BUTTON      | `uint8 id, uint8 state`                     | event  |
| 0x03 | HEARTBEAT   | `uint32 uptime_ms`                          | 1 Hz   |
| 0x04 | STATUS      | `uint8 flags, uint8 reserved`               | boot + on IMU error |

Units: linear acceleration in **m/s²**, angular rate in **rad/s**.
The MPU-6050 is configured at ±4 g / ±500 °/s with the on-chip DLPF at
~44 Hz; conversion happens on the ESP32 so the Pi gets SI floats directly.

---

## Status LED

| Pattern                | Meaning                                |
| ---------------------- | -------------------------------------- |
| Solid on               | IMU healthy, frames flowing            |
| Slow blink (~1 Hz)     | IMU not responding on I²C — check wiring |

---

## One-shot provisioner (recommended)

A sibling script does everything below in the right order, retries the
flaky auto-reset, and grades the result:

```bash
cd firmware/esp32
./environment.sh           # install PlatformIO + build + flash + verify
./environment.sh --check   # verify-only (no install, no flash) on a
                           # device that's already been provisioned
./environment.sh --no-flash  # install + build only
./environment.sh --help    # full option list
```

The script mirrors the style of `roomba_ws/environment.sh`: idempotent,
colour-coded, ends with a `PASS / FAIL / WARN` summary. It also
encodes two real bench-test gotchas: (a) auto-reset failures retry up
to 3 times, and (b) after a successful flash it forces a known-good
hard reset via `esptool` so the chip actually starts running the new
image instead of continuing the previous one.

If you'd rather drive the steps yourself, the manual workflow follows.

## Manual build & flash (PlatformIO)

### One-time install

PlatformIO Core (CLI) is the simplest install on the Pi:

```bash
sudo apt install python3-venv         # if not already present
python3 -m venv ~/.platformio-venv
source ~/.platformio-venv/bin/activate
pip install -U platformio
```

Then add `~/.platformio-venv/bin` to your `PATH` (e.g. in `~/.bashrc`) so
`pio` is available without re-sourcing the venv.

If you'd rather use VS Code: install the official **PlatformIO IDE**
extension; it bundles its own Python/PIO. Either way the project layout
and commands below are the same.

### Build / upload / monitor

From this directory (`firmware/esp32/`):

```bash
pio run                    # compile only
pio run -t upload          # compile + flash to /dev/ttyUSB0 (auto-detected)
pio device monitor         # open serial console at 115200 baud
pio run -t clean           # wipe build artefacts in .pio/
```

After flashing, the blue LED should go solid within ~100 ms (assuming the
IMU is wired and powered). At 115 200 baud you can sanity-check the byte
stream without Python:

```bash
xxd -c 30 < /dev/ttyUSB0 | head
```

You'll see groups of 30 bytes per IMU frame interleaved with shorter
heartbeat/status frames.

### If `pio` can't find the port

Edit [`platformio.ini`](platformio.ini) and uncomment:

```ini
upload_port = /dev/ttyUSB0
monitor_port = /dev/ttyUSB0
```

Or pass it on the CLI: `pio run -t upload --upload-port /dev/ttyUSB0`.

---

## Bench-test the firmware

> A short answer to "does it work, and does the serial show me what the
> buttons and IMU are doing?" — yes, but the stream is **binary frames**,
> not human-readable text. `pio device monitor` will show garbage bytes.
> Use the decoder script below to see decoded values.

The firmware sends the binary framing described in
[`docs/UART_PROTOCOL.md`](../../docs/UART_PROTOCOL.md). To make that
legible during bring-up we ship a tiny standalone Python decoder at
[`tools/decode_serial.py`](tools/decode_serial.py). It needs only
`pyserial` — no ROS, no recon_webui, nothing else from the workspace.

### Step 1 — flash and check the LED

```bash
cd firmware/esp32
pio run -t upload
```

The onboard blue LED behaviour is the fastest visual diagnostic:

| LED                | Meaning                                                        |
| ------------------ | -------------------------------------------------------------- |
| **Solid on**       | IMU healthy, frames flowing.                                   |
| **Slow blink ~1 Hz** | IMU not responding on I²C. Check VCC = 3V3, AD0 = GND, SDA/SCL wiring. |
| **Off**            | Sketch not running — try `pio device monitor` to see boot logs from the bootloader, or re-flash. |

### Step 2 — sanity-check the byte stream (no Python required)

```bash
xxd -c 30 < /dev/ttyUSB0 | head
```

You should see lines like:

```
00000000: a5 5a 04 02 03 00 5d                                  .Z....]
00000007: a5 5a 03 04 c1 09 00 00 78                            .Z......x
00000010: a5 5a 01 18 00 00 00 00 00 00 00 00 db 0f 1d 41 …    .Z.............A
```

The repeating `a5 5a 01 18 ...` patterns are IMU frames at 100 Hz.
`a5 5a 03 04 ...` is the 1 Hz heartbeat. `a5 5a 04 02 ...` is the boot
STATUS frame. If you see this, the firmware is alive — move on.

### Step 3 — decode the stream into human-readable output

```bash
pip install pyserial            # one-time, on whichever machine you'll watch from
python3 firmware/esp32/tools/decode_serial.py
```

Output looks like:

```
Listening on /dev/ttyUSB0 @ 115200 baud — Ctrl+C to quit

[14:02:11] STATUS    flags = 0x03 [BOOT, IMU_OK]
[14:02:11] HEARTBEAT uptime = 0.1 s
[14:02:11] IMU       a=( -0.02, +0.04, +9.78) m/s²  g=(+0.001, -0.002, +0.000) rad/s
[14:02:11] IMU       a=( -0.01, +0.05, +9.79) m/s²  g=(+0.000, -0.001, +0.001) rad/s
[14:02:12] HEARTBEAT uptime = 1.1 s
[14:02:14] BUTTON    SAVE PRESSED
[14:02:14] BUTTON    SAVE RELEASED
[14:02:18] BUTTON    SHUTDOWN PRESSED
[14:02:20] BUTTON    SHUTDOWN LONGPRESS
[14:02:21] BUTTON    SHUTDOWN RELEASED
```

The decoder rate-limits IMU prints to ~10 Hz (out of 100 Hz on the wire)
so the console stays readable. CRC failures are flagged in red — any
nonzero count after a minute of running points at flaky wiring or
ground.

### Step 4 — confirm the IMU actually responds to motion

With the decoder running:

1. **Hold the device flat, sensor side up.** One axis (typically `+az`)
   sits at the strongest value, the other two near zero. **All gyro
   values near zero** (a few hundredths of rad/s of bias is normal).
2. **Tilt the device 90° onto its side.** The dominant accel axis swaps:
   what was on `az` should now appear on `ax` or `ay` depending on tilt
   direction, with the same approximate magnitude.
3. **Rotate it briskly about one axis.** The matching gyro value jumps
   to ±0.5 rad/s or so during the motion and returns to ~0 when still.

Direction-of-gravity tracking is the only thing that matters at this
stage — the Madgwick fusion in H3 estimates and removes accel bias
online. **Do not be alarmed if total accel magnitude reads ~15 m/s²
instead of 9.81 when flat.** Many MPU-6050 modules ship with a
non-zero factory `ZA_OFFSET_USR` (we measured 1544 LSB on one chip
here, which biases the Z reading by ~5.5 m/s²). The boot-time STATUS
diagnostic frame reports this — see "Step 5" below.

> **Hard-won lesson:** *Do not* naively write 0 to the
> `XA/YA/ZA_OFFSET_USR` registers (0x06–0x0B). Bit 0 of each L byte is
> a reserved temp-comp bit; clobbering it makes the readings worse,
> not better. Leave the factory bias in place at this stage and let
> H3's EKF model it.

### Step 4b — read the boot STATUS diagnostic

The firmware emits one extended STATUS frame at boot containing the
chip's self-reported state. The decoder prints it as a single line:

```
[hh:mm:ss] STATUS    flags = 0x03 [BOOT, IMU_OK]  who_am_i=0x68 \
                     accel_cfg=0x08 (AFS_SEL=1)  gyro_cfg=0x08 (FS_SEL=1) \
                     za_offset before=1544 after=1544
```

Read each field:

| Field          | Healthy value                                                    |
| -------------- | ---------------------------------------------------------------- |
| `who_am_i`     | `0x68` for a genuine MPU-6050; `0x70`/`0x72` for accepted clones |
| `AFS_SEL`      | `1` → ±4 g range                                                 |
| `FS_SEL`       | `1` → ±500 °/s range                                              |
| `za_offset before/after` | Whatever the factory loaded — informational. Bias is reported but not corrected; H3 will. |

If `flags` lacks `IMU_OK`, the chip didn't answer on I²C — check the
SDA/SCL wiring and the 3V3 VCC line (NOT 5V).

You may have to wait up to 1 s after launching the decoder for the
first heartbeat to roll around and confirm everything's flowing; the
boot STATUS itself appears in the first ~100 ms after the chip resets,
so it may have already gone by — re-trigger by power-cycling the
ESP32 or running `python3 -m esptool --port /dev/ttyUSB0 --after hard_reset chip_id`
in another terminal first.

### Step 5 — confirm each button independently

With the decoder running, press each button in turn:

| You do                          | Decoder should print                                         |
| ------------------------------- | ------------------------------------------------------------ |
| Tap SAVE                        | `BUTTON SAVE PRESSED` → `BUTTON SAVE RELEASED`               |
| Tap START/STOP                  | `BUTTON STARTSTOP PRESSED` → `BUTTON STARTSTOP RELEASED`     |
| Tap SHUTDOWN                    | `BUTTON SHUTDOWN PRESSED` → `BUTTON SHUTDOWN RELEASED`       |
| Hold SHUTDOWN ≥ 2 s             | `... PRESSED` → `... LONGPRESS` → `... RELEASED`             |
| Hold any other button ≥ 2 s     | Same `LONGPRESS` event (STARTSTOP / SAVE long-press is detected too) |

Common failures:

- **Wrong pair of pins on the switch** (see "Buttons — 4-pin tactile
  switch wiring" above): the GPIO is shorted to GND. Decoder shows the
  button as `PRESSED` immediately on boot and never reports `RELEASED`.
  Fix the wiring.
- **No GND wire:** the GPIO floats, internal pull-up keeps it HIGH,
  nothing ever fires. Decoder shows no BUTTON events at all even when
  you mash the button.
- **Wrong GPIO:** decoder fires the *other* button's event, or no event
  at all. Re-check which switch is wired to which `Dxx` pin.

### Step 6 — sustained 60-second sanity run

Let the decoder run for a full minute without touching anything, then
Ctrl+C. The summary block at the end should show:

```
--- Frame counts ---
  IMU          ~6000           (100 Hz × 60 s)
  BUTTON       0
  HEARTBEAT    60              (1 Hz × 60 s)
  STATUS       1               (just the boot frame)
  CRC_FAIL     0               ← this must be zero
```

A nonzero `CRC_FAIL` after a clean run almost always means a poor
ground or a noisy USB cable; tighten the connections and re-run.

When all six steps pass, the firmware is bench-validated and ready for
the H2.1 Pi-side bridge to consume the same frames over ROS2 topics.

---

## File layout

```
firmware/esp32/
├── README.md                 ← this file
├── environment.sh            ← one-shot install + build + flash + verify
├── platformio.ini            ← board, framework, build flags
├── .gitignore                ← .pio/, .pioenvs/, IDE state
├── src/                      ← all C++ sources (PlatformIO compiles src/*.cpp)
│   ├── main.cpp              ← setup() + loop()
│   ├── config.h              ← pinout + protocol constants
│   ├── framing.h / .cpp      ← UART frame serialiser + CRC8
│   ├── imu.h     / .cpp      ← MPU-6050 driver (Wire.h only)
│   └── buttons.h / .cpp      ← debounced button handler
└── tools/
    └── decode_serial.py      ← bench-test decoder (pyserial only)
```

No external libraries required for the firmware itself — everything
compiles against the stock `espressif32 / arduino` framework's
bundled `Arduino.h` and `Wire.h`. The Python tool needs only
`pyserial`.
