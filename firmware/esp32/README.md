# Recon-Platform-R2 — ESP32 I/O Hub Firmware

Firmware for the ESP32-D coprocessor that reads the MPU-6050 IMU and three
front-panel buttons, then ships the data to the Pi 5 over USB-Serial using
the binary framing in [`src/framing.h`](src/framing.h).

The Pi-side bridge that consumes these frames lands in **Stage H3**.

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
| Button SHUTDOWN| `GPIO 25`   | Other side to `GND`. Internal pull-up enabled. |
| Button RESET   | `GPIO 26`   | "                                              |
| Button SAVE    | `GPIO 27`   | "                                              |
| Status LED     | `GPIO 2`    | Onboard blue LED on most ESP32 DevKit V1s      |
| UART to Pi     | USB         | Use the ESP32's micro-USB port → Pi USB        |

The ESP32 is powered from the Pi's USB port, so no separate supply is needed
on the bench. In the final enclosure the ESP32 will share the battery rail
with the Pi.

To pin all values, see [`src/config.h`](src/config.h).

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

## Build & flash (PlatformIO)

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

## File layout

```
firmware/esp32/
├── README.md                 ← this file
├── platformio.ini            ← board, framework, build flags
├── .gitignore                ← .pio/, .pioenvs/, IDE state
└── src/                      ← all C++ sources (PlatformIO compiles src/*.cpp)
    ├── main.cpp              ← setup() + loop()
    ├── config.h              ← pinout + protocol constants
    ├── framing.h / .cpp      ← UART frame serialiser + CRC8
    ├── imu.h     / .cpp      ← MPU-6050 driver (Wire.h only)
    └── buttons.h / .cpp      ← debounced button handler
```

No external libraries required — everything compiles against the stock
`espressif32 / arduino` framework's bundled `Arduino.h` and `Wire.h`.
