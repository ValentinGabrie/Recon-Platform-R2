# ESP32 Sensor Coprocessor — Firmware Specification

## Overview

The ESP32 acts as a dedicated sensor coprocessor for the Roomba robot. It owns the
**LD-D200 (LD14P) 360° LIDAR** (via UART) and a single **HC-SR04 ultrasonic sensor**
(front, via GPIO), and exposes their readings to the Raspberry Pi 5 over I2C.

- **Role:** I2C slave device
- **I2C Address:** `0x42` (configurable in firmware)
- **Bus Speed:** 100 kHz standard mode (consider 400 kHz fast mode for LIDAR throughput)

## Wiring

### I2C Connection (ESP32 ↔ Pi 5)

| Signal | Pi 5 GPIO | ESP32 Pin | Notes |
|--------|-----------|-----------|-------|
| SDA | GPIO 2 | GPIO 21 | 4.7 kΩ pull-up to 3.3V |
| SCL | GPIO 3 | GPIO 22 | 4.7 kΩ pull-up to 3.3V |
| GND | GND | GND | Common ground required |

> **Important:** Use external 4.7 kΩ pull-up resistors on both SDA and SCL.
> Do not rely on internal MCU pull-ups.

### LD-D200 LIDAR Connection (ESP32 side)

| Signal | ESP32 Pin | LIDAR Pin | Notes |
|--------|-----------|-----------|-------|
| TX (LIDAR → ESP32) | GPIO 16 (UART2 RX) | TX | 3.3V logic, no level shifter needed |
| RX (ESP32 → LIDAR) | GPIO 17 (UART2 TX) | RX | Optional — only needed for motor speed control |
| PWM | GPIO 4 | PWM | Motor speed control (if supported by model) |
| GND | GND | GND | Common ground |
| VCC | — | 5V | Power from 5V rail, not ESP32 3.3V |

> **UART Configuration:** 230400 baud, 8N1. The LD14P protocol sends scan packets
> containing distance + confidence per beam. The ESP32 parses these packets and buffers
> the latest complete 360° scan.

### HC-SR04 Sensor Connection (ESP32 side)

| Sensor | Trigger Pin | Echo Pin |
|--------|-------------|----------|
| Front | GPIO 25 | GPIO 26 |

> **Note:** HC-SR04 operates at 5V logic. Use a voltage divider (5V → 3.3V) on the
> Echo pin to protect the ESP32 GPIO inputs.

## I2C Register Map

### Ultrasonic Registers

| Register (1 byte) | R/W | Data (2 bytes, big-endian uint16) | Description |
|--------------------|-----|-----------------------------------|-------------|
| `0x01` | R | Distance in mm (0–4000) | Front sensor |
| `0x04` | R | Status bitmask (see below) | Sensor health flags |
| `0xFF` | R | Firmware version (major.minor packed) | Checked on driver init |

### LIDAR Registers

| Register (1 byte) | R/W | Data | Description |
|--------------------|-----|------|-------------|
| `0x10` | R | 2 bytes — uint16 beam count (N) | Number of beams in current scan |
| `0x11` | R | 2 bytes — uint16 scan rate (Hz × 10) | Current motor spin rate |
| `0x12` | R | N × 2 bytes — uint16 distances (mm) | Full scan distance array, angle 0° CW |
| `0x13` | R | N × 1 byte — uint8 confidence (0–255) | Per-beam signal confidence |

### Status Bitmask (Register `0x04`)

| Bit | Meaning |
|-----|---------|
| 0 | Front ultrasonic sensor OK |
| 1 | LIDAR motor spinning |
| 2 | LIDAR data valid (at least one complete scan received) |
| 3–7 | Reserved (must be 0) |

### Firmware Version (Register `0xFF`)

The 2-byte value encodes `major.minor`:
- Byte 0 (MSB): Major version
- Byte 1 (LSB): Minor version

Example: `0x0102` = version 1.2

## I2C Read Protocol

The Pi 5 (master) reads sensor data using the following sequence:

### Ultrasonic (registers `0x01`, `0x04`, `0xFF`)
1. **Write** 1 byte: register address (e.g., `0x01` for front sensor)
2. **Repeated START**
3. **Read** 2 bytes: big-endian uint16 value in millimetres

### LIDAR Scan (registers `0x10`–`0x13`)
1. **Read `0x10`** to get beam count N
2. **Read `0x12`** — bulk transfer of N × 2 bytes (distance array)
3. **Optionally read `0x13`** — bulk transfer of N × 1 bytes (confidence array)

> **Bandwidth note:** At N=360 beams, register `0x12` returns 720 bytes. At 100 kHz
> I2C, this takes ~58 ms per scan read (tight for 10 Hz). Consider upgrading to 400 kHz
> fast mode, which reduces this to ~15 ms.

### Error Handling

- If the ultrasonic sensor fails to produce an echo within the timeout period, the ESP32
  must return `0xFFFF` for that sensor's register.
- The Pi-side driver interprets `0xFFFF` as `float('inf')` (no obstacle detected /
  sensor error).
- If the LIDAR has not completed a valid scan, register `0x10` returns 0 (zero beams).

## Firmware Implementation Notes

### Ultrasonic
- Use `pulseIn()` with a timeout of 30 ms for HC-SR04 echo measurement
- Distance calculation: `distance_mm = (pulse_duration_us * 0.343) / 2`
- Update reading in main loop at ≥20 Hz (twice the Pi poll rate)

### LIDAR
- Use `HardwareSerial` (UART2) at 230400 baud to receive LD14P scan packets
- Parse the LD14P protocol: each packet contains start byte, speed, start angle,
  12 data points (distance + confidence), end angle, timestamp, CRC
- Accumulate packets to build a complete 360° scan (typically 36 packets × 12 points = 432 beams)
- Double-buffer the scan array: write to back buffer, swap to front buffer atomically
  when a complete rotation is detected
- Set `LIDAR_DATA_VALID` bit in status register only after first complete scan

### I2C Slave
- Use the Arduino Wire library in slave mode: `Wire.begin(0x42)`
- Implement `Wire.onReceive()` to capture the register address
- Implement `Wire.onRequest()` to respond with data for the last requested register
- For bulk LIDAR reads (`0x12`, `0x13`), send data in chunks within the I2C buffer limit
  (typically 128 bytes on ESP32 Wire library — may need multiple transactions or a
  custom I2C slave implementation for large transfers)

## Testing Without Hardware

For development without a physical ESP32, the `esp32_sensor_node` C++ unit tests
mock the I2C file descriptor using dependency injection. See
`tests/test_esp32_sensor_node.cpp` for details.

To simulate the ESP32 on a second Pi or Linux machine, you can use the `i2c-stub`
kernel module:

```bash
sudo modprobe i2c-stub chip_addr=0x42
```

This creates a virtual I2C device that can be used for integration testing.
