# ESP32 Sensor Coprocessor — Firmware Specification

## Overview

The ESP32 acts as a dedicated sensor coprocessor for the Roomba robot. It owns all
HC-SR04 ultrasonic sensors and exposes their readings to the Raspberry Pi 5 over I2C.

- **Role:** I2C slave device
- **I2C Address:** `0x42` (configurable in firmware)
- **Bus Speed:** 100 kHz standard mode

## Wiring

### I2C Connection (ESP32 ↔ Pi 5)

| Signal | Pi 5 GPIO | ESP32 Pin | Notes |
|--------|-----------|-----------|-------|
| SDA | GPIO 2 | GPIO 21 | 4.7 kΩ pull-up to 3.3V |
| SCL | GPIO 3 | GPIO 22 | 4.7 kΩ pull-up to 3.3V |
| GND | GND | GND | Common ground required |

> **Important:** Use external 4.7 kΩ pull-up resistors on both SDA and SCL.
> Do not rely on internal MCU pull-ups.

### HC-SR04 Sensor Connections (ESP32 side)

| Sensor | Trigger Pin | Echo Pin |
|--------|-------------|----------|
| Front | GPIO 25 | GPIO 26 |
| Left | GPIO 27 | GPIO 14 |
| Right | GPIO 32 | GPIO 33 |

> **Note:** HC-SR04 operates at 5V logic. Use a voltage divider (5V → 3.3V) on the
> Echo pin to protect the ESP32 GPIO inputs.

## I2C Register Map

| Register (1 byte) | R/W | Data (2 bytes, big-endian uint16) | Description |
|--------------------|-----|-----------------------------------|-------------|
| `0x01` | R | Distance in mm (0–4000) | Front sensor |
| `0x02` | R | Distance in mm (0–4000) | Left sensor |
| `0x03` | R | Distance in mm (0–4000) | Right sensor |
| `0x04` | R | Status bitmask (see below) | Sensor health flags |
| `0xFF` | R | Firmware version (major.minor packed) | Checked on driver init |

### Status Bitmask (Register `0x04`)

| Bit | Meaning |
|-----|---------|
| 0 | Front sensor OK |
| 1 | Left sensor OK |
| 2 | Right sensor OK |
| 3–7 | Reserved (must be 0) |

### Firmware Version (Register `0xFF`)

The 2-byte value encodes `major.minor`:
- Byte 0 (MSB): Major version
- Byte 1 (LSB): Minor version

Example: `0x0102` = version 1.2

## I2C Read Protocol

The Pi 5 (master) reads sensor data using the following sequence:

1. **Write** 1 byte: register address (e.g., `0x01` for front sensor)
2. **Repeated START**
3. **Read** 2 bytes: big-endian uint16 value in millimetres

### Error Handling

- If a sensor fails to produce an echo within the timeout period, the ESP32 must
  return `0xFFFF` for that sensor's register.
- The Pi-side driver interprets `0xFFFF` as `float('inf')` (no obstacle detected /
  sensor error).

## Firmware Implementation Notes

- Use the Arduino Wire library in slave mode: `Wire.begin(0x42)`
- Implement `Wire.onReceive()` to capture the register address
- Implement `Wire.onRequest()` to respond with the 2-byte value for the last
  requested register
- Sensor readings should be updated in the main loop at ≥20 Hz (twice the Pi poll rate)
- Use `pulseIn()` with a timeout of 30 ms for HC-SR04 echo measurement
- Distance calculation: `distance_mm = (pulse_duration_us * 0.343) / 2`

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
