// =============================================================================
// imu.h — Minimal MPU-6050 driver over Wire.h (I2C). No external libraries.
//
// Only what we need for handheld SLAM IMU data:
//   - Wake from sleep
//   - Set accelerometer full-scale to ±4 g  (sensitivity 8192 LSB/g)
//   - Set gyro full-scale       to ±500 °/s (sensitivity  65.5 LSB/°/s)
//   - Set DLPF to 44 Hz / 42 Hz (CONFIG=0x03)
//   - Read 14 bytes from 0x3B (accel + temp + gyro), convert to SI units
//
// On a bench MPU-6050 this gives ~3 mg / 0.015 °/s noise floor — plenty
// for Madgwick fusion on the Pi side.
// =============================================================================
#pragma once

#include <Arduino.h>
#include <stdint.h>

struct ImuSample {
    float ax;  // m/s²
    float ay;
    float az;
    float gx;  // rad/s
    float gy;
    float gz;
};

namespace imu {

/// Initialise I2C and configure the MPU-6050.
/// Returns true on success (WHO_AM_I = 0x68).
bool begin();

/// True iff the last begin()/read() saw a working MPU-6050.
bool ok();

/// Read one sample. Returns false if the I2C transaction fails or the
/// device hasn't been begin()'d successfully — `out` is left untouched.
bool read(ImuSample& out);

}  // namespace imu
