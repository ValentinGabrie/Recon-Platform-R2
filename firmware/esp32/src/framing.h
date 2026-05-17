// =============================================================================
// framing.h — Wire format for ESP32 → Pi UART link
//
// Frame layout:
//
//   0       1       2       3       4..4+LEN-1     4+LEN
//   ┌───────┬───────┬───────┬───────┬─────────────┬───────┐
//   │ 0xA5  │ 0x5A  │ TYPE  │ LEN   │ PAYLOAD ... │ CRC8  │
//   └───────┴───────┴───────┴───────┴─────────────┴───────┘
//                              \_______ CRC covers TYPE..end of PAYLOAD ______/
//
// CRC8 polynomial = 0x07 (Dallas/Maxim), init 0x00 — same as the Linux
// `crc8` kernel helper, so the Pi-side bridge can use a stock implementation.
//
// All multi-byte payload fields are little-endian (matches ESP32 + Pi).
// =============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

namespace framing {

// Maximum payload size in bytes (24 = IMU frame, the biggest we send).
constexpr size_t MAX_PAYLOAD = 24;

/// Compute Dallas/Maxim CRC-8 (poly 0x07, init 0x00) over `len` bytes.
uint8_t crc8(const uint8_t* data, size_t len);

/// Send one framed message on `Serial`.
/// `payload` may be nullptr iff `len == 0`.
void send_frame(uint8_t type, const uint8_t* payload, uint8_t len);

// ---- Convenience helpers ----------------------------------------------------

/// IMU sample. ax/ay/az in m/s², gx/gy/gz in rad/s.
void send_imu(float ax, float ay, float az,
              float gx, float gy, float gz);

void send_button(uint8_t id, uint8_t state);

void send_heartbeat(uint32_t uptime_ms);

void send_status(uint8_t flags);

/// Extended boot-diagnostic STATUS frame (payload = 8 bytes):
///   [flags, who_am_i, accel_cfg, gyro_cfg,
///    za_off_before_lo, za_off_before_hi, za_off_after_lo, za_off_after_hi]
/// Sent once at boot so the Pi-side decoder can confirm what the
/// MPU-6050 is reporting about itself (chip identity, range bits,
/// offset register bias).
void send_status_diag(uint8_t flags, uint8_t who_am_i,
                      uint8_t accel_cfg, uint8_t gyro_cfg,
                      int16_t za_offset_before, int16_t za_offset_after);

}  // namespace framing
