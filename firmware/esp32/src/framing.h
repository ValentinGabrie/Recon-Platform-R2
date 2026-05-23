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

// Maximum payload size in bytes. 64 lets LIDAR_FRAME chunks carry a couple
// of LD14P scan packets (47 B each) per frame without bloating the framing
// overhead. IMU/STATUS frames are unchanged.
constexpr size_t MAX_PAYLOAD = 64;

/// Compute Dallas/Maxim CRC-8 (poly 0x07, init 0x00) over `len` bytes.
uint8_t crc8(const uint8_t* data, size_t len);

/// Send one framed message on `Serial`.
/// `payload` may be nullptr iff `len == 0`.
void send_frame(uint8_t type, const uint8_t* payload, uint8_t len);

// ---- Convenience helpers (ESP32 → Pi) --------------------------------------

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

/// Forward up to `len` raw LD14P bytes to the Pi as a single LIDAR_FRAME.
/// `len` must be ≤ MAX_PAYLOAD. Caller is responsible for chunking longer
/// streams.
void send_lidar_frame(const uint8_t* buf, uint8_t len);

/// Acknowledge a LIDAR_EN state change so the Pi-side bridge can show the
/// current motor state in its diagnostics card.
void send_lidar_ack(uint8_t enabled);

// ---- Incoming-frame parser (Pi → ESP32) ------------------------------------

/// Result of FrameParser::feed when a frame completes.
struct ParsedFrame {
    uint8_t type;
    uint8_t len;
    uint8_t payload[MAX_PAYLOAD];
};

/// Stateful byte-feeder. Mirrors the Python parser at
/// roomba_ws/src/recon_hardware/recon_hardware/framing.py.
///
/// Usage:
///   FrameParser p;
///   ParsedFrame out;
///   if (p.feed(byte, out)) { ... handle out ... }
class FrameParser {
public:
    FrameParser();
    /// Feed one byte. Returns true and fills `out` when a frame completes.
    /// Silently drops CRC-failed and over-length frames; caller can monitor
    /// crc_fail_count() / bad_len_count() if they care.
    bool     feed(uint8_t b, ParsedFrame& out);
    uint32_t crc_fail_count() const { return crc_fail_count_; }
    uint32_t bad_len_count()  const { return bad_len_count_; }
private:
    enum State : uint8_t { HUNT0, HUNT1, TYPE, LEN, PAY, CRC };
    State    state_;
    uint8_t  ftype_;
    uint8_t  flen_;
    uint8_t  fidx_;
    uint8_t  buf_[MAX_PAYLOAD];
    uint32_t crc_fail_count_;
    uint32_t bad_len_count_;
};

}  // namespace framing
