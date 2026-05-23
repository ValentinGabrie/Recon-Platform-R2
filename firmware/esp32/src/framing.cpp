// =============================================================================
// framing.cpp — UART frame serialiser. See framing.h for the wire format.
// =============================================================================
#include "framing.h"

#include <string.h>

namespace framing {

uint8_t crc8(const uint8_t* data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}

void send_frame(uint8_t type, const uint8_t* payload, uint8_t len)
{
    if (len > MAX_PAYLOAD) {
        return;  // refuse to send anything that wouldn't fit our scope buffer
    }
    // Build the CRC scope (TYPE + LEN + PAYLOAD) in a stack buffer so we
    // hash it once instead of byte-feeding the CRC routine.
    uint8_t scope[2 + MAX_PAYLOAD];
    scope[0] = type;
    scope[1] = len;
    if (len > 0 && payload != nullptr) {
        memcpy(&scope[2], payload, len);
    }
    const uint8_t crc = crc8(scope, 2 + len);

    Serial.write(SYNC_BYTE_0);
    Serial.write(SYNC_BYTE_1);
    Serial.write(scope, 2 + len);
    Serial.write(crc);
}

void send_imu(float ax, float ay, float az,
              float gx, float gy, float gz)
{
    uint8_t payload[24];
    memcpy(&payload[0],  &ax, 4);
    memcpy(&payload[4],  &ay, 4);
    memcpy(&payload[8],  &az, 4);
    memcpy(&payload[12], &gx, 4);
    memcpy(&payload[16], &gy, 4);
    memcpy(&payload[20], &gz, 4);
    send_frame(FRAME_IMU, payload, 24);
}

void send_button(uint8_t id, uint8_t state)
{
    const uint8_t payload[2] = { id, state };
    send_frame(FRAME_BUTTON, payload, 2);
}

void send_heartbeat(uint32_t uptime_ms)
{
    uint8_t payload[4];
    memcpy(payload, &uptime_ms, 4);  // LE on ESP32
    send_frame(FRAME_HEARTBEAT, payload, 4);
}

void send_status(uint8_t flags)
{
    const uint8_t payload[2] = { flags, 0 };
    send_frame(FRAME_STATUS, payload, 2);
}

void send_status_diag(uint8_t flags, uint8_t who_am_i,
                      uint8_t accel_cfg, uint8_t gyro_cfg,
                      int16_t za_offset_before, int16_t za_offset_after)
{
    uint8_t payload[8];
    payload[0] = flags;
    payload[1] = who_am_i;
    payload[2] = accel_cfg;
    payload[3] = gyro_cfg;
    memcpy(&payload[4], &za_offset_before, 2);  // little-endian int16
    memcpy(&payload[6], &za_offset_after,  2);
    send_frame(FRAME_STATUS, payload, 8);
}

void send_lidar_frame(const uint8_t* buf, uint8_t len)
{
    send_frame(FRAME_LIDAR_FRAME, buf, len);
}

void send_lidar_ack(uint8_t enabled)
{
    send_frame(FRAME_LIDAR_ACK, &enabled, 1);
}

// ---- FrameParser (incoming Pi → ESP32) -------------------------------------

FrameParser::FrameParser()
    : state_(HUNT0), ftype_(0), flen_(0), fidx_(0),
      crc_fail_count_(0), bad_len_count_(0)
{
}

bool FrameParser::feed(uint8_t b, ParsedFrame& out)
{
    switch (state_) {
    case HUNT0:
        if (b == SYNC_BYTE_0) state_ = HUNT1;
        return false;
    case HUNT1:
        if      (b == SYNC_BYTE_1) state_ = TYPE;
        else if (b == SYNC_BYTE_0) /* stay armed on consecutive 0xA5 */;
        else                       state_ = HUNT0;
        return false;
    case TYPE:
        ftype_ = b;
        state_ = LEN;
        return false;
    case LEN:
        flen_ = b;
        if (b > MAX_PAYLOAD) {
            bad_len_count_++;
            state_ = HUNT0;
            return false;
        }
        fidx_  = 0;
        state_ = (b > 0) ? PAY : CRC;
        return false;
    case PAY:
        buf_[fidx_++] = b;
        if (fidx_ == flen_) state_ = CRC;
        return false;
    case CRC: {
        uint8_t scope[2 + MAX_PAYLOAD];
        scope[0] = ftype_;
        scope[1] = flen_;
        if (flen_ > 0) memcpy(&scope[2], buf_, flen_);
        const uint8_t expected = crc8(scope, 2 + flen_);
        state_ = HUNT0;
        if (b != expected) {
            crc_fail_count_++;
            return false;
        }
        out.type = ftype_;
        out.len  = flen_;
        if (flen_ > 0) memcpy(out.payload, buf_, flen_);
        return true;
    }
    }
    state_ = HUNT0;
    return false;
}

}  // namespace framing
