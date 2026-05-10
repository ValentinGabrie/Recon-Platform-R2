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

}  // namespace framing
