// =============================================================================
// main.cpp — Recon-Platform-R2 ESP32 I/O hub firmware (PlatformIO entry point)
//
// Job: read MPU-6050 IMU at 100 Hz, watch 3 buttons, ship everything to the
// Pi over USB-Serial using the binary framing in framing.h. The Pi-side
// bridge (lands in H3) parses these frames and republishes them as ROS2
// topics.
//
// Loop scheduling is cooperative — millis() deadlines drive each task,
// no FreeRTOS, no external libraries beyond Wire.h.
// =============================================================================
#include <Arduino.h>

#include "buttons.h"
#include "config.h"
#include "framing.h"
#include "imu.h"

namespace {

Button g_btn_shutdown;
Button g_btn_startstop;
Button g_btn_save;

uint32_t g_next_imu_ms       = 0;
uint32_t g_next_heartbeat_ms = 0;
uint32_t g_next_led_ms       = 0;
bool     g_led_state         = false;

// ---- LIDAR power state ------------------------------------------------------
bool                 g_lidar_enabled = false;
uint32_t             g_lidar_last_refresh_ms = 0;  // last time a LIDAR_EN=1 frame arrived
framing::FrameParser g_pi_parser;

void set_lidar_enabled(bool enable)
{
    if (g_lidar_enabled != enable) {
        digitalWrite(PIN_LIDAR_EN, enable ? HIGH : LOW);
        g_lidar_enabled = enable;
        framing::send_lidar_ack(enable ? 1 : 0);
    }
}

// Forwards a button event onto the UART. Matches the Button on_event signature.
void on_button_event(uint8_t id, uint8_t state)
{
    framing::send_button(id, state);
}

void handle_incoming_frame(const framing::ParsedFrame& f)
{
    if (f.type == FRAME_LIDAR_EN && f.len == 1) {
        const bool req = f.payload[0] != 0;
        if (req) {
            // Any LIDAR_EN=1 refreshes the watchdog deadline.
            g_lidar_last_refresh_ms = millis();
        }
        set_lidar_enabled(req);
    }
    // Other incoming opcodes can be added here. Unknown types are silently
    // ignored — the parser already filters CRC/length-bad frames.
}

void update_status_led()
{
    // Solid on  ⇒ IMU healthy.
    // Slow blink ⇒ IMU not responding (begin() failed or read() failed).
    if (imu::ok()) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        return;
    }
    const uint32_t now = millis();
    if ((int32_t)(now - g_next_led_ms) >= 0) {
        g_led_state = !g_led_state;
        digitalWrite(PIN_STATUS_LED, g_led_state ? HIGH : LOW);
        g_next_led_ms = now + 500;  // 1 Hz toggle ⇒ 0.5 Hz blink
    }
}

}  // namespace

void setup()
{
    // FAIL-SAFE FIRST: motor stays off through the boot window. Done before
    // anything else (Serial.begin, I2C init, etc.) so a panic / hang during
    // init can't leave the LIDAR spinning silently.
    pinMode(PIN_LIDAR_EN, OUTPUT);
    digitalWrite(PIN_LIDAR_EN, LOW);
    g_lidar_enabled = false;

    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    Serial.begin(UART_BAUD);
    // Give the USB-Serial bridge a moment to enumerate before we spam frames.
    delay(50);

    // LIDAR data ingest on Serial2 (GPIO16 RX, no TX). Bump RX buffer to
    // ~1 KB so a slow loop iteration can't drop a scan packet — at 23 KB/s
    // the default 256 B fills in 11 ms, which we'd occasionally exceed.
    Serial2.setRxBufferSize(1024);
    Serial2.begin(LIDAR_SERIAL2_BAUD, SERIAL_8N1, PIN_LIDAR_SERIAL2_RX, -1);

    g_btn_shutdown.begin (BTN_SHUTDOWN,  PIN_BTN_SHUTDOWN);
    g_btn_startstop.begin(BTN_STARTSTOP, PIN_BTN_STARTSTOP);
    g_btn_save.begin     (BTN_SAVE,      PIN_BTN_SAVE);

    const bool imu_ok = imu::begin();

    uint8_t flags = STATUS_BOOT;
    if (imu_ok) flags |= STATUS_IMU_OK;
    framing::send_status_diag(flags,
                              imu::who_am_i(),
                              imu::accel_cfg(),
                              imu::gyro_cfg(),
                              imu::za_offset_before(),
                              imu::za_offset_after());

    const uint32_t now = millis();
    g_next_imu_ms       = now + IMU_PERIOD_MS;
    g_next_heartbeat_ms = now + HEARTBEAT_PERIOD_MS;
    g_next_led_ms       = now;
}

void loop()
{
    const uint32_t now = millis();

    // ---- Incoming frames from Pi (LIDAR_EN, future opcodes) ----
    // Drain whatever's queued; bounded by available() so we never block.
    while (Serial.available() > 0) {
        framing::ParsedFrame f;
        const int b = Serial.read();
        if (b >= 0 && g_pi_parser.feed((uint8_t)b, f)) {
            handle_incoming_frame(f);
        }
    }

    // ---- LIDAR data relay: drain Serial2 RX into a LIDAR_FRAME envelope ----
    // Only forward when the motor is on; if it's off, the LIDAR has no power
    // and any stray bytes are noise (or echo from the last frame in the UART
    // shift register). One chunk per loop iteration is enough — at the ESP32's
    // typical >1 kHz loop rate this drains ~64 KB/s, well above the ~23 KB/s
    // LD14P stream.
    if (g_lidar_enabled) {
        const int avail = Serial2.available();
        if (avail > 0) {
            uint8_t buf[framing::MAX_PAYLOAD];
            const int to_read =
                (avail > (int)sizeof(buf)) ? (int)sizeof(buf) : avail;
            const int got = Serial2.readBytes(buf, to_read);
            if (got > 0) {
                framing::send_lidar_frame(buf, (uint8_t)got);
            }
        }
    }

    // ---- LIDAR watchdog: if the Pi stops refreshing while motor is on, kill it ----
    // Use signed comparison (matches the existing millis() idiom in this file):
    // a refresh frame that arrived earlier in this loop iteration may have set
    // g_lidar_last_refresh_ms to a value > `now`, which would underflow unsigned
    // subtraction and falsely trip the watchdog one millisecond after every ack.
    if (g_lidar_enabled &&
        (int32_t)(now - g_lidar_last_refresh_ms) > (int32_t)LIDAR_WATCHDOG_MS) {
        set_lidar_enabled(false);
    }

    // ---- Buttons (poll every loop; debounce inside Button::update) ----
    g_btn_shutdown.update (on_button_event);
    g_btn_startstop.update(on_button_event);
    g_btn_save.update     (on_button_event);

    // ---- IMU sample at IMU_RATE_HZ ----
    if ((int32_t)(now - g_next_imu_ms) >= 0) {
        g_next_imu_ms += IMU_PERIOD_MS;
        ImuSample s;
        if (imu::read(s)) {
            framing::send_imu(s.ax, s.ay, s.az, s.gx, s.gy, s.gz);
        } else {
            // Try to recover the bus on the next loop. ok()=false trips the
            // status LED; if the next imu::read() succeeds, we'll resume.
            (void)imu::begin();
            framing::send_status(imu::ok() ? STATUS_IMU_OK : 0);
        }
    }

    // ---- Heartbeat ----
    if ((int32_t)(now - g_next_heartbeat_ms) >= 0) {
        g_next_heartbeat_ms += HEARTBEAT_PERIOD_MS;
        framing::send_heartbeat(now);
    }

    // ---- Status LED ----
    update_status_led();
}
