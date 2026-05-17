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
Button g_btn_reset;
Button g_btn_save;

uint32_t g_next_imu_ms       = 0;
uint32_t g_next_heartbeat_ms = 0;
uint32_t g_next_led_ms       = 0;
bool     g_led_state         = false;

// Forwards a button event onto the UART. Matches the Button on_event signature.
void on_button_event(uint8_t id, uint8_t state)
{
    framing::send_button(id, state);
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
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    Serial.begin(UART_BAUD);
    // Give the USB-Serial bridge a moment to enumerate before we spam frames.
    delay(50);

    g_btn_shutdown.begin(BTN_SHUTDOWN, PIN_BTN_SHUTDOWN);
    g_btn_reset.begin   (BTN_RESET,    PIN_BTN_RESET);
    g_btn_save.begin    (BTN_SAVE,     PIN_BTN_SAVE);

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

    // ---- Buttons (poll every loop; debounce inside Button::update) ----
    g_btn_shutdown.update(on_button_event);
    g_btn_reset.update   (on_button_event);
    g_btn_save.update    (on_button_event);

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
