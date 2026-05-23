// =============================================================================
// config.h — Hardware pinout and protocol tunables for recon_io_hub
//
// Edit a value here and re-flash; everything else compiles unchanged.
//
// All buttons use INPUT_PULLUP — wire each one between the GPIO and GND.
// Pressed = LOW.
// =============================================================================
#pragma once

#include <stdint.h>

// ---- I2C (MPU-6050) ---------------------------------------------------------
// Default ESP32 DevKit V1 / WROOM-32 I2C pins.
constexpr int     PIN_I2C_SDA       = 21;
constexpr int     PIN_I2C_SCL       = 22;
constexpr uint32_t I2C_FREQ_HZ      = 400000;   // Fast-mode I2C
constexpr uint8_t MPU6050_ADDR      = 0x68;     // AD0 tied LOW

// ---- Buttons (active-low, INPUT_PULLUP) -------------------------------------
constexpr int PIN_BTN_SHUTDOWN = 25;
constexpr int PIN_BTN_RESET    = 26;
constexpr int PIN_BTN_SAVE     = 27;

// ---- Status LED -------------------------------------------------------------
// Most ESP32 DevKit V1 boards expose the on-board blue LED on GPIO2.
constexpr int PIN_STATUS_LED = 2;

// ---- LIDAR power control (S8050 low-side switch on the LD14P GND/RX rail) ---
// HIGH ⇒ S8050 saturated ⇒ LIDAR powered.  LOW ⇒ motor off.
// Pin set to OUTPUT + LOW as the very first statement in setup() so the motor
// stays off through the boot window even without an external pull-down.
constexpr int PIN_LIDAR_EN = 4;

// If the Pi stops sending LIDAR_EN refreshes for this long, the firmware
// forces the motor off — protects against a Pi crash leaving the LIDAR
// spinning indefinitely. Set to 3 s; the Pi bridge re-sends every 1 s
// while scanning is active.
constexpr uint32_t LIDAR_WATCHDOG_MS = 3000;

// ---- LIDAR data ingestion (Serial2 from the LD14P TX pin) ------------------
// LD14P streams data at 230400 8N1 on its TX (white) pin. We tap it via
// ESP32 Serial2 RX. ESP32 Serial2 default RX = GPIO16; TX = GPIO17 (unused
// here because the LD14P RX/green wire is tied to LIDAR GND via the S8050
// for internal speed control).
constexpr int      PIN_LIDAR_SERIAL2_RX = 16;
constexpr uint32_t LIDAR_SERIAL2_BAUD   = 230400;

// ---- UART (ESP32 ⇄ Pi) ------------------------------------------------------
// We use the on-board USB-Serial bridge (UART0 on GPIO1/3). Plug the ESP32's
// micro-USB port into the Pi; the Pi sees /dev/ttyUSB0 (CP2102) or /dev/ttyACM0
// depending on the board's USB-Serial chip.
//
// 460800 baud carries IMU (~3 KB/s) + LIDAR data (~23 KB/s @ LD14P's native
// 230400 baud) + framing overhead with headroom. 115200 was fine for IMU-only
// but is too narrow once we add LIDAR_FRAME traffic.
constexpr uint32_t UART_BAUD = 460800;

// ---- Loop schedule ----------------------------------------------------------
// IMU sample rate in Hz. 100 Hz is plenty for handheld scanning and stays
// well under the 11.5 KB/s sustained UART budget at 115200 8N1.
constexpr uint32_t IMU_RATE_HZ      = 100;
constexpr uint32_t IMU_PERIOD_MS    = 1000 / IMU_RATE_HZ;

// Heartbeat — Pi-side bridge uses this to detect a hung ESP32.
constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000;

// Button debounce + long-press thresholds.
constexpr uint32_t BUTTON_DEBOUNCE_MS  = 25;
constexpr uint32_t BUTTON_LONGPRESS_MS = 2000;

// ---- UART framing -----------------------------------------------------------
// Frame: [0xA5][0x5A][TYPE][LEN][PAYLOAD...][CRC8]
// SYNC pattern is unlikely to appear naturally in IMU float streams.
constexpr uint8_t SYNC_BYTE_0 = 0xA5;
constexpr uint8_t SYNC_BYTE_1 = 0x5A;

enum FrameType : uint8_t {
    FRAME_IMU         = 0x01,  // 24 B payload: 6× float32 (ax,ay,az, gx,gy,gz)
    FRAME_BUTTON      = 0x02,  //  2 B payload: uint8 id, uint8 state
    FRAME_HEARTBEAT   = 0x03,  //  4 B payload: uint32 uptime_ms (LE)
    FRAME_STATUS      = 0x04,  //  2 B or 8 B (see send_status_diag)
    // Bidirectional additions (Increment 1 / 2):
    FRAME_LIDAR_FRAME = 0x05,  // N B payload: raw LD14P bytes (ESP32 → Pi)
    FRAME_LIDAR_EN    = 0x06,  //  1 B payload: 0=off, 1=on   (Pi → ESP32)
    FRAME_LIDAR_ACK   = 0x07,  //  1 B payload: current LIDAR_EN state (ESP32 → Pi)
};

enum ButtonId : uint8_t {
    BTN_SHUTDOWN = 0,
    BTN_RESET    = 1,
    BTN_SAVE     = 2,
};

enum ButtonState : uint8_t {
    BTN_RELEASED  = 0,
    BTN_PRESSED   = 1,
    BTN_LONGPRESS = 2,
};

// Status bitmask carried in FRAME_STATUS.payload[0].
constexpr uint8_t STATUS_BOOT      = 1 << 0;  // Set on first frame after boot
constexpr uint8_t STATUS_IMU_OK    = 1 << 1;  // 0 ⇒ IMU not responding on I2C
constexpr uint8_t STATUS_IMU_DATA  = 1 << 2;  // 1 ⇒ at least one fresh sample
