// =============================================================================
// imu.cpp — MPU-6050 register-level driver.
// Datasheet: InvenSense MPU-6000/6050 Register Map rev 4.2.
// =============================================================================
#include "imu.h"

#include <Wire.h>

#include "config.h"

namespace {

// Register addresses we touch.
constexpr uint8_t REG_XA_OFFSET_H  = 0x06;  // user-programmable accel X offset (H)
constexpr uint8_t REG_XA_OFFSET_L  = 0x07;  //                                   (L)
constexpr uint8_t REG_YA_OFFSET_H  = 0x08;
constexpr uint8_t REG_YA_OFFSET_L  = 0x09;
constexpr uint8_t REG_ZA_OFFSET_H  = 0x0A;
constexpr uint8_t REG_ZA_OFFSET_L  = 0x0B;
constexpr uint8_t REG_XG_OFFSET_H  = 0x13;  // user-programmable gyro X offset (H)
constexpr uint8_t REG_XG_OFFSET_L  = 0x14;
constexpr uint8_t REG_YG_OFFSET_H  = 0x15;
constexpr uint8_t REG_YG_OFFSET_L  = 0x16;
constexpr uint8_t REG_ZG_OFFSET_H  = 0x17;
constexpr uint8_t REG_ZG_OFFSET_L  = 0x18;
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

// Conversion factors for our chosen full-scale settings.
//   AFS_SEL = 1 → ±4 g, sensitivity 8192 LSB/g  → 1/8192 g per LSB
//   FS_SEL  = 1 → ±500 °/s, sensitivity 65.5 LSB/(°/s) → 1/65.5 (°/s) per LSB
constexpr float ACCEL_LSB_TO_MS2  = 9.80665f / 8192.0f;
constexpr float GYRO_LSB_TO_RADS  = (PI / 180.0f) / 65.5f;

bool g_ok = false;

// Chip-state diagnostics captured during begin(). Exposed via the imu::
// public API so main.cpp can ship them in a STATUS frame for bench
// debugging.
uint8_t g_who_am_i  = 0;
uint8_t g_accel_cfg = 0;
uint8_t g_gyro_cfg  = 0;
int16_t g_za_offset_before = 0;  // ZA_OFFSET as read on first attach
int16_t g_za_offset_after  = 0;  // ZA_OFFSET after we wrote 0 to it

bool write_reg(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool read_reg(uint8_t reg, uint8_t& value)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(MPU6050_ADDR, (uint8_t)1) != 1) {
        return false;
    }
    value = Wire.read();
    return true;
}

}  // namespace

namespace imu {

bool begin()
{
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
    Wire.setTimeOut(50);  // ms — don't let a stuck bus block the loop

    // WHO_AM_I should read 0x68 on a real MPU-6050 (the upper bits of the
    // I2C address). Some clones return 0x70 or 0x72 — accept those too.
    if (!read_reg(REG_WHO_AM_I, g_who_am_i) ||
        (g_who_am_i != 0x68 && g_who_am_i != 0x70 && g_who_am_i != 0x72))
    {
        g_ok = false;
        return false;
    }

    // Full chip reset (DEVICE_RESET = bit 7 of PWR_MGMT_1). The chip
    // self-clears the bit when reset finishes; datasheet quotes 30–35 ms
    // for "stable IMU operation" after wake-up, so 100 ms is comfortable
    // headroom. Without this, range-config writes that happen too soon
    // after power-on are silently dropped and the accel stays at the
    // ±2 g default — symptom is gravity reading ~2× too high.
    if (!write_reg(REG_PWR_MGMT_1, 0x80)) { g_ok = false; return false; }
    delay(100);

    // Wake from sleep + use PLL with X-axis gyro reference (CLKSEL=1).
    if (!write_reg(REG_PWR_MGMT_1, 0x01)) { g_ok = false; return false; }
    delay(50);

    // 1 kHz / (1 + SMPLRT_DIV) = output rate. With DLPF active (CONFIG != 0
    // and != 7) the gyro output is 1 kHz, so SMPLRT_DIV = 9 → 100 Hz.
    if (!write_reg(REG_SMPLRT_DIV, 9))   { g_ok = false; return false; }
    if (!write_reg(REG_CONFIG, 0x03))    { g_ok = false; return false; }
    if (!write_reg(REG_GYRO_CONFIG, 0x08))  { g_ok = false; return false; } // ±500 °/s
    if (!write_reg(REG_ACCEL_CONFIG, 0x08)) { g_ok = false; return false; } // ±4 g

    // Verify the range bits actually landed (bits [4:3] of each CONFIG
    // register == AFS_SEL/FS_SEL). If they didn't, the LSB-to-SI
    // conversion would be 2×/0.5× wrong and we'd produce garbage
    // physics downstream — fail begin() instead.
    if (!read_reg(REG_GYRO_CONFIG, g_gyro_cfg) ||
        !read_reg(REG_ACCEL_CONFIG, g_accel_cfg))
    {
        g_ok = false;
        return false;
    }
    if ((g_gyro_cfg & 0x18) != 0x08 || (g_accel_cfg & 0x18) != 0x08) {
        g_ok = false;
        return false;
    }

    // Capture the existing ZA_OFFSET for diagnostics — do NOT touch it.
    //
    // On bench testing this chip (MPU-6050, WHO_AM_I=0x68) we found a
    // factory ZA_OFFSET of 1544 LSB. The offset registers (0x06–0x0B
    // for accel, 0x13–0x18 for gyro) interact with a non-public factory
    // trim and reserved bit 0 of each L byte that gates temperature
    // compensation. Naively writing 0 to ZA_OFFSET_L clobbers that
    // reserved bit and *worsens* calibration (gravity total magnitude
    // measured 7.83 m/s² after a naive zero-write vs 15.35 with the
    // factory bias intact). The right time to address accel bias is
    // H3, where the Madgwick + EKF stack estimates and removes it
    // online — until then the chip ships its raw factory-calibrated
    // counts and the Pi-side decoder shows them as-is.
    uint8_t za_h = 0, za_l = 0;
    read_reg(REG_ZA_OFFSET_H, za_h);
    read_reg(REG_ZA_OFFSET_L, za_l);
    g_za_offset_before = (int16_t)((za_h << 8) | za_l);
    g_za_offset_after  = g_za_offset_before;  // unchanged

    g_ok = true;
    return true;
}

uint8_t who_am_i()         { return g_who_am_i; }
uint8_t accel_cfg()        { return g_accel_cfg; }
uint8_t gyro_cfg()         { return g_gyro_cfg; }
int16_t za_offset_before() { return g_za_offset_before; }
int16_t za_offset_after()  { return g_za_offset_after; }

bool ok() { return g_ok; }

bool read(ImuSample& out)
{
    if (!g_ok) {
        return false;
    }

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) {
        g_ok = false;
        return false;
    }
    if (Wire.requestFrom(MPU6050_ADDR, (uint8_t)14) != 14) {
        g_ok = false;
        return false;
    }

    // 14 bytes: AX_H AX_L AY_H AY_L AZ_H AZ_L T_H T_L GX_H GX_L GY_H GY_L GZ_H GZ_L
    int16_t ax = (int16_t)((Wire.read() << 8) | Wire.read());
    int16_t ay = (int16_t)((Wire.read() << 8) | Wire.read());
    int16_t az = (int16_t)((Wire.read() << 8) | Wire.read());
    (void)Wire.read();  // temp high — discard
    (void)Wire.read();  // temp low
    int16_t gx = (int16_t)((Wire.read() << 8) | Wire.read());
    int16_t gy = (int16_t)((Wire.read() << 8) | Wire.read());
    int16_t gz = (int16_t)((Wire.read() << 8) | Wire.read());

    out.ax = ax * ACCEL_LSB_TO_MS2;
    out.ay = ay * ACCEL_LSB_TO_MS2;
    out.az = az * ACCEL_LSB_TO_MS2;
    out.gx = gx * GYRO_LSB_TO_RADS;
    out.gy = gy * GYRO_LSB_TO_RADS;
    out.gz = gz * GYRO_LSB_TO_RADS;
    return true;
}

}  // namespace imu
