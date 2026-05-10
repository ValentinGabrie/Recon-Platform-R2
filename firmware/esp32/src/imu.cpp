// =============================================================================
// imu.cpp — MPU-6050 register-level driver.
// Datasheet: InvenSense MPU-6000/6050 Register Map rev 4.2.
// =============================================================================
#include "imu.h"

#include <Wire.h>

#include "config.h"

namespace {

// Register addresses we touch.
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
    uint8_t who = 0;
    if (!read_reg(REG_WHO_AM_I, who) ||
        (who != 0x68 && who != 0x70 && who != 0x72))
    {
        g_ok = false;
        return false;
    }

    // Wake from sleep + use PLL with X-axis gyro reference.
    if (!write_reg(REG_PWR_MGMT_1, 0x01)) { g_ok = false; return false; }
    delay(2);

    // 1 kHz / (1 + SMPLRT_DIV) = output rate. With DLPF active (CONFIG != 0
    // and != 7) the gyro output is 1 kHz, so SMPLRT_DIV = 9 → 100 Hz.
    if (!write_reg(REG_SMPLRT_DIV, 9))   { g_ok = false; return false; }
    if (!write_reg(REG_CONFIG, 0x03))    { g_ok = false; return false; }
    if (!write_reg(REG_GYRO_CONFIG, 0x08))  { g_ok = false; return false; } // ±500 °/s
    if (!write_reg(REG_ACCEL_CONFIG, 0x08)) { g_ok = false; return false; } // ±4 g

    g_ok = true;
    return true;
}

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
