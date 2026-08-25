/**
 * MPU6050 driver, derived from the Raspberry Pi Pico SDK i2c example.
 *
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "imu.hpp"
#include "pico/stdlib.h"
#include <stdio.h>
#include <cmath>

// Register addresses. Hex is what the bus wants; the decimal in the comment is
// how the "Register Map and Descriptions" datasheet indexes its sections.
#define REG_SMPLRT_DIV    0x19  // 25  - sample rate divider
#define REG_CONFIG        0x1A  // 26  - DLPF / external sync
#define REG_GYRO_CONFIG   0x1B  // 27  - gyro full-scale range, self-test
#define REG_ACCEL_CONFIG  0x1C  // 28  - accel full-scale range, self-test
#define REG_FIFO_EN       0x23  // 35  - which sensors feed the FIFO
#define REG_INT_ENABLE    0x38  // 56  - interrupt sources
#define REG_INT_STATUS    0x3A  // 58  - latched interrupt flags, clears on read
#define REG_ACCEL_XOUT_H  0x3B  // 59  - first of the 14 measurement registers
#define REG_USER_CTRL     0x6A  // 106 - FIFO enable / FIFO reset
#define REG_FIFO_COUNT_H  0x72  // 114 - bytes waiting, high byte first
#define REG_FIFO_R_W      0x74  // 116 - read port; each read pops one byte
#define REG_PWR_MGMT_1    0x6B  // 107 - reset, sleep, clock source
#define REG_WHO_AM_I      0x75  // 117 - fixed identity, reads back 0x68

#define SENSOR_BLOCK_LEN  14    // accel(6) + temp(2) + gyro(6), contiguous

// FIFO_EN bits: all three gyro axes + accel. Bit 7 (TEMP_FIFO_EN) is left
// clear -- we never use the die temperature, and omitting it makes the
// record 12 bytes instead of 14. The layout is otherwise the measurement
// register order with the temperature pair cut out of the middle.
#define FIFO_EN_ALL       0x78

#define USER_CTRL_FIFO_EN     (1u << 6)
#define USER_CTRL_FIFO_RESET  (1u << 2)
#define INT_FIFO_OFLOW        (1u << 4)

static constexpr float PI_F = 3.14159265358979f;

// Write one byte to one register. First byte on the wire is the register
// pointer, the second is the value that lands there.
bool IMU::write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    int r = i2c_write_blocking(bus, addr, buf, 2, false);
    if (r != 2) {
        printf("I2C write to reg 0x%02X failed (%d)\n", reg, r);
        return false;
    }
    return true;
}

// Point the device at a register, then read a run of bytes from it. The
// pointer auto-increments, so len bytes come from reg, reg+1, reg+2, ...
// nostop=true on the write holds the bus so nothing can move the pointer.
bool IMU::read_regs(uint8_t reg, uint8_t *dst, size_t len)
{
    int w = i2c_write_blocking(bus, addr, &reg, 1, true);
    if (w != 1) {
        printf("I2C addressing of reg 0x%02X failed (%d)\n", reg, w);
        return false;
    }
    int r = i2c_read_blocking(bus, addr, dst, len, false);
    if (r != (int)len) {
        printf("I2C read of reg 0x%02X failed (%d)\n", reg, r);
        return false;
    }
    return true;
}

// WHO_AM_I holds address bits 6:1 and ignores the AD0 pin, so a genuine
// MPU6050 answers 0x68 even when strapped to bus address 0x69.
bool IMU::check()
{
    uint8_t id = 0;
    if (!read_regs(REG_WHO_AM_I, &id, 1)) return false;
    printf("WHO_AM_I = 0x%02X (expect 0x68)\n", id);
    if (id != 0x68) {
        printf("  unexpected device - MPU9250/6500 clones report 0x70/0x71\n");
        return false;
    }
    return true;
}

void IMU::reset()
{
    // DEVICE_RESET: restores every register to its power-on default
    write_reg(REG_PWR_MGMT_1, 0x80);
    sleep_ms(100);

    // Clear sleep and select the gyro X PLL as clock source (CLKSEL=1).
    // More stable than the internal 8 MHz RC oscillator the reset default uses.
    write_reg(REG_PWR_MGMT_1, 0x01);
    sleep_ms(50);   // datasheet gives 30 ms gyro start-up time

    // DLPF_CFG=1: 184 Hz accel / 188 Hz gyro, and this is what drops the
    // gyro base rate from 8 kHz to 1 kHz so the divider below works out.
    write_reg(REG_CONFIG, 0x01);

    // Sample rate = 1 kHz / (1 + SMPLRT_DIV). 0 gives the full 1 kHz.
    write_reg(REG_SMPLRT_DIV, 0);

    // Explicit full-scale ranges: +/-250 deg/s and +/-2 g
    write_reg(REG_GYRO_CONFIG, 0x00);
    write_reg(REG_ACCEL_CONFIG, 0x00);
}

bool IMU::init(uint baudrate)
{
    i2c_init(bus, baudrate);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    if (!check()) {
        printf("MPU6050 not responding correctly - check wiring and pull-ups\n");
        return false;
    }

    reset();
    return true;
}

// One transaction covering all 14 measurement registers. The device latches
// the block for the duration of a burst read, so every value here comes from
// the same sampling instant.
bool IMU::read_raw(Sample &out)
{
    uint8_t b[SENSOR_BLOCK_LEN];
    if (!read_regs(REG_ACCEL_XOUT_H, b, SENSOR_BLOCK_LEN)) return false;

    for (int i = 0; i < 3; i++) {
        out.accel[i] = (int16_t)((b[i * 2] << 8) | b[i * 2 + 1]);
    }
    out.temp = (int16_t)((b[6] << 8) | b[7]);
    for (int i = 0; i < 3; i++) {
        out.gyro[i] = (int16_t)((b[8 + i * 2] << 8) | b[9 + i * 2]);
    }
    return true;
}

bool IMU::read_reg(uint8_t reg, uint8_t &val)
{
    return read_regs(reg, &val, 1);
}

bool IMU::read(Sample &out)
{
    if (!read_raw(out)) return false;
    for (int i = 0; i < 3; i++) out.gyro[i] -= gyro_bias[i];
    return true;
}

// Average a stationary run to find the gyro zero offset, and report the
// spread on every axis so real noise can be told apart from a bad part.
void IMU::calibrate(int samples)
{
    int64_t sum[6] = {0};
    int64_t sumsq[6] = {0};
    Sample s;
    int taken = 0;

    printf("\nCalibrating - keep the board still (%d samples)...\n", samples);

    for (int i = 0; i < samples; i++) {
        if (!read_raw(s)) continue;
        int32_t v[6] = {s.accel[0], s.accel[1], s.accel[2],
                        s.gyro[0],  s.gyro[1],  s.gyro[2]};
        for (int j = 0; j < 6; j++) {
            sum[j]   += v[j];
            sumsq[j] += (int64_t)v[j] * v[j];
        }
        taken++;
        sleep_ms(2);    // a new sample lands every 1 ms at 1 kHz
    }

    if (taken == 0) {
        printf("Calibration failed - no samples read\n");
        return;
    }

    float mean[6], sd[6];
    for (int j = 0; j < 6; j++) {
        mean[j] = (float)sum[j] / taken;
        float var = (float)sumsq[j] / taken - mean[j] * mean[j];
        sd[j] = var > 0 ? sqrtf(var) : 0.0f;
    }

    printf("             mean       sd     mean(unit)   sd(unit)\n");
    static const char *an[3] = {"X", "Y", "Z"};
    for (int j = 0; j < 3; j++) {
        printf("  Acc %s  %9.1f %8.1f  %8.4f g  %8.4f g\n",
               an[j], mean[j], sd[j],
               mean[j] / ACCEL_LSB_PER_G, sd[j] / ACCEL_LSB_PER_G);
    }
    for (int j = 3; j < 6; j++) {
        printf("  Gyr %s  %9.1f %8.1f  %6.3f d/s  %6.3f d/s\n",
               an[j - 3], mean[j], sd[j],
               mean[j] / GYRO_LSB_PER_DPS, sd[j] / GYRO_LSB_PER_DPS);
    }

    // Gravity is a known input: at rest the vector should measure exactly 1 g.
    float mag = sqrtf(mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2]);
    float tilt = atan2f(sqrtf(mean[0] * mean[0] + mean[1] * mean[1]), mean[2]); 
    printf("  |a| = %.4f g (want 1.0000), tilt from level = %.1f deg\n",
           mag / ACCEL_LSB_PER_G, tilt * 180.0f / PI_F);

    // Gyro reads zero at rest whatever the orientation, so its offset is safe
    // to remove. Accel bias is not - it needs a known orientation, so the
    // numbers above are reported but left alone.
    for (int j = 0; j < 3; j++) gyro_bias[j] = (int16_t)mean[j + 3];
    printf("  Gyro bias removed: %d, %d, %d\n\n",
           gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}

// ---- FIFO ------------------------------------------------------------------

bool IMU::fifo_reset()
{
    // FIFO_RESET is only honoured while FIFO_EN is clear, so drop the enable,
    // pulse the reset, then bring it back up.
    uint8_t ctrl = 0;
    if (!read_reg(REG_USER_CTRL, ctrl)) return false;
    bool was_enabled = ctrl & USER_CTRL_FIFO_EN;

    if (!write_reg(REG_USER_CTRL, ctrl & ~USER_CTRL_FIFO_EN)) return false;
    if (!write_reg(REG_USER_CTRL, USER_CTRL_FIFO_RESET)) return false;
    if (was_enabled && !write_reg(REG_USER_CTRL, USER_CTRL_FIFO_EN)) return false;

    // Reading the status register clears any overflow left over from before.
    uint8_t status;
    read_reg(REG_INT_STATUS, status);
    return true;
}

bool IMU::fifo_enable(bool on)
{
    if (!on) {
        if (!write_reg(REG_FIFO_EN, 0x00)) return false;
        return write_reg(REG_USER_CTRL, 0x00);
    }

    // Order matters: pick the sources, clear whatever is stale, then start.
    if (!write_reg(REG_FIFO_EN, FIFO_EN_ALL)) return false;
    if (!write_reg(REG_USER_CTRL, USER_CTRL_FIFO_RESET)) return false;
    if (!write_reg(REG_USER_CTRL, USER_CTRL_FIFO_EN)) return false;

    // Overflow is the only useful FIFO interrupt this part has -- there is no
    // programmable watermark. Enabling it means the flag latches in INT_STATUS
    // even with no INT pin wired, which is how fifo_overflowed() reads it.
    return write_reg(REG_INT_ENABLE, INT_FIFO_OFLOW);
}

bool IMU::fifo_count(uint16_t &out)
{
    uint8_t b[2];
    if (!read_regs(REG_FIFO_COUNT_H, b, 2)) return false;
    out = (uint16_t)((b[0] << 8) | b[1]);
    return true;
}

bool IMU::fifo_read(uint8_t *dst, size_t len)
{
    // FIFO_R_W does not auto-increment -- the register pointer stays put and
    // each read pops the next byte, so one addressed burst drains len bytes.
    return read_regs(REG_FIFO_R_W, dst, len);
}

bool IMU::fifo_overflowed(bool &out)
{
    uint8_t status = 0;
    if (!read_reg(REG_INT_STATUS, status)) return false;
    out = (status & INT_FIFO_OFLOW) != 0;
    return true;
}

IMU::Sample IMU::decode(const uint8_t *raw) const
{
    Sample s;
    for (int i = 0; i < 3; i++) {
        s.accel[i] = (int16_t)((raw[i * 2] << 8) | raw[i * 2 + 1]);
    }
    // Not in the FIFO stream -- see FIFO_EN_ALL. read() still fills this in,
    // because the measurement registers always carry it.
    s.temp = 0;
    for (int i = 0; i < 3; i++) {
        s.gyro[i] = (int16_t)((raw[6 + i * 2] << 8) | raw[7 + i * 2]) - gyro_bias[i];
    }
    return s;
}
