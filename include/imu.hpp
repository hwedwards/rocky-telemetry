#pragma once

#include "hardware/i2c.h"
#include <cstdint>
#include <cstddef>

/* MPU6050 MEMS accelerometer + gyroscope on I2C.

   Configured for a 1 kHz output data rate with the on-chip DLPF set to
   ~188 Hz, which is the widest bandwidth that still respects Nyquist at
   1 kHz. Gyro bias is measured by calibrate() and subtracted by read().

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefore I2C) cannot be used at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO sda_pin (GP0) -> SDA on MPU6050 board
   GPIO scl_pin (GP1) -> SCL on MPU6050 board
   3.3v (pin 36) -> VCC on MPU6050 board
   GND (pin 38)  -> GND on MPU6050 board
*/
class IMU
{
    public:
        // One sampling instant. Every field comes from the same burst read, so
        // the values are consistent with each other -- three separate reads
        // cannot promise that. Units are raw LSB; use the to_*() helpers.
        struct Sample {
            int16_t accel[3];   // X, Y, Z
            int16_t gyro[3];    // X, Y, Z, with the calibrated bias removed
            int16_t temp;       // die temperature, not ambient
        };

        // Touches no hardware -- safe to construct as a global before main().
        // Address 0x68 is the default; 0x69 if the AD0 pin is strapped high.
        IMU(i2c_inst_t *bus, uint sda_pin, uint scl_pin, uint8_t addr = 0x68)
            : bus(bus), sda_pin(sda_pin), scl_pin(scl_pin), addr(addr),
              gyro_bias{0, 0, 0} {}

        // Brings up the I2C pins, verifies WHO_AM_I, and configures the device.
        // Returns false if the part does not answer as an MPU6050.
        bool init(uint baudrate = 400 * 1000);

        // Averages a stationary run to find the gyro zero offset, and reports
        // the spread on every axis so real noise can be told apart from a bad
        // part. Blocks for ~2 ms per sample -- keep the board still.
        void calibrate(int samples = 500);

        // One transaction covering all 14 measurement registers.
        // Returns false on a bus error, leaving out untouched.
        bool read(Sample &out);

        // ---- FIFO ----------------------------------------------------------
        // The on-chip FIFO collects samples at the full output data rate while
        // the CPU is busy elsewhere. This is what stops a slow storage write
        // costing us samples: 1024 bytes is ~85 ms of headroom at 1 kHz.

        // One FIFO record: accel(6) + gyro(6), same order as the measurement
        // registers. Temperature is deliberately left out of FIFO_EN -- we do
        // not use it, and dropping it takes the record from 14 to 12 bytes,
        // which buys back both I2C bus time and FIFO depth.
        static constexpr size_t FIFO_SAMPLE_LEN = 12;
        static constexpr size_t FIFO_CAPACITY   = 1024;

        // Enable/disable FIFO collection. Enabling also clears whatever was
        // in there, so the first record out is the first record after start.
        bool fifo_enable(bool on);
        bool fifo_reset();

        // Bytes currently waiting. Not necessarily a whole number of records --
        // the count can be read mid-write, so round down before draining.
        bool fifo_count(uint16_t &out);

        // Bulk read straight out of FIFO_R_W into a caller-supplied buffer.
        // The bytes are stored raw (big-endian pairs); decode() converts one.
        bool fifo_read(uint8_t *dst, size_t len);

        // True if the FIFO wrapped since the last check. Reading the status
        // register clears the flag, so treat a true here as one fault event.
        bool fifo_overflowed(bool &out);

        // Turn one raw 14-byte FIFO record into a Sample, gyro bias removed.
        Sample decode(const uint8_t *raw) const;

        // Scale factors for the configured full-scale ranges (+/-2g, +/-250 deg/s)
        static constexpr float ACCEL_LSB_PER_G  = 16384.0f;
        static constexpr float GYRO_LSB_PER_DPS = 131.0f;

        static float to_g(int16_t raw)       { return raw / ACCEL_LSB_PER_G; }
        static float to_dps(int16_t raw)     { return raw / GYRO_LSB_PER_DPS; }
        static float to_celsius(int16_t raw) { return raw / 340.0f + 36.53f; }

    private:
        bool write_reg(uint8_t reg, uint8_t val);
        bool read_regs(uint8_t reg, uint8_t *dst, size_t len);
        bool check();
        void reset();
        bool read_raw(Sample &out);     // read() without the bias subtraction
        bool read_reg(uint8_t reg, uint8_t &val);

        i2c_inst_t *bus;
        uint sda_pin;
        uint scl_pin;
        uint8_t addr;
        int16_t gyro_bias[3];
};
