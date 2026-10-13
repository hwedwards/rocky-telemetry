#include "imu_fifo.hpp"

// Constant evaluation makes an accidental read beyond either 12-byte input
// a compiler error, even when testing with a cross compiler and no hardware.
constexpr uint8_t mixed[12] = {
    0x12, 0x34, 0xFE, 0xDC, 0x80, 0x00,
    0x7F, 0xFF, 0xFF, 0xFF, 0x00, 0x01
};
constexpr auto sample = imu_fifo::decode(mixed);
static_assert(sample.accel[0] == 4660 && sample.accel[1] == -292 &&
              sample.accel[2] == -32768, "accel order, endian and sign");
static_assert(sample.gyro[0] == 32767 && sample.gyro[1] == -1 &&
              sample.gyro[2] == 1, "gyro starts at byte 6, not byte 8");

constexpr uint8_t zero[12] = {};
constexpr auto stationary = imu_fifo::decode(zero);
static_assert(stationary.accel[0] == 0 && stationary.accel[1] == 0 &&
              stationary.accel[2] == 0 && stationary.gyro[0] == 0 &&
              stationary.gyro[1] == 0 && stationary.gyro[2] == 0, "zero record");

constexpr uint8_t consecutive[24] = {
    0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6,
    0, 7, 0, 8, 0, 9, 0, 10, 0, 11, 0, 12
};
constexpr auto second = imu_fifo::decode(consecutive + imu_fifo::RECORD_LEN);
static_assert(second.accel[0] == 7 && second.accel[1] == 8 &&
              second.accel[2] == 9 && second.gyro[0] == 10 &&
              second.gyro[1] == 11 && second.gyro[2] == 12, "record stride");
static_assert(imu_fifo::ENABLE_MASK == 0x78, "only accel and XYZ gyro enabled");
static_assert(imu_fifo::RECORD_LEN == 12, "six two-byte values");
