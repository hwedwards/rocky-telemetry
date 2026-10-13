#pragma once

#include <cstddef>
#include <cstdint>

// MPU-6000/6050 register map, sections 4.7 and 4.33: enabled sensor
// registers enter the FIFO in ascending address order, high byte first.
namespace imu_fifo {
constexpr uint8_t ENABLE_MASK = (1u << 6) | (1u << 5) | (1u << 4) | (1u << 3);
constexpr size_t RECORD_LEN = 12;

struct Record {
    int16_t accel[3];
    int16_t gyro[3];
};

constexpr int16_t signed_be16(const uint8_t *raw)
{
    const uint16_t value = (uint16_t(raw[0]) << 8) | raw[1];
    // Avoid implementation-defined unsigned-to-signed narrowing.
    return value < 0x8000 ? int16_t(value) : int16_t(int32_t(value) - 0x10000);
}

// Caller supplies one complete RECORD_LEN-byte record. No temperature bytes.
constexpr Record decode(const uint8_t *raw)
{
    Record record{};
    for (size_t axis = 0; axis < 3; ++axis) {
        record.accel[axis] = signed_be16(raw + 2 * axis);
        record.gyro[axis] = signed_be16(raw + 6 + 2 * axis);
    }
    return record;
}
} // namespace imu_fifo
