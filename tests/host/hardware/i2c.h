#pragma once
#include <cstddef>
#include <cstdint>
using uint = unsigned;
struct i2c_inst_t {};
inline uint i2c_init(i2c_inst_t *, uint baud) { return baud; }
int i2c_write_blocking(i2c_inst_t *, uint8_t, const uint8_t *, size_t, bool);
int i2c_read_blocking(i2c_inst_t *, uint8_t, uint8_t *, size_t, bool);
