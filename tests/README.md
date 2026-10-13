# FIFO regression checks

These compile-time tests execute the production FIFO decoder on known 12-byte
records. They check byte order, negative values, signed extremes, gyro offsets,
record stride, and the FIFO enable mask. Constant evaluation also rejects reads
past the end of the exactly sized input arrays.

Run with any C++17 compiler, including the Pico ARM cross compiler:

```sh
arm-none-eabi-g++ -std=c++17 -Wall -Wextra -Werror -Iinclude -fsyntax-only tests/imu_fifo_test.cpp
```

A successful compiler exit means every assertion passed. This does not execute
firmware or verify I2C hardware. The test is also compiled by the normal CMake
build as `imu_fifo_regression`.
