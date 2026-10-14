#include "imu.hpp"
#include <array>
#include <cassert>
#include <cstdio>
#include <deque>

// Model only the documented FIFO gates, register writes and read port.
static std::array<uint8_t, 256> regs{};
static std::deque<uint8_t> fifo;
static uint8_t selected;
static bool fail_write = false;
int i2c_write_blocking(i2c_inst_t *, uint8_t, const uint8_t *data, size_t len, bool) {
    if (fail_write) return -1;
    assert(len == 1 || len == 2);
    selected = data[0];
    if (len == 2) {
        regs[selected] = data[1];
        if (selected == 0x6a && (data[1] & 4)) {
            fifo.clear(); regs[selected] &= ~4;
        }
    }
    return int(len);
}
int i2c_read_blocking(i2c_inst_t *, uint8_t, uint8_t *out, size_t len, bool) {
    if (selected == 0x72) {
        assert(len == 2); out[0] = uint8_t(fifo.size() >> 8); out[1] = uint8_t(fifo.size());
    } else if (selected == 0x74) {
        if (!(regs[0x6a] & 0x40)) return -1; // Disabled FIFO cannot be read.
        assert(fifo.size() >= len);
        for (size_t i = 0; i < len; ++i) { out[i] = fifo.front(); fifo.pop_front(); }
    } else {
        for (size_t i = 0; i < len; ++i) out[i] = regs[selected + i];
    }
    return int(len);
}
static void produce(uint8_t seed) {
    if ((regs[0x6a] & 0x40) && regs[0x23] == imu_fifo::ENABLE_MASK)
        for (uint8_t i = 0; i < 12; ++i) fifo.push_back(seed + i);
}
int main() {
    i2c_inst_t bus;
    IMU imu(&bus, 0, 1);
    Inputs &source = imu;
    assert(source.start()); produce(10);
    assert(source.stop()); produce(80);
    size_t count = 0; assert(source.available(count) && count == 12);
    std::array<uint8_t, 12> bytes{};
    assert(source.read(bytes.data(), bytes.size()));
    for (uint8_t i = 0; i < 12; ++i) assert(bytes[i] == 10 + i);
    assert(source.available(count) && count == 0);
    assert(source.stop()); // Idempotent and no reset.
    assert(source.start()); produce(30);
    assert(source.available(count) && count == 12);
    fail_write = true; assert(!source.stop()); fail_write = false;
    assert(source.stop()); assert(source.read(bytes.data(), bytes.size()));
    for (uint8_t i = 0; i < 12; ++i) assert(bytes[i] == 30 + i);
    std::puts("PASS: IMU stop freezes producers, preserves FIFO reads/bytes, restarts, and propagates I2C failure");
}
