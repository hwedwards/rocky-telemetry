#pragma once

#include <cstdint>

// Events are the only thing that crosses from interrupt context into the
// state machine. Everything the machine does is a reaction to one of these
// arriving in the queue, which is what keeps the main loop a pure event pump.
enum class Event : uint8_t {
    NONE = 0,        // "nothing to re-inject"
    BUTTON_PRESSED,  // debounced press, from the GPIO alarm callback
    DRAIN_FIFO,      // repeating timer: samples are waiting on the MPU6050
    FIFO_OVERFLOW,   // the sensor FIFO wrapped -- we fell behind and lost data
    WRITE_ERROR,     // storage write failed, posted by the writer on core 1
};
