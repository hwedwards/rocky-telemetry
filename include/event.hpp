#pragma once

#include <cstdint>

// Events are the only thing that crosses from interrupt context into the
// state machine. Add more as sources appear (DMA complete, sensor watermark,
// low battery, watchdog warning...).
enum class Event : uint8_t {
    NONE = 0,       // "nothing to re-inject" -- see StateMachine::dispatch
    BUTTON_PRESSED,
};
