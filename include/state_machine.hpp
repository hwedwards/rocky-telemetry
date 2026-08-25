#pragma once

#include "event.hpp"
#include "imu.hpp"
#include "pico/time.h"
#include <cstdint>

enum class State : uint8_t {
    OFF,
    LOGGING,
};

class StateMachine
{
    public:
        // The IMU is owned by main() and outlives the state machine; this only
        // borrows it. The reference is bound once here and cannot be re-seated,
        // which is exactly the guarantee we want.
        explicit StateMachine(IMU &imu)
            : imu(imu), current_state(State::OFF),
              next_sample(nil_time), sample_count(0) {}

        // Runs in main context, never from an ISR, so it is free to block.
        void handle_event(Event event);

        // Called every pass of the main loop. Does nothing unless LOGGING, and
        // never blocks -- it samples only when the interval below has elapsed.
        void poll();

        State get_current_state() const { return current_state; }

    private:
        void enter_logging();
        void enter_off();

        static constexpr int64_t SAMPLE_INTERVAL_US = 10000;  // 100 Hz
        static constexpr uint32_t PRINT_EVERY = 100;          // one line per second

        IMU &imu;
        State current_state;
        absolute_time_t next_sample;
        uint32_t sample_count;
};
