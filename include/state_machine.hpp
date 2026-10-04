#pragma once

#include "event.hpp"
#include "logger.hpp"
#include <cstdint>

enum class State : uint8_t {
    OFF,
    LOGGING,
};

class StateMachine
{
    public:
        // The Logger is owned by main() and outlives the state machine; this
        // only borrows it. A reference member cannot be re-seated after
        // construction, which is exactly the guarantee we want.
        explicit StateMachine(Logger &logger)
            : logger(logger), current_state(State::OFF) {}

        // Runs in main context, never from an ISR. It is allowed to block --
        // but everything it does has to finish well inside the FIFO's 85 ms of
        // headroom, or the next drain arrives late and we lose samples.
        void handle_event(Event event);

        State get_current_state() const { return current_state; }

    private:
        void enter_logging();
        void enter_off();

        Logger &logger;
        State current_state;
};
