#pragma once

#include "event.hpp"
#include "process.hpp"
#include <cstdint>

enum class State : uint8_t {
    OFF,
    LOGGING,
    STOPPING,
};

class StateMachine
{
    public:
        // The Process is owned by main() and outlives the state machine; this
        // only borrows it. A reference member cannot be re-seated after
        // construction, which is exactly the guarantee we want.
        explicit StateMachine(Process &pipeline)
            : pipeline(pipeline), current_state(State::OFF) {}

        // Runs in main context, never from an ISR. It is allowed to block --
        // but while logging its work must finish within the source's
        // buffering headroom. stop() may wait after freezing acquisition.
        void handle_event(Event event);

        State get_current_state() const { return current_state; }

    private:
        void enter_logging();
        void enter_off();

        Process &pipeline;
        State current_state;
};
