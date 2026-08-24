#pragma once

#include "event.hpp"
#include <cstdint>

enum class State : uint8_t {
    OFF,
    LOGGING,
};

class StateMachine
{
    public:
        StateMachine() : current_state(State::OFF) {}

        // Runs in main context, never from an ISR, so it is free to block.
        // Returns an event to re-inject (for transitions that cascade), or
        // Event::NONE when the machine has settled.
        void handle_event(const Event &event); 

        State get_current_state() const { return current_state; }

    private:
        State current_state;
};
