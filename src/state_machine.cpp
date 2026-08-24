#include "state_machine.hpp"

Event StateMachine::dispatch(Event e)
{
    switch (current_state) {
        case State::OFF:
            if (e == Event::BUTTON_PRESSED) {
                current_state = State::LOGGING;
            }
            break;

        case State::LOGGING:
            if (e == Event::BUTTON_PRESSED) {
                current_state = State::OFF;
            }
            break;
    }

    // No cascading transitions yet. When entering LOGGING starts the DMA,
    // this is where you would return the follow-up event.
    return Event::NONE;
}
