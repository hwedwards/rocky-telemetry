#include "state_machine.hpp"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

void StateMachine::enter_logging()
{
    if (!logger.start()) {
        printf("Refusing to enter LOGGING - acquisition would not start\n");
        return;             // stay in OFF; the LED never lies about the state
    }
    current_state = State::LOGGING;
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    printf("Transitioning to LOGGING state\n");
}

void StateMachine::enter_off()
{
    logger.stop();
    current_state = State::OFF;
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    printf("Transitioning to OFF state\n");
}

void StateMachine::handle_event(Event event)
{
    // Posted by core 1, so it can arrive in either state -- a write that was
    // still in flight when we stopped reports after the move to OFF.
    if (event == Event::WRITE_ERROR) {
        logger.on_write_error();
        return;
    }

    switch (current_state) {
        case State::OFF:
            // DRAIN_FIFO and FIFO_OVERFLOW are deliberately unhandled here.
            // A timer tick that was already in the queue when we stopped gets
            // dropped on the floor, which is exactly what we want.
            if (event == Event::BUTTON_PRESSED) {
                enter_logging();
            }
            break;

        case State::LOGGING:
            switch (event) {
                case Event::BUTTON_PRESSED:
                    enter_off();
                    break;

                case Event::DRAIN_FIFO:
                    logger.drain();
                    break;

                case Event::FIFO_OVERFLOW:
                    logger.on_overflow();
                    break;

                default:
                    break;
            }
            break;
    }
}
