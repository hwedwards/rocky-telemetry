#include "state_machine.hpp"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

void StateMachine::enter_logging()
{
    if (!pipeline.start()) {
        printf("Refusing to enter LOGGING - acquisition would not start\n");
        return;
    }
    current_state = State::LOGGING;
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    printf("Transitioning to LOGGING state\n");
}
void StateMachine::enter_off()
{
    if (!pipeline.stop()) {
        current_state = State::STOPPING;
        printf("Shutdown pending; press again to finish stopping\n");
        return;
    }
    current_state = State::OFF;
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    printf("Transitioning to OFF state\n");
}
void StateMachine::handle_event(Event event)
{
    if (event == Event::BUTTON_PRESSED) {
        if (current_state == State::OFF) enter_logging();
        else enter_off(); // STOPPING retries the same stop, never restarts.
    } else {
        pipeline.handle_event(event);
    }
}
