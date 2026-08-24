#include "state_machine.hpp"
#include "event.hpp"
#include <stdio.h> // for printfs
#include <cstdint>
#include "hardware/gpio.h"

// Runs in main context, never from an ISR, so it is free to block.
// Returns an event to re-inject (for transitions that cascade), or
// Event::NONE when the machine has settled.
void StateMachine::handle_event(const Event &event){
    switch (current_state){
        case State::OFF:
            if (event == Event::BUTTON_PRESSED){
                current_state = State::LOGGING;
                printf("Transitioning to LOGGING state\n");
                gpio_put(PICO_DEFAULT_LED_PIN, 1); // Turn on the LED when logging starts
            }
            break;
        case State::LOGGING:
            if (event == Event::BUTTON_PRESSED){
                current_state = State::OFF;
                gpio_put(PICO_DEFAULT_LED_PIN, 0); // Turn off the LED when logging stops
            }
            printf("Transitioning to OFF state\n");
            break;
    }
}
