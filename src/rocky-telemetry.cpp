#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "button.hpp"
#include "state_machine.hpp"
#include "event.hpp"
#include <stdio.h> // for printfs
#include "pico/util/queue.h"

//Maybe setup a seperate button as an object that triggers the state machine. 
// I take in the state machine as a parameter, and then it can call handle_state_transition. 
// I think I may be substantially overcomplicating this. 



#define GPIO_WATCH_PIN 2
void gpio_callback(uint gpio, uint32_t events);

volatile bool logging_state = false;
queue_t event_queue; // Create a queue to hold events
StateMachine state_machine; 

int64_t timer_callback(alarm_id_t id, void *user_data) {
    // Check the logging state and re-enable the intterupt
    if (!gpio_get(GPIO_WATCH_PIN)) { // Meaning, is the button still being pressed? (reading zero voltage)
        // Todo: Add the event to the queue and handle it in the main loop.
        Event event = Event::BUTTON_PRESSED;
        queue_try_add(&event_queue, &event); // Add the event to the queue
    }
    gpio_set_irq_enabled_with_callback(GPIO_WATCH_PIN, GPIO_IRQ_EDGE_FALL, true, gpio_callback); // Re-enable the interrupt
    
    return 0; // Return 0 to indicate that the alarm should not be repeated
}

void gpio_callback( uint gpio, uint32_t events) {
    gpio_set_irq_enabled_with_callback(GPIO_WATCH_PIN, GPIO_IRQ_EDGE_FALL, false, NULL); // First, disable the interrupt
    add_alarm_in_ms(50, &timer_callback, NULL, true); // Start a timer. 
}
int pico_led_init() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK; 
}
int main()
{
    // Initiialisation of the GPIO pin and the LED pin.
    // All this will be encapsulated into a button class later, but for now, we will just do it here.
    stdio_init_all();// Create an instance of the state machine
    queue_init(&event_queue, sizeof(Event), 10); // Initialize the queue to hold up to 10 events
    gpio_init(GPIO_WATCH_PIN);
    gpio_set_dir(GPIO_WATCH_PIN, GPIO_IN);
    gpio_pull_up(GPIO_WATCH_PIN); // The button will be active low, so we need to pull up the voltage. 
    gpio_set_irq_enabled_with_callback(GPIO_WATCH_PIN, GPIO_IRQ_EDGE_FALL, true, gpio_callback);
    int rc = pico_led_init(); 
    hard_assert (rc == PICO_OK);

    while (true) {
        // Check the queue for events if they are present, otherwise do nothing. 
        while (!queue_is_empty(&event_queue)) {
            Event event;
            if (queue_try_remove(&event_queue, &event)) {
                state_machine.handle_event(event); // Handle the event using the state machine
            }
        }
        __wfe(); 
    }
}
