#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include <stdio.h> // for printfs


// First goal is to get the state machine working on button press. CHECK 
// Second, goal is to turn the LED on and OFF based on the state machine. 
// Third goal is to get the cpu to sleep until the interrupt is triggered.

#define GPIO_WATCH_PIN 2
void gpio_callback(uint gpio, uint32_t events);

volatile bool logging_state = false;
int64_t timer_callback(alarm_id_t id, void *user_data) {
    // Check the logging state and re-enable the intterupt
    if (!gpio_get(GPIO_WATCH_PIN)) { // Meaning, is the button still being pressed? (reading zero voltage)
        logging_state = !logging_state; // Toggle the logging state
        printf("Logging state changed: %s\n", logging_state ? "ON" : "OFF");
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
    stdio_init_all();
    gpio_init(GPIO_WATCH_PIN);
    gpio_set_dir(GPIO_WATCH_PIN, GPIO_IN);
    gpio_pull_up(GPIO_WATCH_PIN); // The button will be active low, so we need to pull up the voltage. 
    gpio_set_irq_enabled_with_callback(GPIO_WATCH_PIN, GPIO_IRQ_EDGE_FALL, true, gpio_callback);
    int rc = pico_led_init(); 
    hard_assert (rc == PICO_OK);
    while (true) {
        if (logging_state) {
            // Do logging stuff here
            gpio_put(PICO_DEFAULT_LED_PIN, 1); // Turn on the LED when logging is active
            printf("Logging is active...\n");
        } else {
            gpio_put(PICO_DEFAULT_LED_PIN, 0); // Turn off the LED when logging is not active
            printf("Logging is not active.\n");
        }
        // Note to self: Should probably include a watchdog timer. 
        __wfe(); 
    }
}
