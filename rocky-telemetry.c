#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include <stdio.h> // for printfs


// First goal is to get the state machine working on button press. CHECK 
// Second, goal is to turn the LED on and OFF based on the state machine. 
// Third goal is to get the cpu to sleep until the interrupt is triggered.

#define GPIO_WATCH_PIN 2

volatile bool logging_state = false;

void gpio_callback( uint gpio, uint32_t events) {
    gpio_set_irq_enabled_with_callback(GPIO_WATCH_PIN, GPIO_IRQ_EDGE_RISE, false, NULL);
    // Toggle the logging state on button press
    // In order to debouce the button, I should disable the interrupt, 
    // Start a timer, and if the button is still being pressed, 
    // change the state and re-enable the interrupt.
    // The problem is, that I should not being called such a function from within the interrupt. 
    logging_state = !logging_state;
    // global variables can be corrupted by interrupts (non rentrant functions)
    // printf("Logging state changed: %s\n", logging_state ? "ON" : "OFF");
}
int pico_led_init() {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK; 
}
int main()
{
    // I want to figure out how to do this in a way that is not pinning the cpu at 100%. 
    // Actually I want to be dormant until the interrupt is triggered. 
    stdio_init_all();
    gpio_init(GPIO_WATCH_PIN);
    gpio_set_dir(GPIO_WATCH_PIN, GPIO_IN);
    gpio_pull_up(GPIO_WATCH_PIN); // The button will be active low, so we need to pull up the voltage. 
    gpio_set_irq_enabled_with_callback(GPIO_WATCH_PIN, GPIO_IRQ_EDGE_RISE, true, gpio_callback);
    int rc = pico_led_init(); 
    hard_assert (rc == PICO_OK);
    // I want some logic here that will put the cpu to sleep until the interrupt 
    // is fired and the state changes to logging= true as opposed to polling constantly. 
    while (true) {
        gpio_put(PICO_DEFAULT_LED_PIN, logging_state);
        printf("Logging state: %s\n", logging_state ? "ON" : "OFF");
        sleep_ms(1000); // Sleep for one second to slow down the cpu. 
    }
}
