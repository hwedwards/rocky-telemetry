#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "button.hpp"
#include "state_machine.hpp"
#include "logger.hpp"
#include "storage.hpp"
#include "imu.hpp"
#include "event.hpp"
#include <stdio.h> // for printfs
#include "pico/util/queue.h"

#define GPIO_WATCH_PIN 2
#define IMU_SDA_PIN 0
#define IMU_SCL_PIN 1

void gpio_callback(uint gpio, uint32_t events);

queue_t event_queue; // Everything reaches the state machine through here

// Declaration order matters: globals in one translation unit are constructed
// top to bottom, so each of these exists before the next one binds a reference
// to it. None of the constructors touch hardware -- that happens in main().
IMU imu(i2c_default, IMU_SDA_PIN, IMU_SCL_PIN);
NullStorage storage;                            // swap for the SD backend later
Logger logger(imu, storage, event_queue);
StateMachine state_machine(logger);

// Make the I2C pins available to picotool
bi_decl(bi_2pins_with_func(IMU_SDA_PIN, IMU_SCL_PIN, GPIO_FUNC_I2C));

int64_t timer_callback(alarm_id_t id, void *user_data) {
    // Check the logging state and re-enable the intterupt
    if (!gpio_get(GPIO_WATCH_PIN)) { // Meaning, is the button still being pressed? (reading zero voltage)
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
    stdio_init_all();
    sleep_ms(2000);   // let the USB CDC port enumerate before the first print

    queue_init(&event_queue, sizeof(Event), 16); // Initialize the queue to hold up to 16 events

    int rc = pico_led_init();
    hard_assert (rc == PICO_OK);

    // Everything slow happens here, before the button can generate an event.
    // Mounting the card and opening the file is hundreds of milliseconds, which
    // is why it lives at boot and not in Logger::start().
    if (imu.init()) {
        imu.calibrate(500);
    } else {
        printf("Continuing without a working IMU\n");
    }

    if (!storage.mount() || !storage.open("log.bin")) {
        printf("Storage unavailable - logging will discard data\n");
    }

    // DEBUG: make the null backend behave like an SD card, so the two-core
    // handoff is actually exercised rather than trivially satisfied.
    // 3 ms per ordinary write, and a 250 ms garbage-collection stall every
    // 8th write (~every 4 s at one buffer per 512 ms).
    //
    // The pool holds BUFFER_COUNT * 512 ms = ~2 s, so a 250 ms stall should
    // be absorbed with buffers_dropped still 0. To prove the failure path is
    // real and not silent, raise stall_ms past 2000 -- you should then see
    // dropped buffers and, if it persists, a FIFO overflow. Delete this whole
    // block once the SD backend lands.
    storage.set_delays({3, 250, 8});
    storage.set_dump(4, &imu);      // decode one record every 4th buffer


    // Hands Storage over to core 1 and starts the writer. Nothing on core 0
    // may touch Storage after this point.
    logger.begin();

    // Button last: from here on, a press can arrive at any moment.
    // All this will be encapsulated into a button class later, but for now, we will just do it here.
    gpio_init(GPIO_WATCH_PIN);
    gpio_set_dir(GPIO_WATCH_PIN, GPIO_IN);
    gpio_pull_up(GPIO_WATCH_PIN); // The button will be active low, so we need to pull up the voltage. 
    gpio_set_irq_enabled_with_callback(GPIO_WATCH_PIN, GPIO_IRQ_EDGE_FALL, true, gpio_callback);

    printf("Ready. Press the button to start logging.\n");

    // A pure event pump. queue_remove_blocking parks on __wfe() when there is
    // nothing to do, and the queue's own __sev() on add is what wakes it.
    while (true) {
        Event event;
        queue_remove_blocking(&event_queue, &event);
        state_machine.handle_event(event);
    }
}
