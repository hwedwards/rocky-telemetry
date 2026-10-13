#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "button.hpp"
#include "state_machine.hpp"
#include "logger.hpp"
#include "storage.hpp"
#include "sd_storage.hpp"
#include "imu.hpp"
#include "event.hpp"
#include <stdio.h> // for printfs
#include "pico/util/queue.h"

#define GPIO_WATCH_PIN 2
#define IMU_SDA_PIN 0
#define IMU_SCL_PIN 1

// Micro SD module on SPI1. GP12/14/15 are the SPI1 RX/SCK/TX pins; CS is a
// plain GPIO rather than the block's CSn, because hardware chip select
// deasserts between bytes and an SD command has to hold it low for the whole
// frame. See sd_card.cpp.
#define SD_MISO_PIN 12
#define SD_CS_PIN   13
#define SD_SCK_PIN  14
#define SD_MOSI_PIN 15

void gpio_callback(uint gpio, uint32_t events);

queue_t event_queue; // Everything reaches the state machine through here

// Declaration order matters: globals in one translation unit are constructed
// top to bottom, so each of these exists before the next one binds a reference
// to it. None of the constructors touch hardware -- that happens in main().
IMU imu(i2c_default, IMU_SDA_PIN, IMU_SCL_PIN);
SdStorage storage(spi1, SD_MISO_PIN, SD_MOSI_PIN, SD_SCK_PIN, SD_CS_PIN);
Logger logger(imu, storage, event_queue);
StateMachine state_machine(logger);

// Make the bus pins available to picotool
bi_decl(bi_2pins_with_func(IMU_SDA_PIN, IMU_SCL_PIN, GPIO_FUNC_I2C));
bi_decl(bi_3pins_with_func(SD_MISO_PIN, SD_MOSI_PIN, SD_SCK_PIN, GPIO_FUNC_SPI));
bi_decl(bi_1pin_with_name(SD_CS_PIN, "SD card chip select"));

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

    if (!storage.mount()) {
        printf("SD card unavailable - logging will discard data\n");
    }

    // DEBUG: a destructive round-trip against one block, to separate an
    // electrical fault from a filesystem one. If this fails then nothing
    // above it can work and the FAT32 messages are not worth reading.
    // Point it at a block you do not mind losing -- 0 is the MBR, so what
    // you write there costs you a reformat.
    //
    // storage.raw_card().self_test(0);

    // Opening is hundreds of milliseconds on an SD card, which is exactly
    // why it belongs at boot rather than in Logger::start().
    if (!storage.open("log.bin")) {
        printf("Could not open log.bin - logging will discard data\n");
    }


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
