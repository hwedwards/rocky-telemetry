#include "state_machine.hpp"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

void StateMachine::enter_logging()
{
    current_state = State::LOGGING;
    sample_count = 0;
    next_sample = get_absolute_time();      // first sample on the next poll()
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    printf("Transitioning to LOGGING state\n");
}

void StateMachine::enter_off()
{
    current_state = State::OFF;
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    printf("Transitioning to OFF state (%lu samples)\n",
           (unsigned long)sample_count);
}

void StateMachine::handle_event(Event event)
{
    switch (current_state) {
        case State::OFF:
            if (event == Event::BUTTON_PRESSED) {
                enter_logging();
            }
            break;

        case State::LOGGING:
            if (event == Event::BUTTON_PRESSED) {
                enter_off();
            }
            break;
    }
}

void StateMachine::poll()
{
    if (current_state != State::LOGGING) return;

    // Positive means next_sample is still in the future, so there is nothing
    // to do yet. Comparing absolute times this way is wrap-safe.
    if (absolute_time_diff_us(get_absolute_time(), next_sample) > 0) return;
    next_sample = delayed_by_us(next_sample, SAMPLE_INTERVAL_US);

    IMU::Sample s;
    if (!imu.read(s)) {
        printf("IMU read failed\n");
        return;
    }

    sample_count++;

    // Placeholder sink. This is where the DMA buffer / SD card write goes;
    // printing every sample at 100 Hz would swamp the USB CDC port.
    if (sample_count % PRINT_EVERY == 0) {
        printf("Acc %.3f %.3f %.3f g | Gyr %.2f %.2f %.2f d/s | %.1f C\n",
               IMU::to_g(s.accel[0]), IMU::to_g(s.accel[1]), IMU::to_g(s.accel[2]),
               IMU::to_dps(s.gyro[0]), IMU::to_dps(s.gyro[1]), IMU::to_dps(s.gyro[2]),
               IMU::to_celsius(s.temp));
    }
}
