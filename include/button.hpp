#pragma once
#include "event_source.hpp"
#include "pico/stdlib.h"
#include "pico/util/queue.h"

// One GPIO callback owner for this application. Must outlive the IRQ/alarm.
class Button final : public EventSource {
public:
    Button(uint pin, queue_t &events) : pin(pin), events(events) {}
    Button(const Button &) = delete;
    Button &operator=(const Button &) = delete;
    bool begin() override;
private:
    static void irq(uint gpio, uint32_t flags);
    static int64_t debounce(alarm_id_t id, void *context);
    uint pin;
    queue_t &events;
    inline static Button *active = nullptr;
};
