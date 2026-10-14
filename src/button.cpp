#include "button.hpp"
#include "event.hpp"

bool Button::begin()
{
    if (active && active != this) return false;
    active = this;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_FALL, true, irq);
    return true;
}
void Button::irq(uint gpio, uint32_t)
{
    if (!active || gpio != active->pin) return;
    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_FALL, false);
    if (add_alarm_in_ms(50, debounce, active, true) < 0)
        gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_FALL, true);
}
int64_t Button::debounce(alarm_id_t, void *context)
{
    auto &self = *static_cast<Button *>(context);
    if (!gpio_get(self.pin)) {
        Event event = Event::BUTTON_PRESSED;
        queue_try_add(&self.events, &event);
    }
    gpio_set_irq_enabled(self.pin, GPIO_IRQ_EDGE_FALL, true);
    return 0;
}
