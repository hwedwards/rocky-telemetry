#pragma once
inline void gpio_put(unsigned, bool) {}

#define GPIO_FUNC_I2C 3
inline void gpio_set_function(unsigned, unsigned) {}
inline void gpio_pull_up(unsigned) {}
