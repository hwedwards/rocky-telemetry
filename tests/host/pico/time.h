#pragma once
#include <chrono>
#include <thread>
#include <cstdint>
using absolute_time_t = std::chrono::steady_clock::time_point;
struct repeating_timer_t { void *user_data = nullptr; };
inline bool host_timer_ok = true;
inline bool add_repeating_timer_ms(int32_t, bool (*)(repeating_timer_t *), void *user, repeating_timer_t *timer) {
    timer->user_data = user; return host_timer_ok;
}
inline bool cancel_repeating_timer(repeating_timer_t *) { return true; }
// Accelerate only the control timeout: production 2000 ms becomes 20 ms.
inline absolute_time_t make_timeout_time_ms(uint32_t ms) {
    return std::chrono::steady_clock::now() + std::chrono::milliseconds(ms / 100);
}
inline bool time_reached(absolute_time_t t) { return std::chrono::steady_clock::now() >= t; }
inline void tight_loop_contents() { std::this_thread::yield(); }
