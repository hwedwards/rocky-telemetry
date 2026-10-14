#pragma once
#include <thread>
inline void multicore_launch_core1(void (*entry)()) { std::thread(entry).detach(); }
