#pragma once
#include "event.hpp"

// Control-side interface (core 0). The implementation chooses its workers.
class Process {
public:
    virtual ~Process() = default;
    virtual bool begin() = 0;
    virtual bool start() = 0;
    // True means the worker is quiescent, not that all writes succeeded.
    // Errors are reported separately. False means shutdown is still pending;
    // call stop() again to await the same completion, never start a new session.
    virtual bool stop() = 0;
    virtual bool is_running() const = 0;
    virtual void handle_event(Event event) = 0;
};
