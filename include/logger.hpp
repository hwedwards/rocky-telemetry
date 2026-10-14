#pragma once
#include "process.hpp"
#include "inputs.hpp"
#include "sample_processor.hpp"
#include "outputs.hpp"
#include "pico/time.h"
#include "pico/util/queue.h"

// Concrete two-core pipeline. Dependencies and this object must outlive core 1.
// Core 0: source and control. Core 1: processor then storage.
// Pool bytes never move; queue entries transfer exclusive buffer ownership.
class Logger final : public Process {
public:
    static constexpr size_t BUFFER_LEN = 6144;
    static constexpr size_t BUFFER_COUNT = 4;
    static constexpr int32_t DRAIN_INTERVAL_MS = 50;
    static constexpr uint32_t STOP_TIMEOUT_MS = 2000;
    static_assert(BUFFER_LEN % 512 == 0, "whole SD sectors");
    struct Block { uint8_t *data; size_t len; };
    Logger(Inputs &source, SampleProcessor &processor, Outputs &storage, queue_t &events)
        : source(source), processor(processor), storage(storage), events(events) {}
    Logger(const Logger &) = delete;
    Logger &operator=(const Logger &) = delete;
    bool begin() override;
    bool start() override;
    bool stop() override;
    bool is_running() const override { return running; }
    void handle_event(Event event) override;
private:
    void drain();
    void flush();
    bool take_buffer();
    void on_overflow();
    void report() const;
    static bool drain_timer_cb(repeating_timer_t *rt);
    static void writer_entry();
    void writer_loop();
    Inputs &source;
    SampleProcessor &processor;
    Outputs &storage;
    queue_t &events;
    SampleFormat format{SampleKind::Opaque, 0};
    // Core 0 only.
    repeating_timer_t timer{};
    bool running = false;
    bool writer_started = false;
    bool stopping = false;
    bool source_stopped = true;
    Block current{nullptr, 0};
    size_t fill = 0;
    uint32_t samples_logged = 0;
    uint32_t overflows = 0;
    uint32_t buffers_dropped = 0;
    struct Completion { uint32_t write_errors; uint32_t process_errors; bool synced; };
    Completion completed{}; // Snapshot received by core 0 through done_q.
    // Core 1 only, cumulative across sessions.
    uint32_t write_errors = 0;
    uint32_t process_errors = 0;
    queue_t full_q{}; // core 0 -> core 1; nullptr is ordered sync command
    queue_t free_q{}; // core 1 -> core 0; reuse only after process/write returns
    queue_t done_q{}; // core 1 -> core 0; one outstanding stop acknowledgement
    uint8_t buffers[BUFFER_COUNT][BUFFER_LEN];
    inline static Logger *writer_self = nullptr;
};
