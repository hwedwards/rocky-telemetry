#pragma once

#include "imu.hpp"
#include "storage.hpp"
#include "pico/time.h"
#include "pico/util/queue.h"
#include <cstdint>
#include <cstddef>

// Owns the acquisition path: sensor FIFO -> RAM buffer pool -> Storage.
//
// The work is split across both cores, and the split is by how *bounded* each
// part's latency is, not by how much CPU it costs:
//
//   core 0   timer -> drain() -> fill a buffer -> hand it to core 1
//            Longest operation is the I2C burst read at ~13.5 ms. Bounded.
//            Shares the core with the state machine, whose work is measured
//            in microseconds, so the two sit together happily.
//
//   core 1   writer_loop() -> Storage::write() -> hand the buffer back
//            An SD write is usually ~2 ms but can stall for a quarter of a
//            second when the card runs an internal garbage-collection pass.
//            Unbounded, so it gets a core to itself where it cannot delay a
//            drain or hold up a button press.
//
// Buffers cross between the cores through two queues and are never shared:
// a buffer is owned by exactly one core at any instant, which is what makes
// this safe without any lock beyond the spinlock queue_t already holds.
//
//   full_q   core 0 -> core 1    filled, ready to write
//   free_q   core 1 -> core 0    written, safe to refill
//
// Nothing here runs a sampling loop. start() arms a repeating timer and
// returns in microseconds; the only code in interrupt context is one
// queue_try_add.
class Logger
{
    public:
        // 512 records per buffer -- 6144 bytes at 12 bytes a record. That is
        // 512 ms of flight, and exactly twelve 512-byte sectors, so every
        // handoff is a whole number of blocks landing on a sector boundary.
        // That alignment matters: a write FatFs cannot pass straight through
        // becomes a read-modify-write through its single-sector window.
        static constexpr size_t BUFFER_SAMPLES = 512;
        static constexpr size_t BUFFER_LEN = BUFFER_SAMPLES * IMU::FIFO_SAMPLE_LEN;

        // Four of them: 24 KB out of the 520 KB on the chip, and ~2 s of
        // writer stall absorbed before core 0 runs out of somewhere to put
        // samples. RAM is not the constraint here, so take the headroom.
        static constexpr size_t BUFFER_COUNT = 4;

        static_assert(BUFFER_LEN % 512 == 0,
                      "buffer must be a whole number of SD sectors");

        // 50 ms between drains: ~50 records (600 bytes) each, against 85 ms of
        // FIFO capacity. The margin is what absorbs a late drain.
        static constexpr int32_t DRAIN_INTERVAL_MS = 50;

        // How long stop() waits for core 1 to clear its backlog and sync.
        static constexpr uint32_t STOP_TIMEOUT_MS = 2000;

        // One handoff between the cores. data == nullptr is the sync sentinel:
        // it asks the writer to flush filesystem metadata, and unlike a real
        // block it does not come back through free_q.
        struct Block {
            uint8_t *data;
            size_t   len;
        };

        Logger(IMU &imu, Storage &storage, queue_t &events)
            : imu(imu), storage(storage), events(events) {}

        // Builds the buffer pool and launches core 1. Storage must already be
        // mounted and open: from the moment this returns, Storage belongs to
        // core 1 and core 0 must not touch it again.
        bool begin();

        // Arms the timer and clears the FIFO. Does NOT open the file -- that is
        // hundreds of milliseconds on an SD card and belongs at boot.
        // Returns false if the sensor would not start.
        bool start();

        // Disarms the timer, drains what is left, hands over the partial
        // buffer and waits for core 1 to finish writing it. Blocking, but
        // only on the way out.
        void stop();

        // Called from the state machine on Event::DRAIN_FIFO.
        void drain();

        // Called from the state machine on Event::FIFO_OVERFLOW.
        void on_overflow();

        // Called from the state machine on Event::WRITE_ERROR. The failure
        // happened on core 1; this only reports it, in main context, where
        // printf is not sitting in front of the write path.
        void on_write_error();

        bool is_running() const { return running; }
        void report() const;

    private:
        void flush();                   // hand the filling buffer to core 1
        bool take_buffer();             // claim a free buffer, false if starved
        static bool drain_timer_cb(repeating_timer_t *rt);   // IRQ context, core 0

        static void writer_entry();     // core 1 trampoline
        void writer_loop();             // core 1, never returns

        IMU &imu;
        Storage &storage;
        queue_t &events;

        // ---- core 0 only ---------------------------------------------------
        repeating_timer_t timer{};
        bool running = false;
        bool writer_started = false;
        Block current{nullptr, 0};      // buffer being filled; null if starved
        size_t fill = 0;                // bytes used in current
        uint32_t samples_logged = 0;
        uint32_t overflows = 0;
        uint32_t buffers_dropped = 0;

        // ---- written by core 1, read by core 0 -----------------------------
        // Single writer, single reader, naturally aligned 32-bit: the load
        // cannot tear on Cortex-M33. volatile only stops the compiler from
        // caching it across the report() call.
        volatile uint32_t write_errors = 0;

        // ---- shared --------------------------------------------------------
        queue_t full_q{};
        queue_t free_q{};
        uint8_t buffers[BUFFER_COUNT][BUFFER_LEN];

        static Logger *writer_self;     // core 1 has no user-data argument
};
