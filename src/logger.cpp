#include "logger.hpp"
#include "event.hpp"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <stdio.h>

Logger *Logger::writer_self = nullptr;

// Interrupt context, core 0. The one thing that is safe here is posting to the
// queue -- no I2C, no printf, no storage. queue_try_add ends in an __sev(),
// which is what wakes the main loop out of its blocking remove.
bool Logger::drain_timer_cb(repeating_timer_t *rt)
{
    Logger *self = static_cast<Logger *>(rt->user_data);
    Event e = Event::DRAIN_FIFO;
    queue_try_add(&self->events, &e);
    return true;    // keep repeating
}

// ---- core 1 ----------------------------------------------------------------

void Logger::writer_entry()
{
    writer_self->writer_loop();
}

// The whole of core 1. It blocks on full_q -- queue_remove_blocking parks on
// __wfe() -- so an idle writer costs nothing, and a card that stalls for a
// quarter of a second stalls only here.
//
// Deliberately no printf: stdio is mutex-guarded so it would not corrupt
// anything, but blocking the write path on USB CDC is a silly way to lose
// samples. Failures go back to core 0 as an event instead.
void Logger::writer_loop()
{
    while (true) {
        Block b;
        queue_remove_blocking(&full_q, &b);

        if (b.data == nullptr) {        // sync sentinel, see stop()
            if (!storage.sync()) {
                write_errors++;
                Event e = Event::WRITE_ERROR;
                queue_try_add(&events, &e);
            }
            continue;                   // the sentinel owns no buffer to return
        }

        if (!storage.write(b.data, b.len)) {
            write_errors++;
            Event e = Event::WRITE_ERROR;
            queue_try_add(&events, &e);
        }

        // Ownership goes back to core 0 only now, once the write has finished
        // with the bytes.
        b.len = 0;
        queue_add_blocking(&free_q, &b);
    }
}

// ---- core 0 ----------------------------------------------------------------

bool Logger::begin()
{
    if (writer_started) return true;

    // full_q gets one extra slot so the sync sentinel in stop() always has
    // somewhere to go, even when every buffer is already queued.
    queue_init(&full_q, sizeof(Block), BUFFER_COUNT + 1);
    queue_init(&free_q, sizeof(Block), BUFFER_COUNT);

    for (size_t i = 0; i < BUFFER_COUNT; i++) {
        Block b{buffers[i], 0};
        queue_add_blocking(&free_q, &b);
    }

    writer_self = this;
    multicore_launch_core1(&writer_entry);
    writer_started = true;

    printf("Logger: writer running on core 1, %u buffers of %u bytes (%u ms)\n",
           (unsigned)BUFFER_COUNT, (unsigned)BUFFER_LEN,
           (unsigned)(BUFFER_COUNT * BUFFER_SAMPLES));
    return true;
}

bool Logger::start()
{
    if (running) return true;
    if (!writer_started) {
        printf("Logger: begin() must run before start()\n");
        return false;
    }

    fill = 0;
    samples_logged = 0;
    overflows = 0;
    buffers_dropped = 0;
    write_errors = 0;

    if (!take_buffer()) {
        printf("Logger: no free buffer to start into\n");
        return false;
    }

    if (!imu.fifo_enable(true)) {
        printf("Logger: could not start the sensor FIFO\n");
        return false;
    }

    // Negative period means the next callback is scheduled relative to the
    // start of the previous one, so the rate stays fixed no matter how long
    // the callback took.
    if (!add_repeating_timer_ms(-DRAIN_INTERVAL_MS, &drain_timer_cb, this, &timer)) {
        printf("Logger: no alarm slot available\n");
        imu.fifo_enable(false);
        return false;
    }

    running = true;
    return true;
}

void Logger::stop()
{
    if (!running) return;

    cancel_repeating_timer(&timer);
    running = false;        // set before the last drain so flush() is the final word

    drain();                // whatever landed since the last timer tick
    flush();                // partial buffer

    // flush() returns early on an empty buffer, so core 0 can still be holding
    // one here. Give it back, or the wait below can never be satisfied.
    if (current.data) {
        queue_add_blocking(&free_q, &current);
        current.data = nullptr;
        fill = 0;
    }

    Block sync_cmd{nullptr, 0};
    queue_add_blocking(&full_q, &sync_cmd);

    // A buffer only reaches free_q after its write has completed, so all of
    // them being home means the card has everything we handed over. The
    // sentinel was queued behind the last block, so the sync is done too.
    absolute_time_t deadline = make_timeout_time_ms(STOP_TIMEOUT_MS);
    while (queue_get_level(&free_q) < BUFFER_COUNT) {
        if (time_reached(deadline)) {
            printf("Logger: writer still busy after %u ms - giving up on the "
                   "final flush\n", (unsigned)STOP_TIMEOUT_MS);
            break;
        }
        tight_loop_contents();
    }

    imu.fifo_enable(false);
    report();
}

// Claim a buffer to fill. Non-blocking on purpose: if core 1 is behind and
// holds every buffer, core 0 must not wait for it -- that would reintroduce
// exactly the coupling the two-core split exists to remove.
bool Logger::take_buffer()
{
    if (current.data) return true;

    Block b;
    if (!queue_try_remove(&free_q, &b)) {
        buffers_dropped++;
        return false;
    }

    current = b;
    fill = 0;
    return true;
}

void Logger::drain()
{
    // stop() calls this once after clearing the flag, so gate on the timer
    // being gone rather than on running.
    bool overflowed = false;
    if (imu.fifo_overflowed(overflowed) && overflowed) {
        Event e = Event::FIFO_OVERFLOW;
        queue_try_add(&events, &e);
        return;             // the FIFO contents are already corrupt; let the
                            // state machine decide what to do about it
    }

    uint16_t waiting = 0;
    if (!imu.fifo_count(waiting)) return;

    // The count can be read part-way through the sensor writing a record, so
    // only ever take whole records and leave any remainder for next time.
    // Popping a partial record would shift every record after it.
    waiting -= waiting % IMU::FIFO_SAMPLE_LEN;

    while (waiting > 0) {
        if (!take_buffer()) {
            // Every buffer is out with the writer. Leave the records in the
            // FIFO: it holds 85 ms, and more importantly the stream stays
            // record-aligned. If the writer never catches up the FIFO
            // overflows and on_overflow() resets it -- a clean break in the
            // log rather than a silently corrupted one.
            return;
        }

        size_t space = BUFFER_LEN - fill;
        size_t chunk = waiting < space ? waiting : space;

        if (!imu.fifo_read(current.data + fill, chunk)) return;

        fill += chunk;
        waiting -= chunk;
        samples_logged += chunk / IMU::FIFO_SAMPLE_LEN;

        if (fill == BUFFER_LEN) flush();
    }
}

// Hand the filling buffer to core 1 and give up ownership of it. Never blocks:
// full_q has a slot for every buffer plus the sentinel, so the add always fits.
void Logger::flush()
{
    if (current.data == nullptr || fill == 0) return;

    current.len = fill;
    queue_add_blocking(&full_q, &current);

    current.data = nullptr;     // core 1 owns those bytes now
    fill = 0;
}

void Logger::on_overflow()
{
    overflows++;

    // Everything still in the FIFO is of unknown age now, so throw it away
    // rather than write interleaved garbage into the log.
    imu.fifo_reset();

    printf("Logger: FIFO overflow (#%lu) - the write path is not keeping up\n",
           (unsigned long)overflows);
}

void Logger::on_write_error()
{
    printf("Logger: storage write failed (#%lu)\n",
           (unsigned long)write_errors);
}

void Logger::report() const
{
    printf("Logger: %lu samples, %lu overflows, %lu write errors, "
           "%lu drains with no free buffer\n",
           (unsigned long)samples_logged,
           (unsigned long)overflows,
           (unsigned long)write_errors,
           (unsigned long)buffers_dropped);
    storage.report();
}
