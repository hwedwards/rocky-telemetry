#include "logger.hpp"
#include "event.hpp"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <stdio.h>

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
            bool synced = storage.sync();
            if (!synced) {
                write_errors++;
                Event e = Event::WRITE_ERROR;
                queue_try_add(&events, &e);
            }
            Completion done{write_errors, process_errors, synced};
            queue_add_blocking(&done_q, &done);
            continue;                   // the sentinel owns no buffer to return
        }

        if (!processor.process({b.data, b.len, format})) {
            process_errors++;
            Event e = Event::PROCESS_ERROR;
            queue_try_add(&events, &e);
        } else if (!storage.write(b.data, b.len)) {
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

    if (writer_self && writer_self != this) return false;
    format = source.format();
    if (!format.record_size || BUFFER_LEN % format.record_size != 0 ||
        !processor.accepts(format)) {
        printf("Logger: incompatible source format or processor\n");
        return false;
    }
    if (!queue_init(&full_q, sizeof(Block), BUFFER_COUNT + 1)) return false;
    if (!queue_init(&free_q, sizeof(Block), BUFFER_COUNT)) {
        queue_free(&full_q);
        return false;
    }
    if (!queue_init(&done_q, sizeof(Completion), 1)) {
        queue_free(&free_q);
        queue_free(&full_q);
        return false;
    }
    for (size_t i = 0; i < BUFFER_COUNT; i++) {
        Block b{buffers[i], 0};
        queue_add_blocking(&free_q, &b);
    }
    // Static trampoline binds the core 1 entry point to this instance.
    writer_self = this;
    // Launches core 1 and passes it a function pointer. 
    multicore_launch_core1(&writer_entry);
    writer_started = true;

    printf("Logger: writer running on core 1, %u buffers of %u bytes (%u records)\n",
           (unsigned)BUFFER_COUNT, (unsigned)BUFFER_LEN,
           (unsigned)(BUFFER_COUNT * BUFFER_LEN / format.record_size));
    return true;
}

bool Logger::start()
{
    if (stopping) return false;
    if (running) return true;
    if (!writer_started) {
        printf("Logger: begin() must run before start()\n");
        return false;
    }

    fill = 0;
    samples_logged = 0;
    overflows = 0;
    buffers_dropped = 0;
    // Worker error counters are lifetime totals, never reset across cores.

    if (!take_buffer()) {
        printf("Logger: no free buffer to start into\n");
        return false;
    }

    if (!source.start()) {
        printf("Logger: could not start the source\n");
        source.stop();
        queue_add_blocking(&free_q, &current);
        current.data = nullptr;
        return false;
    }

    // Negative period means the next callback is scheduled relative to the
    // start of the previous one, so the rate stays fixed no matter how long
    // the callback took.
    if (!add_repeating_timer_ms(-DRAIN_INTERVAL_MS, &drain_timer_cb, this, &timer)) {
        printf("Logger: no alarm slot available\n");
        source.stop();
        queue_add_blocking(&free_q, &current);
        current.data = nullptr;
        return false;
    }

    running = true;
    return true;
}

bool Logger::stop()
{
    if (!running && !stopping) return true;
    if (!stopping) {
        cancel_repeating_timer(&timer);
        running = false;
        stopping = true;
        // Freeze acquisition before the final drain. No new samples accumulate
        // while the writer catches up. If starved, retain the previous policy:
        // drain() leaves unread records in the source; no retry/drop policy change.
        source_stopped = source.stop();
        drain();
        flush();
        if (current.data) {
            queue_add_blocking(&free_q, &current);
            current.data = nullptr;
            fill = 0;
        }
        Block sync_cmd{nullptr, 0};
        queue_add_blocking(&full_q, &sync_cmd);
    }
    absolute_time_t deadline = make_timeout_time_ms(STOP_TIMEOUT_MS);
    while (!queue_try_remove(&done_q, &completed)) {
        if (time_reached(deadline)) {
            printf("Logger: shutdown pending; press again to await completion\n");
            return false; // No Outputs reads, resets, or new session while busy.
        }
        tight_loop_contents();
    }
    // The queue acknowledgement synchronizes all worker accesses. All buffers
    // were recycled before sync. The writer now blocks on an empty full_q.
    stopping = false;
    if (!source_stopped) printf("Logger: source stop failed\n");
    if (!completed.synced) printf("Logger: final sync failed; data may be incomplete\n");
    report();
    return true;
}

void Logger::handle_event(Event event)
{
    switch (event) {
    case Event::DRAIN_FIFO: if (running) drain(); break;
    case Event::FIFO_OVERFLOW: if (running) on_overflow(); break;
    case Event::WRITE_ERROR: printf("Logger: storage operation failed\n"); break;
    case Event::PROCESS_ERROR: printf("Logger: processor rejected a batch\n"); break;
    default: break;
    }
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
    // Also called once by stop() after acquisition has been frozen.
    bool overflowed = false;
    if (source.overflowed(overflowed) && overflowed) {
        Event e = Event::FIFO_OVERFLOW;
        queue_try_add(&events, &e);
        return;             // the FIFO contents are already corrupt; let the
                            // state machine decide what to do about it
    }

    size_t waiting = 0;
    if (!source.available(waiting)) return;

    // The count can be read part-way through the sensor writing a record, so
    // only ever take whole records and leave any remainder for next time.
    // Popping a partial record would shift every record after it.
    waiting -= waiting % format.record_size;

    while (waiting > 0) {
        if (!take_buffer()) {
            // Every buffer is out with the writer. Leave the records in the
            // source: its capacity sets the remaining headroom. The stream stays
            // record-aligned. If the writer never catches up the FIFO
            // overflows and on_overflow() resets it -- a clean break in the
            // log rather than a silently corrupted one.
            return;
        }

        size_t space = BUFFER_LEN - fill;
        size_t chunk = waiting < space ? waiting : space;

        if (!source.read(current.data + fill, chunk)) return;

        fill += chunk;
        waiting -= chunk;
        samples_logged += chunk / format.record_size;

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
    source.reset();

    printf("Logger: FIFO overflow (#%lu) - the write path is not keeping up\n",
           (unsigned long)overflows);
}

void Logger::report() const
{
    printf("Logger: %lu samples, %lu overflows, %lu write errors (lifetime), "
           "%lu drains with no free buffer, %lu processor errors (lifetime)\n",
           (unsigned long)samples_logged,
           (unsigned long)overflows,
           (unsigned long)completed.write_errors,
           (unsigned long)buffers_dropped,
           (unsigned long)completed.process_errors);
    storage.report();
}
