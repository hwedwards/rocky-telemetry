#include "logger.hpp"
#include "state_machine.hpp"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>
#include <functional>

static void await(const std::function<bool()> &predicate) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!predicate()) {
        assert(std::chrono::steady_clock::now() < deadline);
        std::this_thread::yield();
    }
}
struct Source : Inputs {
    SampleFormat layout{SampleKind::AccelGyroBe16, 12};
    std::vector<uint8_t> data;
    size_t pos = 0;
    bool start_ok = true, active = false, overflow = false;
    unsigned resets = 0;
    SampleFormat format() const override { return layout; }
    bool start() override { active = start_ok; return start_ok; }
    bool stop() override { active = false; return true; }
    bool available(size_t &bytes) override { bytes = data.size() - pos; return true; }
    bool read(uint8_t *dst, size_t len) override {
        assert(len % layout.record_size == 0 && pos + len <= data.size());
        std::copy_n(data.data() + pos, len, dst); pos += len; return true;
    }
    bool overflowed(bool &value) override { value = overflow; overflow = false; return true; }
    bool reset() override { ++resets; pos = data.size(); return true; }
    void feed(size_t n) {
        assert(pos == data.size()); data.resize(n); pos = 0;
        for (size_t i = 0; i < n; ++i) data[i] = uint8_t(i * 37 + 11);
    }
};
struct TestSampleProcessor : SampleProcessor {
    PassThroughProcessor pass;
    std::atomic<bool> reject{false};
    bool compatible = true;
    std::atomic<unsigned> calls{0};
    bool accepts(SampleFormat f) const override { return compatible && f.kind == SampleKind::AccelGyroBe16 && f.record_size == 12; }
    bool process(SampleBatch batch) override {
        ++calls;
        assert(batch.size % 12 == 0 && batch.size <= Logger::BUFFER_LEN);
        return !reject && pass.process(batch);
    }
};
struct Sink : Outputs {
    std::vector<uint8_t> output; // Read by core 0 only after acknowledgement.
    std::atomic<bool> hold_write{false}, hold_sync{false}, busy{false}, syncing{false};
    std::atomic<bool> fail_write{false}, fail_sync{false};
    std::atomic<unsigned> writes{0}, syncs{0};
    mutable unsigned reports = 0;
    bool mount() override { return true; }
    bool open(const char *) override { return true; }
    bool write(const uint8_t *data, size_t size) override {
        std::vector<uint8_t> before(data, data + size);
        busy = true;
        while (hold_write) std::this_thread::yield();
        // Detect premature recycling or mutation while the sink owns the bytes.
        assert(std::equal(before.begin(), before.end(), data));
        if (!fail_write) output.insert(output.end(), data, data + size);
        busy = false; ++writes; return !fail_write;
    }
    bool sync() override {
        syncing = true;
        while (hold_sync) std::this_thread::yield();
        syncing = false; ++syncs; return !fail_sync;
    }
    void close() override {}
    void report() const override { assert(!busy && !syncing); ++reports; }
};
static bool event_present(queue_t &events, Event wanted) {
    Event e; bool found = false;
    while (queue_try_remove(&events, &e)) if (e == wanted) found = true;
    return found;
}
int main() {
    // Deliberately process-lifetime fixtures: the firmware worker never exits.
    auto &source = *new Source;
    auto &process = *new TestSampleProcessor;
    auto &sink = *new Sink;
    auto &events = *new queue_t;
    queue_init(&events, sizeof(Event), 64);
    auto &logger = *new Logger(source, process, sink, events);
    source.layout.record_size = 0; assert(!logger.begin());
    source.layout.record_size = 13; assert(!logger.begin());
    source.layout.record_size = 12; process.compatible = false; assert(!logger.begin());
    process.compatible = true; assert(logger.begin()); assert(logger.begin());

    // Failed starts must not consume pool entries.
    source.start_ok = false;
    for (int i = 0; i < 6; ++i) assert(!logger.start());
    source.start_ok = true; host_timer_ok = false;
    for (int i = 0; i < 6; ++i) assert(!logger.start());
    host_timer_ok = true;

    // Empty stop returns the held empty buffer, syncs exactly once, then restarts.
    for (int i = 0; i < 6; ++i) { assert(logger.start()); assert(logger.stop()); }
    assert(sink.writes == 0 && sink.syncs == 6);

    // Full blocks plus a partial tail: byte-for-byte identical default output.
    assert(logger.start()); source.feed(Logger::BUFFER_LEN + 24);
    logger.handle_event(Event::DRAIN_FIFO); assert(logger.stop());
    assert(sink.output == source.data); assert(sink.writes == 2);
    auto output_size = sink.output.size();

    // SampleProcessor failure skips the sink, returns the buffer, and reports an event.
    process.reject = true;
    assert(logger.start()); source.feed(12); assert(logger.stop());
    assert(sink.output.size() == output_size && sink.writes == 2);
    assert(event_present(events, Event::PROCESS_ERROR)); process.reject = false;

    // A held first write consumes all four buffers; the fifth remains in source.
    sink.hold_write = true;
    assert(logger.start()); source.feed(Logger::BUFFER_LEN * 5);
    logger.handle_event(Event::DRAIN_FIFO); await([&]{ return sink.busy.load(); });
    assert(source.pos == Logger::BUFFER_LEN * 4);
    logger.handle_event(Event::DRAIN_FIFO);
    assert(source.pos == Logger::BUFFER_LEN * 4);
    sink.hold_write = false; await([&]{ return sink.writes == 6; });
    assert(logger.stop());
    assert(sink.output.size() == output_size + source.data.size());
    assert(std::equal(source.data.begin(), source.data.end(), sink.output.begin() + output_size));

    // Preserve overflow handling: event first, reset on dispatch, keep running.
    assert(logger.start()); source.feed(12); source.overflow = true;
    logger.handle_event(Event::DRAIN_FIFO);
    assert(source.pos == 0 && source.resets == 0);
    assert(event_present(events, Event::FIFO_OVERFLOW));
    logger.handle_event(Event::FIFO_OVERFLOW); assert(source.resets == 1);
    assert(logger.stop());

    // Stop timeout: no report or restart; subsequent stop awaits the same sentinel.
    auto &machine = *new StateMachine(logger);
    sink.hold_write = true;
    machine.handle_event(Event::BUTTON_PRESSED); source.feed(Logger::BUFFER_LEN);
    machine.handle_event(Event::DRAIN_FIFO); await([&]{ return sink.busy.load(); });
    auto reports_before = sink.reports;
    auto syncs_before = sink.syncs.load();
    machine.handle_event(Event::BUTTON_PRESSED);
    assert(machine.get_current_state() == State::STOPPING);
    assert(!logger.start() && sink.reports == reports_before && !source.active);
    sink.hold_sync = true; sink.hold_write = false;
    await([&]{ return sink.syncing.load(); });
    assert(!logger.stop() && sink.reports == reports_before);
    sink.hold_sync = false;
    await([&]{ return sink.syncs == syncs_before + 1; });
    machine.handle_event(Event::BUTTON_PRESSED);
    assert(machine.get_current_state() == State::OFF && sink.reports == reports_before + 1);
    assert(sink.syncs == syncs_before + 1);
    machine.handle_event(Event::BUTTON_PRESSED);
    assert(machine.get_current_state() == State::LOGGING);
    machine.handle_event(Event::BUTTON_PRESSED);
    assert(machine.get_current_state() == State::OFF);

    // Write/sync failures are acknowledged, reported, and do not strand buffers.
    sink.fail_write = true; sink.fail_sync = true;
    assert(logger.start()); source.feed(12); assert(logger.stop());
    assert(event_present(events, Event::WRITE_ERROR));
    sink.fail_write = false; sink.fail_sync = false;
    assert(logger.start()); source.feed(12); assert(logger.stop());
    std::puts("PASS: format validation, default bytes, pool ownership, processor failure, overflow, start failures, empty/full/partial stops, write/sync timeouts, restart and sink errors");
    std::fflush(stdout);
    std::_Exit(0); // Avoid destructing live worker fixtures on host process exit.
}
