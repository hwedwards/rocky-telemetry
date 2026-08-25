#pragma once

#include <cstddef>
#include <cstdint>

class IMU;      // only for NullStorage's debug dump. Storage proper has no
                // opinion about what is in the bytes it is handed.

// Where log data ends up. Logger only ever sees this interface, so the SD
// card backend can land later without the acquisition path changing.
//
// write() is synchronous by contract: when it returns, the caller may reuse
// the buffer. A DMA-driven implementation still honours that -- it starts the
// transfer and sleeps on __wfe() until the completion IRQ, rather than
// spinning on the SPI FIFO.
//
// THREADING: mount() and open() run on core 0 at boot. From the moment
// Logger::begin() launches the writer, every other call comes from core 1 and
// core 0 must not touch this object again. That ordering is what lets the
// implementation stay single-threaded -- FatFs is not reentrant.
class Storage
{
    public:
        virtual ~Storage() = default;

        virtual bool mount() = 0;
        virtual bool open(const char *path) = 0;
        virtual bool write(const uint8_t *data, size_t len) = 0;
        virtual bool sync() = 0;    // flush metadata; safe to power off after
        virtual void close() = 0;

        // Backend statistics, printed by Logger::report(). Safe to read from
        // core 0 because stop() has already waited for the writer to go idle.
        virtual void report() const {}
};

// Counts bytes and throws them away. Lets the whole acquisition path be
// exercised before any SD hardware is wired up.
class NullStorage : public Storage
{
    public:
        // ---- fault injection ------------------------------------------------
        // Left to itself this backend returns instantly, so core 1 never falls
        // behind, free_q never empties, and the starvation path in
        // Logger::take_buffer() never runs -- the entire reason for the
        // two-core split goes untested and every counter reads a comfortable
        // zero. These knobs make it behave like a real card instead.
        struct Delays {
            uint32_t typical_ms  = 0;   // cost of an ordinary write
            uint32_t stall_ms    = 0;   // cost of a write that hits card GC
            uint32_t stall_every = 0;   // one write in this many stalls; 0 = never
        };

        void set_delays(const Delays &d) { delays = d; }

        // Decode and print one record out of every nth buffer, so the byte
        // layout gets checked and not just the byte count -- a sample count
        // cannot catch a wrong offset, but a gyro reading of 8000 deg/s can.
        // Pass 0 to disable.
        //
        // The IMU is borrowed only for decode(). Its gyro bias is fixed during
        // calibrate() at boot, well before core 1 exists, so reading it from
        // the writer core needs no synchronisation.
        void set_dump(uint32_t every_n_buffers, const IMU *imu);

        bool mount() override { return true; }
        bool open(const char *path) override;
        bool write(const uint8_t *data, size_t len) override;
        bool sync() override { return true; }
        void close() override;

        uint32_t bytes_written() const { return bytes; }
        uint32_t writes_done() const { return writes; }
        uint32_t stalls_injected() const { return stalls; }

        void report() const override;

    private:
        uint32_t bytes  = 0;
        uint32_t writes = 0;
        uint32_t stalls = 0;

        Delays delays{};
        uint32_t dump_n = 0;
        const IMU *dump_imu = nullptr;
};
