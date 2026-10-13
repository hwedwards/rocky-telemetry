#pragma once

#include "storage.hpp"
#include "sd_card.hpp"
#include "fat32.hpp"
#include "hardware/spi.h"

// The real backend: SD card over SPI, FAT32, one file.
//
// This class is almost entirely glue. All it adds over SdCard and Fat32 is the
// Storage contract and the counters behind report() -- and of those, the one
// worth reading is worst_write_us(). A logger that never drops a buffer is
// only telling you the pool was deep enough; the worst single write tells you
// how much of that depth you were actually using, and therefore whether the
// margin is real or luck.
//
// THREADING: mount() and open() run on core 0 at boot, before
// Logger::begin(). Every call after that comes from core 1. See storage.hpp.
class SdStorage : public Storage
{
    public:
        // Touches no hardware -- safe to construct as a global before main().
        //
        // cs_pin is driven as an ordinary GPIO, so it does not have to be the
        // pin the SPI block calls CSn. The other three do.
        SdStorage(spi_inst_t *bus, uint miso_pin, uint mosi_pin,
                  uint sck_pin, uint cs_pin)
            : card(bus, miso_pin, mosi_pin, sck_pin, cs_pin), fs(card) {}

        bool mount() override;
        bool open(const char *path) override;
        bool write(const uint8_t *data, size_t len) override;
        bool sync() override;
        void close() override;
        void report() const override;

        // For the boot-time block round-trip in main(). Borrowing the card
        // directly is only safe before Logger::begin() hands it to core 1.
        SdCard &raw_card() { return card; }

        uint32_t bytes_written() const  { return bytes; }
        uint32_t writes_done() const    { return writes; }
        uint32_t worst_write_us() const { return worst_us; }
        uint32_t stalls_seen() const    { return stalls; }

    private:
        // A write that takes longer than this is the card running an internal
        // garbage-collection pass rather than doing anything we asked for.
        // Counted separately because it is the thing the buffer pool exists to
        // absorb, and because a rising count is how a card tells you it is
        // wearing out.
        static constexpr uint32_t STALL_US = 100 * 1000;

        SdCard card;
        Fat32  fs;

        uint32_t bytes    = 0;
        uint32_t writes   = 0;
        uint32_t errors   = 0;
        uint32_t worst_us = 0;
        uint32_t stalls   = 0;
};
