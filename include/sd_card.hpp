#pragma once

#include "hardware/spi.h"
#include <cstdint>
#include <cstddef>

// A micro SD card driven in SPI mode: 512-byte blocks in, 512-byte blocks out.
// Knows nothing about files -- Fat32 sits on top of this.
//
// SPI mode is the slow way to talk to an SD card: one data line instead of
// four, no command queueing, and a protocol that spends bytes on polling. It
// costs one peripheral and four pins, though, and this device produces 12 KB/s
// against a bus that carries over 1 MB/s once init() has raised the clock.
// A hundred-fold margin is not the place to spend effort, so there is no DMA
// here and no reason to reach for SDIO or PIO.
//
// THREADING: none, deliberately. init() runs on core 0 at boot, before
// Logger::begin() exists to hand anything over; every call after that arrives
// from core 1 through Storage. Nothing here is reentrant.
class SdCard
{
    public:
        static constexpr size_t BLOCK_LEN = 512;

        // Clock for the initialisation dance. The spec requires 100-400 kHz
        // until the card reports ready -- it will not answer CMD0 at speed.
        static constexpr uint32_t INIT_BAUD = 400 * 1000;

        // Clock for everything after. The SD "default speed" ceiling is
        // 25 MHz, but that assumes a PCB. On jumper wires to a breakout,
        // 12.5 MHz is the honest number; raise it only once the wiring is
        // short and self_test() still passes.
        static constexpr uint32_t RUN_BAUD = 12500 * 1000;

        // Touches no hardware -- safe to construct as a global before main().
        //
        // cs_pin is driven as an ordinary GPIO, NOT as the SPI peripheral's
        // chip select. Hardware CS deasserts between bytes, and an SD card
        // needs it held low across a whole command-plus-response exchange.
        SdCard(spi_inst_t *bus, uint miso_pin, uint mosi_pin,
               uint sck_pin, uint cs_pin)
            : bus(bus), miso_pin(miso_pin), mosi_pin(mosi_pin),
              sck_pin(sck_pin), cs_pin(cs_pin) {}

        // Brings up the pins and walks the card through the SPI-mode power-up
        // sequence, finishing at RUN_BAUD. Can take up to a second on a cold
        // card -- ACMD41 is allowed to say "still busy" for that long.
        bool init();

        // count blocks starting at lba. Uses CMD18/CMD25 for count > 1, which
        // turns n command round-trips into one.
        bool read_blocks(uint32_t lba, uint8_t *dst, size_t count);
        bool write_blocks(uint32_t lba, const uint8_t *src, size_t count);

        bool is_ready() const { return ready; }

        // Capacity in 512-byte blocks, from the CSD. Zero if unknown.
        uint32_t sector_count() const { return sectors; }

        // True for SDHC/SDXC, where command arguments are block numbers.
        // False for standard-capacity cards, where they are byte offsets.
        // Getting this backwards is the classic "my writes land in the wrong
        // place" bug, so it is latched once from the OCR and never guessed.
        bool is_high_capacity() const { return ccs; }

        // Destructive read-write-verify against one block. Point it at a
        // sector you do not mind losing -- well past the filesystem, or on a
        // card you are happy to reformat. This is the tool for separating an
        // electrical problem from a filesystem one: if this fails, nothing
        // above it can work.
        bool self_test(uint32_t lba);

        // Counters for Storage::report().
        uint32_t protocol_errors() const { return proto_errs; }
        uint32_t timeouts() const        { return timeout_errs; }

    private:
        // ---- SPI plumbing --------------------------------------------------
        void select();
        void deselect();
        uint8_t xchg(uint8_t tx);
        void rx(uint8_t *dst, size_t len);
        void tx(const uint8_t *src, size_t len);

        // Poll DO until the card releases it (reads 0xFF). A card that is
        // programming a block holds it low, and there is no other way to ask.
        bool wait_ready(uint32_t timeout_ms);

        // Returns the R1 byte, or 0xFF if the card never answered.
        uint8_t send_cmd(uint8_t cmd, uint32_t arg);
        uint8_t send_acmd(uint8_t cmd, uint32_t arg);   // CMD55 then cmd

        bool rx_datablock(uint8_t *dst, size_t len);
        bool tx_datablock(const uint8_t *src, uint8_t token);

        bool read_csd();

        // Command arguments are block numbers on SDHC/SDXC and byte offsets
        // on SDSC. One place decides which.
        uint32_t addr_of(uint32_t lba) const { return ccs ? lba : lba * BLOCK_LEN; }

        spi_inst_t *bus;
        uint miso_pin, mosi_pin, sck_pin, cs_pin;

        bool     ready    = false;
        bool     ccs      = false;      // high capacity: address in blocks
        bool     v2       = false;      // answered CMD8
        uint32_t sectors  = 0;
        uint32_t proto_errs   = 0;
        uint32_t timeout_errs = 0;
};
