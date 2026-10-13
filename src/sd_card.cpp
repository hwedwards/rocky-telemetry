#include "sd_card.hpp"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>

// SPI-mode command set. Only the ones this driver actually sends.
namespace {
    constexpr uint8_t GO_IDLE_STATE     = 0;    // CMD0  -- enter SPI mode
    constexpr uint8_t SEND_OP_COND_MMC  = 1;    // CMD1  -- the MMC form of ACMD41
    constexpr uint8_t SEND_IF_COND      = 8;    // CMD8  -- version 2 probe
    constexpr uint8_t SEND_CSD          = 9;    // CMD9  -- capacity
    constexpr uint8_t STOP_TRANSMISSION = 12;   // CMD12 -- end a CMD18 stream
    constexpr uint8_t SET_BLOCKLEN      = 16;   // CMD16 -- SDSC only
    constexpr uint8_t READ_SINGLE       = 17;   // CMD17
    constexpr uint8_t READ_MULTIPLE     = 18;   // CMD18
    constexpr uint8_t SET_WR_ERASE_CNT  = 23;   // ACMD23 -- pre-erase hint
    constexpr uint8_t WRITE_SINGLE      = 24;   // CMD24
    constexpr uint8_t WRITE_MULTIPLE    = 25;   // CMD25
    constexpr uint8_t SD_SEND_OP_COND   = 41;   // ACMD41 -- the init loop
    constexpr uint8_t APP_CMD           = 55;   // CMD55 -- "the next one is an ACMD"
    constexpr uint8_t READ_OCR          = 58;   // CMD58 -- where the CCS bit lives

    // Tokens that prefix a data packet on the wire.
    constexpr uint8_t TOK_START_BLOCK   = 0xFE; // single read/write, and the CSD
    constexpr uint8_t TOK_START_MULTI   = 0xFC; // one block of a CMD25 stream
    constexpr uint8_t TOK_STOP_TRAN     = 0xFD; // end of a CMD25 stream
}

// ---- SPI plumbing ----------------------------------------------------------

uint8_t SdCard::xchg(uint8_t v)
{
    uint8_t r;
    spi_write_read_blocking(bus, &v, &r, 1);
    return r;
}

// Reading means clocking out 0xFF and keeping whatever comes back.
// spi_read_blocking does that with a single repeated fill byte, so there is no
// need for a scratch transmit buffer the size of a sector.
void SdCard::rx(uint8_t *dst, size_t len) { spi_read_blocking(bus, 0xFF, dst, len); }
void SdCard::tx(const uint8_t *src, size_t len) { spi_write_blocking(bus, src, len); }

void SdCard::select()
{
    gpio_put(cs_pin, 0);
    xchg(0xFF);         // one byte of settling before the command frame
}

void SdCard::deselect()
{
    gpio_put(cs_pin, 1);
    // The card needs eight more clocks after CS rises before it lets go of DO.
    // Skip this and the next command reads the tail of the last response.
    xchg(0xFF);
}

// A card that is programming a block holds DO low, and there is no status
// register to ask instead. Polling is the mechanism here, not a workaround.
bool SdCard::wait_ready(uint32_t timeout_ms)
{
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    do {
        if (xchg(0xFF) == 0xFF) return true;
    } while (!time_reached(deadline));
    return false;
}

// Returns the R1 response byte, or 0xFF if the card never answered.
//
// Leaves the card SELECTED on the way out. Several commands are followed by
// more bytes -- R3 and R7 trailers, data blocks -- so closing the frame is the
// caller's job, not this function's.
uint8_t SdCard::send_cmd(uint8_t cmd, uint32_t arg)
{
    // CMD12 lands in the middle of a read stream. The bus is already ours and
    // the card is mid-transfer, so re-selecting it would break the stream.
    if (cmd != STOP_TRANSMISSION) {
        deselect();
        select();
        if (!wait_ready(500)) { timeout_errs++; return 0xFF; }
    }

    uint8_t frame[6];
    frame[0] = (uint8_t)(0x40 | cmd);
    frame[1] = (uint8_t)(arg >> 24);
    frame[2] = (uint8_t)(arg >> 16);
    frame[3] = (uint8_t)(arg >> 8);
    frame[4] = (uint8_t)arg;

    // CRC7 is ignored in SPI mode -- except on the two commands that are sent
    // before the card has been told it is in SPI mode. Both have fixed
    // arguments, so their CRCs are constants rather than a function call.
    frame[5] = (cmd == GO_IDLE_STATE) ? 0x95
             : (cmd == SEND_IF_COND)  ? 0x87
                                      : 0x01;
    tx(frame, sizeof frame);

    if (cmd == STOP_TRANSMISSION) xchg(0xFF);   // stuff byte, discarded

    // The response arrives within eight bytes; poll a little past that.
    for (int i = 0; i < 10; i++) {
        uint8_t r = xchg(0xFF);
        if ((r & 0x80) == 0) return r;          // R1 always has bit 7 clear
    }
    timeout_errs++;
    return 0xFF;
}

uint8_t SdCard::send_acmd(uint8_t cmd, uint32_t arg)
{
    uint8_t r = send_cmd(APP_CMD, 0);
    if (r > 1) return r;        // 0 = ready, 1 = idle; anything else is a fault
    return send_cmd(cmd, arg);
}

// ---- data packets ----------------------------------------------------------

bool SdCard::rx_datablock(uint8_t *dst, size_t len)
{
    // 200 ms is well past the spec's read access time. A card that has not
    // produced a token by now is not going to.
    absolute_time_t deadline = make_timeout_time_ms(200);
    uint8_t token;
    do {
        token = xchg(0xFF);
        if (token != 0xFF) break;
    } while (!time_reached(deadline));

    if (token == 0xFF) { timeout_errs++; return false; }
    if (token != TOK_START_BLOCK) { proto_errs++; return false; }   // error token

    rx(dst, len);
    xchg(0xFF);     // CRC16, discarded -- SPI mode leaves the data CRC off
    xchg(0xFF);
    return true;
}

// src is ignored for TOK_STOP_TRAN, which carries no payload.
bool SdCard::tx_datablock(const uint8_t *src, uint8_t token)
{
    if (!wait_ready(500)) { timeout_errs++; return false; }

    xchg(token);
    if (token == TOK_STOP_TRAN) return true;

    tx(src, BLOCK_LEN);
    xchg(0xFF);     // dummy CRC16
    xchg(0xFF);

    // The card answers a data packet at once, before it starts programming:
    // xxx0sss1, where sss is 010 accepted, 101 CRC error, 110 write error.
    uint8_t resp = (uint8_t)(xchg(0xFF) & 0x1F);
    if (resp != 0x05) { proto_errs++; return false; }
    return true;
}

// ---- public ----------------------------------------------------------------

bool SdCard::init()
{
    ready = false; ccs = false; v2 = false; sectors = 0;

    spi_init(bus, INIT_BAUD);
    spi_set_format(bus, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(sck_pin,  GPIO_FUNC_SPI);

    // The breakout should carry a pull-up on DO, but the card tri-states it
    // whenever CS is high, and a floating input reads as plausible-looking
    // command responses rather than as an obvious fault. This costs nothing.
    gpio_pull_up(miso_pin);

    // CS is an ordinary GPIO on purpose. The SPI peripheral's hardware chip
    // select deasserts between bytes, and every SD command needs it held low
    // across the whole frame-plus-response exchange.
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);

    // Power-up: at least 74 clocks with CS and DI held high. This is what
    // moves the card out of its native SD mode and makes it listen for CMD0.
    sleep_ms(2);
    for (int i = 0; i < 10; i++) xchg(0xFF);

    bool idle = false;
    for (int i = 0; i < 100 && !idle; i++) {
        if (send_cmd(GO_IDLE_STATE, 0) == 0x01) idle = true;
        else sleep_ms(10);
    }
    if (!idle) {
        deselect();
        printf("SD: no answer to CMD0 - check wiring, power and CS\n");
        return false;
    }

    // ACMD41 is entitled to report "still busy" for up to a second on a cold
    // card, so the whole negotiation shares one generous deadline.
    absolute_time_t deadline = make_timeout_time_ms(1500);

    if (send_cmd(SEND_IF_COND, 0x1AA) == 0x01) {
        uint8_t r7[4];
        rx(r7, sizeof r7);                      // R7 trailer
        if (r7[2] != 0x01 || r7[3] != 0xAA) {
            deselect();
            printf("SD: card rejected 2.7-3.6 V operation\n");
            return false;
        }
        v2 = true;

        // HCS set: tell the card we understand block addressing.
        while (send_acmd(SD_SEND_OP_COND, 0x40000000) != 0) {
            if (time_reached(deadline)) {
                deselect();
                printf("SD: card never left the idle state (ACMD41 timed out)\n");
                return false;
            }
        }

        if (send_cmd(READ_OCR, 0) != 0) {
            deselect();
            printf("SD: CMD58 failed - cannot tell block from byte addressing\n");
            return false;
        }
        uint8_t ocr[4];
        rx(ocr, sizeof ocr);
        ccs = (ocr[0] & 0x40) != 0;             // CCS bit: high capacity
    } else {
        // Version 1 SD, or an MMC. Which one is settled by whether ACMD41 is a
        // legal command; an MMC answers with the illegal-command bit set.
        bool is_sd = (send_acmd(SD_SEND_OP_COND, 0) <= 1);

        while (true) {
            uint8_t r = is_sd ? send_acmd(SD_SEND_OP_COND, 0)
                              : send_cmd(SEND_OP_COND_MMC, 0);
            if (r == 0) break;
            if (time_reached(deadline)) {
                deselect();
                printf("SD: legacy card never left the idle state\n");
                return false;
            }
        }
        ccs = false;                            // v1 and MMC are byte-addressed
    }

    // Byte-addressed cards need telling what a block is. Block-addressed ones
    // have it fixed at 512 and will reject the command.
    if (!ccs && send_cmd(SET_BLOCKLEN, BLOCK_LEN) != 0) {
        deselect();
        printf("SD: could not set a 512-byte block length\n");
        return false;
    }

    deselect();

    // The init dance is over, so stop paying for it.
    spi_set_baudrate(bus, RUN_BAUD);
    ready = true;

    read_csd();

    printf("SD: %s card, %s addressing, %lu MB, bus at %lu kHz\n",
           v2 ? "v2+" : "v1/MMC",
           ccs ? "block" : "byte",
           (unsigned long)(sectors / 2048),
           (unsigned long)(spi_get_baudrate(bus) / 1000));
    return true;
}

bool SdCard::read_csd()
{
    if (send_cmd(SEND_CSD, 0) != 0) { deselect(); return false; }

    uint8_t csd[16];
    bool ok = rx_datablock(csd, sizeof csd);
    deselect();
    if (!ok) return false;

    if ((csd[0] >> 6) == 1) {
        // CSD version 2.0: capacity is simply (C_SIZE + 1) * 512 KB.
        uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16) |
                          ((uint32_t)csd[8] << 8) | (uint32_t)csd[9];
        sectors = (c_size + 1) * 1024;
    } else {
        // CSD version 1.0: the older three-field encoding.
        uint32_t c_size = ((uint32_t)(csd[6] & 0x03) << 10) |
                          ((uint32_t)csd[7] << 2) | (uint32_t)(csd[8] >> 6);
        uint32_t mult   = (uint32_t)(((csd[9] & 0x03) << 1) | (csd[10] >> 7));
        uint32_t rdblk  = (uint32_t)(csd[5] & 0x0F);
        sectors = (c_size + 1) << (mult + 2 + rdblk - 9);
    }
    return true;
}

bool SdCard::read_blocks(uint32_t lba, uint8_t *dst, size_t count)
{
    if (!ready || count == 0) return false;

    // One command for the whole run rather than one per block.
    uint8_t cmd = (count > 1) ? READ_MULTIPLE : READ_SINGLE;
    if (send_cmd(cmd, addr_of(lba)) != 0) { deselect(); return false; }

    bool ok = true;
    for (size_t i = 0; i < count && ok; i++) {
        ok = rx_datablock(dst + i * BLOCK_LEN, BLOCK_LEN);
    }

    // The stream has to be closed even when a block failed, or the card stays
    // in read mode and the next command lands inside the data stream.
    if (cmd == READ_MULTIPLE) send_cmd(STOP_TRANSMISSION, 0);

    deselect();
    return ok;
}

bool SdCard::write_blocks(uint32_t lba, const uint8_t *src, size_t count)
{
    if (!ready || count == 0) return false;

    bool ok;
    if (count == 1) {
        if (send_cmd(WRITE_SINGLE, addr_of(lba)) != 0) { deselect(); return false; }
        ok = tx_datablock(src, TOK_START_BLOCK);
    } else {
        // Telling the card how many blocks are coming lets it pre-erase them in
        // one go. That is the difference between a real multi-block write and
        // n single writes in a trenchcoat. Not fatal if the card refuses.
        send_acmd(SET_WR_ERASE_CNT, (uint32_t)count);

        if (send_cmd(WRITE_MULTIPLE, addr_of(lba)) != 0) { deselect(); return false; }

        ok = true;
        for (size_t i = 0; i < count && ok; i++) {
            ok = tx_datablock(src + i * BLOCK_LEN, TOK_START_MULTI);
        }
        if (ok) ok = tx_datablock(nullptr, TOK_STOP_TRAN);
    }

    // Do not return until the card has finished programming. Storage promises
    // the caller may reuse the buffer on return, and core 1 is about to hand
    // that buffer straight back to core 0.
    //
    // This is also where a garbage-collection stall shows up: the card can sit
    // here for a couple of hundred milliseconds. That is precisely the latency
    // the two-core split exists to keep away from the drain path.
    if (!wait_ready(1000)) { timeout_errs++; ok = false; }

    deselect();
    return ok;
}

bool SdCard::self_test(uint32_t lba)
{
    if (!ready) {
        printf("SD self-test: card not initialised\n");
        return false;
    }

    uint8_t out[BLOCK_LEN];
    uint8_t back[BLOCK_LEN];
    for (size_t i = 0; i < BLOCK_LEN; i++) out[i] = (uint8_t)(i * 7 + 0x5A);

    if (!write_blocks(lba, out, 1)) {
        printf("SD self-test: write to block %lu failed\n", (unsigned long)lba);
        return false;
    }

    memset(back, 0, sizeof back);
    if (!read_blocks(lba, back, 1)) {
        printf("SD self-test: read back of block %lu failed\n", (unsigned long)lba);
        return false;
    }

    for (size_t i = 0; i < BLOCK_LEN; i++) {
        if (back[i] != out[i]) {
            // The first mismatching byte says more than pass or fail does.
            // All zeroes or all 0xFF means the data never arrived at all,
            // whereas a single flipped bit points at clock speed or wire length.
            printf("SD self-test: block %lu differs at byte %u "
                   "(wrote 0x%02X, read 0x%02X)\n",
                   (unsigned long)lba, (unsigned)i, out[i], back[i]);
            return false;
        }
    }

    printf("SD self-test: block %lu round-tripped cleanly\n", (unsigned long)lba);
    return true;
}
