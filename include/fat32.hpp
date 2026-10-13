#pragma once

#include "sd_card.hpp"
#include <cstdint>
#include <cstddef>

// Just enough FAT32 to append to one file in the root directory.
//
// This is deliberately not a filesystem. There are no directories, no long
// names, no reading, no deleting, and exactly one file open at a time --
// because that is the entire shape of what Storage asks for. A general
// implementation is several thousand lines and brings a configuration surface
// with it; this is a few hundred and does nothing that is not on the path
// between Logger and the card.
//
// The card must already be formatted FAT32 (the default for anything from
// 4 GB up). Formatting one is a host-side job and does not belong on a rocket.
//
// ALLOCATION POLICY: clusters are claimed one at a time, as the file grows,
// rather than pre-allocated as a contiguous run at open time. Pre-allocation
// buys deterministic write latency, but it costs a guess about how long the
// flight is and it leaves the on-card size wrong until the file is closed --
// so a power cut mid-flight leaves a chkdsk error behind. Growing on demand
// costs one FAT sector write roughly every fifth buffer (32 KB clusters
// against 6 KB writes), which is a couple of milliseconds against a pool that
// absorbs two seconds. The card is consistent at every instant instead.
//
// THREADING: none. Everything after mount() runs on core 1. See storage.hpp.
class Fat32
{
    public:
        static constexpr size_t SECTOR_LEN = SdCard::BLOCK_LEN;

        explicit Fat32(SdCard &card) : card(card) {}

        // Finds the FAT32 volume -- either a bare one at LBA 0 or the first
        // FAT32 partition in the MBR -- and reads its geometry.
        bool mount();

        // Truncating open of a root-directory file. name is 8.3, case
        // insensitive: "log.bin" is fine, "flight_log.binary" is not.
        //
        // If the file already exists its cluster chain is released, so every
        // power-on starts a fresh log. That is the ordinary meaning of opening
        // for writing, but it does mean the previous flight is gone the moment
        // the board boots -- pull the card before powering up again.
        bool create(const char *name);

        // Append len bytes. len need not be a multiple of the sector size; a
        // partial tail is held in RAM until the next call fills it, or until
        // sync() commits it.
        bool append(const uint8_t *data, size_t len);

        // Push the partial sector, the FAT and the directory entry to the
        // card. After this returns true the file is intact on a reader, and
        // the board can lose power without losing what it has been given.
        bool sync();

        bool close();

        bool     is_mounted() const   { return mounted; }
        bool     is_open() const      { return file_open; }
        uint32_t file_size() const    { return size; }
        uint32_t clusters_used() const{ return clusters; }
        uint32_t cluster_bytes() const{ return (uint32_t)sec_per_clus * SECTOR_LEN; }

    private:
        // FAT32 reserves the top four bits of every entry; a chain ends at
        // anything from 0x0FFFFFF8 up.
        static constexpr uint32_t CHAIN_END  = 0x0FFFFFF8;
        static constexpr uint32_t ENTRY_MASK = 0x0FFFFFFF;

        // ---- little-endian field access -----------------------------------
        // FAT stores everything little-endian and so does Cortex-M, but the
        // fields are not naturally aligned inside a sector buffer. Assembling
        // them a byte at a time is both portable and obviously correct.
        static uint16_t rd16(const uint8_t *p);
        static uint32_t rd32(const uint8_t *p);
        static void     wr16(uint8_t *p, uint16_t v);
        static void     wr32(uint8_t *p, uint32_t v);

        static bool to_83(const char *name, char out[11]);

        bool parse_bpb(const uint8_t *sec, uint32_t vbr_lba);

        uint32_t lba_of(uint32_t cluster) const
        { return data_lba + (cluster - 2) * sec_per_clus; }

        // ---- FAT access, through a one-sector write-back cache -------------
        // An allocation scan touches the same FAT sector over and over, so
        // caching one turns a linear search into a memory walk.
        bool fat_load(uint32_t lba);
        bool fat_flush();                                  // to every FAT copy
        bool fat_get(uint32_t cluster, uint32_t &value);
        bool fat_set(uint32_t cluster, uint32_t value);

        bool alloc_cluster(uint32_t prev, uint32_t &out);
        bool free_chain(uint32_t start);
        bool invalidate_fsinfo();

        bool find_or_make_entry(const char name83[11]);
        bool update_dir_entry();

        // Make sure cur_cluster points at a cluster with room in it, claiming
        // a new one if the file has just started or the current one is full.
        bool ensure_cluster();

        SdCard &card;

        // ---- volume geometry ------------------------------------------------
        bool     mounted     = false;
        uint32_t vbr_lba     = 0;
        uint32_t fat_lba     = 0;   // first sector of FAT #0
        uint32_t fat_sectors = 0;   // length of one FAT
        uint32_t data_lba    = 0;   // sector of cluster 2
        uint32_t root_clus   = 0;
        uint32_t cluster_max = 0;   // highest valid cluster number
        uint32_t fsinfo_lba  = 0;   // 0 if the volume has none
        uint8_t  num_fats    = 0;
        uint8_t  sec_per_clus= 0;

        // ---- open file ------------------------------------------------------
        bool     file_open   = false;
        uint32_t first_clus  = 0;   // 0 until the first sector is written
        uint32_t cur_clus    = 0;
        uint32_t sec_in_clus = 0;
        uint32_t size        = 0;   // bytes handed to append(), including sect_fill
        uint32_t clusters    = 0;
        uint32_t dir_lba     = 0;   // where this file's 32-byte entry lives
        uint32_t dir_off     = 0;
        uint32_t next_free   = 2;   // where the next allocation scan starts

        // ---- buffers --------------------------------------------------------
        // 1.5 KB of .bss. Note that these are members rather than locals on
        // purpose: core 1's stack is 2 KB (PICO_CORE1_STACK_SIZE), so a
        // sector buffer on the stack of a function running there is a bad way
        // to find out about stack overflow.
        uint8_t  sect[SECTOR_LEN];      // partial sector waiting to be filled
        size_t   sect_fill   = 0;
        uint8_t  meta[SECTOR_LEN];      // scratch for BPB and directory sectors
        uint8_t  fatbuf[SECTOR_LEN];    // the cached FAT sector
        uint32_t fatbuf_lba  = 0xFFFFFFFF;
        bool     fatbuf_dirty= false;
};
