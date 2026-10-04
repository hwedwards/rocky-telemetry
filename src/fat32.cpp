#include "fat32.hpp"
#include <stdio.h>
#include <string.h>

namespace {
    // There is no RTC on this board, so every file gets the same stamp. A
    // wrong-but-constant date beats a random one: nothing here depends on
    // wall-clock time, and a host listing sorts by name anyway.
    constexpr uint16_t FIXED_DATE = (46 << 9) | (1 << 5) | 1;   // 2026-01-01
    constexpr uint16_t FIXED_TIME = 0;                          // midnight

    constexpr uint8_t  ATTR_VOLUME_ID = 0x08;
    constexpr uint8_t  ATTR_LONG_NAME = 0x0F;
    constexpr uint8_t  ATTR_ARCHIVE   = 0x20;

    constexpr size_t   DIR_ENTRY_LEN  = 32;
    constexpr uint32_t NO_SECTOR      = 0xFFFFFFFF;
}

// ---- little-endian field access --------------------------------------------

uint16_t Fat32::rd16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

uint32_t Fat32::rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void Fat32::wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

void Fat32::wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

// ---- mount -----------------------------------------------------------------

bool Fat32::parse_bpb(const uint8_t *sec, uint32_t base)
{
    uint16_t byts    = rd16(sec + 0x0B);
    uint8_t  spc     = sec[0x0D];
    uint16_t rsvd    = rd16(sec + 0x0E);
    uint8_t  nfats   = sec[0x10];
    uint16_t rootent = rd16(sec + 0x11);
    uint16_t fatsz16 = rd16(sec + 0x16);
    uint32_t totsec  = rd32(sec + 0x20);
    uint32_t fatsz32 = rd32(sec + 0x24);
    uint32_t rootcl  = rd32(sec + 0x2C);
    uint16_t fsinfo  = rd16(sec + 0x30);

    if (byts != SECTOR_LEN) return false;               // 512-byte sectors only
    if (spc == 0 || (spc & (spc - 1)) != 0) return false;   // must be a power of two
    if (nfats == 0 || rsvd == 0) return false;

    // What makes it FAT32 rather than FAT16: no fixed-size root directory,
    // and the FAT length living in the 32-bit field instead of the 16-bit one.
    if (rootent != 0 || fatsz16 != 0 || fatsz32 == 0) return false;
    if (totsec == 0) return false;

    uint32_t sys = rsvd + (uint32_t)nfats * fatsz32;
    if (sys >= totsec) return false;

    uint32_t clus = (totsec - sys) / spc;
    if (clus < 65525) return false;         // below this the volume is FAT16

    vbr_lba      = base;
    fat_lba      = base + rsvd;
    fat_sectors  = fatsz32;
    num_fats     = nfats;
    sec_per_clus = spc;
    data_lba     = base + sys;
    root_clus    = rootcl;
    cluster_max  = clus + 1;                // clusters are numbered from 2
    fsinfo_lba   = fsinfo ? (base + fsinfo) : 0;
    next_free    = 2;
    fatbuf_lba   = NO_SECTOR;
    fatbuf_dirty = false;
    return true;
}

bool Fat32::mount()
{
    mounted = false;

    if (!card.read_blocks(0, meta, 1)) {
        printf("FAT32: could not read sector 0\n");
        return false;
    }
    if (rd16(meta + 510) != 0xAA55) {
        printf("FAT32: no boot signature in sector 0 - is the card formatted?\n");
        return false;
    }

    // Sector 0 is either the volume's own boot record or an MBR carrying a
    // partition table. A real BPB opens with a jump instruction and declares a
    // sane sector size, which is enough to tell the two apart.
    if ((meta[0] == 0xEB || meta[0] == 0xE9) && rd16(meta + 0x0B) == SECTOR_LEN) {
        if (parse_bpb(meta, 0)) {
            mounted = true;
            printf("FAT32: unpartitioned volume, %u sectors per cluster\n",
                   (unsigned)sec_per_clus);
            return true;
        }
    }

    // Otherwise walk the four primary partitions. Copy the table out first --
    // reading a candidate boot sector overwrites meta.
    uint32_t starts[4];
    uint8_t  types[4];
    for (int i = 0; i < 4; i++) {
        types[i]  = meta[446 + 16 * i + 4];
        starts[i] = rd32(meta + 446 + 16 * i + 8);
    }

    for (int i = 0; i < 4; i++) {
        if (types[i] == 0 || starts[i] == 0) continue;
        if (!card.read_blocks(starts[i], meta, 1)) continue;
        if (rd16(meta + 510) != 0xAA55) continue;
        if (!parse_bpb(meta, starts[i])) continue;

        mounted = true;
        printf("FAT32: partition %d at LBA %lu, %u sectors per cluster\n",
               i + 1, (unsigned long)starts[i], (unsigned)sec_per_clus);
        return true;
    }

    printf("FAT32: no FAT32 volume found - reformat the card as FAT32\n");
    return false;
}

// ---- the FAT ---------------------------------------------------------------

bool Fat32::fat_load(uint32_t lba)
{
    if (fatbuf_lba == lba) return true;
    if (!fat_flush()) return false;
    if (!card.read_blocks(lba, fatbuf, 1)) return false;
    fatbuf_lba = lba;
    return true;
}

bool Fat32::fat_flush()
{
    if (!fatbuf_dirty || fatbuf_lba == NO_SECTOR) {
        fatbuf_dirty = false;
        return true;
    }

    // Every copy, not just the first. Cards are normally formatted with two,
    // and a host is entitled to believe either one of them.
    for (uint8_t i = 0; i < num_fats; i++) {
        if (!card.write_blocks(fatbuf_lba + (uint32_t)i * fat_sectors, fatbuf, 1))
            return false;
    }
    fatbuf_dirty = false;
    return true;
}

bool Fat32::fat_get(uint32_t cluster, uint32_t &value)
{
    if (cluster < 2 || cluster > cluster_max) return false;

    uint32_t off = cluster * 4;
    if (!fat_load(fat_lba + off / SECTOR_LEN)) return false;

    value = rd32(fatbuf + (off % SECTOR_LEN)) & ENTRY_MASK;
    return true;
}

bool Fat32::fat_set(uint32_t cluster, uint32_t value)
{
    if (cluster < 2 || cluster > cluster_max) return false;

    uint32_t off = cluster * 4;
    if (!fat_load(fat_lba + off / SECTOR_LEN)) return false;

    uint8_t *p = fatbuf + (off % SECTOR_LEN);
    // The top four bits of a FAT32 entry are reserved. They belong to whoever
    // formatted the card, so carry them through untouched.
    wr32(p, (rd32(p) & ~ENTRY_MASK) | (value & ENTRY_MASK));
    fatbuf_dirty = true;
    return true;
}

// Claim one free cluster, link it onto prev (pass 0 to start a new chain) and
// mark it as the end. The scan starts where the last one finished, so on a
// freshly formatted card this is O(1) rather than a walk from cluster 2.
bool Fat32::alloc_cluster(uint32_t prev, uint32_t &out)
{
    uint32_t total = cluster_max - 1;       // clusters 2 .. cluster_max
    uint32_t c = (next_free < 2 || next_free > cluster_max) ? 2 : next_free;

    for (uint32_t scanned = 0; scanned < total; scanned++) {
        uint32_t v;
        if (!fat_get(c, v)) return false;

        if (v == 0) {
            if (!fat_set(c, CHAIN_END)) return false;
            if (prev != 0 && !fat_set(prev, c)) return false;
            next_free = (c < cluster_max) ? c + 1 : 2;
            clusters++;
            out = c;
            return true;
        }
        c = (c < cluster_max) ? c + 1 : 2;
    }

    printf("FAT32: volume is full\n");
    return false;
}

bool Fat32::free_chain(uint32_t start)
{
    uint32_t c = start;
    // A corrupt chain must not turn into an infinite loop. There cannot be
    // more links than there are clusters.
    uint32_t guard = cluster_max + 1;

    while (c >= 2 && c <= cluster_max && c < CHAIN_END && guard-- > 0) {
        uint32_t nxt;
        if (!fat_get(c, nxt)) return false;
        if (!fat_set(c, 0)) return false;
        if (c < next_free) next_free = c;
        c = nxt;
    }
    return true;
}

// Tell the host its cached free-space number is stale rather than leaving a
// wrong one behind. Best effort: a bad FSInfo is cosmetic, so a failure here
// is not worth failing the open over.
bool Fat32::invalidate_fsinfo()
{
    if (fsinfo_lba == 0) return true;
    if (!card.read_blocks(fsinfo_lba, meta, 1)) return false;

    if (rd32(meta) != 0x41615252 || rd32(meta + 484) != 0x61417272) return true;

    wr32(meta + 488, 0xFFFFFFFF);   // free cluster count: unknown
    wr32(meta + 492, 0xFFFFFFFF);   // next free hint: unknown
    return card.write_blocks(fsinfo_lba, meta, 1);
}

// ---- the directory ---------------------------------------------------------

bool Fat32::to_83(const char *name, char out[11])
{
    memset(out, ' ', 11);
    if (name == nullptr || *name == '\0') return false;

    // Everything FAT forbids in a short name, plus space and dot, which are
    // structural here rather than characters.
    static const char *banned = "\"*+,/:;<=>?[\\]| .";

    const char *dot = strrchr(name, '.');
    int i = 0;
    for (const char *p = name; *p != '\0' && p != dot; p++) {
        if (i >= 8) return false;
        char c = *p;
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        if ((unsigned char)c < 0x20 || strchr(banned, c) != nullptr) return false;
        out[i++] = c;
    }
    if (i == 0) return false;

    if (dot != nullptr) {
        int j = 8;
        for (const char *p = dot + 1; *p != '\0'; p++) {
            if (j >= 11) return false;
            char c = *p;
            if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
            if ((unsigned char)c < 0x20 || strchr(banned, c) != nullptr) return false;
            out[j++] = c;
        }
    }
    return true;
}

// Locate the file's 32-byte entry in the root directory, reusing it if the
// name is already there and claiming a free slot if it is not. On success
// dir_lba and dir_off point at the entry and any previous chain has been
// released.
bool Fat32::find_or_make_entry(const char name83[11])
{
    uint32_t clus      = root_clus;
    uint32_t last_clus = root_clus;
    uint32_t free_lba  = 0;
    uint32_t free_off  = 0;
    bool     have_free = false;
    bool     scanning  = true;
    uint32_t guard     = cluster_max + 1;

    while (scanning && clus >= 2 && clus <= cluster_max &&
           clus < CHAIN_END && guard-- > 0) {
        last_clus = clus;

        for (uint32_t s = 0; s < sec_per_clus && scanning; s++) {
            uint32_t lba = lba_of(clus) + s;
            if (!card.read_blocks(lba, meta, 1)) return false;

            for (uint32_t o = 0; o < SECTOR_LEN; o += DIR_ENTRY_LEN) {
                uint8_t *e = meta + o;

                if (e[0] == 0x00) {             // end of directory
                    if (!have_free) { free_lba = lba; free_off = o; have_free = true; }
                    scanning = false;
                    break;
                }
                if (e[0] == 0xE5) {             // deleted, so reusable
                    if (!have_free) { free_lba = lba; free_off = o; have_free = true; }
                    continue;
                }
                if ((e[0x0B] & ATTR_LONG_NAME) == ATTR_LONG_NAME) continue;
                if ((e[0x0B] & ATTR_VOLUME_ID) != 0) continue;

                if (memcmp(e, name83, 11) == 0) {
                    dir_lba = lba;
                    dir_off = o;

                    // Truncating open. Hand the old chain back before reusing
                    // the entry, or those clusters are lost to the volume with
                    // nothing pointing at them.
                    uint32_t old = ((uint32_t)rd16(e + 0x14) << 16) | rd16(e + 0x1A);
                    if (old >= 2 && !free_chain(old)) return false;
                    return true;
                }
            }
        }

        if (!scanning) break;

        uint32_t nxt;
        if (!fat_get(clus, nxt)) return false;
        clus = nxt;
    }

    if (!have_free) {
        // Root directory is full. Unlike FAT16 it can grow, so extend it by
        // one cluster and zero it -- an unzeroed directory cluster is read as
        // whatever the card last held there.
        uint32_t nc;
        if (!alloc_cluster(last_clus, nc)) return false;

        memset(meta, 0, SECTOR_LEN);
        for (uint32_t s = 0; s < sec_per_clus; s++) {
            if (!card.write_blocks(lba_of(nc) + s, meta, 1)) return false;
        }
        free_lba  = lba_of(nc);
        free_off  = 0;
    }

    if (!card.read_blocks(free_lba, meta, 1)) return false;

    uint8_t *e = meta + free_off;
    memset(e, 0, DIR_ENTRY_LEN);
    memcpy(e, name83, 11);
    e[0x0B] = ATTR_ARCHIVE;
    wr16(e + 0x0E, FIXED_TIME);     // created
    wr16(e + 0x10, FIXED_DATE);
    wr16(e + 0x12, FIXED_DATE);     // accessed
    wr16(e + 0x16, FIXED_TIME);     // written
    wr16(e + 0x18, FIXED_DATE);

    if (!card.write_blocks(free_lba, meta, 1)) return false;

    dir_lba = free_lba;
    dir_off = free_off;
    return true;
}

bool Fat32::update_dir_entry()
{
    if (!card.read_blocks(dir_lba, meta, 1)) return false;

    uint8_t *e = meta + dir_off;
    wr16(e + 0x14, (uint16_t)(first_clus >> 16));
    wr16(e + 0x1A, (uint16_t)(first_clus & 0xFFFF));
    wr32(e + 0x1C, size);
    wr16(e + 0x16, FIXED_TIME);
    wr16(e + 0x18, FIXED_DATE);

    return card.write_blocks(dir_lba, meta, 1);
}

// ---- the file --------------------------------------------------------------

bool Fat32::create(const char *name)
{
    if (!mounted) return false;
    if (file_open) close();

    char n83[11];
    if (!to_83(name, n83)) {
        printf("FAT32: '%s' is not a valid 8.3 name\n", name ? name : "(null)");
        return false;
    }

    first_clus  = 0;
    cur_clus    = 0;
    sec_in_clus = 0;
    size        = 0;
    clusters    = 0;
    sect_fill   = 0;

    if (!find_or_make_entry(n83)) return false;

    file_open = true;

    // Commit the empty file now. If the board dies before the first buffer
    // arrives, the card holds a zero-length file rather than an entry
    // pointing at whatever the last flight left in those clusters.
    if (!update_dir_entry() || !fat_flush()) {
        file_open = false;
        return false;
    }

    invalidate_fsinfo();

    printf("FAT32: '%s' open for writing, %lu byte clusters\n",
           name, (unsigned long)cluster_bytes());
    return true;
}

bool Fat32::ensure_cluster()
{
    if (cur_clus == 0) {                    // nothing written yet
        uint32_t c;
        if (!alloc_cluster(0, c)) return false;
        first_clus  = c;
        cur_clus    = c;
        sec_in_clus = 0;
        return true;
    }

    if (sec_in_clus >= sec_per_clus) {      // current cluster is full
        uint32_t c;
        if (!alloc_cluster(cur_clus, c)) return false;
        cur_clus    = c;
        sec_in_clus = 0;
    }
    return true;
}

bool Fat32::append(const uint8_t *data, size_t len)
{
    if (!file_open) return false;

    while (len > 0) {
        if (sect_fill == 0 && len >= SECTOR_LEN) {
            // Straight through to the card, no copy. This is the path every
            // full buffer takes.
            if (!ensure_cluster()) return false;

            // As many whole sectors as fit in what is left of this cluster, in
            // one multi-block write. At 32 KB clusters against a 6 KB buffer
            // that is normally the whole thing in a single CMD25.
            size_t room = (size_t)(sec_per_clus - sec_in_clus);
            size_t n    = len / SECTOR_LEN;
            if (n > room) n = room;

            if (!card.write_blocks(lba_of(cur_clus) + sec_in_clus, data, n))
                return false;

            data        += n * SECTOR_LEN;
            len         -= n * SECTOR_LEN;
            size        += (uint32_t)(n * SECTOR_LEN);
            sec_in_clus += n;
        } else {
            // A partial sector. Only ever happens either side of the final
            // flush in Logger::stop(), where the buffer holds whole 12-byte
            // records but not a whole number of sectors.
            size_t n = SECTOR_LEN - sect_fill;
            if (n > len) n = len;

            memcpy(sect + sect_fill, data, n);
            sect_fill += n;
            data      += n;
            len       -= n;
            size      += (uint32_t)n;

            if (sect_fill == SECTOR_LEN) {
                if (!ensure_cluster()) return false;
                if (!card.write_blocks(lba_of(cur_clus) + sec_in_clus, sect, 1))
                    return false;
                sect_fill = 0;
                sec_in_clus++;
            }
        }
    }
    return true;
}

bool Fat32::sync()
{
    if (!file_open) return false;

    if (sect_fill > 0) {
        if (!ensure_cluster()) return false;

        // Pad to a whole sector and write it, but do NOT advance past it. The
        // next append() carries on into this same sector and overwrites the
        // padding; the file size in the directory entry is the only thing that
        // decides where the data really ends.
        memset(sect + sect_fill, 0, SECTOR_LEN - sect_fill);
        if (!card.write_blocks(lba_of(cur_clus) + sec_in_clus, sect, 1))
            return false;
    }

    if (!fat_flush()) return false;
    return update_dir_entry();
}

bool Fat32::close()
{
    if (!file_open) return true;
    bool ok = sync();
    file_open = false;
    return ok;
}
