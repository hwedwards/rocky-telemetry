#include "sd_storage.hpp"
#include "pico/time.h"
#include <stdio.h>

bool SdStorage::mount()
{
    bytes = writes = errors = worst_us = stalls = 0;

    if (!card.init()) return false;
    return fs.mount();
}

bool SdStorage::open(const char *path)
{
    if (!fs.is_mounted()) return false;
    return fs.create(path);
}

// Runs on core 1. No printf on the happy path: stdio is mutex-guarded so it
// would not corrupt core 0's output, but blocking the write path on USB CDC is
// a silly way to lose samples. A failure is reported through the event queue
// by Logger, in main context, where printing costs nothing that matters.
bool SdStorage::write(const uint8_t *data, size_t len)
{
    uint32_t t0 = time_us_32();
    bool ok = fs.append(data, len);
    uint32_t dt = time_us_32() - t0;

    if (dt > worst_us) worst_us = dt;
    if (dt > STALL_US) stalls++;

    if (ok) {
        bytes += (uint32_t)len;
        writes++;
    } else {
        errors++;
    }
    return ok;
}

bool SdStorage::sync()
{
    uint32_t t0 = time_us_32();
    bool ok = fs.sync();
    uint32_t dt = time_us_32() - t0;

    // A sync is a write as far as the card is concerned, and it is usually the
    // slowest one -- it touches the FAT and the directory entry, which live
    // nowhere near the data. Let it count towards the worst case.
    if (dt > worst_us) worst_us = dt;
    if (!ok) errors++;
    return ok;
}

void SdStorage::close()
{
    fs.close();
}

void SdStorage::report() const
{
    printf("SdStorage: %lu bytes in %lu writes, %lu errors, "
           "worst write %lu us, %lu stalls over %lu ms\n",
           (unsigned long)bytes,
           (unsigned long)writes,
           (unsigned long)errors,
           (unsigned long)worst_us,
           (unsigned long)stalls,
           (unsigned long)(STALL_US / 1000));

    printf("SdStorage: file is %lu bytes across %lu clusters; "
           "card reported %lu protocol errors, %lu timeouts\n",
           (unsigned long)fs.file_size(),
           (unsigned long)fs.clusters_used(),
           (unsigned long)card.protocol_errors(),
           (unsigned long)card.timeouts());
}
