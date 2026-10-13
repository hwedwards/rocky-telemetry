#include "storage.hpp"
#include "imu.hpp"
#include "hardware/timer.h"
#include <stdio.h>

bool NullStorage::open(const char *path)
{
    bytes  = 0;
    writes = 0;
    stalls = 0;
    printf("NullStorage: pretending to open '%s'\n", path);
    return true;
}

void NullStorage::set_dump(uint32_t every_n_buffers, const IMU *imu)
{
    dump_n   = every_n_buffers;
    dump_imu = imu;
}

// Runs on core 1.
//
// busy_wait rather than sleep_ms: the default alarm pool lives on core 0, and
// a real SD driver polls the card's busy line here anyway, so spinning is the
// faithful simulation. Blocking core 1 for a quarter of a second is precisely
// the abuse the two-core split is supposed to shrug off.
bool NullStorage::write(const uint8_t *data, size_t len)
{
    writes++;

    bool stall = delays.stall_every && (writes % delays.stall_every == 0);
    uint32_t ms = stall ? delays.stall_ms : delays.typical_ms;
    if (stall) stalls++;
    if (ms) busy_wait_ms(ms);

    bytes += len;

    // printf on core 1 is fine here and nowhere else: stdio is mutex-guarded
    // (PICO_STDOUT_MUTEX defaults on) so it cannot corrupt core 0's output,
    // and the only cost is more delay on a core that is allowed to be slow.
    if (dump_n && dump_imu && len >= IMU::FIFO_SAMPLE_LEN &&
        (writes % dump_n) == 0) {
        IMU::Sample s = dump_imu->decode(data);
        printf("  [write %lu] a = %6.3f %6.3f %6.3f g   "
               "g = %8.2f %8.2f %8.2f d/s\n",
               (unsigned long)writes,
               IMU::to_g(s.accel[0]), IMU::to_g(s.accel[1]), IMU::to_g(s.accel[2]),
               IMU::to_dps(s.gyro[0]), IMU::to_dps(s.gyro[1]), IMU::to_dps(s.gyro[2]));
    }

    return true;
}

void NullStorage::report() const
{
    printf("NullStorage: %lu bytes in %lu writes, %lu simulated stalls\n",
           (unsigned long)bytes, (unsigned long)writes, (unsigned long)stalls);
}

void NullStorage::close()
{
    printf("NullStorage: closed after %lu bytes\n", (unsigned long)bytes);
}
