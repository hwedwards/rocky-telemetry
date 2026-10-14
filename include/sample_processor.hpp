#pragma once
#include "inputs.hpp"

// accepts() runs before core 1 launches. process() runs only on core 1.
// In-place, format/length preserving, synchronous: do not retain data after
// returning. A false result discards this batch and reports PROCESS_ERROR.
// The descriptor is passed by value so a processor cannot resize the handoff.
class SampleProcessor {
public:
    virtual ~SampleProcessor() = default;
    virtual bool accepts(SampleFormat format) const = 0;
    virtual bool process(SampleBatch batch) = 0;
};
class PassThroughProcessor final : public SampleProcessor {
public:
    bool accepts(SampleFormat format) const override { return format.record_size != 0; }
    bool process(SampleBatch) override { return true; }
};
