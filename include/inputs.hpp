#pragma once
#include <cstddef>
#include <cstdint>

// Identifies the byte layout, not a runtime schema. Add new layouts explicitly.
enum class SampleKind : uint8_t { AccelGyroBe16, Opaque };
struct SampleFormat {
    SampleKind kind;
    size_t record_size;
};
struct SampleBatch {
    uint8_t *data;
    size_t size;
    SampleFormat format;
};
// Core 0 only. format() is stable for the lifetime of the source. read() consumes
// exactly len bytes on success; callers request whole records. stop() freezes
// acquisition while leaving the final buffered records readable.
class Inputs {
public:
    virtual ~Inputs() = default;
    virtual SampleFormat format() const = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual bool available(size_t &bytes) = 0;
    virtual bool read(uint8_t *dst, size_t len) = 0;
    virtual bool overflowed(bool &value) = 0;
    virtual bool reset() = 0;
};
