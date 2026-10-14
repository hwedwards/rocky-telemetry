#pragma once
class EventSource {
public:
    virtual ~EventSource() = default;
    virtual bool begin() = 0;
};
