#pragma once
#include <condition_variable>
#include <mutex>
#include <deque>
#include <vector>
#include <cstring>
#include <cassert>
using uint = unsigned;
struct queue_t {
    std::mutex mutex;
    std::condition_variable changed;
    std::deque<std::vector<unsigned char>> items;
    size_t size = 0, capacity = 0;
};
inline bool queue_init(queue_t *q, uint size, uint capacity) {
    q->size = size; q->capacity = capacity; return true;
}
inline void queue_free(queue_t *) {}
inline bool queue_try_add(queue_t *q, const void *data) {
    std::lock_guard<std::mutex> lock(q->mutex);
    if (q->items.size() == q->capacity) return false;
    auto p = static_cast<const unsigned char *>(data);
    q->items.emplace_back(p, p + q->size);
    q->changed.notify_all(); return true;
}
inline void queue_add_blocking(queue_t *q, const void *data) {
    std::unique_lock<std::mutex> lock(q->mutex);
    q->changed.wait(lock, [&]{ return q->items.size() < q->capacity; });
    auto p = static_cast<const unsigned char *>(data);
    q->items.emplace_back(p, p + q->size); q->changed.notify_all();
}
inline bool queue_try_remove(queue_t *q, void *out) {
    std::lock_guard<std::mutex> lock(q->mutex);
    if (q->items.empty()) return false;
    std::memcpy(out, q->items.front().data(), q->size);
    q->items.pop_front(); q->changed.notify_all(); return true;
}
inline void queue_remove_blocking(queue_t *q, void *out) {
    std::unique_lock<std::mutex> lock(q->mutex);
    q->changed.wait(lock, [&]{ return !q->items.empty(); });
    std::memcpy(out, q->items.front().data(), q->size);
    q->items.pop_front(); q->changed.notify_all();
}
