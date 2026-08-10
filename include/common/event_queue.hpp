#pragma once

// Bounded event channel between producer threads (e.g. a DDS callback) and
// a polling consumer (e.g. the 50Hz control loop).
//
// This completes the trio of channel primitives used across the KIST repos —
// pick by what the data MEANS, not by shape:
//
//   DataBuffer<T>   state    — only the newest value matters; overwrite,
//                              consumers poll (data_buffer.hpp).
//   EventQueue<T>   events   — every item matters and order matters; under
//                              overflow the OLDEST is evicted (a fresh
//                              command outranks a stale one). Non-blocking
//                              on BOTH sides — safe to drain from an RT
//                              loop.
//   RecordQueue<T>  recording— lossless hand-off to a dedicated writer
//                              thread; refuses the NEWEST on overflow (the
//                              drop is counted), consumer blocks on a CV
//                              (kist-data-collector common/record_queue.hpp).
//
// Multi-producer safe; drain() is atomic, so concurrent consumers each get
// disjoint batches (typical use is a single polling consumer).

#include <deque>
#include <mutex>
#include <optional>
#include <vector>

template <typename T>
class EventQueue {
public:
    explicit EventQueue(size_t capacity) : capacity_(capacity) {}

    // Producer side; never blocks beyond the mutex. If the queue is full the
    // OLDEST item is evicted and returned so the caller can log/count it —
    // eviction is never silent by construction.
    std::optional<T> push(T item) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::optional<T> evicted;
        if (queue_.size() >= capacity_) {
            evicted = std::move(queue_.front());
            queue_.pop_front();
        }
        queue_.push_back(std::move(item));
        return evicted;
    }

    // Consumer side: everything queued so far, in arrival order. Never
    // blocks beyond the mutex; empty vector when there is nothing.
    std::vector<T> drain() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<T> out(std::make_move_iterator(queue_.begin()),
                           std::make_move_iterator(queue_.end()));
        queue_.clear();
        return out;
    }

private:
    std::deque<T>      queue_;
    size_t             capacity_;
    mutable std::mutex mutex_;
};
