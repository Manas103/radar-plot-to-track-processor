#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace radar {

// Lock-free single-producer single-consumer ring buffer. One producer
// thread (detection generation / replay) and one consumer thread (the
// tracker) are the only permitted callers; this is not a general MPMC
// queue.
//
// Memory ordering reasoning:
//   - The producer writes buffer_[tail] (a plain, non-atomic store of a
//     ScanMessage) and then publishes it with tail_.store(next,
//     memory_order_release). The consumer reads tail_ with
//     memory_order_acquire before it reads buffer_[head]. The
//     release/acquire pair establishes a happens-before edge, so the
//     consumer is guaranteed to see the fully-written ScanMessage, not a
//     torn write, without any lock.
//   - Symmetrically the consumer publishes head_ with memory_order_release
//     after it has copied buffer_[head] out, and the producer checks head_
//     with memory_order_acquire before it reuses that slot, so the producer
//     never overwrites a slot the consumer has not finished reading.
//   - The producer's own tail_.load and the consumer's own head_.load (the
//     ones each side uses to compute its next index) are
//     memory_order_relaxed: each thread is the sole writer of that atomic,
//     so there is no cross-thread ordering requirement on reading your own
//     last-written value back.
// This is the standard Boost.Lockfree / Folly-style SPSC ring; capacity
// must be a power of two so the index wrap is a mask instead of a modulo.
template <typename T, std::size_t Capacity>
class SpscRing {
    static_assert(Capacity >= 2 && (Capacity & (Capacity - 1)) == 0,
                  "Capacity must be a power of two >= 2");

public:
    bool try_push(const T& item) {
        const std::size_t tail = tail_.load(std::memory_order_relaxed);
        const std::size_t next = (tail + 1) & kMask;
        if (next == head_.load(std::memory_order_acquire)) {
            return false; // full
        }
        buffer_[tail] = item;
        tail_.store(next, std::memory_order_release);
        return true;
    }

    bool try_pop(T& out) {
        const std::size_t head = head_.load(std::memory_order_relaxed);
        if (head == tail_.load(std::memory_order_acquire)) {
            return false; // empty
        }
        out = buffer_[head];
        head_.store((head + 1) & kMask, std::memory_order_release);
        return true;
    }

    static constexpr std::size_t capacity() { return Capacity; }

private:
    static constexpr std::size_t kMask = Capacity - 1;
    std::array<T, Capacity> buffer_{};
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
};

} // namespace radar
