#include "radar/spsc_ring.hpp"

#include <atomic>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

using namespace radar;

TEST(SpscRing, PushPopSingleThreadedFifoOrder) {
    SpscRing<int, 8> ring;
    for (int i = 0; i < 5; ++i) EXPECT_TRUE(ring.try_push(i));
    for (int i = 0; i < 5; ++i) {
        int v = -1;
        EXPECT_TRUE(ring.try_pop(v));
        EXPECT_EQ(v, i);
    }
    int v;
    EXPECT_FALSE(ring.try_pop(v)); // empty
}

TEST(SpscRing, FullRingRejectsPush) {
    SpscRing<int, 4> ring; // holds at most 3 items (one slot always kept free)
    EXPECT_TRUE(ring.try_push(1));
    EXPECT_TRUE(ring.try_push(2));
    EXPECT_TRUE(ring.try_push(3));
    EXPECT_FALSE(ring.try_push(4)); // full
    int v;
    EXPECT_TRUE(ring.try_pop(v));
    EXPECT_EQ(v, 1);
    EXPECT_TRUE(ring.try_push(4)); // now there is room again
}

// Two real threads, no locks: the consumer must see a strictly increasing
// sequence with no gaps and no duplicates, which is only possible if the
// release/acquire pairing documented in spsc_ring.hpp actually establishes
// happens-before between the producer's write and the consumer's read. This
// is also the test run under ThreadSanitizer (see docs/tsan_clean_run.txt).
TEST(SpscRing, ConcurrentProducerConsumerPreservesOrderAndCompleteness) {
    constexpr int kCount = 200000;
    SpscRing<int, 1024> ring;
    std::atomic<bool> producer_done{false};

    std::thread producer([&]() {
        for (int i = 0; i < kCount; ++i) {
            while (!ring.try_push(i)) std::this_thread::yield();
        }
        producer_done.store(true, std::memory_order_release);
    });

    std::vector<int> received;
    received.reserve(kCount);
    while (true) {
        int v;
        if (ring.try_pop(v)) {
            received.push_back(v);
        } else if (producer_done.load(std::memory_order_acquire)) {
            if (ring.try_pop(v)) {
                received.push_back(v);
                continue;
            }
            break;
        } else {
            std::this_thread::yield();
        }
    }
    producer.join();

    ASSERT_EQ(received.size(), static_cast<std::size_t>(kCount));
    for (int i = 0; i < kCount; ++i) {
        ASSERT_EQ(received[static_cast<std::size_t>(i)], i) << "at index " << i;
    }
}
