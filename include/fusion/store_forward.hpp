#pragma once

#include <cstdint>
#include <deque>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include "fusion/link_sim.hpp"

namespace fusion {

struct QueuedMessage {
    std::string guid;
    std::string payload;
    double last_attempt_s = -1e18;
    bool acked = false;
};

// Sender-side store-and-forward: a message that cannot be sent this tick
// (loss, outage, bandwidth budget) stays queued and is retried after
// `retransmit_interval_s`. The ACK channel is its own independent loss
// roll (ACKs are small enough that bandwidth is never their limit, only
// loss is), which is why the same message can genuinely reach the
// receiver more than once: the forward send can succeed while its ACK is
// lost, so the sender, not knowing that, retransmits anyway.
// `ReceiverLedger`'s GUID dedup is what makes that safe.
class StoreAndForwardSender {
public:
    StoreAndForwardSender(DegradedLink& forward_link, double retransmit_interval_s, double ack_loss_probability,
                           unsigned seed);

    void enqueue(std::string guid, std::string payload);

    // Attempts every queued message's turn at `time_s`; returns the GUIDs
    // the receiver actually saw this tick (forward delivery succeeded),
    // including re-deliveries of an already-applied message.
    std::vector<std::string> tick(double time_s);

    std::size_t queue_depth() const { return queue_.size(); }
    std::uint64_t forward_attempts() const { return forward_attempts_; }
    std::uint64_t forward_successes() const { return forward_successes_; }

private:
    DegradedLink& forward_link_;
    double retransmit_interval_s_;
    double ack_loss_probability_;
    std::mt19937_64 ack_rng_;
    std::deque<QueuedMessage> queue_;
    std::uint64_t forward_attempts_ = 0;
    std::uint64_t forward_successes_ = 0;
};

// Applies each delivered message exactly once, by GUID, regardless of how
// many times the sender's retries actually delivered it over the wire.
class ReceiverLedger {
public:
    bool apply(const std::string& guid);
    std::size_t applied_count() const { return applied_guids_.size(); }
    std::size_t duplicate_deliveries() const { return duplicate_deliveries_; }

private:
    std::unordered_set<std::string> applied_guids_;
    std::size_t duplicate_deliveries_ = 0;
};

} // namespace fusion
