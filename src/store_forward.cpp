#include "fusion/store_forward.hpp"

namespace fusion {

StoreAndForwardSender::StoreAndForwardSender(DegradedLink& forward_link, double retransmit_interval_s,
                                              double ack_loss_probability, unsigned seed)
    : forward_link_(forward_link),
      retransmit_interval_s_(retransmit_interval_s),
      ack_loss_probability_(ack_loss_probability),
      ack_rng_(seed) {}

void StoreAndForwardSender::enqueue(std::string guid, std::string payload) {
    queue_.push_back(QueuedMessage{std::move(guid), std::move(payload), -1e18, false});
}

std::vector<std::string> StoreAndForwardSender::tick(double time_s) {
    std::vector<std::string> delivered_guids;
    std::bernoulli_distribution ack_loss_dist(ack_loss_probability_);

    for (auto& msg : queue_) {
        if (msg.acked) continue;
        if (time_s - msg.last_attempt_s < retransmit_interval_s_) continue;

        msg.last_attempt_s = time_s;
        ++forward_attempts_;
        if (forward_link_.try_send(time_s, msg.payload.size())) {
            ++forward_successes_;
            delivered_guids.push_back(msg.guid);
            if (!ack_loss_dist(ack_rng_)) {
                msg.acked = true;
            }
            // else: delivered, but the ACK was lost; stays queued and
            // will be retransmitted, which is exactly the duplicate-
            // delivery risk the receiver's GUID dedup exists for.
        }
        // else: not delivered this attempt; stays queued, retried later.
    }

    std::deque<QueuedMessage> remaining;
    for (auto& msg : queue_) {
        if (!msg.acked) {
            remaining.push_back(std::move(msg));
        }
    }
    queue_ = std::move(remaining);

    return delivered_guids;
}

bool ReceiverLedger::apply(const std::string& guid) {
    auto result = applied_guids_.insert(guid);
    if (!result.second) {
        ++duplicate_deliveries_;
        return false;
    }
    return true;
}

} // namespace fusion
