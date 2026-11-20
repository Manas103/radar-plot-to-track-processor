#pragma once

#include <cstddef>
#include <random>

namespace fusion {

struct LinkSimParams {
    double kbps = 64.0;
    double loss_probability = 0.20;
    double outage_start_s = 0.0;
    double outage_duration_s = 0.0; // 0 = no scheduled outage
    unsigned seed = 1;
};

// A degraded 64 kbps, 20%-loss link with one scheduled outage window, the
// shape a tactical radio relay takes. This is the only thing that knows
// about bandwidth, loss and the outage; `StoreAndForwardSender`
// (store_forward.hpp) is the only thing that knows to retry.
class DegradedLink {
public:
    explicit DegradedLink(const LinkSimParams& params);

    // Attempts to send `num_bytes` at `time_s`; paces against the link's
    // own cumulative byte budget since time 0 and returns false during
    // the scheduled outage, when this attempt's random loss roll fails,
    // or when the byte budget has not yet caught up. Never blocks;
    // callers decide what "retry later" means.
    bool try_send(double time_s, std::size_t num_bytes);

    bool in_outage(double time_s) const;

private:
    LinkSimParams params_;
    std::mt19937_64 rng_;
    double bytes_sent_ = 0.0;
    // Bandwidth budget accrues only while the link is not in outage, so
    // recovery cannot "burst catch up" using capacity the outage never
    // actually had; see link_sim.cpp.
    double active_elapsed_s_ = 0.0;
    double last_time_s_ = 0.0;
};

} // namespace fusion
