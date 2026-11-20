#include "fusion/link_sim.hpp"

namespace fusion {

DegradedLink::DegradedLink(const LinkSimParams& params) : params_(params), rng_(params.seed) {}

bool DegradedLink::in_outage(double time_s) const {
    if (params_.outage_duration_s <= 0.0) {
        return false;
    }
    return time_s >= params_.outage_start_s && time_s < params_.outage_start_s + params_.outage_duration_s;
}

bool DegradedLink::try_send(double time_s, std::size_t num_bytes) {
    if (time_s > last_time_s_) {
        if (!in_outage(time_s)) {
            active_elapsed_s_ += (time_s - last_time_s_);
        }
        last_time_s_ = time_s;
    }

    if (in_outage(time_s)) {
        return false;
    }
    double bytes_per_second_budget = params_.kbps * 1000.0 / 8.0;
    double budget_so_far = bytes_per_second_budget * active_elapsed_s_;
    if (bytes_sent_ + static_cast<double>(num_bytes) > budget_so_far) {
        return false; // over the link's bandwidth budget for this instant
    }
    std::bernoulli_distribution loss_dist(params_.loss_probability);
    if (loss_dist(rng_)) {
        return false; // lost on the wire
    }
    bytes_sent_ += static_cast<double>(num_bytes);
    return true;
}

} // namespace fusion
