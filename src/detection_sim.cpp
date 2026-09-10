#include "radar/detection_sim.hpp"

#include <algorithm>
#include <cmath>

namespace radar {

DetectionSimulator::DetectionSimulator(const SimulatorParams& params)
    : params_(params), rng_(params.seed) {
    std::uniform_real_distribution<double> pos_dist(0.0, params_.area_size_m);
    std::uniform_real_distribution<double> speed_dist(-params_.speed_max_mps, params_.speed_max_mps);
    targets_.reserve(static_cast<std::size_t>(params_.num_targets));
    for (int i = 0; i < params_.num_targets; ++i) {
        targets_.push_back(SimTarget{pos_dist(rng_), pos_dist(rng_), speed_dist(rng_), speed_dist(rng_)});
    }
}

ScanMessage DetectionSimulator::next_scan() {
    ScanMessage scan;
    scan.scan_id = ++scan_id_;
    scan.timestamp_s = static_cast<double>(scan_id_) * params_.dt_seconds;

    std::normal_distribution<double> accel_dist(0.0, params_.maneuver_accel_sigma);
    std::normal_distribution<double> meas_noise(0.0, params_.measurement_sigma_m);
    std::bernoulli_distribution detect_dist(params_.probability_detect);
    std::poisson_distribution<int> clutter_dist(params_.clutter_rate_per_scan);
    std::uniform_real_distribution<double> area_dist(0.0, params_.area_size_m);

    std::uint32_t count = 0;
    for (auto& t : targets_) {
        // Small random-walk maneuvering acceleration, then propagate and
        // reflect off the area boundary so targets stay in the patch
        // instead of wandering off to infinity.
        t.vx += accel_dist(rng_) * params_.dt_seconds;
        t.vy += accel_dist(rng_) * params_.dt_seconds;
        t.vx = std::clamp(t.vx, -params_.speed_max_mps, params_.speed_max_mps);
        t.vy = std::clamp(t.vy, -params_.speed_max_mps, params_.speed_max_mps);
        t.x += t.vx * params_.dt_seconds;
        t.y += t.vy * params_.dt_seconds;
        if (t.x < 0.0 || t.x > params_.area_size_m) {
            t.vx = -t.vx;
            t.x = std::clamp(t.x, 0.0, params_.area_size_m);
        }
        if (t.y < 0.0 || t.y > params_.area_size_m) {
            t.vy = -t.vy;
            t.y = std::clamp(t.y, 0.0, params_.area_size_m);
        }

        if (!detect_dist(rng_)) continue; // missed detection this scan (Pd)
        if (count >= kMaxTargets) break;
        scan.detections[count].x = t.x + meas_noise(rng_);
        scan.detections[count].y = t.y + meas_noise(rng_);
        ++count;
    }

    int clutter = clutter_dist(rng_);
    for (int c = 0; c < clutter && count < kMaxTargets; ++c) {
        scan.detections[count].x = area_dist(rng_);
        scan.detections[count].y = area_dist(rng_);
        ++count;
    }

    scan.count = count;
    return scan;
}

} // namespace radar
