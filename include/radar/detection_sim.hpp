#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "radar/types.hpp"

namespace radar {

struct SimulatorParams {
    int num_targets = 500;
    unsigned seed = 1;
    double area_size_m = 20000.0;   // targets roam a 20km x 20km maritime patch
    double speed_max_mps = 15.0;    // ~29 kn, a plausible surface-vessel max speed
    double maneuver_accel_sigma = 0.3; // m/s^2 per-axis random-walk acceleration
    double dt_seconds = 1.0;
    double probability_detect = 0.97;  // Pd, not every scan sees every target
    double clutter_rate_per_scan = 2.0; // mean false-alarm count (Poisson)
    double measurement_sigma_m = 50.0;
};

// Simulates a fixed population of maneuvering surface targets and emits one
// noisy ScanMessage per antenna revolution. Missed detections (Pd < 1) and
// clutter false alarms are both modeled because a plot-to-track pipeline
// that only ever sees perfect, clutter-free detections cannot honestly
// exercise the M-of-N deletion logic or the gate's false-alarm rejection.
class DetectionSimulator {
public:
    explicit DetectionSimulator(const SimulatorParams& params);

    ScanMessage next_scan();

private:
    SimulatorParams params_;
    struct SimTarget {
        double x, y, vx, vy;
    };
    std::vector<SimTarget> targets_;
    std::mt19937_64 rng_;
    std::uint64_t scan_id_ = 0;
};

} // namespace radar
