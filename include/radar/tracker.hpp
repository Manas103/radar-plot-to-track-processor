#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include "radar/kalman.hpp"
#include "radar/types.hpp"

namespace radar {

struct TrackerParams {
    double dt_seconds = 1.0;
    double gate_threshold = 9.21034; // kChiSquareGate99_2dof, duplicated to avoid a header cycle
    double process_noise_density = 4.0; // m^2/s^3, moderate maneuvering allowance
    double measurement_sigma_m = 50.0;  // detector range/bearing noise folded to Cartesian meters
    int confirm_window_n = 5;           // M-of-N window size
    int confirm_hits_m = 3;             // hits needed within the window to confirm
    int delete_consecutive_misses = 3;  // consecutive misses that deletes a confirmed track
};

struct Track {
    std::uint32_t id = 0;
    TrackStatus status = TrackStatus::kTentative;
    KalmanCV filter;
    std::deque<bool> hit_history; // true = associated this scan, oldest first
    int consecutive_misses = 0;
    std::uint64_t scans_since_start = 0;
};

// Per-scan statistics, useful both for the M-of-N invariant tests and for
// the deterministic canonical log used by the replay / oracle-comparison
// tooling.
struct ScanResult {
    std::uint64_t scan_id = 0;
    int num_confirmed = 0;
    int num_tentative = 0;
    int num_associations = 0;
    int num_new_tracks = 0;
    int num_deleted = 0;
};

// Owns the track list end to end: predict, chi-square-gated GNN
// association, Kalman update, M-of-N confirmation/deletion, and track
// spawn/delete. Deleted tracks are removed from the live list at the end of
// the scan that deletes them (their id is never reused, which is one of the
// conservation invariants tests/test_tracker.cpp checks).
class TrackManager {
public:
    explicit TrackManager(TrackerParams params) : params_(params) {}

    ScanResult process_scan(const ScanMessage& scan);

    const std::vector<Track>& tracks() const { return tracks_; }
    std::uint32_t tracks_spawned_total() const { return next_id_ - 1; }

private:
    void confirm_and_delete_logic(Track& t, bool hit);

    TrackerParams params_;
    std::vector<Track> tracks_;
    std::uint32_t next_id_ = 1;
};

} // namespace radar
