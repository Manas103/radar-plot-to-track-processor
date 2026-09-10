#include "radar/tracker.hpp"

#include <algorithm>

#include "radar/assignment.hpp"
#include "radar/gating.hpp"

namespace radar {

void TrackManager::confirm_and_delete_logic(Track& t, bool hit) {
    t.hit_history.push_back(hit);
    while (static_cast<int>(t.hit_history.size()) > params_.confirm_window_n) {
        t.hit_history.pop_front();
    }
    t.consecutive_misses = hit ? 0 : (t.consecutive_misses + 1);

    if (t.status == TrackStatus::kTentative) {
        int hits = static_cast<int>(std::count(t.hit_history.begin(), t.hit_history.end(), true));
        if (hits >= params_.confirm_hits_m) {
            t.status = TrackStatus::kConfirmed;
        } else if (static_cast<int>(t.hit_history.size()) >= params_.confirm_window_n &&
                   hits < params_.confirm_hits_m) {
            // Used the full window and still short: this tentative track
            // never earned confirmation, drop it rather than let it linger.
            t.status = TrackStatus::kDeleted;
        }
    } else if (t.status == TrackStatus::kConfirmed) {
        if (t.consecutive_misses >= params_.delete_consecutive_misses) {
            t.status = TrackStatus::kDeleted;
        }
    }
}

ScanResult TrackManager::process_scan(const ScanMessage& scan) {
    ScanResult result;
    result.scan_id = scan.scan_id;

    // 1. Predict every live track forward by dt.
    for (auto& t : tracks_) {
        if (t.status == TrackStatus::kDeleted) continue;
        t.filter.predict(params_.dt_seconds);
    }

    // 2. Build the chi-square gate graph between live tracks and this
    //    scan's detections.
    std::vector<GateEdge> edges;
    std::vector<Vec2> innovations(tracks_.size());
    std::vector<Mat2> innov_cov(tracks_.size());
    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
        if (tracks_[ti].status == TrackStatus::kDeleted) continue;
        for (std::uint32_t di = 0; di < scan.count; ++di) {
            Vec2 y;
            Mat2 s;
            tracks_[ti].filter.innovation(scan.detections[di], y, s);
            Mat2 s_inv = mat2_inverse(s);
            Vec2 sy = mat2_mul_vec2(s_inv, y);
            double maha_sq = vec2_dot(y, sy);
            if (within_gate(maha_sq, params_.gate_threshold)) {
                edges.push_back(GateEdge{static_cast<int>(ti), static_cast<int>(di), maha_sq});
            }
        }
    }

    // 3. Solve the gated global-nearest-neighbor assignment (Hungarian per
    //    connected component; see assignment.hpp for why that is exact).
    std::vector<Association> assoc =
        solve_gated_assignment(edges, static_cast<int>(tracks_.size()), static_cast<int>(scan.count),
                                params_.gate_threshold * 10.0);

    std::vector<bool> track_hit(tracks_.size(), false);
    std::vector<bool> det_used(scan.count, false);
    for (const auto& a : assoc) {
        Vec2 y;
        Mat2 s;
        tracks_[a.track_idx].filter.innovation(scan.detections[a.det_idx], y, s);
        tracks_[a.track_idx].filter.update(y, s);
        track_hit[a.track_idx] = true;
        det_used[a.det_idx] = true;
    }
    result.num_associations = static_cast<int>(assoc.size());

    // 4. M-of-N confirm/delete bookkeeping for every live track.
    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
        if (tracks_[ti].status == TrackStatus::kDeleted) continue;
        confirm_and_delete_logic(tracks_[ti], track_hit[ti]);
        tracks_[ti].scans_since_start++;
    }

    // 5. Spawn a new tentative track for every unassociated detection.
    for (std::uint32_t di = 0; di < scan.count; ++di) {
        if (det_used[di]) continue;
        Track t;
        t.id = next_id_++;
        t.status = TrackStatus::kTentative;
        t.filter.init(scan.detections[di].x, scan.detections[di].y, 0.0, 0.0,
                       params_.process_noise_density, params_.measurement_sigma_m);
        t.hit_history.push_back(true);
        t.scans_since_start = 1;
        tracks_.push_back(std::move(t));
        result.num_new_tracks++;
    }

    // 6. Remove tracks that were just deleted; ids are never reused.
    int before = static_cast<int>(tracks_.size());
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                  [](const Track& t) { return t.status == TrackStatus::kDeleted; }),
                  tracks_.end());
    result.num_deleted = before - static_cast<int>(tracks_.size());

    for (const auto& t : tracks_) {
        if (t.status == TrackStatus::kConfirmed) result.num_confirmed++;
        if (t.status == TrackStatus::kTentative) result.num_tentative++;
    }

    return result;
}

} // namespace radar
