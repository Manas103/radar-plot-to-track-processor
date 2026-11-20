#pragma once

#include <cstdint>
#include <vector>

#include "fusion/adsb_sim.hpp"
#include "fusion/ais_sim.hpp"
#include "fusion/types.hpp"
#include "radar/detection_sim.hpp"
#include "radar/types.hpp"

namespace fusion {

struct FusionScenarioParams {
    int num_radar_targets = 2;
    int num_adsb_targets = 2;
    int num_ais_targets = 2;
    unsigned seed = 1;
    double area_size_m = 20000.0;
    double speed_max_mps = 15.0;
    double maneuver_accel_sigma = 0.3;
    double dt_seconds = 1.0;
    double radar_measurement_sigma_m = 50.0;
    GeoRef geo_ref{37.5, -122.3};
};

// One fused scan: the merged, three-feed detection list a single
// chi-square-gated GNN/Kalman/M-of-N `radar::TrackManager` (unmodified,
// the same class the base tracker uses) processes as one track picture.
// `detection_source` is parallel to `scan.detections[0..scan.count)`, for
// reporting which feed(s) contributed to a track; the tracker itself
// never sees it.
struct FusedScanResult {
    radar::ScanMessage scan;
    std::vector<Source> detection_source;
};

// Composes the base tracker's own, unmodified `radar::DetectionSimulator`
// with this extension's `AdsbSimulator` and `AisSimulator`, merging all
// three feeds' positions into one `radar::ScanMessage` per tick.
class FusionScenario {
public:
    explicit FusionScenario(const FusionScenarioParams& params);

    FusedScanResult next_scan();

private:
    FusionScenarioParams params_;
    radar::DetectionSimulator radar_sim_;
    AdsbSimulator adsb_sim_;
    AisSimulator ais_sim_;
    std::uint64_t scan_id_ = 0;
};

} // namespace fusion
