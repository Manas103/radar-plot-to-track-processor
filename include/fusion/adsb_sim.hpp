#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "fusion/adsb.hpp"
#include "fusion/types.hpp"

namespace fusion {

struct AdsbSimParams {
    int num_aircraft = 2;
    unsigned seed = 1;
    double area_size_m = 20000.0;
    double speed_max_mps = 230.0;        // ~450 kn, a plausible airliner cruise groundspeed
    double maneuver_accel_sigma = 1.0;
    double dt_seconds = 1.0;
};

// One real DF17 encode-then-decode round trip's result for one simulated
// aircraft this scan: ground truth never reaches the tracker directly,
// only what `fusion::adsb::decode_airborne_position` actually recovered
// from the encoded frame (see adsb.hpp).
struct AdsbPositionReport {
    std::uint32_t icao24 = 0;
    bool crc_ok = false;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
};

class AdsbSimulator {
public:
    AdsbSimulator(const AdsbSimParams& params, GeoRef geo_ref);
    std::vector<AdsbPositionReport> next_reports();

private:
    struct Aircraft {
        double x, y, vx, vy;
    };
    AdsbSimParams params_;
    GeoRef geo_ref_;
    std::vector<Aircraft> aircraft_;
    std::vector<std::uint32_t> icaos_;
    std::mt19937_64 rng_;
    std::uint64_t scan_id_ = 0;
};

} // namespace fusion
