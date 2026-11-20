#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "fusion/types.hpp"

namespace fusion {

struct AisSimParams {
    int num_vessels = 2;
    unsigned seed = 1;
    double area_size_m = 20000.0;
    double speed_max_mps = 7.7;            // ~15 kn, a plausible vessel cruise speed
    double maneuver_accel_sigma = 0.1;     // gentler random-walk accel than radar's maneuvering targets
    double position_noise_sigma_m = 20.0;  // AIS GPS-derived fix, tighter than radar's return noise
    double dt_seconds = 1.0;
};

// A simulated AIS-shaped position report (the fields a real AIS Class A
// position report, message type 1/2/3, carries). "AIS-shaped" because
// this is not a bit-level AIS/NMEA encoding the way adsb.hpp's frames are
// a real DF17 encoding; see the README "Honest framing".
struct AisReport {
    std::uint32_t mmsi = 0;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double sog_knots = 0.0;
    double cog_deg = 0.0;
};

class AisSimulator {
public:
    AisSimulator(const AisSimParams& params, GeoRef geo_ref);
    std::vector<AisReport> next_reports();

private:
    struct Vessel {
        double x, y, vx, vy;
    };
    AisSimParams params_;
    GeoRef geo_ref_;
    std::vector<Vessel> vessels_;
    std::vector<std::uint32_t> mmsis_;
    std::mt19937_64 rng_;
};

} // namespace fusion
