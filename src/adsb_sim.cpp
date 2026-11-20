#include "fusion/adsb_sim.hpp"

#include <algorithm>

#include "fusion/geo.hpp"

namespace fusion {

AdsbSimulator::AdsbSimulator(const AdsbSimParams& params, GeoRef geo_ref)
    : params_(params), geo_ref_(geo_ref), rng_(params.seed) {
    std::uniform_real_distribution<double> pos_dist(-params_.area_size_m / 2.0, params_.area_size_m / 2.0);
    std::uniform_real_distribution<double> speed_dist(-params_.speed_max_mps, params_.speed_max_mps);
    aircraft_.reserve(static_cast<std::size_t>(params_.num_aircraft));
    for (int i = 0; i < params_.num_aircraft; ++i) {
        aircraft_.push_back(Aircraft{pos_dist(rng_), pos_dist(rng_), speed_dist(rng_), speed_dist(rng_)});
        icaos_.push_back(0xA00000u + static_cast<std::uint32_t>(i));
    }
}

std::vector<AdsbPositionReport> AdsbSimulator::next_reports() {
    ++scan_id_;
    std::normal_distribution<double> accel_dist(0.0, params_.maneuver_accel_sigma);

    std::vector<AdsbPositionReport> reports;
    reports.reserve(aircraft_.size());

    double half = params_.area_size_m / 2.0;
    for (std::size_t i = 0; i < aircraft_.size(); ++i) {
        Aircraft& a = aircraft_[i];
        a.vx += accel_dist(rng_) * params_.dt_seconds;
        a.vy += accel_dist(rng_) * params_.dt_seconds;
        a.vx = std::clamp(a.vx, -params_.speed_max_mps, params_.speed_max_mps);
        a.vy = std::clamp(a.vy, -params_.speed_max_mps, params_.speed_max_mps);
        a.x += a.vx * params_.dt_seconds;
        a.y += a.vy * params_.dt_seconds;
        if (a.x < -half || a.x > half) {
            a.vx = -a.vx;
            a.x = std::clamp(a.x, -half, half);
        }
        if (a.y < -half || a.y > half) {
            a.vy = -a.vy;
            a.y = std::clamp(a.y, -half, half);
        }

        LatLon truth = local_xy_to_latlon(geo_ref_, a.x, a.y);
        bool odd = (scan_id_ % 2) == 1;
        adsb::Frame frame = adsb::encode_airborne_position(icaos_[i], truth.lat_deg, truth.lon_deg, odd);
        adsb::DecodedFrame decoded =
            adsb::decode_airborne_position(frame, geo_ref_.ref_lat_deg, geo_ref_.ref_lon_deg);

        AdsbPositionReport report;
        report.icao24 = decoded.icao24;
        report.crc_ok = decoded.crc_ok;
        report.lat_deg = decoded.lat_deg;
        report.lon_deg = decoded.lon_deg;
        reports.push_back(report);
    }
    return reports;
}

} // namespace fusion
