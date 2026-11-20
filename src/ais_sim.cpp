#include "fusion/ais_sim.hpp"

#include <algorithm>
#include <cmath>

#include "fusion/geo.hpp"

namespace fusion {

AisSimulator::AisSimulator(const AisSimParams& params, GeoRef geo_ref)
    : params_(params), geo_ref_(geo_ref), rng_(params.seed) {
    std::uniform_real_distribution<double> pos_dist(-params_.area_size_m / 2.0, params_.area_size_m / 2.0);
    std::uniform_real_distribution<double> speed_dist(-params_.speed_max_mps, params_.speed_max_mps);
    vessels_.reserve(static_cast<std::size_t>(params_.num_vessels));
    for (int i = 0; i < params_.num_vessels; ++i) {
        vessels_.push_back(Vessel{pos_dist(rng_), pos_dist(rng_), speed_dist(rng_), speed_dist(rng_)});
        mmsis_.push_back(366000000u + static_cast<std::uint32_t>(i));
    }
}

std::vector<AisReport> AisSimulator::next_reports() {
    std::normal_distribution<double> accel_dist(0.0, params_.maneuver_accel_sigma);
    std::normal_distribution<double> noise_dist(0.0, params_.position_noise_sigma_m);

    std::vector<AisReport> reports;
    reports.reserve(vessels_.size());

    double half = params_.area_size_m / 2.0;
    for (std::size_t i = 0; i < vessels_.size(); ++i) {
        Vessel& v = vessels_[i];
        v.vx += accel_dist(rng_) * params_.dt_seconds;
        v.vy += accel_dist(rng_) * params_.dt_seconds;
        v.vx = std::clamp(v.vx, -params_.speed_max_mps, params_.speed_max_mps);
        v.vy = std::clamp(v.vy, -params_.speed_max_mps, params_.speed_max_mps);
        v.x += v.vx * params_.dt_seconds;
        v.y += v.vy * params_.dt_seconds;
        if (v.x < -half || v.x > half) {
            v.vx = -v.vx;
            v.x = std::clamp(v.x, -half, half);
        }
        if (v.y < -half || v.y > half) {
            v.vy = -v.vy;
            v.y = std::clamp(v.y, -half, half);
        }

        double noisy_x = v.x + noise_dist(rng_);
        double noisy_y = v.y + noise_dist(rng_);
        LatLon pos = local_xy_to_latlon(geo_ref_, noisy_x, noisy_y);

        double speed_mps = std::hypot(v.vx, v.vy);
        double course_deg = std::fmod(std::atan2(v.vx, v.vy) * 180.0 / M_PI + 360.0, 360.0);

        AisReport report;
        report.mmsi = mmsis_[i];
        report.lat_deg = pos.lat_deg;
        report.lon_deg = pos.lon_deg;
        report.sog_knots = speed_mps * 1.9438452; // m/s -> knots
        report.cog_deg = course_deg;
        reports.push_back(report);
    }
    return reports;
}

} // namespace fusion
