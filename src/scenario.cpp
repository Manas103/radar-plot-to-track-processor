#include "fusion/scenario.hpp"

#include "fusion/geo.hpp"

namespace fusion {

namespace {

radar::SimulatorParams make_radar_params(const FusionScenarioParams& p) {
    radar::SimulatorParams rp;
    rp.num_targets = p.num_radar_targets;
    rp.seed = p.seed;
    rp.area_size_m = p.area_size_m;
    rp.speed_max_mps = p.speed_max_mps;
    rp.maneuver_accel_sigma = p.maneuver_accel_sigma;
    rp.dt_seconds = p.dt_seconds;
    rp.probability_detect = 1.0;     // fusion scenarios gate detection realism in each feed's own sim
    rp.clutter_rate_per_scan = 0.0;  // clutter is already covered by the base tracker's own tests
    rp.measurement_sigma_m = p.radar_measurement_sigma_m;
    return rp;
}

AdsbSimParams make_adsb_params(const FusionScenarioParams& p) {
    AdsbSimParams ap;
    ap.num_aircraft = p.num_adsb_targets;
    ap.seed = p.seed + 1000; // distinct stream from the radar and AIS feeds
    ap.area_size_m = p.area_size_m;
    ap.dt_seconds = p.dt_seconds;
    return ap;
}

AisSimParams make_ais_params(const FusionScenarioParams& p) {
    AisSimParams sp;
    sp.num_vessels = p.num_ais_targets;
    sp.seed = p.seed + 2000; // distinct stream from the radar and ADS-B feeds
    sp.area_size_m = p.area_size_m;
    sp.dt_seconds = p.dt_seconds;
    return sp;
}

} // namespace

FusionScenario::FusionScenario(const FusionScenarioParams& params)
    : params_(params),
      radar_sim_(make_radar_params(params)),
      adsb_sim_(make_adsb_params(params), params.geo_ref),
      ais_sim_(make_ais_params(params), params.geo_ref) {}

FusedScanResult FusionScenario::next_scan() {
    ++scan_id_;
    FusedScanResult result;

    radar::ScanMessage radar_scan = radar_sim_.next_scan();
    for (std::uint32_t i = 0; i < radar_scan.count; ++i) {
        if (result.scan.count >= radar::kMaxTargets) break;
        result.scan.detections[result.scan.count] = radar_scan.detections[i];
        result.detection_source.push_back(Source::kRadar);
        ++result.scan.count;
    }

    for (const AdsbPositionReport& r : adsb_sim_.next_reports()) {
        if (!r.crc_ok) continue; // a frame that failed CRC carries no trustworthy position
        if (result.scan.count >= radar::kMaxTargets) break;
        double x, y;
        latlon_to_local_xy(params_.geo_ref, r.lat_deg, r.lon_deg, x, y);
        result.scan.detections[result.scan.count] = radar::Detection{x, y};
        result.detection_source.push_back(Source::kAdsb);
        ++result.scan.count;
    }

    for (const AisReport& r : ais_sim_.next_reports()) {
        if (result.scan.count >= radar::kMaxTargets) break;
        double x, y;
        latlon_to_local_xy(params_.geo_ref, r.lat_deg, r.lon_deg, x, y);
        result.scan.detections[result.scan.count] = radar::Detection{x, y};
        result.detection_source.push_back(Source::kAis);
        ++result.scan.count;
    }

    result.scan.scan_id = scan_id_;
    result.scan.timestamp_s = static_cast<double>(scan_id_) * params_.dt_seconds;
    return result;
}

} // namespace fusion
