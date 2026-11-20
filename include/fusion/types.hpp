#pragma once

#include <cstdint>

namespace fusion {

// A fused track picture's three feeds. Radar and AIS positions are
// already in, or go straight to, local tangent-plane meters; ADS-B
// positions are round-tripped through a real DF17 encode/decode (see
// adsb.hpp) before landing in the same local frame.
enum class Source : std::uint8_t { kRadar = 0, kAdsb = 1, kAis = 2 };

inline const char* source_name(Source s) {
    switch (s) {
        case Source::kRadar:
            return "radar";
        case Source::kAdsb:
            return "adsb";
        case Source::kAis:
            return "ais";
    }
    return "unknown";
}

// The local tangent-plane origin every feed's lat/lon projects through.
// Equirectangular, not a geodesic projection: accurate to well under a
// meter across the few-kilometer scenario areas this project uses, and
// not intended to be accurate at a scale where that approximation breaks
// down (see geo.hpp).
struct GeoRef {
    double ref_lat_deg = 0.0;
    double ref_lon_deg = 0.0;
};

struct LatLon {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
};

// Ground truth for one simulated target before its source's feed pipeline
// (direct passthrough for radar, lat/lon round trip for AIS, a real DF17
// encode/decode round trip for ADS-B) turns it into a Detection.
struct SourcedTarget {
    double x = 0.0; // local tangent-plane meters, ground truth
    double y = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    Source source = Source::kRadar;
};

} // namespace fusion
