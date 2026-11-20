#include "fusion/geo.hpp"

#include <cmath>

namespace fusion {

LatLon local_xy_to_latlon(const GeoRef& ref, double x, double y) {
    double ref_lat_rad = ref.ref_lat_deg * M_PI / 180.0;
    double lat_deg = ref.ref_lat_deg + (y / kEarthRadiusM) * (180.0 / M_PI);
    double lon_deg = ref.ref_lon_deg + (x / (kEarthRadiusM * std::cos(ref_lat_rad))) * (180.0 / M_PI);
    return {lat_deg, lon_deg};
}

void latlon_to_local_xy(const GeoRef& ref, double lat_deg, double lon_deg, double& x, double& y) {
    double ref_lat_rad = ref.ref_lat_deg * M_PI / 180.0;
    y = (lat_deg - ref.ref_lat_deg) * (M_PI / 180.0) * kEarthRadiusM;
    x = (lon_deg - ref.ref_lon_deg) * (M_PI / 180.0) * kEarthRadiusM * std::cos(ref_lat_rad);
}

} // namespace fusion
