#pragma once

#include "fusion/types.hpp"

namespace fusion {

constexpr double kEarthRadiusM = 6371000.0;

// Equirectangular local tangent-plane projection, not a geodesic one.
// Honest over the few-kilometer scenario areas this project uses; not
// claimed to be accurate at any larger scale.
LatLon local_xy_to_latlon(const GeoRef& ref, double x, double y);
void latlon_to_local_xy(const GeoRef& ref, double lat_deg, double lon_deg, double& x, double& y);

} // namespace fusion
