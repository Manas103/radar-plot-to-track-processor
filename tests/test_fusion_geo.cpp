#include "fusion/geo.hpp"

#include <gtest/gtest.h>

using namespace fusion;

TEST(Geo, RoundTripWithinScenarioArea) {
    GeoRef ref{37.5, -122.3};
    for (double x : {-10000.0, -1234.5, 0.0, 5678.9, 10000.0}) {
        for (double y : {-10000.0, -2222.2, 0.0, 3333.3, 10000.0}) {
            LatLon ll = local_xy_to_latlon(ref, x, y);
            double rx, ry;
            latlon_to_local_xy(ref, ll.lat_deg, ll.lon_deg, rx, ry);
            EXPECT_NEAR(rx, x, 1e-6);
            EXPECT_NEAR(ry, y, 1e-6);
        }
    }
}

TEST(Geo, OriginMapsToTheReferencePoint) {
    GeoRef ref{10.0, 20.0};
    LatLon ll = local_xy_to_latlon(ref, 0.0, 0.0);
    EXPECT_NEAR(ll.lat_deg, 10.0, 1e-9);
    EXPECT_NEAR(ll.lon_deg, 20.0, 1e-9);
}

TEST(Geo, NorthIsPositiveYAndEastIsPositiveX) {
    GeoRef ref{0.0, 0.0};
    LatLon north = local_xy_to_latlon(ref, 0.0, 1000.0);
    LatLon east = local_xy_to_latlon(ref, 1000.0, 0.0);
    EXPECT_GT(north.lat_deg, ref.ref_lat_deg);
    EXPECT_NEAR(north.lon_deg, ref.ref_lon_deg, 1e-9);
    EXPECT_GT(east.lon_deg, ref.ref_lon_deg);
    EXPECT_NEAR(east.lat_deg, ref.ref_lat_deg, 1e-9);
}
