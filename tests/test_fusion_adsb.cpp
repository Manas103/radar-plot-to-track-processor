#include "fusion/adsb.hpp"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "fusion/geo.hpp"

using namespace fusion;
using namespace fusion::adsb;

TEST(Adsb, RoundTripRecoversPositionWithinCprQuantization) {
    GeoRef ref{37.5, -122.3};
    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> dxy(-10000.0, 10000.0);

    double max_err_m = 0.0;
    for (int i = 0; i < 2000; ++i) {
        double x = dxy(rng);
        double y = dxy(rng);
        LatLon truth = local_xy_to_latlon(ref, x, y);
        bool odd = (i % 2) == 1;
        Frame f = encode_airborne_position(0xABCDEF, truth.lat_deg, truth.lon_deg, odd);
        DecodedFrame d = decode_airborne_position(f, ref.ref_lat_deg, ref.ref_lon_deg);
        ASSERT_TRUE(d.crc_ok);
        double dx, dy;
        latlon_to_local_xy(ref, d.lat_deg, d.lon_deg, dx, dy);
        max_err_m = std::max(max_err_m, std::hypot(dx - x, dy - y));
    }
    // CPR's 17-bit quantization resolves to roughly 5m at this latitude;
    // a correct decode should never be far outside that.
    EXPECT_LT(max_err_m, 10.0);
}

TEST(Adsb, TamperedCrcIsDetected) {
    Frame f = encode_airborne_position(0x123456, 37.51, -122.29, false);
    f[0] ^= 0x01;
    DecodedFrame d = decode_airborne_position(f, 37.5, -122.3);
    EXPECT_FALSE(d.crc_ok);
}

TEST(Adsb, UntamperedCrcPasses) {
    Frame f = encode_airborne_position(0x123456, 37.51, -122.29, false);
    DecodedFrame d = decode_airborne_position(f, 37.5, -122.3);
    EXPECT_TRUE(d.crc_ok);
}

TEST(Adsb, DecodedIcaoMatchesEncoded) {
    Frame f = encode_airborne_position(0x00ABCD, 40.0, -74.0, true);
    DecodedFrame d = decode_airborne_position(f, 40.0, -74.0);
    EXPECT_EQ(d.icao24, 0x00ABCDu);
}

TEST(Adsb, OddFormatBitRoundTrips) {
    Frame even_frame = encode_airborne_position(0x1, 30.0, -90.0, false);
    Frame odd_frame = encode_airborne_position(0x1, 30.0, -90.0, true);
    DecodedFrame even_decoded = decode_airborne_position(even_frame, 30.0, -90.0);
    DecodedFrame odd_decoded = decode_airborne_position(odd_frame, 30.0, -90.0);
    EXPECT_FALSE(even_decoded.odd);
    EXPECT_TRUE(odd_decoded.odd);
}

TEST(Adsb, CprNlIsLargestNearTheEquatorAndShrinksTowardThePoles) {
    double nl_equator = cpr_nl(0.0);
    double nl_mid = cpr_nl(45.0);
    double nl_high = cpr_nl(80.0);
    EXPECT_GT(nl_equator, nl_mid);
    EXPECT_GT(nl_mid, nl_high);
    EXPECT_GE(nl_high, 1.0);
}
