#include "fusion/adsb.hpp"

#include <algorithm>
#include <cmath>

namespace fusion::adsb {

namespace {

constexpr std::uint32_t kGeneratorPolynomial = 0xFFF409; // Mode S CRC-24 generator

void set_bits(Frame& frame, int start_bit, int num_bits, std::uint32_t value) {
    for (int i = 0; i < num_bits; ++i) {
        int bit_value = (value >> (num_bits - 1 - i)) & 1;
        int global_bit = start_bit + i;
        int byte_idx = global_bit / 8;
        int bit_in_byte = 7 - (global_bit % 8);
        if (bit_value) {
            frame[byte_idx] |= static_cast<std::uint8_t>(1u << bit_in_byte);
        } else {
            frame[byte_idx] &= static_cast<std::uint8_t>(~(1u << bit_in_byte));
        }
    }
}

std::uint32_t get_bits(const Frame& frame, int start_bit, int num_bits) {
    std::uint32_t value = 0;
    for (int i = 0; i < num_bits; ++i) {
        int global_bit = start_bit + i;
        int byte_idx = global_bit / 8;
        int bit_in_byte = 7 - (global_bit % 8);
        int bit_value = (frame[byte_idx] >> bit_in_byte) & 1;
        value = (value << 1) | static_cast<std::uint32_t>(bit_value);
    }
    return value;
}

// Always non-negative, unlike std::fmod for a negative `a`.
double wrap_mod(double a, double b) {
    double r = std::fmod(a, b);
    if (r < 0.0) {
        r += b;
    }
    return r;
}

} // namespace

std::uint32_t crc24(const Frame& frame) {
    // Standard bit-at-a-time LFSR CRC: the first 88 bits (DF+CA+ICAO+ME)
    // are the message; the register's final value is the 24-bit
    // remainder, with no need to separately append 24 zero bits, the same
    // shape as any textbook bit-serial CRC.
    std::uint32_t reg = 0;
    for (int i = 0; i < 88; ++i) {
        int byte_idx = i / 8;
        int bit_idx = 7 - (i % 8);
        int bit = (frame[byte_idx] >> bit_idx) & 1;
        int top_bit = (reg >> 23) & 1;
        reg = ((reg << 1) | static_cast<std::uint32_t>(bit)) & 0xFFFFFF;
        if (top_bit) {
            reg ^= kGeneratorPolynomial;
        }
    }
    return reg & 0xFFFFFF;
}

double cpr_nl(double lat_deg) {
    double lat = std::fabs(lat_deg);
    if (lat >= 87.0) {
        return 1.0;
    }
    if (lat < 1e-9) {
        return 59.0;
    }
    constexpr double kNZ = 15.0;
    double a = 1.0 - std::cos(M_PI / (2.0 * kNZ));
    double b = std::cos(M_PI / 180.0 * lat);
    b = b * b;
    double val = 1.0 - a / b;
    val = std::max(-1.0, std::min(1.0, val));
    double nl = std::floor(2.0 * M_PI / std::acos(val));
    return std::max(1.0, nl);
}

Frame encode_airborne_position(std::uint32_t icao24, double lat_deg, double lon_deg, bool odd) {
    Frame frame{};
    set_bits(frame, 0, 5, 17);     // DF17
    set_bits(frame, 5, 3, 5);      // CA, an arbitrary valid capability code
    set_bits(frame, 8, 24, icao24);
    set_bits(frame, 32, 5, kAirbornePositionTypeCode); // TC
    set_bits(frame, 37, 2, 0);     // SS
    set_bits(frame, 39, 1, 0);     // NIC supplement
    set_bits(frame, 40, 12, 0);    // altitude code, unused by this decode
    set_bits(frame, 52, 1, 0);     // T
    set_bits(frame, 53, 1, odd ? 1 : 0); // F, the CPR even/odd format bit

    double dlat = odd ? 360.0 / 59.0 : 360.0 / 60.0;
    double lat_mod = wrap_mod(lat_deg, dlat);
    std::uint32_t lat_cpr = static_cast<std::uint32_t>(std::floor(lat_mod / dlat * 131072.0 + 0.5)) & 0x1FFFF;

    double nl = cpr_nl(lat_deg);
    double ni = std::max(nl - (odd ? 1.0 : 0.0), 1.0);
    double dlon = 360.0 / ni;
    double lon_mod = wrap_mod(lon_deg, dlon);
    std::uint32_t lon_cpr = static_cast<std::uint32_t>(std::floor(lon_mod / dlon * 131072.0 + 0.5)) & 0x1FFFF;

    set_bits(frame, 54, 17, lat_cpr);
    set_bits(frame, 71, 17, lon_cpr);

    std::uint32_t crc = crc24(frame);
    set_bits(frame, 88, 24, crc);
    return frame;
}

DecodedFrame decode_airborne_position(const Frame& frame, double ref_lat_deg, double ref_lon_deg) {
    DecodedFrame out;

    std::uint32_t computed = crc24(frame);
    std::uint32_t transmitted = get_bits(frame, 88, 24);
    out.crc_ok = (computed == transmitted);
    out.icao24 = get_bits(frame, 8, 24);

    bool odd = get_bits(frame, 53, 1) != 0;
    out.odd = odd;

    double lat_cpr = static_cast<double>(get_bits(frame, 54, 17)) / 131072.0;
    double lon_cpr = static_cast<double>(get_bits(frame, 71, 17)) / 131072.0;

    // Local (not globally unambiguous) CPR decode: valid when the
    // supplied reference is within the same latitude/longitude zone as
    // the transmitting target, true by construction for this project's
    // scenario areas (see adsb.hpp).
    double dlat = odd ? 360.0 / 59.0 : 360.0 / 60.0;
    double ref_lat_mod = wrap_mod(ref_lat_deg, dlat);
    double j = std::floor(ref_lat_deg / dlat) + std::round(ref_lat_mod / dlat - lat_cpr);
    double lat = dlat * (j + lat_cpr);

    double nl = cpr_nl(lat);
    double ni = std::max(nl - (odd ? 1.0 : 0.0), 1.0);
    double dlon = 360.0 / ni;
    double ref_lon_mod = wrap_mod(ref_lon_deg, dlon);
    double m = std::floor(ref_lon_deg / dlon) + std::round(ref_lon_mod / dlon - lon_cpr);
    double lon = dlon * (m + lon_cpr);

    out.lat_deg = lat;
    out.lon_deg = lon;
    return out;
}

} // namespace fusion::adsb
