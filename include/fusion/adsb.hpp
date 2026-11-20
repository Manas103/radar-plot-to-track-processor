#pragma once

#include <array>
#include <cstdint>

namespace fusion::adsb {

// A 112-bit (14-byte) Mode S extended squitter frame.
using Frame = std::array<std::uint8_t, 14>;

// DF17 airborne-position ME subtype used by both encode and decode; fixed
// so a round trip only has to agree on CPR lat/lon, not on which subtype
// produced the frame.
constexpr int kAirbornePositionTypeCode = 11;

// Mode S CRC-24: generator polynomial 0xFFF409, computed bit by bit over
// the first 88 bits (DF+CA+ICAO+ME) with a standard LFSR, the same
// algorithm the telemetry-ground-station sibling's modes_crc24.hpp uses
// for the same message type. Believed correct per widely published Mode S
// documentation; validated here by round trip and by an independent
// Python re-implementation, not against externally captured RF (see
// README "Honest framing").
std::uint32_t crc24(const Frame& frame);

// Encodes a synthetic DF17 airborne-position frame: `icao24` the 24-bit
// ICAO address, `lat_deg`/`lon_deg` the true position, `odd` selects the
// even/odd CPR format bit. This is this project's only way to produce a
// frame (there is no real receiver on this machine); `decode_position`
// below is the real part, a genuine implementation of the public DF17
// decode algorithm.
Frame encode_airborne_position(std::uint32_t icao24, double lat_deg, double lon_deg, bool odd);

struct DecodedFrame {
    bool crc_ok = false;
    std::uint32_t icao24 = 0;
    bool odd = false;
    double lat_deg = 0.0;
    double lon_deg = 0.0;
};

// Decodes a DF17 airborne-position frame using *local* (not globally
// unambiguous) CPR decoding: given one frame and a known reference
// position within about 300 nautical miles (ICAO Annex 10's stated bound
// for local decode), this recovers position without needing a matched
// even/odd frame pair. This fused tracker's own GeoRef origin is exactly
// such a known reference (a ground station near the scenario area), which
// is the real-world case this simpler decode mode is for.
DecodedFrame decode_airborne_position(const Frame& frame, double ref_lat_deg, double ref_lon_deg);

// ICAO Annex 10's "number of longitude zones" function, degree 15 (NZ=15),
// used by both CPR encode and decode so a change to one cannot silently
// diverge from the other.
double cpr_nl(double lat_deg);

} // namespace fusion::adsb
