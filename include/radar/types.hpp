#pragma once

#include <array>
#include <cstdint>

namespace radar {

// Upper bound on targets per scan. Sized for the 500-target claim with a
// little headroom; the ring buffer and ScanMessage size are both derived
// from this constant.
constexpr int kMaxTargets = 600;

// A single noisy detection in radar-plane Cartesian coordinates (meters),
// already de-rotated from range/bearing by the simulated front end.
struct Detection {
    double x = 0.0;
    double y = 0.0;
};

// One full antenna revolution's worth of detections, timestamped by the
// pacing clock (virtual or wall). This is the unit carried across the
// detection-producer -> tracking-thread ring buffer, and the unit recorded
// to / replayed from a session log.
struct ScanMessage {
    std::uint64_t scan_id = 0;
    double timestamp_s = 0.0;
    std::uint32_t count = 0;
    std::array<Detection, kMaxTargets> detections{};
};

enum class TrackStatus : std::uint8_t {
    kTentative = 0,
    kConfirmed = 1,
    kDeleted = 2,
};

} // namespace radar
