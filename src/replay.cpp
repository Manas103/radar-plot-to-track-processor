#include "radar/replay.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace radar {

void write_scan_log_header(std::ostream& os) {
    // setprecision(10) is enough to round-trip sub-millimeter precision
    // across the 0-20km simulated area used in this project, and the
    // setting persists on the stream for every subsequent write_scan call.
    os << std::setprecision(10);
    os << "RADAR_SCAN_LOG v1\n";
}

void write_scan(std::ostream& os, const ScanMessage& scan) {
    os << std::setprecision(10);
    os << "SCAN " << scan.scan_id << ' ' << scan.timestamp_s << ' ' << scan.count << '\n';
    for (std::uint32_t i = 0; i < scan.count; ++i) {
        os << scan.detections[i].x << ' ' << scan.detections[i].y << '\n';
    }
}

bool read_next_scan(std::istream& is, ScanMessage& out) {
    std::string token;
    // Skip the header line if present; otherwise land on the next "SCAN".
    while (is >> token) {
        if (token == "RADAR_SCAN_LOG") {
            std::string version;
            is >> version;
            continue;
        }
        if (token == "SCAN") break;
        return false; // unexpected token, treat as end of usable data
    }
    if (!is || token != "SCAN") return false;

    std::uint64_t scan_id = 0;
    double ts = 0.0;
    std::uint32_t count = 0;
    if (!(is >> scan_id >> ts >> count)) return false;

    out = ScanMessage{};
    out.scan_id = scan_id;
    out.timestamp_s = ts;
    out.count = std::min<std::uint32_t>(count, kMaxTargets);
    for (std::uint32_t i = 0; i < count; ++i) {
        double x = 0.0, y = 0.0;
        if (!(is >> x >> y)) return false;
        if (i < kMaxTargets) {
            out.detections[i].x = x;
            out.detections[i].y = y;
        }
    }
    return true;
}

std::string canonical_scan_line(const ScanResult& result, const std::vector<Track>& tracks) {
    std::vector<std::uint32_t> confirmed_ids;
    for (const auto& t : tracks) {
        if (t.status == TrackStatus::kConfirmed) confirmed_ids.push_back(t.id);
    }
    std::sort(confirmed_ids.begin(), confirmed_ids.end());

    std::ostringstream oss;
    oss << "S " << result.scan_id << " C" << confirmed_ids.size() << " T" << result.num_tentative
        << " IDS:";
    for (std::size_t i = 0; i < confirmed_ids.size(); ++i) {
        if (i) oss << ',';
        oss << confirmed_ids[i];
    }
    oss << '\n';
    return oss.str();
}

} // namespace radar
