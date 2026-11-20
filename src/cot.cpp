#include "fusion/cot.hpp"

#include <cstdio>
#include <ctime>

namespace fusion::cot {

namespace {

std::time_t scenario_epoch_2026_01_01() {
    // Computed via timegm rather than a hardcoded Unix timestamp, so this
    // is never at risk of a misremembered magic constant.
    std::tm tm{};
    tm.tm_year = 2026 - 1900;
    tm.tm_mon = 0;
    tm.tm_mday = 1;
    tm.tm_hour = 0;
    tm.tm_min = 0;
    tm.tm_sec = 0;
    return timegm(&tm);
}

std::string xml_escape(const std::string& in) {
    std::string out;
    out.reserve(in.size());
    for (char c : in) {
        switch (c) {
            case '&':
                out += "&amp;";
                break;
            case '<':
                out += "&lt;";
                break;
            case '>':
                out += "&gt;";
                break;
            case '"':
                out += "&quot;";
                break;
            default:
                out += c;
        }
    }
    return out;
}

} // namespace

std::string format_cot_time(double epoch_seconds_since_2026) {
    std::time_t t = scenario_epoch_2026_01_01() + static_cast<std::time_t>(epoch_seconds_since_2026);
    std::tm utc{};
    gmtime_r(&t, &utc);
    char buf[48];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ", utc.tm_year + 1900, utc.tm_mon + 1,
                  utc.tm_mday, utc.tm_hour, utc.tm_min, utc.tm_sec);
    return std::string(buf);
}

std::string build_event(const Event& event) {
    char point_buf[256];
    std::snprintf(point_buf, sizeof(point_buf), "<point lat=\"%.6f\" lon=\"%.6f\" hae=\"9999999.0\" ce=\"9999999.0\" le=\"9999999.0\"/>",
                  event.lat_deg, event.lon_deg);

    char track_buf[128];
    std::snprintf(track_buf, sizeof(track_buf), "<track course=\"%.1f\" speed=\"%.2f\"/>", event.course_deg,
                  event.speed_mps);

    std::string xml;
    xml += "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n";
    xml += "<event version=\"2.0\" uid=\"" + xml_escape(event.uid) + "\" type=\"" + xml_escape(event.type) +
           "\" time=\"" + format_cot_time(event.time_s) + "\" start=\"" + format_cot_time(event.time_s) +
           "\" stale=\"" + format_cot_time(event.stale_s) + "\" how=\"m-g\">\n";
    xml += "  " + std::string(point_buf) + "\n";
    xml += "  <detail>\n";
    xml += "    <contact callsign=\"" + xml_escape(event.uid) + "\"/>\n";
    xml += "    " + std::string(track_buf) + "\n";
    if (!event.remarks.empty()) {
        xml += "    <remarks>" + xml_escape(event.remarks) + "</remarks>\n";
    }
    xml += "  </detail>\n";
    xml += "</event>\n";
    return xml;
}

} // namespace fusion::cot
