#pragma once

#include <cstdint>
#include <string>

namespace fusion::cot {

// One Cursor-on-Target event, the shape `build_event` renders to XML and
// `cot_schema.xsd` (docs/) validates. `time_s` is seconds since a fixed
// scenario epoch (2026-01-01T00:00:00Z), not wall-clock time: this is a
// synthetic scenario, not a live feed, and the README says so.
struct Event {
    std::string uid;
    std::string type;       // CoT 2525-style type, e.g. "a-u-A" (air), "a-u-S" (surface)
    double time_s = 0.0;    // event generation time
    double stale_s = 0.0;   // when this report should be considered stale
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double course_deg = 0.0;
    double speed_mps = 0.0;
    std::string remarks;
};

// Formats `epoch_seconds_since_2026` as "YYYY-MM-DDTHH:MM:SSZ" against the
// fixed scenario epoch 2026-01-01T00:00:00Z.
std::string format_cot_time(double epoch_seconds_since_2026);

// Renders one schema-valid Cursor-on-Target 2.0 event XML document.
std::string build_event(const Event& event);

} // namespace fusion::cot
