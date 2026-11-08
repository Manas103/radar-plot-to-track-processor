#include "autolabel/types.hpp"

#include <sstream>

namespace autolabel {

const char* event_class_name(EventClass c) {
    switch (c) {
        case EventClass::kLaneChange: return "lane_change";
        case EventClass::kCrossingPaths: return "crossing_paths";
        case EventClass::kUnprotectedLeft: return "unprotected_left";
        case EventClass::kCutIn: return "cut_in";
        case EventClass::kHardBrake: return "hard_brake";
        case EventClass::kCloseProximity: return "close_proximity";
        case EventClass::kMerge: return "merge";
        case EventClass::kHighAcceleration: return "high_acceleration";
        case EventClass::kCyclistVehicleCloseApproach: return "cyclist_vehicle_close_approach";
        case EventClass::kPickupDropoffApproach: return "pickup_dropoff_approach";
        case EventClass::kPullover: return "pullover";
    }
    return "unknown";
}

bool Event::operator==(const Event& o) const {
    return segment_id == o.segment_id && agent_a == o.agent_a && agent_b == o.agent_b &&
           event_class == o.event_class && start_step == o.start_step && end_step == o.end_step;
}

std::string canonical_event_line(const Event& e) {
    std::ostringstream os;
    os << e.segment_id << '\t' << e.agent_a << '\t' << e.agent_b << '\t' << event_class_name(e.event_class)
       << '\t' << e.start_step << '\t' << e.end_step << '\n';
    return os.str();
}

} // namespace autolabel
