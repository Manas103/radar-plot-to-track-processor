#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace autolabel {

enum class AgentType : std::uint8_t {
    kVehicle = 0,
    kCyclist = 1,
};

// One instant of an agent's state. Produced on demand by Scenario::pose_at,
// never materialized for a whole track: the benchmark corpus is large enough
// (up to 150 agents x 100,000 steps) that storing every pose would dominate
// memory, and a closed-form generator is also what makes two independent
// runs over the same seed bit-for-bit identical without any stored state to
// get out of sync.
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double heading_rad = 0.0;
    double speed_mps = 0.0;
};

// A scripted lateral maneuver: agent shifts from lane_from to lane_from+1 (or
// -1) linearly over [t_start, t_end], used to plant lane_change, merge and
// cut_in instances at known times for the validation corpus.
struct LaneChangeScript {
    int t_start = 0;
    int t_end = 0;
    int lane_from = 0;
    int lane_to = 0;
};

// A scripted longitudinal speed pulse: linear ramp to accel_mps2 and back,
// used to plant hard_brake and high_acceleration instances.
struct SpeedPulseScript {
    int t_start = 0;
    int t_end = 0;
    double accel_mps2 = 0.0; // negative: braking, positive: accelerating
};

// One agent's full, closed-form definition. pose_at(t) is pure and
// deterministic: same t, same Pose, forever, from any thread.
struct AgentSpec {
    std::uint32_t id = 0;
    AgentType type = AgentType::kVehicle;
    double lane_width_m = 3.5;
    int lane_index = 0;             // for corridor_axis == 0: y = lane_index * lane_width_m
    int corridor_axis = 0;          // 0: travels along +x, 1: travels along +y (crossing traffic)
    double base_speed_mps = 14.0;   // ~31mph nominal cruise
    double x0 = 0.0;                // position at t=0 along the travel axis
    double phase_noise = 0.0;       // small deterministic per-agent offset, not randomness per step
    std::vector<LaneChangeScript> lane_changes;
    std::vector<SpeedPulseScript> speed_pulses;
};

// One driving segment: a fixed cast of agents and a duration, dt-stepped.
struct Scenario {
    std::uint32_t segment_id = 0;
    double dt_s = 0.1;
    int num_steps = 0;
    std::vector<AgentSpec> agents;
};

enum class EventClass : std::uint8_t {
    kLaneChange = 0,
    kCrossingPaths = 1,
    kUnprotectedLeft = 2,
    kCutIn = 3,
    kHardBrake = 4,
    kCloseProximity = 5,
    kMerge = 6,
    kHighAcceleration = 7,
    kCyclistVehicleCloseApproach = 8,
    kPickupDropoffApproach = 9,
    kPullover = 10,
};

constexpr int kNumEventClasses = 11;

const char* event_class_name(EventClass c);

// A mined (or ground-truth) event. For single-agent predicates (lane_change,
// hard_brake, high_acceleration), agent_b == agent_a.
struct Event {
    std::uint32_t segment_id = 0;
    std::uint32_t agent_a = 0;
    std::uint32_t agent_b = 0;
    EventClass event_class = EventClass::kLaneChange;
    int start_step = 0;
    int end_step = 0;

    bool operator==(const Event& o) const;
};

// Canonical, deterministic text form of one event (used both for the TSV
// output and for the bit-for-bit replay hash).
std::string canonical_event_line(const Event& e);

} // namespace autolabel
