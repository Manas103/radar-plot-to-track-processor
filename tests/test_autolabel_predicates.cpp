#include <gtest/gtest.h>

#include <cmath>

#include "autolabel/predicates.hpp"
#include "autolabel/scenario_gen.hpp"

using namespace autolabel;

namespace {
constexpr double kDt = 0.1;
AgentSpec cruiser(std::uint32_t id, int lane, double speed, double x0, int axis = 0) {
    AgentSpec a;
    a.id = id;
    a.corridor_axis = axis;
    a.lane_index = lane;
    a.base_speed_mps = speed;
    a.x0 = x0;
    return a;
}
} // namespace

TEST(Predicates, LaneChangeFiresOnScriptedShift) {
    AgentSpec a = cruiser(1, 5, 15.0, -50.0);
    a.lane_changes.push_back({200, 245, 5, 6});
    Thresholds th;
    Hit hit;
    bool found = false;
    for (int t = 195; t < 260 && !found; ++t) found = detect_lane_change(a, t, kDt, th, &hit);
    EXPECT_TRUE(found);
}

TEST(Predicates, LaneChangeDoesNotFireOnStraightCruise) {
    AgentSpec a = cruiser(1, 5, 15.0, -50.0);
    Thresholds th;
    Hit hit;
    for (int t = 0; t < 2000; t += 20) EXPECT_FALSE(detect_lane_change(a, t, kDt, th, &hit));
}

TEST(Predicates, HardBrakeFiresOnDecelPulse) {
    AgentSpec a = cruiser(1, 5, 16.0, -50.0);
    a.speed_pulses.push_back({300, 340, -4.0});
    Thresholds th;
    Hit hit;
    EXPECT_TRUE(detect_hard_brake(a, 305, kDt, th, &hit));
}

TEST(Predicates, HighAccelerationFiresOnAccelPulse) {
    AgentSpec a = cruiser(1, 5, 10.0, -50.0);
    a.speed_pulses.push_back({400, 440, 3.0});
    Thresholds th;
    Hit hit;
    EXPECT_TRUE(detect_high_acceleration(a, 405, kDt, th, &hit));
}

TEST(Predicates, HardBrakeAndHighAccelAreMutuallyExclusiveOnCruise) {
    AgentSpec a = cruiser(1, 5, 14.0, -50.0);
    Thresholds th;
    Hit hit;
    EXPECT_FALSE(detect_hard_brake(a, 500, kDt, th, &hit));
    EXPECT_FALSE(detect_high_acceleration(a, 500, kDt, th, &hit));
}

TEST(Predicates, CloseProximityFiresWhenAgentsStaySub6m) {
    AgentSpec a = cruiser(1, 14, 14.0, 10.0);
    AgentSpec b = cruiser(2, 14, 14.0, 14.0); // 4m ahead, same lane, same speed: gap never changes
    Thresholds th;
    Hit hit;
    EXPECT_TRUE(detect_close_proximity(a, b, 100, kDt, th, &hit));
}

TEST(Predicates, CloseProximityDoesNotFireWhenFar) {
    AgentSpec a = cruiser(1, 0, 14.0, 0.0);
    AgentSpec b = cruiser(2, 19, 14.0, 0.0); // far lane, same x
    Thresholds th;
    Hit hit;
    EXPECT_FALSE(detect_close_proximity(a, b, 100, kDt, th, &hit));
}

TEST(Predicates, CrossingPathsFiresOnPerpendicularApproach) {
    AgentSpec a = cruiser(1, 2, 9.0, -(9.0 * 60.0), 0);
    AgentSpec b = cruiser(2, 0, 12.0, -(12.0 * 60.0), 1);
    Thresholds th;
    Hit hit;
    EXPECT_TRUE(detect_crossing_paths(a, b, 600, kDt, th, &hit));
}

TEST(Predicates, CrossingPathsDoesNotFireOnParallelTraffic) {
    AgentSpec a = cruiser(1, 2, 14.0, 0.0, 0);
    AgentSpec b = cruiser(2, 3, 14.0, 0.0, 0);
    Thresholds th;
    Hit hit;
    EXPECT_FALSE(detect_crossing_paths(a, b, 100, kDt, th, &hit));
}

TEST(Predicates, UnprotectedLeftFiresWhenTurningAgentCrossesOncomingTraffic) {
    AgentSpec turning = cruiser(1, 2, 9.0, -(9.0 * 60.0), 0);
    turning.lane_changes.push_back({585, 615, 2, -2});
    AgentSpec oncoming = cruiser(2, 0, 12.0, -(12.0 * 60.0), 1);
    Thresholds th;
    Hit hit;
    bool found = false;
    for (int t = 590; t < 615 && !found; ++t)
        found = detect_unprotected_left(turning, oncoming, t, kDt, th, &hit);
    EXPECT_TRUE(found);
}

TEST(Predicates, CutInFiresWhenAgentShiftsJustAheadOfFollower) {
    AgentSpec lead = cruiser(1, 10, 13.0, -40.0);
    AgentSpec cutter = cruiser(2, 11, 14.5, -144.25);
    cutter.lane_changes.push_back({700, 730, 11, 10});
    Thresholds th;
    Hit hit;
    EXPECT_TRUE(detect_cut_in(cutter, lead, 715, kDt, th, &hit));
}

TEST(Predicates, CutInDoesNotFireWithoutALaneShift) {
    AgentSpec lead = cruiser(1, 10, 13.0, -40.0);
    AgentSpec follower = cruiser(2, 10, 13.0, -55.0);
    Thresholds th;
    Hit hit;
    EXPECT_FALSE(detect_cut_in(follower, lead, 715, kDt, th, &hit));
}

TEST(Predicates, MergeFiresInsideDeclaredZone) {
    AgentSpec a = cruiser(1, 15, 15.0, 400.0 - 15.0 * 90.0);
    a.lane_changes.push_back({900, 945, 15, 14});
    Thresholds th;
    Hit hit;
    bool found = false;
    for (int t = 895; t < 950 && !found; ++t) found = detect_merge(a, a, t, kDt, th, 400.0, 600.0, &hit);
    EXPECT_TRUE(found);
}

TEST(Predicates, MergeDoesNotFireOutsideDeclaredZone) {
    AgentSpec a = cruiser(1, 15, 15.0, -2000.0);
    a.lane_changes.push_back({900, 945, 15, 14});
    Thresholds th;
    Hit hit;
    bool found = false;
    for (int t = 895; t < 950; ++t) found = found || detect_merge(a, a, t, kDt, th, 400.0, 600.0, &hit);
    EXPECT_FALSE(found);
}

TEST(Predicates, CyclistVehicleCloseApproachFiresOnTightPass) {
    AgentSpec cyclist;
    cyclist.id = 1;
    cyclist.type = AgentType::kCyclist;
    cyclist.lane_width_m = 1.3;
    cyclist.lane_index = 18;
    cyclist.base_speed_mps = 5.0;
    cyclist.x0 = -30.0;

    AgentSpec vehicle;
    vehicle.id = 2;
    vehicle.type = AgentType::kVehicle;
    vehicle.lane_width_m = 1.3;
    vehicle.lane_index = 17;
    vehicle.base_speed_mps = 15.0;
    vehicle.x0 = -30.0; // aligned so the pair is lateral-only at t=0

    Thresholds th;
    Hit hit;
    EXPECT_TRUE(detect_cyclist_vehicle_close_approach(vehicle, cyclist, 0, kDt, th, &hit));
}

TEST(Predicates, CyclistVehicleCloseApproachRequiresOneOfEachType) {
    AgentSpec v1 = cruiser(1, 17, 15.0, -30.0);
    AgentSpec v2 = cruiser(2, 18, 15.0, -30.0);
    Thresholds th;
    Hit hit;
    EXPECT_FALSE(detect_cyclist_vehicle_close_approach(v1, v2, 0, kDt, th, &hit));
}
