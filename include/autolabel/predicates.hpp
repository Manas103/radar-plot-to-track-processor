#pragma once

#include "autolabel/types.hpp"

namespace autolabel {

// All nine predicates are built from a small shared vocabulary of geometric
// primitives (lateral offset from a lane centerline, heading difference,
// sustained longitudinal accel/decel, straight-line path-intersection time
// gap, perpendicular passing distance) rather than nine independent blobs of
// logic; the "composable" claim is this struct and the handful of helper
// functions in predicates.cpp that every detect_* below calls into.
struct Thresholds {
    double lane_change_min_lateral_m = 2.5;  // most of one lane width
    int lane_change_window_steps = 45;       // ~4.5s at dt=0.1s to complete a shift

    double hard_brake_decel_mps2 = 3.5;
    double high_accel_mps2 = 2.5;
    int accel_sustain_steps = 10; // ~1.0s at dt=0.1s

    double close_proximity_m = 6.0;
    int close_proximity_sustain_steps = 5; // ~0.5s

    double cyclist_passing_distance_m = 1.5;

    double crossing_heading_min_deg = 25.0;
    double crossing_heading_max_deg = 155.0;
    double crossing_time_gap_s = 1.5;
    double crossing_max_lookahead_s = 8.0;

    double unprotected_left_turn_rate_deg_s = 8.0;
    double unprotected_left_oncoming_speed_mps = 5.0;

    double cut_in_follow_gap_m = 10.0;
    int cut_in_window_steps = 50;

    double pullover_stop_speed_mps = 2.0; // speed at the end of the lateral shift must drop below this
};

struct Hit {
    int start_step = 0;
    int end_step = 0;
};

// Single-agent predicates: evaluated once per agent per candidate step, not
// gated by the pairwise grid prefilter (there is no second agent to be
// close to).
bool detect_lane_change(const AgentSpec& a, int t, double dt_s, const Thresholds& th, Hit* out);
bool detect_hard_brake(const AgentSpec& a, int t, double dt_s, const Thresholds& th, Hit* out);
bool detect_high_acceleration(const AgentSpec& a, int t, double dt_s, const Thresholds& th, Hit* out);

// A lateral shift (the same lane_shift_core lane_change and merge use)
// that ends with the agent's speed below pullover_stop_speed_mps: a
// vehicle easing toward the curb and coming to a near-stop, anywhere in
// the segment.
bool detect_pullover(const AgentSpec& a, int t, double dt_s, const Thresholds& th, Hit* out);

// Exactly detect_pullover's lateral-shift-plus-stop pattern, gated to a
// declared pickup/dropoff zone on the longitudinal axis (the same
// zone-gating idiom detect_merge uses for merge_zone_x_start_m/end_m): a
// pullover inside a flagged PUDO curb zone is a pickup/dropoff approach,
// the same maneuver shape with a different, location-dependent meaning.
bool detect_pickup_dropoff_approach(const AgentSpec& a, int t, double dt_s, const Thresholds& th,
                                     double pudo_zone_x_start_m, double pudo_zone_x_end_m, Hit* out);

// Pairwise predicates: only ever called on a pair the grid prefilter has
// already judged close enough at step t to be worth the extra arithmetic.
bool detect_crossing_paths(const AgentSpec& a, const AgentSpec& b, int t, double dt_s, const Thresholds& th,
                            Hit* out);
bool detect_unprotected_left(const AgentSpec& a, const AgentSpec& b, int t, double dt_s, const Thresholds& th,
                              Hit* out);
bool detect_cut_in(const AgentSpec& a, const AgentSpec& b, int t, double dt_s, const Thresholds& th, Hit* out);
bool detect_close_proximity(const AgentSpec& a, const AgentSpec& b, int t, double dt_s, const Thresholds& th,
                             Hit* out);
bool detect_merge(const AgentSpec& a, const AgentSpec& b, int t, double dt_s, const Thresholds& th,
                   double merge_zone_x_start_m, double merge_zone_x_end_m, Hit* out);
bool detect_cyclist_vehicle_close_approach(const AgentSpec& a, const AgentSpec& b, int t, double dt_s,
                                            const Thresholds& th, Hit* out);

} // namespace autolabel
