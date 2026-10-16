#include "autolabel/predicates.hpp"

#include <algorithm>
#include <cmath>

#include "autolabel/scenario_gen.hpp"

namespace autolabel {

namespace {

constexpr double kPi = 3.14159265358979323846;

double wrap_to_pi(double rad) {
    while (rad > kPi) rad -= 2.0 * kPi;
    while (rad < -kPi) rad += 2.0 * kPi;
    return rad;
}

// Shared primitive 1: heading difference, wrapped, in degrees. Every
// predicate that cares about relative orientation (crossing_paths,
// unprotected_left) goes through this.
double heading_diff_deg(double ha, double hb) {
    return std::abs(wrap_to_pi(ha - hb)) * 180.0 / kPi;
}

// Shared primitive 2: Euclidean distance between two poses. Used by
// close_proximity and as the "nearby" gate inside
// cyclist_vehicle_close_approach.
double distance(const Pose& a, const Pose& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Shared primitive 3: which world coordinate is "lateral" for this agent
// depends only on which axis it travels along.
double lateral_coord(const AgentSpec& a, const Pose& p) { return a.corridor_axis == 0 ? p.y : p.x; }
double longitudinal_coord(const AgentSpec& a, const Pose& p) { return a.corridor_axis == 0 ? p.x : p.y; }

int clamp_step(int t) { return std::max(0, t); }

// Shared primitive 4: a sustained lateral shift over a window, used by both
// detect_lane_change (any zone) and detect_merge (gated to a declared x
// range). This is the one piece of logic both predicates compose from.
bool lane_shift_core(const AgentSpec& a, int t, double dt_s, const Thresholds& th, Hit* out,
                      double* mid_longitudinal_out) {
    const int half = std::max(1, th.lane_change_window_steps / 2);
    const int t0 = clamp_step(t - half);
    const int t1 = t + half;
    const Pose p0 = pose_at(a, t0, dt_s);
    const Pose p1 = pose_at(a, t1, dt_s);
    const double lateral_delta = std::abs(lateral_coord(a, p1) - lateral_coord(a, p0));
    if (lateral_delta < th.lane_change_min_lateral_m) return false;
    if (out) {
        out->start_step = t0;
        out->end_step = t1;
    }
    if (mid_longitudinal_out) {
        const Pose pm = pose_at(a, t, dt_s);
        *mid_longitudinal_out = longitudinal_coord(a, pm);
    }
    return true;
}

// Shared primitive 5: straight-line path intersection and the two agents'
// times to reach it. This is the one computation crossing_paths and
// unprotected_left both compose from.
struct CrossingSolution {
    bool crosses = false;
    double time_gap_s = 0.0;
    double heading_diff_deg = 0.0;
};

CrossingSolution solve_crossing(const Pose& pa, double speed_a_floor, double heading_a, const Pose& pb,
                                 double speed_b_floor, double heading_b, const Thresholds& th) {
    CrossingSolution out;
    const double da_x = std::cos(heading_a), da_y = std::sin(heading_a);
    const double db_x = std::cos(heading_b), db_y = std::sin(heading_b);
    const double det = da_x * (-db_y) - (-db_x) * da_y;
    if (std::abs(det) < 1e-6) return out; // parallel: no single crossing point

    const double rhs_x = pb.x - pa.x;
    const double rhs_y = pb.y - pa.y;
    const double s = (rhs_x * (-db_y) - (-db_x) * rhs_y) / det;
    const double u = (da_x * rhs_y - da_y * rhs_x) / det;
    if (s < 0.0 || u < 0.0) return out; // intersection is behind one of them

    if (speed_a_floor < 1e-3 || speed_b_floor < 1e-3) return out;
    const double t_a = s / speed_a_floor;
    const double t_b = u / speed_b_floor;
    if (t_a > th.crossing_max_lookahead_s || t_b > th.crossing_max_lookahead_s) return out;

    out.crosses = true;
    out.time_gap_s = std::abs(t_a - t_b);
    out.heading_diff_deg = heading_diff_deg(heading_a, heading_b);
    return out;
}

} // namespace

bool detect_lane_change(const AgentSpec& a, int t, double dt_s, const Thresholds& th, Hit* out) {
    return lane_shift_core(a, t, dt_s, th, out, nullptr);
}

bool detect_hard_brake(const AgentSpec& a, int t, double dt_s, const Thresholds& th, Hit* out) {
    const int sustain = th.accel_sustain_steps;
    const double s0 = pose_at(a, t, dt_s).speed_mps;
    const double s1 = pose_at(a, t + sustain, dt_s).speed_mps;
    const double accel = (s1 - s0) / (sustain * dt_s);
    if (accel > -th.hard_brake_decel_mps2) return false;
    if (out) {
        out->start_step = t;
        out->end_step = t + sustain;
    }
    return true;
}

bool detect_high_acceleration(const AgentSpec& a, int t, double dt_s, const Thresholds& th, Hit* out) {
    const int sustain = th.accel_sustain_steps;
    const double s0 = pose_at(a, t, dt_s).speed_mps;
    const double s1 = pose_at(a, t + sustain, dt_s).speed_mps;
    const double accel = (s1 - s0) / (sustain * dt_s);
    if (accel < th.high_accel_mps2) return false;
    if (out) {
        out->start_step = t;
        out->end_step = t + sustain;
    }
    return true;
}

bool detect_close_proximity(const AgentSpec& a, const AgentSpec& b, int t, double dt_s, const Thresholds& th,
                             Hit* out) {
    const int sustain = th.close_proximity_sustain_steps;
    const int mid = t + sustain / 2;
    for (int ti : {t, mid, t + sustain}) {
        if (distance(pose_at(a, ti, dt_s), pose_at(b, ti, dt_s)) > th.close_proximity_m) return false;
    }
    if (out) {
        out->start_step = t;
        out->end_step = t + sustain;
    }
    return true;
}

bool detect_crossing_paths(const AgentSpec& a, const AgentSpec& b, int t, double dt_s, const Thresholds& th,
                            Hit* out) {
    const Pose pa = pose_at(a, t, dt_s);
    const Pose pb = pose_at(b, t, dt_s);
    const CrossingSolution sol = solve_crossing(pa, pa.speed_mps, pa.heading_rad, pb, pb.speed_mps,
                                                 pb.heading_rad, th);
    if (!sol.crosses) return false;
    if (sol.heading_diff_deg < th.crossing_heading_min_deg || sol.heading_diff_deg > th.crossing_heading_max_deg)
        return false;
    if (sol.time_gap_s > th.crossing_time_gap_s) return false;
    if (out) {
        out->start_step = clamp_step(t - 5);
        out->end_step = t + 5;
    }
    return true;
}

bool detect_unprotected_left(const AgentSpec& a, const AgentSpec& b, int t, double dt_s, const Thresholds& th,
                              Hit* out) {
    const Pose pa = pose_at(a, t, dt_s);
    const Pose pb = pose_at(b, t, dt_s);
    const CrossingSolution sol = solve_crossing(pa, pa.speed_mps, pa.heading_rad, pb, pb.speed_mps,
                                                 pb.heading_rad, th);
    if (!sol.crosses) return false;
    if (sol.time_gap_s > th.crossing_time_gap_s) return false;

    const int half = 5;
    const double turn_rate_a =
        heading_diff_deg(pose_at(a, t + half, dt_s).heading_rad, pose_at(a, clamp_step(t - half), dt_s).heading_rad) /
        (2.0 * half * dt_s);
    const double turn_rate_b =
        heading_diff_deg(pose_at(b, t + half, dt_s).heading_rad, pose_at(b, clamp_step(t - half), dt_s).heading_rad) /
        (2.0 * half * dt_s);

    const bool a_turning = turn_rate_a >= th.unprotected_left_turn_rate_deg_s;
    const bool b_turning = turn_rate_b >= th.unprotected_left_turn_rate_deg_s;
    if (!a_turning && !b_turning) return false;

    const double oncoming_speed = a_turning ? pb.speed_mps : pa.speed_mps;
    if (oncoming_speed < th.unprotected_left_oncoming_speed_mps) return false;

    if (out) {
        out->start_step = clamp_step(t - half);
        out->end_step = t + half;
    }
    return true;
}

bool detect_cut_in(const AgentSpec& a, const AgentSpec& b, int t, double dt_s, const Thresholds& th, Hit* out) {
    if (a.corridor_axis != b.corridor_axis) return false;
    const int half = std::max(1, th.cut_in_window_steps / 2);
    const int t0 = clamp_step(t - half);
    const int t1 = t + half;

    const Pose a0 = pose_at(a, t0, dt_s), a1 = pose_at(a, t1, dt_s);
    const Pose b0 = pose_at(b, t0, dt_s), b1 = pose_at(b, t1, dt_s);

    const double lateral_gap0 = std::abs(lateral_coord(a, a0) - lateral_coord(b, b0));
    const double lateral_gap1 = std::abs(lateral_coord(a, a1) - lateral_coord(b, b1));
    const double different_lane_m = std::max(a.lane_width_m, b.lane_width_m) * 0.75;
    if (lateral_gap0 < different_lane_m) return false; // started in different lanes
    if (lateral_gap1 > different_lane_m * 0.3) return false; // ended up in (roughly) the same lane

    const double longitudinal_gap1 = longitudinal_coord(a, a1) - longitudinal_coord(b, b1);
    if (longitudinal_gap1 <= 0.0 || longitudinal_gap1 > th.cut_in_follow_gap_m) return false; // a must end up just ahead of b

    if (out) {
        out->start_step = t0;
        out->end_step = t1;
    }
    return true;
}

bool detect_merge(const AgentSpec& a, const AgentSpec& b, int t, double dt_s, const Thresholds& th,
                   double merge_zone_x_start_m, double merge_zone_x_end_m, Hit* out) {
    double mid_x = 0.0;
    if (a.corridor_axis == 0 && lane_shift_core(a, t, dt_s, th, out, &mid_x)) {
        if (mid_x >= merge_zone_x_start_m && mid_x <= merge_zone_x_end_m) return true;
    }
    if (b.corridor_axis == 0 && lane_shift_core(b, t, dt_s, th, out, &mid_x)) {
        if (mid_x >= merge_zone_x_start_m && mid_x <= merge_zone_x_end_m) return true;
    }
    return false;
}

bool detect_cyclist_vehicle_close_approach(const AgentSpec& a, const AgentSpec& b, int t, double dt_s,
                                            const Thresholds& th, Hit* out) {
    if (a.type == b.type) return false; // needs exactly one vehicle and one cyclist
    const AgentSpec& vehicle = (a.type == AgentType::kVehicle) ? a : b;
    const AgentSpec& cyclist = (a.type == AgentType::kCyclist) ? a : b;

    const Pose pv = pose_at(vehicle, t, dt_s);
    const Pose pc = pose_at(cyclist, t, dt_s);
    const double wx = pc.x - pv.x, wy = pc.y - pv.y;
    const double ux = std::cos(pv.heading_rad), uy = std::sin(pv.heading_rad);
    const double lateral_offset = std::abs(ux * wy - uy * wx);
    const double overall_distance = distance(pv, pc);

    if (overall_distance > th.close_proximity_m * 1.5) return false;
    if (lateral_offset > th.cyclist_passing_distance_m) return false;

    if (out) {
        out->start_step = clamp_step(t - 5);
        out->end_step = t + 5;
    }
    return true;
}

} // namespace autolabel
