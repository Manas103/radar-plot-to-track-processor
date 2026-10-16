#include "autolabel/scenario_gen.hpp"

#include <cmath>
#include <random>
#include <sstream>

namespace autolabel {

namespace {

// Closed-form longitudinal displacement contributed by one speed pulse at
// continuous time t_s, relative to the agent's base speed. The pulse is a
// symmetric triangular velocity deviation (ramp up over the first half,
// back down over the second half), so its integral (displacement) is a
// piecewise quadratic with no term requiring integration from t=0: this is
// what keeps pose_at O(1) per pulse instead of O(t).
double pulse_displacement(const SpeedPulseScript& p, double dt_s, double t_s) {
    const double pulse_start_s = p.t_start * dt_s;
    const double pulse_end_s = p.t_end * dt_s;
    if (t_s <= pulse_start_s) return 0.0;
    const double half_s = (pulse_end_s - pulse_start_s) / 2.0;
    const double pulse_mid_s = pulse_start_s + half_s;
    const double a = p.accel_mps2;
    if (t_s <= pulse_mid_s) {
        const double u = t_s - pulse_start_s;
        return 0.5 * a * u * u;
    }
    if (t_s <= pulse_end_s) {
        const double w = t_s - pulse_mid_s;
        return 0.5 * a * half_s * half_s + a * half_s * w - 0.5 * a * w * w;
    }
    return a * half_s * half_s;
}

double pulse_velocity_deviation(const SpeedPulseScript& p, double dt_s, double t_s) {
    const double pulse_start_s = p.t_start * dt_s;
    const double pulse_end_s = p.t_end * dt_s;
    if (t_s <= pulse_start_s || t_s >= pulse_end_s) return 0.0;
    const double half_s = (pulse_end_s - pulse_start_s) / 2.0;
    const double pulse_mid_s = pulse_start_s + half_s;
    const double a = p.accel_mps2;
    if (t_s <= pulse_mid_s) return a * (t_s - pulse_start_s);
    return a * (pulse_end_s - t_s);
}

double lane_lateral(const AgentSpec& agent, double dt_s, double t_s, double* lateral_velocity_out) {
    double lateral = agent.lane_index * agent.lane_width_m;
    double lateral_velocity = 0.0;
    for (const LaneChangeScript& lc : agent.lane_changes) {
        const double t_start_s = lc.t_start * dt_s;
        const double t_end_s = lc.t_end * dt_s;
        const double from_y = lc.lane_from * agent.lane_width_m;
        const double to_y = lc.lane_to * agent.lane_width_m;
        if (t_s <= t_start_s) {
            // No contribution yet; base lane_index already reflects lane_from
            // for scripts not yet started, by generator convention.
            continue;
        }
        if (t_s >= t_end_s) {
            lateral = to_y;
            continue;
        }
        const double frac = (t_s - t_start_s) / (t_end_s - t_start_s);
        lateral = from_y + frac * (to_y - from_y);
        lateral_velocity = (to_y - from_y) / (t_end_s - t_start_s);
    }
    if (lateral_velocity_out) *lateral_velocity_out = lateral_velocity;
    return lateral;
}

} // namespace

Pose pose_at(const AgentSpec& agent, int t, double dt_s) {
    const double t_s = t * dt_s;

    double dv = 0.0;
    double displacement = 0.0;
    for (const SpeedPulseScript& p : agent.speed_pulses) {
        dv += pulse_velocity_deviation(p, dt_s, t_s);
        displacement += pulse_displacement(p, dt_s, t_s);
    }
    const double longitudinal_speed = agent.base_speed_mps + dv;
    const double s = agent.x0 + agent.base_speed_mps * t_s + displacement;

    double lateral_velocity = 0.0;
    const double lateral = lane_lateral(agent, dt_s, t_s, &lateral_velocity);

    Pose pose;
    double vx = 0.0, vy = 0.0;
    if (agent.corridor_axis == 0) {
        pose.x = s;
        pose.y = lateral;
        vx = longitudinal_speed;
        vy = lateral_velocity;
    } else {
        pose.y = s;
        pose.x = lateral;
        vy = longitudinal_speed;
        vx = lateral_velocity;
    }
    pose.heading_rad = std::atan2(vy, vx);
    pose.speed_mps = std::sqrt(vx * vx + vy * vy);
    return pose;
}

Scenario make_bench_corpus(const BenchCorpusParams& p) {
    Scenario s;
    s.segment_id = p.segment_id;
    s.dt_s = p.dt_s;
    s.num_steps = p.num_steps;

    std::mt19937 rng(p.seed);
    std::uniform_int_distribution<int> lane_dist(0, 19); // 20-lane busy corridor
    std::uniform_real_distribution<double> speed_dist(11.0, 18.0);
    std::uniform_real_distribution<double> spacing_dist(0.0, 10.0);

    s.agents.reserve(p.num_agents);
    for (int i = 0; i < p.num_agents; ++i) {
        AgentSpec a;
        a.id = static_cast<std::uint32_t>(i + 1);
        a.type = AgentType::kVehicle;
        a.corridor_axis = 0;
        a.lane_index = lane_dist(rng);
        a.base_speed_mps = speed_dist(rng);
        // Agents are spread roughly one every ~10m per lane along a long
        // corridor so that, at any instant, only nearby-lane, nearby-x
        // agents are candidates: this is what gives the grid prefilter
        // something real to drop.
        a.x0 = static_cast<double>(i) * spacing_dist(rng) - static_cast<double>(p.num_agents) * 5.0;
        s.agents.push_back(a);
    }
    return s;
}

namespace {

void add_cruise_agent(Scenario& s, std::uint32_t id, AgentType type, int axis, int lane, double speed,
                       double x0) {
    AgentSpec a;
    a.id = id;
    a.type = type;
    a.corridor_axis = axis;
    a.lane_index = lane;
    a.base_speed_mps = speed;
    a.x0 = x0;
    s.agents.push_back(a);
}

} // namespace

std::vector<Scenario> make_validation_corpus(int num_segments, unsigned seed) {
    std::vector<Scenario> out;
    out.reserve(num_segments);

    for (int seg = 0; seg < num_segments; ++seg) {
        std::mt19937 rng(seed + static_cast<unsigned>(seg) * 7919u);
        std::uniform_real_distribution<double> jitter(-3.0, 3.0);

        Scenario s;
        s.segment_id = static_cast<std::uint32_t>(seg + 1);
        s.dt_s = 0.1;
        s.num_steps = 1200; // 120s per segment
        std::uint32_t next_id = 1;

        // Quiet background: a few cruising vehicles that never interact.
        for (int i = 0; i < 4; ++i) {
            add_cruise_agent(s, next_id++, AgentType::kVehicle, 0, i, 14.0 + jitter(rng),
                              -200.0 + i * 40.0);
        }

        // 1) lane_change: agent shifts from lane 5 to lane 6 at t=[200,245].
        {
            AgentSpec a;
            a.id = next_id++;
            a.corridor_axis = 0;
            a.lane_index = 5;
            a.base_speed_mps = 15.0;
            a.x0 = -50.0;
            a.lane_changes.push_back({200, 245, 5, 6});
            s.agents.push_back(a);
        }

        // 2) hard_brake: sustained decel pulse at t=[300,340].
        {
            AgentSpec a;
            a.id = next_id++;
            a.corridor_axis = 0;
            a.lane_index = 7;
            a.base_speed_mps = 16.0;
            a.x0 = -80.0;
            a.speed_pulses.push_back({300, 340, -4.0});
            s.agents.push_back(a);
        }

        // 3) high_acceleration: sustained positive accel pulse at t=[400,440].
        {
            AgentSpec a;
            a.id = next_id++;
            a.corridor_axis = 0;
            a.lane_index = 8;
            a.base_speed_mps = 10.0;
            a.x0 = -60.0;
            a.speed_pulses.push_back({400, 440, 3.0});
            s.agents.push_back(a);
        }

        // 4) crossing_paths + unprotected_left: a straight corridor-1 agent
        // ("oncoming" cross traffic) and a corridor-0 agent that lane-shifts
        // across it near the intersection at x=0, y=0, around t=600.
        {
            AgentSpec cross;
            cross.id = next_id++;
            cross.corridor_axis = 1; // travels along +y, crossing the main corridor
            cross.lane_index = 0;
            cross.base_speed_mps = 12.0;
            // y(t_s) = x0 + speed*t_s; want y(t_s=60s, i.e. t=600) == 0.
            cross.x0 = -(12.0 * 60.0);
            s.agents.push_back(cross);

            AgentSpec turning;
            turning.id = next_id++;
            turning.corridor_axis = 0;
            turning.lane_index = 2;
            turning.base_speed_mps = 9.0;
            turning.x0 = -(9.0 * 60.0); // reaches x=0 near t=600
            // A pronounced lateral maneuver right at the intersection stands
            // in for the turn arc (see README "Honest framing"): the turn-rate
            // predicate reads this as a heading deviation at a crossing, not
            // a literal quarter-circle.
            turning.lane_changes.push_back({585, 615, 2, -2});
            s.agents.push_back(turning);
        }

        // 5) cut_in: agent A lane-shifts into agent B's lane just ahead of B.
        {
            AgentSpec lead;
            lead.id = next_id++;
            lead.corridor_axis = 0;
            lead.lane_index = 10;
            lead.base_speed_mps = 13.0;
            lead.x0 = -40.0;
            s.agents.push_back(lead);

            AgentSpec cutter;
            cutter.id = next_id++;
            cutter.corridor_axis = 0;
            cutter.lane_index = 11;
            cutter.base_speed_mps = 14.5;
            // x0 chosen so the cutter is ~3m ahead of `lead` at the lane
            // change's midpoint (t=715, 71.5s): close enough in speed
            // (1.5 m/s relative) that this small gap does not drift past
            // Thresholds::cut_in_follow_gap_m by the time the predicate's
            // window closes at t+25.
            cutter.x0 = -144.25;
            cutter.lane_changes.push_back({700, 730, 11, 10});
            s.agents.push_back(cutter);
        }

        // 6) close_proximity: two agents in the same lane, closely spaced,
        // both cruising (no maneuver), for the full segment.
        {
            add_cruise_agent(s, next_id++, AgentType::kVehicle, 0, 14, 14.0, 10.0);
            add_cruise_agent(s, next_id++, AgentType::kVehicle, 0, 14, 14.0, 14.0);
        }

        // 7) merge: a lane_change whose window sits inside [400, 600] m, the
        // scenario's declared merge zone.
        {
            AgentSpec a;
            a.id = next_id++;
            a.corridor_axis = 0;
            a.lane_index = 15;
            a.base_speed_mps = 15.0;
            a.x0 = 400.0 - 15.0 * 90.0; // crosses x=400..600 around t=[900,945]
            a.lane_changes.push_back({900, 945, 15, 14});
            s.agents.push_back(a);
        }

        // 8) cyclist_vehicle_close_approach: a cyclist in lane 18, a vehicle
        // passing in lane 17 (adjacent) at a tight lateral gap.
        {
            AgentSpec cyclist;
            cyclist.id = next_id++;
            cyclist.type = AgentType::kCyclist;
            cyclist.corridor_axis = 0;
            cyclist.lane_width_m = 1.3; // narrow bike lane offset
            cyclist.lane_index = 18;
            cyclist.base_speed_mps = 5.0;
            cyclist.x0 = -30.0;
            s.agents.push_back(cyclist);

            AgentSpec passing_vehicle;
            passing_vehicle.id = next_id++;
            passing_vehicle.type = AgentType::kVehicle;
            passing_vehicle.corridor_axis = 0;
            passing_vehicle.lane_width_m = 1.3;
            passing_vehicle.lane_index = 17;
            passing_vehicle.base_speed_mps = 15.0;
            passing_vehicle.x0 = -60.0;
            s.agents.push_back(passing_vehicle);
        }

        out.push_back(s);
    }
    return out;
}

void write_scenario_corpus(std::ostream& os, const std::vector<Scenario>& corpus) {
    os << "AUTOLABEL_CORPUS v1\n";
    for (const Scenario& s : corpus) {
        os << "SEGMENT\t" << s.segment_id << '\t' << s.dt_s << '\t' << s.num_steps << '\n';
        for (const AgentSpec& a : s.agents) {
            os << "AGENT\t" << a.id << '\t' << static_cast<int>(a.type) << '\t' << a.corridor_axis << '\t'
               << a.lane_width_m << '\t' << a.lane_index << '\t' << a.base_speed_mps << '\t' << a.x0 << '\t';
            for (std::size_t i = 0; i < a.lane_changes.size(); ++i) {
                const LaneChangeScript& lc = a.lane_changes[i];
                if (i) os << ';';
                os << lc.t_start << ',' << lc.t_end << ',' << lc.lane_from << ',' << lc.lane_to;
            }
            os << '\t';
            for (std::size_t i = 0; i < a.speed_pulses.size(); ++i) {
                const SpeedPulseScript& p = a.speed_pulses[i];
                if (i) os << ';';
                os << p.t_start << ',' << p.t_end << ',' << p.accel_mps2;
            }
            os << '\n';
        }
    }
}

} // namespace autolabel
