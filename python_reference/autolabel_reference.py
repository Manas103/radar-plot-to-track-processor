"""Independent, from-scratch Python re-implementation of the autolabel
predicates (see include/autolabel/predicates.hpp). Shares no code with the
C++ side; the only thing shared is the scenario file format
(autolabel/scenario_gen.hpp's write_scenario_corpus), which is agent
parameters, not predicate logic. Used by run_oracle_comparison.py as the
independent reference oracle for the C++ miner.

This brute-forces every agent pair at every timestep (no grid prefilter):
it is a correctness oracle, not a throughput benchmark, and the validation
corpus is small enough (about 20 segments x 16 agents x 1200 steps) that
brute force runs in a few seconds.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

EVENT_CLASSES = [
    "lane_change",
    "crossing_paths",
    "unprotected_left",
    "cut_in",
    "hard_brake",
    "close_proximity",
    "merge",
    "high_acceleration",
    "cyclist_vehicle_close_approach",
    "pickup_dropoff_approach",
    "pullover",
]

VEHICLE, CYCLIST = 0, 1


@dataclass
class LaneChange:
    t_start: int
    t_end: int
    lane_from: int
    lane_to: int


@dataclass
class SpeedPulse:
    t_start: int
    t_end: int
    accel: float


@dataclass
class Agent:
    id: int
    type: int
    axis: int
    lane_width: float
    lane_index: int
    base_speed: float
    x0: float
    lane_changes: list = field(default_factory=list)
    speed_pulses: list = field(default_factory=list)


@dataclass
class Segment:
    segment_id: int
    dt_s: float
    num_steps: int
    agents: list


def load_corpus(path: str):
    segments = []
    with open(path, "r") as f:
        lines = f.read().splitlines()
    assert lines[0] == "AUTOLABEL_CORPUS v1"
    seg = None
    for line in lines[1:]:
        if not line:
            continue
        parts = line.split("\t")
        if parts[0] == "SEGMENT":
            if seg is not None:
                segments.append(seg)
            seg = Segment(int(parts[1]), float(parts[2]), int(parts[3]), [])
        elif parts[0] == "AGENT":
            lane_changes = []
            if parts[8]:
                for token in parts[8].split(";"):
                    a, b, c, d = token.split(",")
                    lane_changes.append(LaneChange(int(a), int(b), int(c), int(d)))
            speed_pulses = []
            if parts[9]:
                for token in parts[9].split(";"):
                    a, b, c = token.split(",")
                    speed_pulses.append(SpeedPulse(int(a), int(b), float(c)))
            agent = Agent(
                id=int(parts[1]),
                type=int(parts[2]),
                axis=int(parts[3]),
                lane_width=float(parts[4]),
                lane_index=int(parts[5]),
                base_speed=float(parts[6]),
                x0=float(parts[7]),
                lane_changes=lane_changes,
                speed_pulses=speed_pulses,
            )
            seg.agents.append(agent)
    if seg is not None:
        segments.append(seg)
    return segments


def pulse_velocity_deviation(p: SpeedPulse, dt_s: float, t_s: float) -> float:
    start_s, end_s = p.t_start * dt_s, p.t_end * dt_s
    if t_s <= start_s or t_s >= end_s:
        return 0.0
    half_s = (end_s - start_s) / 2.0
    mid_s = start_s + half_s
    if t_s <= mid_s:
        return p.accel * (t_s - start_s)
    return p.accel * (end_s - t_s)


def pulse_displacement(p: SpeedPulse, dt_s: float, t_s: float) -> float:
    start_s, end_s = p.t_start * dt_s, p.t_end * dt_s
    if t_s <= start_s:
        return 0.0
    half_s = (end_s - start_s) / 2.0
    mid_s = start_s + half_s
    a = p.accel
    if t_s <= mid_s:
        u = t_s - start_s
        return 0.5 * a * u * u
    if t_s <= end_s:
        w = t_s - mid_s
        return 0.5 * a * half_s * half_s + a * half_s * w - 0.5 * a * w * w
    return a * half_s * half_s


def lane_lateral(agent: Agent, dt_s: float, t_s: float):
    lateral = agent.lane_index * agent.lane_width
    lateral_velocity = 0.0
    for lc in agent.lane_changes:
        start_s, end_s = lc.t_start * dt_s, lc.t_end * dt_s
        from_y, to_y = lc.lane_from * agent.lane_width, lc.lane_to * agent.lane_width
        if t_s <= start_s:
            continue
        if t_s >= end_s:
            lateral = to_y
            continue
        frac = (t_s - start_s) / (end_s - start_s)
        lateral = from_y + frac * (to_y - from_y)
        lateral_velocity = (to_y - from_y) / (end_s - start_s)
    return lateral, lateral_velocity


def pose_at(agent: Agent, t: int, dt_s: float):
    t_s = t * dt_s
    dv = sum(pulse_velocity_deviation(p, dt_s, t_s) for p in agent.speed_pulses)
    disp = sum(pulse_displacement(p, dt_s, t_s) for p in agent.speed_pulses)
    longitudinal_speed = agent.base_speed + dv
    s = agent.x0 + agent.base_speed * t_s + disp
    lateral, lateral_velocity = lane_lateral(agent, dt_s, t_s)

    if agent.axis == 0:
        x, y = s, lateral
        vx, vy = longitudinal_speed, lateral_velocity
    else:
        y, x = s, lateral
        vy, vx = longitudinal_speed, lateral_velocity

    heading = math.atan2(vy, vx)
    speed = math.hypot(vx, vy)
    return x, y, heading, speed


# ---- thresholds (mirrors Thresholds in predicates.hpp) ----
TH = dict(
    lane_change_min_lateral_m=2.5,
    lane_change_window_steps=45,
    hard_brake_decel=3.5,
    high_accel=2.5,
    accel_sustain_steps=10,
    close_proximity_m=6.0,
    close_proximity_sustain_steps=5,
    cyclist_passing_distance_m=1.5,
    crossing_heading_min_deg=25.0,
    crossing_heading_max_deg=155.0,
    crossing_time_gap_s=1.5,
    crossing_max_lookahead_s=8.0,
    unprotected_left_turn_rate_deg_s=8.0,
    unprotected_left_oncoming_speed_mps=5.0,
    cut_in_follow_gap_m=10.0,
    cut_in_window_steps=50,
    pullover_stop_speed_mps=2.0,
)


def heading_diff_deg(ha, hb):
    d = ha - hb
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return abs(d) * 180.0 / math.pi


def lateral_coord(agent: Agent, pose):
    return pose[1] if agent.axis == 0 else pose[0]


def longitudinal_coord(agent: Agent, pose):
    return pose[0] if agent.axis == 0 else pose[1]


def detect_lane_change(a: Agent, t: int, dt_s: float):
    half = max(1, TH["lane_change_window_steps"] // 2)
    t0, t1 = max(0, t - half), t + half
    p0, p1 = pose_at(a, t0, dt_s), pose_at(a, t1, dt_s)
    delta = abs(lateral_coord(a, p1) - lateral_coord(a, p0))
    return (t0, t1) if delta >= TH["lane_change_min_lateral_m"] else None


def detect_hard_brake(a: Agent, t: int, dt_s: float):
    sustain = TH["accel_sustain_steps"]
    s0, s1 = pose_at(a, t, dt_s)[3], pose_at(a, t + sustain, dt_s)[3]
    accel = (s1 - s0) / (sustain * dt_s)
    return (t, t + sustain) if accel <= -TH["hard_brake_decel"] else None


def detect_high_acceleration(a: Agent, t: int, dt_s: float):
    sustain = TH["accel_sustain_steps"]
    s0, s1 = pose_at(a, t, dt_s)[3], pose_at(a, t + sustain, dt_s)[3]
    accel = (s1 - s0) / (sustain * dt_s)
    return (t, t + sustain) if accel >= TH["high_accel"] else None


def detect_close_proximity(a: Agent, b: Agent, t: int, dt_s: float):
    sustain = TH["close_proximity_sustain_steps"]
    mid = t + sustain // 2
    for ti in (t, mid, t + sustain):
        pa, pb = pose_at(a, ti, dt_s), pose_at(b, ti, dt_s)
        if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) > TH["close_proximity_m"]:
            return None
    return (t, t + sustain)


def solve_crossing(pa, pb):
    da = (math.cos(pa[2]), math.sin(pa[2]))
    db = (math.cos(pb[2]), math.sin(pb[2]))
    det = da[0] * (-db[1]) - (-db[0]) * da[1]
    if abs(det) < 1e-6:
        return None
    rhs = (pb[0] - pa[0], pb[1] - pa[1])
    s = (rhs[0] * (-db[1]) - (-db[0]) * rhs[1]) / det
    u = (da[0] * rhs[1] - da[1] * rhs[0]) / det
    if s < 0.0 or u < 0.0:
        return None
    speed_a, speed_b = pa[3], pb[3]
    if speed_a < 1e-3 or speed_b < 1e-3:
        return None
    t_a, t_b = s / speed_a, u / speed_b
    if t_a > TH["crossing_max_lookahead_s"] or t_b > TH["crossing_max_lookahead_s"]:
        return None
    return abs(t_a - t_b), heading_diff_deg(pa[2], pb[2])


def detect_crossing_paths(a: Agent, b: Agent, t: int, dt_s: float):
    pa, pb = pose_at(a, t, dt_s), pose_at(b, t, dt_s)
    sol = solve_crossing(pa, pb)
    if sol is None:
        return None
    time_gap, heading_diff = sol
    if not (TH["crossing_heading_min_deg"] <= heading_diff <= TH["crossing_heading_max_deg"]):
        return None
    if time_gap > TH["crossing_time_gap_s"]:
        return None
    return (max(0, t - 5), t + 5)


def detect_unprotected_left(a: Agent, b: Agent, t: int, dt_s: float):
    pa, pb = pose_at(a, t, dt_s), pose_at(b, t, dt_s)
    sol = solve_crossing(pa, pb)
    if sol is None:
        return None
    time_gap, _ = sol
    if time_gap > TH["crossing_time_gap_s"]:
        return None
    half = 5
    ha0, ha1 = pose_at(a, max(0, t - half), dt_s)[2], pose_at(a, t + half, dt_s)[2]
    hb0, hb1 = pose_at(b, max(0, t - half), dt_s)[2], pose_at(b, t + half, dt_s)[2]
    rate_a = heading_diff_deg(ha1, ha0) / (2 * half * dt_s)
    rate_b = heading_diff_deg(hb1, hb0) / (2 * half * dt_s)
    a_turning = rate_a >= TH["unprotected_left_turn_rate_deg_s"]
    b_turning = rate_b >= TH["unprotected_left_turn_rate_deg_s"]
    if not a_turning and not b_turning:
        return None
    oncoming_speed = pb[3] if a_turning else pa[3]
    if oncoming_speed < TH["unprotected_left_oncoming_speed_mps"]:
        return None
    return (max(0, t - half), t + half)


def detect_cut_in(a: Agent, b: Agent, t: int, dt_s: float):
    if a.axis != b.axis:
        return None
    half = max(1, TH["cut_in_window_steps"] // 2)
    t0, t1 = max(0, t - half), t + half
    a0, a1 = pose_at(a, t0, dt_s), pose_at(a, t1, dt_s)
    b0, b1 = pose_at(b, t0, dt_s), pose_at(b, t1, dt_s)
    lateral_gap0 = abs(lateral_coord(a, a0) - lateral_coord(b, b0))
    lateral_gap1 = abs(lateral_coord(a, a1) - lateral_coord(b, b1))
    different_lane = max(a.lane_width, b.lane_width) * 0.75
    if lateral_gap0 < different_lane:
        return None
    if lateral_gap1 > different_lane * 0.3:
        return None
    gap1 = longitudinal_coord(a, a1) - longitudinal_coord(b, b1)
    if not (0.0 < gap1 <= TH["cut_in_follow_gap_m"]):
        return None
    return (t0, t1)


def detect_merge(a: Agent, t: int, dt_s: float, zone_start: float, zone_end: float):
    if a.axis != 0:
        return None
    hit = detect_lane_change(a, t, dt_s)
    if hit is None:
        return None
    mid_x = pose_at(a, t, dt_s)[0]
    if zone_start <= mid_x <= zone_end:
        return hit
    return None


def detect_pullover(a: Agent, t: int, dt_s: float):
    hit = detect_lane_change(a, t, dt_s)
    if hit is None:
        return None
    end_speed = pose_at(a, hit[1], dt_s)[3]
    return hit if end_speed <= TH["pullover_stop_speed_mps"] else None


def detect_pickup_dropoff_approach(a: Agent, t: int, dt_s: float, zone_start: float, zone_end: float):
    hit = detect_lane_change(a, t, dt_s)
    if hit is None:
        return None
    mid_x = pose_at(a, t, dt_s)[0]
    if not (zone_start <= mid_x <= zone_end):
        return None
    end_speed = pose_at(a, hit[1], dt_s)[3]
    return hit if end_speed <= TH["pullover_stop_speed_mps"] else None


def detect_cyclist_vehicle_close_approach(a: Agent, b: Agent, t: int, dt_s: float):
    if a.type == b.type:
        return None
    vehicle, cyclist = (a, b) if a.type == VEHICLE else (b, a)
    pv, pc = pose_at(vehicle, t, dt_s), pose_at(cyclist, t, dt_s)
    wx, wy = pc[0] - pv[0], pc[1] - pv[1]
    ux, uy = math.cos(pv[2]), math.sin(pv[2])
    lateral_offset = abs(ux * wy - uy * wx)
    overall = math.hypot(wx, wy)
    if overall > TH["close_proximity_m"] * 1.5:
        return None
    if lateral_offset > TH["cyclist_passing_distance_m"]:
        return None
    return (max(0, t - 5), t + 5)


def _sorted_pair(ia, ib):
    return (ia, ib) if ia <= ib else (ib, ia)


def mine_segment(seg: Segment):
    """Brute-force, no grid prefilter: returns a dict event_class -> set of
    (agent_a, agent_b) pairs with at least one detected occurrence. Exact
    start/end step agreement with the C++ side is not required (the two
    implementations merge overlapping windows independently); which
    (segment, pair, class) triples fired is what is compared.
    """
    hits = {c: set() for c in EVENT_CLASSES}
    agents = seg.agents
    n = len(agents)
    for t in range(seg.num_steps):
        for a in agents:
            if detect_lane_change(a, t, seg.dt_s):
                hits["lane_change"].add((a.id, a.id))
            if detect_hard_brake(a, t, seg.dt_s):
                hits["hard_brake"].add((a.id, a.id))
            if detect_high_acceleration(a, t, seg.dt_s):
                hits["high_acceleration"].add((a.id, a.id))
            if detect_merge(a, t, seg.dt_s, 400.0, 600.0):
                hits["merge"].add((a.id, a.id))
            # Same precedence as the C++ miner: a hit inside the declared
            # PUDO zone is recorded as pickup_dropoff_approach only, not
            # also as a generic pullover.
            if detect_pickup_dropoff_approach(a, t, seg.dt_s, 1200.0, 1400.0):
                hits["pickup_dropoff_approach"].add((a.id, a.id))
            elif detect_pullover(a, t, seg.dt_s):
                hits["pullover"].add((a.id, a.id))
        for i in range(n):
            for j in range(i + 1, n):
                a, b = agents[i], agents[j]
                pair = _sorted_pair(a.id, b.id)
                if detect_close_proximity(a, b, t, seg.dt_s):
                    hits["close_proximity"].add(pair)
                if detect_crossing_paths(a, b, t, seg.dt_s):
                    hits["crossing_paths"].add(pair)
                if detect_unprotected_left(a, b, t, seg.dt_s):
                    hits["unprotected_left"].add(pair)
                if detect_cut_in(a, b, t, seg.dt_s) or detect_cut_in(b, a, t, seg.dt_s):
                    hits["cut_in"].add(pair)
                if detect_cyclist_vehicle_close_approach(a, b, t, seg.dt_s):
                    hits["cyclist_vehicle_close_approach"].add(pair)
    return hits
