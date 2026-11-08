#include "autolabel/miner.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "autolabel/scenario_gen.hpp"

namespace autolabel {

namespace {

using Clock = std::chrono::steady_clock;

struct RawHit {
    std::uint32_t agent_a;
    std::uint32_t agent_b;
    EventClass event_class;
    int start_step;
    int end_step;
};

void push_sorted_pair(std::uint32_t id_i, std::uint32_t id_j, std::uint32_t* a, std::uint32_t* b) {
    *a = std::min(id_i, id_j);
    *b = std::max(id_i, id_j);
}

void scan_pairwise_range(const Scenario& scenario, const MineParams& params, int t_begin, int t_end,
                          std::vector<RawHit>* hits, ScanStats* stats) {
    const std::size_t n = scenario.agents.size();
    std::vector<Pose> poses(n);
    std::vector<CellCoord> cells(n);
    Hit hit;

    for (int t = t_begin; t < t_end; ++t) {
        for (std::size_t i = 0; i < n; ++i) {
            poses[i] = pose_at(scenario.agents[i], t, scenario.dt_s);
            cells[i] = cell_of(poses[i], params.prefilter);
        }
        for (std::size_t i = 0; i < n; ++i) {
            for (std::size_t j = i + 1; j < n; ++j) {
                stats->pairs_scanned++;
                if (!cells_adjacent(cells[i], cells[j])) continue;
                stats->pairs_candidate++;

                const AgentSpec& a = scenario.agents[i];
                const AgentSpec& b = scenario.agents[j];
                std::uint32_t id_a, id_b;
                push_sorted_pair(a.id, b.id, &id_a, &id_b);

                if (detect_close_proximity(a, b, t, scenario.dt_s, params.thresholds, &hit))
                    hits->push_back({id_a, id_b, EventClass::kCloseProximity, hit.start_step, hit.end_step});
                if (detect_crossing_paths(a, b, t, scenario.dt_s, params.thresholds, &hit))
                    hits->push_back({id_a, id_b, EventClass::kCrossingPaths, hit.start_step, hit.end_step});
                if (detect_unprotected_left(a, b, t, scenario.dt_s, params.thresholds, &hit))
                    hits->push_back(
                        {id_a, id_b, EventClass::kUnprotectedLeft, hit.start_step, hit.end_step});
                // cut_in is asymmetric (one agent ends up ahead of the
                // other), but which of a pair is agents[i] vs agents[j] is
                // an accident of generation order, not meaning; try both
                // role assignments.
                if (detect_cut_in(a, b, t, scenario.dt_s, params.thresholds, &hit) ||
                    detect_cut_in(b, a, t, scenario.dt_s, params.thresholds, &hit))
                    hits->push_back({id_a, id_b, EventClass::kCutIn, hit.start_step, hit.end_step});
                if (a.type != b.type &&
                    detect_cyclist_vehicle_close_approach(a, b, t, scenario.dt_s, params.thresholds, &hit))
                    hits->push_back(
                        {id_a, id_b, EventClass::kCyclistVehicleCloseApproach, hit.start_step, hit.end_step});
            }
        }
    }
}

void scan_single_agent_range(const Scenario& scenario, const MineParams& params, int t_begin, int t_end,
                              std::vector<RawHit>* hits) {
    Hit hit;
    for (int t = t_begin; t < t_end; ++t) {
        for (const AgentSpec& a : scenario.agents) {
            if (detect_lane_change(a, t, scenario.dt_s, params.thresholds, &hit))
                hits->push_back({a.id, a.id, EventClass::kLaneChange, hit.start_step, hit.end_step});
            if (detect_hard_brake(a, t, scenario.dt_s, params.thresholds, &hit))
                hits->push_back({a.id, a.id, EventClass::kHardBrake, hit.start_step, hit.end_step});
            if (detect_high_acceleration(a, t, scenario.dt_s, params.thresholds, &hit))
                hits->push_back({a.id, a.id, EventClass::kHighAcceleration, hit.start_step, hit.end_step});
            // detect_merge takes a pairwise-shaped signature only because it
            // shares lane_shift_core with detect_lane_change; the merge
            // decision itself needs only one agent and the declared zone, so
            // it is run here, in the single-agent sweep, with the same agent
            // in both slots, rather than depending on an arbitrary
            // grid-adjacent neighbor being a candidate at the same instant.
            if (detect_merge(a, a, t, scenario.dt_s, params.thresholds, params.merge_zone_x_start_m,
                              params.merge_zone_x_end_m, &hit))
                hits->push_back({a.id, a.id, EventClass::kMerge, hit.start_step, hit.end_step});
            // detect_pickup_dropoff_approach is checked before detect_pullover
            // on the same timestep: a lateral-shift-plus-stop inside the
            // declared PUDO zone is a pickup/dropoff approach specifically,
            // not also a generic pullover, so a hit here skips the
            // broader pullover check for this (agent, t).
            if (detect_pickup_dropoff_approach(a, t, scenario.dt_s, params.thresholds, params.pudo_zone_x_start_m,
                                                params.pudo_zone_x_end_m, &hit)) {
                hits->push_back({a.id, a.id, EventClass::kPickupDropoffApproach, hit.start_step, hit.end_step});
            } else if (detect_pullover(a, t, scenario.dt_s, params.thresholds, &hit)) {
                hits->push_back({a.id, a.id, EventClass::kPullover, hit.start_step, hit.end_step});
            }
        }
    }
}

std::vector<std::pair<int, int>> partition_range(int total, int num_parts) {
    std::vector<std::pair<int, int>> parts;
    num_parts = std::max(1, num_parts);
    const int chunk = (total + num_parts - 1) / num_parts;
    for (int start = 0; start < total; start += chunk) {
        parts.emplace_back(start, std::min(total, start + chunk));
    }
    return parts;
}

// Merges raw per-timestep hits into one event per contiguous (or
// near-contiguous) occurrence. A maneuver that trips a predicate at many
// consecutive candidate timesteps (its detection window slides) must not
// become dozens of near-duplicate events.
std::vector<Event> merge_hits(std::uint32_t segment_id, std::vector<RawHit>& hits) {
    std::sort(hits.begin(), hits.end(), [](const RawHit& x, const RawHit& y) {
        if (x.agent_a != y.agent_a) return x.agent_a < y.agent_a;
        if (x.agent_b != y.agent_b) return x.agent_b < y.agent_b;
        if (x.event_class != y.event_class) return x.event_class < y.event_class;
        return x.start_step < y.start_step;
    });

    std::vector<Event> events;
    constexpr int kMergeGapSteps = 10;
    for (const RawHit& h : hits) {
        if (!events.empty()) {
            Event& last = events.back();
            if (last.agent_a == h.agent_a && last.agent_b == h.agent_b && last.event_class == h.event_class &&
                h.start_step <= last.end_step + kMergeGapSteps) {
                last.end_step = std::max(last.end_step, h.end_step);
                continue;
            }
        }
        Event e;
        e.segment_id = segment_id;
        e.agent_a = h.agent_a;
        e.agent_b = h.agent_b;
        e.event_class = h.event_class;
        e.start_step = h.start_step;
        e.end_step = h.end_step;
        events.push_back(e);
    }
    return events;
}

} // namespace

MineResult mine_scenario(const Scenario& scenario, const MineParams& params) {
    MineResult result;
    std::vector<RawHit> all_hits;

    if (params.run_pairwise) {
        const auto parts = partition_range(scenario.num_steps, params.num_threads);
        std::vector<std::vector<RawHit>> per_thread_hits(parts.size());
        std::vector<ScanStats> per_thread_stats(parts.size());
        std::vector<std::thread> threads;
        const auto t0 = Clock::now();
        for (std::size_t k = 0; k < parts.size(); ++k) {
            threads.emplace_back(scan_pairwise_range, std::cref(scenario), std::cref(params), parts[k].first,
                                  parts[k].second, &per_thread_hits[k], &per_thread_stats[k]);
        }
        for (auto& th : threads) th.join();
        result.pairwise_elapsed_s = std::chrono::duration<double>(Clock::now() - t0).count();

        for (std::size_t k = 0; k < parts.size(); ++k) {
            result.pairwise_scan.pairs_scanned += per_thread_stats[k].pairs_scanned;
            result.pairwise_scan.pairs_candidate += per_thread_stats[k].pairs_candidate;
            all_hits.insert(all_hits.end(), per_thread_hits[k].begin(), per_thread_hits[k].end());
        }
    }

    if (params.run_single_agent) {
        const auto t0 = Clock::now();
        scan_single_agent_range(scenario, params, 0, scenario.num_steps, &all_hits);
        result.single_agent_elapsed_s = std::chrono::duration<double>(Clock::now() - t0).count();
    }

    result.events = merge_hits(scenario.segment_id, all_hits);
    std::sort(result.events.begin(), result.events.end(), [](const Event& x, const Event& y) {
        if (x.segment_id != y.segment_id) return x.segment_id < y.segment_id;
        if (x.agent_a != y.agent_a) return x.agent_a < y.agent_a;
        if (x.agent_b != y.agent_b) return x.agent_b < y.agent_b;
        if (x.event_class != y.event_class) return x.event_class < y.event_class;
        return x.start_step < y.start_step;
    });
    return result;
}

MineResult mine_corpus(const std::vector<Scenario>& scenarios, const MineParams& params) {
    MineResult total;
    for (const Scenario& s : scenarios) {
        MineResult r = mine_scenario(s, params);
        total.events.insert(total.events.end(), r.events.begin(), r.events.end());
        total.pairwise_scan.pairs_scanned += r.pairwise_scan.pairs_scanned;
        total.pairwise_scan.pairs_candidate += r.pairwise_scan.pairs_candidate;
        total.pairwise_elapsed_s += r.pairwise_elapsed_s;
        total.single_agent_elapsed_s += r.single_agent_elapsed_s;
    }
    return total;
}

} // namespace autolabel
