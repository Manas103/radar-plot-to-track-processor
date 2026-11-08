#pragma once

#include <cstdint>
#include <ostream>
#include <vector>

#include "autolabel/types.hpp"

namespace autolabel {

// pose_at evaluates an agent's closed-form trajectory at step t: constant
// cruise speed along its corridor axis, modified by any LaneChangeScript
// (linear lateral interpolation between the two lane centerlines) and any
// SpeedPulseScript (a linear accel ramp up and back down) that covers t.
// Pure function of (agent, t, dt_s): no RNG, no mutable state, so the same
// call from any thread, in any order, any number of times, returns the
// identical Pose.
Pose pose_at(const AgentSpec& agent, int t, double dt_s);

struct BenchCorpusParams {
    std::uint32_t segment_id = 1;
    int num_agents = 150;
    int num_steps = 100'000;
    double dt_s = 0.1;
    unsigned seed = 1;
};

// The large, single-segment throughput/prefilter-rate benchmark corpus: a
// busy multi-lane corridor with no scripted maneuvers. This is deliberately
// not a realistic driving segment (150 simultaneous agents in one segment
// is far denser than a real logged scene); it exists to measure scan
// throughput and the grid prefilter's drop rate at a fixed, reproducible
// scale, not to exercise predicate correctness.
Scenario make_bench_corpus(const BenchCorpusParams& p);

// The small, richly-scripted validation corpus: every one of the 9 event
// classes is planted at a known (agent pair, time window) at least twice,
// against a quiet background, so the independent Python reference miner has
// something deterministic and interpretable to agree (or disagree) with.
// num_segments independent segments, seeded off `seed + segment index`.
std::vector<Scenario> make_validation_corpus(int num_segments, unsigned seed);

// One agent, one segment, one randomly chosen maneuver (lane_change,
// pullover, or pickup_dropoff_approach, roughly evenly split), timing,
// lane-shift magnitude and longitudinal position all randomized within
// ranges the matching predicate can actually fire on. This is a
// dedicated mining pass for the maneuver-similarity-search project
// (Python layer under maneuver_search/), not the main validation
// corpus: it is not meant to exercise pairwise predicates, correctness
// against the Python oracle, or replay determinism (those are already
// covered by make_validation_corpus). Deliberately not multi-agent
// traffic: single-agent-per-segment keeps a 220,000-segment mining pass
// a single-agent-only scan, with no O(n^2) pairwise cost at all.
struct MiningCorpusParams {
    int num_segments = 220000;
    unsigned seed = 1;
    double pudo_zone_x_start_m = 1200.0;
    double pudo_zone_x_end_m = 1400.0;
};
std::vector<Scenario> make_mining_corpus(const MiningCorpusParams& p);

// Plain-text export of a corpus's agent parameters (not poses: pose_at is
// closed-form, so the parameters are the whole scenario). This is the
// shared input the independent Python reference miner reads; only the data
// is shared, not any predicate or mining logic.
void write_scenario_corpus(std::ostream& os, const std::vector<Scenario>& corpus);

} // namespace autolabel
