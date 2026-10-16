#pragma once

#include <cstdint>
#include <vector>

#include "autolabel/grid_prefilter.hpp"
#include "autolabel/predicates.hpp"
#include "autolabel/types.hpp"

namespace autolabel {

struct MineParams {
    PrefilterParams prefilter;
    Thresholds thresholds;
    double merge_zone_x_start_m = 400.0;
    double merge_zone_x_end_m = 600.0;
    int num_threads = 8;
    bool run_pairwise = true;
    bool run_single_agent = true;
};

struct MineResult {
    std::vector<Event> events; // sorted, deterministic order; see miner.cpp
    ScanStats pairwise_scan;
    double pairwise_elapsed_s = 0.0;
    double single_agent_elapsed_s = 0.0;
};

// Scans every agent-pair-timestep in `scenario` (grid-prefiltered before any
// predicate runs) across `params.num_threads` threads partitioned by
// timestep range, plus every agent-timestep for the three single-agent
// predicates, and returns every detected event in one fixed, sorted order
// (segment, agent_a, agent_b, event_class, start_step) so two runs over the
// same scenario and params always produce byte-identical output.
MineResult mine_scenario(const Scenario& scenario, const MineParams& params);

// Convenience for multi-segment corpora: mines each scenario and
// concatenates results (sorted first by segment_id); stats are summed.
MineResult mine_corpus(const std::vector<Scenario>& scenarios, const MineParams& params);

} // namespace autolabel
