#pragma once

#include <cstdint>
#include <vector>

#include "autolabel/types.hpp"

namespace autolabel {

struct PrefilterParams {
    double cell_size_m = 12.0; // uniform grid cell edge length
};

// (ax, ay): integer grid cell coordinates of a pose under a uniform grid of
// edge length cell_size_m.
struct CellCoord {
    std::int64_t cx = 0;
    std::int64_t cy = 0;
};

CellCoord cell_of(const Pose& pose, const PrefilterParams& params);

// True if two cells are the same cell or Chebyshev-adjacent (any of the 8
// neighbors), i.e. the agents could plausibly be within interaction range
// at this instant. This is the entire prefilter: an integer coordinate
// subtraction and two comparisons, O(1) per pair, intentionally cheaper
// than any predicate so the scan rate is bounded by this check, not by the
// geometry the surviving candidates run next.
bool cells_adjacent(const CellCoord& a, const CellCoord& b);

struct ScanStats {
    std::uint64_t pairs_scanned = 0;   // every (agent pair, timestep) the grid check ran on
    std::uint64_t pairs_candidate = 0; // pairs that passed cells_adjacent and got predicate time
};

} // namespace autolabel
