#pragma once

#include <cstddef>
#include <vector>

namespace radar {

// Solves a rectangular minimum-cost assignment (Kuhn-Munkres / Hungarian
// algorithm, O(n^3)) over a dense cost matrix with n_rows rows and n_cols
// columns (n_rows <= n_cols is NOT required by the caller; this function
// transposes internally if needed). cost[i * n_cols + j] is the cost of
// assigning row i to column j.
//
// Returns, for each row, the assigned column index, or -1 if n_rows >
// n_cols left that row unmatched. This is the textbook potentials-based
// O(n^3) formulation (as in the classic e-maxx / competitive-programming
// reference implementation); it is an exact solver, not a heuristic, which
// is what "GNN/Hungarian-style assignment, not just greedy nearest" means
// in this project's claims.
std::vector<int> hungarian_min_cost(const std::vector<double>& cost, int n_rows, int n_cols);

// One edge in the chi-square gate graph: track row `track_idx` and
// detection `det_idx` are within each other's validation gate at cost
// (squared Mahalanobis distance) `cost`.
struct GateEdge {
    int track_idx;
    int det_idx;
    double cost;
};

// One assignment result: track_idx associated with det_idx.
struct Association {
    int track_idx;
    int det_idx;
};

// Decomposes the bipartite gate graph into connected components (tracks and
// detections that share no gate edge with anything outside the component
// cannot affect each other's optimal assignment) and solves each component
// independently with the Hungarian algorithm. This is mathematically
// equivalent to solving one global assignment problem where inter-component
// costs are infinite, and is far cheaper in the common case where most
// components are small (well-separated targets), while still being an
// exact global-nearest-neighbor solve, not a greedy shortcut.
//
// num_tracks / num_dets size the bipartite node space. gate_reject_cost is
// the sentinel used for a track/detection pair that is inside the same
// connected component but not itself gated (can happen when the component
// has 3+ members chained through other pairs); it must be finite so the
// solver still terminates, and strictly larger than kChiSquareGate99_2dof so
// it is never preferred over a real gated pair.
std::vector<Association> solve_gated_assignment(const std::vector<GateEdge>& edges,
                                                  int num_tracks, int num_dets,
                                                  double gate_reject_cost);

} // namespace radar
