#include "radar/assignment.hpp"

#include <algorithm>
#include <limits>
#include <map>

namespace radar {

namespace {
constexpr double kInf = 1.0e18;

// Kuhn-Munkres on a cost matrix with rows <= cols, 1-indexed internally
// (the classic potentials formulation reads cleanest that way). Returns,
// for each 0-indexed row, the assigned 0-indexed column.
std::vector<int> hungarian_square_or_wide(const std::vector<double>& cost, int n, int m) {
    std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);
    std::vector<int> p(m + 1, 0), way(m + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(m + 1, kInf);
        std::vector<bool> used(m + 1, false);
        do {
            used[j0] = true;
            int i0 = p[j0];
            int j1 = -1;
            double delta = kInf;
            for (int j = 1; j <= m; ++j) {
                if (used[j]) continue;
                double cur = cost[static_cast<std::size_t>(i0 - 1) * m + (j - 1)] - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    std::vector<int> row_to_col(n, -1);
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) row_to_col[p[j] - 1] = j - 1;
    }
    return row_to_col;
}
} // namespace

std::vector<int> hungarian_min_cost(const std::vector<double>& cost, int n_rows, int n_cols) {
    if (n_rows == 0 || n_cols == 0) return std::vector<int>(n_rows, -1);

    if (n_rows <= n_cols) {
        return hungarian_square_or_wide(cost, n_rows, n_cols);
    }

    // Transpose so the solver's row-count <= col-count assumption holds,
    // then map the transposed solution back to the caller's orientation.
    std::vector<double> transposed(static_cast<std::size_t>(n_cols) * n_rows);
    for (int i = 0; i < n_rows; ++i)
        for (int j = 0; j < n_cols; ++j)
            transposed[static_cast<std::size_t>(j) * n_rows + i] = cost[static_cast<std::size_t>(i) * n_cols + j];

    std::vector<int> t_row_to_col = hungarian_square_or_wide(transposed, n_cols, n_rows);
    std::vector<int> result(n_rows, -1);
    for (int j = 0; j < n_cols; ++j) {
        int assigned_row = t_row_to_col[j]; // an original row index, or -1
        if (assigned_row >= 0) result[assigned_row] = j;
    }
    return result;
}

namespace {
// Simple union-find over a combined node space: track nodes [0, num_tracks)
// and detection nodes [num_tracks, num_tracks + num_dets).
class UnionFind {
public:
    explicit UnionFind(int n) : parent_(n) {
        for (int i = 0; i < n; ++i) parent_[i] = i;
    }
    int find(int x) {
        while (parent_[x] != x) {
            parent_[x] = parent_[parent_[x]];
            x = parent_[x];
        }
        return x;
    }
    void unite(int a, int b) {
        a = find(a);
        b = find(b);
        if (a != b) parent_[a] = b;
    }

private:
    std::vector<int> parent_;
};
} // namespace

std::vector<Association> solve_gated_assignment(const std::vector<GateEdge>& edges, int num_tracks,
                                                  int num_dets, double gate_reject_cost) {
    std::vector<Association> results;
    if (edges.empty()) return results;

    UnionFind uf(num_tracks + num_dets);
    for (const auto& e : edges) {
        uf.unite(e.track_idx, num_tracks + e.det_idx);
    }

    // Group edges by component root.
    std::map<int, std::vector<GateEdge>> components;
    for (const auto& e : edges) {
        components[uf.find(e.track_idx)].push_back(e);
    }

    for (auto& [root, comp_edges] : components) {
        // Local index remapping for this component only.
        std::map<int, int> track_local, det_local;
        for (const auto& e : comp_edges) {
            if (track_local.find(e.track_idx) == track_local.end()) {
                int idx = static_cast<int>(track_local.size());
                track_local[e.track_idx] = idx;
            }
            if (det_local.find(e.det_idx) == det_local.end()) {
                int idx = static_cast<int>(det_local.size());
                det_local[e.det_idx] = idx;
            }
        }
        int local_tracks = static_cast<int>(track_local.size());
        int local_dets = static_cast<int>(det_local.size());
        int side = std::max(local_tracks, local_dets);

        // Square cost matrix, padded with a dummy-assignment cost that sits
        // strictly between the real gated cost range (< kChiSquareGate99_2dof)
        // and the "same component but not itself gated" sentinel
        // (gate_reject_cost), so Hungarian always prefers a real gate edge,
        // then a dummy (unassigned), then a same-component non-gated pair.
        const double dummy_cost = gate_reject_cost * 0.5;
        std::vector<double> cost(static_cast<std::size_t>(side) * side, gate_reject_cost);
        for (int i = 0; i < side; ++i)
            for (int j = 0; j < side; ++j)
                if (i >= local_tracks || j >= local_dets) cost[static_cast<std::size_t>(i) * side + j] = dummy_cost;

        for (const auto& e : comp_edges) {
            int ti = track_local[e.track_idx];
            int di = det_local[e.det_idx];
            cost[static_cast<std::size_t>(ti) * side + di] = e.cost;
        }

        std::vector<int> assignment = hungarian_min_cost(cost, side, side);

        std::vector<int> track_of_local(local_tracks, -1), det_of_local(local_dets, -1);
        for (const auto& [orig, loc] : track_local) track_of_local[loc] = orig;
        for (const auto& [orig, loc] : det_local) det_of_local[loc] = orig;

        for (int ti = 0; ti < local_tracks; ++ti) {
            int dj = assignment[ti];
            if (dj < 0 || dj >= local_dets) continue; // matched to a dummy column
            if (cost[static_cast<std::size_t>(ti) * side + dj] >= gate_reject_cost) continue; // not truly gated
            results.push_back(Association{track_of_local[ti], det_of_local[dj]});
        }
    }

    return results;
}

} // namespace radar
