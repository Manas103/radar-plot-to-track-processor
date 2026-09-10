#include "radar/assignment.hpp"

#include <gtest/gtest.h>

using namespace radar;

TEST(Hungarian, KnownThreeByThreeOptimalIsFive) {
    // Textbook example: optimal assignment is row0->col1, row1->col0,
    // row2->col2, total cost 1 + 2 + 2 = 5. A greedy row-by-row nearest
    // pick (row0->col1 cost 1, row1->col2 cost 5 since col0 unavailable
    // isn't even forced here, but row2 then has fewer good options) can
    // land on a worse total; Hungarian must find the true optimum.
    std::vector<double> cost = {
        4, 1, 3,
        2, 0, 5,
        3, 2, 2,
    };
    std::vector<int> assign = hungarian_min_cost(cost, 3, 3);
    ASSERT_EQ(assign.size(), 3u);
    double total = 0.0;
    for (int i = 0; i < 3; ++i) {
        ASSERT_NE(assign[i], -1);
        total += cost[static_cast<std::size_t>(i) * 3 + assign[i]];
    }
    EXPECT_DOUBLE_EQ(total, 5.0);
    // Also check every column is used exactly once (a valid perfect matching).
    std::vector<bool> col_used(3, false);
    for (int i = 0; i < 3; ++i) {
        EXPECT_FALSE(col_used[static_cast<std::size_t>(assign[i])]);
        col_used[static_cast<std::size_t>(assign[i])] = true;
    }
}

TEST(Hungarian, RectangularMoreColumnsThanRows) {
    // 2 rows, 3 cols: row0 cheapest at col2, row1 cheapest at col0, and
    // they do not conflict, so the optimum uses both cheapest cells.
    std::vector<double> cost = {
        9, 9, 1,
        1, 9, 9,
    };
    std::vector<int> assign = hungarian_min_cost(cost, 2, 3);
    ASSERT_EQ(assign.size(), 2u);
    EXPECT_EQ(assign[0], 2);
    EXPECT_EQ(assign[1], 0);
}

TEST(Hungarian, RectangularMoreRowsThanColumns) {
    // 3 rows, 2 cols: one row must go unmatched (-1).
    std::vector<double> cost = {
        1, 9,
        9, 1,
        5, 5,
    };
    std::vector<int> assign = hungarian_min_cost(cost, 3, 2);
    ASSERT_EQ(assign.size(), 3u);
    int unmatched = 0;
    for (int v : assign) unmatched += (v == -1) ? 1 : 0;
    EXPECT_EQ(unmatched, 1);
    // Row 0 and row 1 have unambiguous cheap options that don't conflict;
    // the optimal solution takes both and leaves row 2 unmatched.
    EXPECT_EQ(assign[0], 0);
    EXPECT_EQ(assign[1], 1);
    EXPECT_EQ(assign[2], -1);
}

TEST(GatedAssignment, OptimalBeatsGreedyNearest) {
    // track0's nearest detection is det0 (cost 1.0), but taking it forces
    // track1 into its worst option (det1, cost 5.0): greedy total 6.0.
    // The optimal assignment (track0->det1, track1->det0) totals 2.7 and is
    // what a real Hungarian/GNN solver must return.
    std::vector<GateEdge> edges = {
        {0, 0, 1.0},
        {0, 1, 1.5},
        {1, 0, 1.2},
        {1, 1, 5.0},
    };
    auto result = solve_gated_assignment(edges, 2, 2, 100.0);
    ASSERT_EQ(result.size(), 2u);

    int det_for_track0 = -1, det_for_track1 = -1;
    for (auto& a : result) {
        if (a.track_idx == 0) det_for_track0 = a.det_idx;
        if (a.track_idx == 1) det_for_track1 = a.det_idx;
    }
    EXPECT_EQ(det_for_track0, 1);
    EXPECT_EQ(det_for_track1, 0);
}

TEST(GatedAssignment, DisjointComponentsSolvedIndependently) {
    // Track/detection pair (0,0) is isolated from pair (1,1); a shared
    // solve must not let one component's cost influence the other's
    // assignment.
    std::vector<GateEdge> edges = {
        {0, 0, 2.0},
        {1, 1, 3.0},
    };
    auto result = solve_gated_assignment(edges, 2, 2, 100.0);
    ASSERT_EQ(result.size(), 2u);
    bool has_00 = false, has_11 = false;
    for (auto& a : result) {
        if (a.track_idx == 0 && a.det_idx == 0) has_00 = true;
        if (a.track_idx == 1 && a.det_idx == 1) has_11 = true;
    }
    EXPECT_TRUE(has_00);
    EXPECT_TRUE(has_11);
}

TEST(GatedAssignment, NoEdgesProducesNoAssociations) {
    std::vector<GateEdge> edges;
    auto result = solve_gated_assignment(edges, 3, 3, 100.0);
    EXPECT_TRUE(result.empty());
}

TEST(GatedAssignment, NoDuplicateTrackOrDetectionInResult) {
    // A slightly larger component (3 tracks, 3 detections, fully gated)
    // must still produce a valid one-to-one matching with no track or
    // detection used twice.
    std::vector<GateEdge> edges = {
        {0, 0, 4}, {0, 1, 1}, {0, 2, 3}, {1, 0, 2}, {1, 1, 0}, {1, 2, 5},
        {2, 0, 3}, {2, 1, 2}, {2, 2, 2},
    };
    auto result = solve_gated_assignment(edges, 3, 3, 1000.0);
    ASSERT_EQ(result.size(), 3u);
    std::vector<bool> track_used(3, false), det_used(3, false);
    for (auto& a : result) {
        EXPECT_FALSE(track_used[static_cast<std::size_t>(a.track_idx)]);
        EXPECT_FALSE(det_used[static_cast<std::size_t>(a.det_idx)]);
        track_used[static_cast<std::size_t>(a.track_idx)] = true;
        det_used[static_cast<std::size_t>(a.det_idx)] = true;
    }
}
