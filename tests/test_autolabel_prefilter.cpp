#include <gtest/gtest.h>

#include "autolabel/grid_prefilter.hpp"

using namespace autolabel;

TEST(Prefilter, SameCellIsAdjacent) {
    PrefilterParams p;
    Pose a{0.0, 0.0, 0.0, 0.0};
    Pose b{1.0, 1.0, 0.0, 0.0};
    EXPECT_TRUE(cells_adjacent(cell_of(a, p), cell_of(b, p)));
}

TEST(Prefilter, NeighborCellIsAdjacent) {
    PrefilterParams p;
    p.cell_size_m = 10.0;
    Pose a{5.0, 5.0, 0.0, 0.0};
    Pose b{15.0, 5.0, 0.0, 0.0}; // one cell over
    EXPECT_TRUE(cells_adjacent(cell_of(a, p), cell_of(b, p)));
}

TEST(Prefilter, FarCellIsNotAdjacent) {
    PrefilterParams p;
    p.cell_size_m = 10.0;
    Pose a{0.0, 0.0, 0.0, 0.0};
    Pose b{1000.0, 0.0, 0.0, 0.0};
    EXPECT_FALSE(cells_adjacent(cell_of(a, p), cell_of(b, p)));
}

TEST(Prefilter, NegativeCoordinatesHandledConsistently) {
    PrefilterParams p;
    p.cell_size_m = 10.0;
    Pose a{-5.0, -5.0, 0.0, 0.0};
    Pose b{-1.0, -1.0, 0.0, 0.0};
    EXPECT_TRUE(cells_adjacent(cell_of(a, p), cell_of(b, p)));
    Pose c{-500.0, 0.0, 0.0, 0.0};
    EXPECT_FALSE(cells_adjacent(cell_of(a, p), cell_of(c, p)));
}
