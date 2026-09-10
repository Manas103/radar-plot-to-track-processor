#include "radar/gating.hpp"

#include <gtest/gtest.h>

using namespace radar;

TEST(Gating, AcceptsInsideThreshold) {
    EXPECT_TRUE(within_gate(0.0));
    EXPECT_TRUE(within_gate(9.0));
    EXPECT_TRUE(within_gate(kChiSquareGate99_2dof - 0.001));
}

TEST(Gating, RejectsAtOrAboveThreshold) {
    EXPECT_FALSE(within_gate(kChiSquareGate99_2dof));
    EXPECT_FALSE(within_gate(kChiSquareGate99_2dof + 0.001));
    EXPECT_FALSE(within_gate(1000.0));
}

TEST(Gating, CustomThresholdIsRespected) {
    EXPECT_TRUE(within_gate(5.0, 6.0));
    EXPECT_FALSE(within_gate(6.0, 6.0));
}
