#pragma once

namespace radar {

// Chi-square threshold for 2 degrees of freedom (x, y) at 99% confidence.
// A detection whose squared Mahalanobis distance from a track's predicted
// position exceeds this value is outside the track's validation gate and is
// never offered to the assignment stage for that track.
// Reference value (chi-square inverse CDF, 2 DOF, p=0.99): 9.21034.
constexpr double kChiSquareGate99_2dof = 9.21034;

inline bool within_gate(double mahalanobis_sq, double threshold = kChiSquareGate99_2dof) {
    return mahalanobis_sq < threshold;
}

} // namespace radar
