#pragma once

#include "radar/linalg.hpp"
#include "radar/types.hpp"

namespace radar {

// Constant-velocity Kalman filter, state = [x, y, vx, vy].
//
// Process noise uses the standard discrete white-noise-acceleration (DWNA)
// model (Bar-Shalom, "Estimation with Applications to Tracking and
// Navigation", section on nearly-constant-velocity models):
//   Q = q * [[dt^4/4,      0, dt^3/2,      0],
//            [     0, dt^4/4,      0, dt^3/2],
//            [dt^3/2,      0,   dt^2,      0],
//            [     0, dt^3/2,      0,   dt^2]]
// where q is the continuous-time acceleration process-noise spectral
// density (m^2/s^3), tuned to the simulated targets' maneuvering rate.
class KalmanCV {
public:
    void init(double x, double y, double vx, double vy, double q_density,
              double measurement_sigma_m);

    // Advance state and covariance by dt seconds (the 1 Hz scan cadence in
    // this project, but the filter itself does not assume any particular
    // dt).
    void predict(double dt);

    // Innovation y = z - H*x and innovation covariance S = H*P*H^T + R,
    // computed against the *current* (already predicted) state, without
    // committing an update. The gating stage uses this to compute the
    // Mahalanobis distance before deciding whether to associate.
    void innovation(const Detection& z, Vec2& y_out, Mat2& s_out) const;

    // Commits a measurement update given a previously computed innovation
    // and innovation covariance (so gating and update never recompute S
    // twice).
    void update(const Vec2& y, const Mat2& s);

    double x() const { return x_.v[0]; }
    double y() const { return x_.v[1]; }
    double vx() const { return x_.v[2]; }
    double vy() const { return x_.v[3]; }
    const Mat4& covariance() const { return p_; }

    // Trace of the position sub-block of P, used by tests to detect
    // covariance blow-up (a track whose uncertainty diverges instead of
    // converging is a real failure mode, not a hypothetical one; see the
    // README Findings section).
    double position_covariance_trace() const { return p_.m[0][0] + p_.m[1][1]; }

private:
    Vec4 x_;
    Mat4 p_;
    double q_density_ = 1.0;
    Mat2 r_;
};

} // namespace radar
