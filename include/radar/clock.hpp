#pragma once

#include <chrono>
#include <thread>

namespace radar {

// Abstracts only the *pacing* source: what timestamp a scan gets stamped
// with, and whether the producer actually sleeps between scans. It is
// deliberately NOT used to measure per-scan processing latency or soak
// deadlines; those are always measured against std::chrono::steady_clock,
// which reflects real CPU work regardless of which IClock is in use. That
// split is what makes the virtual-clock soak (see PacedVirtualClock) an
// honest fast-forward rather than a way to fake the deadline check.
class IClock {
public:
    virtual ~IClock() = default;
    virtual double now_seconds() const = 0;
    // Advances the clock by exactly period_seconds and, for a real-time
    // clock, blocks until that much wall time has actually passed.
    virtual void advance(double period_seconds) = 0;
};

// Real-time pacing: sleeps for the requested period. Used for the small
// "live" demo run where a human might actually watch scans arrive at 1 Hz.
class WallClock : public IClock {
public:
    double now_seconds() const override {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - epoch_).count();
    }

    void advance(double period_seconds) override {
        std::this_thread::sleep_for(std::chrono::duration<double>(period_seconds));
        t_ += period_seconds;
    }

private:
    std::chrono::steady_clock::time_point epoch_ = std::chrono::steady_clock::now();
    double t_ = 0.0;
};

// Virtual pacing: advances the logical timestamp by exactly period_seconds
// with no real sleep at all. The 6-hour soak (21,600 scans at a virtual 1 Hz
// cadence) runs back-to-back in real seconds using this clock, while every
// scan still does its real association + Kalman + M-of-N work and every
// deadline check still compares real wall-clock compute time against the
// 1-second-equivalent budget. This is the technique documented in the
// README's honest-framing section, precedented by
// device-telemetry-policy-middleware's injectable virtual clock.
class VirtualClock : public IClock {
public:
    double now_seconds() const override { return t_; }
    void advance(double period_seconds) override { t_ += period_seconds; }

private:
    double t_ = 0.0;
};

} // namespace radar
