// radar_cli: one binary, several subcommands, all built on the same
// two-thread (detection-producer -> lock-free ring -> tracking-thread)
// pipeline, so record/replay/benchmark/soak are never separate code paths
// that could quietly disagree with each other.
//
// Subcommands:
//   record <out.log> <num_scans> [num_targets=500] [seed=1]
//     Runs the live simulator through the pipeline and writes every scan's
//     raw detections to out.log (the "recorded scans" claim's input).
//   track <in.log> <out_tracks.log>
//     Reads a recorded or scenario log, runs the real tracker, writes the
//     canonical per-scan track-history log, prints the SHA-256 of it. This
//     is also what the Python oracle comparison feeds through: the same
//     scenario file, the same canonical output format, two independent
//     implementations.
//   verify-replay <in.log>
//     Runs `track` twice over the same input and compares the two SHA-256
//     digests: the bit-for-bit replay claim.
//   latency-bench <num_scans=10000> [num_targets=500] [seed=1]
//     Reports the true max (not a percentile) per-scan latency.
//   soak <num_scans=21600> [num_targets=500] [seed=1]
//     Runs num_scans back to back on a VirtualClock (no real sleeping)
//     while still measuring true wall-clock compute time per scan against a
//     1.0s deadline budget. See clock.hpp for why that is an honest
//     fast-forward and not a shortcut on the deadline check.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "radar/clock.hpp"
#include "radar/detection_sim.hpp"
#include "radar/replay.hpp"
#include "radar/sha256.hpp"
#include "radar/spsc_ring.hpp"
#include "radar/tracker.hpp"
#include "radar/types.hpp"

namespace {

using radar::ScanMessage;
using Clock = std::chrono::steady_clock;

struct PipelineOptions {
    radar::IClock* pacing_clock = nullptr; // non-null: producer paces with advance(dt) per scan
    double dt_seconds = 1.0;
    double deadline_budget_s = 0.0; // >0: consumer checks process latency against this
    std::ostream* track_log = nullptr; // non-null: canonical line written per scan
};

struct PipelineStats {
    std::uint64_t scans_processed = 0;
    std::uint64_t missed_deadlines = 0;
    double max_latency_s = 0.0;
    double wall_clock_elapsed_s = 0.0;
    std::string final_hash;
    std::vector<double> latencies_s;
};

// produce_next: returns false when the source is exhausted.
PipelineStats run_pipeline(std::function<bool(ScanMessage&)> produce_next,
                            const radar::TrackerParams& tracker_params, const PipelineOptions& opts,
                            bool collect_latencies) {
    radar::SpscRing<ScanMessage, 16> ring;
    std::atomic<bool> producer_done{false};

    std::thread producer([&]() {
        ScanMessage msg;
        while (produce_next(msg)) {
            if (opts.pacing_clock) opts.pacing_clock->advance(opts.dt_seconds);
            while (!ring.try_push(msg)) {
                std::this_thread::yield();
            }
        }
        producer_done.store(true, std::memory_order_release);
    });

    PipelineStats stats;
    std::string canonical_accum;
    radar::TrackManager tracker(tracker_params);

    const auto pipeline_start = Clock::now();
    while (true) {
        ScanMessage msg;
        bool have_msg = ring.try_pop(msg);
        if (!have_msg) {
            if (producer_done.load(std::memory_order_acquire)) {
                if (!ring.try_pop(msg)) break;
                have_msg = true;
            } else {
                std::this_thread::yield();
                continue;
            }
        }

        const auto t0 = Clock::now();
        radar::ScanResult result = tracker.process_scan(msg);
        const auto t1 = Clock::now();
        const double dur_s = std::chrono::duration<double>(t1 - t0).count();

        stats.max_latency_s = std::max(stats.max_latency_s, dur_s);
        if (opts.deadline_budget_s > 0.0 && dur_s > opts.deadline_budget_s) stats.missed_deadlines++;
        if (collect_latencies) stats.latencies_s.push_back(dur_s);

        std::string line = radar::canonical_scan_line(result, tracker.tracks());
        canonical_accum += line;
        if (opts.track_log) *opts.track_log << line;

        stats.scans_processed++;
    }
    stats.wall_clock_elapsed_s = std::chrono::duration<double>(Clock::now() - pipeline_start).count();

    producer.join();
    stats.final_hash = radar::sha256_hex(canonical_accum);
    return stats;
}

std::function<bool(ScanMessage&)> live_producer(std::shared_ptr<radar::DetectionSimulator> sim,
                                                  std::uint64_t num_scans, std::ofstream* raw_log) {
    auto remaining = std::make_shared<std::uint64_t>(num_scans);
    return [sim, remaining, raw_log](ScanMessage& out) -> bool {
        if (*remaining == 0) return false;
        out = sim->next_scan();
        if (raw_log) radar::write_scan(*raw_log, out);
        --(*remaining);
        return true;
    };
}

std::function<bool(ScanMessage&)> file_producer(std::shared_ptr<std::ifstream> in) {
    return [in](ScanMessage& out) -> bool { return radar::read_next_scan(*in, out); };
}

radar::TrackerParams default_tracker_params() { return radar::TrackerParams{}; }

radar::SimulatorParams make_sim_params(int num_targets, unsigned seed) {
    radar::SimulatorParams p;
    p.num_targets = num_targets;
    p.seed = seed;
    return p;
}

int cmd_record(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "usage: radar_cli record <out.log> <num_scans> [num_targets=500] [seed=1]\n";
        return 2;
    }
    std::string out_path = argv[2];
    std::uint64_t num_scans = std::stoull(argv[3]);
    int num_targets = argc > 4 ? std::stoi(argv[4]) : 500;
    unsigned seed = argc > 5 ? static_cast<unsigned>(std::stoul(argv[5])) : 1u;

    std::ofstream raw_log(out_path, std::ios::trunc);
    if (!raw_log) {
        std::cerr << "failed to open " << out_path << " for writing\n";
        return 1;
    }
    radar::write_scan_log_header(raw_log);

    auto sim = std::make_shared<radar::DetectionSimulator>(make_sim_params(num_targets, seed));
    radar::VirtualClock vclock; // recording itself does not need to take real wall time
    PipelineOptions opts;
    opts.pacing_clock = &vclock;

    PipelineStats stats =
        run_pipeline(live_producer(sim, num_scans, &raw_log), default_tracker_params(), opts, false);

    std::cout << "recorded " << stats.scans_processed << " scans to " << out_path << "\n";
    std::cout << "informational track-history hash: " << stats.final_hash << "\n";
    return 0;
}

int cmd_track(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "usage: radar_cli track <in.log> <out_tracks.log>\n";
        return 2;
    }
    auto in = std::make_shared<std::ifstream>(argv[2]);
    if (!*in) {
        std::cerr << "failed to open " << argv[2] << " for reading\n";
        return 1;
    }
    std::ofstream out(argv[3], std::ios::trunc);
    if (!out) {
        std::cerr << "failed to open " << argv[3] << " for writing\n";
        return 1;
    }

    PipelineOptions opts;
    opts.track_log = &out;
    PipelineStats stats = run_pipeline(file_producer(in), default_tracker_params(), opts, false);

    std::cout << "processed " << stats.scans_processed << " scans from " << argv[2] << "\n";
    std::cout << "track-history sha256: " << stats.final_hash << "\n";
    return 0;
}

int cmd_verify_replay(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: radar_cli verify-replay <in.log>\n";
        return 2;
    }
    std::string in_path = argv[2];

    auto run_once = [&]() -> std::string {
        auto in = std::make_shared<std::ifstream>(in_path);
        PipelineOptions opts;
        PipelineStats stats = run_pipeline(file_producer(in), default_tracker_params(), opts, false);
        std::cout << "  run processed " << stats.scans_processed << " scans, hash " << stats.final_hash
                   << "\n";
        return stats.final_hash;
    };

    std::cout << "verify-replay: two independent runs over " << in_path << "\n";
    std::string hash1 = run_once();
    std::string hash2 = run_once();

    bool match = (hash1 == hash2);
    std::cout << "hash_1=" << hash1 << "\n";
    std::cout << "hash_2=" << hash2 << "\n";
    std::cout << (match ? "REPLAY_BIT_FOR_BIT=PASS\n" : "REPLAY_BIT_FOR_BIT=FAIL\n");
    return match ? 0 : 1;
}

int cmd_latency_bench(int argc, char** argv) {
    std::uint64_t num_scans = argc > 2 ? std::stoull(argv[2]) : 10000;
    int num_targets = argc > 3 ? std::stoi(argv[3]) : 500;
    unsigned seed = argc > 4 ? static_cast<unsigned>(std::stoul(argv[4])) : 1u;

    auto sim = std::make_shared<radar::DetectionSimulator>(make_sim_params(num_targets, seed));
    radar::VirtualClock vclock;
    PipelineOptions opts;
    opts.pacing_clock = &vclock;
    opts.deadline_budget_s = 1.0;

    PipelineStats stats =
        run_pipeline(live_producer(sim, num_scans, nullptr), default_tracker_params(), opts, true);

    std::vector<double> sorted = stats.latencies_s;
    std::sort(sorted.begin(), sorted.end());
    auto pct = [&](double p) {
        if (sorted.empty()) return 0.0;
        std::size_t idx = static_cast<std::size_t>(p * (sorted.size() - 1));
        return sorted[idx];
    };
    double sum = 0.0;
    for (double v : sorted) sum += v;
    double mean = sorted.empty() ? 0.0 : sum / static_cast<double>(sorted.size());

    std::cout << "num_targets=" << num_targets << " num_scans=" << stats.scans_processed << "\n";
    std::cout << "min_ms=" << (sorted.empty() ? 0.0 : sorted.front() * 1000.0) << "\n";
    std::cout << "mean_ms=" << mean * 1000.0 << "\n";
    std::cout << "p50_ms=" << pct(0.50) * 1000.0 << "\n";
    std::cout << "p99_ms=" << pct(0.99) * 1000.0 << "\n";
    std::cout << "WORST_CASE_MS=" << stats.max_latency_s * 1000.0 << "\n";
    std::cout << "missed_deadlines_at_1000ms=" << stats.missed_deadlines << "\n";
    return 0;
}

int cmd_soak(int argc, char** argv) {
    std::uint64_t num_scans = argc > 2 ? std::stoull(argv[2]) : 21600;
    int num_targets = argc > 3 ? std::stoi(argv[3]) : 500;
    unsigned seed = argc > 4 ? static_cast<unsigned>(std::stoul(argv[4])) : 1u;

    auto sim = std::make_shared<radar::DetectionSimulator>(make_sim_params(num_targets, seed));
    radar::VirtualClock vclock;
    PipelineOptions opts;
    opts.pacing_clock = &vclock;
    opts.deadline_budget_s = 1.0;

    const auto wall_start = Clock::now();
    PipelineStats stats =
        run_pipeline(live_producer(sim, num_scans, nullptr), default_tracker_params(), opts, false);
    const double wall_s = std::chrono::duration<double>(Clock::now() - wall_start).count();

    std::cout << "SOAK_DISCLOSURE: measured via an injectable virtual clock, not 6 real hours. "
                 "VirtualClock advanced "
              << num_scans
              << " logical 1.0s scan intervals with 0 real sleep between scans; every scan's "
                 "process_scan() duration below was measured against std::chrono::steady_clock (real "
                 "CPU time) and checked against the 1.0s-equivalent deadline budget.\n";
    std::cout << "num_targets=" << num_targets << "\n";
    std::cout << "scans_processed=" << stats.scans_processed << "\n";
    std::cout << "MISSED_DEADLINES=" << stats.missed_deadlines << "\n";
    std::cout << "max_latency_ms=" << stats.max_latency_s * 1000.0 << "\n";
    std::cout << "real_wall_clock_elapsed_s=" << wall_s << "\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: radar_cli <record|track|verify-replay|latency-bench|soak> ...\n";
        return 2;
    }
    std::string cmd = argv[1];
    if (cmd == "record") return cmd_record(argc, argv);
    if (cmd == "track") return cmd_track(argc, argv);
    if (cmd == "verify-replay") return cmd_verify_replay(argc, argv);
    if (cmd == "latency-bench") return cmd_latency_bench(argc, argv);
    if (cmd == "soak") return cmd_soak(argc, argv);

    std::cerr << "unknown subcommand: " << cmd << "\n";
    return 2;
}
