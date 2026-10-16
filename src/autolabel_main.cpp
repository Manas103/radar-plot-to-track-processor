// autolabel_cli: mines behavioral events out of synthetic driving-log
// scenarios.
//
// Subcommands:
//   bench [num_agents=150] [num_steps=100000] [threads=8] [seed=1]
//     Runs the pairwise grid-filtered scan alone over one large procedural
//     corridor scenario and reports pairs scanned, candidates surviving the
//     prefilter, and throughput. This is the 1.1B/2.4M/96% claim's source.
//   mine <out_events.tsv> [num_segments=20] [seed=1]
//     Runs the full single-agent + pairwise pipeline over the validation
//     corpus and writes every event to out_events.tsv (one line per event,
//     radar::sha256-able), printing the event-log hash.
//   verify-replay [num_segments=20] [seed=1]
//     Runs `mine` twice over the same validation corpus and compares the
//     two event-log hashes: the bit-for-bit replay claim.
//   export-scenarios <out.tsv> [num_segments=20] [seed=1]
//     Writes the validation corpus's agent parameters (not mined events) to
//     a plain-text file the independent Python reference miner reads, so
//     both sides mine the identical scenarios without sharing any
//     predicate or mining code.

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "autolabel/miner.hpp"
#include "autolabel/scenario_gen.hpp"
#include "radar/sha256.hpp"

namespace {

std::string events_to_text(const std::vector<autolabel::Event>& events) {
    std::ostringstream os;
    for (const auto& e : events) os << autolabel::canonical_event_line(e);
    return os.str();
}

int cmd_bench(int argc, char** argv) {
    autolabel::BenchCorpusParams p;
    p.num_agents = argc > 2 ? std::stoi(argv[2]) : 150;
    p.num_steps = argc > 3 ? std::stoi(argv[3]) : 100000;
    int threads = argc > 4 ? std::stoi(argv[4]) : 8;
    p.seed = argc > 5 ? static_cast<unsigned>(std::stoul(argv[5])) : 1u;

    autolabel::Scenario scenario = autolabel::make_bench_corpus(p);

    autolabel::MineParams mp;
    mp.num_threads = threads;
    mp.run_single_agent = false; // bench isolates the pairwise scan + prefilter

    autolabel::MineResult result = autolabel::mine_scenario(scenario, mp);

    const double throughput = result.pairwise_elapsed_s > 0.0
                                   ? static_cast<double>(result.pairwise_scan.pairs_scanned) / result.pairwise_elapsed_s
                                   : 0.0;
    const double drop_pct = result.pairwise_scan.pairs_scanned > 0
                                 ? 100.0 * (1.0 - static_cast<double>(result.pairwise_scan.pairs_candidate) /
                                                      static_cast<double>(result.pairwise_scan.pairs_scanned))
                                 : 0.0;

    std::cout << "num_agents=" << p.num_agents << " num_steps=" << p.num_steps << " threads=" << threads
              << "\n";
    std::cout << "PAIRS_SCANNED=" << result.pairwise_scan.pairs_scanned << "\n";
    std::cout << "PAIRS_CANDIDATE=" << result.pairwise_scan.pairs_candidate << "\n";
    std::cout << "ELAPSED_S=" << result.pairwise_elapsed_s << "\n";
    std::cout << "THROUGHPUT_PAIRS_PER_SEC=" << throughput << "\n";
    std::cout << "PREFILTER_DROP_PCT=" << drop_pct << "\n";
    std::cout << "EVENTS_FOUND=" << result.events.size()
              << " (incidental: the bench corpus plants no maneuvers, so these are background agents that "
                 "happened to end up close_proximity by random lane/speed assignment, not a correctness "
                 "signal; see the validation corpus for that)\n";
    return 0;
}

int cmd_mine(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: autolabel_cli mine <out_events.tsv> [num_segments=20] [seed=1]\n";
        return 2;
    }
    std::string out_path = argv[2];
    int num_segments = argc > 3 ? std::stoi(argv[3]) : 20;
    unsigned seed = argc > 4 ? static_cast<unsigned>(std::stoul(argv[4])) : 1u;

    std::vector<autolabel::Scenario> corpus = autolabel::make_validation_corpus(num_segments, seed);
    autolabel::MineParams mp;
    mp.num_threads = 8;

    autolabel::MineResult result = autolabel::mine_corpus(corpus, mp);

    std::ofstream out(out_path, std::ios::trunc);
    if (!out) {
        std::cerr << "failed to open " << out_path << " for writing\n";
        return 1;
    }
    out << "segment_id\tagent_a\tagent_b\tevent_class\tstart_step\tend_step\n";
    for (const auto& e : result.events) out << autolabel::canonical_event_line(e);

    std::string hash = radar::sha256_hex(events_to_text(result.events));
    std::cout << "segments=" << corpus.size() << " events=" << result.events.size() << "\n";
    std::cout << "EVENT_LOG_SHA256=" << hash << "\n";
    for (int c = 0; c < autolabel::kNumEventClasses; ++c) {
        auto cls = static_cast<autolabel::EventClass>(c);
        int count = 0;
        for (const auto& e : result.events) count += (e.event_class == cls);
        std::cout << "  " << autolabel::event_class_name(cls) << ": " << count << "\n";
    }
    return 0;
}

int cmd_verify_replay(int argc, char** argv) {
    int num_segments = argc > 2 ? std::stoi(argv[2]) : 20;
    unsigned seed = argc > 3 ? static_cast<unsigned>(std::stoul(argv[3])) : 1u;

    auto run_once = [&]() -> std::string {
        std::vector<autolabel::Scenario> corpus = autolabel::make_validation_corpus(num_segments, seed);
        autolabel::MineParams mp;
        mp.num_threads = 8;
        autolabel::MineResult result = autolabel::mine_corpus(corpus, mp);
        std::string hash = radar::sha256_hex(events_to_text(result.events));
        std::cout << "  run produced " << result.events.size() << " events, hash " << hash << "\n";
        return hash;
    };

    std::cout << "verify-replay: two independent mining runs over " << num_segments
              << " validation segments\n";
    std::string h1 = run_once();
    std::string h2 = run_once();
    bool match = (h1 == h2);
    std::cout << "hash_1=" << h1 << "\nhash_2=" << h2 << "\n";
    std::cout << (match ? "REPLAY_BIT_FOR_BIT=PASS\n" : "REPLAY_BIT_FOR_BIT=FAIL\n");
    return match ? 0 : 1;
}

int cmd_export_scenarios(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: autolabel_cli export-scenarios <out.tsv> [num_segments=20] [seed=1]\n";
        return 2;
    }
    std::string out_path = argv[2];
    int num_segments = argc > 3 ? std::stoi(argv[3]) : 20;
    unsigned seed = argc > 4 ? static_cast<unsigned>(std::stoul(argv[4])) : 1u;

    std::vector<autolabel::Scenario> corpus = autolabel::make_validation_corpus(num_segments, seed);
    std::ofstream out(out_path, std::ios::trunc);
    if (!out) {
        std::cerr << "failed to open " << out_path << " for writing\n";
        return 1;
    }
    autolabel::write_scenario_corpus(out, corpus);
    std::cout << "exported " << corpus.size() << " segments to " << out_path << "\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: autolabel_cli <bench|mine|verify-replay|export-scenarios> ...\n";
        return 2;
    }
    std::string cmd = argv[1];
    if (cmd == "bench") return cmd_bench(argc, argv);
    if (cmd == "mine") return cmd_mine(argc, argv);
    if (cmd == "verify-replay") return cmd_verify_replay(argc, argv);
    if (cmd == "export-scenarios") return cmd_export_scenarios(argc, argv);
    std::cerr << "unknown subcommand: " << cmd << "\n";
    return 2;
}
