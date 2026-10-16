#include <gtest/gtest.h>

#include <array>

#include "autolabel/miner.hpp"
#include "autolabel/scenario_gen.hpp"

using namespace autolabel;

TEST(Miner, PairsScannedMatchesCombinatorialCount) {
    BenchCorpusParams p;
    p.num_agents = 10;
    p.num_steps = 50;
    Scenario s = make_bench_corpus(p);

    MineParams mp;
    mp.num_threads = 4;
    mp.run_single_agent = false;

    MineResult r = mine_scenario(s, mp);
    const std::uint64_t expected = 10ull * 9ull / 2ull * 50ull;
    EXPECT_EQ(r.pairwise_scan.pairs_scanned, expected);
    EXPECT_LE(r.pairwise_scan.pairs_candidate, r.pairwise_scan.pairs_scanned);
}

TEST(Miner, ThreadCountDoesNotChangeScanCount) {
    BenchCorpusParams p;
    p.num_agents = 24;
    p.num_steps = 200;
    Scenario s = make_bench_corpus(p);

    MineParams mp1;
    mp1.num_threads = 1;
    mp1.run_single_agent = false;
    MineParams mp8 = mp1;
    mp8.num_threads = 8;

    MineResult r1 = mine_scenario(s, mp1);
    MineResult r8 = mine_scenario(s, mp8);
    EXPECT_EQ(r1.pairwise_scan.pairs_scanned, r8.pairwise_scan.pairs_scanned);
    EXPECT_EQ(r1.pairwise_scan.pairs_candidate, r8.pairwise_scan.pairs_candidate);
}

TEST(Miner, ValidationCorpusFindsEveryEventClass) {
    std::vector<Scenario> corpus = make_validation_corpus(3, 42);
    MineParams mp;
    mp.num_threads = 4;

    MineResult r = mine_corpus(corpus, mp);

    std::array<int, kNumEventClasses> counts{};
    for (const Event& e : r.events) counts[static_cast<int>(e.event_class)]++;
    for (int c = 0; c < kNumEventClasses; ++c) {
        EXPECT_GT(counts[c], 0) << "event class " << event_class_name(static_cast<EventClass>(c))
                                  << " was never planted-and-detected";
    }
}

TEST(Miner, TwoRunsOverSameCorpusProduceIdenticalEvents) {
    MineParams mp;
    mp.num_threads = 8;

    std::vector<Scenario> corpus1 = make_validation_corpus(5, 7);
    std::vector<Scenario> corpus2 = make_validation_corpus(5, 7);
    MineResult r1 = mine_corpus(corpus1, mp);
    MineResult r2 = mine_corpus(corpus2, mp);

    ASSERT_EQ(r1.events.size(), r2.events.size());
    for (std::size_t i = 0; i < r1.events.size(); ++i) EXPECT_TRUE(r1.events[i] == r2.events[i]);
}
