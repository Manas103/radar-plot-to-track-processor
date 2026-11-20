#include "fusion/scenario.hpp"

#include <gtest/gtest.h>

#include "radar/tracker.hpp"

using namespace fusion;

TEST(FusionScenario, EveryConfiguredSourceProducesAtLeastOneDetectionOverManyScans) {
    FusionScenarioParams p;
    p.num_radar_targets = 2;
    p.num_adsb_targets = 2;
    p.num_ais_targets = 2;
    p.seed = 1;
    FusionScenario scenario(p);

    bool saw_radar = false, saw_adsb = false, saw_ais = false;
    for (int i = 0; i < 20; ++i) {
        FusedScanResult result = scenario.next_scan();
        for (Source s : result.detection_source) {
            if (s == Source::kRadar) saw_radar = true;
            if (s == Source::kAdsb) saw_adsb = true;
            if (s == Source::kAis) saw_ais = true;
        }
    }
    EXPECT_TRUE(saw_radar);
    EXPECT_TRUE(saw_adsb);
    EXPECT_TRUE(saw_ais);
}

TEST(FusionScenario, DetectionSourceIsParallelToScanDetections) {
    FusionScenarioParams p;
    p.num_radar_targets = 1;
    p.num_adsb_targets = 1;
    p.num_ais_targets = 1;
    FusionScenario scenario(p);
    FusedScanResult result = scenario.next_scan();
    EXPECT_EQ(result.detection_source.size(), result.scan.count);
}

TEST(FusionScenario, OneConfirmedTrackPerSourceEmergesForASparseCleanScenario) {
    // One target per feed, no clutter in this scenario's own feeds (Pd=1,
    // no clutter in make_radar_params), well separated: the unmodified
    // TrackManager should confirm exactly 3 tracks regardless of which
    // feed each one came from, which is the "three feeds correlated into
    // one track picture" claim's minimum bar.
    FusionScenarioParams p;
    p.num_radar_targets = 1;
    p.num_adsb_targets = 1;
    p.num_ais_targets = 1;
    p.seed = 3;
    FusionScenario scenario(p);
    radar::TrackManager tracker(radar::TrackerParams{});

    int confirmed = 0;
    for (int i = 0; i < 15; ++i) {
        FusedScanResult result = scenario.next_scan();
        radar::ScanResult sr = tracker.process_scan(result.scan);
        confirmed = sr.num_confirmed;
    }
    EXPECT_EQ(confirmed, 3);
}
