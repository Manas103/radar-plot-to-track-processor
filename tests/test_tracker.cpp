#include "radar/tracker.hpp"

#include <algorithm>

#include <gtest/gtest.h>

using namespace radar;

namespace {

ScanMessage make_scan(std::uint64_t id, std::vector<Detection> dets) {
    ScanMessage s;
    s.scan_id = id;
    s.timestamp_s = static_cast<double>(id);
    s.count = static_cast<std::uint32_t>(dets.size());
    for (std::size_t i = 0; i < dets.size(); ++i) s.detections[i] = dets[i];
    return s;
}

TrackerParams tight_params() {
    TrackerParams p;
    p.confirm_window_n = 5;
    p.confirm_hits_m = 3;
    p.delete_consecutive_misses = 3;
    return p;
}

} // namespace

TEST(Tracker, ScanCountConservation) {
    // num_confirmed + num_tentative must always equal the live track list
    // size: no track can silently exist outside both categories.
    TrackManager tm(tight_params());
    for (int i = 1; i <= 10; ++i) {
        auto result = tm.process_scan(make_scan(i, {Detection{1000.0 + i * 5.0, 2000.0}}));
        EXPECT_EQ(result.num_confirmed + result.num_tentative, static_cast<int>(tm.tracks().size()));
    }
}

TEST(Tracker, NoDuplicateTrackIds) {
    TrackManager tm(tight_params());
    // Three well-separated detections every scan spawn (at most) three new
    // tentative tracks on the first scan; afterwards they should associate,
    // never respawn.
    for (int i = 1; i <= 8; ++i) {
        tm.process_scan(make_scan(i, {Detection{1000.0, 1000.0}, Detection{5000.0, 5000.0},
                                       Detection{9000.0, 9000.0}}));
    }
    std::vector<std::uint32_t> ids;
    for (const auto& t : tm.tracks()) ids.push_back(t.id);
    std::sort(ids.begin(), ids.end());
    ASSERT_EQ(ids.size(), 3u);
    for (std::size_t i = 1; i < ids.size(); ++i) EXPECT_NE(ids[i], ids[i - 1]);
    // Confirms only 3 tracks were ever spawned in total, not 24 (3 per scan
    // x 8 scans), i.e. association is actually working, not just spawning.
    EXPECT_EQ(tm.tracks_spawned_total(), 3u);
}

TEST(Tracker, ConfirmsAfterMOfNHits) {
    TrackManager tm(tight_params()); // 3-of-5 to confirm
    TrackStatus last_status = TrackStatus::kTentative;
    for (int i = 1; i <= 3; ++i) {
        tm.process_scan(make_scan(i, {Detection{1000.0, 1000.0}}));
        ASSERT_EQ(tm.tracks().size(), 1u);
        last_status = tm.tracks()[0].status;
    }
    // On the 3rd consecutive hit (3-of-5, and 3-of-3 so far) the track must
    // be confirmed.
    EXPECT_EQ(last_status, TrackStatus::kConfirmed);
}

TEST(Tracker, DeletesConfirmedTrackAfterConsecutiveMisses) {
    TrackerParams p = tight_params();
    TrackManager tm(p);
    // Confirm it first.
    for (int i = 1; i <= 3; ++i) tm.process_scan(make_scan(i, {Detection{1000.0, 1000.0}}));
    ASSERT_EQ(tm.tracks().size(), 1u);
    ASSERT_EQ(tm.tracks()[0].status, TrackStatus::kConfirmed);

    // Now feed empty scans (no detections at all): p.delete_consecutive_misses
    // misses in a row should delete the confirmed track.
    ScanResult result;
    for (int i = 4; i <= 3 + p.delete_consecutive_misses; ++i) {
        result = tm.process_scan(make_scan(i, {}));
    }
    EXPECT_EQ(tm.tracks().size(), 0u);
    EXPECT_EQ(result.num_deleted, 1);
}

TEST(Tracker, GateActuallyRejectsFarDetection) {
    // A detection tens of kilometers from an established track must not be
    // absorbed by it; it should spawn its own new track instead. This is
    // the "gating threshold actually rejecting what it should" invariant.
    TrackManager tm(tight_params());
    for (int i = 1; i <= 3; ++i) tm.process_scan(make_scan(i, {Detection{1000.0, 1000.0}}));
    ASSERT_EQ(tm.tracks().size(), 1u);
    std::uint32_t original_id = tm.tracks()[0].id;

    auto result = tm.process_scan(
        make_scan(4, {Detection{1000.0, 1000.0}, Detection{500000.0, 500000.0}}));
    EXPECT_EQ(result.num_new_tracks, 1); // the far detection spawned a new track
    ASSERT_EQ(tm.tracks().size(), 2u);

    bool original_still_present = false;
    for (const auto& t : tm.tracks()) {
        if (t.id == original_id) {
            original_still_present = true;
            // Its position should still be near the original cluster, not
            // dragged 500km away by an erroneous association.
            EXPECT_LT(t.filter.x(), 5000.0);
            EXPECT_LT(t.filter.y(), 5000.0);
        }
    }
    EXPECT_TRUE(original_still_present);
}

TEST(Tracker, TentativeTrackNeverConfirmedIsDroppedNotLeftLingering) {
    // A single detection that never repeats (no association on subsequent
    // scans) should eventually be dropped once the M-of-N window closes
    // without reaching the confirm threshold, rather than accumulating
    // forever as a permanent tentative track.
    TrackerParams p = tight_params();
    TrackManager tm(p);
    tm.process_scan(make_scan(1, {Detection{1000.0, 1000.0}}));
    ASSERT_EQ(tm.tracks().size(), 1u);
    for (int i = 2; i <= 1 + p.confirm_window_n; ++i) {
        tm.process_scan(make_scan(i, {}));
    }
    EXPECT_EQ(tm.tracks().size(), 0u);
}
