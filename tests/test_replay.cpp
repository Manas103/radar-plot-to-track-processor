#include "radar/replay.hpp"

#include <sstream>

#include <gtest/gtest.h>

using namespace radar;

TEST(Replay, WriteReadRoundTripExactValues) {
    ScanMessage s;
    s.scan_id = 42;
    s.timestamp_s = 42.0;
    s.count = 3;
    s.detections[0] = Detection{1234.5678, -999.125};
    s.detections[1] = Detection{0.0, 0.0};
    s.detections[2] = Detection{19999.99, 15000.5};

    std::ostringstream out;
    write_scan_log_header(out);
    write_scan(out, s);

    std::istringstream in(out.str());
    ScanMessage read_back;
    ASSERT_TRUE(read_next_scan(in, read_back));

    EXPECT_EQ(read_back.scan_id, s.scan_id);
    EXPECT_DOUBLE_EQ(read_back.timestamp_s, s.timestamp_s);
    EXPECT_EQ(read_back.count, s.count);
    for (std::uint32_t i = 0; i < s.count; ++i) {
        EXPECT_NEAR(read_back.detections[i].x, s.detections[i].x, 1e-6);
        EXPECT_NEAR(read_back.detections[i].y, s.detections[i].y, 1e-6);
    }
}

TEST(Replay, MultipleScansReadInOrder) {
    std::ostringstream out;
    write_scan_log_header(out);
    for (std::uint64_t i = 1; i <= 5; ++i) {
        ScanMessage s;
        s.scan_id = i;
        s.timestamp_s = static_cast<double>(i);
        s.count = 1;
        s.detections[0] = Detection{static_cast<double>(i) * 10.0, 0.0};
        write_scan(out, s);
    }

    std::istringstream in(out.str());
    ScanMessage s;
    std::uint64_t count = 0;
    while (read_next_scan(in, s)) {
        ++count;
        EXPECT_EQ(s.scan_id, count);
    }
    EXPECT_EQ(count, 5u);
}

TEST(Replay, EmptyStreamReturnsFalse) {
    std::istringstream in("");
    ScanMessage s;
    EXPECT_FALSE(read_next_scan(in, s));
}

TEST(Replay, CanonicalLineListsSortedConfirmedIdsOnly) {
    std::vector<Track> tracks(3);
    tracks[0].id = 7;
    tracks[0].status = TrackStatus::kConfirmed;
    tracks[1].id = 3;
    tracks[1].status = TrackStatus::kConfirmed;
    tracks[2].id = 99;
    tracks[2].status = TrackStatus::kTentative;

    ScanResult r;
    r.scan_id = 5;
    r.num_tentative = 1;

    std::string line = canonical_scan_line(r, tracks);
    EXPECT_EQ(line, "S 5 C2 T1 IDS:3,7\n");
}
