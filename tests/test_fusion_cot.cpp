#include "fusion/cot.hpp"

#include <gtest/gtest.h>

using namespace fusion::cot;

TEST(Cot, BuildEventContainsRequiredElementsAndAttributes) {
    Event e;
    e.uid = "TRACK-7";
    e.type = "a-u-G";
    e.time_s = 100.0;
    e.stale_s = 130.0;
    e.lat_deg = 37.5;
    e.lon_deg = -122.3;
    e.course_deg = 90.0;
    e.speed_mps = 12.5;

    std::string xml = build_event(e);
    EXPECT_NE(xml.find("<event "), std::string::npos);
    EXPECT_NE(xml.find("uid=\"TRACK-7\""), std::string::npos);
    EXPECT_NE(xml.find("type=\"a-u-G\""), std::string::npos);
    EXPECT_NE(xml.find("<point "), std::string::npos);
    EXPECT_NE(xml.find("lat=\"37.500000\""), std::string::npos);
    EXPECT_NE(xml.find("lon=\"-122.300000\""), std::string::npos);
    EXPECT_NE(xml.find("<detail>"), std::string::npos);
    EXPECT_NE(xml.find("<contact "), std::string::npos);
    EXPECT_NE(xml.find("<track "), std::string::npos);
}

TEST(Cot, FormatTimeProducesIso8601Utc) {
    std::string t = format_cot_time(0.0);
    EXPECT_EQ(t, "2026-01-01T00:00:00Z");
}

TEST(Cot, FormatTimeAdvancesByWholeSeconds) {
    std::string t = format_cot_time(3661.0); // 1h 1m 1s past the epoch
    EXPECT_EQ(t, "2026-01-01T01:01:01Z");
}

TEST(Cot, StaleIsAfterTime) {
    Event e;
    e.uid = "TRACK-1";
    e.type = "a-u-G";
    e.time_s = 10.0;
    e.stale_s = 40.0;
    std::string xml = build_event(e);
    EXPECT_NE(xml.find("start=\"2026-01-01T00:00:10Z\""), std::string::npos);
    EXPECT_NE(xml.find("stale=\"2026-01-01T00:00:40Z\""), std::string::npos);
}

TEST(Cot, RemarksAreXmlEscaped) {
    Event e;
    e.uid = "TRACK-1";
    e.type = "a-u-G";
    e.remarks = "a & b < c";
    std::string xml = build_event(e);
    EXPECT_NE(xml.find("a &amp; b &lt; c"), std::string::npos);
}
