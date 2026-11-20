#include "fusion/link_sim.hpp"

#include <gtest/gtest.h>

using namespace fusion;

TEST(LinkSim, InOutageWindowIsExact) {
    LinkSimParams p;
    p.outage_start_s = 100.0;
    p.outage_duration_s = 50.0;
    DegradedLink link(p);
    EXPECT_FALSE(link.in_outage(99.9));
    EXPECT_TRUE(link.in_outage(100.0));
    EXPECT_TRUE(link.in_outage(149.9));
    EXPECT_FALSE(link.in_outage(150.0));
}

TEST(LinkSim, NoScheduledOutageMeansNeverInOutage) {
    LinkSimParams p;
    p.outage_duration_s = 0.0;
    DegradedLink link(p);
    EXPECT_FALSE(link.in_outage(0.0));
    EXPECT_FALSE(link.in_outage(1e9));
}

TEST(LinkSim, NothingSendsDuringOutageRegardlessOfLossOrBudget) {
    LinkSimParams p;
    p.kbps = 1000000.0; // effectively unlimited bandwidth
    p.loss_probability = 0.0; // and no loss, to isolate the outage effect
    p.outage_start_s = 0.0;
    p.outage_duration_s = 10.0;
    DegradedLink link(p);
    for (double t = 0.0; t < 10.0; t += 1.0) {
        EXPECT_FALSE(link.try_send(t, 10));
    }
}

TEST(LinkSim, ZeroLossAndAmpleBandwidthAlwaysSucceedsOutsideOutage) {
    LinkSimParams p;
    p.kbps = 1000000.0;
    p.loss_probability = 0.0;
    p.outage_duration_s = 0.0;
    DegradedLink link(p);
    for (double t = 1.0; t < 100.0; t += 1.0) {
        EXPECT_TRUE(link.try_send(t, 10));
    }
}

TEST(LinkSim, CertainLossNeverSucceeds) {
    LinkSimParams p;
    p.kbps = 1000000.0;
    p.loss_probability = 1.0;
    p.outage_duration_s = 0.0;
    DegradedLink link(p);
    for (double t = 1.0; t < 50.0; t += 1.0) {
        EXPECT_FALSE(link.try_send(t, 10));
    }
}

TEST(LinkSim, BandwidthBudgetDoesNotBurstCatchUpAfterOutage) {
    // At 64 kbps = 8000 bytes/sec, a 10-second outage would, if budget
    // kept accruing during it, let a 160,000-byte message through
    // instantly at outage end. It must not.
    LinkSimParams p;
    p.kbps = 64.0;
    p.loss_probability = 0.0;
    p.outage_start_s = 0.0;
    p.outage_duration_s = 10.0;
    DegradedLink link(p);
    for (double t = 0.0; t < 10.0; t += 1.0) {
        link.try_send(t, 1); // all fail (outage), but must not accrue budget
    }
    EXPECT_FALSE(link.try_send(10.0, 160000));
}
