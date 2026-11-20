#include "fusion/store_forward.hpp"

#include <gtest/gtest.h>

using namespace fusion;

TEST(ReceiverLedger, FirstApplyOfAGuidSucceeds) {
    ReceiverLedger ledger;
    EXPECT_TRUE(ledger.apply("a"));
    EXPECT_EQ(ledger.applied_count(), 1u);
    EXPECT_EQ(ledger.duplicate_deliveries(), 0u);
}

TEST(ReceiverLedger, SecondApplyOfTheSameGuidIsCountedAsADuplicateNotANewApplication) {
    ReceiverLedger ledger;
    ledger.apply("a");
    bool second = ledger.apply("a");
    EXPECT_FALSE(second);
    EXPECT_EQ(ledger.applied_count(), 1u); // still exactly one, never double-counted
    EXPECT_EQ(ledger.duplicate_deliveries(), 1u);
}

TEST(ReceiverLedger, DistinctGuidsAreCountedIndependently) {
    ReceiverLedger ledger;
    ledger.apply("a");
    ledger.apply("b");
    ledger.apply("a");
    EXPECT_EQ(ledger.applied_count(), 2u);
    EXPECT_EQ(ledger.duplicate_deliveries(), 1u);
}

TEST(StoreAndForwardSender, EventuallyDeliversUnderCertainLossWithEnoughRetries) {
    LinkSimParams lp;
    lp.kbps = 1000000.0;
    lp.loss_probability = 0.5; // half the attempts fail
    DegradedLink link(lp);
    StoreAndForwardSender sender(link, /*retransmit_interval_s=*/1.0, /*ack_loss_probability=*/0.0, 7);
    sender.enqueue("guid-1", "payload");

    ReceiverLedger ledger;
    for (double t = 1.0; t <= 200.0 && sender.queue_depth() > 0; t += 1.0) {
        for (const std::string& g : sender.tick(t)) {
            ledger.apply(g);
        }
    }
    EXPECT_EQ(sender.queue_depth(), 0u);
    EXPECT_EQ(ledger.applied_count(), 1u);
}

TEST(StoreAndForwardSender, AckLossCausesARealDuplicateDeliveryCaughtByTheLedger) {
    // Forward link never loses or gates by bandwidth; ACK is always lost
    // on the first attempt, guaranteeing one genuine retransmission.
    LinkSimParams lp;
    lp.kbps = 1000000.0;
    lp.loss_probability = 0.0;
    DegradedLink link(lp);
    StoreAndForwardSender sender(link, /*retransmit_interval_s=*/1.0, /*ack_loss_probability=*/1.0, 1);
    sender.enqueue("guid-1", "payload");

    ReceiverLedger ledger;
    std::size_t deliveries = 0;
    for (double t = 1.0; t <= 5.0; t += 1.0) {
        for (const std::string& g : sender.tick(t)) {
            ++deliveries;
            ledger.apply(g);
        }
    }
    EXPECT_GE(deliveries, 2u); // delivered more than once because the ACK kept getting lost
    EXPECT_EQ(ledger.applied_count(), 1u); // but the ledger still counted it exactly once
    EXPECT_GE(ledger.duplicate_deliveries(), 1u);
}

TEST(StoreAndForwardSender, NothingIsDeliveredDuringAScheduledOutage) {
    LinkSimParams lp;
    lp.kbps = 1000000.0;
    lp.loss_probability = 0.0;
    lp.outage_start_s = 0.0;
    lp.outage_duration_s = 20.0;
    DegradedLink link(lp);
    StoreAndForwardSender sender(link, 1.0, 0.0, 1);
    sender.enqueue("guid-1", "payload");

    for (double t = 1.0; t < 20.0; t += 1.0) {
        EXPECT_TRUE(sender.tick(t).empty());
    }
    EXPECT_FALSE(sender.tick(21.0).empty()); // delivered once the outage ends
}
