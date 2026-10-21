#include <gtest/gtest.h>

#include <stdexcept>

#include "trackguard/replay/replay_clock.h"

namespace {

using trackguard::replay::ReplayClock;
using trackguard::replay::Timestamp;

TEST(ReplayClockTest, MapsFrameIndexToVirtualTimestamp) {
    const ReplayClock clock(10.0);

    EXPECT_EQ(clock.timestampForFrame(0), Timestamp(0));
    EXPECT_EQ(clock.timestampForFrame(1), Timestamp(100'000));
    EXPECT_EQ(clock.timestampForFrame(2), Timestamp(200'000));
    EXPECT_EQ(clock.timestampForFrame(10), Timestamp(1'000'000));
}

TEST(ReplayClockTest, TickUpdatesCurrentFrameAndTime) {
    ReplayClock clock(10.0);

    EXPECT_FALSE(clock.hasTicked());
    EXPECT_THROW(static_cast<void>(clock.now()), std::logic_error);

    clock.tick(0);
    EXPECT_TRUE(clock.hasTicked());
    EXPECT_EQ(clock.frameIndex(), 0);
    EXPECT_EQ(clock.now(), Timestamp(0));

    clock.tick(3);
    EXPECT_EQ(clock.frameIndex(), 3);
    EXPECT_EQ(clock.now(), Timestamp(300'000));
}

TEST(ReplayClockTest, RejectsInvalidOrNonMonotonicTicks) {
    ReplayClock clock(10.0);

    EXPECT_THROW(clock.tick(-1), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(clock.timestampForFrame(-1)), std::invalid_argument);

    clock.tick(2);
    EXPECT_THROW(clock.tick(2), std::logic_error);
    EXPECT_THROW(clock.tick(1), std::logic_error);
}

TEST(ReplayClockTest, ResetStartsASecondReplay) {
    ReplayClock clock(20.0);
    clock.tick(4);
    EXPECT_EQ(clock.now(), Timestamp(200'000));

    clock.reset();
    EXPECT_FALSE(clock.hasTicked());
    EXPECT_THROW(static_cast<void>(clock.frameIndex()), std::logic_error);

    clock.tick(0);
    EXPECT_EQ(clock.now(), Timestamp(0));
}

TEST(ReplayClockTest, SupportsNonIntegerFramePeriodsWithRounding) {
    const ReplayClock clock(7.0);

    EXPECT_EQ(clock.timestampForFrame(1), Timestamp(142'857));
    EXPECT_EQ(clock.timestampForFrame(2), Timestamp(285'714));
}

TEST(ReplayClockTest, RejectsInvalidFrameRate) {
    EXPECT_THROW(ReplayClock(0.0), std::invalid_argument);
    EXPECT_THROW(ReplayClock(-10.0), std::invalid_argument);
}

}  // namespace