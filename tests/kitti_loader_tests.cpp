#include <gtest/gtest.h>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

#include "trackguard/replay/kitti_loader.h"

namespace {

std::string makeOxtsRow(int frame_index) {
    std::ostringstream row;
    for (int field = 0; field < 30; ++field) {
        if (field != 0) {
            row << ' ';
        }

        double value = static_cast<double>(frame_index + field);
        if (field == 5) {
            value = 0.5 + frame_index;
        } else if (field == 8) {
            value = 10.0 + frame_index;
        } else if (field == 9) {
            value = 1.0 + frame_index;
        } else if (field == 10) {
            value = 0.1;
        } else if (field == 19) {
            value = 0.02;
        } else if (field >= 25) {
            value = static_cast<double>(field - 25);
        }
        row << value;
    }
    return row.str();
}

TEST(KittiTrackingLoaderTest, ParsesAndGroupsLabelsByFrame) {
    std::istringstream labels(
        "0 7 Car 0.00 0 0.15 500.0 150.0 620.0 300.0 1.50 1.80 4.00 "
        "2.10 1.60 30.00 -0.05\n"
        "0 8 Pedestrian 0.10 1 -0.20 100.0 120.0 140.0 240.0 1.70 0.60 0.60 "
        "-1.20 1.00 20.00 0.10\n"
        "1 7 Car 0.00 0 0.14 501.0 150.0 621.0 300.0 1.50 1.80 4.00 "
        "2.20 1.60 29.00 -0.04\n"
        "2 7 Car 0.00 0 0.13 502.0 150.0 622.0 300.0 1.50 1.80 4.00 "
        "2.30 1.60 28.00 -0.03 0.91\n"
        "2 -1 DontCare -1 -1 -10 100.0 100.0 200.0 200.0 -1.0 -1.0 -1.0 "
        "-1000.0 -1000.0 -1000.0 -10.0\n");
    std::istringstream oxts(
        makeOxtsRow(0) + "\n" + makeOxtsRow(1) + "\n" + makeOxtsRow(2) + "\n");

    const trackguard::replay::KittiTrackingLoader loader(10.0);
    const trackguard::replay::KittiSequence sequence = loader.load(labels, oxts);

    ASSERT_EQ(sequence.issues.size(), 0U);
    ASSERT_EQ(sequence.frames.size(), 3U);
    ASSERT_EQ(sequence.frames[0].labels.size(), 2U);
    ASSERT_EQ(sequence.frames[0].detections.size(), 2U);
    ASSERT_EQ(sequence.frames[1].labels.size(), 1U);
    ASSERT_EQ(sequence.frames[2].labels.size(), 2U);
    ASSERT_EQ(sequence.frames[2].detections.size(), 1U);

    const auto& car = sequence.frames[0].detections[0];
    EXPECT_EQ(car.frame_index, 0);
    EXPECT_EQ(car.ground_truth_track_id, 7);
    EXPECT_EQ(car.object_type, "Car");
    EXPECT_NEAR(car.pos.x, 2.10, 1e-9);
    EXPECT_NEAR(car.pos.y, 30.00, 1e-9);
    EXPECT_NEAR(car.bbox[0], 500.0, 1e-9);
    EXPECT_NEAR(car.dimensions[1], 1.80, 1e-9);
    EXPECT_NEAR(car.rotation_y, -0.05, 1e-9);
    EXPECT_FALSE(car.velocity_valid);
    EXPECT_EQ(car.t, trackguard::replay::Timestamp(0));

    EXPECT_EQ(sequence.frames[2].labels[1].type, "DontCare");
    EXPECT_EQ(sequence.frames[2].labels[1].ground_truth_track_id, -1);
    EXPECT_NEAR(*sequence.frames[2].labels[0].score, 0.91, 1e-9);
}

TEST(KittiTrackingLoaderTest, ParsesOxtsAndAssignsReplayTimestamps) {
    std::istringstream labels;
    std::istringstream oxts(
        makeOxtsRow(0) + "\n" + makeOxtsRow(1) + "\n" + makeOxtsRow(2) + "\n" +
        makeOxtsRow(3) + "\n" + makeOxtsRow(4) + "\n");

    const trackguard::replay::KittiTrackingLoader loader(10.0);
    const trackguard::replay::KittiSequence sequence = loader.load(labels, oxts);

    ASSERT_EQ(sequence.issues.size(), 0U);
    ASSERT_EQ(sequence.frames.size(), 5U);
    ASSERT_TRUE(sequence.frames[2].ego_motion.has_value());

    const auto& sample = *sequence.frames[2].ego_motion;
    EXPECT_EQ(sample.frame_index, 2);
    EXPECT_EQ(sample.t, trackguard::replay::Timestamp(200'000));
    EXPECT_NEAR(sample.yaw_rad, 2.5, 1e-9);
    EXPECT_NEAR(sample.velocity_forward_mps, 12.0, 1e-9);
    EXPECT_NEAR(sample.velocity_left_mps, 3.0, 1e-9);
    EXPECT_NEAR(sample.velocity_up_mps, 0.1, 1e-9);
    EXPECT_NEAR(sample.angular_rate_z_radps, 0.02, 1e-9);
    EXPECT_EQ(sample.navstat, 0);
    EXPECT_EQ(sample.orimode, 4);
}

TEST(KittiTrackingLoaderTest, MalformedOxtsRowDoesNotShiftLaterFrames) {
    std::istringstream labels;
    std::istringstream oxts(
        makeOxtsRow(0) + "\n"
        "0 1 2 3 4 5 6 7 8\n"
        + makeOxtsRow(2) + "\n");

    const trackguard::replay::KittiTrackingLoader loader(10.0);
    const trackguard::replay::KittiSequence sequence = loader.load(labels, oxts);

    ASSERT_EQ(sequence.issues.size(), 1U);
    ASSERT_EQ(sequence.frames.size(), 3U);
    EXPECT_FALSE(sequence.frames[1].ego_motion.has_value());
    ASSERT_TRUE(sequence.frames[2].ego_motion.has_value());
    EXPECT_EQ(sequence.frames[2].ego_motion->frame_index, 2);
    EXPECT_EQ(sequence.frames[2].ego_motion->t, trackguard::replay::Timestamp(200'000));
    EXPECT_EQ(sequence.issues[0].line_number, 2U);
    EXPECT_EQ(sequence.issues[0].frame_index, 1);
}

TEST(KittiTrackingLoaderTest, RejectsMalformedLabelAndRetainsDiagnostic) {
    std::istringstream labels("0 7 Car 0.0 0 0.0 1.0 2.0\n");
    std::istringstream oxts;

    const trackguard::replay::KittiTrackingLoader loader(10.0);
    const trackguard::replay::KittiSequence sequence = loader.load(labels, oxts);

    ASSERT_EQ(sequence.issues.size(), 1U);
    EXPECT_EQ(sequence.issues[0].source, "label_02");
    EXPECT_EQ(sequence.issues[0].line_number, 1U);
    EXPECT_EQ(sequence.issues[0].frame_index, 0);
    ASSERT_EQ(sequence.frames.size(), 1U);
    EXPECT_TRUE(sequence.frames[0].labels.empty());
    EXPECT_TRUE(sequence.frames[0].detections.empty());
}

TEST(KittiTrackingLoaderTest, RequiresPositiveFrameRate) {
    EXPECT_THROW(trackguard::replay::KittiTrackingLoader(0.0), std::invalid_argument);
    EXPECT_THROW(trackguard::replay::KittiTrackingLoader(-10.0), std::invalid_argument);
}

}  // namespace