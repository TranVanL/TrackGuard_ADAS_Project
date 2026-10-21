#pragma once

#include <array>
#include <chrono>
#include <istream>
#include <optional>
#include <string>
#include "trackguard/sensors/raw_detection.h"

#include <vector>

namespace trackguard::replay {

using Timestamp = trackguard::sensors::Timestamp;
using Vec2 = trackguard::sensors::Vec2;
using SensorType = trackguard::sensors::SensorType;
using RawDetection = trackguard::sensors::RawDetection;

struct KittiObjectLabel {
    int frame_index{-1};
    int ground_truth_track_id{-1};
    std::string type;
    double truncated{0.0};
    int occluded{0};
    double alpha{0.0};
    std::array<double, 4> bbox{};
    std::array<double, 3> dimensions{};
    std::array<double, 3> location{};
    double rotation_y{0.0};
    std::optional<double> score;
};

struct EgoMotionSample {
    int frame_index{-1};
    Timestamp t{};

    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
    double roll_rad{0.0};
    double pitch_rad{0.0};
    double yaw_rad{0.0};

    double velocity_north_mps{0.0};
    double velocity_east_mps{0.0};
    double velocity_forward_mps{0.0};
    double velocity_left_mps{0.0};
    double velocity_up_mps{0.0};

    double acceleration_x_mps2{0.0};
    double acceleration_y_mps2{0.0};
    double acceleration_z_mps2{0.0};
    double acceleration_forward_mps2{0.0};
    double acceleration_left_mps2{0.0};
    double acceleration_up_mps2{0.0};

    double angular_rate_x_radps{0.0};
    double angular_rate_y_radps{0.0};
    double angular_rate_z_radps{0.0};
    double angular_rate_forward_radps{0.0};
    double angular_rate_left_radps{0.0};
    double angular_rate_up_radps{0.0};

    double position_accuracy_m{0.0};
    double velocity_accuracy_mps{0.0};
    int navstat{0};
    int numsats{0};
    int posmode{0};
    int velmode{0};
    int orimode{0};
};

struct ParseIssue {
    std::string source;
    std::size_t line_number{0U};
    int frame_index{-1};
    std::string message;
};

struct KittiFrame {
    int frame_index{-1};
    Timestamp t{};
    std::vector<KittiObjectLabel> labels;
    std::vector<RawDetection> detections;
    std::optional<EgoMotionSample> ego_motion;
};

struct KittiSequence {
    std::vector<KittiFrame> frames;
    std::vector<ParseIssue> issues;
};

class KittiTrackingLoader {
public:
    explicit KittiTrackingLoader(double frame_rate_hz = 10.0);

    KittiSequence load(
        const std::string& label_path,
        const std::string& oxts_path) const;

    KittiSequence load(
        std::istream& label_stream,
        std::istream& oxts_stream,
        const std::string& label_source = "label_02",
        const std::string& oxts_source = "oxts") const;

private:
    [[nodiscard]] Timestamp timestampForFrame(int frame_index) const;

    double frame_rate_hz_;
};

}  // namespace trackguard::replay