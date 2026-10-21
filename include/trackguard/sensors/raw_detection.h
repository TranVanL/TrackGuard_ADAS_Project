#pragma once

#include <array>
#include <chrono>
#include <string>

namespace trackguard::sensors {

using Timestamp = std::chrono::microseconds;

struct Vec2 {
    double x{0.0};
    double y{0.0};
};

enum class SensorType {
    Camera,
    Radar,
    Lidar,
};

struct RawDetection {
    Timestamp t{};
    SensorType sensor{SensorType::Camera};

    
    Vec2 pos;
    Vec2 vel;
    bool velocity_valid{false};
    double confidence{1.0}; 

    int frame_index{-1};
    int ground_truth_track_id{-1};
    std::string object_type;
    double truncated{0.0};
    int occluded{0};
    double alpha{0.0};
    std::array<double, 4> bbox{};
    std::array<double, 3> dimensions{};
    std::array<double, 3> camera_location{};
    double rotation_y{0.0};
};

}  // namespace trackguard::sensors