#pragma once

#include <cstdint>
#include <string>

namespace trackguard::config {

struct ProjectConfig {
    std::string name;
    std::string version;
};

struct ReplayConfig {
    double frame_rate_hz{0.0};
    bool deterministic{false};
};

struct ThresholdConfig {
    double camera_confidence_min{0.0};
    double radar_range_min_m{0.0};
    double radar_range_max_m{0.0};
};

struct BuildConfig {
    int cxx_standard{0};
    bool enable_visualization{false};
    bool enable_ros2_publisher{false};
};

struct RunConfig {
    ProjectConfig project;
    ReplayConfig replay;
    std::uint64_t rng_seed{0U};
    ThresholdConfig thresholds;
    BuildConfig build;

    static RunConfig loadFromFile(const std::string& path);
};

}  // namespace trackguard::config