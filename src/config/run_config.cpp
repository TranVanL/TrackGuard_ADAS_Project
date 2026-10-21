#include "trackguard/config/run_config.h"

#include <cmath>
#include <fstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace trackguard::config {
namespace {

template <typename T>
T readRequired(const YAML::Node& parent, const char* key, const std::string& path) {
    const YAML::Node value = parent[key];
    if (!value) {
        throw std::runtime_error("Missing required config key '" + path + "." + key + "'");
    }

    try {
        return value.as<T>();
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            "Invalid config value for '" + path + "." + key + "': " + error.what());
    }
}

YAML::Node readSection(const YAML::Node& root, const char* key) {
    const YAML::Node section = root[key];
    if (!section || !section.IsMap()) {
        throw std::runtime_error("Missing or invalid config section '" + std::string(key) + "'");
    }
    return section;
}

void validate(const RunConfig& config) {
    if (config.project.name.empty()) {
        throw std::runtime_error("Config validation failed: project.name must not be empty");
    }
    if (config.project.version.empty()) {
        throw std::runtime_error("Config validation failed: project.version must not be empty");
    }
    if (!std::isfinite(config.replay.frame_rate_hz) || config.replay.frame_rate_hz <= 0.0) {
        throw std::runtime_error(
            "Config validation failed: replay.frame_rate_hz must be finite and positive");
    }
    if (!std::isfinite(config.thresholds.camera_confidence_min) ||
        config.thresholds.camera_confidence_min < 0.0 ||
        config.thresholds.camera_confidence_min > 1.0) {
        throw std::runtime_error(
            "Config validation failed: thresholds.camera_confidence_min must be in [0, 1]");
    }
    if (!std::isfinite(config.thresholds.radar_range_min_m) ||
        !std::isfinite(config.thresholds.radar_range_max_m) ||
        config.thresholds.radar_range_min_m < 0.0 ||
        config.thresholds.radar_range_max_m <= config.thresholds.radar_range_min_m) {
        throw std::runtime_error(
            "Config validation failed: radar range must satisfy 0 <= min < max");
    }
    if (config.build.cxx_standard != 17) {
        throw std::runtime_error("Config validation failed: build.cxx_standard must be 17");
    }
}

}  // namespace

RunConfig RunConfig::loadFromFile(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("Cannot open config file: " + path);
    }

    YAML::Node root;
    try {
        root = YAML::Load(input);
    } catch (const YAML::Exception& error) {
        throw std::runtime_error("Cannot parse config file '" + path + "': " + error.what());
    }
    if (!root || !root.IsMap()) {
        throw std::runtime_error("Config root must be a YAML map: " + path);
    }

    const YAML::Node project_node = readSection(root, "project");
    const YAML::Node replay_node = readSection(root, "replay");
    const YAML::Node randomness_node = readSection(root, "randomness");
    const YAML::Node thresholds_node = readSection(root, "thresholds");
    const YAML::Node build_node = readSection(root, "build");

    RunConfig config;
    config.project.name = readRequired<std::string>(project_node, "name", "project");
    config.project.version = readRequired<std::string>(project_node, "version", "project");
    config.replay.frame_rate_hz = readRequired<double>(replay_node, "frame_rate_hz", "replay");
    config.replay.deterministic = readRequired<bool>(replay_node, "deterministic", "replay");
    config.rng_seed = readRequired<std::uint64_t>(randomness_node, "rng_seed", "randomness");
    config.thresholds.camera_confidence_min =
        readRequired<double>(thresholds_node, "camera_confidence_min", "thresholds");
    config.thresholds.radar_range_min_m =
        readRequired<double>(thresholds_node, "radar_range_min_m", "thresholds");
    config.thresholds.radar_range_max_m =
        readRequired<double>(thresholds_node, "radar_range_max_m", "thresholds");
    config.build.cxx_standard = readRequired<int>(build_node, "cxx_standard", "build");
    config.build.enable_visualization =
        readRequired<bool>(build_node, "enable_visualization", "build");
    config.build.enable_ros2_publisher =
        readRequired<bool>(build_node, "enable_ros2_publisher", "build");

    validate(config);
    return config;
}

}  // namespace trackguard::config