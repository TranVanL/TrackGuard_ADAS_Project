#include <cassert>
#include <cstdio>
#include <fstream>
#include <string>

#include "trackguard/config/run_config.h"
#include "trackguard/random/random_engine.h"

namespace {

const std::string kDefaultConfig = std::string(TRACKGUARD_SOURCE_DIR) + "/config/default.yaml";

void expectConfigFailure(const std::string& contents, const std::string& filename) {
    const std::string path = std::string("/tmp/") + filename;
    {
        std::ofstream output(path);
        assert(output);
        output << contents;
    }

    bool failed = false;
    try {
        static_cast<void>(trackguard::config::RunConfig::loadFromFile(path));
    } catch (const std::exception&) {
        failed = true;
    }
    std::remove(path.c_str());
    assert(failed);
}

void testConfigLoads() {
    const auto config = trackguard::config::RunConfig::loadFromFile(kDefaultConfig);
    assert(config.project.name == "TrackGuard");
    assert(config.replay.frame_rate_hz == 10.0);
    assert(config.rng_seed == 20261014U);
    assert(config.thresholds.radar_range_min_m < config.thresholds.radar_range_max_m);
}

void testSameSeedIsDeterministic() {
    trackguard::random::RandomEngine first(1234U);
    trackguard::random::RandomEngine second(1234U);
    for (int index = 0; index < 32; ++index) {
        assert(first.nextUInt64() == second.nextUInt64());
    }
    for (int index = 0; index < 16; ++index) {
        assert(first.uniform(-2.0, 5.0) == second.uniform(-2.0, 5.0));
        assert(first.normal(0.0, 1.0) == second.normal(0.0, 1.0));
    }
}

void testDifferentSeedsDiverge() {
    trackguard::random::RandomEngine first(1234U);
    trackguard::random::RandomEngine second(5678U);
    assert(first.nextUInt64() != second.nextUInt64());
}

void testInvalidConfigsFail() {
    expectConfigFailure(
        "project:\n  name: TrackGuard\n  version: 0.1.0\n"
        "replay:\n  frame_rate_hz: 10\n  deterministic: true\n"
        "randomness:\n  rng_seed: 1\n"
        "thresholds:\n  camera_confidence_min: 1.5\n"
        "  radar_range_min_m: 0.5\n  radar_range_max_m: 10\n"
        "build:\n  cxx_standard: 17\n  enable_visualization: false\n"
        "  enable_ros2_publisher: false\n",
        "trackguard_invalid_config.yaml");

    expectConfigFailure(
        "project:\n  name: TrackGuard\n  version: 0.1.0\n"
        "replay:\n  frame_rate_hz: 10\n  deterministic: true\n"
        "thresholds:\n  camera_confidence_min: 0.3\n"
        "  radar_range_min_m: 0.5\n  radar_range_max_m: 10\n"
        "build:\n  cxx_standard: 17\n  enable_visualization: false\n"
        "  enable_ros2_publisher: false\n",
        "trackguard_missing_seed.yaml");
}

}  // namespace

int main() {
    testConfigLoads();
    testSameSeedIsDeterministic();
    testDifferentSeedsDiverge();
    testInvalidConfigsFail();
    return 0;
}