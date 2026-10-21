#include <exception>
#include <iomanip>
#include <iostream>
#include <string>

#include "trackguard/config/run_config.h"
#include "trackguard/random/random_engine.h"
#include "trackguard/replay/kitti_loader.h"
#include "trackguard/replay/replay_clock.h"

int main(int argc, char* argv[]) {
    if (argc > 4) {
        std::cerr << "Usage: " << argv[0]
                  << " [config.yaml] [label_02.txt] [oxts.txt]\n";
        return 2;
    }

    const std::string project_root = TRACKGUARD_SOURCE_DIR;
    const std::string default_config_path = project_root + "/config/default.yaml";
    const std::string default_label_path =
        project_root + "/data/kitti_subset/label_02/0000.txt";
    const std::string default_oxts_path = project_root + "/data/kitti_subset/oxts/0000.txt";
    const std::string config_path = argc > 1 ? argv[1] : default_config_path;
    const std::string label_path = argc > 2 ? argv[2] : default_label_path;
    const std::string oxts_path = argc > 3 ? argv[3] : default_oxts_path;

    try {
        const trackguard::config::RunConfig config =
            trackguard::config::RunConfig::loadFromFile(config_path);
        trackguard::random::RandomEngine rng(config.rng_seed);
        const trackguard::replay::KittiTrackingLoader loader(config.replay.frame_rate_hz);
        const trackguard::replay::KittiSequence sequence =
            loader.load(label_path, oxts_path);
        trackguard::replay::ReplayClock clock(config.replay.frame_rate_hz);

        std::cout << "TrackGuard Day 5 replay started.\n"
                  << "  project       : " << config.project.name << " "
                  << config.project.version << "\n"
                  << "  frame rate    : " << config.replay.frame_rate_hz << " Hz\n"
                  << "  deterministic : " << std::boolalpha << config.replay.deterministic << "\n"
                  << "  rng seed      : " << rng.seed() << "\n"
                  << "  label file    : " << label_path << "\n"
                  << "  oxts file     : " << oxts_path << "\n"
                  << "  frames        : " << sequence.frames.size() << "\n";

        for (const auto& issue : sequence.issues) {
            std::cerr << "Warning: " << issue.source << ":" << issue.line_number
                      << " frame=" << issue.frame_index << ": " << issue.message << '\n';
        }

        std::cout << std::fixed << std::setprecision(2);
        for (const auto& frame : sequence.frames) {
            clock.tick(frame.frame_index);
            const auto timestamp_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(clock.now()).count();
            std::cout << "frame=" << clock.frameIndex() << " t_ms=" << timestamp_ms
                      << " detections=" << frame.detections.size()
                      << " ego=" << (frame.ego_motion.has_value() ? "yes" : "no") << '\n';

            for (std::size_t index = 0U; index < frame.detections.size(); ++index) {
                const auto& detection = frame.detections[index];
                std::cout << "  detection[" << index << "] type=" << detection.object_type
                          << " gt_track_id=" << detection.ground_truth_track_id
                          << " pos=(" << detection.pos.x << ", " << detection.pos.y << ")\n";
            }

            if (frame.ego_motion.has_value()) {
                const auto& ego = *frame.ego_motion;
                std::cout << "  ego vf=" << ego.velocity_forward_mps
                          << " vl=" << ego.velocity_left_mps
                          << " yaw=" << ego.yaw_rad
                          << " yaw_rate_z=" << ego.angular_rate_z_radps << '\n';
            }
        }

        std::cout << "Replay completed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "TrackGuard startup failed: " << error.what() << '\n';
        return 1;
    }
}