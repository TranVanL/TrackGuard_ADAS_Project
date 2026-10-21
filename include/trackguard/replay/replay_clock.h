#pragma once

#include <chrono>

#include "trackguard/sensors/raw_detection.h"

namespace trackguard::replay {

using Timestamp = trackguard::sensors::Timestamp;

class ReplayClock {
public:
    explicit ReplayClock(double frame_rate_hz = 10.0);

    void reset() noexcept;

    // Advance virtual time to a KITTI frame. Frames must be supplied in
    // strictly increasing order; no wall-clock sleeping occurs here.
    void tick(int frame_index);

    [[nodiscard]] bool hasTicked() const noexcept { return current_frame_index_ >= 0; }
    [[nodiscard]] int frameIndex() const;
    [[nodiscard]] Timestamp now() const;
    [[nodiscard]] Timestamp timestampForFrame(int frame_index) const;
    [[nodiscard]] double frameRateHz() const noexcept { return frame_rate_hz_; }

private:
    double frame_rate_hz_;
    int current_frame_index_{-1};
    Timestamp current_timestamp_{};
};

}  // namespace trackguard::replay