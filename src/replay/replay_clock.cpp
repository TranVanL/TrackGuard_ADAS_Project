#include "trackguard/replay/replay_clock.h"

#include <cmath>
#include <stdexcept>

namespace trackguard::replay {

ReplayClock::ReplayClock(double frame_rate_hz) : frame_rate_hz_(frame_rate_hz) {
    if (!std::isfinite(frame_rate_hz_) || frame_rate_hz_ <= 0.0) {
        throw std::invalid_argument("ReplayClock frame rate must be finite and positive");
    }
}

void ReplayClock::reset() noexcept {
    current_frame_index_ = -1;
    current_timestamp_ = Timestamp{};
}

void ReplayClock::tick(int frame_index) {
    if (frame_index < 0) {
        throw std::invalid_argument("ReplayClock frame index must be non-negative");
    }
    if (hasTicked() && frame_index <= current_frame_index_) {
        throw std::logic_error("ReplayClock frame index must increase monotonically");
    }

    current_frame_index_ = frame_index;
    current_timestamp_ = timestampForFrame(frame_index);
}

int ReplayClock::frameIndex() const {
    if (!hasTicked()) {
        throw std::logic_error("ReplayClock has not ticked a frame yet");
    }
    return current_frame_index_;
}

Timestamp ReplayClock::now() const {
    if (!hasTicked()) {
        throw std::logic_error("ReplayClock has not ticked a frame yet");
    }
    return current_timestamp_;
}

Timestamp ReplayClock::timestampForFrame(int frame_index) const {
    if (frame_index < 0) {
        throw std::invalid_argument("ReplayClock frame index must be non-negative");
    }

    const auto timestamp_us = static_cast<long long>(std::llround(
        static_cast<double>(frame_index) * 1'000'000.0 / frame_rate_hz_));
    return Timestamp(timestamp_us);
}

}  // namespace trackguard::replay