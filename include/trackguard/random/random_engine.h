#pragma once

#include <cstdint>
#include <random>
#include <stdexcept>

namespace trackguard::random {

class RandomEngine {
public:
    explicit RandomEngine(std::uint64_t seed) : seed_(seed), engine_(seed) {}

    [[nodiscard]] std::uint64_t seed() const noexcept { return seed_; }

    std::uint64_t nextUInt64() { return engine_(); }

    double uniform(double lower, double upper) {
        if (lower > upper) {
            throw std::invalid_argument("RandomEngine uniform bounds are reversed");
        }
        std::uniform_real_distribution<double> distribution(lower, upper);
        return distribution(engine_);
    }

    double normal(double mean, double standard_deviation) {
        if (standard_deviation < 0.0) {
            throw std::invalid_argument("RandomEngine normal standard deviation is negative");
        }
        std::normal_distribution<double> distribution(mean, standard_deviation);
        return distribution(engine_);
    }

private:
    std::uint64_t seed_;
    std::mt19937_64 engine_;
};

}  // namespace trackguard::random