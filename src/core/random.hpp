#pragma once
#include <cmath>
#include <cstdint>
#include <limits>

#include "core/math.hpp"

namespace skein {

class Random {
public:
    using result_type = uint32_t;

    explicit Random(uint64_t seed = 0x853C49E6748FEA9Bull, uint64_t stream = 0xDA3E39CB94B95BDBull) {
        reseed(seed, stream);
    }

    void reseed(uint64_t seed, uint64_t stream = 0xDA3E39CB94B95BDBull) {
        state_ = 0;
        inc_ = (stream << 1) | 1u;
        next();
        state_ += seed;
        next();
    }

    uint32_t next() {
        uint64_t old = state_;
        state_ = old * 6364136223846793005ull + inc_;
        uint32_t shifted = static_cast<uint32_t>(((old >> 18) ^ old) >> 27);
        uint32_t rot = static_cast<uint32_t>(old >> 59);
        return (shifted >> rot) | (shifted << ((0u - rot) & 31u));
    }

    uint32_t operator()() { return next(); }
    static constexpr uint32_t min() { return 0; }
    static constexpr uint32_t max() { return std::numeric_limits<uint32_t>::max(); }

    uint32_t below(uint32_t bound) {
        uint64_t m = static_cast<uint64_t>(next()) * bound;
        uint32_t low = static_cast<uint32_t>(m);
        if (low < bound) {
            uint32_t threshold = (0u - bound) % bound;
            while (low < threshold) {
                m = static_cast<uint64_t>(next()) * bound;
                low = static_cast<uint32_t>(m);
            }
        }
        return static_cast<uint32_t>(m >> 32);
    }

    int range(int lo, int hi) { return lo + static_cast<int>(below(static_cast<uint32_t>(hi - lo + 1))); }
    float unit() { return static_cast<float>(next() >> 8) * (1.0f / 16777216.0f); }
    float uniform(float lo, float hi) { return lo + (hi - lo) * unit(); }
    float signedUnit() { return unit() * 2.0f - 1.0f; }
    bool chance(float p) { return unit() < p; }

    float normal() {
        float u = 1.0f - unit();
        float v = unit();
        return std::sqrt(-2.0f * std::log(u)) * std::cos(2.0f * PI * v);
    }

    Vec3 inUnitSphere() {
        for (;;) {
            Vec3 p{signedUnit(), signedUnit(), signedUnit()};
            if (length2(p) <= 1.0f) return p;
        }
    }

    Random split() {
        uint64_t seed = static_cast<uint64_t>(next()) << 32;
        seed |= next();
        uint64_t stream = static_cast<uint64_t>(next()) << 32;
        stream |= next();
        return Random(seed, stream);
    }

    uint64_t state() const { return state_; }
    uint64_t increment() const { return inc_; }

private:
    uint64_t state_ = 0;
    uint64_t inc_ = 1;
};

}  // namespace skein
