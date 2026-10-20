#pragma once
#include <cmath>
#include <cstdint>

#include "core/math.hpp"

namespace skein::noise {

inline float hash(int x, int y, int z, uint32_t seed) {
    uint32_t h = seed ^ 0x9E3779B9u;
    h ^= static_cast<uint32_t>(x) * 0x85EBCA6Bu;
    h = (h ^ (h >> 13)) * 0xC2B2AE35u;
    h ^= static_cast<uint32_t>(y) * 0x27D4EB2Fu;
    h = (h ^ (h >> 15)) * 0x165667B1u;
    h ^= static_cast<uint32_t>(z) * 0x9E3779B1u;
    h = (h ^ (h >> 16)) * 0x85EBCA77u;
    h ^= h >> 13;
    return static_cast<float>(h >> 8) * (2.0f / 16777216.0f) - 1.0f;
}

inline float value(const Vec3& p, uint32_t seed = 0) {
    const float fx = std::floor(p.x), fy = std::floor(p.y), fz = std::floor(p.z);
    const int x = static_cast<int>(fx), y = static_cast<int>(fy), z = static_cast<int>(fz);
    auto fade = [](float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); };
    const float u = fade(p.x - fx), v = fade(p.y - fy), w = fade(p.z - fz);
    auto lerp = [](float a, float b, float t) { return a + (b - a) * t; };
    auto row = [&](int dy, int dz) { return lerp(hash(x, y + dy, z + dz, seed), hash(x + 1, y + dy, z + dz, seed), u); };
    return lerp(lerp(row(0, 0), row(1, 0), v), lerp(row(0, 1), row(1, 1), v), w);
}

inline float fbm(const Vec3& p, int octaves = 4, uint32_t seed = 0) {
    float sum = 0, amplitude = 0.5f, norm = 0;
    Vec3 q = p;
    for (int i = 0; i < octaves; ++i) {
        sum += value(q, seed + static_cast<uint32_t>(i) * 131u) * amplitude;
        norm += amplitude;
        amplitude *= 0.5f;
        q = q * 2.03f;
    }
    return sum / norm;
}

}  // namespace skein::noise
