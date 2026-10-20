#pragma once
#include <cstdint>
#include <limits>
#include <vector>

#include "core/math.hpp"
#include "core/random.hpp"

namespace skein {

struct ParticleInstance {
    Vec3 position;
    float size;
    uint32_t color;
    float rotation;
    float glow;
};

struct ParticleBurst {
    Vec3 position;
    Vec3 velocity;
    float speed = 4.0f;
    float spread = 1.0f;
    int count = 100;
    float life = 2.0f;
    float lifeJitter = 0.3f;
    float size = 0.1f;
    float sizeJitter = 0.3f;
    float spin = 6.0f;
    float glow = 1.0f;
    Vec3 colors[4] = {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}};
    int colorCount = 1;
};

class ParticleSystem {
public:
    Vec3 gravity{0.0f, -9.81f, 0.0f};
    float drag = 0.5f;
    float floor = -std::numeric_limits<float>::infinity();
    float bounce = 0.3f;
    float friction = 0.6f;
    bool additive = false;
    bool round = false;
    size_t capacity = 200000;

    void burst(const ParticleBurst& b, Random& random);
    void update(float dt);
    void clear();

    size_t size() const { return instances_.size(); }
    const std::vector<ParticleInstance>& instances() const { return instances_; }

private:
    struct State {
        Vec3 velocity;
        float age;
        float life;
        float baseSize;
        float spin;
    };

    std::vector<ParticleInstance> instances_;
    std::vector<State> states_;
};

}  // namespace skein
