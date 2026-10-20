#include "scene/particles.hpp"

#include <algorithm>

#include "scene/components.hpp"

namespace skein {

void ParticleSystem::burst(const ParticleBurst& b, Random& random) {
    const int room = static_cast<int>(capacity > instances_.size() ? capacity - instances_.size() : 0);
    const int count = std::min(b.count, room);
    const int colors = std::clamp(b.colorCount, 1, 4);
    const Vec3 base = length2(b.velocity) > 0 ? normalize(b.velocity) : Vec3{0, 1, 0};
    for (int i = 0; i < count; ++i) {
        Vec3 direction = normalize(base + random.inUnitSphere() * b.spread);
        float speed = b.speed * random.uniform(0.5f, 1.0f);
        float size = b.size * (1.0f + b.sizeJitter * random.signedUnit());
        Vec3 color = b.colors[colors > 1 ? random.below(static_cast<uint32_t>(colors)) : 0];
        instances_.push_back(ParticleInstance{b.position, size, packTint(color), random.uniform(0.0f, 2.0f * PI),
                                              b.glow});
        states_.push_back(State{direction * speed, 0.0f, b.life * (1.0f + b.lifeJitter * random.signedUnit()), size,
                                b.spin * random.signedUnit()});
    }
}

void ParticleSystem::update(float dt) {
    const float damping = std::exp(-drag * dt);
    for (size_t i = 0; i < instances_.size();) {
        State& s = states_[i];
        ParticleInstance& p = instances_[i];
        s.age += dt;
        if (s.age >= s.life) {
            instances_[i] = instances_.back();
            states_[i] = states_.back();
            instances_.pop_back();
            states_.pop_back();
            continue;
        }
        s.velocity = (s.velocity + gravity * dt) * damping;
        p.position += s.velocity * dt;
        if (p.position.y < floor) {
            p.position.y = floor;
            s.velocity.y = -s.velocity.y * bounce;
            s.velocity.x *= friction;
            s.velocity.z *= friction;
            s.spin *= friction;
        }
        p.rotation += s.spin * dt;
        float remaining = (s.life - s.age) / (0.25f * s.life);
        p.size = s.baseSize * std::min(1.0f, remaining);
        ++i;
    }
}

void ParticleSystem::clear() {
    instances_.clear();
    states_.clear();
}

}  // namespace skein
