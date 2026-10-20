#include "scene/particles.hpp"
#include "test.hpp"

using namespace skein;

TEST(particles_are_deterministic_for_a_seed) {
    ParticleBurst b;
    b.position = Vec3{0, 2, 0};
    b.velocity = Vec3{0, 1, 0};
    b.count = 500;
    b.colorCount = 3;
    b.colors[1] = Vec3{1, 0, 0};
    b.colors[2] = Vec3{0, 0, 1};
    ParticleSystem a, c;
    Random ra(7), rc(7);
    a.burst(b, ra);
    c.burst(b, rc);
    for (int i = 0; i < 60; ++i) {
        a.update(1.0f / 60.0f);
        c.update(1.0f / 60.0f);
    }
    CHECK_EQ(a.size(), c.size());
    for (size_t i = 0; i < a.size(); ++i) {
        CHECK_EQ(a.instances()[i].position.x, c.instances()[i].position.x);
        CHECK_EQ(a.instances()[i].position.y, c.instances()[i].position.y);
        CHECK_EQ(a.instances()[i].color, c.instances()[i].color);
    }
}

TEST(particles_die_rest_on_the_floor_and_respect_capacity) {
    ParticleSystem system;
    system.floor = 0.0f;
    system.capacity = 300;
    ParticleBurst b;
    b.position = Vec3{0, 1, 0};
    b.count = 200;
    b.life = 3.0f;
    b.lifeJitter = 0.0f;
    Random random(1);
    system.burst(b, random);
    system.burst(b, random);
    CHECK_EQ(system.size(), size_t{300});

    float lowest = 1e9f;
    for (int i = 0; i < 150; ++i) {
        system.update(1.0f / 60.0f);
        for (const ParticleInstance& p : system.instances()) lowest = std::min(lowest, p.position.y);
    }
    CHECK(lowest >= 0.0f);
    float resting = 0;
    for (const ParticleInstance& p : system.instances()) resting = std::max(resting, p.position.y);
    CHECK(resting < 0.05f);

    for (int i = 0; i < 40; ++i) system.update(1.0f / 60.0f);
    for (const ParticleInstance& p : system.instances()) CHECK(p.size < b.size * (1.0f + b.sizeJitter));
    for (int i = 0; i < 20; ++i) system.update(1.0f / 60.0f);
    CHECK_EQ(system.size(), size_t{0});
}
