#include "physics/physics.hpp"
#include "test.hpp"

#include <algorithm>
#include <random>
#include <set>

#include "core/jobs.hpp"
#include "scene/scene.hpp"

using namespace skein;

namespace {

Entity spawnSphere(Scene& s, Vec3 pos, Vec3 vel, float radius, float invMass = 1.0f) {
    Transform t;
    t.position = pos;
    Entity e = s.create(t);
    s.world.add<Velocity>(e, Velocity{vel, Vec3{0, 0, 0}});
    Collider c;
    c.radius = radius;
    c.invMass = invMass;
    c.kind = static_cast<uint32_t>(ColliderKind::Sphere);
    s.world.add<Collider>(e, c);
    return e;
}

/// Reference broadphase: every unordered pair, tested directly.
std::set<std::pair<uint32_t, uint32_t>> bruteForceOverlaps(const std::vector<Vec3>& pos,
                                                           const std::vector<float>& radius) {
    std::set<std::pair<uint32_t, uint32_t>> out;
    for (uint32_t i = 0; i < pos.size(); ++i)
        for (uint32_t j = i + 1; j < pos.size(); ++j) {
            float r = radius[i] + radius[j];
            if (length2(pos[j] - pos[i]) < r * r) out.insert({i, j});
        }
    return out;
}

}  // namespace

TEST(hashed_grid_finds_the_same_contacts_as_brute_force) {
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> p(-20.0f, 20.0f);
    std::uniform_real_distribution<float> r(0.2f, 0.9f);

    Scene scene;
    std::vector<Vec3> positions;
    std::vector<float> radii;
    for (int i = 0; i < 3000; ++i) {
        Vec3 pos{p(rng), p(rng), p(rng)};
        float rad = r(rng);
        positions.push_back(pos);
        radii.push_back(rad);
        spawnSphere(scene, pos, Vec3{0, 0, 0}, rad);
    }

    PhysicsWorld physics;
    physics.settings.gravity = Vec3{0, 0, 0};
    physics.settings.linearDamping = 0.0f;
    physics.settings.useBounds = false;
    physics.settings.cellSize = 2.0f;
    PhysicsStats stats = physics.step(scene, 0.0f, nullptr);

    auto expected = bruteForceOverlaps(positions, radii);
    CHECK_EQ(static_cast<size_t>(stats.contacts), expected.size());
    CHECK(stats.pairsTested < static_cast<uint64_t>(positions.size()) * positions.size() / 8);
}

TEST(hashed_grid_matches_brute_force_when_threaded) {
    std::mt19937 rng(13);
    std::uniform_real_distribution<float> p(-15.0f, 15.0f);

    Scene scene;
    std::vector<Vec3> positions;
    std::vector<float> radii;
    for (int i = 0; i < 6000; ++i) {
        Vec3 pos{p(rng), p(rng), p(rng)};
        positions.push_back(pos);
        radii.push_back(0.5f);
        spawnSphere(scene, pos, Vec3{0, 0, 0}, 0.5f);
    }

    JobSystem jobs(4);
    PhysicsWorld physics;
    physics.settings.gravity = Vec3{0, 0, 0};
    physics.settings.linearDamping = 0.0f;
    physics.settings.useBounds = false;
    PhysicsStats stats = physics.step(scene, 0.0f, &jobs);
    CHECK_EQ(static_cast<size_t>(stats.contacts), bruteForceOverlaps(positions, radii).size());
}

TEST(head_on_spheres_bounce_apart) {
    Scene scene;
    Entity a = spawnSphere(scene, Vec3{-1.0f, 0, 0}, Vec3{4.0f, 0, 0}, 0.6f);
    Entity b = spawnSphere(scene, Vec3{1.0f, 0, 0}, Vec3{-4.0f, 0, 0}, 0.6f);

    PhysicsWorld physics;
    physics.settings.gravity = Vec3{0, 0, 0};
    physics.settings.linearDamping = 0.0f;
    physics.settings.useBounds = false;

    bool touched = false;
    for (int i = 0; i < 60; ++i) {
        PhysicsStats s = physics.step(scene, 1.0f / 60.0f, nullptr);
        if (s.contacts > 0) touched = true;
    }
    CHECK(touched);

    float va = scene.world.get<Velocity>(a).linear.x;
    float vb = scene.world.get<Velocity>(b).linear.x;
    CHECK(va < 0.0f);
    CHECK(vb > 0.0f);
    float gap = scene.world.get<Transform>(b).position.x - scene.world.get<Transform>(a).position.x;
    CHECK(gap > 1.1f);
}

TEST(a_static_body_is_not_pushed_by_a_dynamic_one) {
    Scene scene;
    Entity wall = spawnSphere(scene, Vec3{0, 0, 0}, Vec3{0, 0, 0}, 1.0f, 0.0f);
    Entity ball = spawnSphere(scene, Vec3{3.0f, 0, 0}, Vec3{-5.0f, 0, 0}, 0.5f, 1.0f);

    PhysicsWorld physics;
    physics.settings.gravity = Vec3{0, 0, 0};
    physics.settings.linearDamping = 0.0f;
    physics.settings.useBounds = false;
    for (int i = 0; i < 90; ++i) physics.step(scene, 1.0f / 60.0f, nullptr);

    CHECK_NEAR(scene.world.get<Transform>(wall).position.x, 0.0, 1e-4);
    CHECK_NEAR(scene.world.get<Velocity>(wall).linear.x, 0.0, 1e-4);
    CHECK(scene.world.get<Velocity>(ball).linear.x > 0.0f);
}

TEST(bounded_simulation_keeps_falling_bodies_inside_the_box) {
    std::mt19937 rng(19);
    std::uniform_real_distribution<float> p(-10.0f, 10.0f);
    Scene scene;
    for (int i = 0; i < 800; ++i)
        spawnSphere(scene, Vec3{p(rng), 20.0f + p(rng), p(rng)}, Vec3{p(rng), 0, p(rng)}, 0.5f);

    PhysicsWorld physics;
    physics.settings.boundsMin = Vec3{-15, 0, -15};
    physics.settings.boundsMax = Vec3{15, 40, 15};

    JobSystem jobs(3);
    for (int i = 0; i < 240; ++i) physics.step(scene, 1.0f / 60.0f, &jobs);

    forEach<Transform>(scene.world, [&](Entity, Transform& t) {
        CHECK(t.position.y >= -0.01f);
        CHECK(t.position.y <= 40.01f);
        CHECK(std::fabs(t.position.x) <= 15.01f);
        CHECK(std::fabs(t.position.z) <= 15.01f);
        CHECK(std::isfinite(t.position.x) && std::isfinite(t.position.y) && std::isfinite(t.position.z));
    });
}

TEST(resting_pile_does_not_gain_energy) {
    Scene scene;
    for (int i = 0; i < 200; ++i)
        spawnSphere(scene, Vec3{static_cast<float>(i % 10) - 5.0f, 1.0f + static_cast<float>(i / 10),
                                static_cast<float>(i % 7) - 3.0f},
                    Vec3{0, 0, 0}, 0.5f);

    PhysicsWorld physics;
    physics.settings.boundsMin = Vec3{-10, 0, -10};
    physics.settings.boundsMax = Vec3{10, 50, 10};
    for (int i = 0; i < 600; ++i) physics.step(scene, 1.0f / 60.0f, nullptr);

    double kinetic = 0;
    forEach<Velocity>(scene.world, [&](Entity, Velocity& v) { kinetic += length2(v.linear); });
    CHECK(std::isfinite(kinetic));
    CHECK(kinetic < 200.0 * 25.0);
}

TEST(sphere_resolves_against_a_box_collider) {
    Scene scene;
    Transform boxT;
    boxT.position = Vec3{0, 0, 0};
    Entity box = scene.create(boxT);
    Collider bc;
    bc.kind = static_cast<uint32_t>(ColliderKind::Box);
    bc.halfExtents = Vec3{2.0f, 0.5f, 2.0f};
    bc.invMass = 0.0f;
    scene.world.add<Collider>(box, bc);
    scene.world.add<Velocity>(box, Velocity{});

    Entity ball = spawnSphere(scene, Vec3{0, 3.0f, 0}, Vec3{0, 0, 0}, 0.5f);

    PhysicsWorld physics;
    physics.settings.useBounds = false;
    for (int i = 0; i < 300; ++i) physics.step(scene, 1.0f / 60.0f, nullptr);

    float y = scene.world.get<Transform>(ball).position.y;
    CHECK(y > 0.4f);
    CHECK(y < 1.6f);
}
