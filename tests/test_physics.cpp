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

TEST(broadphase_stays_exact_when_collider_sizes_vary_wildly) {
    std::mt19937 rng(91);
    std::uniform_real_distribution<float> p(-30.0f, 30.0f);
    std::uniform_real_distribution<float> small(0.1f, 0.5f);
    std::uniform_real_distribution<float> huge(3.0f, 7.0f);

    Scene scene;
    std::vector<Vec3> positions;
    std::vector<float> radii;
    for (int i = 0; i < 4000; ++i) {
        Vec3 pos{p(rng), p(rng), p(rng)};
        float rad = (i % 40 == 0) ? huge(rng) : small(rng);
        positions.push_back(pos);
        radii.push_back(rad);
        spawnSphere(scene, pos, Vec3{0, 0, 0}, rad);
    }

    PhysicsWorld physics;
    physics.settings.gravity = Vec3{0, 0, 0};
    physics.settings.linearDamping = 0.0f;
    physics.settings.useBounds = false;
    physics.settings.cellSize = 1.0f;
    PhysicsStats stats = physics.step(scene, 0.0f, nullptr);

    auto expected = bruteForceOverlaps(positions, radii);
    CHECK(!expected.empty());
    CHECK_EQ(static_cast<size_t>(stats.contacts), expected.size());
}

TEST(coloured_solver_matches_between_one_thread_and_many) {
    auto buildPile = [](Scene& scene) {
        std::mt19937 rng(5);
        std::uniform_real_distribution<float> p(-6.0f, 6.0f);
        std::uniform_real_distribution<float> h(0.5f, 22.0f);
        std::uniform_real_distribution<float> rad(0.35f, 0.8f);
        for (int i = 0; i < 4000; ++i)
            spawnSphere(scene, Vec3{p(rng), h(rng), p(rng)}, Vec3{0, 0, 0}, rad(rng));
    };

    Scene serialScene, threadedScene;
    buildPile(serialScene);
    buildPile(threadedScene);

    PhysicsWorld serialPhysics, threadedPhysics;
    serialPhysics.settings.boundsMin = threadedPhysics.settings.boundsMin = Vec3{-8, 0, -8};
    serialPhysics.settings.boundsMax = threadedPhysics.settings.boundsMax = Vec3{8, 40, 8};
    JobSystem jobs(6);

    PhysicsStats last{};
    for (int i = 0; i < 90; ++i) {
        serialPhysics.step(serialScene, 1.0f / 60.0f, nullptr);
        last = threadedPhysics.step(threadedScene, 1.0f / 60.0f, &jobs);
    }

    CHECK(last.contacts > 2000);
    CHECK(last.colors > 1);

    Pool<Transform>& a = serialScene.world.pool<Transform>();
    Pool<Transform>& b = threadedScene.world.pool<Transform>();
    CHECK_EQ(a.data.size(), b.data.size());
    for (size_t i = 0; i < a.data.size(); ++i) {
        CHECK_EQ(a.dense[i], b.dense[i]);
        CHECK(a.data[i].position.x == b.data[i].position.x);
        CHECK(a.data[i].position.y == b.data[i].position.y);
        CHECK(a.data[i].position.z == b.data[i].position.z);
    }
}

TEST(a_stack_holds_its_height_at_the_default_iteration_count) {
    // Cancelling each contact's approach velocity from scratch every frame
    // cannot hold a stack up: the reaction has to travel down the column, and
    // two iterations is nowhere near enough hops. The impulse cached from last
    // frame is what makes the column stand.
    Scene scene;
    PhysicsWorld world;
    world.settings.boundsMin = Vec3{-6, 0, -6};
    world.settings.boundsMax = Vec3{6, 40, 6};
    world.settings.restitutionFloor = 0.0f;

    const float radius = 0.5f;
    std::vector<Entity> column;
    for (int i = 0; i < 8; ++i) {
        Entity e = spawnSphere(scene, Vec3{0, radius + static_cast<float>(i) * 1.02f, 0}, Vec3{0, 0, 0}, radius);
        scene.world.tryGet<Collider>(e)->restitution = 0.0f;
        column.push_back(e);
    }

    for (int i = 0; i < 600; ++i) world.step(scene, 1.0f / 60.0f);

    for (size_t i = 1; i < column.size(); ++i) {
        float gap = scene.world.tryGet<Transform>(column[i])->position.y -
                    scene.world.tryGet<Transform>(column[i - 1])->position.y;
        CHECK(gap > 2.0f * radius - 0.08f);
    }
    float top = scene.world.tryGet<Transform>(column.back())->position.y;
    CHECK(top > radius + 7.0f * (2.0f * radius - 0.08f));
}

TEST(a_settled_pile_falls_asleep_and_a_moving_body_wakes_it) {
    Scene scene;
    PhysicsWorld world;
    world.settings.boundsMin = Vec3{-6, 0, -6};
    world.settings.boundsMax = Vec3{6, 40, 6};
    world.settings.restitutionFloor = 0.0f;

    std::mt19937 rng(31337);
    std::uniform_real_distribution<float> u(-4.0f, 4.0f);
    std::vector<Entity> bodies;
    for (int i = 0; i < 400; ++i) {
        Entity e = spawnSphere(scene, Vec3{u(rng), 1.0f + static_cast<float>(i) * 0.05f, u(rng)}, Vec3{0, 0, 0}, 0.5f);
        scene.world.tryGet<Collider>(e)->restitution = 0.0f;
        bodies.push_back(e);
    }

    for (int i = 0; i < 1200; ++i) world.step(scene, 1.0f / 60.0f);
    const PhysicsStats& settled = world.stats();
    CHECK(settled.awake * 4 < settled.bodies);

    std::vector<Vec3> before;
    for (Entity e : bodies) before.push_back(scene.world.tryGet<Transform>(e)->position);
    world.step(scene, 1.0f / 60.0f);
    double drift = 0;
    for (size_t i = 0; i < bodies.size(); ++i)
        drift += length(scene.world.tryGet<Transform>(bodies[i])->position - before[i]);
    // A sleeping pile is not merely slow, it is still.
    CHECK(drift / static_cast<double>(bodies.size()) < 0.002);

    Entity bullet = spawnSphere(scene, Vec3{0, 30, 0}, Vec3{0, -60, 0}, 0.5f);
    uint32_t sleepingBefore = world.stats().bodies - world.stats().awake;
    for (int i = 0; i < 40; ++i) world.step(scene, 1.0f / 60.0f);
    uint32_t sleepingAfter = world.stats().bodies - world.stats().awake;
    CHECK(sleepingAfter < sleepingBefore);
    CHECK(scene.world.tryGet<Transform>(bullet)->position.y < 25.0f);
}

TEST(a_sleeping_pile_is_no_more_interpenetrated_than_an_awake_one) {
    // Freezing a body is only sound if it was resting somewhere legal. Deep
    // overlaps are the failure mode: once two overlapping bodies both sleep,
    // nothing is left to push them apart.
    const float radius = 0.5f;
    auto settle = [&](bool allowSleep) {
        Scene scene;
        PhysicsWorld world;
        world.settings.allowSleep = allowSleep;
        world.settings.boundsMin = Vec3{-6, 0, -6};
        world.settings.boundsMax = Vec3{6, 40, 6};
        world.settings.restitutionFloor = 0.0f;
        std::mt19937 rng(4242);
        std::uniform_real_distribution<float> u(-4.0f, 4.0f);
        std::vector<Entity> bodies;
        for (int i = 0; i < 300; ++i) {
            Entity e =
                spawnSphere(scene, Vec3{u(rng), 1.0f + static_cast<float>(i) * 0.06f, u(rng)}, Vec3{0, 0, 0}, radius);
            scene.world.tryGet<Collider>(e)->restitution = 0.0f;
            bodies.push_back(e);
        }
        for (int i = 0; i < 1400; ++i) world.step(scene, 1.0f / 60.0f);
        std::vector<Vec3> pos;
        for (Entity e : bodies) pos.push_back(scene.world.tryGet<Transform>(e)->position);
        float worst = 0;
        for (size_t i = 0; i < pos.size(); ++i) {
            CHECK(pos[i].y >= radius - 0.05f);
            // A body may not fall asleep buried in a wall either.
            CHECK(std::abs(pos[i].x) <= 6.0f - radius + 0.05f);
            CHECK(std::abs(pos[i].z) <= 6.0f - radius + 0.05f);
            for (size_t j = i + 1; j < pos.size(); ++j)
                worst = std::max(worst, 2.0f * radius - length(pos[j] - pos[i]));
        }
        return std::make_pair(worst, world.stats().awake);
    };
    auto [sleepingOverlap, sleepingAwake] = settle(true);
    auto [awakeOverlap, awakeCount] = settle(false);
    CHECK(sleepingAwake * 4 < 300u);
    CHECK_EQ(awakeCount, 300u);
    CHECK(sleepingOverlap <= awakeOverlap + 0.02f);
}
