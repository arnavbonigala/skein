#include "scene/scene.hpp"
#include "test.hpp"

#include <cstring>
#include <random>

#include "core/jobs.hpp"
#include "physics/physics.hpp"
#include "render/render_list.hpp"
#include "scene/serialize.hpp"

using namespace skein;

namespace {

Transform trs(Vec3 p, Vec3 s = Vec3{1, 1, 1}, float yaw = 0.0f) {
    Transform t;
    t.position = p;
    t.scale = s;
    t.rotation = Quat::euler(0, yaw, 0);
    return t;
}

Mat4 expectedWorld(const Scene& scene, Entity e) {
    Mat4 acc = Mat4::identity();
    std::vector<Entity> chain;
    Entity cur = e;
    while (cur != NULL_ENTITY) {
        chain.push_back(cur);
        cur = scene.parentOf(cur);
    }
    for (size_t i = chain.size(); i-- > 0;) {
        const Transform* t = scene.world.tryGet<Transform>(chain[i]);
        acc = acc * composeTRS(t->position, t->rotation, t->scale);
    }
    return acc;
}

void checkMatrixNear(const Mat4& a, const Mat4& b, double eps) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) CHECK_NEAR(a.m[c][r], b.m[c][r], eps);
}

}  // namespace

TEST(nested_transforms_match_a_manual_parent_chain) {
    Scene scene;
    Entity root = scene.create(trs(Vec3{5, 0, 0}, Vec3{2, 2, 2}, 0.7f));
    Entity mid = scene.create(trs(Vec3{0, 3, 0}, Vec3{0.5f, 0.5f, 0.5f}, -0.3f), root);
    Entity leaf = scene.create(trs(Vec3{1, 0, 1}, Vec3{1, 1, 1}, 1.2f), mid);

    scene.updateTransforms(nullptr);
    checkMatrixNear(scene.worldMatrix(leaf), expectedWorld(scene, leaf), 1e-4);
    checkMatrixNear(scene.worldMatrix(mid), expectedWorld(scene, mid), 1e-4);
    CHECK_EQ(scene.depthLevels(), size_t{3});
}

TEST(deep_chains_resolve_in_depth_order) {
    Scene scene;
    const int depth = 64;
    std::vector<Entity> chain;
    Entity parent = NULL_ENTITY;
    for (int i = 0; i < depth; ++i) {
        Entity e = scene.create(trs(Vec3{0, 1, 0}), parent);
        chain.push_back(e);
        parent = e;
    }
    scene.updateTransforms(nullptr);
    CHECK_EQ(scene.depthLevels(), static_cast<size_t>(depth));
    CHECK_NEAR(scene.worldMatrix(chain.back()).m[3][1], static_cast<double>(depth), 1e-3);
}

TEST(threaded_transform_update_matches_single_threaded) {
    Scene serial;
    Scene threaded;
    std::mt19937 rng(101);
    std::uniform_real_distribution<float> u(-10.0f, 10.0f);

    std::vector<Entity> roots[2];
    Scene* scenes[2] = {&serial, &threaded};
    for (int s = 0; s < 2; ++s) {
        std::mt19937 local(101);
        std::uniform_real_distribution<float> d(-10.0f, 10.0f);
        for (int i = 0; i < 5000; ++i) {
            Entity parent = NULL_ENTITY;
            if (i > 100 && (local() % 4) == 0) parent = roots[s][local() % roots[s].size()];
            Entity e = scenes[s]->create(trs(Vec3{d(local), d(local), d(local)}, Vec3{1, 1, 1}, d(local)), parent);
            roots[s].push_back(e);
        }
    }
    (void)rng;
    (void)u;

    JobSystem jobs(4);
    serial.updateTransforms(nullptr);
    threaded.updateTransforms(&jobs);

    for (size_t i = 0; i < roots[0].size(); ++i)
        checkMatrixNear(serial.worldMatrix(roots[0][i]), threaded.worldMatrix(roots[1][i]), 1e-5);
}

TEST(reparenting_updates_the_resolved_world_matrix) {
    Scene scene;
    Entity a = scene.create(trs(Vec3{10, 0, 0}));
    Entity b = scene.create(trs(Vec3{0, 0, 20}));
    Entity child = scene.create(trs(Vec3{1, 1, 1}), a);
    scene.updateTransforms(nullptr);
    CHECK_NEAR(scene.worldMatrix(child).m[3][0], 11.0, 1e-4);

    scene.setParent(child, b);
    scene.updateTransforms(nullptr);
    CHECK_NEAR(scene.worldMatrix(child).m[3][0], 1.0, 1e-4);
    CHECK_NEAR(scene.worldMatrix(child).m[3][2], 21.0, 1e-4);

    scene.setParent(child, NULL_ENTITY);
    scene.updateTransforms(nullptr);
    CHECK_NEAR(scene.worldMatrix(child).m[3][2], 1.0, 1e-4);
}

TEST(destroying_a_parent_promotes_children_to_roots) {
    Scene scene;
    Entity parent = scene.create(trs(Vec3{100, 0, 0}));
    Entity child = scene.create(trs(Vec3{1, 0, 0}), parent);
    scene.updateTransforms(nullptr);
    CHECK_NEAR(scene.worldMatrix(child).m[3][0], 101.0, 1e-4);

    scene.destroy(parent);
    scene.updateTransforms(nullptr);
    CHECK_NEAR(scene.worldMatrix(child).m[3][0], 1.0, 1e-4);
}

TEST(cull_bounds_follow_the_world_transform) {
    Scene scene;
    Entity e = scene.create(trs(Vec3{4, 0, 0}, Vec3{3, 3, 3}));
    CullBounds cb;
    cb.localCenter = Vec3{0, 0, 0};
    cb.localExtent = Vec3{0.5f, 0.5f, 0.5f};
    scene.world.add<CullBounds>(e, cb);
    scene.world.add<Renderable>(e, Renderable{});
    scene.updateTransforms(nullptr);

    const CullBounds& out = scene.world.get<CullBounds>(e);
    CHECK_NEAR(out.center.x, 4.0, 1e-4);
    CHECK_NEAR(out.extent.x, 1.5, 1e-4);
    CHECK_NEAR(out.radius, length(Vec3{1.5f, 1.5f, 1.5f}), 1e-4);
}

TEST(culling_keeps_exactly_the_objects_a_brute_force_test_keeps) {
    Scene scene;
    std::mt19937 rng(77);
    std::uniform_real_distribution<float> p(-120.0f, 120.0f);
    for (int i = 0; i < 20000; ++i) {
        Entity e = scene.create(trs(Vec3{p(rng), p(rng), p(rng)}));
        scene.world.add<Renderable>(e, Renderable{static_cast<uint32_t>(i % 4), static_cast<uint32_t>(i % 3), 1, 0});
        CullBounds cb;
        cb.localExtent = Vec3{0.5f, 0.5f, 0.5f};
        scene.world.add<CullBounds>(e, cb);
    }

    JobSystem jobs(4);
    scene.updateTransforms(&jobs);

    Mat4 vp = perspective(radians(60.0f), 16.0f / 9.0f, 0.5f, 250.0f) *
              lookAt(Vec3{0, 30, 90}, Vec3{0, 0, 0}, Vec3{0, 1, 0});
    Frustum f = extractFrustum(vp);

    uint32_t expected = 0;
    forEach<CullBounds>(scene.world, [&](Entity, CullBounds& cb) {
        if (frustumIntersectsSphere(f, cb.center, cb.radius) && frustumIntersectsAABB(f, cb.center, cb.extent))
            ++expected;
    });

    CullSystem culler;
    RenderList list;
    culler.build(scene, f, 3, list, &jobs);

    CHECK_EQ(list.totalCandidates, uint32_t{20000});
    CHECK_EQ(list.visible, expected);
    CHECK(list.visible > 0);
    CHECK(list.visible < 20000);
    CHECK_EQ(list.instances.size(), static_cast<size_t>(list.visible));

    uint32_t summed = 0;
    for (const DrawBatch& b : list.batches) {
        CHECK_EQ(b.first, summed);
        summed += b.count;
    }
    CHECK_EQ(summed, list.visible);
    CHECK(list.batches.size() <= 12);
}

TEST(hidden_renderables_are_excluded_from_batches) {
    Scene scene;
    for (int i = 0; i < 100; ++i) {
        Entity e = scene.create(trs(Vec3{0, 0, -5}));
        scene.world.add<Renderable>(e, Renderable{0, 0, static_cast<uint32_t>(i % 2), 0});
        scene.world.add<CullBounds>(e, CullBounds{});
    }
    scene.updateTransforms(nullptr);
    Frustum f = extractFrustum(perspective(radians(90.0f), 1.0f, 0.1f, 100.0f));
    CullSystem culler;
    RenderList list;
    culler.build(scene, f, 1, list, nullptr);
    CHECK_EQ(list.visible, uint32_t{50});
}

TEST(scene_survives_a_serialization_round_trip) {
    Scene original;
    std::mt19937 rng(43);
    std::uniform_real_distribution<float> u(-30.0f, 30.0f);

    std::vector<Entity> created;
    for (int i = 0; i < 2000; ++i) {
        Entity parent = NULL_ENTITY;
        if (i > 50 && (rng() % 5) == 0) parent = created[rng() % created.size()];
        Entity e = original.create(trs(Vec3{u(rng), u(rng), u(rng)}, Vec3{1, 2, 3}, u(rng)), parent);
        created.push_back(e);
        if (i % 2 == 0)
            original.world.add<Renderable>(e, Renderable{static_cast<uint32_t>(i % 5), static_cast<uint32_t>(i % 3), 1, 0});
        if (i % 3 == 0) original.world.add<Velocity>(e, Velocity{Vec3{u(rng), u(rng), u(rng)}, Vec3{0, 0, 0}});
        if (i % 7 == 0) {
            Collider c;
            c.radius = 0.25f + static_cast<float>(i % 4);
            original.world.add<Collider>(e, c);
        }
    }
    for (int i = 0; i < 400; ++i) original.destroy(created[static_cast<size_t>(rng()) % created.size()]);
    original.updateTransforms(nullptr);

    std::vector<uint8_t> blob = serializeScene(original);
    Scene restored;
    std::string error;
    CHECK(deserializeScene(restored, blob.data(), blob.size(), error));
    CHECK(error.empty());

    CHECK_EQ(restored.world.aliveCount(), original.world.aliveCount());
    CHECK_EQ(restored.world.pool<Renderable>().size(), original.world.pool<Renderable>().size());
    CHECK_EQ(restored.world.pool<Collider>().size(), original.world.pool<Collider>().size());
    CHECK_EQ(restored.world.pool<Parent>().size(), original.world.pool<Parent>().size());

    restored.updateTransforms(nullptr);
    original.world.each([&](Entity e) {
        CHECK(restored.world.alive(e));
        const Transform* a = original.world.tryGet<Transform>(e);
        const Transform* b = restored.world.tryGet<Transform>(e);
        CHECK(a && b);
        CHECK_NEAR(a->position.x, b->position.x, 1e-6);
        CHECK_NEAR(a->rotation.y, b->rotation.y, 1e-6);
        CHECK_EQ(original.parentOf(e), restored.parentOf(e));
        checkMatrixNear(original.worldMatrix(e), restored.worldMatrix(e), 1e-4);
    });
}

TEST(empty_and_single_entity_scenes_survive_every_system) {
    Scene scene;
    JobSystem jobs(3);
    PhysicsWorld physics;
    CullSystem culler;
    RenderList list;
    Frustum f = extractFrustum(perspective(radians(60.0f), 1.6f, 0.1f, 100.0f) *
                               lookAt(Vec3{0, 2, 8}, Vec3{0, 0, 0}, Vec3{0, 1, 0}));

    scene.updateTransforms(&jobs);
    physics.step(scene, 1.0f / 60.0f, &jobs);
    culler.sortSpatially(scene);
    culler.maintain(scene);
    culler.build(scene, f, 4, list, &jobs);
    CHECK_EQ(list.visible, uint32_t{0});
    CHECK_EQ(list.drawCalls(), uint32_t{0});

    std::vector<uint8_t> blob = serializeScene(scene);
    Scene restored;
    std::string error;
    CHECK(deserializeScene(restored, blob.data(), blob.size(), error));
    CHECK_EQ(restored.world.aliveCount(), size_t{0});

    Entity only = scene.create(trs(Vec3{0, 0, 0}));
    scene.world.add<Renderable>(only, Renderable{0, 0, 1, 0});
    CullBounds cb;
    cb.localExtent = Vec3{1, 1, 1};
    scene.world.add<CullBounds>(only, cb);
    Collider c;
    c.radius = 1.0f;
    c.invMass = 1.0f;
    scene.world.add<Collider>(only, c);

    scene.updateTransforms(&jobs);
    physics.step(scene, 1.0f / 60.0f, &jobs);
    culler.sortSpatially(scene);
    culler.maintain(scene);
    culler.build(scene, f, 4, list, &jobs);
    CHECK_EQ(list.visible, uint32_t{1});
    CHECK_EQ(list.drawCalls(), uint32_t{1});
    CHECK(scene.world.tryGet<Transform>(only) != nullptr);
}

TEST(a_morton_sorted_scene_round_trips_with_every_component_still_paired) {
    Scene original;
    std::mt19937 rng(2027);
    std::uniform_real_distribution<float> u(-60.0f, 60.0f);
    std::vector<Entity> created;
    for (int i = 0; i < 6000; ++i) {
        Entity parent = (i > 100 && (rng() % 6) == 0) ? created[rng() % created.size()] : NULL_ENTITY;
        Entity e = original.create(trs(Vec3{u(rng), u(rng), u(rng)}), parent);
        created.push_back(e);
        original.world.add<Renderable>(e, Renderable{static_cast<uint32_t>(i % 4), static_cast<uint32_t>(i % 3), 1, 0});
        CullBounds cb;
        cb.localExtent = Vec3{0.5f, 0.5f, 0.5f};
        original.world.add<CullBounds>(e, cb);
        if (i % 3 == 0) original.world.add<Velocity>(e, Velocity{Vec3{u(rng), 0, u(rng)}, Vec3{}});
    }
    for (int i = 0; i < 500; ++i) original.destroy(created[static_cast<size_t>(rng()) % created.size()]);
    original.updateTransforms(nullptr);

    // Sorting rewrites the dense arrays and the sparse maps under the
    // serializer's feet, which is exactly when a stale index would show up.
    CullSystem culler;
    culler.sortSpatially(original);

    std::vector<uint8_t> blob = serializeScene(original);
    Scene restored;
    std::string error;
    CHECK(deserializeScene(restored, blob.data(), blob.size(), error));
    CHECK_EQ(restored.world.aliveCount(), original.world.aliveCount());

    Pool<Renderable>& src = original.world.pool<Renderable>();
    for (size_t i = 0; i < src.dense.size(); ++i) {
        Entity e = src.dense[i];
        const Renderable* r = restored.world.tryGet<Renderable>(e);
        const CullBounds* cb = restored.world.tryGet<CullBounds>(e);
        CHECK(r != nullptr);
        CHECK(cb != nullptr);
        CHECK_EQ(r->mesh, src.data[i].mesh);
        CHECK_EQ(r->material, src.data[i].material);
        const Transform* a = original.world.tryGet<Transform>(e);
        const Transform* b = restored.world.tryGet<Transform>(e);
        CHECK(a && b);
        CHECK_EQ(a->position.x, b->position.x);
        CHECK_EQ(a->position.z, b->position.z);
    }
    restored.updateTransforms(nullptr);
    Mat4 vp = perspective(radians(60.0f), 1.6f, 0.5f, 200.0f) *
              lookAt(Vec3{0, 20, 90}, Vec3{0, 0, 0}, Vec3{0, 1, 0});
    Frustum f = extractFrustum(vp);
    CullSystem flat;
    flat.useClusters = false;
    RenderList before, after;
    flat.build(original, f, 3, before, nullptr);
    flat.build(restored, f, 3, after, nullptr);
    CHECK_EQ(after.visible, before.visible);
}

TEST(loader_rejects_corrupt_scene_data) {
    Scene scene;
    scene.create(trs(Vec3{1, 2, 3}));
    std::vector<uint8_t> blob = serializeScene(scene);

    std::string error;
    Scene target;
    CHECK(!deserializeScene(target, blob.data(), 4, error));
    CHECK(!error.empty());

    std::vector<uint8_t> badMagic = blob;
    badMagic[0] ^= 0xFF;
    CHECK(!deserializeScene(target, badMagic.data(), badMagic.size(), error));

    std::vector<uint8_t> truncated(blob.begin(), blob.begin() + static_cast<long>(blob.size() * 2 / 3));
    CHECK(!deserializeScene(target, truncated.data(), truncated.size(), error));
}

TEST(unknown_component_pools_are_skipped_not_fatal) {
    Scene scene;
    Entity e = scene.create(trs(Vec3{1, 1, 1}));
    scene.world.add<Renderable>(e, Renderable{2, 1, 1, 0});
    std::vector<uint8_t> blob = serializeScene(scene);

    std::string renamed = "Renderable";
    for (size_t i = 0; i + renamed.size() <= blob.size(); ++i) {
        if (std::memcmp(blob.data() + i, renamed.data(), renamed.size()) == 0) {
            blob[i] = 'X';
            break;
        }
    }

    Scene target;
    std::string error;
    CHECK(deserializeScene(target, blob.data(), blob.size(), error));
    CHECK_EQ(target.world.aliveCount(), size_t{1});
    CHECK_EQ(target.world.pool<Renderable>().size(), size_t{0});
    CHECK_EQ(target.world.pool<Transform>().size(), size_t{1});
}

TEST(cluster_order_is_only_resorted_once_motion_has_loosened_it) {
    Scene scene;
    std::mt19937 rng(77);
    std::uniform_real_distribution<float> p(-100.0f, 100.0f);
    std::vector<Entity> spawned;
    for (int i = 0; i < 20000; ++i) {
        Entity e = scene.create(trs(Vec3{p(rng), p(rng), p(rng)}));
        scene.world.add<Renderable>(e, Renderable{static_cast<uint32_t>(i % 3), 0, 1, 0});
        CullBounds cb;
        cb.localExtent = Vec3{0.5f, 0.5f, 0.5f};
        scene.world.add<CullBounds>(e, cb);
        spawned.push_back(e);
    }
    JobSystem jobs(4);
    scene.updateTransforms(&jobs);

    CullSystem culler;
    culler.sortSpatially(scene);
    CHECK_EQ(culler.stats().sorts, uint32_t{1});
    CHECK(culler.spreadBaseline() > 0.0f);

    culler.maintain(scene);
    CHECK_EQ(culler.stats().sorts, uint32_t{1});

    for (Entity e : spawned) scene.world.tryGet<Transform>(e)->position = Vec3{p(rng), p(rng), p(rng)};
    scene.updateTransforms(&jobs);
    culler.maintain(scene);
    CHECK_EQ(culler.stats().sorts, uint32_t{2});

    Mat4 vp = perspective(radians(60.0f), 16.0f / 9.0f, 0.5f, 220.0f) *
              lookAt(Vec3{0, 30, 110}, Vec3{0, 0, 0}, Vec3{0, 1, 0});
    Frustum f = extractFrustum(vp);
    CullSystem flat;
    flat.useClusters = false;
    RenderList flatList, clusteredList;
    flat.build(scene, f, 1, flatList, &jobs);
    culler.build(scene, f, 1, clusteredList, &jobs);
    CHECK_EQ(clusteredList.visible, flatList.visible);
}

TEST(spatial_clustering_matches_the_flat_cull_and_skips_most_objects) {
    Scene scene;
    std::mt19937 rng(404);
    std::uniform_real_distribution<float> p(-140.0f, 140.0f);
    std::vector<Entity> spawned;
    for (int i = 0; i < 30000; ++i) {
        Entity e = scene.create(trs(Vec3{p(rng), p(rng), p(rng)}));
        scene.world.add<Renderable>(e, Renderable{static_cast<uint32_t>(i % 4), static_cast<uint32_t>(i % 3), 1, 0});
        CullBounds cb;
        cb.localExtent = Vec3{0.6f, 0.6f, 0.6f};
        scene.world.add<CullBounds>(e, cb);
        spawned.push_back(e);
    }

    JobSystem jobs(4);
    scene.updateTransforms(&jobs);
    Mat4 vp = perspective(radians(55.0f), 16.0f / 9.0f, 0.5f, 260.0f) *
              lookAt(Vec3{0, 40, 130}, Vec3{0, 0, 0}, Vec3{0, 1, 0});
    Frustum f = extractFrustum(vp);

    CullSystem flat;
    flat.useClusters = false;
    RenderList flatList;
    flat.build(scene, f, 3, flatList, &jobs);

    CullSystem clustered;
    clustered.sortSpatially(scene);
    scene.updateTransforms(&jobs);
    RenderList clusteredList;
    clustered.build(scene, f, 3, clusteredList, &jobs);

    CHECK_EQ(clusteredList.visible, flatList.visible);
    CHECK_EQ(clusteredList.totalCandidates, flatList.totalCandidates);

    const CullStats& s = clustered.stats();
    CHECK(s.clustersInside > 0);
    CHECK(s.clustersOutside > 0);
    CHECK(s.objectsTested < flatList.totalCandidates / 2);

    // Sorting permutes the pools, so every handle must still resolve.
    for (Entity e : spawned) {
        CHECK(scene.world.tryGet<CullBounds>(e) != nullptr);
        CHECK(scene.world.tryGet<Renderable>(e) != nullptr);
    }
    Pool<CullBounds>& bounds = scene.world.pool<CullBounds>();
    for (size_t i = 0; i < bounds.dense.size(); ++i)
        CHECK(scene.world.tryGet<CullBounds>(bounds.dense[i]) == &bounds.data[i]);
}
