#include "physics/physics.hpp"
#include "script/script.hpp"
#include "test.hpp"

#include <lua.hpp>

#include "assets/mesh.hpp"
#include "physics/physics.hpp"
#include "scene/scene.hpp"

using namespace skein;

namespace {

struct Harness {
    Scene scene;
    Assets assets;
    PhysicsWorld physics;
    ScriptSystem script;

    Harness() {
        assets.addMesh(primitives::cube(1.0f));
        assets.addMesh(primitives::sphere(0.5f, 8, 6));
        Material m;
        m.name = "red";
        m.albedo = Vec3{1, 0, 0};
        assets.addMaterial(m);
        script.bind(&scene, &assets, &physics);
    }

    void run(const char* source) {
        std::string error;
        if (!script.doString(source, "=test", error)) failAt(__FILE__, __LINE__, error);
    }
};

}  // namespace

TEST(lua_spawn_creates_a_fully_formed_entity) {
    Harness h;
    h.run(R"(
        e = skein.spawn{
          position = {1, 2, 3},
          scale = {x = 2, y = 2, z = 2},
          mesh = "sphere",
          material = 0,
          velocity = {0, 5, 0},
          collider = {kind = "sphere", radius = 0.5, restitution = 0.25},
        }
    )");

    CHECK_EQ(h.scene.world.aliveCount(), size_t{1});
    Entity e = NULL_ENTITY;
    h.scene.world.each([&](Entity x) { e = x; });

    const Transform* t = h.scene.world.tryGet<Transform>(e);
    CHECK(t != nullptr);
    CHECK_NEAR(t->position.y, 2.0, 1e-5);
    CHECK_NEAR(t->scale.z, 2.0, 1e-5);

    const Renderable* r = h.scene.world.tryGet<Renderable>(e);
    CHECK(r != nullptr);
    CHECK_EQ(r->mesh, h.assets.findMesh("sphere"));

    const CullBounds* cb = h.scene.world.tryGet<CullBounds>(e);
    CHECK(cb != nullptr);
    CHECK_NEAR(cb->localExtent.x, 0.5, 1e-3);

    const Velocity* v = h.scene.world.tryGet<Velocity>(e);
    CHECK(v != nullptr);
    CHECK_NEAR(v->linear.y, 5.0, 1e-5);

    const Collider* c = h.scene.world.tryGet<Collider>(e);
    CHECK(c != nullptr);
    CHECK_NEAR(c->restitution, 0.25, 1e-5);
}

TEST(lua_reads_state_written_by_the_engine) {
    Harness h;
    h.run("e = skein.spawn{position = {0, 0, 0}, mesh = 'cube', collider = {radius = 0.5}}");

    Entity e = NULL_ENTITY;
    h.scene.world.each([&](Entity x) { e = x; });
    h.scene.world.get<Transform>(e).position = Vec3{7, 8, 9};

    h.run("x, y, z = skein.position(e)");
    lua_State* L = h.script.state();
    lua_getglobal(L, "y");
    CHECK_NEAR(lua_tonumber(L, -1), 8.0, 1e-5);
    lua_pop(L, 1);
}

TEST(per_entity_update_callbacks_run_every_frame) {
    Harness h;
    h.run(R"(
        ticks = 0
        e = skein.spawn{position = {0, 0, 0}}
        skein.on_update(e, function(self, dt)
          ticks = ticks + 1
          local x, y, z = skein.position(self)
          skein.set_position(self, x + dt, y, z)
        end)
    )");

    for (int i = 0; i < 10; ++i) h.script.update(0.1f);

    lua_State* L = h.script.state();
    lua_getglobal(L, "ticks");
    CHECK_EQ(static_cast<int>(lua_tointeger(L, -1)), 10);
    lua_pop(L, 1);

    Entity e = NULL_ENTITY;
    h.scene.world.each([&](Entity x) { e = x; });
    CHECK_NEAR(h.scene.world.get<Transform>(e).position.x, 1.0, 1e-4);
    CHECK(h.script.errors().empty());
}

TEST(on_frame_global_is_called_with_delta_time) {
    Harness h;
    h.run("total = 0\nfunction on_frame(dt) total = total + dt end");
    for (int i = 0; i < 4; ++i) h.script.update(0.25f);
    lua_State* L = h.script.state();
    lua_getglobal(L, "total");
    CHECK_NEAR(lua_tonumber(L, -1), 1.0, 1e-5);
    lua_pop(L, 1);
    CHECK_NEAR(h.script.elapsed(), 1.0, 1e-5);
}

TEST(a_failing_callback_is_reported_and_unbound) {
    Harness h;
    h.run(R"(
        e = skein.spawn{position = {0, 0, 0}}
        skein.on_update(e, function() error("boom") end)
    )");

    h.script.update(0.016f);
    CHECK_EQ(h.script.errors().size(), size_t{1});
    CHECK(h.script.errors()[0].find("boom") != std::string::npos);

    h.script.update(0.016f);
    CHECK_EQ(h.script.errors().size(), size_t{1});
}

TEST(scripts_can_destroy_entities_during_their_own_update) {
    Harness h;
    h.run(R"(
        for i = 1, 50 do
          local e = skein.spawn{position = {i, 0, 0}}
          skein.on_update(e, function(self, dt) skein.destroy(self) end)
        end
    )");
    CHECK_EQ(h.scene.world.aliveCount(), size_t{50});
    h.script.update(0.016f);
    CHECK_EQ(h.scene.world.aliveCount(), size_t{0});
    CHECK(h.script.errors().empty());
}

TEST(syntax_errors_are_returned_not_thrown) {
    Harness h;
    std::string error;
    CHECK(!h.script.doString("this is not lua", "=bad", error));
    CHECK(!error.empty());
}

TEST(lua_driven_entities_are_simulated_by_the_physics_system) {
    Harness h;
    h.run(R"(
        for i = 1, 100 do
          skein.spawn{
            position = {(i % 10) - 5, 10 + i * 0.5, (i // 10) - 5},
            mesh = "sphere",
            collider = {radius = 0.5},
          }
        end
    )");

    PhysicsWorld physics;
    physics.settings.boundsMin = Vec3{-20, 0, -20};
    physics.settings.boundsMax = Vec3{20, 80, 20};
    for (int i = 0; i < 120; ++i) physics.step(h.scene, 1.0f / 60.0f, nullptr);

    int landed = 0;
    forEach<Transform>(h.scene.world, [&](Entity, Transform& t) {
        CHECK(t.position.y >= -0.01f);
        if (t.position.y < 12.0f) ++landed;
    });
    CHECK(landed > 0);
}

TEST(unknown_mesh_names_raise_a_lua_error_without_leaking_entities) {
    Harness h;
    std::string error;
    CHECK(!h.script.doString("skein.spawn{mesh = 'not_a_mesh'}", "=bad", error));
    CHECK(error.find("not_a_mesh") != std::string::npos);
    CHECK_EQ(h.scene.world.aliveCount(), size_t{0});
}

TEST(lua_can_raycast_the_world_it_spawned) {
    Harness h;
    h.run(R"(
        target = skein.spawn{position = {0, 0, 12}, collider = {radius = 1.5}}
    )");
    h.physics.settings.gravity = Vec3{0, 0, 0};
    h.physics.settings.useBounds = false;
    h.physics.step(h.scene, 0.0f, nullptr);

    h.run(R"(
        hitEntity, distance, nx, ny, nz = skein.raycast(0, 0, 0, 0, 0, 1, 100)
        missed = skein.raycast(0, 0, 0, 0, 1, 0, 100)
    )");

    lua_State* L = h.script.state();
    lua_getglobal(L, "target");
    Entity spawned = static_cast<Entity>(lua_tointeger(L, -1));
    lua_pop(L, 1);
    lua_getglobal(L, "hitEntity");
    CHECK_EQ(static_cast<uint64_t>(lua_tointeger(L, -1)), static_cast<uint64_t>(spawned));
    lua_pop(L, 1);

    lua_getglobal(L, "distance");
    CHECK_NEAR(lua_tonumber(L, -1), 10.5, 1e-3);
    lua_pop(L, 1);
    lua_getglobal(L, "nz");
    CHECK_NEAR(lua_tonumber(L, -1), -1.0, 1e-4);
    lua_pop(L, 1);
    lua_getglobal(L, "missed");
    CHECK(lua_isnil(L, -1));
    lua_pop(L, 1);
}
