#include <lua.hpp>

#include <vector>
#include <cstdio>

#include "assets/mesh.hpp"
#include "core/profiler.hpp"
#include "physics/physics.hpp"
#include "scene/scene.hpp"
#include "script/script.hpp"

namespace skein {
namespace {

ScriptSystem& host(lua_State* L) {
    return *static_cast<ScriptSystem*>(lua_touserdata(L, lua_upvalueindex(1)));
}

Entity checkEntity(lua_State* L, int idx) {
    return static_cast<Entity>(luaL_checkinteger(L, idx));
}

float optNumberField(lua_State* L, int table, const char* key, float fallback) {
    lua_getfield(L, table, key);
    float v = lua_isnumber(L, -1) ? static_cast<float>(lua_tonumber(L, -1)) : fallback;
    lua_pop(L, 1);
    return v;
}

/// Reads {x, y, z} or {1, 2, 3} from a field of the table at `table`.
Vec3 optVec3Field(lua_State* L, int table, const char* key, Vec3 fallback) {
    lua_getfield(L, table, key);
    if (!lua_istable(L, -1)) {
        lua_pop(L, 1);
        return fallback;
    }
    int t = lua_gettop(L);
    Vec3 v = fallback;
    const char* named[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
        lua_getfield(L, t, named[i]);
        if (lua_isnumber(L, -1)) {
            v[i] = static_cast<float>(lua_tonumber(L, -1));
        } else {
            lua_pop(L, 1);
            lua_rawgeti(L, t, i + 1);
            if (lua_isnumber(L, -1)) v[i] = static_cast<float>(lua_tonumber(L, -1));
        }
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    return v;
}

void pushVec3(lua_State* L, const Vec3& v) {
    lua_pushnumber(L, v.x);
    lua_pushnumber(L, v.y);
    lua_pushnumber(L, v.z);
}

int l_spawn(lua_State* L) {
    ScriptSystem& h = host(L);
    Scene* scene = h.scene();
    if (!scene) return luaL_error(L, "no scene bound");
    bool hasTable = lua_istable(L, 1);

    Transform t;
    if (hasTable) {
        t.position = optVec3Field(L, 1, "position", Vec3{0, 0, 0});
        t.scale = optVec3Field(L, 1, "scale", Vec3{1, 1, 1});
        float yaw = optNumberField(L, 1, "yaw", 0.0f);
        float pitch = optNumberField(L, 1, "pitch", 0.0f);
        if (yaw != 0.0f || pitch != 0.0f) t.rotation = Quat::euler(pitch, yaw, 0.0f);
    }

    Entity e = scene->create(t);

    if (hasTable) {
        lua_getfield(L, 1, "mesh");
        bool hasMesh = !lua_isnil(L, -1);
        uint32_t mesh = 0;
        if (lua_isnumber(L, -1)) {
            mesh = static_cast<uint32_t>(lua_tointeger(L, -1));
        } else if (lua_isstring(L, -1) && h.assets()) {
            std::string wanted = lua_tostring(L, -1);
            uint32_t found = h.assets()->findMesh(wanted);
            if (found == ~0u) {
                lua_pop(L, 1);
                scene->destroy(e);
                return luaL_error(L, "unknown mesh '%s'", wanted.c_str());
            }
            mesh = found;
        }
        lua_pop(L, 1);

        uint32_t material = static_cast<uint32_t>(optNumberField(L, 1, "material", 0.0f));
        if (hasMesh) {
            scene->world.add<Renderable>(e, Renderable{mesh, material, 1, 0});
            CullBounds cb;
            if (h.assets() && mesh < h.assets()->meshCount()) {
                const AABB& b = h.assets()->mesh(mesh).bounds;
                cb.localCenter = b.center();
                cb.localExtent = b.extent();
            }
            cb.center = cb.localCenter;
            cb.extent = cb.localExtent;
            cb.radius = length(cb.extent);
            scene->world.add<CullBounds>(e, cb);
        }

        lua_getfield(L, 1, "velocity");
        bool hasVel = lua_istable(L, -1);
        lua_pop(L, 1);
        if (hasVel) scene->world.add<Velocity>(e, Velocity{optVec3Field(L, 1, "velocity", Vec3{0, 0, 0}), Vec3{0, 0, 0}});

        lua_getfield(L, 1, "collider");
        if (lua_istable(L, -1)) {
            int ct = lua_gettop(L);
            Collider c;
            lua_getfield(L, ct, "kind");
            if (lua_isstring(L, -1))
                c.kind = std::string(lua_tostring(L, -1)) == "box" ? static_cast<uint32_t>(ColliderKind::Box)
                                                                   : static_cast<uint32_t>(ColliderKind::Sphere);
            lua_pop(L, 1);
            c.radius = optNumberField(L, ct, "radius", c.radius);
            c.invMass = optNumberField(L, ct, "inv_mass", c.invMass);
            c.restitution = optNumberField(L, ct, "restitution", c.restitution);
            c.halfExtents = optVec3Field(L, ct, "half_extents", c.halfExtents);
            scene->world.add<Collider>(e, c);
            if (!scene->world.has<Velocity>(e)) scene->world.add<Velocity>(e, Velocity{});
        }
        lua_pop(L, 1);
    }

    lua_pushinteger(L, static_cast<lua_Integer>(e));
    return 1;
}

int l_destroy(lua_State* L) {
    Scene* scene = host(L).scene();
    if (scene) scene->destroy(checkEntity(L, 1));
    return 0;
}

int l_alive(lua_State* L) {
    Scene* scene = host(L).scene();
    lua_pushboolean(L, scene && scene->world.alive(checkEntity(L, 1)));
    return 1;
}

int l_position(lua_State* L) {
    Scene* scene = host(L).scene();
    const Transform* t = scene ? scene->world.tryGet<Transform>(checkEntity(L, 1)) : nullptr;
    if (!t) return 0;
    pushVec3(L, t->position);
    return 3;
}

int l_setPosition(lua_State* L) {
    Scene* scene = host(L).scene();
    Transform* t = scene ? scene->world.tryGet<Transform>(checkEntity(L, 1)) : nullptr;
    if (!t) return 0;
    t->position = Vec3{static_cast<float>(luaL_checknumber(L, 2)), static_cast<float>(luaL_checknumber(L, 3)),
                       static_cast<float>(luaL_checknumber(L, 4))};
    return 0;
}

int l_velocity(lua_State* L) {
    Scene* scene = host(L).scene();
    const Velocity* v = scene ? scene->world.tryGet<Velocity>(checkEntity(L, 1)) : nullptr;
    if (!v) return 0;
    pushVec3(L, v->linear);
    return 3;
}

int l_setVelocity(lua_State* L) {
    Scene* scene = host(L).scene();
    if (!scene) return 0;
    Entity e = checkEntity(L, 1);
    Vec3 value{static_cast<float>(luaL_checknumber(L, 2)), static_cast<float>(luaL_checknumber(L, 3)),
               static_cast<float>(luaL_checknumber(L, 4))};
    if (Velocity* v = scene->world.tryGet<Velocity>(e))
        v->linear = value;
    else if (scene->world.alive(e))
        scene->world.add<Velocity>(e, Velocity{value, Vec3{0, 0, 0}});
    return 0;
}

int l_setScale(lua_State* L) {
    Scene* scene = host(L).scene();
    Transform* t = scene ? scene->world.tryGet<Transform>(checkEntity(L, 1)) : nullptr;
    if (!t) return 0;
    float s = static_cast<float>(luaL_checknumber(L, 2));
    t->scale = Vec3{s, static_cast<float>(luaL_optnumber(L, 3, s)), static_cast<float>(luaL_optnumber(L, 4, s))};
    return 0;
}

int l_setRotation(lua_State* L) {
    Scene* scene = host(L).scene();
    Transform* t = scene ? scene->world.tryGet<Transform>(checkEntity(L, 1)) : nullptr;
    if (!t) return 0;
    t->rotation = Quat::euler(static_cast<float>(luaL_checknumber(L, 2)),
                              static_cast<float>(luaL_checknumber(L, 3)),
                              static_cast<float>(luaL_optnumber(L, 4, 0.0)));
    return 0;
}

int l_setParent(lua_State* L) {
    Scene* scene = host(L).scene();
    if (!scene) return 0;
    Entity child = checkEntity(L, 1);
    Entity parent = lua_isnoneornil(L, 2) ? NULL_ENTITY : checkEntity(L, 2);
    scene->setParent(child, parent);
    return 0;
}

int l_setMaterial(lua_State* L) {
    Scene* scene = host(L).scene();
    Renderable* r = scene ? scene->world.tryGet<Renderable>(checkEntity(L, 1)) : nullptr;
    if (!r) return 0;
    r->material = static_cast<uint32_t>(luaL_checkinteger(L, 2));
    return 0;
}

int l_setVisible(lua_State* L) {
    Scene* scene = host(L).scene();
    Renderable* r = scene ? scene->world.tryGet<Renderable>(checkEntity(L, 1)) : nullptr;
    if (!r) return 0;
    r->visible = lua_toboolean(L, 2) ? 1u : 0u;
    return 0;
}

int l_entityCount(lua_State* L) {
    Scene* scene = host(L).scene();
    lua_pushinteger(L, scene ? static_cast<lua_Integer>(scene->world.aliveCount()) : 0);
    return 1;
}

int l_meshId(lua_State* L) {
    Assets* assets = host(L).assets();
    if (!assets) return 0;
    uint32_t id = assets->findMesh(luaL_checkstring(L, 1));
    if (id == ~0u) return 0;
    lua_pushinteger(L, id);
    return 1;
}

int l_materialId(lua_State* L) {
    Assets* assets = host(L).assets();
    if (!assets) return 0;
    uint32_t id = assets->findMaterial(luaL_checkstring(L, 1));
    if (id == ~0u) return 0;
    lua_pushinteger(L, id);
    return 1;
}

int l_addMaterial(lua_State* L) {
    Assets* assets = host(L).assets();
    if (!assets) return 0;
    luaL_checktype(L, 1, LUA_TTABLE);
    Material m;
    lua_getfield(L, 1, "name");
    if (lua_isstring(L, -1)) m.name = lua_tostring(L, -1);
    lua_pop(L, 1);
    m.albedo = optVec3Field(L, 1, "albedo", m.albedo);
    m.emissive = optVec3Field(L, 1, "emissive", m.emissive);
    m.roughness = optNumberField(L, 1, "roughness", m.roughness);
    m.metallic = optNumberField(L, 1, "metallic", m.metallic);
    lua_pushinteger(L, assets->addMaterial(std::move(m)));
    return 1;
}

int l_onUpdate(lua_State* L) {
    ScriptSystem& h = host(L);
    Scene* scene = h.scene();
    if (!scene) return 0;
    Entity e = checkEntity(L, 1);
    if (!scene->world.alive(e)) return 0;
    luaL_checktype(L, 2, LUA_TFUNCTION);

    if (Script* existing = scene->world.tryGet<Script>(e))
        if (existing->ref >= 0) luaL_unref(L, LUA_REGISTRYINDEX, existing->ref);

    lua_pushvalue(L, 2);
    int ref = luaL_ref(L, LUA_REGISTRYINDEX);
    scene->world.add<Script>(e, Script{ref, 0});
    return 0;
}

int l_time(lua_State* L) {
    lua_pushnumber(L, host(L).elapsed());
    return 1;
}

int l_log(lua_State* L) {
    int n = lua_gettop(L);
    std::string line;
    for (int i = 1; i <= n; ++i) {
        if (i > 1) line += '\t';
        const char* s = luaL_tolstring(L, i, nullptr);
        line += s ? s : "";
        lua_pop(L, 1);
    }
    std::printf("[lua] %s\n", line.c_str());
    return 0;
}

/// `skein.raycast(ox, oy, oz, dx, dy, dz [, maxDistance])` -> entity, distance,
/// nx, ny, nz, or nothing when the ray hits nothing. The query runs against the
/// grid the last physics step built.
int l_raycast(lua_State* L) {
    PhysicsWorld* physics = host(L).physics();
    if (!physics) return 0;
    Vec3 origin{static_cast<float>(luaL_checknumber(L, 1)), static_cast<float>(luaL_checknumber(L, 2)),
                static_cast<float>(luaL_checknumber(L, 3))};
    Vec3 dir{static_cast<float>(luaL_checknumber(L, 4)), static_cast<float>(luaL_checknumber(L, 5)),
             static_cast<float>(luaL_checknumber(L, 6))};
    float maxDistance = static_cast<float>(luaL_optnumber(L, 7, 1e6));
    RayHit hit = physics->raycast(origin, dir, maxDistance);
    if (!hit.hit) return 0;
    lua_pushinteger(L, static_cast<lua_Integer>(hit.entity));
    lua_pushnumber(L, hit.distance);
    pushVec3(L, hit.normal);
    return 5;
}

/// `skein.overlap_sphere(x, y, z, radius)` -> array of entities.
int l_overlapSphere(lua_State* L) {
    PhysicsWorld* physics = host(L).physics();
    lua_newtable(L);
    if (!physics) return 1;
    Vec3 center{static_cast<float>(luaL_checknumber(L, 1)), static_cast<float>(luaL_checknumber(L, 2)),
                static_cast<float>(luaL_checknumber(L, 3))};
    std::vector<Entity> found;
    physics->overlapSphere(center, static_cast<float>(luaL_checknumber(L, 4)), found);
    for (size_t i = 0; i < found.size(); ++i) {
        lua_pushinteger(L, static_cast<lua_Integer>(found[i]));
        lua_rawseti(L, -2, static_cast<lua_Integer>(i + 1));
    }
    return 1;
}

const luaL_Reg API[] = {
    {"spawn", l_spawn},
    {"destroy", l_destroy},
    {"alive", l_alive},
    {"position", l_position},
    {"set_position", l_setPosition},
    {"velocity", l_velocity},
    {"set_velocity", l_setVelocity},
    {"set_scale", l_setScale},
    {"set_rotation", l_setRotation},
    {"set_parent", l_setParent},
    {"set_material", l_setMaterial},
    {"set_visible", l_setVisible},
    {"entity_count", l_entityCount},
    {"mesh", l_meshId},
    {"material", l_materialId},
    {"add_material", l_addMaterial},
    {"on_update", l_onUpdate},
    {"raycast", l_raycast},
    {"overlap_sphere", l_overlapSphere},
    {"time", l_time},
    {"log", l_log},
    {nullptr, nullptr},
};

int traceback(lua_State* L) {
    const char* msg = lua_tostring(L, 1);
    luaL_traceback(L, L, msg ? msg : "(non-string error)", 1);
    return 1;
}

}  // namespace

ScriptSystem::ScriptSystem() {
    L_ = luaL_newstate();
    luaL_openlibs(L_);
    openApi();
}

ScriptSystem::~ScriptSystem() {
    if (L_) lua_close(L_);
}

void ScriptSystem::openApi() {
    lua_newtable(L_);
    for (const luaL_Reg* r = API; r->name; ++r) {
        lua_pushlightuserdata(L_, this);
        lua_pushcclosure(L_, r->func, 1);
        lua_setfield(L_, -2, r->name);
    }
    lua_setglobal(L_, "skein");
}

void ScriptSystem::bind(Scene* scene, Assets* assets, PhysicsWorld* physics) {
    scene_ = scene;
    assets_ = assets;
    physics_ = physics;
}

bool ScriptSystem::doString(const std::string& source, const std::string& chunkName, std::string& error) {
    lua_pushcfunction(L_, traceback);
    int handler = lua_gettop(L_);
    if (luaL_loadbuffer(L_, source.data(), source.size(), chunkName.c_str()) != LUA_OK) {
        error = lua_tostring(L_, -1);
        lua_pop(L_, 2);
        return false;
    }
    if (lua_pcall(L_, 0, 0, handler) != LUA_OK) {
        error = lua_tostring(L_, -1);
        lua_pop(L_, 2);
        return false;
    }
    lua_pop(L_, 1);
    return true;
}

bool ScriptSystem::doFile(const std::string& path, std::string& error) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        error = "cannot open " + path;
        return false;
    }
    std::string source;
    char buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) source.append(buf, n);
    std::fclose(f);
    return doString(source, "@" + path, error);
}

void ScriptSystem::update(float dt) {
    SKEIN_PROFILE("script/update");
    elapsed_ += dt;
    if (!scene_) return;

    lua_pushcfunction(L_, traceback);
    int handler = lua_gettop(L_);

    lua_getglobal(L_, "on_frame");
    if (lua_isfunction(L_, -1)) {
        lua_pushnumber(L_, dt);
        if (lua_pcall(L_, 1, 0, handler) != LUA_OK) {
            errors_.emplace_back(lua_tostring(L_, -1));
            lua_pop(L_, 1);
        }
    } else {
        lua_pop(L_, 1);
    }

    Pool<Script>& scripts = scene_->world.pool<Script>();
    std::vector<Entity> targets(scripts.dense.begin(), scripts.dense.end());
    for (Entity e : targets) {
        Script* s = scripts.tryGet(e);
        if (!s || s->ref < 0) continue;
        int ref = s->ref;
        lua_rawgeti(L_, LUA_REGISTRYINDEX, ref);
        if (!lua_isfunction(L_, -1)) {
            lua_pop(L_, 1);
            continue;
        }
        lua_pushinteger(L_, static_cast<lua_Integer>(e));
        lua_pushnumber(L_, dt);
        if (lua_pcall(L_, 2, 0, handler) != LUA_OK) {
            errors_.emplace_back(lua_tostring(L_, -1));
            lua_pop(L_, 1);
            if (Script* still = scripts.tryGet(e)) {
                luaL_unref(L_, LUA_REGISTRYINDEX, still->ref);
                still->ref = -1;
            }
        }
    }

    lua_pop(L_, 1);
}

size_t ScriptSystem::scriptedEntities() const {
    if (!scene_) return 0;
    return scene_->world.pool<Script>().size();
}

size_t ScriptSystem::memoryBytes() const {
    if (!L_) return 0;
    return static_cast<size_t>(lua_gc(L_, LUA_GCCOUNT)) * 1024 +
           static_cast<size_t>(lua_gc(L_, LUA_GCCOUNTB));
}

}  // namespace skein
