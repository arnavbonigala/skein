#pragma once
#include <string>
#include <vector>

struct lua_State;

namespace skein {

class Scene;
class Assets;
class PhysicsWorld;

/// Lua 5.x host. Scripts see a `skein` table that manipulates entities through
/// the same component pools the C++ systems use, so a scripted entity is not a
/// special case anywhere else in the engine.
class ScriptSystem {
public:
    ScriptSystem();
    ~ScriptSystem();

    ScriptSystem(const ScriptSystem&) = delete;
    ScriptSystem& operator=(const ScriptSystem&) = delete;

    /// `physics` is optional; without it `skein.raycast` reports no hit.
    void bind(Scene* scene, Assets* assets, PhysicsWorld* physics = nullptr);

    bool doString(const std::string& source, const std::string& chunkName, std::string& error);
    bool doFile(const std::string& path, std::string& error);

    /// Calls the global `on_frame(dt)` when present, then every per-entity
    /// callback registered through `skein.on_update`.
    void update(float dt);

    /// Scripts that raised an error are unbound; their messages land here.
    const std::vector<std::string>& errors() const { return errors_; }
    void clearErrors() { errors_.clear(); }

    size_t scriptedEntities() const;
    size_t memoryBytes() const;

    lua_State* state() { return L_; }
    Scene* scene() { return scene_; }
    Assets* assets() { return assets_; }
    PhysicsWorld* physics() { return physics_; }
    float elapsed() const { return elapsed_; }

private:
    void openApi();

    lua_State* L_ = nullptr;
    Scene* scene_ = nullptr;
    Assets* assets_ = nullptr;
    PhysicsWorld* physics_ = nullptr;
    std::vector<std::string> errors_;
    float elapsed_ = 0.0f;
};

}  // namespace skein
