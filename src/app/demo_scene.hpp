#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "assets/mesh.hpp"
#include "physics/physics.hpp"
#include "scene/scene.hpp"
#include "script/script.hpp"

namespace skein {

class JobSystem;

struct DemoConfig {
    int entityCount = 100000;
    int renderableCount = 25000;
    int colliderCount = 30000;
    int hierarchyChildren = 12000;
    int pointLights = 24;
    float fieldExtent = 200.0f;
    float fieldHeight = 90.0f;
    uint32_t seed = 1337;
    bool runScripts = true;
    /// Hanging chains of jointed spheres, for a scene where the solver has a
    /// constraint that is not a contact. Zero in the benchmark, so the tables
    /// measure one scene.
    int ropes = 0;
    int ropeLinks = 10;
    /// OBJ files loaded concurrently at build time; empty means the demo's own.
    std::vector<std::string> objPaths;
};

/// The world the demo and the benchmark both drive: one large entity field,
/// a slice of it renderable, a slice simulated, and a scripted layer on top.
class Demo {
public:
    Scene scene;
    Assets assets;
    PhysicsWorld physics;
    ScriptSystem script;
    DemoConfig config;

    void build(const DemoConfig& cfg, JobSystem* jobs);
    bool loadScript(const std::string& path, std::string& error);

    /// Runs scripts, kinematics, physics and the transform hierarchy for one frame.
    void update(float dt, JobSystem* jobs);

    /// Advances every entity that carries a Velocity but no Collider. This is
    /// the wide, purely data-parallel pass the ECS benchmark measures.
    void stepKinematics(float dt, JobSystem* jobs);

    size_t bytesUsed() const;
    uint32_t meshCount() const { return static_cast<uint32_t>(assets.meshCount()); }
    const std::vector<uint32_t>& meshIds() const { return meshIds_; }

private:
    std::vector<uint32_t> meshIds_;
    Vec3 fieldMin_{-200, 0, -200};
    Vec3 fieldMax_{200, 90, 200};
};

}  // namespace skein
