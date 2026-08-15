#pragma once
#include "core/math.hpp"
#include "ecs/world.hpp"

namespace skein {

/// Local-space TRS. World matrices live in WorldTransform so the hot transform
/// pass reads one array and writes another.
struct Transform {
    Vec3 position{0, 0, 0};
    Quat rotation{};
    Vec3 scale{1, 1, 1};
};

struct WorldTransform {
    Mat4 matrix{};
};

struct Parent {
    Entity value = NULL_ENTITY;
};

struct Velocity {
    Vec3 linear{0, 0, 0};
    Vec3 angular{0, 0, 0};
};

enum class ColliderKind : uint32_t { Sphere = 0, Box = 1 };

struct Collider {
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};
    float radius = 0.5f;
    uint32_t kind = static_cast<uint32_t>(ColliderKind::Sphere);
    float invMass = 1.0f;
    float restitution = 0.5f;
    /// How long the body has been slow enough to be a sleep candidate, and
    /// whether it has crossed the threshold. The physics world owns both.
    float sleepTimer = 0.0f;
    uint32_t asleep = 0;
};

struct Renderable {
    uint32_t mesh = 0;
    uint32_t material = 0;
    uint32_t visible = 1;
    uint32_t pad = 0;
};

/// Local mesh bounds plus the world-space bounds the transform pass refreshes
/// each frame. Keeping both in one record means culling touches a single array.
struct CullBounds {
    Vec3 localCenter{0, 0, 0};
    Vec3 localExtent{0.5f, 0.5f, 0.5f};
    Vec3 center{0, 0, 0};
    Vec3 extent{0.5f, 0.5f, 0.5f};
    float radius = 1.0f;
    uint32_t pad = 0;
};

enum class LightKind : uint32_t { Directional = 0, Point = 1 };

struct Light {
    Vec3 color{1, 1, 1};
    float intensity = 1.0f;
    Vec3 direction{0, -1, 0};
    float range = 20.0f;
    uint32_t kind = static_cast<uint32_t>(LightKind::Point);
    uint32_t pad[3] = {0, 0, 0};
};

/// Reference into the Lua registry for a per-entity update function.
struct Script {
    int32_t ref = -1;
    int32_t pad = 0;
};

/// Registers every built-in component so scenes can be serialized by name.
void registerCoreComponents();

}  // namespace skein
