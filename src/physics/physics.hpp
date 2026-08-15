#pragma once
#include <cstdint>
#include <vector>

#include "core/math.hpp"
#include "ecs/world.hpp"

namespace skein {

class Scene;
class JobSystem;

struct PhysicsSettings {
    Vec3 gravity{0.0f, -9.81f, 0.0f};
    float linearDamping = 0.02f;
    float cellSize = 2.0f;
    float restitutionFloor = 0.35f;
    /// Optional axis-aligned box the simulation keeps bodies inside.
    bool useBounds = true;
    Vec3 boundsMin{-60, 0, -60};
    Vec3 boundsMax{60, 120, 60};
    int solverIterations = 2;
};

struct PhysicsStats {
    uint32_t bodies = 0;
    uint64_t pairsTested = 0;
    uint32_t contacts = 0;
    uint32_t gridCells = 0;
};

/// Broadphase is a hashed uniform grid built with a counting sort, so the
/// per-step cost is linear and the neighbour scan reads contiguous memory.
class PhysicsWorld {
public:
    PhysicsSettings settings;

    PhysicsStats step(Scene& scene, float dt, JobSystem* jobs = nullptr);

    const PhysicsStats& stats() const { return stats_; }
    size_t bytesUsed() const;

private:
    struct Contact {
        uint32_t a, b;
        Vec3 normal;
        float depth;
    };

    void gather(Scene& scene);
    void integrate(float dt, JobSystem* jobs);
    void buildGrid(JobSystem* jobs);
    void findContacts(JobSystem* jobs);
    void resolve();
    void scatter(Scene& scene);

    std::vector<Entity> entity_;
    std::vector<Vec3> position_;
    std::vector<Vec3> velocity_;
    std::vector<Vec3> halfExtent_;
    std::vector<float> radius_;
    std::vector<float> invMass_;
    std::vector<float> restitution_;
    std::vector<uint32_t> kind_;

    std::vector<int32_t> cellCoord_;
    std::vector<uint32_t> bucketOf_;
    std::vector<uint32_t> cellStart_;
    std::vector<uint32_t> sorted_;
    std::vector<std::vector<Contact>> contactChunks_;
    std::vector<Contact> contacts_;
    uint32_t bucketMask_ = 0;

    PhysicsStats stats_;
};

}  // namespace skein
