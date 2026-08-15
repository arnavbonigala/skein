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
    /// Lower bound on the broadphase cell. The grid widens itself to fit the
    /// largest collider, so a value below that is raised rather than obeyed.
    float cellSize = 2.0f;
    float restitutionFloor = 0.35f;
    /// Optional axis-aligned box the simulation keeps bodies inside.
    bool useBounds = true;
    Vec3 boundsMin{-60, 0, -60};
    Vec3 boundsMax{60, 120, 60};
    int solverIterations = 2;
    /// A body slower than `sleepSpeed` for `sleepTime` seconds stops being
    /// integrated and solved until something touches it.
    /// Coulomb friction coefficient shared by every contact.
    /// ponytail: global, move onto Collider when materials need to differ.
    float friction = 0.4f;
    bool allowSleep = true;
    /// Must sit above gravity * dt, or a body resting on another never falls
    /// below it: gravity re-accelerates it every step and the contact impulse
    /// cancels it right back.
    float sleepSpeed = 0.3f;
    float sleepTime = 0.6f;
};

struct PhysicsStats {
    uint32_t bodies = 0;
    uint64_t pairsTested = 0;
    uint32_t contacts = 0;
    uint32_t gridCells = 0;
    /// Independent sets the contact graph was coloured into, plus the tail of
    /// contacts that ran out of colours and are solved serially.
    uint32_t colors = 0;
    uint32_t serialContacts = 0;
    /// Cell registrations, which exceeds the body count when colliders span
    /// more than one cell.
    uint32_t gridEntries = 0;
    float cellSize = 0;
    /// Bodies still being integrated and solved this step.
    uint32_t awake = 0;
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
    void colorContacts();
    void solveRange(uint32_t begin, uint32_t end, bool positional, float invDt);
    void solveBounds(size_t begin, size_t end, float invDt);
    void warmStart(uint32_t begin, uint32_t end);
    void loadCachedImpulses(JobSystem* jobs);
    void storeCachedImpulses();
    void resolve(float dt, JobSystem* jobs);
    void updateSleep(float dt, JobSystem* jobs);
    void scatter(Scene& scene, JobSystem* jobs);

    std::vector<Entity> entity_;
    std::vector<Vec3> position_;
    std::vector<Vec3> velocity_;
    std::vector<Vec3> pseudo_;
    std::vector<Vec3> halfExtent_;
    std::vector<float> radius_;
    std::vector<float> invMass_;
    std::vector<float> restitution_;
    std::vector<uint32_t> kind_;
    std::vector<float> sleepTimer_;
    std::vector<uint8_t> asleep_;

    std::vector<float> reach_;
    std::vector<int32_t> bodyLo_;
    std::vector<int32_t> bodyHi_;
    std::vector<uint32_t> entryOffset_;
    std::vector<uint32_t> entryBucket_;
    std::vector<uint32_t> cellStart_;
    struct GridEntry {
        float x, y, z, reach;
        uint64_t cell;
        uint32_t body;
        uint32_t pad;
    };
    std::vector<GridEntry> entries_;
    std::vector<std::vector<Contact>> contactChunks_;
    std::vector<Contact> contacts_;
    std::vector<float> normalImpulse_;
    std::vector<float> deepest_;
    std::vector<float> restitutionBias_;
    std::vector<uint64_t> contactKey_;
    std::vector<uint64_t> cacheKey_;
    std::vector<float> cacheImpulse_;
    uint32_t cacheMask_ = 0;
    std::vector<uint64_t> bodyColorMask_;
    std::vector<uint32_t> contactColor_;
    std::vector<uint32_t> colorStart_;
    std::vector<uint32_t> colorOrder_;
    uint32_t bucketMask_ = 0;
    uint32_t colorCount_ = 0;
    float cell_ = 2.0f;
    float maxReach_ = 0.0f;
    float meanReach_ = 0.0f;

    PhysicsStats stats_;
};

}  // namespace skein
