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
    /// Four is what a stack of turned boxes needs: a box face is held by four
    /// contact points at once and two passes leave the pair still sinking.
    /// Spheres converge at two.
    int solverIterations = 4;
    /// Upper bound on how many times a step may be split when a body would
    /// otherwise cross a collider between two tests. 1 disables splitting.
    int maxSubsteps = 4;
    /// Reuse each contact's accumulated impulse next frame. Off is the naive
    /// solver, kept so the benchmark can measure what warm starting buys.
    bool warmStart = true;
    /// Widen a moving body by the distance it covers this step and let contacts
    /// form across the gap, so an approach is stopped at the surface instead of
    /// being discovered after it has already gone through. Off is the naive
    /// discrete test, kept so the benchmark can measure the difference.
    bool speculativeContacts = true;
    /// Test rotated boxes as oriented boxes rather than as their axis-aligned
    /// extent. Bodies whose rotation is (near) identity take the cheap path
    /// either way, so this only costs what the scene actually turns.
    bool rotatedBoxes = true;
    /// Solve contacts at the point they touch rather than through the centres,
    /// so an off-centre hit spins the body. Off is the purely linear solver,
    /// kept so the benchmark can measure what the rotation costs.
    bool angularContacts = true;
    float angularDamping = 0.05f;
    /// Coulomb friction coefficient shared by every contact.
    /// ponytail: global, move onto Collider when materials need to differ.
    float friction = 0.4f;
    /// A body slower than `sleepSpeed` for `sleepTime` seconds stops being
    /// integrated and solved until something touches it.
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
    /// Pairs left after the cheap reach test, and how many of those were
    /// dropped because another cell owns the pair.
    uint64_t nearPairs = 0;
    uint64_t duplicatePairs = 0;
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
    /// Times the step was split to keep a fast body from crossing a collider.
    uint32_t substeps = 1;
};

/// Broadphase is a hashed uniform grid built with a counting sort, so the
/// per-step cost is linear and the neighbour scan reads contiguous memory.
/// Result of a ray query. `entity` is only meaningful when `hit` is set.
struct RayHit {
    bool hit = false;
    Entity entity = 0;
    float distance = 0.0f;
    Vec3 point{0, 0, 0};
    Vec3 normal{0, 0, 0};
};

class PhysicsWorld {
public:
    PhysicsSettings settings;

    PhysicsStats step(Scene& scene, float dt, JobSystem* jobs = nullptr);

    /// Nearest collider along `origin + dir * t` for t in [0, maxDistance],
    /// walked cell by cell through the grid the last step built. `dir` need not
    /// be normalised. Returns a miss when the world has not been stepped yet.
    RayHit raycast(const Vec3& origin, const Vec3& dir, float maxDistance) const;

    /// Every collider overlapping the sphere, appended to `out`. Same grid and
    /// same caveat as `raycast`: it answers for the last step's positions.
    void overlapSphere(const Vec3& center, float radius, std::vector<Entity>& out) const;

    const PhysicsStats& stats() const { return stats_; }
    size_t bytesUsed() const;

private:
    struct Contact {
        uint32_t a, b;
        Vec3 normal;
        float depth;
        /// Which point of a multi-point manifold this is, so two contacts
        /// between the same pair keep separate cached impulses.
        uint32_t id;
        /// Where the pair touches, in world space. Only read when the angular
        /// solver is on, but computing it is a few adds inside a test that has
        /// already found everything it needs.
        Vec3 point;
    };

    void gather(Scene& scene);
    void integrate(float dt, JobSystem* jobs);
    void buildGrid(JobSystem* jobs);
    void findContacts(JobSystem* jobs);
    void colorContacts();
    void solveRange(uint32_t begin, uint32_t end, bool positional, float invDt);
    void solveBounds(size_t begin, size_t end, float invDt);
    float boundsReach(size_t i, int axis) const;
    void warmStart(uint32_t begin, uint32_t end);
    void loadCachedImpulses(JobSystem* jobs);
    void storeCachedImpulses(JobSystem* jobs);
    void resolve(float dt, JobSystem* jobs);
    void updateSleep(float dt, JobSystem* jobs);
    void scatter(Scene& scene, JobSystem* jobs);

    std::vector<Entity> entity_;
    std::vector<Vec3> position_;
    std::vector<Vec3> velocity_;
    std::vector<Vec3> angular_;
    std::vector<Quat> orientation_;
    /// World-space box axes, three per body, and whether the body is a box
    /// turned far enough for them to differ from the world axes.
    std::vector<Vec3> axis_;
    std::vector<uint8_t> rotated_;
    /// Isotropic inverse inertia: one scalar rather than a tensor.
    /// ponytail: a long box therefore tumbles like a ball of the same size;
    /// store the diagonal and rotate it if elongated bodies ever matter.
    std::vector<float> invInertia_;
    std::vector<Vec3> pseudo_;
    std::vector<Vec3> pseudoSpin_;
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
        uint32_t awake;
    };
    std::vector<GridEntry> entries_;
    std::vector<std::vector<Contact>> contactChunks_;
    std::vector<Contact> contacts_;
    std::vector<float> normalImpulse_;
    std::vector<Vec3> tangentImpulse_;
    std::vector<float> deepest_;
    std::vector<float> restitutionBias_;
    std::vector<float> approach_;
    std::vector<uint64_t> contactKey_;
    /// Key and impulse share a cache line, since a probe reads both.
    struct CacheSlot {
        uint64_t key = 0;
        float impulse = 0.0f;
        /// Fastest approach seen while the pair was still apart, so a bounce
        /// is sized by the speed the body arrived at rather than by the speed
        /// left after the gap constraint has already slowed it down.
        float approach = 0.0f;
        Vec3 tangent{0, 0, 0};
    };
    std::vector<CacheSlot> cache_;
    uint32_t cacheMask_ = 0;
    std::vector<uint64_t> bodyColorMask_;
    std::vector<uint32_t> contactColor_;
    std::vector<uint32_t> colorStart_;
    std::vector<uint32_t> colorOrder_;
    uint32_t bucketMask_ = 0;
    uint32_t colorCount_ = 0;
    uint64_t frame_ = 0;
    float cell_ = 2.0f;
    float maxReach_ = 0.0f;
    std::vector<float> sweep_;
    float stepDt_ = 0.0f;
    float minThin_ = 0.0f;
    float maxSpeed2_ = 0.0f;
    float meanReach_ = 0.0f;

    PhysicsStats stats_;
};

}  // namespace skein
