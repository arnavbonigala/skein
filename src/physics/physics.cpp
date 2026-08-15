#include "physics/physics.hpp"

#include <algorithm>
#include <atomic>

#include "core/jobs.hpp"
#include "core/profiler.hpp"
#include "scene/scene.hpp"

namespace skein {
namespace {

constexpr size_t CONTACT_GRAIN = 512;

uint32_t hashCell(int32_t x, int32_t y, int32_t z) {
    uint32_t h = static_cast<uint32_t>(x) * 73856093u;
    h ^= static_cast<uint32_t>(y) * 19349663u;
    h ^= static_cast<uint32_t>(z) * 83492791u;
    h ^= h >> 15;
    return h;
}

uint32_t nextPow2(uint32_t v) {
    uint32_t p = 1;
    while (p < v) p <<= 1;
    return p;
}

}  // namespace

void PhysicsWorld::gather(Scene& scene) {
    SKEIN_PROFILE("physics/gather");
    Pool<Collider>& colliders = scene.world.pool<Collider>();
    Pool<Transform>& transforms = scene.world.pool<Transform>();
    Pool<Velocity>& velocities = scene.world.pool<Velocity>();

    size_t n = colliders.dense.size();
    entity_.clear();
    position_.clear();
    velocity_.clear();
    halfExtent_.clear();
    radius_.clear();
    invMass_.clear();
    restitution_.clear();
    kind_.clear();
    entity_.reserve(n);
    position_.reserve(n);
    velocity_.reserve(n);
    halfExtent_.reserve(n);
    radius_.reserve(n);
    invMass_.reserve(n);
    restitution_.reserve(n);
    kind_.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        Entity e = colliders.dense[i];
        const Transform* t = transforms.tryGet(e);
        if (!t) continue;
        const Collider& c = colliders.data[i];
        const Velocity* v = velocities.tryGet(e);
        entity_.push_back(e);
        position_.push_back(t->position);
        velocity_.push_back(v ? v->linear : Vec3{0, 0, 0});
        float s = maxComponent(vabs(t->scale));
        halfExtent_.push_back(c.halfExtents * s);
        radius_.push_back(c.radius * s);
        invMass_.push_back(c.invMass);
        restitution_.push_back(c.restitution);
        kind_.push_back(c.kind);
    }
}

void PhysicsWorld::integrate(float dt, JobSystem* jobs) {
    SKEIN_PROFILE("physics/integrate");
    size_t n = position_.size();
    const Vec3 g = settings.gravity;
    const float damp = std::max(0.0f, 1.0f - settings.linearDamping * dt);
    auto body = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            if (invMass_[i] > 0.0f) velocity_[i] += g * dt;
            velocity_[i] *= damp;
            position_[i] += velocity_[i] * dt;
        }
    };
    if (jobs && n >= 4096)
        jobs->parallelFor(n, 4096, body);
    else
        body(0, n);
}

void PhysicsWorld::buildGrid(JobSystem* jobs) {
    SKEIN_PROFILE("physics/broadphase");
    size_t n = position_.size();
    uint32_t buckets = std::max(nextPow2(static_cast<uint32_t>(n * 2 + 1)), 64u);
    bucketMask_ = buckets - 1;

    cellCoord_.resize(n * 3);
    bucketOf_.resize(n);
    sorted_.resize(n);
    cellStart_.assign(buckets + 1, 0);

    const float inv = 1.0f / settings.cellSize;
    auto classify = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            int32_t cx = static_cast<int32_t>(std::floor(position_[i].x * inv));
            int32_t cy = static_cast<int32_t>(std::floor(position_[i].y * inv));
            int32_t cz = static_cast<int32_t>(std::floor(position_[i].z * inv));
            cellCoord_[i * 3 + 0] = cx;
            cellCoord_[i * 3 + 1] = cy;
            cellCoord_[i * 3 + 2] = cz;
            bucketOf_[i] = hashCell(cx, cy, cz) & bucketMask_;
        }
    };
    if (jobs && n >= 8192)
        jobs->parallelFor(n, 8192, classify);
    else
        classify(0, n);

    for (size_t i = 0; i < n; ++i) ++cellStart_[bucketOf_[i] + 1];
    uint32_t used = 0;
    for (size_t i = 1; i < cellStart_.size(); ++i) {
        if (cellStart_[i] != 0) ++used;
        cellStart_[i] += cellStart_[i - 1];
    }
    std::vector<uint32_t> cursor(cellStart_.begin(), cellStart_.end() - 1);
    for (size_t i = 0; i < n; ++i) sorted_[cursor[bucketOf_[i]]++] = static_cast<uint32_t>(i);
    stats_.gridCells = used;
}

void PhysicsWorld::findContacts(JobSystem* jobs) {
    SKEIN_PROFILE("physics/narrowphase");
    size_t n = position_.size();
    size_t chunks = n == 0 ? 0 : (n + CONTACT_GRAIN - 1) / CONTACT_GRAIN;
    contactChunks_.resize(chunks);
    for (auto& c : contactChunks_) c.clear();
    std::atomic<uint64_t> pairs{0};

    auto sphereRadius = [&](size_t i) {
        return kind_[i] == static_cast<uint32_t>(ColliderKind::Sphere) ? radius_[i]
                                                                      : length(halfExtent_[i]);
    };

    auto narrow = [&](size_t i, size_t j, Contact& out) -> bool {
        bool boxA = kind_[i] == static_cast<uint32_t>(ColliderKind::Box);
        bool boxB = kind_[j] == static_cast<uint32_t>(ColliderKind::Box);
        if (!boxA && !boxB) {
            Vec3 d = position_[j] - position_[i];
            float rsum = radius_[i] + radius_[j];
            float dist2 = length2(d);
            if (dist2 >= rsum * rsum || dist2 < 1e-12f) return false;
            float dist = std::sqrt(dist2);
            out.normal = d / dist;
            out.depth = rsum - dist;
            return true;
        }
        if (boxA && boxB) {
            Vec3 d = position_[j] - position_[i];
            Vec3 overlap = (halfExtent_[i] + halfExtent_[j]) - vabs(d);
            if (overlap.x <= 0 || overlap.y <= 0 || overlap.z <= 0) return false;
            int axis = overlap.x < overlap.y ? (overlap.x < overlap.z ? 0 : 2) : (overlap.y < overlap.z ? 1 : 2);
            Vec3 nrm{0, 0, 0};
            nrm[axis] = d[axis] < 0 ? -1.0f : 1.0f;
            out.normal = nrm;
            out.depth = overlap[axis];
            return true;
        }
        size_t s = boxA ? j : i;
        size_t b = boxA ? i : j;
        Vec3 rel = position_[s] - position_[b];
        Vec3 clamped{std::clamp(rel.x, -halfExtent_[b].x, halfExtent_[b].x),
                     std::clamp(rel.y, -halfExtent_[b].y, halfExtent_[b].y),
                     std::clamp(rel.z, -halfExtent_[b].z, halfExtent_[b].z)};
        Vec3 delta = rel - clamped;
        float dist2 = length2(delta);
        float r = radius_[s];
        if (dist2 >= r * r) return false;
        Vec3 nrm;
        float depth;
        if (dist2 < 1e-12f) {
            Vec3 slack = halfExtent_[b] - vabs(rel);
            int axis = slack.x < slack.y ? (slack.x < slack.z ? 0 : 2) : (slack.y < slack.z ? 1 : 2);
            nrm = Vec3{0, 0, 0};
            nrm[axis] = rel[axis] < 0 ? -1.0f : 1.0f;
            depth = r + slack[axis];
        } else {
            float dist = std::sqrt(dist2);
            nrm = delta / dist;
            depth = r - dist;
        }
        if (boxA) {
            out.normal = nrm;
        } else {
            out.normal = -nrm;
        }
        out.depth = depth;
        return true;
    };

    auto scan = [&](size_t begin, size_t end) {
        std::vector<Contact>& sink = contactChunks_[begin / CONTACT_GRAIN];
        uint64_t localPairs = 0;
        for (size_t i = begin; i < end; ++i) {
            int32_t cx = cellCoord_[i * 3 + 0];
            int32_t cy = cellCoord_[i * 3 + 1];
            int32_t cz = cellCoord_[i * 3 + 2];
            float reach = sphereRadius(i);
            for (int dz = -1; dz <= 1; ++dz)
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        int32_t nx = cx + dx, ny = cy + dy, nz = cz + dz;
                        uint32_t bucket = hashCell(nx, ny, nz) & bucketMask_;
                        for (uint32_t k = cellStart_[bucket]; k < cellStart_[bucket + 1]; ++k) {
                            uint32_t j = sorted_[k];
                            if (j <= i) continue;
                            if (cellCoord_[j * 3 + 0] != nx || cellCoord_[j * 3 + 1] != ny ||
                                cellCoord_[j * 3 + 2] != nz)
                                continue;
                            ++localPairs;
                            float cull = reach + sphereRadius(j);
                            if (length2(position_[j] - position_[i]) > cull * cull) continue;
                            Contact c{static_cast<uint32_t>(i), j, Vec3{0, 1, 0}, 0};
                            if (narrow(i, j, c)) sink.push_back(c);
                        }
                    }
        }
        pairs.fetch_add(localPairs, std::memory_order_relaxed);
    };

    if (jobs && n >= CONTACT_GRAIN * 2)
        jobs->parallelFor(n, CONTACT_GRAIN, scan);
    else if (n > 0)
        scan(0, n);

    contacts_.clear();
    for (auto& c : contactChunks_) contacts_.insert(contacts_.end(), c.begin(), c.end());
    stats_.pairsTested = pairs.load();
    stats_.contacts = static_cast<uint32_t>(contacts_.size());
}

void PhysicsWorld::resolve() {
    SKEIN_PROFILE("physics/solve");
    const float slop = 0.001f;
    const float correction = 0.4f;
    for (int iter = 0; iter < std::max(1, settings.solverIterations); ++iter) {
        for (const Contact& c : contacts_) {
            float imA = invMass_[c.a];
            float imB = invMass_[c.b];
            float imSum = imA + imB;
            if (imSum <= 0.0f) continue;
            Vec3 rel = velocity_[c.b] - velocity_[c.a];
            float vn = dot(rel, c.normal);
            if (vn < 0.0f) {
                float e = std::min(restitution_[c.a], restitution_[c.b]);
                float jImp = -(1.0f + e) * vn / imSum;
                Vec3 impulse = c.normal * jImp;
                velocity_[c.a] -= impulse * imA;
                velocity_[c.b] += impulse * imB;
            }
            if (iter == 0) {
                float push = std::max(c.depth - slop, 0.0f) * correction / imSum;
                velocity_[c.a] -= c.normal * (push * imA);
                velocity_[c.b] += c.normal * (push * imB);
            }
        }
    }

    if (!settings.useBounds) return;
    for (size_t i = 0; i < position_.size(); ++i) {
        if (invMass_[i] <= 0.0f) continue;
        float r = kind_[i] == static_cast<uint32_t>(ColliderKind::Sphere) ? radius_[i]
                                                                         : maxComponent(halfExtent_[i]);
        for (int axis = 0; axis < 3; ++axis) {
            float lo = settings.boundsMin[axis] + r;
            float hi = settings.boundsMax[axis] - r;
            if (position_[i][axis] < lo) {
                position_[i][axis] = lo;
                if (velocity_[i][axis] < 0) velocity_[i][axis] = -velocity_[i][axis] * settings.restitutionFloor;
            } else if (position_[i][axis] > hi) {
                position_[i][axis] = hi;
                if (velocity_[i][axis] > 0) velocity_[i][axis] = -velocity_[i][axis] * settings.restitutionFloor;
            }
        }
    }
}

void PhysicsWorld::scatter(Scene& scene) {
    SKEIN_PROFILE("physics/scatter");
    Pool<Transform>& transforms = scene.world.pool<Transform>();
    Pool<Velocity>& velocities = scene.world.pool<Velocity>();
    for (size_t i = 0; i < entity_.size(); ++i) {
        if (Transform* t = transforms.tryGet(entity_[i])) t->position = position_[i];
        if (Velocity* v = velocities.tryGet(entity_[i])) v->linear = velocity_[i];
    }
}

PhysicsStats PhysicsWorld::step(Scene& scene, float dt, JobSystem* jobs) {
    SKEIN_PROFILE("physics/step");
    gather(scene);
    stats_.bodies = static_cast<uint32_t>(position_.size());
    if (position_.empty()) {
        stats_.contacts = 0;
        stats_.pairsTested = 0;
        return stats_;
    }
    integrate(dt, jobs);
    buildGrid(jobs);
    findContacts(jobs);
    resolve();
    scatter(scene);
    return stats_;
}

size_t PhysicsWorld::bytesUsed() const {
    size_t total = position_.capacity() * sizeof(Vec3) * 3 + entity_.capacity() * sizeof(Entity) +
                   radius_.capacity() * sizeof(float) * 3 + kind_.capacity() * sizeof(uint32_t) +
                   cellCoord_.capacity() * sizeof(int32_t) + bucketOf_.capacity() * sizeof(uint32_t) +
                   cellStart_.capacity() * sizeof(uint32_t) + sorted_.capacity() * sizeof(uint32_t) +
                   contacts_.capacity() * sizeof(Contact);
    for (const auto& c : contactChunks_) total += c.capacity() * sizeof(Contact);
    return total;
}

}  // namespace skein
