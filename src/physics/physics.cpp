#include "physics/physics.hpp"

#include <algorithm>
#include <atomic>
#include <bit>

#include "core/jobs.hpp"
#include "core/profiler.hpp"
#include "scene/scene.hpp"

namespace skein {
namespace {

constexpr size_t BUCKET_GRAIN = 2048;
constexpr size_t SOLVE_GRAIN = 1024;
/// Colours live in a 64-bit per-body mask; contacts that find no free colour
/// land in this bucket and are solved on one thread at the end of each pass.
constexpr uint32_t MAX_COLORS = 64;
constexpr uint32_t SERIAL_COLOR = MAX_COLORS;
constexpr float MAX_CELL_SPAN = 4.0f;

uint32_t hashCell(int32_t x, int32_t y, int32_t z) {
    uint32_t h = static_cast<uint32_t>(x) * 73856093u;
    h ^= static_cast<uint32_t>(y) * 19349663u;
    h ^= static_cast<uint32_t>(z) * 83492791u;
    h ^= h >> 15;
    return h;
}

/// Exact cell identity. Two cells that share a hash bucket still differ here,
/// which is what keeps a body registered in several cells from reporting the
/// same pair twice.
uint64_t packCell(int32_t x, int32_t y, int32_t z) {
    constexpr uint64_t bias = 1ull << 20;
    return ((static_cast<uint64_t>(x) + bias) & 0x1FFFFFull) << 42 |
           ((static_cast<uint64_t>(y) + bias) & 0x1FFFFFull) << 21 |
           ((static_cast<uint64_t>(z) + bias) & 0x1FFFFFull);
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
    reach_.clear();
    reach_.reserve(n);
    entity_.reserve(n);
    position_.reserve(n);
    velocity_.reserve(n);
    halfExtent_.reserve(n);
    radius_.reserve(n);
    invMass_.reserve(n);
    restitution_.reserve(n);
    kind_.reserve(n);

    maxReach_ = 0.0f;
    double reachSum = 0.0;
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
        float reach = c.kind == static_cast<uint32_t>(ColliderKind::Sphere) ? c.radius * s
                                                                           : length(c.halfExtents * s);
        reach_.push_back(reach);
        maxReach_ = std::max(maxReach_, reach);
        reachSum += reach;
    }
    meanReach_ = position_.empty() ? 0.0f : static_cast<float>(reachSum / static_cast<double>(position_.size()));
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
    const size_t n = position_.size();

    // A cell sized to the largest body would make every cell coarse enough to
    // swamp the scan, so the grid follows the typical body and lets oversized
    // ones occupy the several cells they actually cover. The floor keeps a
    // single huge collider from spanning an unbounded number of cells.
    cell_ = std::max({settings.cellSize, 2.0f * meanReach_, 2.0f * maxReach_ / MAX_CELL_SPAN});
    if (cell_ <= 0.0f) cell_ = 1.0f;
    stats_.cellSize = cell_;
    const float inv = 1.0f / cell_;

    bodyLo_.resize(n * 3);
    bodyHi_.resize(n * 3);
    entryOffset_.resize(n + 1);

    auto classify = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            const Vec3 p = position_[i];
            const float r = reach_[i];
            uint32_t span = 1;
            for (int axis = 0; axis < 3; ++axis) {
                int32_t lo = static_cast<int32_t>(std::floor((p[axis] - r) * inv));
                int32_t hi = static_cast<int32_t>(std::floor((p[axis] + r) * inv));
                bodyLo_[i * 3 + axis] = lo;
                bodyHi_[i * 3 + axis] = hi;
                span *= static_cast<uint32_t>(hi - lo + 1);
            }
            entryOffset_[i + 1] = span;
        }
    };
    if (jobs && n >= 8192)
        jobs->parallelFor(n, 8192, classify);
    else
        classify(0, n);

    entryOffset_[0] = 0;
    for (size_t i = 0; i < n; ++i) entryOffset_[i + 1] += entryOffset_[i];
    const uint32_t entries = entryOffset_[n];

    uint32_t buckets = std::max(nextPow2(entries * 2 + 1), 64u);
    bucketMask_ = buckets - 1;
    entryBucket_.resize(entries);
        cellStart_.assign(buckets + 1, 0);

    auto scatterCells = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            uint32_t at = entryOffset_[i];
            for (int32_t z = bodyLo_[i * 3 + 2]; z <= bodyHi_[i * 3 + 2]; ++z)
                for (int32_t y = bodyLo_[i * 3 + 1]; y <= bodyHi_[i * 3 + 1]; ++y)
                    for (int32_t x = bodyLo_[i * 3 + 0]; x <= bodyHi_[i * 3 + 0]; ++x)
                        entryBucket_[at++] = hashCell(x, y, z) & bucketMask_;
        }
    };
    if (jobs && n >= 8192)
        jobs->parallelFor(n, 8192, scatterCells);
    else
        scatterCells(0, n);

    for (uint32_t i = 0; i < entries; ++i) ++cellStart_[entryBucket_[i] + 1];
    uint32_t used = 0;
    for (size_t i = 1; i < cellStart_.size(); ++i) {
        if (cellStart_[i] != 0) ++used;
        cellStart_[i] += cellStart_[i - 1];
    }
    // The scan reads cell identity, position and reach straight out of the
    // sorted entry array, so rejecting a candidate never touches the body
    // arrays at all.
    entries_.resize(entries);
    std::vector<uint32_t> cursor(cellStart_.begin(), cellStart_.end() - 1);
    for (size_t i = 0; i < n; ++i) {
        GridEntry e;
        e.x = position_[i].x;
        e.y = position_[i].y;
        e.z = position_[i].z;
        e.reach = reach_[i];
        e.body = static_cast<uint32_t>(i);
        uint32_t k = entryOffset_[i];
        for (int32_t cz = bodyLo_[i * 3 + 2]; cz <= bodyHi_[i * 3 + 2]; ++cz)
            for (int32_t cy = bodyLo_[i * 3 + 1]; cy <= bodyHi_[i * 3 + 1]; ++cy)
                for (int32_t cx = bodyLo_[i * 3 + 0]; cx <= bodyHi_[i * 3 + 0]; ++cx) {
                    e.cell = packCell(cx, cy, cz);
                    entries_[cursor[entryBucket_[k++]]++] = e;
                }
    }
    // Entries land grouped by hash bucket; sorting each bucket by the exact
    // cell key turns every bucket into contiguous runs of one cell, which is
    // what lets the scan walk cells directly instead of hashing per body.
    auto sortBuckets = [&](size_t begin, size_t end) {
        for (size_t b = begin; b < end; ++b) {
            uint32_t from = cellStart_[b], to = cellStart_[b + 1];
            if (to - from > 1)
                std::sort(entries_.begin() + from, entries_.begin() + to,
                          [](const GridEntry& a, const GridEntry& b) { return a.cell < b.cell; });
        }
    };
    if (jobs && buckets >= 8192)
        jobs->parallelFor(buckets, 4096, sortBuckets);
    else
        sortBuckets(0, buckets);

    stats_.gridCells = used;
    stats_.gridEntries = entries;
}

void PhysicsWorld::findContacts(JobSystem* jobs) {
    SKEIN_PROFILE("physics/narrowphase");
    std::atomic<uint64_t> pairs{0};

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

    // Bodies are registered in every cell they overlap, so an overlapping pair
    // is guaranteed to meet in at least one shared cell. Reporting only from
    // the lowest shared cell keeps each pair unique without a visited set.
    const uint32_t buckets = bucketMask_ + 1;
    const size_t chunks = buckets == 0 ? 0 : (buckets + BUCKET_GRAIN - 1) / BUCKET_GRAIN;
    contactChunks_.resize(chunks);
    for (auto& c : contactChunks_) c.clear();

    auto scan = [&](size_t begin, size_t end) {
        std::vector<Contact>& sink = contactChunks_[begin / BUCKET_GRAIN];
        uint64_t localPairs = 0;
        for (size_t b = begin; b < end; ++b) {
            const uint32_t from = cellStart_[b], to = cellStart_[b + 1];
            for (uint32_t runStart = from; runStart < to;) {
                const uint64_t key = entries_[runStart].cell;
                uint32_t runEnd = runStart + 1;
                while (runEnd < to && entries_[runEnd].cell == key) ++runEnd;
                for (uint32_t a = runStart; a + 1 < runEnd; ++a) {
                    const GridEntry& ea = entries_[a];
                    for (uint32_t c = a + 1; c < runEnd; ++c) {
                        const GridEntry& eb = entries_[c];
                        float dx = eb.x - ea.x, dy = eb.y - ea.y, dz = eb.z - ea.z;
                        float cull = ea.reach + eb.reach;
                        ++localPairs;
                        if (dx * dx + dy * dy + dz * dz > cull * cull) continue;
                        const uint32_t i = std::min(ea.body, eb.body), j = std::max(ea.body, eb.body);
                        if (key != packCell(std::max(bodyLo_[i * 3], bodyLo_[j * 3]),
                                            std::max(bodyLo_[i * 3 + 1], bodyLo_[j * 3 + 1]),
                                            std::max(bodyLo_[i * 3 + 2], bodyLo_[j * 3 + 2])))
                            continue;
                        Contact contact{i, j, Vec3{0, 1, 0}, 0};
                        if (narrow(i, j, contact)) sink.push_back(contact);
                    }
                }
                runStart = runEnd;
            }
        }
        pairs.fetch_add(localPairs, std::memory_order_relaxed);
    };

    if (jobs && buckets >= BUCKET_GRAIN * 2)
        jobs->parallelFor(buckets, BUCKET_GRAIN, scan);
    else if (buckets > 0)
        scan(0, buckets);

    contacts_.clear();
    for (auto& c : contactChunks_) contacts_.insert(contacts_.end(), c.begin(), c.end());
    stats_.pairsTested = pairs.load();
    stats_.contacts = static_cast<uint32_t>(contacts_.size());
}

void PhysicsWorld::colorContacts() {
    SKEIN_PROFILE("physics/color");
    const size_t n = contacts_.size();
    bodyColorMask_.assign(position_.size(), 0);
    contactColor_.resize(n);
    colorCount_ = 0;
    uint32_t serial = 0;

    for (size_t i = 0; i < n; ++i) {
        const Contact& c = contacts_[i];
        const bool dynA = invMass_[c.a] > 0.0f;
        const bool dynB = invMass_[c.b] > 0.0f;
        uint64_t used = (dynA ? bodyColorMask_[c.a] : 0ull) | (dynB ? bodyColorMask_[c.b] : 0ull);
        if (used == ~0ull) {
            contactColor_[i] = SERIAL_COLOR;
            ++serial;
            continue;
        }
        uint32_t color = static_cast<uint32_t>(std::countr_one(used));
        uint64_t bit = 1ull << color;
        if (dynA) bodyColorMask_[c.a] |= bit;
        if (dynB) bodyColorMask_[c.b] |= bit;
        contactColor_[i] = color;
        colorCount_ = std::max(colorCount_, color + 1);
    }

    colorStart_.assign(MAX_COLORS + 2, 0);
    for (size_t i = 0; i < n; ++i) ++colorStart_[contactColor_[i] + 1];
    for (size_t i = 1; i < colorStart_.size(); ++i) colorStart_[i] += colorStart_[i - 1];
    colorOrder_.resize(n);
    std::vector<uint32_t> cursor(colorStart_.begin(), colorStart_.end() - 1);
    for (size_t i = 0; i < n; ++i) colorOrder_[cursor[contactColor_[i]]++] = static_cast<uint32_t>(i);

    stats_.colors = colorCount_;
    stats_.serialContacts = serial;
}

void PhysicsWorld::solveRange(uint32_t begin, uint32_t end, bool positional) {
    const float slop = 0.001f;
    const float correction = 0.4f;
    for (uint32_t idx = begin; idx < end; ++idx) {
        const Contact& c = contacts_[colorOrder_[idx]];
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
            if (imA > 0.0f) velocity_[c.a] -= impulse * imA;
            if (imB > 0.0f) velocity_[c.b] += impulse * imB;
        }
        if (positional) {
            float push = std::max(c.depth - slop, 0.0f) * correction / imSum;
            if (imA > 0.0f) velocity_[c.a] -= c.normal * (push * imA);
            if (imB > 0.0f) velocity_[c.b] += c.normal * (push * imB);
        }
    }
}

void PhysicsWorld::resolve(JobSystem* jobs) {
    SKEIN_PROFILE("physics/solve");
    for (int iter = 0; iter < std::max(1, settings.solverIterations); ++iter) {
        const bool positional = iter == 0;
        for (uint32_t color = 0; color < colorCount_; ++color) {
            uint32_t begin = colorStart_[color];
            uint32_t end = colorStart_[color + 1];
            size_t span = end - begin;
            if (jobs && span >= SOLVE_GRAIN * 2)
                jobs->parallelFor(span, SOLVE_GRAIN, [&](size_t lo, size_t hi) {
                    solveRange(begin + static_cast<uint32_t>(lo), begin + static_cast<uint32_t>(hi), positional);
                });
            else
                solveRange(begin, end, positional);
        }
        solveRange(colorStart_[SERIAL_COLOR], colorStart_[SERIAL_COLOR + 1], positional);
    }

    if (!settings.useBounds) return;
    auto clampToBounds = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
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
    };
    if (jobs && position_.size() >= 8192)
        jobs->parallelFor(position_.size(), 4096, clampToBounds);
    else
        clampToBounds(0, position_.size());
}

void PhysicsWorld::scatter(Scene& scene, JobSystem* jobs) {
    SKEIN_PROFILE("physics/scatter");
    Pool<Transform>& transforms = scene.world.pool<Transform>();
    Pool<Velocity>& velocities = scene.world.pool<Velocity>();
    auto write = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            if (Transform* t = transforms.tryGet(entity_[i])) t->position = position_[i];
            if (Velocity* v = velocities.tryGet(entity_[i])) v->linear = velocity_[i];
        }
    };
    if (jobs && entity_.size() >= 8192)
        jobs->parallelFor(entity_.size(), 4096, write);
    else
        write(0, entity_.size());
}

PhysicsStats PhysicsWorld::step(Scene& scene, float dt, JobSystem* jobs) {
    SKEIN_PROFILE("physics/step");
    gather(scene);
    stats_.bodies = static_cast<uint32_t>(position_.size());
    if (position_.empty()) {
        stats_.contacts = 0;
        stats_.pairsTested = 0;
        stats_.colors = 0;
        stats_.serialContacts = 0;
        return stats_;
    }
    integrate(dt, jobs);
    buildGrid(jobs);
    findContacts(jobs);
    colorContacts();
    resolve(jobs);
    scatter(scene, jobs);
    return stats_;
}

size_t PhysicsWorld::bytesUsed() const {
    size_t total = position_.capacity() * sizeof(Vec3) * 3 + entity_.capacity() * sizeof(Entity) +
                   radius_.capacity() * sizeof(float) * 3 + kind_.capacity() * sizeof(uint32_t) +
                   (bodyLo_.capacity() + bodyHi_.capacity()) * sizeof(int32_t) +
                   (entryOffset_.capacity() + entryBucket_.capacity()) * sizeof(uint32_t) +
                   cellStart_.capacity() * sizeof(uint32_t) +                    contacts_.capacity() * sizeof(Contact) + entries_.capacity() * sizeof(GridEntry) + bodyColorMask_.capacity() * sizeof(uint64_t) +
                   contactColor_.capacity() * sizeof(uint32_t) + colorOrder_.capacity() * sizeof(uint32_t);
    for (const auto& c : contactChunks_) total += c.capacity() * sizeof(Contact);
    return total;
}

}  // namespace skein
