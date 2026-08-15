#include "physics/physics.hpp"

#include <algorithm>
#include <atomic>
#include <bit>
#include <type_traits>

#include "core/jobs.hpp"
#include "core/profiler.hpp"
#include "scene/scene.hpp"

namespace skein {

/// Overlap the solver leaves alone, to keep resting contacts from buzzing.
constexpr float kSlop = 0.005f;
/// Fraction of the remaining overlap the positional pass removes per iteration,
/// and the speed that push is allowed to reach.
constexpr float kCorrection = 0.2f;
constexpr float kMaxSeparation = 2.0f;
/// Overlap a sleeping body refuses to tolerate.
// ponytail: absolute, not relative to body size; scale it by radius if scenes
// ever mix boulders with pebbles.
constexpr float kWakeDepth = 0.05f;
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

/// Stable identity for a contact across frames, so last frame's impulse can be
/// found again after the pair list has been rebuilt from scratch.
uint64_t contactKey(uint64_t ea, uint64_t eb) {
    uint64_t lo = ea < eb ? ea : eb;
    uint64_t hi = ea < eb ? eb : ea;
    uint64_t h = lo * 0x9E3779B97F4A7C15ull;
    h ^= (hi + 0xD6E8FEB86659FD93ull + (h << 6) + (h >> 2));
    h *= 0xFF51AFD7ED558CCDull;
    h ^= h >> 29;
    return h | 1ull;
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
    sleepTimer_.clear();
    asleep_.clear();
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
    sleepTimer_.reserve(n);
    asleep_.reserve(n);

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
        sleepTimer_.push_back(c.sleepTimer);
        asleep_.push_back(settings.allowSleep ? static_cast<uint8_t>(c.asleep) : uint8_t{0});
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
            if (asleep_[i]) continue;
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
        e.awake = asleep_[i] ? 0u : 1u;
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
    const float wakeSpeed2 = settings.sleepSpeed * settings.sleepSpeed;
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
                uint32_t awake = entries_[runStart].awake;
                while (runEnd < to && entries_[runEnd].cell == key) awake += entries_[runEnd++].awake;
                // A cell where everything is asleep can only produce pairs that
                // are both asleep, and those are skipped anyway.
                if (awake == 0) {
                    runStart = runEnd;
                    continue;
                }
                for (uint32_t a = runStart; a + 1 < runEnd; ++a) {
                    const GridEntry& ea = entries_[a];
                    for (uint32_t c = a + 1; c < runEnd; ++c) {
                        const GridEntry& eb = entries_[c];
                        float dx = eb.x - ea.x, dy = eb.y - ea.y, dz = eb.z - ea.z;
                        float cull = ea.reach + eb.reach;
                        ++localPairs;
                        if (dx * dx + dy * dy + dz * dz > cull * cull) continue;
                        const uint32_t i = std::min(ea.body, eb.body), j = std::max(ea.body, eb.body);
                        if (asleep_[i] && asleep_[j]) continue;
                        if (key != packCell(std::max(bodyLo_[i * 3], bodyLo_[j * 3]),
                                            std::max(bodyLo_[i * 3 + 1], bodyLo_[j * 3 + 1]),
                                            std::max(bodyLo_[i * 3 + 2], bodyLo_[j * 3 + 2])))
                            continue;
                        Contact contact{i, j, Vec3{0, 1, 0}, 0};
                        if (!narrow(i, j, contact)) continue;
                        // Only a body that is actually moving wakes a sleeper,
                        // or one jittering body would cascade through a settled
                        // pile and wake all of it. An overlap deep enough to
                        // matter wakes it regardless: once both sides sleep,
                        // nothing is left to push them apart.
                        const bool deep = contact.depth > kWakeDepth;
                        if (asleep_[i] && (deep || length2(velocity_[j]) > wakeSpeed2))
                            std::atomic_ref<uint8_t>(asleep_[i]).store(0, std::memory_order_relaxed);
                        if (asleep_[j] && (deep || length2(velocity_[i]) > wakeSpeed2))
                            std::atomic_ref<uint8_t>(asleep_[j]).store(0, std::memory_order_relaxed);
                        sink.push_back(contact);
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

void PhysicsWorld::solveRange(uint32_t begin, uint32_t end, bool positional, float invDt) {
    const float friction = settings.friction;
    for (uint32_t idx = begin; idx < end; ++idx) {
        const uint32_t k = colorOrder_[idx];
        const Contact& c = contacts_[k];
        float imA = invMass_[c.a];
        float imB = invMass_[c.b];
        float imSum = imA + imB;
        if (imSum <= 0.0f) continue;
        Vec3 rel = velocity_[c.b] - velocity_[c.a];
        float vn = dot(rel, c.normal);

        // The bounce target is fixed before the first iteration, or accumulating
        // impulses would cancel the restitution back out on the next pass.
        float lambda = -(vn - restitutionBias_[k]) / imSum;
        // The accumulated impulse is what can be clamped against zero and
        // carried into the next frame; a per-iteration impulse cannot.
        float& total = normalImpulse_[k];
        float previous = total;
        total = std::max(previous + lambda, 0.0f);
        float applied = total - previous;
        if (applied != 0.0f) {
            Vec3 impulse = c.normal * applied;
            if (imA > 0.0f) velocity_[c.a] -= impulse * imA;
            if (imB > 0.0f) velocity_[c.b] += impulse * imB;
        }

        // Coulomb friction along the contact tangent, bounded by the normal
        // impulse holding the pair together. Without it a pile of frictionless
        // spheres slides forever and never settles.
        if (total > 0.0f) {
            rel = velocity_[c.b] - velocity_[c.a];
            Vec3 tangent = rel - c.normal * dot(rel, c.normal);
            float tl = length(tangent);
            if (tl > 1e-5f) {
                tangent *= 1.0f / tl;
                float jt = std::min(tl / imSum, friction * total);
                Vec3 tImpulse = tangent * jt;
                if (imA > 0.0f) velocity_[c.a] += tImpulse * imA;
                if (imB > 0.0f) velocity_[c.b] -= tImpulse * imB;
            }
        }

        if (positional) {
            // Contacts are colour-partitioned, so a body is touched by at most
            // one contact per pass and this needs no atomics.
            deepest_[c.a] = std::max(deepest_[c.a], c.depth);
            deepest_[c.b] = std::max(deepest_[c.b], c.depth);
            // Split impulse: the depth correction moves bodies apart through a
            // separate pseudo velocity that is integrated into position and
            // then dropped, so pushing an overlap out never feeds real motion.
            float target = std::min(std::max(c.depth - kSlop, 0.0f) * kCorrection * invDt, kMaxSeparation);
            float pvn = dot(pseudo_[c.b] - pseudo_[c.a], c.normal);
            float push = (target - pvn) / imSum;
            if (push > 0.0f) {
                if (imA > 0.0f) pseudo_[c.a] -= c.normal * (push * imA);
                if (imB > 0.0f) pseudo_[c.b] += c.normal * (push * imB);
            }
        }
    }
}

void PhysicsWorld::warmStart(uint32_t begin, uint32_t end) {
    for (uint32_t idx = begin; idx < end; ++idx) {
        const uint32_t k = colorOrder_[idx];
        const Contact& c = contacts_[k];
        float p = normalImpulse_[k];
        if (p == 0.0f) continue;
        Vec3 impulse = c.normal * p;
        if (invMass_[c.a] > 0.0f) velocity_[c.a] -= impulse * invMass_[c.a];
        if (invMass_[c.b] > 0.0f) velocity_[c.b] += impulse * invMass_[c.b];
    }
}

void PhysicsWorld::loadCachedImpulses(JobSystem* jobs) {
    SKEIN_PROFILE("physics/impulseCache");
    const size_t n = contacts_.size();
    contactKey_.resize(n);
    normalImpulse_.resize(n);
    restitutionBias_.resize(n);
    auto body = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            uint64_t key = contactKey(entity_[contacts_[i].a], entity_[contacts_[i].b]);
            contactKey_[i] = key;
            float found = 0.0f;
            if (settings.warmStart && cacheMask_ != 0) {
                for (uint32_t slot = static_cast<uint32_t>(key) & cacheMask_;; slot = (slot + 1) & cacheMask_) {
                    if (cache_[slot].key == 0) break;
                    if (cache_[slot].key == key) {
                        found = cache_[slot].impulse;
                        break;
                    }
                }
            }
            normalImpulse_[i] = found;
            // Resting contacts must not bounce, so restitution only enters
            // above a speed where a collision is what is actually happening.
            float vn = dot(velocity_[contacts_[i].b] - velocity_[contacts_[i].a], contacts_[i].normal);
            float e = vn < -1.0f ? std::min(restitution_[contacts_[i].a], restitution_[contacts_[i].b]) : 0.0f;
            restitutionBias_[i] = -e * vn;
        }
    };
    if (jobs && n >= 8192)
        jobs->parallelFor(n, 4096, body);
    else
        body(0, n);
}

void PhysicsWorld::storeCachedImpulses(JobSystem* jobs) {
    SKEIN_PROFILE("physics/impulseStore");
    const size_t n = contacts_.size();
    uint32_t slots = std::max(nextPow2(static_cast<uint32_t>(n) * 2 + 1), 64u);
    cacheMask_ = slots - 1;
    cache_.resize(slots);
    auto clear = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) cache_[i] = CacheSlot{};
    };
    // Claiming a slot with a compare-exchange is all the synchronisation an
    // insert-only table needs, so the whole pass runs on the job system.
    auto insert = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            if (normalImpulse_[i] <= 0.0f) continue;
            uint64_t key = contactKey_[i];
            for (uint32_t slot = static_cast<uint32_t>(key) & cacheMask_;; slot = (slot + 1) & cacheMask_) {
                std::atomic_ref<uint64_t> cell(cache_[slot].key);
                uint64_t empty = 0;
                if (cell.load(std::memory_order_relaxed) == key ||
                    cell.compare_exchange_strong(empty, key, std::memory_order_relaxed)) {
                    cache_[slot].impulse = normalImpulse_[i];
                    break;
                }
            }
        }
    };
    if (jobs && n >= 8192) {
        jobs->parallelFor(slots, 8192, clear);
        jobs->parallelFor(n, 4096, insert);
    } else {
        clear(0, slots);
        insert(0, n);
    }
}

void PhysicsWorld::solveBounds(size_t begin, size_t end, float invDt) {
    const float friction = settings.friction;
    for (size_t i = begin; i < end; ++i) {
        if (invMass_[i] <= 0.0f || asleep_[i]) continue;
        float r = kind_[i] == static_cast<uint32_t>(ColliderKind::Sphere) ? radius_[i]
                                                                         : maxComponent(halfExtent_[i]);
        for (int axis = 0; axis < 3; ++axis) {
            const float lo = settings.boundsMin[axis] + r;
            const float hi = settings.boundsMax[axis] - r;
            float depth;
            float sign;
            if (position_[i][axis] < lo) {
                depth = lo - position_[i][axis];
                sign = 1.0f;
            } else if (position_[i][axis] > hi) {
                depth = position_[i][axis] - hi;
                sign = -1.0f;
            } else {
                continue;
            }
            float vn = velocity_[i][axis] * sign;
            if (vn < 0.0f) {
                velocity_[i][axis] = -velocity_[i][axis] * settings.restitutionFloor;
                // The wall carries the whole normal impulse, so its friction
                // budget is what stops bodies skating along the floor forever.
                float budget = friction * (-vn);
                for (int t = 0; t < 3; ++t) {
                    if (t == axis) continue;
                    float& vt = velocity_[i][t];
                    float mag = std::abs(vt);
                    vt = mag <= budget ? 0.0f : vt - std::copysign(budget, vt);
                }
            }
            float target = std::min(std::max(depth - kSlop, 0.0f) * kCorrection * invDt, kMaxSeparation);
            float& pn = pseudo_[i][axis];
            if (pn * sign < target) pn = target * sign;
        }
    }
}

void PhysicsWorld::resolve(float dt, JobSystem* jobs) {
    SKEIN_PROFILE("physics/solve");
    pseudo_.assign(position_.size(), Vec3{0, 0, 0});
    deepest_.assign(position_.size(), 0.0f);
    const float invDt = dt > 1e-6f ? 1.0f / dt : 0.0f;
    loadCachedImpulses(jobs);
    // Warm starting has to be its own pass over every contact. Folded into the
    // first solve iteration it does nothing at all: solving a contact right
    // after replaying its impulse overwrites exactly what was replayed, and the
    // benefit comes from the *other* contacts already being loaded.
    for (uint32_t color = 0; settings.warmStart && color <= colorCount_; ++color) {
        uint32_t begin = colorStart_[color == colorCount_ ? SERIAL_COLOR : color];
        uint32_t end = colorStart_[(color == colorCount_ ? SERIAL_COLOR : color) + 1];
        size_t span = end - begin;
        if (jobs && span >= SOLVE_GRAIN * 2 && color != colorCount_)
            jobs->parallelFor(span, SOLVE_GRAIN, [&](size_t lo, size_t hi) {
                warmStart(begin + static_cast<uint32_t>(lo), begin + static_cast<uint32_t>(hi));
            });
        else
            warmStart(begin, end);
    }
    for (int iter = 0; iter < std::max(1, settings.solverIterations); ++iter) {
        const bool positional = true;
        if (settings.useBounds) {
            if (jobs && position_.size() >= 8192)
                jobs->parallelFor(position_.size(), 4096, [&](size_t lo, size_t hi) { solveBounds(lo, hi, invDt); });
            else
                solveBounds(0, position_.size(), invDt);
        }
        for (uint32_t color = 0; color < colorCount_; ++color) {
            uint32_t begin = colorStart_[color];
            uint32_t end = colorStart_[color + 1];
            size_t span = end - begin;
            if (jobs && span >= SOLVE_GRAIN * 2)
                jobs->parallelFor(span, SOLVE_GRAIN, [&](size_t lo, size_t hi) {
                    solveRange(begin + static_cast<uint32_t>(lo), begin + static_cast<uint32_t>(hi), positional, invDt);
                });
            else
                solveRange(begin, end, positional, invDt);
        }
        solveRange(colorStart_[SERIAL_COLOR], colorStart_[SERIAL_COLOR + 1], positional, invDt);
    }

    if (settings.warmStart) storeCachedImpulses(jobs);

    auto applyPseudo = [&](size_t begin, size_t end) {
        // A sleeper stays exactly where it was when it fell asleep; nudging it
        // is how two sleeping bodies end up merged with nothing to part them.
        for (size_t i = begin; i < end; ++i)
            if (!asleep_[i]) position_[i] += pseudo_[i] * dt;
    };
    if (jobs && position_.size() >= 8192)
        jobs->parallelFor(position_.size(), 4096, applyPseudo);
    else
        applyPseudo(0, position_.size());

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

void PhysicsWorld::updateSleep(float dt, JobSystem* jobs) {
    SKEIN_PROFILE("physics/sleep");
    const size_t n = position_.size();
    const float threshold = settings.sleepSpeed * settings.sleepSpeed;
    std::atomic<uint32_t> awake{0};
    auto body = [&](size_t begin, size_t end) {
        uint32_t localAwake = 0;
        for (size_t i = begin; i < end; ++i) {
            if (invMass_[i] <= 0.0f) continue;
            // Still being pushed out of an overlap counts as moving, or bodies
            // would freeze while the solver was mid-way through separating them.
            // A body still buried in something has not come to rest, however
            // still it looks: freezing it there would leave the overlap
            // permanent, since sleepers no longer push each other apart.
            const bool separating = deepest_[i] > kWakeDepth;
            if (length2(velocity_[i]) > threshold || separating) {
                sleepTimer_[i] = 0.0f;
                asleep_[i] = 0;
            } else {
                sleepTimer_[i] += dt;
                if (sleepTimer_[i] >= settings.sleepTime) {
                    asleep_[i] = 1;
                    velocity_[i] = Vec3{0, 0, 0};
                }
            }
            if (!asleep_[i]) ++localAwake;
        }
        awake.fetch_add(localAwake, std::memory_order_relaxed);
    };
    if (jobs && n >= 8192)
        jobs->parallelFor(n, 4096, body);
    else
        body(0, n);
    stats_.awake = awake.load();
}

void PhysicsWorld::scatter(Scene& scene, JobSystem* jobs) {
    SKEIN_PROFILE("physics/scatter");
    Pool<Transform>& transforms = scene.world.pool<Transform>();
    Pool<Velocity>& velocities = scene.world.pool<Velocity>();
    Pool<Collider>& colliders = scene.world.pool<Collider>();
    auto write = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            if (Transform* t = transforms.tryGet(entity_[i])) t->position = position_[i];
            if (Velocity* v = velocities.tryGet(entity_[i])) v->linear = velocity_[i];
            if (Collider* c = colliders.tryGet(entity_[i])) {
                c->sleepTimer = sleepTimer_[i];
                c->asleep = asleep_[i];
            }
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
        stats_.awake = 0;
        return stats_;
    }
    integrate(dt, jobs);
    buildGrid(jobs);
    findContacts(jobs);
    colorContacts();
    resolve(dt, jobs);
    if (settings.allowSleep) updateSleep(dt, jobs);
    else stats_.awake = static_cast<uint32_t>(position_.size());
    scatter(scene, jobs);
    return stats_;
}

size_t PhysicsWorld::bytesUsed() const {
    auto bytes = [](const auto& v) { return v.capacity() * sizeof(typename std::decay_t<decltype(v)>::value_type); };
    size_t total = bytes(entity_) + bytes(position_) + bytes(velocity_) + bytes(pseudo_) + bytes(halfExtent_) +
                   bytes(radius_) + bytes(invMass_) + bytes(restitution_) + bytes(kind_) + bytes(sleepTimer_) +
                   bytes(asleep_) + bytes(deepest_) + bytes(reach_) + bytes(bodyLo_) + bytes(bodyHi_) +
                   bytes(entryOffset_) + bytes(entryBucket_) + bytes(cellStart_) + bytes(entries_) +
                   bytes(contacts_) + bytes(normalImpulse_) + bytes(restitutionBias_) + bytes(contactKey_) +
                   bytes(cache_) + bytes(bodyColorMask_) + bytes(contactColor_) +
                   bytes(colorStart_) + bytes(colorOrder_);
    for (const auto& c : contactChunks_) total += bytes(c);
    return total;
}

}  // namespace skein
