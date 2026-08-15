#include "physics/physics.hpp"

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <limits>
#include <type_traits>

#include "core/jobs.hpp"
#include "core/profiler.hpp"
#include "scene/scene.hpp"

namespace skein {

/// Overlap the solver leaves alone, to keep resting contacts from buzzing.
constexpr float kSlop = 0.005f;
/// Fraction of the remaining overlap the positional pass removes per iteration,
/// and the speed that push is allowed to reach.
constexpr float kCorrection = 0.35f;
constexpr float kMaxSeparation = 2.0f;
/// Fraction of the normal impulse a contact may spend resisting the pair's
/// relative spin. Sliding friction cannot stop a ball that is already rolling —
/// the contact point is stationary — so without this a pile of spheres jostles
/// forever and never sleeps.
constexpr float kRollingFriction = 0.25f;
/// How far a clipped manifold point may sit above the face and still be kept.
/// A resting box tips by a fraction of a millimetre each frame; dropping the
/// corners that lift by that much leaves it standing on two points and tipping
/// further. The solver already reads a point above the surface as a bound on
/// approach rather than an overlap, so keeping them costs nothing.
constexpr float kManifoldSkin = 0.05f;
/// Overlap a sleeping body refuses to tolerate.
// ponytail: absolute, not relative to body size; scale it by radius if scenes
// ever mix boulders with pebbles.
constexpr float kWakeDepth = 0.05f;
/// How deep a body may still be when it is allowed to fall asleep. Tighter than
/// the depth that wakes a sleeper: a sleeper stops being pushed out, so it keeps
/// whatever overlap it had, and the solver settles well under this given frames.
constexpr float kSleepDepth = 0.015f;
/// How often sleeping bodies are measured against each other again.
constexpr uint64_t kSleepAudit = 16;
namespace {

constexpr size_t BUCKET_GRAIN = 2048;
/// Contacts per solver chunk. Small, because a colour is solved and then
/// waited on: leaving a colour serial costs more than the dispatch does now
/// that a parallel pass queues one task per worker rather than one per chunk.
constexpr size_t SOLVE_GRAIN = 128;
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

/// Half-width of a box along an arbitrary direction.
float project(const Vec3* axes, const Vec3& half, const Vec3& n) {
    return std::abs(dot(axes[0], n)) * half.x + std::abs(dot(axes[1], n)) * half.y +
           std::abs(dot(axes[2], n)) * half.z;
}

/// Corner furthest along `n`.
Vec3 support(const Vec3& center, const Vec3* axes, const Vec3& half, const Vec3& n) {
    return center + axes[0] * std::copysign(half.x, dot(axes[0], n)) +
           axes[1] * std::copysign(half.y, dot(axes[1], n)) + axes[2] * std::copysign(half.z, dot(axes[2], n));
}

/// One box as the clipper sees it.
struct BoxRef {
    Vec3 center;
    const Vec3* axes;
    Vec3 half;
};

/// Clip the incident box's nearest face against the side planes of the
/// reference face and keep the points that are still touching. A single point
/// cannot hold a flat face down: a box resting on a box rocks about it forever,
/// which is what the corners of the contact patch are for.
int clipFace(const BoxRef& ref, int refAxis, float refSign, const BoxRef& inc, float margin, Vec3* points,
             float* depths, uint32_t* ids) {
    const Vec3 refNormal = ref.axes[refAxis] * refSign;

    int incAxis = 0;
    float most = -1.0f;
    for (int k = 0; k < 3; ++k) {
        float d = std::abs(dot(inc.axes[k], refNormal));
        if (d <= most) continue;
        most = d;
        incAxis = k;
    }
    const float incSign = dot(inc.axes[incAxis], refNormal) > 0.0f ? -1.0f : 1.0f;
    const int iu = (incAxis + 1) % 3, iv = (incAxis + 2) % 3;
    const Vec3 incCenter = inc.center + inc.axes[incAxis] * (incSign * inc.half[incAxis]);
    const Vec3 eu = inc.axes[iu] * inc.half[iu];
    const Vec3 ev = inc.axes[iv] * inc.half[iv];

    Vec3 poly[8] = {incCenter - eu - ev, incCenter + eu - ev, incCenter + eu + ev, incCenter - eu + ev};
    // Each point carries where it came from — an incident corner, or the edge
    // between two of them cut by one of the reference face's side planes. That
    // is what a point is called next frame too, however far the pair has crept,
    // so the impulse it accumulated finds it again.
    uint32_t tag[8] = {0, 1, 2, 3};
    int count = 4;
    Vec3 scratch[8];
    uint32_t scratchTag[8];

    const int ru = (refAxis + 1) % 3, rv = (refAxis + 2) % 3;
    const int sideAxis[4] = {ru, ru, rv, rv};
    const float sideSign[4] = {1.0f, -1.0f, 1.0f, -1.0f};
    for (int plane = 0; plane < 4 && count > 0; ++plane) {
        const Vec3 n = ref.axes[sideAxis[plane]] * sideSign[plane];
        const float limit = ref.half[sideAxis[plane]] + dot(ref.center, n);
        int out = 0;
        for (int i = 0; i < count; ++i) {
            const Vec3& a = poly[i];
            const Vec3& b = poly[(i + 1) % count];
            const float da = dot(a, n) - limit;
            const float db = dot(b, n) - limit;
            if (da <= 0.0f) {
                scratchTag[out] = tag[i];
                scratch[out++] = a;
            }
            if ((da < 0.0f) != (db < 0.0f) && out < 8) {
                scratchTag[out] = 4u + static_cast<uint32_t>(plane) * 4u + (tag[i] & 3u);
                scratch[out++] = a + (b - a) * (da / (da - db));
            }
            if (out >= 8) break;
        }
        count = out;
        for (int i = 0; i < count; ++i) {
            poly[i] = scratch[i];
            tag[i] = scratchTag[i];
        }
    }

    const float faceOffset = dot(ref.center, refNormal) + ref.half[refAxis];
    int kept = 0;
    for (int i = 0; i < count; ++i) {
        const float depth = faceOffset - dot(poly[i], refNormal);
        if (depth < -margin) continue;
        points[kept] = poly[i];
        depths[kept] = depth;
        ids[kept] = tag[i] | (static_cast<uint32_t>(refAxis) << 8) | (static_cast<uint32_t>(incAxis) << 10) |
                    (refSign > 0.0f ? 1u << 12 : 0u) | (incSign > 0.0f ? 1u << 13 : 0u);
        ++kept;
    }
    if (kept <= 4) return kept;
    // Four points is what a box face needs, and which four decides whether the
    // pair is stable: dropping the shallowest leaves the choice to rounding
    // noise on a patch whose points are all at the same depth, and the set then
    // changes every frame. Picking the widest quad instead depends on the
    // shape of the patch, which does not flicker.
    int pick[4];
    pick[0] = 0;
    for (int i = 1; i < kept; ++i)
        if (depths[i] > depths[pick[0]]) pick[0] = i;
    pick[1] = pick[0];
    float far = -1.0f;
    for (int i = 0; i < kept; ++i) {
        float d = length2(points[i] - points[pick[0]]);
        if (d <= far) continue;
        far = d;
        pick[1] = i;
    }
    const Vec3 edge = points[pick[1]] - points[pick[0]];
    pick[2] = pick[0];
    float widest = -1.0f;
    for (int i = 0; i < kept; ++i) {
        float area = length2(cross(edge, points[i] - points[pick[0]]));
        if (area <= widest) continue;
        widest = area;
        pick[2] = i;
    }
    pick[3] = pick[0];
    float best = -1.0f;
    for (int i = 0; i < kept; ++i) {
        if (i == pick[0] || i == pick[1] || i == pick[2]) continue;
        float area = length2(cross(points[pick[2]] - points[pick[1]], points[i] - points[pick[1]])) +
                     length2(cross(points[pick[2]] - points[pick[0]], points[i] - points[pick[0]]));
        if (area <= best) continue;
        best = area;
        pick[3] = i;
    }
    Vec3 keptPoints[4];
    float keptDepths[4];
    uint32_t keptIds[4];
    for (int i = 0; i < 4; ++i) {
        keptPoints[i] = points[pick[i]];
        keptDepths[i] = depths[pick[i]];
        keptIds[i] = ids[pick[i]];
    }
    for (int i = 0; i < 4; ++i) {
        points[i] = keptPoints[i];
        depths[i] = keptDepths[i];
        ids[i] = keptIds[i];
    }
    return 4;
}

/// Separating-axis test between two oriented boxes. Returns false only when a
/// axis leaves them further apart than `margin`; otherwise `depth` is the least
/// overlap found, negative when the pair is still apart, and `normal` points
/// from a towards b. The contact point is the midpoint of the two support
/// points along that axis, which is the corner or edge actually in contact.
bool satBoxes(const Vec3& ca, const Vec3* axa, const Vec3& ha, const Vec3& cb, const Vec3* axb, const Vec3& hb,
              float margin, Vec3& normal, float& depth, Vec3& point, int& bestIndex) {
    const Vec3 d = cb - ca;
    // The separating axis is the one that leaves the least overlap: it is the
    // shortest push that would part them, and every other axis overlaps more
    // only because it is looking along the pair rather than across it.
    float best = std::numeric_limits<float>::max();
    float bestBiased = std::numeric_limits<float>::max();
    Vec3 bestAxis{0, 1, 0};
    bestIndex = -1;
    int index = 0;
    auto test = [&](const Vec3& n) {
        const int slot = index++;
        float len2 = length2(n);
        if (len2 < 1e-8f) return true;
        const Vec3 u = n / std::sqrt(len2);
        float overlap = project(axa, ha, u) + project(axb, hb, u) - std::abs(dot(d, u));
        if (overlap < -margin) return false;
        // A face axis is preferred over an edge-cross axis that is barely
        // better, since the face is the one with a patch to clip and a normal
        // that does not swing about as the pair settles.
        float biased = overlap + (slot < 6 ? 0.0f : 1e-3f);
        // Two parallel faces overlap by the same amount along both of their
        // normals. Requiring a real improvement to switch keeps the choice from
        // alternating on rounding noise, which would hand the solver a fresh set
        // of contact points every frame and lose everything it had accumulated.
        if (biased < bestBiased - 1e-4f) {
            bestBiased = biased;
            best = overlap;
            bestAxis = dot(d, u) < 0.0f ? -u : u;
            bestIndex = slot;
        }
        return true;
    };
    for (int i = 0; i < 3; ++i)
        if (!test(axa[i])) return false;
    for (int i = 0; i < 3; ++i)
        if (!test(axb[i])) return false;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (!test(cross(axa[i], axb[j]))) return false;
    normal = bestAxis;
    depth = best;
    point = (support(ca, axa, ha, bestAxis) + support(cb, axb, hb, -bestAxis)) * 0.5f;
    return true;
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

    // Eleven push_backs per body means eleven size counters kept live across
    // the loop; writing through a single cursor into pre-sized arrays is the
    // same work with one. A collider without a transform is skipped, so the
    // cursor can trail the source index and the arrays shrink at the end.
    size_t n = colliders.dense.size();
    entity_.resize(n);
    position_.resize(n);
    velocity_.resize(n);
    angular_.resize(n);
    orientation_.resize(n);
    invInertia_.resize(n);
    axis_.resize(n * 3);
    rotated_.resize(n);
    halfExtent_.resize(n);
    radius_.resize(n);
    invMass_.resize(n);
    restitution_.resize(n);
    kind_.resize(n);
    sleepTimer_.resize(n);
    asleep_.resize(n);
    reach_.resize(n);

    maxReach_ = 0.0f;
    minThin_ = 0.0f;
    maxSpeed2_ = 0.0f;
    double reachSum = 0.0;
    const float sleepSpeed2 = settings.sleepSpeed * settings.sleepSpeed;
    size_t m = 0;
    for (size_t i = 0; i < n; ++i) {
        Entity e = colliders.dense[i];
        const Transform* t = transforms.tryGet(e);
        if (!t) continue;
        const Collider& c = colliders.data[i];
        const Velocity* v = velocities.tryGet(e);
        Vec3 linear = v ? v->linear : Vec3{0, 0, 0};
        float s = maxComponent(vabs(t->scale));
        entity_[m] = e;
        position_[m] = t->position;
        velocity_[m] = linear;
        angular_[m] = v ? v->angular : Vec3{0, 0, 0};
        orientation_[m] = t->rotation;
        // A box whose rotation is identity is its own axis-aligned extent, and
        // the cheap test is exact for it. Everything else carries its three
        // world axes so the narrowphase can separate it along them.
        bool turned = settings.rotatedBoxes && c.kind == static_cast<uint32_t>(ColliderKind::Box) &&
                      std::abs(t->rotation.w) < 0.99999f;
        rotated_[m] = turned ? uint8_t{1} : uint8_t{0};
        if (turned) {
            axis_[m * 3 + 0] = rotate(t->rotation, Vec3{1, 0, 0});
            axis_[m * 3 + 1] = rotate(t->rotation, Vec3{0, 1, 0});
            axis_[m * 3 + 2] = rotate(t->rotation, Vec3{0, 0, 1});
        }
        halfExtent_[m] = c.halfExtents * s;
        radius_[m] = c.radius * s;
        invMass_[m] = c.invMass;
        restitution_[m] = c.restitution;
        kind_[m] = c.kind;
        sleepTimer_[m] = c.sleepTimer;
        // Anything outside physics that gives a sleeper velocity — a script, the
        // editor, a loaded scene — has to wake it, or it hangs wherever it was
        // put. ponytail: a teleport that leaves velocity alone still will not
        // wake it; store the last scattered position if that ever comes up.
        bool asleep = settings.allowSleep && c.asleep != 0 && length2(linear) <= sleepSpeed2;
        asleep_[m] = asleep ? uint8_t{1} : uint8_t{0};
        float reach = c.kind == static_cast<uint32_t>(ColliderKind::Sphere) ? c.radius * s
                                                                           : length(c.halfExtents * s);
        reach_[m] = reach;
        // Inertia of a solid sphere of the collider's own reach, which is the
        // one number that makes contact torque behave: too small and a graze
        // spins the body up out of nothing, too large and nothing ever tumbles.
        invInertia_[m] = c.invMass > 0.0f && reach > 0.0f ? c.invMass / (0.4f * reach * reach) : 0.0f;
        maxReach_ = std::max(maxReach_, reach);
        reachSum += reach;
        // Tunneling is bounded by the thinnest dimension in the world, not by
        // the bounding radius: a wide floor slab is easy to shoot through
        // exactly where it is thin.
        float thin = c.kind == static_cast<uint32_t>(ColliderKind::Sphere)
                         ? radius_[m]
                         : minComponent(halfExtent_[m]);
        if (thin > 0.0f) minThin_ = minThin_ == 0.0f ? thin : std::min(minThin_, thin);
        maxSpeed2_ = std::max(maxSpeed2_, length2(linear));
        ++m;
    }
    if (m != n) {
        entity_.resize(m);
        position_.resize(m);
        velocity_.resize(m);
        angular_.resize(m);
        orientation_.resize(m);
        invInertia_.resize(m);
        axis_.resize(m * 3);
        rotated_.resize(m);
        halfExtent_.resize(m);
        radius_.resize(m);
        invMass_.resize(m);
        restitution_.resize(m);
        kind_.resize(m);
        sleepTimer_.resize(m);
        asleep_.resize(m);
        reach_.resize(m);
    }
    meanReach_ = m == 0 ? 0.0f : static_cast<float>(reachSum / static_cast<double>(m));
}

void PhysicsWorld::integrateVelocities(float dt, JobSystem* jobs) {
    SKEIN_PROFILE("physics/integrate");
    size_t n = position_.size();
    const Vec3 g = settings.gravity;
    const float damp = std::max(0.0f, 1.0f - settings.linearDamping * dt);
    const float spinDamp = std::max(0.0f, 1.0f - settings.angularDamping * dt);
    const bool angular = settings.angularContacts;
    auto body = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            if (asleep_[i]) continue;
            if (invMass_[i] > 0.0f) velocity_[i] += g * dt;
            velocity_[i] *= damp;
            if (angular) angular_[i] *= spinDamp;
        }
    };
    if (jobs && n >= 4096)
        jobs->parallelFor(n, 4096, body);
    else
        body(0, n);
}

void PhysicsWorld::integratePositions(float dt, JobSystem* jobs) {
    SKEIN_PROFILE("physics/integrate");
    size_t n = position_.size();
    const bool angular = settings.angularContacts;
    auto body = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            if (asleep_[i]) continue;
            const Vec3 step = velocity_[i] * dt;
            position_[i] += step;
            if (!angular) continue;
            float speed = length(angular_[i]);
            if (speed > 1e-6f) {
                Quat turn = Quat::axisAngle(angular_[i] / speed, speed * dt);
                orientation_[i] = normalize(turn * orientation_[i]);
                // What the contact anchors have to be turned by. Kept as its
                // own quaternion so a contact never needs the orientation the
                // narrowphase saw, only the difference.
                spinDelta_[i] = normalize(turn * spinDelta_[i]);
            }
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

    sweep_.resize(n);
    motion_.resize(n);
    bodyLo_.resize(n * 3);
    bodyHi_.resize(n * 3);
    entryOffset_.resize(n + 1);

    auto classify = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            const Vec3 p = position_[i];
            // A body is registered as wide as it needs to be for a pair it will
            // meet during the step to already share a cell with it. Only the
            // part of the motion its own extent does not already cover counts:
            // a pair is missed when it crosses more than the two extents put
            // together, so reach + max(0, motion - reach) per side is still a
            // bound and costs a slow body nothing. The clamp is what the
            // substep count is derived from: past one cell of motion the
            // inflation would spread one body over the whole grid, and
            // splitting the step is cheaper.
            const float sweep =
                settings.speculativeContacts
                    ? std::clamp(length(velocity_[i]) * stepDt_ - reach_[i], 0.0f, cell_)
                    : 0.0f;
            sweep_[i] = sweep;
            // The grid is built where the bodies are now and the solve moves
            // them afterwards, so the narrowphase has to accept a pair that is
            // still apart by as much as the step will close.
            motion_[i] = length(velocity_[i]) * stepDt_;
            const float r = reach_[i] + sweep;
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
        e.reach = reach_[i] + sweep_[i];
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
    std::atomic<uint64_t> near{0};
    std::atomic<uint64_t> duplicates{0};

    // `margin` is how far apart the pair may still be and count: the distance
    // they can close before the next test. A contact found across a gap carries
    // a negative depth, and the solver reads that as "may approach this far".
    // Returns how many contact points the pair produced: one for anything
    // curved, up to four where two box faces meet, since a single point cannot
    // stop a flat face from rocking about it.
    auto narrowAll = [&](size_t i, size_t j, Contact* manifold, float margin) -> int {
        Contact& out = manifold[0];
        bool boxA = kind_[i] == static_cast<uint32_t>(ColliderKind::Box);
        bool boxB = kind_[j] == static_cast<uint32_t>(ColliderKind::Box);
        if (!boxA && !boxB) {
            Vec3 d = position_[j] - position_[i];
            float rsum = radius_[i] + radius_[j];
            float reach = rsum + margin;
            float dist2 = length2(d);
            if (dist2 >= reach * reach || dist2 < 1e-12f) return 0;
            float dist = std::sqrt(dist2);
            out.normal = d / dist;
            out.depth = rsum - dist;
            out.point = position_[i] + out.normal * (radius_[i] - 0.5f * out.depth);
            return 1;
        }
        if (boxA && boxB) {
            if (rotated_[i] || rotated_[j]) {
                // Fifteen axes is a lot to spend on a pair the three world axes
                // already separate, and the grid hands the narrowphase plenty of
                // those: the cell scan only knows the pair is within reach.
                Vec3 gap = vabs(position_[j] - position_[i]);
                for (int axis = 0; axis < 3; ++axis)
                    if (gap[axis] > boundsReach(i, axis) + boundsReach(j, axis) + margin) return 0;
                static const Vec3 world[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
                const Vec3* axa = rotated_[i] ? &axis_[i * 3] : world;
                const Vec3* axb = rotated_[j] ? &axis_[j * 3] : world;
                int axisIndex = -1;
                if (!satBoxes(position_[i], axa, halfExtent_[i], position_[j], axb, halfExtent_[j], margin,
                              out.normal, out.depth, out.point, axisIndex))
                    return 0;
                if (axisIndex < 0 || axisIndex >= 6) return 1;
                // The axis that separated them names the face doing the
                // holding; the other box's nearest face is clipped against it.
                const BoxRef boxA{position_[i], axa, halfExtent_[i]};
                const BoxRef boxB{position_[j], axb, halfExtent_[j]};
                const bool refIsA = axisIndex < 3;
                const BoxRef& ref = refIsA ? boxA : boxB;
                const BoxRef& inc = refIsA ? boxB : boxA;
                const int refAxis = axisIndex % 3;
                const Vec3 refNormal = refIsA ? out.normal : -out.normal;
                const float refSign = dot(ref.axes[refAxis], refNormal) < 0.0f ? -1.0f : 1.0f;
                Vec3 points[8];
                float depths[8];
                uint32_t ids[8];
                int count = clipFace(ref, refAxis, refSign, inc, margin + kManifoldSkin, points, depths, ids);
                if (count == 0) return 1;
                for (int k = 0; k < count; ++k) {
                    manifold[k].a = out.a;
                    manifold[k].b = out.b;
                    manifold[k].normal = out.normal;
                    manifold[k].depth = depths[k];
                    manifold[k].id = ids[k] | (refIsA ? 1u << 14 : 0u);
                    manifold[k].point = points[k];
                }
                return count;
            }
            Vec3 d = position_[j] - position_[i];
            Vec3 overlap = (halfExtent_[i] + halfExtent_[j] + Vec3{margin, margin, margin}) - vabs(d);
            if (overlap.x <= 0 || overlap.y <= 0 || overlap.z <= 0) return 0;
            int axis = overlap.x < overlap.y ? (overlap.x < overlap.z ? 0 : 2) : (overlap.y < overlap.z ? 1 : 2);
            Vec3 nrm{0, 0, 0};
            nrm[axis] = d[axis] < 0 ? -1.0f : 1.0f;
            out.normal = nrm;
            out.depth = overlap[axis] - margin;
            // The corners of the rectangle the two faces share, on the plane
            // halfway through the overlap. A single point in the middle of that
            // rectangle holds the pair apart just as well but does nothing to
            // stop it pivoting, and a box resting on one point tips.
            float lo[3], hi[3];
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::max(position_[i][k] - halfExtent_[i][k], position_[j][k] - halfExtent_[j][k]);
                hi[k] = std::min(position_[i][k] + halfExtent_[i][k], position_[j][k] + halfExtent_[j][k]);
                if (lo[k] > hi[k]) lo[k] = hi[k] = 0.5f * (lo[k] + hi[k]);
            }
            const int u = (axis + 1) % 3, v = (axis + 2) % 3;
            const float plane = 0.5f * (lo[axis] + hi[axis]);
            // A rectangle thinner than the slop is an edge or a corner, and its
            // duplicate corners would each apply the whole contact impulse.
            const float wide = hi[u] - lo[u] > kSlop, deep = hi[v] - lo[v] > kSlop;
            int count = 0;
            for (int cu = 0; cu < (wide ? 2 : 1); ++cu)
                for (int cv = 0; cv < (deep ? 2 : 1); ++cv) {
                    Vec3 p;
                    p[axis] = plane;
                    p[u] = cu ? hi[u] : lo[u];
                    p[v] = cv ? hi[v] : lo[v];
                    manifold[count] = out;
                    manifold[count].point = p;
                    manifold[count].id = static_cast<uint32_t>(1 + cu * 2 + cv);
                    ++count;
                }
            return count;
        }
        size_t s = boxA ? j : i;
        size_t b = boxA ? i : j;
        // A turned box is met in its own frame: the clamp is the same test,
        // and the point and normal are rotated back out at the end.
        const bool turned = rotated_[b] != 0;
        const Quat inv = turned ? conjugate(orientation_[b]) : Quat{};
        Vec3 rel = position_[s] - position_[b];
        if (turned) rel = rotate(inv, rel);
        Vec3 clamped{std::clamp(rel.x, -halfExtent_[b].x, halfExtent_[b].x),
                     std::clamp(rel.y, -halfExtent_[b].y, halfExtent_[b].y),
                     std::clamp(rel.z, -halfExtent_[b].z, halfExtent_[b].z)};
        Vec3 delta = rel - clamped;
        float dist2 = length2(delta);
        float r = radius_[s];
        float reach = r + margin;
        if (dist2 >= reach * reach) return 0;
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
        Vec3 contactPoint = position_[b] + (turned ? rotate(orientation_[b], clamped) : clamped);
        if (turned) nrm = rotate(orientation_[b], nrm);
        if (boxA) {
            out.normal = nrm;
        } else {
            out.normal = -nrm;
        }
        out.depth = depth;
        out.point = contactPoint;
        return 1;
    };

    // Bodies are registered in every cell they overlap, so an overlapping pair
    // is guaranteed to meet in at least one shared cell. Reporting only from
    // the lowest shared cell keeps each pair unique without a visited set.
    const bool audit = (frame_ % kSleepAudit) == 0;
    const uint32_t buckets = bucketMask_ + 1;
    const size_t chunks = buckets == 0 ? 0 : (buckets + BUCKET_GRAIN - 1) / BUCKET_GRAIN;
    contactChunks_.resize(chunks);
    for (auto& c : contactChunks_) c.clear();

    auto scan = [&](size_t begin, size_t end) {
        std::vector<Contact>& sink = contactChunks_[begin / BUCKET_GRAIN];
        uint64_t localPairs = 0, localNear = 0, localDuplicates = 0;
        for (size_t b = begin; b < end; ++b) {
            const uint32_t from = cellStart_[b], to = cellStart_[b + 1];
            for (uint32_t runStart = from; runStart < to;) {
                const uint64_t key = entries_[runStart].cell;
                uint32_t runEnd = runStart + 1;
                uint32_t awake = entries_[runStart].awake;
                while (runEnd < to && entries_[runEnd].cell == key) awake += entries_[runEnd++].awake;
                // A cell where everything is asleep only produces pairs that
                // are both asleep, and those are skipped except on an audit
                // frame.
                if (awake == 0 && !audit) {
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
                        ++localNear;
                        const uint32_t i = std::min(ea.body, eb.body), j = std::max(ea.body, eb.body);
                        // Two sleepers cannot push each other apart, so an
                        // overlap that appeared just as the second one fell
                        // asleep would otherwise be permanent. Every audit
                        // frame the pair is measured again — but only to wake
                        // them, since solving a contact for a body the rest of
                        // the step skips would leave velocity nobody applies.
                        if (asleep_[i] && asleep_[j]) {
                            if (!audit) continue;
                            Contact probe[4];
                            probe[0] = Contact{i, j, Vec3{0, 1, 0}, 0, 0, Vec3{0, 0, 0}};
                            if (narrowAll(i, j, probe, 0.0f) > 0 && probe[0].depth > kSleepDepth) {
                                // The timer has to go back with the flag: this
                                // pair contributes no contact this step, so
                                // nothing else would stop the sleep test at the
                                // end of the step from putting them straight
                                // back under.
                                std::atomic_ref<uint8_t>(asleep_[i]).store(0, std::memory_order_relaxed);
                                std::atomic_ref<uint8_t>(asleep_[j]).store(0, std::memory_order_relaxed);
                                sleepTimer_[i] = 0.0f;
                                sleepTimer_[j] = 0.0f;
                            }
                            continue;
                        }
                        if (key != packCell(std::max(bodyLo_[i * 3], bodyLo_[j * 3]),
                                            std::max(bodyLo_[i * 3 + 1], bodyLo_[j * 3 + 1]),
                                            std::max(bodyLo_[i * 3 + 2], bodyLo_[j * 3 + 2]))) {
                            ++localDuplicates;
                            continue;
                        }
                        Contact manifold[4];
                        manifold[0] = Contact{i, j, Vec3{0, 1, 0}, 0, 0, Vec3{0, 0, 0}};
                        int found = narrowAll(i, j, manifold, motion_[i] + motion_[j] + kManifoldSkin);
                        if (found == 0) continue;
                        const Contact& contact = manifold[0];
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
                        for (int m = 0; m < found; ++m) {
                            manifold[m].anchorA = manifold[m].point - position_[i];
                            manifold[m].anchorB = manifold[m].point - position_[j];
                            sink.push_back(manifold[m]);
                        }
                    }
                }
                runStart = runEnd;
            }
        }
        pairs.fetch_add(localPairs, std::memory_order_relaxed);
        near.fetch_add(localNear, std::memory_order_relaxed);
        duplicates.fetch_add(localDuplicates, std::memory_order_relaxed);
    };

    if (jobs && buckets >= BUCKET_GRAIN * 2)
        jobs->parallelFor(buckets, BUCKET_GRAIN, scan);
    else if (buckets > 0)
        scan(0, buckets);

    contacts_.clear();
    for (auto& c : contactChunks_) contacts_.insert(contacts_.end(), c.begin(), c.end());
    stats_.pairsTested = pairs.load();
    stats_.nearPairs = near.load();
    stats_.duplicatePairs = duplicates.load();
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
    const bool angular = settings.angularContacts;
    for (uint32_t idx = begin; idx < end; ++idx) {
        const uint32_t k = colorOrder_[idx];
        const Contact& c = contacts_[k];
        float imA = invMass_[c.a];
        float imB = invMass_[c.b];
        float imSum = imA + imB;
        if (imSum <= 0.0f) continue;
        const float iiA = angular ? invInertia_[c.a] : 0.0f;
        const float iiB = angular ? invInertia_[c.b] : 0.0f;
        // Where the touching point is now: the arm the narrowphase measured,
        // turned by however far the body has turned since. A stack settles by
        // rotating a few degrees, and an arm left pointing where the pair used
        // to touch resolves the wrong constraint by exactly that much.
        const Vec3 armA = rotate(spinDelta_[c.a], c.anchorA);
        const Vec3 armB = rotate(spinDelta_[c.b], c.anchorB);
        const Vec3 rA = angular ? armA : Vec3{0, 0, 0};
        const Vec3 rB = angular ? armB : Vec3{0, 0, 0};
        // Velocity of the touching points, not of the centres: an impulse at
        // arm's length is what turns a glancing hit into a spin.
        auto pointVel = [&](uint32_t body, const Vec3& r, float ii) {
            return ii > 0.0f ? velocity_[body] + cross(angular_[body], r) : velocity_[body];
        };
        // Effective mass along an axis, including how much the pair resists
        // being rotated by an impulse there.
        auto effMass = [&](const Vec3& u) {
            Vec3 ca = cross(rA, u), cb = cross(rB, u);
            return imSum + iiA * length2(ca) + iiB * length2(cb);
        };
        auto applyImpulse = [&](const Vec3& j) {
            if (imA > 0.0f) velocity_[c.a] -= j * imA;
            if (imB > 0.0f) velocity_[c.b] += j * imB;
            if (iiA > 0.0f) angular_[c.a] -= cross(rA, j) * iiA;
            if (iiB > 0.0f) angular_[c.b] += cross(rB, j) * iiB;
        };
        Vec3 rel = pointVel(c.b, rB, iiB) - pointVel(c.a, rA, iiA);
        float vn = dot(rel, c.normal);
        const float normalMass = angular ? effMass(c.normal) : imSum;
        // How deep the pair is *now*, not when the narrowphase looked: the
        // bodies have been moving between substeps, and a constraint that keeps
        // answering with the depth it was born with is solving last position's
        // problem.
        const float depth = c.depth - dot((position_[c.b] + armB) - (position_[c.a] + armA), c.normal);

        // A contact across a gap does not forbid approach, it bounds it: the
        // pair may close exactly the distance between them and no more, which
        // lands the body on the surface instead of inside it or past it.
        float separation = depth < 0.0f ? -depth * invDt : 0.0f;
        float lambda = -(vn + separation) / normalMass;
        // The accumulated impulse is what can be clamped against zero and
        // carried into the next frame; a per-iteration impulse cannot.
        float& total = normalImpulse_[k];
        float previous = total;
        total = std::max(previous + lambda, 0.0f);
        float applied = total - previous;
        if (applied != 0.0f) applyImpulse(c.normal * applied);

        // Coulomb friction along the contact tangent, bounded by the normal
        // impulse holding the pair together. Without it a pile of frictionless
        // spheres slides forever and never settles.
        // Friction accumulates like the normal impulse does, as a vector in the
        // contact plane clamped against the normal impulse holding the pair
        // together. Recomputing it from scratch each iteration cannot hold a
        // slope: the correction it applied last iteration is invisible to it.
        {
            rel = pointVel(c.b, rB, iiB) - pointVel(c.a, rA, iiA);
            Vec3 slide = rel - c.normal * dot(rel, c.normal);
            float sl = length(slide);
            // The tangent mass belongs to the direction the pair is actually
            // sliding in; the two in-plane axes do not resist a spin equally.
            float tangentMass = angular && sl > 1e-9f ? effMass(slide / sl) : imSum;
            Vec3 want = tangentImpulse_[k] - slide / tangentMass;
            float limit = friction * total;
            float wl = length(want);
            if (wl > limit) want = wl > 1e-12f ? want * (limit / wl) : Vec3{0, 0, 0};
            Vec3 applyT = want - tangentImpulse_[k];
            tangentImpulse_[k] = want;
            applyImpulse(applyT);
        }

        if (angular && total > 0.0f) {
            Vec3 spin = angular_[c.b] - angular_[c.a];
            float wl = length(spin);
            float iiSum = iiA + iiB;
            if (wl > 1e-9f && iiSum > 0.0f) {
                float budget = kRollingFriction * total * 0.5f * (reach_[c.a] + reach_[c.b]);
                float applyR = std::min(wl / iiSum, budget);
                Vec3 dir = spin / wl;
                angular_[c.a] += dir * (applyR * iiA);
                angular_[c.b] -= dir * (applyR * iiB);
            }
        }

        if (positional) {
            // Contacts are colour-partitioned, so a body is touched by at most
            // one contact per pass and this needs no atomics.
            deepest_[c.a] = std::max(deepest_[c.a], depth);
            deepest_[c.b] = std::max(deepest_[c.b], depth);
            // Split impulse: the depth correction moves bodies apart through a
            // separate pseudo velocity that is integrated into position and
            // then dropped, so pushing an overlap out never feeds real motion.
            float target = std::min(std::max(depth - kSlop, 0.0f) * kCorrection * invDt, kMaxSeparation);
            // The push has to be able to turn the body as well as move it: a box
            // resting on one deep corner is separated by rotating about the
            // others, and a solver that can only translate lifts it off them
            // instead, leaving it tilted for good.
            Vec3 pa = pseudo_[c.a] + (iiA > 0.0f ? cross(pseudoSpin_[c.a], rA) : Vec3{0, 0, 0});
            Vec3 pb = pseudo_[c.b] + (iiB > 0.0f ? cross(pseudoSpin_[c.b], rB) : Vec3{0, 0, 0});
            float pvn = dot(pb - pa, c.normal);
            float push = (target - pvn) / normalMass;
            if (push > 0.0f) {
                const Vec3 j = c.normal * push;
                if (imA > 0.0f) pseudo_[c.a] -= j * imA;
                if (imB > 0.0f) pseudo_[c.b] += j * imB;
                if (iiA > 0.0f) pseudoSpin_[c.a] -= cross(rA, j) * iiA;
                if (iiB > 0.0f) pseudoSpin_[c.b] += cross(rB, j) * iiB;
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
        Vec3 impulse = c.normal * p + tangentImpulse_[k];
        if (invMass_[c.a] > 0.0f) velocity_[c.a] -= impulse * invMass_[c.a];
        if (invMass_[c.b] > 0.0f) velocity_[c.b] += impulse * invMass_[c.b];
        if (!settings.angularContacts) continue;
        if (invInertia_[c.a] > 0.0f) angular_[c.a] -= cross(c.point - position_[c.a], impulse) * invInertia_[c.a];
        if (invInertia_[c.b] > 0.0f) angular_[c.b] += cross(c.point - position_[c.b], impulse) * invInertia_[c.b];
    }
}

void PhysicsWorld::loadCachedImpulses(JobSystem* jobs) {
    SKEIN_PROFILE("physics/impulseCache");
    const size_t n = contacts_.size();
    contactKey_.resize(n);
    normalImpulse_.resize(n);
    tangentImpulse_.resize(n);
    restitutionBias_.resize(n);
    approach_.resize(n);
    auto body = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            uint64_t key = contactKey(entity_[contacts_[i].a], entity_[contacts_[i].b]) ^
                           (static_cast<uint64_t>(contacts_[i].id) * 0x9E3779B97F4A7C15ull);
            key |= 1ull;
            contactKey_[i] = key;
            float found = 0.0f;
            float foundApproach = 0.0f;
            Vec3 foundTangent{0, 0, 0};
            if (cacheMask_ != 0) {
                for (uint32_t slot = static_cast<uint32_t>(key) & cacheMask_;; slot = (slot + 1) & cacheMask_) {
                    if (cache_[slot].key == 0) break;
                    if (cache_[slot].key == key) {
                        foundApproach = cache_[slot].approach;
                        if (settings.warmStart) {
                            found = cache_[slot].impulse;
                            foundTangent = cache_[slot].tangent;
                        }
                        break;
                    }
                }
            }
            normalImpulse_[i] = found;
            // The plane the impulse was accumulated in is not exactly the plane
            // it will be replayed in, so the tangent part is projected back
            // onto the current one before it is reused.
            Vec3 t = foundTangent - contacts_[i].normal * dot(foundTangent, contacts_[i].normal);
            tangentImpulse_[i] = t;
            // Resting contacts must not bounce, so restitution only enters
            // above a speed where a collision is what is actually happening.
            // While the pair is still apart the gap constraint owns the contact
            // and restitution stays out of it, or the body would be turned
            // around before it ever arrived; the approach speed is carried
            // forward instead and spent on the frame the two actually meet.
            const Contact& ci = contacts_[i];
            Vec3 pa = settings.angularContacts ? velocity_[ci.a] + cross(angular_[ci.a], ci.point - position_[ci.a])
                                               : velocity_[ci.a];
            Vec3 pb = settings.angularContacts ? velocity_[ci.b] + cross(angular_[ci.b], ci.point - position_[ci.b])
                                               : velocity_[ci.b];
            float vn = dot(pb - pa, ci.normal);
            float approach = std::max(-vn, foundApproach);
            // The bounce is spent on the step the pair actually meets, which is
            // not the step they overlap on: a solver that stops the body exactly
            // at the surface never produces an overlap to trigger it. The gap it
            // will close this step is what says they meet.
            const bool meets = ci.depth >= 0.0f || -ci.depth <= std::max(approach, 0.0f) * stepDt_;
            if (!meets) {
                restitutionBias_[i] = 0.0f;
                approach_[i] = approach;
            } else {
                float e = approach > 1.0f ? std::min(restitution_[contacts_[i].a], restitution_[contacts_[i].b])
                                          : 0.0f;
                restitutionBias_[i] = e * approach;
                approach_[i] = 0.0f;
            }
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
            if (normalImpulse_[i] <= 0.0f && approach_[i] <= 0.0f) continue;
            uint64_t key = contactKey_[i];
            for (uint32_t slot = static_cast<uint32_t>(key) & cacheMask_;; slot = (slot + 1) & cacheMask_) {
                std::atomic_ref<uint64_t> cell(cache_[slot].key);
                uint64_t empty = 0;
                if (cell.load(std::memory_order_relaxed) == key ||
                    cell.compare_exchange_strong(empty, key, std::memory_order_relaxed)) {
                    cache_[slot].impulse = normalImpulse_[i];
                    cache_[slot].tangent = tangentImpulse_[i];
                    cache_[slot].approach = approach_[i];
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

/// How far the body reaches along one world axis. A turned box reaches further
/// than any of its half extents, and treating it as if it did not is what
/// leaves it permanently buried in the floor.
float PhysicsWorld::boundsReach(size_t i, int axis) const {
    if (kind_[i] == static_cast<uint32_t>(ColliderKind::Sphere)) return radius_[i];
    if (!rotated_[i]) return halfExtent_[i][axis];
    const Vec3* ax = &axis_[i * 3];
    return std::abs(ax[0][axis]) * halfExtent_[i].x + std::abs(ax[1][axis]) * halfExtent_[i].y +
           std::abs(ax[2][axis]) * halfExtent_[i].z;
}

void PhysicsWorld::solveBounds(size_t begin, size_t end, float invDt) {
    const float friction = settings.friction;
    for (size_t i = begin; i < end; ++i) {
        if (invMass_[i] <= 0.0f || asleep_[i]) continue;
        for (int axis = 0; axis < 3; ++axis) {
            const float r = boundsReach(i, axis);
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
                Vec3 stopped{0, 0, 0};
                for (int t = 0; t < 3; ++t) {
                    if (t == axis) continue;
                    float& vt = velocity_[i][t];
                    float mag = std::abs(vt);
                    float before = vt;
                    vt = mag <= budget ? 0.0f : vt - std::copysign(budget, vt);
                    stopped[t] = vt - before;
                }
                // The wall takes that velocity away at the body's surface, not
                // at its centre, so a ball that skids across the floor leaves
                // rolling instead of sliding to a dead stop.
                if (settings.angularContacts && invInertia_[i] > 0.0f) {
                    Vec3 arm{0, 0, 0};
                    arm[axis] = -sign * r;
                    angular_[i] += cross(arm, stopped / invMass_[i]) * invInertia_[i];
                    // A ball rolling along the floor has a stationary contact
                    // point, so sliding friction never touches it again. The
                    // same budget spent against the spin is what brings it to
                    // rest instead of letting it roll out of the scene.
                    float spinBudget = kRollingFriction * (-vn) / r;
                    float wl = length(angular_[i]);
                    angular_[i] = wl <= spinBudget ? Vec3{0, 0, 0} : angular_[i] * (1.0f - spinBudget / wl);
                }
            }
            // Being buried in a wall counts the same as being buried in
            // another body: it has to block sleep, or the body freezes there.
            deepest_[i] = std::max(deepest_[i], depth);
            float target = std::min(std::max(depth - kSlop, 0.0f) * kCorrection * invDt, kMaxSeparation);
            float& pn = pseudo_[i][axis];
            if (pn * sign < target) pn = target * sign;
        }
    }
}

// Bounce is applied once, after the substeps have finished pressing the pair
// apart. Mixed into the substep solve it fights its own output: the substep
// that answered the bounce sees the body leaving and winds the impulse it just
// applied back down to stop it.
void PhysicsWorld::applyRestitution(uint32_t begin, uint32_t end) {
    const bool angular = settings.angularContacts;
    for (uint32_t idx = begin; idx < end; ++idx) {
        const uint32_t k = colorOrder_[idx];
        if (restitutionBias_[k] <= 1e-4f || normalImpulse_[k] <= 0.0f) continue;
        const Contact& c = contacts_[k];
        float imA = invMass_[c.a], imB = invMass_[c.b];
        float imSum = imA + imB;
        if (imSum <= 0.0f) continue;
        const float iiA = angular ? invInertia_[c.a] : 0.0f;
        const float iiB = angular ? invInertia_[c.b] : 0.0f;
        const Vec3 rA = angular ? c.point - position_[c.a] : Vec3{0, 0, 0};
        const Vec3 rB = angular ? c.point - position_[c.b] : Vec3{0, 0, 0};
        Vec3 va = iiA > 0.0f ? velocity_[c.a] + cross(angular_[c.a], rA) : velocity_[c.a];
        Vec3 vb = iiB > 0.0f ? velocity_[c.b] + cross(angular_[c.b], rB) : velocity_[c.b];
        float vn = dot(vb - va, c.normal);
        if (vn > restitutionBias_[k]) continue;
        Vec3 ca = cross(rA, c.normal), cb = cross(rB, c.normal);
        float normalMass = imSum + iiA * length2(ca) + iiB * length2(cb);
        float lambda = -(vn - restitutionBias_[k]) / normalMass;
        float& total = normalImpulse_[k];
        float previous = total;
        total = std::max(previous + lambda, 0.0f);
        float applied = total - previous;
        if (applied == 0.0f) continue;
        const Vec3 j = c.normal * applied;
        if (imA > 0.0f) velocity_[c.a] -= j * imA;
        if (imB > 0.0f) velocity_[c.b] += j * imB;
        if (iiA > 0.0f) angular_[c.a] -= cross(rA, j) * iiA;
        if (iiB > 0.0f) angular_[c.b] += cross(rB, j) * iiB;
    }
}

void PhysicsWorld::resolve(float dt, JobSystem* jobs) {
    SKEIN_PROFILE("physics/solve");
    pseudo_.assign(position_.size(), Vec3{0, 0, 0});
    pseudoSpin_.assign(position_.size(), Vec3{0, 0, 0});
    deepest_.assign(position_.size(), 0.0f);
    spinDelta_.assign(position_.size(), Quat{});
    const int substeps = std::max(1, settings.solverSubsteps);
    const float h = dt / static_cast<float>(substeps);
    const float invDt = h > 1e-6f ? 1.0f / h : 0.0f;
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
    // The positional push is collected across every substep and spent once, at
    // the end of the step. Applied and cleared per substep it is a quarter of
    // the push and a stack of spheres sinks through itself over a few hundred
    // frames.
    auto applyPseudo = [&](float sdt) {
        const bool angular = settings.angularContacts;
        auto pass = [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                // A sleeper stays exactly where it was when it fell asleep;
                // nudging it is how two sleeping bodies end up merged with
                // nothing to part them.
                if (asleep_[i]) continue;
                position_[i] += pseudo_[i] * sdt;
                if (angular) {
                    float spin = length(pseudoSpin_[i]);
                    if (spin > 1e-8f)
                        orientation_[i] = normalize(
                            Quat::axisAngle(pseudoSpin_[i] / spin, spin * sdt) * orientation_[i]);
                }
                pseudo_[i] = Vec3{0, 0, 0};
                pseudoSpin_[i] = Vec3{0, 0, 0};
            }
        };
        if (jobs && position_.size() >= 8192)
            jobs->parallelFor(position_.size(), 4096, pass);
        else
            pass(0, position_.size());
    };
    for (int sub = 0; sub < substeps; ++sub) {
      integrateVelocities(h, jobs);
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
      integratePositions(h, jobs);
    }

    for (uint32_t color = 0; color <= colorCount_; ++color) {
        uint32_t slot = color == colorCount_ ? SERIAL_COLOR : color;
        applyRestitution(colorStart_[slot], colorStart_[slot + 1]);
    }

    applyPseudo(dt);
    if (settings.warmStart) storeCachedImpulses(jobs);

    if (!settings.useBounds) return;
    auto clampToBounds = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            if (invMass_[i] <= 0.0f) continue;
            for (int axis = 0; axis < 3; ++axis) {
                const float r = boundsReach(i, axis);
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
            const bool separating = deepest_[i] > kSleepDepth;
            // Spin has to count, or a body left rotating in place is declared
            // asleep and then frozen mid-turn.
            const float spin2 = settings.angularContacts ? length2(angular_[i]) * reach_[i] * reach_[i] : 0.0f;
            if (length2(velocity_[i]) > threshold || spin2 > threshold || separating) {
                sleepTimer_[i] = 0.0f;
                asleep_[i] = 0;
            } else {
                sleepTimer_[i] += dt;
                if (sleepTimer_[i] >= settings.sleepTime) {
                    asleep_[i] = 1;
                    velocity_[i] = Vec3{0, 0, 0};
                    angular_[i] = Vec3{0, 0, 0};
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
            if (Transform* t = transforms.tryGet(entity_[i])) {
                t->position = position_[i];
                if (settings.angularContacts) t->rotation = orientation_[i];
            }
            if (Velocity* v = velocities.tryGet(entity_[i])) {
                v->linear = velocity_[i];
                if (settings.angularContacts) v->angular = angular_[i];
            }
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
    ++frame_;
    gather(scene);
    stats_.bodies = static_cast<uint32_t>(position_.size());
    if (position_.empty()) {
        stats_.contacts = 0;
        stats_.pairsTested = 0;
        stats_.nearPairs = 0;
        stats_.duplicatePairs = 0;
        stats_.colors = 0;
        stats_.serialContacts = 0;
        stats_.awake = 0;
        return stats_;
    }
    // A body that crosses more than the thinnest collider in the world in one
    // step can end up on the far side before anything is tested, so the step is
    // split until nothing moves that far. The count is global because the grid
    // and the contact list are: substepping one body alone would mean rebuilding
    // both for a single pair. ponytail: one bullet therefore costs everyone;
    // a per-island split is the upgrade if a scene mixes the two.
    uint32_t substeps = 1;
    if (settings.maxSubsteps > 1 && minThin_ > 0.0f) {
        // A pair is still caught as long as the mover ends the step inside the
        // other body's extent, so the bound is the sum of the two thinnest
        // half-extents in the world, not one of them. Speculative contacts
        // raise that bound to a whole grid cell, because a body inflated by its
        // own motion cannot pass anything without sharing a cell with it first.
        float bound = 2.0f * minThin_;
        if (settings.speculativeContacts) bound = std::max(bound, cell_);
        float motion = std::sqrt(maxSpeed2_) * dt;
        substeps = static_cast<uint32_t>(std::ceil(motion / bound));
        substeps = std::clamp(substeps, 1u, static_cast<uint32_t>(settings.maxSubsteps));
    }
    stats_.substeps = substeps;
    const float h = dt / static_cast<float>(substeps);
    stepDt_ = h;
    for (uint32_t s = 0; s < substeps; ++s) {
        buildGrid(jobs);
        findContacts(jobs);
        colorContacts();
        resolve(h, jobs);
    }
    if (settings.allowSleep) updateSleep(dt, jobs);
    else stats_.awake = static_cast<uint32_t>(position_.size());
    scatter(scene, jobs);
    return stats_;
}

namespace {

/// Nearest positive intersection of a ray with a sphere, or -1.
float raySphere(const Vec3& origin, const Vec3& dir, const Vec3& center, float radius) {
    Vec3 m = origin - center;
    float b = dot(m, dir);
    float c = length2(m) - radius * radius;
    if (c > 0.0f && b > 0.0f) return -1.0f;
    float disc = b * b - c;
    if (disc < 0.0f) return -1.0f;
    float t = -b - std::sqrt(disc);
    return t < 0.0f ? 0.0f : t;
}

/// Slab test. Fills `normal` with the face the ray enters through.
float rayBox(const Vec3& origin, const Vec3& dir, const Vec3& center, const Vec3& half, Vec3& normal) {
    float tMin = 0.0f, tMax = std::numeric_limits<float>::max();
    int axis = 0;
    float sign = 1.0f;
    for (int i = 0; i < 3; ++i) {
        float o = origin[i] - center[i];
        if (std::abs(dir[i]) < 1e-8f) {
            if (std::abs(o) > half[i]) return -1.0f;
            continue;
        }
        float inv = 1.0f / dir[i];
        float t1 = (-half[i] - o) * inv;
        float t2 = (half[i] - o) * inv;
        float enterSign = -1.0f;
        if (t1 > t2) {
            std::swap(t1, t2);
            enterSign = 1.0f;
        }
        if (t1 > tMin) {
            tMin = t1;
            axis = i;
            sign = enterSign;
        }
        tMax = std::min(tMax, t2);
        if (tMin > tMax) return -1.0f;
    }
    normal = Vec3{0, 0, 0};
    normal[axis] = sign;
    return tMin;
}

}  // namespace

RayHit PhysicsWorld::raycast(const Vec3& origin, const Vec3& dir, float maxDistance) const {
    RayHit hit;
    float len = length(dir);
    if (entries_.empty() || len < 1e-8f || maxDistance <= 0.0f) return hit;
    const Vec3 d = dir / len;

    // The ray walks cells in order, so the first hit found in a cell is final
    // once the ray has left that cell — but not before: a body registered in
    // several cells can be entered from a cell it only touches the edge of.
    const float inv = 1.0f / cell_;
    int32_t cx = static_cast<int32_t>(std::floor(origin.x * inv));
    int32_t cy = static_cast<int32_t>(std::floor(origin.y * inv));
    int32_t cz = static_cast<int32_t>(std::floor(origin.z * inv));
    int32_t stepX = d.x > 0 ? 1 : -1, stepY = d.y > 0 ? 1 : -1, stepZ = d.z > 0 ? 1 : -1;

    auto boundary = [&](float o, int32_t c, float dc, int32_t s) {
        if (std::abs(dc) < 1e-8f) return std::numeric_limits<float>::max();
        float edge = static_cast<float>(s > 0 ? c + 1 : c) * cell_;
        return (edge - o) / dc;
    };
    float tMaxX = boundary(origin.x, cx, d.x, stepX);
    float tMaxY = boundary(origin.y, cy, d.y, stepY);
    float tMaxZ = boundary(origin.z, cz, d.z, stepZ);
    const float tDeltaX = std::abs(d.x) < 1e-8f ? std::numeric_limits<float>::max() : cell_ / std::abs(d.x);
    const float tDeltaY = std::abs(d.y) < 1e-8f ? std::numeric_limits<float>::max() : cell_ / std::abs(d.y);
    const float tDeltaZ = std::abs(d.z) < 1e-8f ? std::numeric_limits<float>::max() : cell_ / std::abs(d.z);

    float best = maxDistance;
    float travelled = 0.0f;
    while (travelled <= maxDistance) {
        const uint64_t key = packCell(cx, cy, cz);
        const uint32_t bucket = hashCell(cx, cy, cz) & bucketMask_;
        for (uint32_t i = cellStart_[bucket]; i < cellStart_[bucket + 1]; ++i) {
            const GridEntry& e = entries_[i];
            if (e.cell != key) continue;
            const uint32_t b = e.body;
            float t;
            Vec3 normal;
            if (kind_[b] == static_cast<uint32_t>(ColliderKind::Sphere)) {
                t = raySphere(origin, d, position_[b], radius_[b]);
                if (t >= 0.0f) normal = normalize(origin + d * t - position_[b]);
            } else if (rotated_[b]) {
                // The ray meets a turned box in the box's own frame, where it
                // is the same slab test; only the face normal comes back out.
                const Quat inv = conjugate(orientation_[b]);
                t = rayBox(rotate(inv, origin - position_[b]), rotate(inv, d), Vec3{0, 0, 0}, halfExtent_[b], normal);
                if (t >= 0.0f) normal = rotate(orientation_[b], normal);
            } else {
                t = rayBox(origin, d, position_[b], halfExtent_[b], normal);
            }
            if (t < 0.0f || t > best) continue;
            best = t;
            hit.hit = true;
            hit.entity = entity_[b];
            hit.distance = t;
            hit.point = origin + d * t;
            hit.normal = normal;
        }
        float exit = std::min(tMaxX, std::min(tMaxY, tMaxZ));
        if (hit.hit && best <= exit) break;
        travelled = exit;
        if (travelled > maxDistance) break;
        if (tMaxX < tMaxY && tMaxX < tMaxZ) {
            cx += stepX;
            tMaxX += tDeltaX;
        } else if (tMaxY < tMaxZ) {
            cy += stepY;
            tMaxY += tDeltaY;
        } else {
            cz += stepZ;
            tMaxZ += tDeltaZ;
        }
    }
    return hit;
}

void PhysicsWorld::overlapSphere(const Vec3& center, float radius, std::vector<Entity>& out) const {
    if (entries_.empty() || radius <= 0.0f) return;
    const float inv = 1.0f / cell_;
    int32_t lo[3], hi[3];
    for (int axis = 0; axis < 3; ++axis) {
        lo[axis] = static_cast<int32_t>(std::floor((center[axis] - radius) * inv));
        hi[axis] = static_cast<int32_t>(std::floor((center[axis] + radius) * inv));
    }
    // A body spanning several cells appears once per cell it covers, so the
    // same entity can be met more than once; the run is short and the check is
    // a scan of what was just appended rather than a set.
    const size_t first = out.size();
    for (int32_t z = lo[2]; z <= hi[2]; ++z)
        for (int32_t y = lo[1]; y <= hi[1]; ++y)
            for (int32_t x = lo[0]; x <= hi[0]; ++x) {
                const uint64_t key = packCell(x, y, z);
                const uint32_t bucket = hashCell(x, y, z) & bucketMask_;
                for (uint32_t i = cellStart_[bucket]; i < cellStart_[bucket + 1]; ++i) {
                    const GridEntry& e = entries_[i];
                    if (e.cell != key) continue;
                    const uint32_t b = e.body;
                    bool overlaps;
                    if (kind_[b] == static_cast<uint32_t>(ColliderKind::Sphere)) {
                        float sum = radius + radius_[b];
                        overlaps = length2(position_[b] - center) < sum * sum;
                    } else {
                        Vec3 d = center - position_[b];
                        if (rotated_[b]) d = rotate(conjugate(orientation_[b]), d);
                        Vec3 clamped{std::clamp(d.x, -halfExtent_[b].x, halfExtent_[b].x),
                                     std::clamp(d.y, -halfExtent_[b].y, halfExtent_[b].y),
                                     std::clamp(d.z, -halfExtent_[b].z, halfExtent_[b].z)};
                        overlaps = length2(d - clamped) < radius * radius;
                    }
                    if (!overlaps) continue;
                    Entity found = entity_[b];
                    if (std::find(out.begin() + static_cast<ptrdiff_t>(first), out.end(), found) == out.end())
                        out.push_back(found);
                }
            }
}

size_t PhysicsWorld::bytesUsed() const {
    auto bytes = [](const auto& v) { return v.capacity() * sizeof(typename std::decay_t<decltype(v)>::value_type); };
    size_t total = bytes(pseudoSpin_) + bytes(axis_) + bytes(rotated_) + bytes(entity_) + bytes(position_) + bytes(velocity_) + bytes(angular_) +
                   bytes(orientation_) + bytes(invInertia_) + bytes(tangentImpulse_) + bytes(approach_) +
                   bytes(sweep_) + bytes(pseudo_) + bytes(halfExtent_) +
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
