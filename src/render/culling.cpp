#include "render/render_list.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>

#include "core/jobs.hpp"
#include "core/profiler.hpp"
#include "scene/scene.hpp"

namespace skein {
namespace {
constexpr size_t CULL_GRAIN = 2048;
constexpr uint32_t MAX_SORT_BUCKETS = 1u << 16;

uint32_t spreadBits(uint32_t v) {
    v &= 0x3FF;
    v = (v | (v << 16)) & 0x030000FF;
    v = (v | (v << 8)) & 0x0300F00F;
    v = (v | (v << 4)) & 0x030C30C3;
    v = (v | (v << 2)) & 0x09249249;
    return v;
}

uint32_t morton(const Vec3& p, const Vec3& origin, float inv) {
    auto axis = [&](float v, float o) {
        float t = (v - o) * inv;
        return static_cast<uint32_t>(std::clamp(t, 0.0f, 1023.0f));
    };
    return (spreadBits(axis(p.x, origin.x)) << 2) | (spreadBits(axis(p.y, origin.y)) << 1) |
           spreadBits(axis(p.z, origin.z));
}
}  // namespace

void CullSystem::sortSpatially(Scene& scene) {
    SKEIN_PROFILE("render/spatialSort");
    Pool<Renderable>& renderables = scene.world.pool<Renderable>();
    Pool<CullBounds>& bounds = scene.world.pool<CullBounds>();
    const size_t n = renderables.dense.size();
    if (n < CLUSTER_SIZE || bounds.dense.size() != n) return;

    Vec3 lo{1e30f, 1e30f, 1e30f};
    Vec3 hi{-1e30f, -1e30f, -1e30f};
    for (size_t i = 0; i < n; ++i) {
        const CullBounds* cb = bounds.tryGet(renderables.dense[i]);
        if (!cb) return;
        lo = vmin(lo, cb->center);
        hi = vmax(hi, cb->center);
    }
    float span = maxComponent(hi - lo);
    float inv = span > 1e-6f ? 1023.0f / span : 0.0f;

    sortKeys_.resize(n);
    sortOrder_.resize(n);
    for (size_t i = 0; i < n; ++i) {
        sortKeys_[i] = morton(bounds.tryGet(renderables.dense[i])->center, lo, inv);
        sortOrder_[i] = static_cast<uint32_t>(i);
    }
    std::sort(sortOrder_.begin(), sortOrder_.end(),
              [&](uint32_t a, uint32_t b) { return sortKeys_[a] < sortKeys_[b]; });

    renderables.permute(sortOrder_, denseScratch_, renderableScratch_);

    // The bounds pool is permuted to the same entity order, which lets the cull
    // loop read both pools by index instead of going through the sparse map.
    std::vector<uint32_t> boundsOrder(n);
    for (size_t i = 0; i < n; ++i) {
        uint32_t at = bounds.sparse[entityIndex(renderables.dense[i])];
        boundsOrder[i] = at;
    }
    bounds.permute(boundsOrder, denseScratch_, boundsScratch_);

    // World transforms cover every entity, not just the drawn ones, so the
    // renderables are hoisted to the front in the same order and everything
    // else keeps its relative position. The instance fill then reads them
    // without going through the sparse map either.
    Pool<WorldTransform>& worlds = scene.world.pool<WorldTransform>();
    if (worlds.dense.size() >= n) {
        std::vector<uint8_t> taken(worlds.dense.size(), 0);
        std::vector<uint32_t> worldOrder;
        worldOrder.reserve(worlds.dense.size());
        bool complete = true;
        for (size_t i = 0; i < n; ++i) {
            uint32_t idx = entityIndex(renderables.dense[i]);
            if (idx >= worlds.sparse.size() || worlds.sparse[idx] == POOL_TOMBSTONE) {
                complete = false;
                break;
            }
            uint32_t at = worlds.sparse[idx];
            taken[at] = 1;
            worldOrder.push_back(at);
        }
        if (complete) {
            for (uint32_t i = 0; i < worlds.dense.size(); ++i)
                if (!taken[i]) worldOrder.push_back(i);
            worlds.permute(worldOrder, denseScratch_, worldScratch_);
        }
    }
    buildClusters(scene, nullptr);
    baseline_ = measureSpread();
    spread_ = baseline_;
    ++stats_.sorts;
}

void CullSystem::buildClusters(Scene& scene, JobSystem* jobs) {
    SKEIN_PROFILE("render/clusterBounds");
    Pool<CullBounds>& bounds = scene.world.pool<CullBounds>();
    Pool<Renderable>& renderables = scene.world.pool<Renderable>();
    const size_t n = renderables.dense.size();
    const size_t subCount = (n + SUBCLUSTER_SIZE - 1) / SUBCLUSTER_SIZE;
    const size_t count = (n + CLUSTER_SIZE - 1) / CLUSTER_SIZE;
    subs_.resize(subCount);
    clusters_.resize(count);

    const bool aligned = bounds.dense.size() == n;
    auto body = [&](size_t begin, size_t end) {
        for (size_t c = begin; c < end; ++c) {
            size_t firstSub = c * (CLUSTER_SIZE / SUBCLUSTER_SIZE);
            size_t lastSub = std::min(firstSub + CLUSTER_SIZE / SUBCLUSTER_SIZE, subCount);
            Vec3 clo{1e30f, 1e30f, 1e30f};
            Vec3 chi{-1e30f, -1e30f, -1e30f};
            for (size_t sc = firstSub; sc < lastSub; ++sc) {
                size_t from = sc * SUBCLUSTER_SIZE;
                size_t to = std::min(from + SUBCLUSTER_SIZE, n);
                Vec3 lo{1e30f, 1e30f, 1e30f};
                Vec3 hi{-1e30f, -1e30f, -1e30f};
                for (size_t i = from; i < to; ++i) {
                    Entity e = renderables.dense[i];
                    const CullBounds* cb = (aligned && bounds.dense[i] == e) ? &bounds.data[i] : bounds.tryGet(e);
                    if (!cb) continue;
                    lo = vmin(lo, cb->center - cb->extent);
                    hi = vmax(hi, cb->center + cb->extent);
                }
                subs_[sc].center = (lo + hi) * 0.5f;
                subs_[sc].extent = (hi - lo) * 0.5f;
                clo = vmin(clo, lo);
                chi = vmax(chi, hi);
            }
            clusters_[c].center = (clo + chi) * 0.5f;
            clusters_[c].extent = (chi - clo) * 0.5f;
        }
    };
    if (jobs && count >= 32)
        jobs->parallelFor(count, 16, body);
    else
        body(0, count);
}

float CullSystem::measureSpread() const {
    if (clusters_.empty()) return 0.0f;
    double total = 0;
    for (const ClusterBounds& c : clusters_) total += length(c.extent);
    return static_cast<float>(total / static_cast<double>(clusters_.size()));
}

void CullSystem::maintain(Scene& scene) {
    if (!useClusters) return;
    Pool<CullBounds>& bounds = scene.world.pool<CullBounds>();
    if (clusters_.empty() || bounds.data.empty() || baseline_ <= 0.0f) {
        sortSpatially(scene);
        return;
    }
    buildClusters(scene, nullptr);
    spread_ = measureSpread();
    // Clusters only loosen as their members drift apart, so the mean cluster
    // size straight after a sort is the right baseline. Scene density varies
    // far too much for an absolute threshold to mean anything.
    if (spread_ > baseline_ * resortThreshold) sortSpatially(scene);
}

void CullSystem::build(Scene& scene, const Frustum& frustum, uint32_t materialCount, RenderList& out,
                       JobSystem* jobs, bool applyFrustum) {
    SKEIN_PROFILE("render/cull");
    out.clear();

    Pool<Renderable>& renderables = scene.world.pool<Renderable>();
    Pool<CullBounds>& bounds = scene.world.pool<CullBounds>();
    Pool<WorldTransform>& worlds = scene.world.pool<WorldTransform>();

    const size_t n = renderables.dense.size();
    out.totalCandidates = static_cast<uint32_t>(n);
    if (n == 0) return;

    size_t chunkCount = (n + CULL_GRAIN - 1) / CULL_GRAIN;
    chunks_.resize(chunkCount);
    for (auto& c : chunks_) c.clear();

    const bool clustered = useClusters && applyFrustum && n >= CLUSTER_SIZE * 4;
    if (clustered) buildClusters(scene, jobs);

    const bool aligned = bounds.dense.size() == n;
    std::atomic<uint32_t> outsideCount{0}, insideCount{0}, straddleCount{0}, testedCount{0};
    std::atomic<uint32_t> subOutsideCount{0}, subInsideCount{0}, subStraddleCount{0};

    auto cullRange = [&](size_t begin, size_t end) {
        std::vector<uint32_t>& sink = chunks_[begin / CULL_GRAIN];
        uint32_t localOutside = 0, localInside = 0, localStraddle = 0, localTested = 0;
        uint32_t localSubOutside = 0, localSubInside = 0, localSubStraddle = 0;

        auto emit = [&](size_t from, size_t to, bool needTest) {
            for (size_t i = from; i < to; ++i) {
                if (!renderables.data[i].visible) continue;
                if (needTest) {
                    Entity e = renderables.dense[i];
                    const CullBounds* cb = (aligned && bounds.dense[i] == e) ? &bounds.data[i] : bounds.tryGet(e);
                    if (cb) {
                        ++localTested;
                        if (!frustumIntersectsSphere(frustum, cb->center, cb->radius)) continue;
                        if (!frustumIntersectsAABB(frustum, cb->center, cb->extent)) continue;
                    }
                }
                sink.push_back(static_cast<uint32_t>(i));
            }
        };

        if (!clustered) {
            emit(begin, end, applyFrustum);
        } else {
            for (size_t base = begin; base < end; base += CLUSTER_SIZE) {
                const size_t stop = std::min(base + CLUSTER_SIZE, end);
                FrustumFit fit = frustumClassifyAABB(frustum, clusters_[base / CLUSTER_SIZE].center,
                                                    clusters_[base / CLUSTER_SIZE].extent);
                if (fit == FrustumFit::Outside) {
                    ++localOutside;
                    continue;
                }
                if (fit == FrustumFit::Inside) {
                    ++localInside;
                    emit(base, stop, false);
                    continue;
                }
                ++localStraddle;
                // A straddling cluster still holds spatially coherent runs, so
                // its sub-clusters get their own verdict before anything falls
                // back to per-object tests.
                for (size_t from = base; from < stop; from += SUBCLUSTER_SIZE) {
                    const size_t to = std::min(from + SUBCLUSTER_SIZE, stop);
                    const ClusterBounds& sub = subs_[from / SUBCLUSTER_SIZE];
                    FrustumFit sfit = frustumClassifyAABB(frustum, sub.center, sub.extent);
                    if (sfit == FrustumFit::Outside) {
                        ++localSubOutside;
                        continue;
                    }
                    if (sfit == FrustumFit::Inside) {
                        ++localSubInside;
                        emit(from, to, false);
                    } else {
                        ++localSubStraddle;
                        emit(from, to, true);
                    }
                }
            }
        }
        subOutsideCount.fetch_add(localSubOutside, std::memory_order_relaxed);
        subInsideCount.fetch_add(localSubInside, std::memory_order_relaxed);
        subStraddleCount.fetch_add(localSubStraddle, std::memory_order_relaxed);
        outsideCount.fetch_add(localOutside, std::memory_order_relaxed);
        insideCount.fetch_add(localInside, std::memory_order_relaxed);
        straddleCount.fetch_add(localStraddle, std::memory_order_relaxed);
        testedCount.fetch_add(localTested, std::memory_order_relaxed);
    };

    if (jobs && chunkCount > 1)
        jobs->parallelFor(n, CULL_GRAIN, cullRange);
    else
        cullRange(0, n);

    stats_.clusters = clustered ? static_cast<uint32_t>(clusters_.size()) : 0;
    stats_.clustersOutside = outsideCount.load();
    stats_.clustersInside = insideCount.load();
    stats_.clustersStraddling = straddleCount.load();
    stats_.subclusters = clustered ? static_cast<uint32_t>(subs_.size()) : 0;
    stats_.subclustersOutside = subOutsideCount.load();
    stats_.subclustersInside = subInsideCount.load();
    stats_.subclustersStraddling = subStraddleCount.load();
    stats_.objectsTested = testedCount.load();

    visible_.clear();
    for (const auto& c : chunks_) visible_.insert(visible_.end(), c.begin(), c.end());
    out.visible = static_cast<uint32_t>(visible_.size());
    if (visible_.empty()) return;

    const uint32_t mats = std::max(materialCount, 1u);
    uint32_t maxMesh = 0;
    keys_.resize(visible_.size());
    for (size_t i = 0; i < visible_.size(); ++i) maxMesh = std::max(maxMesh, renderables.data[visible_[i]].mesh);
    const uint64_t bucketNeed = (static_cast<uint64_t>(maxMesh) + 1) * mats;

    if (bucketNeed <= MAX_SORT_BUCKETS) {
        SKEIN_PROFILE("render/batchSort");
        counts_.assign(static_cast<size_t>(bucketNeed) + 1, 0);
        for (size_t i = 0; i < visible_.size(); ++i) {
            const Renderable& r = renderables.data[visible_[i]];
            uint32_t key = r.mesh * mats + std::min(r.material, mats - 1);
            keys_[i] = key;
            ++counts_[key + 1];
        }
        for (size_t i = 1; i < counts_.size(); ++i) counts_[i] += counts_[i - 1];
        order_.resize(visible_.size());
        std::vector<uint32_t> cursor(counts_.begin(), counts_.end() - 1);
        for (size_t i = 0; i < visible_.size(); ++i) order_[cursor[keys_[i]]++] = static_cast<uint32_t>(i);
    } else {
        SKEIN_PROFILE("render/batchSort");
        order_.resize(visible_.size());
        for (size_t i = 0; i < visible_.size(); ++i) {
            const Renderable& r = renderables.data[visible_[i]];
            keys_[i] = r.mesh * mats + std::min(r.material, mats - 1);
            order_[i] = static_cast<uint32_t>(i);
        }
        std::sort(order_.begin(), order_.end(), [&](uint32_t a, uint32_t b) { return keys_[a] < keys_[b]; });
    }

    const bool worldsAligned = worlds.dense.size() >= n;
    out.instances.resize(visible_.size());
    auto fill = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            uint32_t slot = visible_[order_[i]];
            Entity e = renderables.dense[slot];
            if (worldsAligned && worlds.dense[slot] == e) {
                out.instances[i] = worlds.data[slot].matrix;
                continue;
            }
            const WorldTransform* wt = worlds.tryGet(e);
            out.instances[i] = wt ? wt->matrix : Mat4::identity();
        }
    };
    if (jobs && visible_.size() >= CULL_GRAIN * 2)
        jobs->parallelFor(visible_.size(), CULL_GRAIN, fill);
    else
        fill(0, visible_.size());

    uint32_t runStart = 0;
    uint32_t runKey = keys_[order_[0]];
    for (uint32_t i = 1; i <= static_cast<uint32_t>(order_.size()); ++i) {
        uint32_t key = i < order_.size() ? keys_[order_[i]] : ~0u;
        if (key == runKey) continue;
        const Renderable& r = renderables.data[visible_[order_[runStart]]];
        out.batches.push_back(DrawBatch{r.mesh, std::min(r.material, mats - 1), runStart, i - runStart});
        runStart = i;
        runKey = key;
    }
}

size_t CullSystem::bytesUsed() const {
    size_t total = visible_.capacity() * sizeof(uint32_t) + keys_.capacity() * sizeof(uint32_t) +
                   counts_.capacity() * sizeof(uint32_t) + order_.capacity() * sizeof(uint32_t) +
                   clusters_.capacity() * sizeof(ClusterBounds) + sortKeys_.capacity() * sizeof(uint32_t) +
                   sortOrder_.capacity() * sizeof(uint32_t) + denseScratch_.capacity() * sizeof(Entity) +
                   renderableScratch_.capacity() * sizeof(Renderable) +
                   boundsScratch_.capacity() * sizeof(CullBounds) +
                   worldScratch_.capacity() * sizeof(WorldTransform) +
                   subs_.capacity() * sizeof(ClusterBounds);
    for (const auto& c : chunks_) total += c.capacity() * sizeof(uint32_t);
    return total;
}

}  // namespace skein
