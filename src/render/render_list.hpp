#pragma once
#include <cstdint>
#include <vector>

#include "core/math.hpp"
#include "scene/components.hpp"

namespace skein {

class Scene;
class JobSystem;

/// One instanced draw: `count` consecutive matrices in RenderList::instances
/// share a mesh and a material.
struct DrawBatch {
    uint32_t mesh = 0;
    uint32_t material = 0;
    uint32_t first = 0;
    uint32_t count = 0;
};

struct RenderList {
    std::vector<DrawBatch> batches;
    std::vector<Mat4> instances;
    uint32_t totalCandidates = 0;
    uint32_t visible = 0;

    void clear() {
        batches.clear();
        instances.clear();
        totalCandidates = 0;
        visible = 0;
    }
    /// Draw calls saved by merging visible objects into instanced batches.
    uint32_t drawCalls() const { return static_cast<uint32_t>(batches.size()); }
};

struct CullStats {
    uint32_t clusters = 0;
    uint32_t clustersOutside = 0;
    uint32_t clustersInside = 0;
    uint32_t clustersStraddling = 0;
    uint32_t subclusters = 0;
    uint32_t subclustersOutside = 0;
    uint32_t subclustersInside = 0;
    uint32_t subclustersStraddling = 0;
    /// Objects that needed their own frustum test, as opposed to inheriting a
    /// verdict from their cluster.
    uint32_t objectsTested = 0;
    uint32_t sorts = 0;
};

/// Frustum culls every renderable and groups survivors into instanced batches.
/// The visibility pass is chunked across the job system; the batching pass is a
/// counting sort over the (mesh, material) key so it stays linear.
///
/// Culling works on clusters of consecutive pool entries rather than on single
/// objects. That only pays off if neighbours in the array are neighbours in
/// space, so the pools are periodically permuted into Morton order; a cluster
/// then usually falls wholly inside or wholly outside the frustum and settles
/// its members with one test instead of 128.
class CullSystem {
public:
    static constexpr uint32_t CLUSTER_SIZE = 128;
    static constexpr uint32_t SUBCLUSTER_SIZE = 16;

    /// `applyFrustum` exists so the benchmark can measure the same batching
    /// path with culling switched off.
    void build(Scene& scene, const Frustum& frustum, uint32_t materialCount, RenderList& out,
               JobSystem* jobs = nullptr, bool applyFrustum = true);

    /// Permutes the Renderable and CullBounds pools into Morton order. Cheap
    /// enough to run every few seconds and pointless to run every frame.
    void sortSpatially(Scene& scene);

    /// Objects drift out of Morton order as they move; this re-sorts once the
    /// clusters have loosened enough to stop paying for themselves.
    void maintain(Scene& scene);

    bool useClusters = true;
    float resortThreshold = 1.6f;

    const CullStats& stats() const { return stats_; }
    /// Mean cluster half-diagonal now, and what it was straight after the last
    /// sort. The ratio is how far the Morton order has decayed.
    float clusterSpread() const { return spread_; }
    float spreadBaseline() const { return baseline_; }

    size_t bytesUsed() const;

private:
    void buildClusters(Scene& scene, JobSystem* jobs);
    float measureSpread() const;

    struct ClusterBounds {
        Vec3 center;
        Vec3 extent;
    };

    std::vector<std::vector<uint32_t>> chunks_;
    std::vector<uint32_t> visible_;
    std::vector<uint32_t> keys_;
    std::vector<uint32_t> counts_;
    std::vector<uint32_t> order_;
    std::vector<ClusterBounds> clusters_;
    std::vector<ClusterBounds> subs_;
    std::vector<uint32_t> sortKeys_;
    std::vector<uint32_t> sortOrder_;
    std::vector<Entity> denseScratch_;
    std::vector<Renderable> renderableScratch_;
    std::vector<CullBounds> boundsScratch_;
    std::vector<WorldTransform> worldScratch_;
    CullStats stats_;
    float spread_ = 0.0f;
    float baseline_ = 0.0f;
};

}  // namespace skein
