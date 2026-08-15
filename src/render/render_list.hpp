#pragma once
#include <cstdint>
#include <vector>

#include "core/math.hpp"

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

/// Frustum culls every renderable and groups survivors into instanced batches.
/// The visibility pass is chunked across the job system; the batching pass is a
/// counting sort over the (mesh, material) key so it stays linear.
class CullSystem {
public:
    /// `applyFrustum` exists so the benchmark can measure the same batching
    /// path with culling switched off.
    void build(Scene& scene, const Frustum& frustum, uint32_t materialCount, RenderList& out,
               JobSystem* jobs = nullptr, bool applyFrustum = true);

    size_t bytesUsed() const;

private:
    std::vector<std::vector<uint32_t>> chunks_;
    std::vector<uint32_t> visible_;
    std::vector<uint32_t> keys_;
    std::vector<uint32_t> counts_;
    std::vector<uint32_t> order_;
};

}  // namespace skein
