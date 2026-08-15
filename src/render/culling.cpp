#include "render/render_list.hpp"

#include <algorithm>

#include "core/jobs.hpp"
#include "core/profiler.hpp"
#include "scene/scene.hpp"

namespace skein {
namespace {
constexpr size_t CULL_GRAIN = 2048;
constexpr uint32_t MAX_SORT_BUCKETS = 1u << 16;
}  // namespace

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

    auto cullRange = [&](size_t begin, size_t end) {
        std::vector<uint32_t>& sink = chunks_[begin / CULL_GRAIN];
        for (size_t i = begin; i < end; ++i) {
            if (!renderables.data[i].visible) continue;
            Entity e = renderables.dense[i];
            const CullBounds* cb = applyFrustum ? bounds.tryGet(e) : nullptr;
            if (cb) {
                if (!frustumIntersectsSphere(frustum, cb->center, cb->radius)) continue;
                if (!frustumIntersectsAABB(frustum, cb->center, cb->extent)) continue;
            }
            sink.push_back(static_cast<uint32_t>(i));
        }
    };

    if (jobs && chunkCount > 1)
        jobs->parallelFor(n, CULL_GRAIN, cullRange);
    else
        cullRange(0, n);

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

    out.instances.resize(visible_.size());
    auto fill = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            Entity e = renderables.dense[visible_[order_[i]]];
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
                   counts_.capacity() * sizeof(uint32_t) + order_.capacity() * sizeof(uint32_t);
    for (const auto& c : chunks_) total += c.capacity() * sizeof(uint32_t);
    return total;
}

}  // namespace skein
