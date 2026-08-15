#include "scene/scene.hpp"

#include <algorithm>

#include "core/profiler.hpp"

namespace skein {

void registerCoreComponents() {
    static bool done = false;
    if (done) return;
    done = true;
    registerComponent<Transform>("Transform");
    registerComponent<WorldTransform>("WorldTransform");
    registerComponent<Parent>("Parent");
    registerComponent<Velocity>("Velocity");
    registerComponent<Collider>("Collider");
    registerComponent<Joint>("Joint");
    registerComponent<Renderable>("Renderable");
    registerComponent<CullBounds>("CullBounds");
    registerComponent<Light>("Light");
    registerComponent<Script>("Script");
}

Scene::Scene() { registerCoreComponents(); }

Entity Scene::create(const Transform& local, Entity parent) {
    Entity e = world.create();
    world.add<Transform>(e, local);
    world.add<WorldTransform>(e, WorldTransform{composeTRS(local.position, local.rotation, local.scale)});
    if (parent != NULL_ENTITY) world.add<Parent>(e, Parent{parent});
    hierarchyDirty_ = true;
    return e;
}

void Scene::destroy(Entity e) {
    world.destroy(e);
    hierarchyDirty_ = true;
}

void Scene::setParent(Entity child, Entity parent) {
    if (parent == NULL_ENTITY)
        world.remove<Parent>(child);
    else
        world.add<Parent>(child, Parent{parent});
    hierarchyDirty_ = true;
}

Entity Scene::parentOf(Entity e) const {
    const Parent* p = world.tryGet<Parent>(e);
    return p ? p->value : NULL_ENTITY;
}

Mat4 Scene::worldMatrix(Entity e) const {
    const WorldTransform* wt = world.tryGet<WorldTransform>(e);
    return wt ? wt->matrix : Mat4::identity();
}

void Scene::rebuildOrder() {
    SKEIN_PROFILE("scene/rebuildOrder");
    const Pool<Transform>& transforms = world.pool<Transform>();
    const Pool<Parent>& parents = world.pool<Parent>();

    depthScratch_.assign(world.capacity(), 0xFFFFFFFFu);
    uint32_t maxDepth = 0;

    std::vector<uint32_t> chain;
    for (Entity e : transforms.dense) {
        uint32_t idx = entityIndex(e);
        if (depthScratch_[idx] != 0xFFFFFFFFu) continue;
        chain.clear();
        Entity cur = e;
        uint32_t depth = 0;
        while (true) {
            uint32_t curIdx = entityIndex(cur);
            if (depthScratch_[curIdx] != 0xFFFFFFFFu) {
                depth = depthScratch_[curIdx];
                break;
            }
            const Parent* p = parents.tryGet(cur);
            if (!p || p->value == NULL_ENTITY || !world.alive(p->value) ||
                !transforms.contains(p->value) || chain.size() > 4096) {
                depthScratch_[curIdx] = 0;
                depth = 0;
                break;
            }
            chain.push_back(curIdx);
            cur = p->value;
        }
        for (size_t i = chain.size(); i-- > 0;) {
            depthScratch_[chain[i]] = ++depth;
            maxDepth = std::max(maxDepth, depth);
        }
    }

    levelOffsets_.assign(maxDepth + 2, 0);
    for (Entity e : transforms.dense) ++levelOffsets_[depthScratch_[entityIndex(e)] + 1];
    for (size_t i = 1; i < levelOffsets_.size(); ++i) levelOffsets_[i] += levelOffsets_[i - 1];

    order_.resize(transforms.dense.size());
    std::vector<uint32_t> cursor(levelOffsets_.begin(), levelOffsets_.end() - 1);
    for (Entity e : transforms.dense) order_[cursor[depthScratch_[entityIndex(e)]]++] = e;

    hierarchyDirty_ = false;
}

void Scene::updateTransforms(JobSystem* jobs) {
    SKEIN_PROFILE("scene/updateTransforms");
    if (hierarchyDirty_) rebuildOrder();

    Pool<Transform>& transforms = world.pool<Transform>();
    Pool<WorldTransform>& worlds = world.pool<WorldTransform>();
    Pool<CullBounds>& bounds = world.pool<CullBounds>();

    auto processRange = [&](size_t begin, size_t end, bool roots) {
        for (size_t i = begin; i < end; ++i) {
            Entity e = order_[i];
            const Transform* t = transforms.tryGet(e);
            WorldTransform* wt = worlds.tryGet(e);
            if (!t || !wt) continue;
            Mat4 local = composeTRS(t->position, t->rotation, t->scale);
            if (roots) {
                wt->matrix = local;
            } else {
                Entity p = parentOf(e);
                const WorldTransform* pw = worlds.tryGet(p);
                wt->matrix = pw ? pw->matrix * local : local;
            }
            if (CullBounds* cb = bounds.tryGet(e)) {
                AABB local_box;
                local_box.min = cb->localCenter - cb->localExtent;
                local_box.max = cb->localCenter + cb->localExtent;
                AABB wb = transformAABB(local_box, wt->matrix);
                cb->center = wb.center();
                cb->extent = wb.extent();
                cb->radius = length(cb->extent);
            }
        }
    };

    for (size_t level = 0; level + 1 < levelOffsets_.size(); ++level) {
        size_t begin = levelOffsets_[level];
        size_t end = levelOffsets_[level + 1];
        if (begin >= end) continue;
        bool roots = level == 0;
        if (jobs && end - begin >= 512)
            jobs->parallelFor(end - begin, 1024,
                              [&](size_t a, size_t b) { processRange(begin + a, begin + b, roots); });
        else
            processRange(begin, end, roots);
    }
}

}  // namespace skein
