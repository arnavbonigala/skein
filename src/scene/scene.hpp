#pragma once
#include <vector>

#include "core/jobs.hpp"
#include "ecs/world.hpp"
#include "scene/components.hpp"

namespace skein {

/// Owns the entity world and the transform hierarchy. Entities are kept in a
/// depth-sorted order so world matrices resolve in flat passes, one level at a
/// time, with no recursion and no per-node pointer chasing.
class Scene {
public:
    Scene();

    World world;

    Entity create(const Transform& local = {}, Entity parent = NULL_ENTITY);
    void destroy(Entity e);
    void setParent(Entity child, Entity parent);
    Entity parentOf(Entity e) const;

    /// Recomputes every world matrix and refreshes world-space cull bounds.
    /// Passing a job system runs each hierarchy level in parallel.
    void updateTransforms(JobSystem* jobs = nullptr);

    /// Depth-sorted entity order; index 0..levelOffsets[1] are roots.
    const std::vector<Entity>& order() const { return order_; }
    const std::vector<uint32_t>& levelOffsets() const { return levelOffsets_; }
    size_t depthLevels() const { return levelOffsets_.empty() ? 0 : levelOffsets_.size() - 1; }

    void markHierarchyDirty() { hierarchyDirty_ = true; }
    bool hierarchyDirty() const { return hierarchyDirty_; }

    Mat4 worldMatrix(Entity e) const;

private:
    void rebuildOrder();

    std::vector<Entity> order_;
    std::vector<uint32_t> levelOffsets_;
    std::vector<uint32_t> depthScratch_;
    bool hierarchyDirty_ = true;
};

}  // namespace skein
