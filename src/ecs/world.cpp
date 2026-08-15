#include "ecs/world.hpp"

#include <algorithm>

namespace skein {

Entity World::create() {
    uint32_t index;
    if (!freeList_.empty()) {
        index = freeList_.back();
        freeList_.pop_back();
    } else {
        index = static_cast<uint32_t>(generations_.size());
        generations_.push_back(0);
        occupied_.push_back(0);
    }
    occupied_[index] = 1;
    ++liveCount_;
    return makeEntity(index, generations_[index]);
}

void World::destroy(Entity e) {
    if (!alive(e)) return;
    uint32_t index = entityIndex(e);
    for (auto& p : pools_)
        if (p) p->remove(e);
    ++generations_[index];
    occupied_[index] = 0;
    freeList_.push_back(index);
    --liveCount_;
}

bool World::alive(Entity e) const {
    uint32_t index = entityIndex(e);
    return index < generations_.size() && occupied_[index] && generations_[index] == entityGeneration(e);
}

void World::reserve(size_t n) {
    generations_.reserve(n);
    occupied_.reserve(n);
}

void World::restoreEntities(const std::vector<Entity>& entities) {
    uint32_t maxIndex = 0;
    for (Entity e : entities) maxIndex = std::max(maxIndex, entityIndex(e));
    generations_.assign(entities.empty() ? 0 : maxIndex + 1, 0);
    occupied_.assign(generations_.size(), 0);
    freeList_.clear();
    liveCount_ = 0;
    for (Entity e : entities) {
        uint32_t i = entityIndex(e);
        generations_[i] = entityGeneration(e);
        occupied_[i] = 1;
        ++liveCount_;
    }
    for (uint32_t i = static_cast<uint32_t>(generations_.size()); i-- > 0;)
        if (!occupied_[i]) freeList_.push_back(i);
}

void World::clear() {
    for (auto& p : pools_)
        if (p) p->clear();
    generations_.clear();
    occupied_.clear();
    freeList_.clear();
    liveCount_ = 0;
}

IPool* World::ensurePool(ComponentId id) {
    if (pools_.size() <= id) pools_.resize(id + 1);
    if (!pools_[id]) {
        const auto* entry = ComponentRegistry::instance().get(id);
        if (!entry || !entry->create) return nullptr;
        pools_[id] = entry->create();
    }
    return pools_[id].get();
}

size_t World::bytesUsed() const {
    size_t total = generations_.capacity() * sizeof(uint32_t) + occupied_.capacity() +
                   freeList_.capacity() * sizeof(uint32_t);
    for (const auto& p : pools_)
        if (p) total += p->bytesUsed();
    return total;
}

}  // namespace skein
