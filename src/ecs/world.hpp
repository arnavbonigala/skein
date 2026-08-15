#pragma once
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <type_traits>
#include <cassert>

namespace skein {

using Entity = uint64_t;
constexpr Entity NULL_ENTITY = ~0ull;

inline uint32_t entityIndex(Entity e) { return static_cast<uint32_t>(e & 0xFFFFFFFFull); }
inline uint32_t entityGeneration(Entity e) { return static_cast<uint32_t>(e >> 32); }
inline Entity makeEntity(uint32_t index, uint32_t gen) {
    return (static_cast<Entity>(gen) << 32) | index;
}

using ComponentId = uint32_t;
constexpr ComponentId INVALID_COMPONENT = ~0u;

namespace detail {
inline ComponentId nextComponentId() {
    static ComponentId counter = 0;
    return counter++;
}
}  // namespace detail

template <typename T>
ComponentId componentId() {
    static const ComponentId id = detail::nextComponentId();
    return id;
}

/// Type-erased handle so the World can destroy and serialize pools it does not know statically.
struct IPool {
    virtual ~IPool() = default;
    virtual bool remove(Entity e) = 0;
    virtual bool contains(Entity e) const = 0;
    virtual size_t size() const = 0;
    virtual void clear() = 0;
    virtual const char* name() const = 0;
    virtual size_t elementSize() const = 0;
    virtual const void* rawData() const = 0;
    virtual const Entity* rawEntities() const = 0;
    virtual void insertRaw(Entity e, const void* value) = 0;
    virtual size_t bytesUsed() const = 0;
};

constexpr uint32_t POOL_TOMBSTONE = ~0u;

/// Sparse set: components live in a contiguous array, iteration never touches the sparse map.
template <typename T>
struct Pool final : IPool {
    std::vector<uint32_t> sparse;
    std::vector<Entity> dense;
    std::vector<T> data;
    std::string typeName;

    explicit Pool(std::string n) : typeName(std::move(n)) {}

    const char* name() const override { return typeName.c_str(); }
    size_t size() const override { return dense.size(); }
    size_t elementSize() const override { return sizeof(T); }
    const void* rawData() const override { return data.data(); }
    const Entity* rawEntities() const override { return dense.data(); }

    size_t bytesUsed() const override {
        return sparse.capacity() * sizeof(uint32_t) + dense.capacity() * sizeof(Entity) +
               data.capacity() * sizeof(T);
    }

    bool contains(Entity e) const override {
        uint32_t i = entityIndex(e);
        return i < sparse.size() && sparse[i] != POOL_TOMBSTONE && dense[sparse[i]] == e;
    }

    T* tryGet(Entity e) {
        uint32_t i = entityIndex(e);
        if (i >= sparse.size()) return nullptr;
        uint32_t d = sparse[i];
        if (d == POOL_TOMBSTONE || dense[d] != e) return nullptr;
        return &data[d];
    }
    const T* tryGet(Entity e) const { return const_cast<Pool*>(this)->tryGet(e); }

    T& insert(Entity e, T value) {
        uint32_t i = entityIndex(e);
        if (i >= sparse.size()) sparse.resize(i + 1, POOL_TOMBSTONE);
        uint32_t d = sparse[i];
        if (d != POOL_TOMBSTONE && dense[d] == e) {
            data[d] = std::move(value);
            return data[d];
        }
        sparse[i] = static_cast<uint32_t>(dense.size());
        dense.push_back(e);
        data.push_back(std::move(value));
        return data.back();
    }

    void insertRaw(Entity e, const void* value) override {
        static_assert(std::is_trivially_copyable_v<T>);
        T tmp;
        std::memcpy(&tmp, value, sizeof(T));
        insert(e, tmp);
    }

    bool remove(Entity e) override {
        uint32_t i = entityIndex(e);
        if (i >= sparse.size()) return false;
        uint32_t d = sparse[i];
        if (d == POOL_TOMBSTONE || dense[d] != e) return false;
        uint32_t last = static_cast<uint32_t>(dense.size() - 1);
        if (d != last) {
            dense[d] = dense[last];
            data[d] = std::move(data[last]);
            sparse[entityIndex(dense[d])] = d;
        }
        dense.pop_back();
        data.pop_back();
        sparse[i] = POOL_TOMBSTONE;
        return true;
    }

    void clear() override {
        sparse.clear();
        dense.clear();
        data.clear();
    }
};

/// Registry of every component type ever registered, keyed by stable name for serialization.
struct ComponentRegistry {
    struct Entry {
        std::string name;
        ComponentId id = INVALID_COMPONENT;
        size_t size = 0;
        std::unique_ptr<IPool> (*create)() = nullptr;
    };
    std::vector<Entry> entries;

    static ComponentRegistry& instance() {
        static ComponentRegistry r;
        return r;
    }

    template <typename T>
    void add(const std::string& name) {
        static_assert(std::is_trivially_copyable_v<T>, "serializable components must be POD");
        ComponentId id = componentId<T>();
        if (id < entries.size() && entries[id].id == id) return;
        if (entries.size() <= id) entries.resize(id + 1);
        entries[id] = Entry{name, id, sizeof(T), nullptr};
        entries[id].create = +[]() -> std::unique_ptr<IPool> {
            const auto* self = ComponentRegistry::instance().get(componentId<T>());
            return std::unique_ptr<IPool>(new Pool<T>(self ? self->name : std::string("unnamed")));
        };
    }

    const Entry* find(const std::string& name) const {
        for (const auto& e : entries)
            if (e.id != INVALID_COMPONENT && e.name == name) return &e;
        return nullptr;
    }
    const Entry* get(ComponentId id) const {
        if (id < entries.size() && entries[id].id == id) return &entries[id];
        return nullptr;
    }
};

template <typename T>
void registerComponent(const std::string& name) {
    ComponentRegistry::instance().add<T>(name);
}

class World {
public:
    Entity create();
    void destroy(Entity e);
    bool alive(Entity e) const;
    size_t aliveCount() const { return liveCount_; }
    size_t capacity() const { return generations_.size(); }
    void clear();

    /// Reserves entity slots up front so bulk spawning does not reallocate mid-loop.
    void reserve(size_t n);

    /// Recreates an exact set of entity ids in a cleared world so that stored
    /// entity references (parents, script targets) survive a round trip.
    void restoreEntities(const std::vector<Entity>& entities);

    template <typename T>
    Pool<T>& pool() {
        ComponentId id = componentId<T>();
        if (pools_.size() <= id) pools_.resize(id + 1);
        if (!pools_[id]) {
            const auto* entry = ComponentRegistry::instance().get(id);
            pools_[id] = std::make_unique<Pool<T>>(entry ? entry->name : std::string("unnamed"));
        }
        return *static_cast<Pool<T>*>(pools_[id].get());
    }

    template <typename T>
    const Pool<T>& pool() const {
        return const_cast<World*>(this)->pool<T>();
    }

    template <typename T>
    T& add(Entity e, T value = {}) {
        assert(alive(e));
        return pool<T>().insert(e, std::move(value));
    }

    template <typename T>
    bool remove(Entity e) {
        return pool<T>().remove(e);
    }

    template <typename T>
    T* tryGet(Entity e) {
        return pool<T>().tryGet(e);
    }

    template <typename T>
    const T* tryGet(Entity e) const {
        return pool<T>().tryGet(e);
    }

    template <typename T>
    T& get(Entity e) {
        T* p = pool<T>().tryGet(e);
        assert(p);
        return *p;
    }

    template <typename T>
    bool has(Entity e) const {
        return pool<T>().contains(e);
    }

    IPool* poolById(ComponentId id) {
        return id < pools_.size() ? pools_[id].get() : nullptr;
    }
    size_t poolCount() const { return pools_.size(); }

    /// Ensures a pool exists for a registered component id, used when loading scenes.
    IPool* ensurePool(ComponentId id);

    size_t bytesUsed() const;

    template <typename Fn>
    void each(Fn&& fn) const {
        for (uint32_t i = 0; i < generations_.size(); ++i)
            if (occupied_[i]) fn(makeEntity(i, generations_[i]));
    }

private:
    std::vector<uint32_t> generations_;
    std::vector<uint8_t> occupied_;
    std::vector<uint32_t> freeList_;
    std::vector<std::unique_ptr<IPool>> pools_;
    size_t liveCount_ = 0;
};

/// Iterates the smallest matching pool and probes the rest; the hot array stays contiguous.
template <typename Primary, typename... Rest, typename Fn>
void forEach(World& w, Fn&& fn) {
    Pool<Primary>& p = w.pool<Primary>();
    auto probe = [&](Entity e) { return (... && w.pool<Rest>().contains(e)); };
    for (size_t i = p.dense.size(); i-- > 0;) {
        Entity e = p.dense[i];
        if constexpr (sizeof...(Rest) > 0) {
            if (!probe(e)) continue;
            fn(e, p.data[i], *w.pool<Rest>().tryGet(e)...);
        } else {
            fn(e, p.data[i]);
        }
    }
}

}  // namespace skein
