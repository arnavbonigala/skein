#include "scene/serialize.hpp"

#include <cstring>
#include <fstream>

#include "scene/scene.hpp"

namespace skein {
namespace {

void writeBytes(std::vector<uint8_t>& out, const void* src, size_t n) {
    const uint8_t* p = static_cast<const uint8_t*>(src);
    out.insert(out.end(), p, p + n);
}

template <typename T>
void writePod(std::vector<uint8_t>& out, const T& value) {
    writeBytes(out, &value, sizeof(T));
}

void writeString(std::vector<uint8_t>& out, const std::string& s) {
    writePod(out, static_cast<uint32_t>(s.size()));
    writeBytes(out, s.data(), s.size());
}

struct Reader {
    const uint8_t* data;
    size_t size;
    size_t pos = 0;

    bool bytes(void* dst, size_t n) {
        if (pos + n > size) return false;
        std::memcpy(dst, data + pos, n);
        pos += n;
        return true;
    }
    template <typename T>
    bool pod(T& value) {
        return bytes(&value, sizeof(T));
    }
    bool str(std::string& out) {
        uint32_t len = 0;
        if (!pod(len)) return false;
        if (pos + len > size) return false;
        out.assign(reinterpret_cast<const char*>(data + pos), len);
        pos += len;
        return true;
    }
    bool skip(size_t n) {
        if (pos + n > size) return false;
        pos += n;
        return true;
    }
};

}  // namespace

std::vector<uint8_t> serializeScene(const Scene& scene) {
    World& world = const_cast<Scene&>(scene).world;
    std::vector<uint8_t> out;

    std::vector<Entity> entities;
    entities.reserve(world.aliveCount());
    world.each([&](Entity e) { entities.push_back(e); });

    writePod(out, SCENE_MAGIC);
    writePod(out, SCENE_VERSION);
    writePod(out, static_cast<uint64_t>(entities.size()));
    writeBytes(out, entities.data(), entities.size() * sizeof(Entity));

    std::vector<const IPool*> saved;
    for (ComponentId id = 0; id < world.poolCount(); ++id) {
        const IPool* p = const_cast<World&>(world).poolById(id);
        if (!p || p->size() == 0) continue;
        const auto* entry = ComponentRegistry::instance().get(id);
        if (!entry) continue;
        if (entry->name == "Script") continue;
        saved.push_back(p);
    }

    writePod(out, static_cast<uint32_t>(saved.size()));
    for (const IPool* p : saved) {
        writeString(out, p->name());
        writePod(out, static_cast<uint32_t>(p->elementSize()));
        writePod(out, static_cast<uint64_t>(p->size()));
        writeBytes(out, p->rawEntities(), p->size() * sizeof(Entity));
        writeBytes(out, p->rawData(), p->size() * p->elementSize());
    }
    return out;
}

bool deserializeScene(Scene& scene, const uint8_t* data, size_t size, std::string& error) {
    registerCoreComponents();
    Reader r{data, size};

    uint32_t magic = 0, version = 0;
    if (!r.pod(magic) || !r.pod(version)) {
        error = "truncated header";
        return false;
    }
    if (magic != SCENE_MAGIC) {
        error = "bad magic";
        return false;
    }
    if (version > SCENE_VERSION) {
        error = "scene version " + std::to_string(version) + " is newer than " + std::to_string(SCENE_VERSION);
        return false;
    }

    uint64_t entityCount = 0;
    if (!r.pod(entityCount)) {
        error = "truncated entity count";
        return false;
    }
    if (entityCount > (size - r.pos) / sizeof(Entity)) {
        error = "entity table exceeds file";
        return false;
    }
    std::vector<Entity> entities(entityCount);
    if (!r.bytes(entities.data(), entities.size() * sizeof(Entity))) {
        error = "truncated entity table";
        return false;
    }

    scene.world.clear();
    scene.world.restoreEntities(entities);

    uint32_t poolCount = 0;
    if (!r.pod(poolCount)) {
        error = "truncated pool count";
        return false;
    }

    for (uint32_t i = 0; i < poolCount; ++i) {
        std::string name;
        uint32_t elementSize = 0;
        uint64_t count = 0;
        if (!r.str(name) || !r.pod(elementSize) || !r.pod(count)) {
            error = "truncated pool header";
            return false;
        }
        size_t payload = static_cast<size_t>(count) * (sizeof(Entity) + elementSize);
        if (payload > size - r.pos) {
            error = "pool '" + name + "' exceeds file";
            return false;
        }

        const auto* entry = ComponentRegistry::instance().find(name);
        if (!entry || entry->size != elementSize) {
            if (!r.skip(payload)) {
                error = "truncated pool '" + name + "'";
                return false;
            }
            continue;
        }

        std::vector<Entity> owners(static_cast<size_t>(count));
        if (!r.bytes(owners.data(), owners.size() * sizeof(Entity))) {
            error = "truncated pool entities";
            return false;
        }
        IPool* pool = scene.world.ensurePool(entry->id);
        if (!pool) {
            error = "no pool factory for '" + name + "'";
            return false;
        }
        std::vector<uint8_t> element(elementSize);
        for (uint64_t k = 0; k < count; ++k) {
            if (!r.bytes(element.data(), elementSize)) {
                error = "truncated pool data";
                return false;
            }
            if (scene.world.alive(owners[k])) pool->insertRaw(owners[k], element.data());
        }
    }

    scene.markHierarchyDirty();
    return true;
}

bool saveScene(const Scene& scene, const std::string& path, std::string& error) {
    std::vector<uint8_t> blob = serializeScene(scene);
    std::ofstream file(path, std::ios::binary);
    if (!file) {
        error = "cannot write " + path;
        return false;
    }
    file.write(reinterpret_cast<const char*>(blob.data()), static_cast<std::streamsize>(blob.size()));
    if (!file) {
        error = "write failed for " + path;
        return false;
    }
    return true;
}

bool loadScene(Scene& scene, const std::string& path, std::string& error) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        error = "cannot open " + path;
        return false;
    }
    std::streamsize size = file.tellg();
    file.seekg(0);
    std::vector<uint8_t> blob(static_cast<size_t>(size));
    if (size > 0 && !file.read(reinterpret_cast<char*>(blob.data()), size)) {
        error = "read failed for " + path;
        return false;
    }
    return deserializeScene(scene, blob.data(), blob.size(), error);
}

}  // namespace skein
