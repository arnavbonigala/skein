#pragma once
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/math.hpp"

namespace skein {

class JobSystem;

struct Vertex {
    Vec3 position;
    Vec3 normal;
    Vec2 uv;
};

struct MeshData {
    std::string name;
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    AABB bounds;

    void recomputeBounds();
    /// Area-weighted vertex normals; used when a source file supplies none.
    void recomputeNormals();
    size_t bytesUsed() const {
        return vertices.capacity() * sizeof(Vertex) + indices.capacity() * sizeof(uint32_t);
    }
};

struct Material {
    Vec3 albedo{0.8f, 0.8f, 0.8f};
    float roughness = 0.5f;
    Vec3 emissive{0, 0, 0};
    float metallic = 0.0f;
    std::string name;
};

namespace primitives {
MeshData cube(float size = 1.0f);
MeshData sphere(float radius = 0.5f, int segments = 24, int rings = 16);
MeshData plane(float size = 1.0f, int subdivisions = 1);
MeshData cone(float radius = 0.5f, float height = 1.0f, int segments = 20);
}  // namespace primitives

/// Parses a Wavefront OBJ into a single mesh, merging groups and welding
/// duplicate position/normal/uv triplets. Returns false and fills `error`
/// on malformed input.
bool loadObj(const std::string& path, MeshData& out, std::string& error);
bool parseObj(const std::string& text, MeshData& out, std::string& error);

/// Central store of meshes and materials, addressed by dense integer handles
/// so component data stays small and cache friendly.
class Assets {
public:
    uint32_t addMesh(MeshData mesh);
    uint32_t addMaterial(Material material);

    const MeshData& mesh(uint32_t id) const { return meshes_[id]; }
    const Material& material(uint32_t id) const { return materials_[id]; }
    Material& materialMut(uint32_t id) { return materials_[id]; }
    size_t meshCount() const { return meshes_.size(); }
    size_t materialCount() const { return materials_.size(); }

    /// Returns the existing id when a mesh or material of that name is present.
    uint32_t findMesh(const std::string& name) const;
    uint32_t findMaterial(const std::string& name) const;

    /// Parses OBJ files concurrently and registers them in deterministic order.
    /// Returns the ids of successfully loaded meshes; failures are appended to `errors`.
    std::vector<uint32_t> loadObjBatch(const std::vector<std::string>& paths, JobSystem* jobs,
                                       std::vector<std::string>* errors = nullptr);

    size_t bytesUsed() const;

private:
    std::vector<MeshData> meshes_;
    std::vector<Material> materials_;
    std::unordered_map<std::string, uint32_t> meshIndex_;
    std::unordered_map<std::string, uint32_t> materialIndex_;
};

}  // namespace skein
