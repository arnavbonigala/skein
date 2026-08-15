#include "assets/mesh.hpp"

#include <atomic>

#include "core/jobs.hpp"

namespace skein {

void MeshData::recomputeBounds() {
    bounds = AABB{};
    for (const Vertex& v : vertices) bounds.expand(v.position);
}

void MeshData::recomputeNormals() {
    for (Vertex& v : vertices) v.normal = Vec3{0, 0, 0};
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        uint32_t a = indices[i], b = indices[i + 1], c = indices[i + 2];
        Vec3 n = cross(vertices[b].position - vertices[a].position, vertices[c].position - vertices[a].position);
        vertices[a].normal += n;
        vertices[b].normal += n;
        vertices[c].normal += n;
    }
    for (Vertex& v : vertices) v.normal = normalize(v.normal);
}

namespace primitives {

MeshData cube(float size) {
    float h = size * 0.5f;
    MeshData m;
    m.name = "cube";
    const Vec3 normals[6] = {{0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}};
    const Vec3 tangents[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 0, -1}, {0, 0, 1}, {1, 0, 0}, {1, 0, 0}};
    for (int f = 0; f < 6; ++f) {
        Vec3 n = normals[f];
        Vec3 t = tangents[f];
        Vec3 b = cross(n, t);
        uint32_t base = static_cast<uint32_t>(m.vertices.size());
        const float su[4] = {-1, 1, 1, -1};
        const float sv[4] = {-1, -1, 1, 1};
        for (int i = 0; i < 4; ++i) {
            Vertex v;
            v.position = (n + t * su[i] + b * sv[i]) * h;
            v.normal = n;
            v.uv = Vec2{su[i] * 0.5f + 0.5f, sv[i] * 0.5f + 0.5f};
            m.vertices.push_back(v);
        }
        uint32_t idx[6] = {base, base + 1, base + 2, base, base + 2, base + 3};
        m.indices.insert(m.indices.end(), idx, idx + 6);
    }
    m.recomputeBounds();
    return m;
}

MeshData sphere(float radius, int segments, int rings) {
    MeshData m;
    m.name = "sphere";
    segments = std::max(segments, 3);
    rings = std::max(rings, 2);
    for (int y = 0; y <= rings; ++y) {
        float v = static_cast<float>(y) / static_cast<float>(rings);
        float phi = v * PI;
        for (int x = 0; x <= segments; ++x) {
            float u = static_cast<float>(x) / static_cast<float>(segments);
            float theta = u * 2.0f * PI;
            Vec3 n{std::sin(phi) * std::cos(theta), std::cos(phi), std::sin(phi) * std::sin(theta)};
            m.vertices.push_back(Vertex{n * radius, n, Vec2{u, 1.0f - v}});
        }
    }
    int stride = segments + 1;
    for (int y = 0; y < rings; ++y) {
        for (int x = 0; x < segments; ++x) {
            uint32_t a = static_cast<uint32_t>(y * stride + x);
            uint32_t b = static_cast<uint32_t>((y + 1) * stride + x);
            uint32_t idx[6] = {a, b, a + 1, a + 1, b, b + 1};
            m.indices.insert(m.indices.end(), idx, idx + 6);
        }
    }
    m.recomputeBounds();
    return m;
}

MeshData plane(float size, int subdivisions) {
    MeshData m;
    m.name = "plane";
    subdivisions = std::max(subdivisions, 1);
    float h = size * 0.5f;
    for (int y = 0; y <= subdivisions; ++y) {
        for (int x = 0; x <= subdivisions; ++x) {
            float u = static_cast<float>(x) / static_cast<float>(subdivisions);
            float v = static_cast<float>(y) / static_cast<float>(subdivisions);
            m.vertices.push_back(Vertex{Vec3{-h + u * size, 0, -h + v * size}, Vec3{0, 1, 0}, Vec2{u, v}});
        }
    }
    int stride = subdivisions + 1;
    for (int y = 0; y < subdivisions; ++y) {
        for (int x = 0; x < subdivisions; ++x) {
            uint32_t a = static_cast<uint32_t>(y * stride + x);
            uint32_t b = static_cast<uint32_t>((y + 1) * stride + x);
            uint32_t idx[6] = {a, a + 1, b, b, a + 1, b + 1};
            m.indices.insert(m.indices.end(), idx, idx + 6);
        }
    }
    m.recomputeBounds();
    return m;
}

MeshData cone(float radius, float height, int segments) {
    MeshData m;
    m.name = "cone";
    segments = std::max(segments, 3);
    Vec3 apex{0, height * 0.5f, 0};
    float baseY = -height * 0.5f;
    for (int i = 0; i < segments; ++i) {
        float t0 = static_cast<float>(i) / static_cast<float>(segments) * 2.0f * PI;
        float t1 = static_cast<float>(i + 1) / static_cast<float>(segments) * 2.0f * PI;
        Vec3 p0{std::cos(t0) * radius, baseY, std::sin(t0) * radius};
        Vec3 p1{std::cos(t1) * radius, baseY, std::sin(t1) * radius};
        uint32_t base = static_cast<uint32_t>(m.vertices.size());
        m.vertices.push_back(Vertex{apex, Vec3{0, 1, 0}, Vec2{0.5f, 1}});
        m.vertices.push_back(Vertex{p0, Vec3{0, 1, 0}, Vec2{0, 0}});
        m.vertices.push_back(Vertex{p1, Vec3{0, 1, 0}, Vec2{1, 0}});
        m.vertices.push_back(Vertex{Vec3{0, baseY, 0}, Vec3{0, -1, 0}, Vec2{0.5f, 0.5f}});
        m.vertices.push_back(Vertex{p1, Vec3{0, -1, 0}, Vec2{1, 0}});
        m.vertices.push_back(Vertex{p0, Vec3{0, -1, 0}, Vec2{0, 0}});
        for (uint32_t k = 0; k < 6; ++k) m.indices.push_back(base + k);
    }
    m.recomputeNormals();
    m.recomputeBounds();
    return m;
}

}  // namespace primitives

uint32_t Assets::addMesh(MeshData mesh) {
    if (!mesh.bounds.valid()) mesh.recomputeBounds();
    uint32_t id = static_cast<uint32_t>(meshes_.size());
    if (!mesh.name.empty()) meshIndex_[mesh.name] = id;
    meshes_.push_back(std::move(mesh));
    return id;
}

uint32_t Assets::addMaterial(Material material) {
    uint32_t id = static_cast<uint32_t>(materials_.size());
    if (!material.name.empty()) materialIndex_[material.name] = id;
    materials_.push_back(std::move(material));
    return id;
}

uint32_t Assets::findMesh(const std::string& name) const {
    auto it = meshIndex_.find(name);
    return it == meshIndex_.end() ? ~0u : it->second;
}

uint32_t Assets::findMaterial(const std::string& name) const {
    auto it = materialIndex_.find(name);
    return it == materialIndex_.end() ? ~0u : it->second;
}

std::vector<uint32_t> Assets::loadObjBatch(const std::vector<std::string>& paths, JobSystem* jobs,
                                           std::vector<std::string>* errors) {
    std::vector<MeshData> parsed(paths.size());
    std::vector<std::string> failures(paths.size());

    auto parseRange = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) loadObj(paths[i], parsed[i], failures[i]);
    };
    if (jobs)
        jobs->parallelFor(paths.size(), 1, parseRange);
    else
        parseRange(0, paths.size());

    std::vector<uint32_t> ids;
    ids.reserve(paths.size());
    for (size_t i = 0; i < paths.size(); ++i) {
        if (!failures[i].empty() || parsed[i].vertices.empty()) {
            if (errors) errors->push_back(paths[i] + ": " + (failures[i].empty() ? "empty mesh" : failures[i]));
            continue;
        }
        ids.push_back(addMesh(std::move(parsed[i])));
    }
    return ids;
}

size_t Assets::bytesUsed() const {
    size_t total = 0;
    for (const auto& m : meshes_) total += m.bytesUsed();
    return total + materials_.capacity() * sizeof(Material);
}

}  // namespace skein
