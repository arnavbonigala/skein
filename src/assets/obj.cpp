#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <unordered_map>

#include "assets/mesh.hpp"

namespace skein {
namespace {

struct Ref {
    int v = 0, t = 0, n = 0;
    bool operator==(const Ref& o) const { return v == o.v && t == o.t && n == o.n; }
};

struct RefHash {
    size_t operator()(const Ref& r) const {
        size_t h = static_cast<size_t>(r.v) * 73856093u;
        h ^= static_cast<size_t>(r.t) * 19349663u;
        h ^= static_cast<size_t>(r.n) * 83492791u;
        return h;
    }
};

/// Resolves an OBJ index, which is 1-based and may be negative (relative to end).
bool resolve(int raw, size_t count, size_t& out) {
    if (raw > 0) {
        if (static_cast<size_t>(raw) > count) return false;
        out = static_cast<size_t>(raw) - 1;
        return true;
    }
    if (raw < 0) {
        long idx = static_cast<long>(count) + raw;
        if (idx < 0) return false;
        out = static_cast<size_t>(idx);
        return true;
    }
    return false;
}

Ref parseRef(const char* token) {
    Ref r;
    char buf[128];
    size_t len = std::strlen(token);
    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    std::memcpy(buf, token, len);
    buf[len] = 0;

    char* p = buf;
    r.v = std::atoi(p);
    char* slash = std::strchr(p, '/');
    if (!slash) return r;
    p = slash + 1;
    if (*p != '/') r.t = std::atoi(p);
    slash = std::strchr(p, '/');
    if (!slash) return r;
    r.n = std::atoi(slash + 1);
    return r;
}

}  // namespace

bool parseObj(const std::string& text, MeshData& out, std::string& error) {
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<Vec2> uvs;
    std::unordered_map<Ref, uint32_t, RefHash> lookup;

    out.vertices.clear();
    out.indices.clear();
    bool anyNormals = false;

    std::istringstream stream(text);
    std::string line;
    size_t lineNo = 0;
    std::vector<uint32_t> face;

    while (std::getline(stream, line)) {
        ++lineNo;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const char* s = line.c_str();
        while (*s == ' ' || *s == '\t') ++s;
        if (*s == '#' || *s == 0) continue;

        if (s[0] == 'v' && (s[1] == ' ' || s[1] == '\t')) {
            float x = 0, y = 0, z = 0;
            if (std::sscanf(s + 1, "%f %f %f", &x, &y, &z) != 3) {
                error = "bad vertex on line " + std::to_string(lineNo);
                return false;
            }
            positions.push_back({x, y, z});
        } else if (s[0] == 'v' && s[1] == 'n') {
            float x = 0, y = 0, z = 0;
            if (std::sscanf(s + 2, "%f %f %f", &x, &y, &z) != 3) {
                error = "bad normal on line " + std::to_string(lineNo);
                return false;
            }
            normals.push_back({x, y, z});
        } else if (s[0] == 'v' && s[1] == 't') {
            float u = 0, v = 0;
            if (std::sscanf(s + 2, "%f %f", &u, &v) < 1) {
                error = "bad texcoord on line " + std::to_string(lineNo);
                return false;
            }
            uvs.push_back({u, v});
        } else if (s[0] == 'f' && (s[1] == ' ' || s[1] == '\t')) {
            face.clear();
            std::istringstream fs(s + 1);
            std::string token;
            while (fs >> token) {
                Ref ref = parseRef(token.c_str());
                auto it = lookup.find(ref);
                if (it != lookup.end()) {
                    face.push_back(it->second);
                    continue;
                }
                size_t pi = 0;
                if (!resolve(ref.v, positions.size(), pi)) {
                    error = "vertex index out of range on line " + std::to_string(lineNo);
                    return false;
                }
                Vertex vtx;
                vtx.position = positions[pi];
                size_t ni = 0;
                if (ref.n != 0 && resolve(ref.n, normals.size(), ni)) {
                    vtx.normal = normals[ni];
                    anyNormals = true;
                }
                size_t ti = 0;
                if (ref.t != 0 && resolve(ref.t, uvs.size(), ti)) vtx.uv = uvs[ti];
                uint32_t idx = static_cast<uint32_t>(out.vertices.size());
                out.vertices.push_back(vtx);
                lookup.emplace(ref, idx);
                face.push_back(idx);
            }
            if (face.size() < 3) {
                error = "degenerate face on line " + std::to_string(lineNo);
                return false;
            }
            for (size_t i = 1; i + 1 < face.size(); ++i) {
                out.indices.push_back(face[0]);
                out.indices.push_back(face[i]);
                out.indices.push_back(face[i + 1]);
            }
        }
    }

    if (out.vertices.empty()) {
        error = "no geometry";
        return false;
    }
    if (!anyNormals) out.recomputeNormals();
    out.recomputeBounds();
    return true;
}

bool loadObj(const std::string& path, MeshData& out, std::string& error) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        error = "cannot open " + path;
        return false;
    }
    std::ostringstream buf;
    buf << file.rdbuf();
    if (out.name.empty()) {
        size_t slash = path.find_last_of("/\\");
        size_t dot = path.find_last_of('.');
        size_t start = slash == std::string::npos ? 0 : slash + 1;
        out.name = path.substr(start, dot == std::string::npos || dot < start ? std::string::npos : dot - start);
    }
    return parseObj(buf.str(), out, error);
}

}  // namespace skein
