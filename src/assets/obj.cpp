#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

#include "assets/mesh.hpp"

namespace skein {
namespace {

struct Ref {
    int v = 0, t = 0, n = 0;
    bool operator==(const Ref& o) const { return v == o.v && t == o.t && n == o.n; }
};

uint32_t refHash(const Ref& r) {
    uint32_t h = static_cast<uint32_t>(r.v) * 73856093u;
    h ^= static_cast<uint32_t>(r.t) * 19349663u;
    h ^= static_cast<uint32_t>(r.n) * 83492791u;
    return h;
}

/// Open-addressed weld table. One lookup per face corner is the parser's
/// hottest operation, and a node per corner in a std::unordered_map costs more
/// in allocation and pointer chasing than the whole rest of the parse.
/// A `v` of 0 is not a legal OBJ index, so it marks a free slot.
class WeldTable {
public:
    WeldTable() : slots_(1024) {}

    /// Returns the welded vertex index, or inserts `next` and returns it.
    uint32_t insert(const Ref& r, uint32_t next, bool& inserted) {
        uint32_t mask = static_cast<uint32_t>(slots_.size()) - 1;
        for (uint32_t i = refHash(r) & mask;; i = (i + 1) & mask) {
            Slot& slot = slots_[i];
            if (slot.ref.v == 0) {
                slot.ref = r;
                slot.index = next;
                inserted = true;
                if (++used_ * 10 > slots_.size() * 7) grow();
                return next;
            }
            if (slot.ref == r) {
                inserted = false;
                return slot.index;
            }
        }
    }

private:
    struct Slot {
        Ref ref;
        uint32_t index = 0;
    };

    void grow() {
        std::vector<Slot> bigger(slots_.size() * 2);
        uint32_t mask = static_cast<uint32_t>(bigger.size()) - 1;
        for (const Slot& slot : slots_) {
            if (slot.ref.v == 0) continue;
            uint32_t i = refHash(slot.ref) & mask;
            while (bigger[i].ref.v != 0) i = (i + 1) & mask;
            bigger[i] = slot;
        }
        slots_.swap(bigger);
    }

    std::vector<Slot> slots_;
    size_t used_ = 0;
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

/// Reads `v`, `v/t`, `v//n` or `v/t/n` in place. Returns the end of the token.
const char* parseRef(const char* p, const char* end, Ref& r) {
    auto integer = [&](const char*& q) {
        int sign = 1;
        if (q < end && (*q == '-' || *q == '+')) sign = *q++ == '-' ? -1 : 1;
        int value = 0;
        while (q < end && *q >= '0' && *q <= '9') value = value * 10 + (*q++ - '0');
        return sign * value;
    };
    r = Ref{};
    r.v = integer(p);
    if (p < end && *p == '/') {
        ++p;
        if (p < end && *p != '/') r.t = integer(p);
        if (p < end && *p == '/') {
            ++p;
            r.n = integer(p);
        }
    }
    return p;
}

}  // namespace

bool parseObj(const std::string& text, MeshData& out, std::string& error) {
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<Vec2> uvs;
    WeldTable lookup;

    out.vertices.clear();
    out.indices.clear();
    bool anyNormals = false;

    // The text is walked in place. A `getline` into a std::string plus a
    // `sscanf` per line spends most of its time on allocation and format
    // parsing rather than on the numbers, and an OBJ is nothing but numbers.
    const char* p = text.c_str();
    const char* end = p + text.size();
    size_t lineNo = 0;
    std::vector<uint32_t> face;

    auto skipBlanks = [&](const char*& q) {
        while (q < end && (*q == ' ' || *q == '\t')) ++q;
    };
    // strtof stops at the first character it cannot use and the buffer is
    // NUL-terminated as a whole, so it is safe to call on any interior pointer.
    auto readFloat = [&](const char*& q, float& value) {
        skipBlanks(q);
        char* stop = nullptr;
        value = std::strtof(q, &stop);
        if (stop == q) return false;
        q = stop;
        return true;
    };

    while (p < end) {
        ++lineNo;
        const char* eol = static_cast<const char*>(std::memchr(p, '\n', static_cast<size_t>(end - p)));
        const char* lineEnd = eol ? eol : end;
        const char* s = p;
        p = eol ? eol + 1 : end;
        if (lineEnd > s && lineEnd[-1] == '\r') --lineEnd;
        skipBlanks(s);
        if (s >= lineEnd || *s == '#') continue;

        if (s[0] == 'v' && s + 1 < lineEnd && (s[1] == ' ' || s[1] == '\t')) {
            const char* q = s + 1;
            float x = 0, y = 0, z = 0;
            if (!readFloat(q, x) || !readFloat(q, y) || !readFloat(q, z)) {
                error = "bad vertex on line " + std::to_string(lineNo);
                return false;
            }
            positions.push_back({x, y, z});
        } else if (s[0] == 'v' && s + 1 < lineEnd && s[1] == 'n') {
            const char* q = s + 2;
            float x = 0, y = 0, z = 0;
            if (!readFloat(q, x) || !readFloat(q, y) || !readFloat(q, z)) {
                error = "bad normal on line " + std::to_string(lineNo);
                return false;
            }
            normals.push_back({x, y, z});
        } else if (s[0] == 'v' && s + 1 < lineEnd && s[1] == 't') {
            const char* q = s + 2;
            float u = 0, v = 0;
            if (!readFloat(q, u)) {
                error = "bad texcoord on line " + std::to_string(lineNo);
                return false;
            }
            readFloat(q, v);
            uvs.push_back({u, v});
        } else if (s[0] == 'f' && s + 1 < lineEnd && (s[1] == ' ' || s[1] == '\t')) {
            face.clear();
            const char* q = s + 1;
            for (;;) {
                skipBlanks(q);
                if (q >= lineEnd) break;
                Ref ref;
                const char* after = parseRef(q, lineEnd, ref);
                if (after == q) break;
                q = after;
                bool fresh = false;
                uint32_t idx = lookup.insert(ref, static_cast<uint32_t>(out.vertices.size()), fresh);
                if (!fresh) {
                    face.push_back(idx);
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
                out.vertices.push_back(vtx);
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
