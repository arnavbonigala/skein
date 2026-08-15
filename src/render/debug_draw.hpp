#pragma once
#include <string>
#include <vector>

#include "core/math.hpp"
#include "render/shader.hpp"

namespace skein {

/// Immediate-mode line list, rebuilt and uploaded once per frame. Used to show
/// what the culler kept and where the shadow camera is looking.
class DebugDraw {
public:
    ~DebugDraw();

    bool init(std::string& error);
    void clear() { vertices_.clear(); }
    bool empty() const { return vertices_.empty(); }

    void line(const Vec3& a, const Vec3& b, const Vec3& color);
    void box(const Vec3& center, const Vec3& extent, const Vec3& color);
    /// Draws the wireframe of the volume that `viewProj` projects to clip space.
    void frustum(const Mat4& viewProj, const Vec3& color);

    void flush(const Mat4& viewProj);

private:
    struct LineVertex {
        Vec3 position;
        Vec3 color;
    };

    Shader shader_;
    unsigned vao_ = 0;
    unsigned vbo_ = 0;
    size_t capacity_ = 0;
    std::vector<LineVertex> vertices_;
};

}  // namespace skein
