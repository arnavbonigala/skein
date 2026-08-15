#include "render/debug_draw.hpp"

#include "render/gl.hpp"
#include "render/shaders.hpp"

namespace skein {

DebugDraw::~DebugDraw() {
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
}

bool DebugDraw::init(std::string& error) {
    if (!shader_.compile(shaders::LINE_VERTEX, shaders::LINE_FRAGMENT, error)) return false;

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(LineVertex),
                          reinterpret_cast<void*>(offsetof(LineVertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(LineVertex),
                          reinterpret_cast<void*>(offsetof(LineVertex, color)));
    glBindVertexArray(0);
    return glCheck("DebugDraw::init");
}

void DebugDraw::line(const Vec3& a, const Vec3& b, const Vec3& color) {
    vertices_.push_back(LineVertex{a, color});
    vertices_.push_back(LineVertex{b, color});
}

void DebugDraw::box(const Vec3& center, const Vec3& extent, const Vec3& color) {
    Vec3 corner[8];
    for (int i = 0; i < 8; ++i)
        corner[i] = center + Vec3{(i & 1) ? extent.x : -extent.x, (i & 2) ? extent.y : -extent.y,
                                  (i & 4) ? extent.z : -extent.z};
    const int edges[12][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7},
                              {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& e : edges) line(corner[e[0]], corner[e[1]], color);
}

void DebugDraw::frustum(const Mat4& viewProj, const Vec3& color) {
    Mat4 inv = inverse(viewProj);
    Vec3 corner[8];
    for (int i = 0; i < 8; ++i) {
        Vec4 clip{(i & 1) ? 1.0f : -1.0f, (i & 2) ? 1.0f : -1.0f, (i & 4) ? 1.0f : -1.0f, 1.0f};
        Vec4 world = inv * clip;
        corner[i] = world.xyz() / (std::fabs(world.w) < 1e-6f ? 1.0f : world.w);
    }
    const int edges[12][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7},
                              {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& e : edges) line(corner[e[0]], corner[e[1]], color);
}

void DebugDraw::flush(const Mat4& viewProj) {
    if (vertices_.empty() || !shader_.valid()) return;

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    size_t bytes = vertices_.size() * sizeof(LineVertex);
    if (bytes > capacity_) {
        capacity_ = bytes * 2;
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(capacity_), nullptr, GL_STREAM_DRAW);
    }
    glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(bytes), vertices_.data());

    shader_.bind();
    shader_.setMat4("uViewProj", viewProj);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices_.size()));
    glBindVertexArray(0);
    vertices_.clear();
}

}  // namespace skein
