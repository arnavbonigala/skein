#include "render/overlay.hpp"

#include <algorithm>

#include "render/gl.hpp"

namespace skein {

namespace {

const char* OVERLAY_VERTEX = R"(#version 410 core
layout(location = 0) in vec2 aPosition;
layout(location = 1) in vec2 aUv;
layout(location = 2) in vec4 aColor;
layout(location = 3) in vec4 aOutlineColor;
layout(location = 4) in float aOutline;

uniform vec2 uScreen;

out vec2 vUv;
out vec4 vColor;
out vec4 vOutlineColor;
out float vOutline;

void main() {
    vUv = aUv;
    vColor = aColor;
    vOutlineColor = aOutlineColor;
    vOutline = aOutline;
    gl_Position = vec4(aPosition.x / uScreen.x * 2.0 - 1.0, 1.0 - aPosition.y / uScreen.y * 2.0, 0.0, 1.0);
}
)";

const char* OVERLAY_FRAGMENT = R"(#version 410 core
in vec2 vUv;
in vec4 vColor;
in vec4 vOutlineColor;
in float vOutline;

uniform sampler2D uAtlas;
uniform float uEdge;

out vec4 fragColor;

void main() {
    if (vUv.x < 0.0) {
        fragColor = vColor;
        return;
    }
    float d = texture(uAtlas, vUv).r - uEdge;
    float w = max(fwidth(d) * 0.5, 1e-4);
    float fill = smoothstep(-w, w, d);
    float body = smoothstep(-w, w, d + vOutline);
    vec4 color = mix(vOutlineColor, vColor, fill);
    fragColor = vec4(color.rgb, color.a * body);
}
)";

uint32_t pack(const Vec4& c) {
    auto byte = [](float v) { return static_cast<uint32_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f); };
    return byte(c.x) | byte(c.y) << 8 | byte(c.z) << 16 | byte(c.w) << 24;
}

}  // namespace

Overlay::~Overlay() {
    if (texture_) glDeleteTextures(1, &texture_);
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
}

bool Overlay::init(const std::string& fontPath, std::string& error) {
    if (!font_.load(fontPath.empty() ? Font::systemPath() : fontPath, error)) return false;
    if (!shader_.compile(OVERLAY_VERTEX, OVERLAY_FRAGMENT, error)) return false;

    glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, font_.atlasWidth(), font_.atlasHeight(), 0, GL_RED, GL_UNSIGNED_BYTE,
                 font_.atlas().data());
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    const auto offset = [](size_t bytes) { return reinterpret_cast<void*>(bytes); };
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), offset(offsetof(Vertex, x)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), offset(offsetof(Vertex, u)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Vertex), offset(offsetof(Vertex, color)));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(Vertex), offset(offsetof(Vertex, outlineColor)));
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex), offset(offsetof(Vertex, outline)));
    glBindVertexArray(0);
    return glCheck("Overlay::init");
}

void Overlay::quad(float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1, uint32_t color,
                   uint32_t outlineColor, float outline) {
    const Vertex a{x0, y0, u0, v0, color, outlineColor, outline}, b{x1, y0, u1, v0, color, outlineColor, outline},
        c{x1, y1, u1, v1, color, outlineColor, outline}, d{x0, y1, u0, v1, color, outlineColor, outline};
    vertices_.insert(vertices_.end(), {a, b, c, a, c, d});
}

void Overlay::rect(float x, float y, float w, float h, const Vec4& color) {
    quad(x, y, x + w, y + h, -1, -1, -1, -1, pack(color), 0, 0);
}

void Overlay::text(std::string_view text, float x, float y, float size, const Vec4& color, float outline,
                   const Vec4& outlineColor) {
    glyphs_.clear();
    font_.layout(text, x, y, size, glyphs_);
    const float threshold = std::min(outline * Font::BAKE_PX / size * Font::SLOPE, Font::EDGE);
    const uint32_t fill = pack(color), edge = pack(outlineColor);
    for (const GlyphQuad& g : glyphs_) quad(g.x0, g.y0, g.x1, g.y1, g.u0, g.v0, g.u1, g.v1, fill, edge, threshold);
}

void Overlay::flush(unsigned framebuffer, int width, int height) {
    if (vertices_.empty() || !shader_.valid()) return;

    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glViewport(0, 0, width, height);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    const size_t bytes = vertices_.size() * sizeof(Vertex);
    if (bytes > capacity_) {
        capacity_ = bytes * 2;
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(capacity_), nullptr, GL_STREAM_DRAW);
    }
    glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(bytes), vertices_.data());

    shader_.bind();
    shader_.setVec2("uScreen", static_cast<float>(width), static_cast<float>(height));
    shader_.setFloat("uEdge", Font::EDGE);
    shader_.setInt("uAtlas", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices_.size()));

    glBindVertexArray(0);
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    vertices_.clear();
}

}  // namespace skein
