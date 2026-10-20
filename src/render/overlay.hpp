#pragma once
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "assets/font.hpp"
#include "core/math.hpp"
#include "render/shader.hpp"

namespace skein {

class Overlay {
public:
    ~Overlay();

    bool init(const std::string& fontPath, std::string& error);
    const Font& font() const { return font_; }

    void rect(float x, float y, float w, float h, const Vec4& color);
    void text(std::string_view text, float x, float y, float size, const Vec4& color, float outline = 0.0f,
              const Vec4& outlineColor = Vec4{0.0f, 0.0f, 0.0f, 1.0f});
    Vec2 measure(std::string_view text, float size) const { return font_.measure(text, size); }

    void flush(unsigned framebuffer, int width, int height);

private:
    struct Vertex {
        float x, y, u, v;
        uint32_t color, outlineColor;
        float outline;
    };

    void quad(float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1, uint32_t color,
              uint32_t outlineColor, float outline);

    Font font_;
    Shader shader_;
    unsigned vao_ = 0;
    unsigned vbo_ = 0;
    unsigned texture_ = 0;
    size_t capacity_ = 0;
    std::vector<Vertex> vertices_;
    std::vector<GlyphQuad> glyphs_;
};

}  // namespace skein
