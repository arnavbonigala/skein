#pragma once
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "core/math.hpp"

struct stbtt_fontinfo;

namespace skein {

struct GlyphQuad {
    float x0, y0, x1, y1;
    float u0, v0, u1, v1;
};

class Font {
public:
    static constexpr float BAKE_PX = 48.0f;
    static constexpr int PADDING = 8;
    static constexpr float EDGE = 128.0f / 255.0f;
    static constexpr float SLOPE = 16.0f / 255.0f;

    Font();
    ~Font();
    Font(Font&&) noexcept;
    Font& operator=(Font&&) noexcept;

    bool load(const std::string& path, std::string& error);
    bool loadMemory(std::vector<unsigned char> data, std::string& error);
    bool loaded() const { return !atlas_.empty(); }

    static std::string systemPath();

    Vec2 measure(std::string_view text, float size) const;
    float lineHeight(float size) const { return lineGap_ * size / BAKE_PX; }
    float ascent(float size) const { return ascent_ * size / BAKE_PX; }
    void layout(std::string_view text, float x, float y, float size, std::vector<GlyphQuad>& out) const;

    const std::vector<unsigned char>& atlas() const { return atlas_; }
    int atlasWidth() const { return atlasWidth_; }
    int atlasHeight() const { return atlasHeight_; }

private:
    static constexpr int FIRST = 32;
    static constexpr int COUNT = 95;

    struct Glyph {
        float advance = 0;
        float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        float u0 = 0, v0 = 0, u1 = 0, v1 = 0;
    };

    float kern(int a, int b) const;

    std::vector<unsigned char> data_;
    std::unique_ptr<stbtt_fontinfo> info_;
    float scale_ = 0;
    float ascent_ = 0;
    float lineGap_ = 0;
    Glyph glyphs_[COUNT];
    std::vector<unsigned char> atlas_;
    int atlasWidth_ = 0;
    int atlasHeight_ = 0;
};

}  // namespace skein
