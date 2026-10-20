#include "assets/font.hpp"

#include <algorithm>
#include <fstream>
#include <iterator>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#define STB_TRUETYPE_IMPLEMENTATION
#define STBTT_STATIC
#include "stb/stb_truetype.h"
#pragma clang diagnostic pop

namespace skein {

Font::Font() = default;
Font::~Font() = default;
Font::Font(Font&&) noexcept = default;
Font& Font::operator=(Font&&) noexcept = default;

std::string Font::systemPath() {
    const char* candidates[] = {
        "/System/Library/Fonts/Supplemental/Arial Rounded Bold.ttf",
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf",
        "C:/Windows/Fonts/arialbd.ttf",
    };
    for (const char* path : candidates)
        if (std::ifstream(path, std::ios::binary)) return path;
    return {};
}

bool Font::load(const std::string& path, std::string& error) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        error = "cannot open " + path;
        return false;
    }
    return loadMemory(std::vector<unsigned char>(std::istreambuf_iterator<char>(file), {}), error);
}

bool Font::loadMemory(std::vector<unsigned char> data, std::string& error) {
    data_ = std::move(data);
    info_ = std::make_unique<stbtt_fontinfo>();
    int offset = stbtt_GetFontOffsetForIndex(data_.data(), 0);
    if (offset < 0 || !stbtt_InitFont(info_.get(), data_.data(), offset)) {
        error = "not a TrueType font";
        atlas_.clear();
        return false;
    }
    scale_ = stbtt_ScaleForPixelHeight(info_.get(), BAKE_PX);
    int ascent = 0, descent = 0, gap = 0;
    stbtt_GetFontVMetrics(info_.get(), &ascent, &descent, &gap);
    ascent_ = static_cast<float>(ascent) * scale_;
    lineGap_ = static_cast<float>(ascent - descent + gap) * scale_;

    struct Bitmap {
        unsigned char* pixels;
        int w, h, xoff, yoff;
    };
    Bitmap bitmaps[COUNT];
    atlasWidth_ = 1024;
    int x = 1, y = 1, row = 0;
    int slots[COUNT][2];
    for (int i = 0; i < COUNT; ++i) {
        Bitmap& b = bitmaps[i];
        b.pixels = stbtt_GetCodepointSDF(info_.get(), scale_, FIRST + i, PADDING, 128,
                                         SLOPE * 255.0f, &b.w, &b.h, &b.xoff, &b.yoff);
        if (!b.pixels) b.w = b.h = 0;
        if (x + b.w + 1 > atlasWidth_) {
            x = 1;
            y += row + 1;
            row = 0;
        }
        slots[i][0] = x;
        slots[i][1] = y;
        x += b.w + 1;
        row = std::max(row, b.h);
    }
    atlasHeight_ = 1;
    while (atlasHeight_ < y + row + 1) atlasHeight_ *= 2;
    atlas_.assign(static_cast<size_t>(atlasWidth_) * atlasHeight_, 0);

    const float iw = 1.0f / static_cast<float>(atlasWidth_), ih = 1.0f / static_cast<float>(atlasHeight_);
    for (int i = 0; i < COUNT; ++i) {
        Bitmap& b = bitmaps[i];
        Glyph& g = glyphs_[i];
        int advance = 0, bearing = 0;
        stbtt_GetCodepointHMetrics(info_.get(), FIRST + i, &advance, &bearing);
        g.advance = static_cast<float>(advance) * scale_;
        if (!b.pixels) continue;
        for (int r = 0; r < b.h; ++r)
            std::copy_n(b.pixels + r * b.w, b.w,
                        atlas_.begin() + static_cast<ptrdiff_t>((slots[i][1] + r) * atlasWidth_ + slots[i][0]));
        g.x0 = static_cast<float>(b.xoff);
        g.y0 = static_cast<float>(b.yoff);
        g.x1 = g.x0 + static_cast<float>(b.w);
        g.y1 = g.y0 + static_cast<float>(b.h);
        g.u0 = static_cast<float>(slots[i][0]) * iw;
        g.v0 = static_cast<float>(slots[i][1]) * ih;
        g.u1 = static_cast<float>(slots[i][0] + b.w) * iw;
        g.v1 = static_cast<float>(slots[i][1] + b.h) * ih;
        stbtt_FreeSDF(b.pixels, nullptr);
    }
    return true;
}

float Font::kern(int a, int b) const {
    return static_cast<float>(stbtt_GetCodepointKernAdvance(info_.get(), a, b)) * scale_;
}

Vec2 Font::measure(std::string_view text, float size) const {
    if (!loaded()) return {};
    float width = 0, line = 0;
    int lines = 1, prev = 0;
    for (char ch : text) {
        int c = static_cast<unsigned char>(ch);
        if (c == '\n') {
            width = std::max(width, line);
            line = 0;
            prev = 0;
            ++lines;
            continue;
        }
        if (c < FIRST || c >= FIRST + COUNT) c = '?';
        if (prev) line += kern(prev, c);
        line += glyphs_[c - FIRST].advance;
        prev = c;
    }
    const float s = size / BAKE_PX;
    return Vec2{std::max(width, line) * s, static_cast<float>(lines) * lineGap_ * s};
}

void Font::layout(std::string_view text, float x, float y, float size, std::vector<GlyphQuad>& out) const {
    if (!loaded()) return;
    const float s = size / BAKE_PX;
    float penX = 0, baseline = ascent_;
    int prev = 0;
    for (char ch : text) {
        int c = static_cast<unsigned char>(ch);
        if (c == '\n') {
            penX = 0;
            baseline += lineGap_;
            prev = 0;
            continue;
        }
        if (c < FIRST || c >= FIRST + COUNT) c = '?';
        if (prev) penX += kern(prev, c);
        const Glyph& g = glyphs_[c - FIRST];
        if (g.x1 > g.x0)
            out.push_back(GlyphQuad{x + (penX + g.x0) * s, y + (baseline + g.y0) * s, x + (penX + g.x1) * s,
                                    y + (baseline + g.y1) * s, g.u0, g.v0, g.u1, g.v1});
        penX += g.advance;
        prev = c;
    }
}

}  // namespace skein
