#pragma once
#include <string>
#include <vector>

#include "core/math.hpp"
#include "render/shader.hpp"

namespace skein {

struct PostOptions {
    int msaa = 4;
    float exposure = 1.0f;
    float saturation = 1.05f;
    float contrast = 1.0f;
    float vignette = 0.25f;
    bool bloom = true;
    float bloomThreshold = 1.6f;
    float bloomKnee = 0.5f;
    float bloomStrength = 0.15f;
    float bloomRadius = 1.0f;
    bool ssao = true;
    float ssaoRadius = 0.6f;
    float ssaoBias = 0.03f;
    float ssaoPower = 1.4f;
    float ssaoAmount = 0.9f;
    float outline = 0.0f;
    float outlineWidth = 1.0f;
    float outlineLow = 0.02f;
    float outlineHigh = 0.08f;
    Vec3 outlineColor{0.02f, 0.02f, 0.03f};
    bool fxaa = true;
};

class PostChain {
public:
    ~PostChain();

    bool init(std::string& error);
    bool resize(int width, int height, int msaa);

    void beginScene(const Vec3& clearColor);
    void finish(const PostOptions& options, const Mat4& projection);
    void present(int windowWidth, int windowHeight) const;

    unsigned sceneFramebuffer() const { return sceneFbo_; }
    unsigned finalFramebuffer() const { return finalFbo_; }
    int width() const { return width_; }
    int height() const { return height_; }
    size_t gpuBytes() const;

private:
    struct Target {
        unsigned fbo = 0;
        unsigned texture = 0;
        int width = 0;
        int height = 0;
    };

    void destroyTargets();
    bool makeTarget(Target& t, int width, int height, unsigned internalFormat, unsigned format, unsigned type);
    void draw(const Target& into) const;

    Shader ssao_, ssaoBlur_, prefilter_, down_, up_, composite_, fxaa_;
    unsigned vao_ = 0;
    unsigned sceneFbo_ = 0;
    unsigned sceneColor_ = 0;
    unsigned sceneDepth_ = 0;
    unsigned resolveFbo_ = 0;
    unsigned hdrTexture_ = 0;
    unsigned depthTexture_ = 0;
    Target ao_, aoBlur_, ldr_;
    std::vector<Target> bloom_;
    unsigned finalFbo_ = 0;
    unsigned finalTexture_ = 0;
    int width_ = 0;
    int height_ = 0;
    int msaa_ = 0;
};

}  // namespace skein
