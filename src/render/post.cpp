#include "render/post.hpp"

#include <algorithm>

#include "render/gl.hpp"
#include "render/post_shaders.hpp"

namespace skein {

PostChain::~PostChain() {
    destroyTargets();
    if (vao_) glDeleteVertexArrays(1, &vao_);
}

bool PostChain::init(std::string& error) {
    const char* v = shaders::FULLSCREEN_VERTEX;
    if (!ssao_.compile(v, shaders::SSAO_FRAGMENT, error)) return false;
    if (!ssaoBlur_.compile(v, shaders::SSAO_BLUR_FRAGMENT, error)) return false;
    if (!prefilter_.compile(v, shaders::BLOOM_PREFILTER_FRAGMENT, error)) return false;
    if (!down_.compile(v, shaders::BLOOM_DOWN_FRAGMENT, error)) return false;
    if (!up_.compile(v, shaders::BLOOM_UP_FRAGMENT, error)) return false;
    if (!composite_.compile(v, shaders::COMPOSITE_FRAGMENT, error)) return false;
    if (!fxaa_.compile(v, shaders::FXAA_FRAGMENT, error)) return false;
    glGenVertexArrays(1, &vao_);
    return glCheck("PostChain::init");
}

void PostChain::destroyTargets() {
    auto drop = [](Target& t) {
        if (t.fbo) glDeleteFramebuffers(1, &t.fbo);
        if (t.texture) glDeleteTextures(1, &t.texture);
        t = Target{};
    };
    drop(ao_);
    drop(aoBlur_);
    drop(ldr_);
    for (Target& t : bloom_) drop(t);
    bloom_.clear();
    const unsigned fbos[3] = {sceneFbo_, resolveFbo_, finalFbo_};
    for (unsigned f : fbos)
        if (f) glDeleteFramebuffers(1, &f);
    if (sceneColor_) glDeleteRenderbuffers(1, &sceneColor_);
    if (sceneDepth_) glDeleteRenderbuffers(1, &sceneDepth_);
    const unsigned textures[3] = {hdrTexture_, depthTexture_, finalTexture_};
    for (unsigned t : textures)
        if (t) glDeleteTextures(1, &t);
    sceneFbo_ = resolveFbo_ = finalFbo_ = sceneColor_ = sceneDepth_ = hdrTexture_ = depthTexture_ = finalTexture_ = 0;
    width_ = height_ = 0;
}

bool PostChain::makeTarget(Target& t, int width, int height, unsigned internalFormat, unsigned format, unsigned type) {
    t.width = std::max(width, 1);
    t.height = std::max(height, 1);
    glGenTextures(1, &t.texture);
    glBindTexture(GL_TEXTURE_2D, t.texture);
    glTexImage2D(GL_TEXTURE_2D, 0, static_cast<GLint>(internalFormat), t.width, t.height, 0, format, type, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glGenFramebuffers(1, &t.fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, t.fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t.texture, 0);
    return glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
}

bool PostChain::resize(int width, int height, int msaa) {
    width = std::max(width, 1);
    height = std::max(height, 1);
    GLint maxSamples = 1;
    glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);
    msaa = std::clamp(msaa, 1, static_cast<int>(maxSamples));
    if (width == width_ && height == height_ && msaa == msaa_) return true;
    destroyTargets();
    width_ = width;
    height_ = height;
    msaa_ = msaa;

    glGenRenderbuffers(1, &sceneColor_);
    glBindRenderbuffer(GL_RENDERBUFFER, sceneColor_);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_RGBA16F, width, height);
    glGenRenderbuffers(1, &sceneDepth_);
    glBindRenderbuffer(GL_RENDERBUFFER, sceneDepth_);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa, GL_DEPTH_COMPONENT24, width, height);
    glGenFramebuffers(1, &sceneFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFbo_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, sceneColor_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, sceneDepth_);
    bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;

    glGenTextures(1, &hdrTexture_);
    glBindTexture(GL_TEXTURE_2D, hdrTexture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_HALF_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glGenTextures(1, &depthTexture_);
    glBindTexture(GL_TEXTURE_2D, depthTexture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glGenFramebuffers(1, &resolveFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, resolveFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, hdrTexture_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthTexture_, 0);
    ok = ok && glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;

    ok = ok && makeTarget(ao_, width / 2, height / 2, GL_R8, GL_RED, GL_UNSIGNED_BYTE);
    ok = ok && makeTarget(aoBlur_, width / 2, height / 2, GL_R8, GL_RED, GL_UNSIGNED_BYTE);
    ok = ok && makeTarget(ldr_, width, height, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
    int w = width / 2, h = height / 2;
    for (int level = 0; level < 6 && w >= 4 && h >= 4; ++level) {
        bloom_.emplace_back();
        ok = ok && makeTarget(bloom_.back(), w, h, GL_RGBA16F, GL_RGBA, GL_HALF_FLOAT);
        w /= 2;
        h /= 2;
    }
    glGenTextures(1, &finalTexture_);
    glBindTexture(GL_TEXTURE_2D, finalTexture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGenFramebuffers(1, &finalFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, finalFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, finalTexture_, 0);
    ok = ok && glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return ok && glCheck("PostChain::resize");
}

void PostChain::beginScene(const Vec3& clearColor) {
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFbo_);
    glViewport(0, 0, width_, height_);
    glClearColor(clearColor.x, clearColor.y, clearColor.z, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void PostChain::draw(const Target& into) const {
    glBindFramebuffer(GL_FRAMEBUFFER, into.fbo);
    glViewport(0, 0, into.width, into.height);
    glDrawArrays(GL_TRIANGLES, 0, 3);
}

void PostChain::finish(const PostOptions& o, const Mat4& projection) {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, sceneFbo_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, resolveFbo_);
    glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_DEPTH_BUFFER_BIT, GL_NEAREST);

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glBindVertexArray(vao_);
    const float projA = projection.m[2][2];
    const float projB = projection.m[3][2];
    auto bindTexture = [](int unit, unsigned texture) {
        glActiveTexture(static_cast<GLenum>(GL_TEXTURE0 + unit));
        glBindTexture(GL_TEXTURE_2D, texture);
    };

    if (o.ssao) {
        ssao_.bind();
        bindTexture(0, depthTexture_);
        ssao_.setInt("uDepth", 0);
        ssao_.setMat4("uProj", projection);
        ssao_.setMat4("uInvProj", inverse(projection));
        ssao_.setVec2("uTexel", 1.0f / static_cast<float>(ao_.width), 1.0f / static_cast<float>(ao_.height));
        ssao_.setFloat("uRadius", o.ssaoRadius);
        ssao_.setFloat("uBias", o.ssaoBias);
        ssao_.setFloat("uPower", o.ssaoPower);
        draw(ao_);
        ssaoBlur_.bind();
        ssaoBlur_.setInt("uAo", 0);
        ssaoBlur_.setInt("uDepth", 1);
        ssaoBlur_.setFloat("uProjA", projA);
        ssaoBlur_.setFloat("uProjB", projB);
        bindTexture(1, depthTexture_);
        bindTexture(0, ao_.texture);
        ssaoBlur_.setVec2("uStep", 1.0f / static_cast<float>(ao_.width), 0);
        draw(aoBlur_);
        bindTexture(0, aoBlur_.texture);
        ssaoBlur_.setVec2("uStep", 0, 1.0f / static_cast<float>(ao_.height));
        draw(ao_);
    }

    const bool bloom = o.bloom && !bloom_.empty();
    if (bloom) {
        prefilter_.bind();
        bindTexture(0, hdrTexture_);
        prefilter_.setInt("uSource", 0);
        prefilter_.setVec2("uTexel", 1.0f / static_cast<float>(width_), 1.0f / static_cast<float>(height_));
        prefilter_.setFloat("uThreshold", o.bloomThreshold);
        prefilter_.setFloat("uKnee", std::max(o.bloomKnee, 1e-3f));
        prefilter_.setFloat("uExposure", o.exposure);
        draw(bloom_[0]);
        down_.bind();
        down_.setInt("uSource", 0);
        for (size_t i = 1; i < bloom_.size(); ++i) {
            bindTexture(0, bloom_[i - 1].texture);
            down_.setVec2("uTexel", 1.0f / static_cast<float>(bloom_[i - 1].width),
                                         1.0f / static_cast<float>(bloom_[i - 1].height));
            draw(bloom_[i]);
        }
        up_.bind();
        up_.setInt("uSource", 0);
        up_.setFloat("uRadius", o.bloomRadius);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
        for (size_t i = bloom_.size() - 1; i > 0; --i) {
            bindTexture(0, bloom_[i].texture);
            up_.setVec2("uTexel", 1.0f / static_cast<float>(bloom_[i].width),
                                       1.0f / static_cast<float>(bloom_[i].height));
            draw(bloom_[i - 1]);
        }
        glDisable(GL_BLEND);
    }

    composite_.bind();
    bindTexture(0, hdrTexture_);
    bindTexture(1, ao_.texture);
    bindTexture(2, bloom ? bloom_[0].texture : 0);
    bindTexture(3, depthTexture_);
    composite_.setInt("uHdr", 0);
    composite_.setInt("uAo", 1);
    composite_.setInt("uBloom", 2);
    composite_.setInt("uDepth", 3);
    composite_.setInt("uUseAo", o.ssao ? 1 : 0);
    composite_.setInt("uUseBloom", bloom ? 1 : 0);
    composite_.setVec2("uTexel", 1.0f / static_cast<float>(width_), 1.0f / static_cast<float>(height_));
    composite_.setFloat("uAoAmount", o.ssaoAmount);
    composite_.setFloat("uBloomStrength", o.bloomStrength);
    composite_.setFloat("uExposure", o.exposure);
    composite_.setFloat("uSaturation", o.saturation);
    composite_.setFloat("uContrast", o.contrast);
    composite_.setFloat("uVignette", o.vignette);
    composite_.setFloat("uOutline", o.outline);
    composite_.setFloat("uOutlineWidth", o.outlineWidth);
    composite_.setFloat("uOutlineLow", o.outlineLow);
    composite_.setFloat("uOutlineHigh", o.outlineHigh);
    composite_.setVec3("uOutlineColor", o.outlineColor);
    composite_.setFloat("uProjA", projA);
    composite_.setFloat("uProjB", projB);
    if (o.fxaa) {
        draw(ldr_);
        fxaa_.bind();
        bindTexture(0, ldr_.texture);
        fxaa_.setInt("uSource", 0);
        fxaa_.setVec2("uTexel", 1.0f / static_cast<float>(width_), 1.0f / static_cast<float>(height_));
    }
    glBindFramebuffer(GL_FRAMEBUFFER, finalFbo_);
    glViewport(0, 0, width_, height_);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    for (int unit = 3; unit >= 0; --unit) bindTexture(unit, 0);
    glBindVertexArray(0);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
}

void PostChain::present(int windowWidth, int windowHeight) const {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, finalFbo_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(0, 0, width_, height_, 0, 0, windowWidth, windowHeight, GL_COLOR_BUFFER_BIT, GL_LINEAR);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

size_t PostChain::gpuBytes() const {
    size_t pixels = static_cast<size_t>(width_) * static_cast<size_t>(height_);
    size_t total = pixels * static_cast<size_t>(msaa_) * (8 + 4) + pixels * (8 + 4 + 4 + 4) + pixels / 2;
    for (const Target& t : bloom_) total += static_cast<size_t>(t.width) * static_cast<size_t>(t.height) * 8;
    return total;
}

}  // namespace skein
