#include "render/renderer.hpp"

#include <algorithm>
#include <cstdio>

#include "assets/mesh.hpp"
#include "core/jobs.hpp"
#include "core/profiler.hpp"
#include "render/gl.hpp"
#include "render/shaders.hpp"
#include "scene/scene.hpp"

namespace skein {

Renderer::~Renderer() {
    if (shadowFbo_) glDeleteFramebuffers(1, &shadowFbo_);
    if (shadowTexture_) glDeleteTextures(1, &shadowTexture_);
    if (timerQueries_[0]) glDeleteQueries(2, timerQueries_);
}

bool Renderer::init(std::string& error) {
    if (!lit_.compile(shaders::LIT_VERTEX, shaders::LIT_FRAGMENT, error)) return false;
    if (!depth_.compile(shaders::DEPTH_VERTEX, shaders::DEPTH_FRAGMENT, error)) return false;
    if (!createShadowTarget()) {
        error = "shadow framebuffer is incomplete";
        return false;
    }
    glGenQueries(2, timerQueries_);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    return glCheck("Renderer::init");
}

bool Renderer::createShadowTarget() {
    glGenTextures(1, &shadowTexture_);
    glBindTexture(GL_TEXTURE_2D, shadowTexture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, shadowSize_, shadowSize_, 0, GL_DEPTH_COMPONENT,
                 GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    const float border[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);

    glGenFramebuffers(1, &shadowFbo_);
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowTexture_, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return complete;
}

void Renderer::uploadAssets(const Assets& assets) {
    instances_.upload(nullptr, 0);
    meshes_.clear();
    meshes_.resize(assets.meshCount());
    meshBytes_ = 0;
    for (size_t i = 0; i < assets.meshCount(); ++i) {
        meshes_[i].upload(assets.mesh(static_cast<uint32_t>(i)), instances_.id());
        meshBytes_ += meshes_[i].bytesUsed();
    }
}

void Renderer::resize(int width, int height) {
    width_ = std::max(width, 1);
    height_ = std::max(height, 1);
}

void Renderer::collectLights(Scene& scene, const Vec3& cameraPosition) {
    pointLights_.clear();
    bool sawSun = false;
    forEach<Light, WorldTransform>(scene.world, [&](Entity, Light& light, WorldTransform& wt) {
        if (light.kind == static_cast<uint32_t>(LightKind::Directional)) {
            if (!sawSun) {
                sunDirection_ = normalize(light.direction);
                sunColor_ = light.color;
                sunIntensity_ = light.intensity;
                sawSun = true;
            }
            return;
        }
        Vec3 pos{wt.matrix.m[3][0], wt.matrix.m[3][1], wt.matrix.m[3][2]};
        pointLights_.push_back(PointLight{Vec4{pos, light.range},
                                          Vec4{light.color, light.intensity}});
    });

    if (pointLights_.size() > static_cast<size_t>(shaders::MAX_POINT_LIGHTS)) {
        std::partial_sort(pointLights_.begin(), pointLights_.begin() + shaders::MAX_POINT_LIGHTS,
                          pointLights_.end(), [&](const PointLight& a, const PointLight& b) {
                              return length2(a.posRange.xyz() - cameraPosition) <
                                     length2(b.posRange.xyz() - cameraPosition);
                          });
        pointLights_.resize(static_cast<size_t>(shaders::MAX_POINT_LIGHTS));
    }
}

Mat4 Renderer::sunMatrix(const Vec3& focus) const {
    Vec3 dir = normalize(sunDirection_);
    Vec3 up = std::fabs(dir.y) > 0.95f ? Vec3{0, 0, 1} : Vec3{0, 1, 0};
    float half = options.shadowExtent;
    Vec3 eye = focus - dir * (options.shadowDistance * 0.5f);
    Mat4 view = lookAt(eye, focus, up);
    Mat4 proj = orthographic(-half, half, -half, half, 0.1f, options.shadowDistance);
    return proj * view;
}

void Renderer::drawList(const RenderList& list, const Assets& assets, Shader& shader, bool applyMaterials,
                        uint32_t instanceBase, uint32_t& drawCallsOut, uint64_t& trianglesOut) {
    for (const DrawBatch& batch : list.batches) {
        if (batch.mesh >= meshes_.size()) continue;
        const GpuMesh& mesh = meshes_[batch.mesh];
        if (applyMaterials) {
            const Material& mat = assets.material(std::min<uint32_t>(
                batch.material, static_cast<uint32_t>(assets.materialCount() ? assets.materialCount() - 1 : 0)));
            shader.setVec3("uAlbedo", mat.albedo);
            shader.setVec3("uEmissive", mat.emissive);
            shader.setFloat("uRoughness", mat.roughness);
            shader.setFloat("uMetallic", mat.metallic);
        }
        if (options.instancing) {
            mesh.drawInstanced(instanceBase + batch.first, batch.count);
            ++drawCallsOut;
        } else {
            for (uint32_t i = 0; i < batch.count; ++i) {
                mesh.drawInstanced(instanceBase + batch.first + i, 1);
                ++drawCallsOut;
            }
        }
        trianglesOut += static_cast<uint64_t>(mesh.triangleCount()) * batch.count;
    }
}

void Renderer::render(Scene& scene, const Assets& assets, const Mat4& view, const Mat4& projection,
                      const Vec3& cameraPosition, JobSystem* jobs) {
    SKEIN_PROFILE("render/frame");
    Clock::time_point cpuStart = Clock::now();

    if (timerQueries_[0]) glBeginQuery(GL_TIME_ELAPSED, timerQueries_[timerSlot_]);

    const Mat4 viewProj = projection * view;
    const Frustum frustum = extractFrustum(viewProj);

    collectLights(scene, cameraPosition);

    Clock::time_point cullStart = Clock::now();
    // The Morton order decays as objects move; maintain() re-sorts only once
    // the clusters have loosened enough to stop paying for themselves.
    culler_.useClusters = options.clusterCulling;
    shadowCuller_.useClusters = options.clusterCulling;
    if (options.frustumCulling && options.clusterCulling && (frameIndex_++ % 30) == 0) culler_.maintain(scene);
    culler_.build(scene, frustum, static_cast<uint32_t>(assets.materialCount()), mainList_, jobs,
                  options.frustumCulling);

    Mat4 lightViewProj = Mat4::identity();
    if (options.shadows) {
        lightViewProj = sunMatrix(Vec3{cameraPosition.x, 0.0f, cameraPosition.z});
        shadowCuller_.build(scene, extractFrustum(lightViewProj),
                            static_cast<uint32_t>(assets.materialCount()), shadowList_, jobs, true);
    } else {
        shadowList_.clear();
    }
    stats_.cullMs = millisSince(cullStart);
    stats_.objectsTested = culler_.stats().objectsTested;

    const size_t mainCount = mainList_.instances.size();
    const size_t shadowCount = shadowList_.instances.size();
    {
        SKEIN_PROFILE("render/uploadInstances");
        std::vector<Mat4>& staging = mainList_.instances;
        staging.insert(staging.end(), shadowList_.instances.begin(), shadowList_.instances.end());
        instances_.upload(staging.data(), staging.size());
        staging.resize(mainCount);
    }

    Clock::time_point submitStart = Clock::now();
    stats_.drawCalls = 0;
    stats_.shadowDrawCalls = 0;
    stats_.triangles = 0;

    if (options.shadows && shadowCount > 0) {
        SKEIN_PROFILE("render/shadowPass");
        glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
        glViewport(0, 0, shadowSize_, shadowSize_);
        glClear(GL_DEPTH_BUFFER_BIT);
        glCullFace(GL_FRONT);
        depth_.bind();
        depth_.setMat4("uViewProj", lightViewProj);
        uint64_t ignoredTriangles = 0;
        drawList(shadowList_, assets, depth_, false, static_cast<uint32_t>(mainCount), stats_.shadowDrawCalls,
                 ignoredTriangles);
        glCullFace(GL_BACK);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    {
        SKEIN_PROFILE("render/mainPass");
        glViewport(0, 0, width_, height_);
        glClearColor(options.skyColor.x, options.skyColor.y, options.skyColor.z, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        lit_.bind();
        lit_.setMat4("uViewProj", viewProj);
        lit_.setMat4("uLightViewProj", lightViewProj);
        lit_.setVec3("uCameraPos", cameraPosition);
        lit_.setVec3("uSunDirection", sunDirection_);
        lit_.setVec3("uSunColor", sunColor_);
        lit_.setFloat("uSunIntensity", sunIntensity_);
        lit_.setVec3("uAmbientSky", options.ambientSky);
        lit_.setVec3("uAmbientGround", options.ambientGround);
        lit_.setVec3("uFogColor", options.skyColor);
        lit_.setFloat("uFogDensity", options.fogDensity);
        lit_.setInt("uShadowEnabled", options.shadows ? 1 : 0);
        lit_.setFloat("uShadowTexel", 1.0f / static_cast<float>(shadowSize_));
        lit_.setInt("uPointCount", static_cast<int>(pointLights_.size()));
        if (!pointLights_.empty()) {
            lightPosRange_.clear();
            lightColor_.clear();
            for (const PointLight& l : pointLights_) {
                lightPosRange_.push_back(l.posRange);
                lightColor_.push_back(l.colorIntensity);
            }
            lit_.setVec4Array("uPointPosRange", lightPosRange_.data(), static_cast<int>(lightPosRange_.size()));
            lit_.setVec4Array("uPointColor", lightColor_.data(), static_cast<int>(lightColor_.size()));
        }
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, shadowTexture_);
        lit_.setInt("uShadowMap", 0);

        drawList(mainList_, assets, lit_, true, 0, stats_.drawCalls, stats_.triangles);
    }

    glBindVertexArray(0);
    stats_.submitMs = millisSince(submitStart);
    stats_.candidates = mainList_.totalCandidates;
    stats_.visible = mainList_.visible;
    stats_.pointLights = static_cast<uint32_t>(pointLights_.size());
    stats_.gpuBytes = meshBytes_ + instances_.capacityBytes() +
                      static_cast<size_t>(shadowSize_) * static_cast<size_t>(shadowSize_) * 3;

    if (timerQueries_[0]) {
        glEndQuery(GL_TIME_ELAPSED);
        int previous = timerSlot_ ^ 1;
        if (timerPrimed_) {
            GLint available = 0;
            glGetQueryObjectiv(timerQueries_[previous], GL_QUERY_RESULT_AVAILABLE, &available);
            // A cheap frame can outrun the driver and leave the query pending
            // forever, so after a few misses the result is worth one stall.
            if (!available && ++timerMisses_ >= 4) available = 1;
            if (available) {
                GLuint64 elapsed = 0;
                glGetQueryObjectui64v(timerQueries_[previous], GL_QUERY_RESULT, &elapsed);
                stats_.gpuMs = static_cast<double>(elapsed) * 1e-6;
                timerMisses_ = 0;
            }
        }
        timerSlot_ = previous;
        timerPrimed_ = true;
    }

    stats_.cpuMs = millisSince(cpuStart);
    Profiler::instance().setCounter("draw calls", stats_.drawCalls + stats_.shadowDrawCalls);
    Profiler::instance().setCounter("visible", stats_.visible);
    Profiler::instance().setCounter("triangles", static_cast<double>(stats_.triangles));
}

}  // namespace skein
