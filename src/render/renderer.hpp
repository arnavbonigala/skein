#pragma once
#include <string>
#include <vector>

#include "core/math.hpp"
#include "render/gpu_mesh.hpp"
#include "render/render_list.hpp"
#include "render/shader.hpp"

namespace skein {

class Assets;
class Scene;
class JobSystem;

struct RenderOptions {
    bool frustumCulling = true;
    bool clusterCulling = true;
    bool instancing = true;
    bool shadows = true;
    bool wireframeBounds = false;
    Vec3 skyColor{0.06f, 0.07f, 0.10f};
    Vec3 ambientSky{0.10f, 0.12f, 0.17f};
    Vec3 ambientGround{0.04f, 0.035f, 0.03f};
    float fogDensity = 0.0035f;
    float shadowExtent = 55.0f;
    float shadowDistance = 120.0f;
};

struct RenderStats {
    uint32_t candidates = 0;
    uint32_t visible = 0;
    uint32_t drawCalls = 0;
    uint32_t shadowDrawCalls = 0;
    uint64_t triangles = 0;
    uint32_t pointLights = 0;
    /// Objects that needed their own frustum test rather than inheriting a
    /// cluster's verdict.
    uint32_t objectsTested = 0;
    double cullMs = 0;
    double submitMs = 0;
    double cpuMs = 0;
    double gpuMs = 0;
    size_t gpuBytes = 0;
};

/// Forward renderer for OpenGL 4.1 core. Visible objects are culled and grouped
/// into instanced batches on the CPU, streamed into a single instance buffer,
/// and drawn with one call per (mesh, material) pair.
class Renderer {
public:
    ~Renderer();

    bool init(std::string& error);
    void uploadAssets(const Assets& assets);
    void resize(int width, int height);

    void render(Scene& scene, const Assets& assets, const Mat4& view, const Mat4& projection,
                const Vec3& cameraPosition, JobSystem* jobs);

    RenderOptions options;
    const RenderStats& stats() const { return stats_; }
    const RenderList& mainList() const { return mainList_; }
    size_t cpuSideBytes() const { return culler_.bytesUsed() + shadowCuller_.bytesUsed(); }

private:
    struct PointLight {
        Vec4 posRange;
        Vec4 colorIntensity;
    };

    void collectLights(Scene& scene, const Vec3& cameraPosition);
    Mat4 sunMatrix(const Vec3& focus) const;
    void drawList(const RenderList& list, const Assets& assets, Shader& shader, bool applyMaterials,
                  uint32_t instanceBase, uint32_t& drawCallsOut, uint64_t& trianglesOut);
    bool createShadowTarget();

    Shader lit_;
    Shader depth_;
    InstanceBuffer instances_;
    std::vector<GpuMesh> meshes_;

    CullSystem culler_;
    CullSystem shadowCuller_;
    uint64_t frameIndex_ = 0;
    int timerMisses_ = 0;
    RenderList mainList_;
    RenderList shadowList_;

    std::vector<PointLight> pointLights_;
    std::vector<Vec4> lightPosRange_;
    std::vector<Vec4> lightColor_;
    Vec3 sunDirection_{-0.45f, -0.85f, -0.3f};
    Vec3 sunColor_{1.0f, 0.96f, 0.88f};
    float sunIntensity_ = 2.6f;

    unsigned shadowFbo_ = 0;
    unsigned shadowTexture_ = 0;
    int shadowSize_ = 2048;

    unsigned timerQueries_[2] = {0, 0};
    int timerSlot_ = 0;
    bool timerPrimed_ = false;

    int width_ = 1280;
    int height_ = 720;
    size_t meshBytes_ = 0;
    RenderStats stats_;
};

}  // namespace skein
