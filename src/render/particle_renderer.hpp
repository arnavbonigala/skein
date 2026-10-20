#pragma once
#include <string>
#include <vector>

#include "core/math.hpp"
#include "render/shader.hpp"

namespace skein {

class ParticleSystem;

class ParticleRenderer {
public:
    ~ParticleRenderer();

    bool init(std::string& error);
    void upload(const std::vector<const ParticleSystem*>& systems);
    void draw(const std::vector<const ParticleSystem*>& systems, const Mat4& view, const Mat4& viewProj,
              const Vec3& light);
    size_t gpuBytes() const { return capacity_; }

private:
    Shader shader_;
    unsigned vao_ = 0;
    unsigned vbo_ = 0;
    size_t capacity_ = 0;
    std::vector<size_t> offsets_;
};

}  // namespace skein
