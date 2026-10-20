#include "render/particle_renderer.hpp"

#include "render/gl.hpp"
#include "scene/particles.hpp"

namespace skein {

namespace {

const char* PARTICLE_VERTEX = R"(#version 410 core
layout(location = 0) in vec4 aPositionSize;
layout(location = 1) in vec4 aColor;
layout(location = 2) in vec2 aRotationGlow;

uniform mat4 uViewProj;
uniform vec3 uRight;
uniform vec3 uUp;
uniform vec3 uLight;

out vec2 vCorner;
out vec3 vColor;

const vec2 CORNERS[6] = vec2[](vec2(-1, -1), vec2(1, -1), vec2(1, 1), vec2(-1, -1), vec2(1, 1), vec2(-1, 1));

void main() {
    vec2 corner = CORNERS[gl_VertexID];
    float c = cos(aRotationGlow.x), s = sin(aRotationGlow.x);
    vec2 turned = vec2(c * corner.x - s * corner.y, s * corner.x + c * corner.y) * aPositionSize.w;
    vec3 world = aPositionSize.xyz + uRight * turned.x + uUp * turned.y;
    vCorner = corner;
    vColor = aColor.rgb * uLight * aRotationGlow.y * (0.8 + 0.2 * cos(aRotationGlow.x * 1.7));
    gl_Position = uViewProj * vec4(world, 1.0);
}
)";

const char* PARTICLE_FRAGMENT = R"(#version 410 core
in vec2 vCorner;
in vec3 vColor;

uniform int uRound;
uniform int uAdditive;

out vec4 fragColor;

void main() {
    float r2 = dot(vCorner, vCorner);
    if (uAdditive == 1) {
        float falloff = max(1.0 - r2, 0.0);
        fragColor = vec4(vColor * falloff * falloff, 1.0);
        return;
    }
    if (uRound == 1 && r2 > 1.0) discard;
    fragColor = vec4(vColor, 1.0);
}
)";

}  // namespace

ParticleRenderer::~ParticleRenderer() {
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
}

bool ParticleRenderer::init(std::string& error) {
    if (!shader_.compile(PARTICLE_VERTEX, PARTICLE_FRAGMENT, error)) return false;
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    const auto offset = [](size_t bytes) { return reinterpret_cast<void*>(bytes); };
    const GLsizei stride = sizeof(ParticleInstance);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, stride, offset(offsetof(ParticleInstance, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, stride, offset(offsetof(ParticleInstance, color)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, offset(offsetof(ParticleInstance, rotation)));
    for (unsigned i = 0; i < 3; ++i) glVertexAttribDivisor(i, 1);
    glBindVertexArray(0);
    return glCheck("ParticleRenderer::init");
}

void ParticleRenderer::upload(const std::vector<const ParticleSystem*>& systems) {
    offsets_.clear();
    size_t total = 0;
    for (const ParticleSystem* system : systems) {
        offsets_.push_back(total);
        total += system->instances().size();
    }
    if (total == 0) return;
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    const size_t bytes = total * sizeof(ParticleInstance);
    if (bytes > capacity_) capacity_ = bytes * 2;
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(capacity_), nullptr, GL_STREAM_DRAW);
    for (size_t i = 0; i < systems.size(); ++i) {
        const auto& instances = systems[i]->instances();
        if (instances.empty()) continue;
        glBufferSubData(GL_ARRAY_BUFFER, static_cast<GLintptr>(offsets_[i] * sizeof(ParticleInstance)),
                        static_cast<GLsizeiptr>(instances.size() * sizeof(ParticleInstance)), instances.data());
    }
}

void ParticleRenderer::draw(const std::vector<const ParticleSystem*>& systems, const Mat4& view,
                            const Mat4& viewProj, const Vec3& light) {
    if (offsets_.size() != systems.size() || !shader_.valid()) return;

    shader_.bind();
    shader_.setMat4("uViewProj", viewProj);
    shader_.setVec3("uRight", Vec3{view.m[0][0], view.m[1][0], view.m[2][0]});
    shader_.setVec3("uUp", Vec3{view.m[0][1], view.m[1][1], view.m[2][1]});
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glDisable(GL_CULL_FACE);

    const GLsizei stride = sizeof(ParticleInstance);
    for (int pass = 0; pass < 2; ++pass) {
        const bool additive = pass == 1;
        for (size_t i = 0; i < systems.size(); ++i) {
            const ParticleSystem& system = *systems[i];
            if (system.additive != additive || system.instances().empty()) continue;
            const size_t base = offsets_[i] * sizeof(ParticleInstance);
            const auto at = [base](size_t field) { return reinterpret_cast<void*>(base + field); };
            glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, stride, at(offsetof(ParticleInstance, position)));
            glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, stride, at(offsetof(ParticleInstance, color)));
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, at(offsetof(ParticleInstance, rotation)));
            shader_.setVec3("uLight", additive ? Vec3{1, 1, 1} : light);
            shader_.setInt("uRound", system.round ? 1 : 0);
            shader_.setInt("uAdditive", additive ? 1 : 0);
            if (additive) {
                glEnable(GL_BLEND);
                glBlendFunc(GL_ONE, GL_ONE);
                glDepthMask(GL_FALSE);
            }
            glDrawArraysInstanced(GL_TRIANGLES, 0, 6, static_cast<GLsizei>(system.instances().size()));
        }
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    glBindVertexArray(0);
}

}  // namespace skein
