#include "render/gpu_mesh.hpp"

#include "assets/mesh.hpp"
#include "render/gl.hpp"

namespace skein {

InstanceBuffer::~InstanceBuffer() { destroy(); }

void InstanceBuffer::destroy() {
    if (vbo_) glDeleteBuffers(1, &vbo_);
    vbo_ = 0;
    capacity_ = 0;
}

void InstanceBuffer::ensure(size_t bytes) {
    if (!vbo_) glGenBuffers(1, &vbo_);
    if (bytes <= capacity_) return;
    size_t want = capacity_ ? capacity_ : 64 * sizeof(Mat4);
    while (want < bytes) want *= 2;
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(want), nullptr, GL_STREAM_DRAW);
    capacity_ = want;
}

void InstanceBuffer::upload(const Mat4* data, size_t count) {
    size_t bytes = count * sizeof(Mat4);
    ensure(bytes);
    if (count == 0) return;
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(capacity_), nullptr, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(bytes), data);
}

GpuMesh::~GpuMesh() { destroy(); }

GpuMesh::GpuMesh(GpuMesh&& other) noexcept
    : vao_(other.vao_),
      vbo_(other.vbo_),
      ebo_(other.ebo_),
      instanceVbo_(other.instanceVbo_),
      indexCount_(other.indexCount_),
      bytes_(other.bytes_) {
    other.vao_ = other.vbo_ = other.ebo_ = 0;
    other.indexCount_ = 0;
    other.bytes_ = 0;
}

GpuMesh& GpuMesh::operator=(GpuMesh&& other) noexcept {
    if (this != &other) {
        destroy();
        vao_ = other.vao_;
        vbo_ = other.vbo_;
        ebo_ = other.ebo_;
        instanceVbo_ = other.instanceVbo_;
        indexCount_ = other.indexCount_;
        bytes_ = other.bytes_;
        other.vao_ = other.vbo_ = other.ebo_ = 0;
        other.indexCount_ = 0;
        other.bytes_ = 0;
    }
    return *this;
}

void GpuMesh::destroy() {
    if (ebo_) glDeleteBuffers(1, &ebo_);
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
    vao_ = vbo_ = ebo_ = 0;
    indexCount_ = 0;
    bytes_ = 0;
}

bool GpuMesh::upload(const MeshData& mesh, unsigned instanceVbo) {
    destroy();
    if (mesh.vertices.empty() || mesh.indices.empty()) return false;

    instanceVbo_ = instanceVbo;
    indexCount_ = static_cast<uint32_t>(mesh.indices.size());
    bytes_ = mesh.vertices.size() * sizeof(Vertex) + mesh.indices.size() * sizeof(uint32_t);

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.vertices.size() * sizeof(Vertex)),
                 mesh.vertices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &ebo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.indices.size() * sizeof(uint32_t)),
                 mesh.indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, uv)));

    glBindBuffer(GL_ARRAY_BUFFER, instanceVbo_);
    for (int col = 0; col < 4; ++col) {
        GLuint attrib = static_cast<GLuint>(3 + col);
        glEnableVertexAttribArray(attrib);
        glVertexAttribPointer(attrib, 4, GL_FLOAT, GL_FALSE, sizeof(Mat4),
                              reinterpret_cast<void*>(static_cast<uintptr_t>(col * sizeof(Vec4))));
        glVertexAttribDivisor(attrib, 1);
    }

    glBindVertexArray(0);
    return glCheck("GpuMesh::upload");
}

void GpuMesh::drawInstanced(uint32_t firstInstance, uint32_t count) const {
    if (!vao_ || count == 0) return;
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, instanceVbo_);
    const uintptr_t base = static_cast<uintptr_t>(firstInstance) * sizeof(Mat4);
    for (int col = 0; col < 4; ++col) {
        GLuint attrib = static_cast<GLuint>(3 + col);
        glVertexAttribPointer(attrib, 4, GL_FLOAT, GL_FALSE, sizeof(Mat4),
                              reinterpret_cast<void*>(base + static_cast<uintptr_t>(col) * sizeof(Vec4)));
    }
    glDrawElementsInstanced(GL_TRIANGLES, static_cast<GLsizei>(indexCount_), GL_UNSIGNED_INT, nullptr,
                            static_cast<GLsizei>(count));
}

}  // namespace skein
