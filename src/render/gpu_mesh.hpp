#pragma once
#include <cstddef>
#include <cstdint>

#include "core/math.hpp"

namespace skein {

struct MeshData;

/// Per-frame stream of instance matrices. Reallocation orphans the old storage
/// so the driver never has to stall waiting on the previous frame's draws.
class InstanceBuffer {
public:
    ~InstanceBuffer();
    void destroy();

    void upload(const Mat4* data, size_t count);
    unsigned id() const { return vbo_; }
    size_t capacityBytes() const { return capacity_; }

private:
    void ensure(size_t bytes);

    unsigned vbo_ = 0;
    size_t capacity_ = 0;
};

/// Vertex/index buffers plus a VAO wired for instanced drawing: attributes 0-2
/// come from the mesh, 3-6 are the columns of a per-instance model matrix.
class GpuMesh {
public:
    ~GpuMesh();
    GpuMesh() = default;
    GpuMesh(GpuMesh&& other) noexcept;
    GpuMesh& operator=(GpuMesh&& other) noexcept;
    GpuMesh(const GpuMesh&) = delete;
    GpuMesh& operator=(const GpuMesh&) = delete;

    bool upload(const MeshData& mesh, unsigned instanceVbo);
    void destroy();

    /// Draws `count` instances starting at `firstInstance` in the shared stream.
    void drawInstanced(uint32_t firstInstance, uint32_t count) const;

    uint32_t indexCount() const { return indexCount_; }
    uint32_t triangleCount() const { return indexCount_ / 3; }
    size_t bytesUsed() const { return bytes_; }

private:
    unsigned vao_ = 0;
    unsigned vbo_ = 0;
    unsigned ebo_ = 0;
    unsigned instanceVbo_ = 0;
    uint32_t indexCount_ = 0;
    size_t bytes_ = 0;
};

}  // namespace skein
