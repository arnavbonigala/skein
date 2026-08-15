#pragma once
#include <string>
#include <unordered_map>

#include "core/math.hpp"

namespace skein {

/// A linked GLSL program with a lazily populated uniform-location cache.
class Shader {
public:
    Shader() = default;
    ~Shader();
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    bool compile(const char* vertexSource, const char* fragmentSource, std::string& error);
    void destroy();

    void bind() const;
    unsigned id() const { return program_; }
    bool valid() const { return program_ != 0; }

    void setMat4(const char* name, const Mat4& value);
    void setVec3(const char* name, const Vec3& value);
    void setVec4(const char* name, const Vec4& value);
    void setFloat(const char* name, float value);
    void setInt(const char* name, int value);
    void setVec3Array(const char* name, const Vec3* values, int count);
    void setVec4Array(const char* name, const Vec4* values, int count);

private:
    int location(const char* name);

    unsigned program_ = 0;
    std::unordered_map<std::string, int> uniforms_;
};

}  // namespace skein
