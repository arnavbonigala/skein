#include "render/shader.hpp"

#include <cstdio>
#include <vector>

#include "render/gl.hpp"

namespace skein {
namespace {

bool compileStage(GLenum stage, const char* source, GLuint& out, std::string& error) {
    GLuint shader = glCreateShader(stage);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> log(static_cast<size_t>(len > 0 ? len : 1));
        glGetShaderInfoLog(shader, len, nullptr, log.data());
        error = std::string(stage == GL_VERTEX_SHADER ? "vertex: " : "fragment: ") + log.data();
        glDeleteShader(shader);
        return false;
    }
    out = shader;
    return true;
}

}  // namespace

bool glCheck(const char* where) {
    GLenum err = glGetError();
    if (err == GL_NO_ERROR) return true;
    do {
        std::fprintf(stderr, "[gl] error 0x%04x at %s\n", err, where);
        err = glGetError();
    } while (err != GL_NO_ERROR);
    return false;
}

Shader::~Shader() { destroy(); }

Shader::Shader(Shader&& other) noexcept : program_(other.program_), uniforms_(std::move(other.uniforms_)) {
    other.program_ = 0;
}

Shader& Shader::operator=(Shader&& other) noexcept {
    if (this != &other) {
        destroy();
        program_ = other.program_;
        uniforms_ = std::move(other.uniforms_);
        other.program_ = 0;
    }
    return *this;
}

void Shader::destroy() {
    if (program_) glDeleteProgram(program_);
    program_ = 0;
    uniforms_.clear();
}

bool Shader::compile(const char* vertexSource, const char* fragmentSource, std::string& error) {
    destroy();
    GLuint vs = 0, fs = 0;
    if (!compileStage(GL_VERTEX_SHADER, vertexSource, vs, error)) return false;
    if (!compileStage(GL_FRAGMENT_SHADER, fragmentSource, fs, error)) {
        glDeleteShader(vs);
        return false;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint ok = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> log(static_cast<size_t>(len > 0 ? len : 1));
        glGetProgramInfoLog(program, len, nullptr, log.data());
        error = std::string("link: ") + log.data();
        glDeleteProgram(program);
        return false;
    }
    program_ = program;
    return true;
}

void Shader::bind() const { glUseProgram(program_); }

int Shader::location(const char* name) {
    auto it = uniforms_.find(name);
    if (it != uniforms_.end()) return it->second;
    int loc = glGetUniformLocation(program_, name);
    uniforms_.emplace(name, loc);
    return loc;
}

void Shader::setMat4(const char* name, const Mat4& value) {
    int loc = location(name);
    if (loc >= 0) glUniformMatrix4fv(loc, 1, GL_FALSE, value.data());
}

void Shader::setVec3(const char* name, const Vec3& value) {
    int loc = location(name);
    if (loc >= 0) glUniform3f(loc, value.x, value.y, value.z);
}

void Shader::setVec4(const char* name, const Vec4& value) {
    int loc = location(name);
    if (loc >= 0) glUniform4f(loc, value.x, value.y, value.z, value.w);
}

void Shader::setFloat(const char* name, float value) {
    int loc = location(name);
    if (loc >= 0) glUniform1f(loc, value);
}

void Shader::setInt(const char* name, int value) {
    int loc = location(name);
    if (loc >= 0) glUniform1i(loc, value);
}

void Shader::setVec3Array(const char* name, const Vec3* values, int count) {
    int loc = location(name);
    if (loc >= 0 && count > 0) glUniform3fv(loc, count, &values->x);
}

void Shader::setVec4Array(const char* name, const Vec4* values, int count) {
    int loc = location(name);
    if (loc >= 0 && count > 0) glUniform4fv(loc, count, &values->x);
}

}  // namespace skein
