#pragma once
#define GL_SILENCE_DEPRECATION 1
#include <OpenGL/gl3.h>

#define GLFW_INCLUDE_NONE 1
#include <GLFW/glfw3.h>

namespace skein {

/// Drains the GL error queue and reports the first error with its call site.
/// Returns true when the queue was clean.
bool glCheck(const char* where);

}  // namespace skein
