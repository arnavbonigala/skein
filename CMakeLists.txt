cmake_minimum_required(VERSION 3.20)

if(APPLE AND NOT CMAKE_OSX_ARCHITECTURES)
  # uname -m reports x86_64 when cmake itself runs under Rosetta, so ask the
  # kernel for the real machine instead.
  execute_process(COMMAND sysctl -n hw.optional.arm64 OUTPUT_VARIABLE SKEIN_IS_ARM
                  OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(SKEIN_IS_ARM STREQUAL "1")
    set(CMAKE_OSX_ARCHITECTURES arm64 CACHE STRING "" FORCE)
  else()
    set(CMAKE_OSX_ARCHITECTURES x86_64 CACHE STRING "" FORCE)
  endif()
  list(APPEND CMAKE_PREFIX_PATH /opt/homebrew /usr/local)
endif()

project(skein CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

find_path(LUA_INCLUDE_DIR lua.h PATH_SUFFIXES lua5.5 lua5.4 lua)
find_library(LUA_LIBRARY NAMES lua lua5.5 lua5.4)
if(NOT LUA_INCLUDE_DIR OR NOT LUA_LIBRARY)
  message(FATAL_ERROR "Lua development files not found")
endif()

add_library(skein STATIC
  src/core/jobs.cpp
  src/core/profiler.cpp
  src/ecs/world.cpp
  src/scene/scene.cpp
  src/scene/serialize.cpp
  src/assets/mesh.cpp
  src/assets/obj.cpp
  src/physics/physics.cpp
  src/render/culling.cpp
  src/script/lua_bindings.cpp
)
target_include_directories(skein PUBLIC src ${LUA_INCLUDE_DIR})
target_link_libraries(skein PUBLIC ${LUA_LIBRARY})
find_package(Threads REQUIRED)
target_link_libraries(skein PUBLIC Threads::Threads)

add_library(skein_app STATIC src/app/demo_scene.cpp src/app/camera.cpp)
target_link_libraries(skein_app PUBLIC skein)
target_compile_definitions(skein_app PUBLIC SKEIN_ASSET_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets")

find_package(glfw3 QUIET)
if(glfw3_FOUND)
  find_library(SKEIN_OPENGL OpenGL)
  add_library(skein_gl STATIC
    src/render/shader.cpp
    src/render/gpu_mesh.cpp
    src/render/renderer.cpp
    src/render/debug_draw.cpp
  )
  target_link_libraries(skein_gl PUBLIC skein_app glfw ${SKEIN_OPENGL})
  target_compile_definitions(skein_gl PUBLIC GL_SILENCE_DEPRECATION)

  add_executable(skein_demo src/app/main.cpp)
  target_link_libraries(skein_demo PRIVATE skein_gl)
else()
  message(STATUS "glfw3 not found; skipping the interactive skein_demo target")
endif()

add_executable(skein_bench bench/bench.cpp)
target_link_libraries(skein_bench PRIVATE skein_app)

add_executable(skein_tests
  tests/test_main.cpp
  tests/test_math.cpp
  tests/test_ecs.cpp
  tests/test_jobs.cpp
  tests/test_physics.cpp
  tests/test_scene.cpp
  tests/test_script.cpp
  tests/test_assets.cpp
)
target_include_directories(skein_tests PRIVATE tests)
target_link_libraries(skein_tests PRIVATE skein)

enable_testing()
add_test(NAME unit COMMAND skein_tests)
