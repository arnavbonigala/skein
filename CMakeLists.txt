cmake_minimum_required(VERSION 3.20)

if(APPLE AND NOT CMAKE_OSX_ARCHITECTURES)
  execute_process(COMMAND uname -m OUTPUT_VARIABLE SKEIN_HOST_ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(CMAKE_OSX_ARCHITECTURES ${SKEIN_HOST_ARCH} CACHE STRING "" FORCE)
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
