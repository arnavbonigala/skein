#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace skein {

class Scene;

/// Binary scene format. Component pools are written as raw arrays keyed by the
/// name they were registered under, so a build that adds or drops a component
/// type can still load older files. Entity ids are preserved exactly, which
/// keeps stored entity references such as Parent valid. Little-endian hosts only.
constexpr uint32_t SCENE_MAGIC = 0x314e4b53;  // "SKN1"
constexpr uint32_t SCENE_VERSION = 1;

std::vector<uint8_t> serializeScene(const Scene& scene);
bool deserializeScene(Scene& scene, const uint8_t* data, size_t size, std::string& error);

bool saveScene(const Scene& scene, const std::string& path, std::string& error);
bool loadScene(Scene& scene, const std::string& path, std::string& error);

}  // namespace skein
