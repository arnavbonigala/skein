#include "app/camera.hpp"

namespace skein {

Vec3 Camera::forward() const {
    return normalize(Vec3{std::cos(pitch) * std::cos(yaw), std::sin(pitch), std::cos(pitch) * std::sin(yaw)});
}

Vec3 Camera::right() const { return normalize(cross(forward(), Vec3{0, 1, 0})); }

Mat4 Camera::view() const { return lookAt(position, position + forward(), Vec3{0, 1, 0}); }

Mat4 Camera::projection(float aspect) const {
    return perspective(fovY, aspect > 0.0f ? aspect : 1.0f, nearPlane, farPlane);
}

void Camera::look(float deltaYaw, float deltaPitch) {
    yaw += deltaYaw;
    pitch = std::clamp(pitch + deltaPitch, -PI * 0.49f, PI * 0.49f);
}

void Camera::move(const Vec3& localDelta, float dt, float boost) {
    Vec3 f = forward();
    Vec3 r = right();
    Vec3 step = r * localDelta.x + Vec3{0, 1, 0} * localDelta.y + f * localDelta.z;
    if (length2(step) > 0.0f) position += normalize(step) * (speed * boost * dt);
}

}  // namespace skein
