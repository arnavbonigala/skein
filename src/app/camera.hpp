#pragma once
#include "core/math.hpp"

namespace skein {

/// Free-flying camera driven by yaw/pitch in radians.
struct Camera {
    Vec3 position{0.0f, 18.0f, 90.0f};
    float yaw = -PI * 0.5f;
    float pitch = -0.18f;
    float fovY = radians(65.0f);
    float nearPlane = 0.2f;
    float farPlane = 600.0f;
    float speed = 28.0f;

    Vec3 forward() const;
    Vec3 right() const;
    Mat4 view() const;
    Mat4 projection(float aspect) const;

    void look(float deltaYaw, float deltaPitch);
    void move(const Vec3& localDelta, float dt, float boost);
};

}  // namespace skein
