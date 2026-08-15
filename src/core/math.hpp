#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>

namespace skein {

constexpr float PI = 3.14159265358979323846f;

inline float radians(float deg) { return deg * (PI / 180.0f); }
inline float degrees(float rad) { return rad * (180.0f / PI); }

struct Vec2 {
    float x = 0, y = 0;
    Vec2() = default;
    Vec2(float x_, float y_) : x(x_), y(y_) {}
};

struct Vec3 {
    float x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(float s) : x(s), y(s), z(s) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    float& operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }
};

struct Vec4 {
    float x = 0, y = 0, z = 0, w = 0;
    Vec4() = default;
    Vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    Vec4(const Vec3& v, float w_) : x(v.x), y(v.y), z(v.z), w(w_) {}
    Vec3 xyz() const { return {x, y, z}; }
    float& operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3& a, const Vec3& b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline Vec3 operator/(const Vec3& a, const Vec3& b) { return {a.x / b.x, a.y / b.y, a.z / b.z}; }
inline Vec3 operator*(const Vec3& a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(float s, const Vec3& a) { return a * s; }
inline Vec3 operator/(const Vec3& a, float s) { return a * (1.0f / s); }
inline Vec3 operator-(const Vec3& a) { return {-a.x, -a.y, -a.z}; }
inline Vec3& operator+=(Vec3& a, const Vec3& b) { a = a + b; return a; }
inline Vec3& operator-=(Vec3& a, const Vec3& b) { a = a - b; return a; }
inline Vec3& operator*=(Vec3& a, float s) { a = a * s; return a; }
inline bool operator==(const Vec3& a, const Vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }

inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length2(const Vec3& v) { return dot(v, v); }
inline float length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalize(const Vec3& v) {
    float l2 = dot(v, v);
    if (l2 <= 1e-20f) return {0, 0, 0};
    return v * (1.0f / std::sqrt(l2));
}
inline Vec3 vmin(const Vec3& a, const Vec3& b) { return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)}; }
inline Vec3 vmax(const Vec3& a, const Vec3& b) { return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)}; }
inline Vec3 vabs(const Vec3& a) { return {std::fabs(a.x), std::fabs(a.y), std::fabs(a.z)}; }
inline Vec3 lerp(const Vec3& a, const Vec3& b, float t) { return a + (b - a) * t; }
inline float maxComponent(const Vec3& v) { return std::max(v.x, std::max(v.y, v.z)); }

inline Vec4 operator+(const Vec4& a, const Vec4& b) { return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w}; }
inline Vec4 operator*(const Vec4& a, float s) { return {a.x * s, a.y * s, a.z * s, a.w * s}; }
inline float dot(const Vec4& a, const Vec4& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

/// Unit quaternion, xyz vector part + w scalar part.
struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
    Quat() = default;
    Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    static Quat axisAngle(const Vec3& axis, float angle) {
        Vec3 n = normalize(axis);
        float h = angle * 0.5f;
        float s = std::sin(h);
        return {n.x * s, n.y * s, n.z * s, std::cos(h)};
    }
    static Quat euler(float pitch, float yaw, float roll) {
        float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
        float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
        float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
        return {sp * cy * cr - cp * sy * sr,
                cp * sy * cr + sp * cy * sr,
                cp * cy * sr - sp * sy * cr,
                cp * cy * cr + sp * sy * sr};
    }
};

inline Quat operator*(const Quat& a, const Quat& b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline Quat normalize(const Quat& q) {
    float l = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (l <= 1e-20f) return {};
    float inv = 1.0f / l;
    return {q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}
inline Quat conjugate(const Quat& q) { return {-q.x, -q.y, -q.z, q.w}; }
inline Vec3 rotate(const Quat& q, const Vec3& v) {
    Vec3 u{q.x, q.y, q.z};
    Vec3 t = cross(u, v) * 2.0f;
    return v + t * q.w + cross(u, t);
}

/// Column-major 4x4 matrix; m[c][r] matches GLSL memory layout.
struct Mat4 {
    float m[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

    static Mat4 identity() { return {}; }
    static Mat4 zero() {
        Mat4 r;
        for (int c = 0; c < 4; ++c)
            for (int j = 0; j < 4; ++j) r.m[c][j] = 0;
        return r;
    }
    const float* data() const { return &m[0][0]; }
};

inline Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r = Mat4::zero();
    for (int c = 0; c < 4; ++c)
        for (int k = 0; k < 4; ++k) {
            float bk = b.m[c][k];
            for (int j = 0; j < 4; ++j) r.m[c][j] += a.m[k][j] * bk;
        }
    return r;
}

inline Vec4 operator*(const Mat4& a, const Vec4& v) {
    Vec4 r{0, 0, 0, 0};
    for (int c = 0; c < 4; ++c) {
        float s = v[c];
        for (int j = 0; j < 4; ++j) (&r.x)[j] += a.m[c][j] * s;
    }
    return r;
}

inline Mat4 transpose(const Mat4& a) {
    Mat4 r;
    for (int c = 0; c < 4; ++c)
        for (int j = 0; j < 4; ++j) r.m[c][j] = a.m[j][c];
    return r;
}

inline Mat4 translation(const Vec3& t) {
    Mat4 r;
    r.m[3][0] = t.x;
    r.m[3][1] = t.y;
    r.m[3][2] = t.z;
    return r;
}

inline Mat4 scaling(const Vec3& s) {
    Mat4 r;
    r.m[0][0] = s.x;
    r.m[1][1] = s.y;
    r.m[2][2] = s.z;
    return r;
}

inline Mat4 toMat4(const Quat& q) {
    Mat4 r;
    float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    r.m[0][0] = 1 - 2 * (yy + zz); r.m[0][1] = 2 * (xy + wz);     r.m[0][2] = 2 * (xz - wy);
    r.m[1][0] = 2 * (xy - wz);     r.m[1][1] = 1 - 2 * (xx + zz); r.m[1][2] = 2 * (yz + wx);
    r.m[2][0] = 2 * (xz + wy);     r.m[2][1] = 2 * (yz - wx);     r.m[2][2] = 1 - 2 * (xx + yy);
    return r;
}

/// Builds T * R * S without a generic matrix multiply.
inline Mat4 composeTRS(const Vec3& t, const Quat& q, const Vec3& s) {
    Mat4 r = toMat4(q);
    for (int j = 0; j < 3; ++j) {
        r.m[0][j] *= s.x;
        r.m[1][j] *= s.y;
        r.m[2][j] *= s.z;
    }
    r.m[3][0] = t.x;
    r.m[3][1] = t.y;
    r.m[3][2] = t.z;
    return r;
}

inline Mat4 inverse(const Mat4& mat) {
    const float* a = mat.data();
    float inv[16];
    inv[0]  =  a[5]*a[10]*a[15] - a[5]*a[11]*a[14] - a[9]*a[6]*a[15] + a[9]*a[7]*a[14] + a[13]*a[6]*a[11] - a[13]*a[7]*a[10];
    inv[4]  = -a[4]*a[10]*a[15] + a[4]*a[11]*a[14] + a[8]*a[6]*a[15] - a[8]*a[7]*a[14] - a[12]*a[6]*a[11] + a[12]*a[7]*a[10];
    inv[8]  =  a[4]*a[9]*a[15]  - a[4]*a[11]*a[13] - a[8]*a[5]*a[15] + a[8]*a[7]*a[13] + a[12]*a[5]*a[11] - a[12]*a[7]*a[9];
    inv[12] = -a[4]*a[9]*a[14]  + a[4]*a[10]*a[13] + a[8]*a[5]*a[14] - a[8]*a[6]*a[13] - a[12]*a[5]*a[10] + a[12]*a[6]*a[9];
    inv[1]  = -a[1]*a[10]*a[15] + a[1]*a[11]*a[14] + a[9]*a[2]*a[15] - a[9]*a[3]*a[14] - a[13]*a[2]*a[11] + a[13]*a[3]*a[10];
    inv[5]  =  a[0]*a[10]*a[15] - a[0]*a[11]*a[14] - a[8]*a[2]*a[15] + a[8]*a[3]*a[14] + a[12]*a[2]*a[11] - a[12]*a[3]*a[10];
    inv[9]  = -a[0]*a[9]*a[15]  + a[0]*a[11]*a[13] + a[8]*a[1]*a[15] - a[8]*a[3]*a[13] - a[12]*a[1]*a[11] + a[12]*a[3]*a[9];
    inv[13] =  a[0]*a[9]*a[14]  - a[0]*a[10]*a[13] - a[8]*a[1]*a[14] + a[8]*a[2]*a[13] + a[12]*a[1]*a[10] - a[12]*a[2]*a[9];
    inv[2]  =  a[1]*a[6]*a[15]  - a[1]*a[7]*a[14]  - a[5]*a[2]*a[15] + a[5]*a[3]*a[14] + a[13]*a[2]*a[7]  - a[13]*a[3]*a[6];
    inv[6]  = -a[0]*a[6]*a[15]  + a[0]*a[7]*a[14]  + a[4]*a[2]*a[15] - a[4]*a[3]*a[14] - a[12]*a[2]*a[7]  + a[12]*a[3]*a[6];
    inv[10] =  a[0]*a[5]*a[15]  - a[0]*a[7]*a[13]  - a[4]*a[1]*a[15] + a[4]*a[3]*a[13] + a[12]*a[1]*a[7]  - a[12]*a[3]*a[5];
    inv[14] = -a[0]*a[5]*a[14]  + a[0]*a[6]*a[13]  + a[4]*a[1]*a[14] - a[4]*a[2]*a[13] - a[12]*a[1]*a[6]  + a[12]*a[2]*a[5];
    inv[3]  = -a[1]*a[6]*a[11]  + a[1]*a[7]*a[10]  + a[5]*a[2]*a[11] - a[5]*a[3]*a[10] - a[9]*a[2]*a[7]   + a[9]*a[3]*a[6];
    inv[7]  =  a[0]*a[6]*a[11]  - a[0]*a[7]*a[10]  - a[4]*a[2]*a[11] + a[4]*a[3]*a[10] + a[8]*a[2]*a[7]   - a[8]*a[3]*a[6];
    inv[11] = -a[0]*a[5]*a[11]  + a[0]*a[7]*a[9]   + a[4]*a[1]*a[11] - a[4]*a[3]*a[9]  - a[8]*a[1]*a[7]   + a[8]*a[3]*a[5];
    inv[15] =  a[0]*a[5]*a[10]  - a[0]*a[6]*a[9]   - a[4]*a[1]*a[10] + a[4]*a[2]*a[9]  + a[8]*a[1]*a[6]   - a[8]*a[2]*a[5];

    float det = a[0]*inv[0] + a[1]*inv[4] + a[2]*inv[8] + a[3]*inv[12];
    Mat4 out;
    if (std::fabs(det) < 1e-20f) return out;
    det = 1.0f / det;
    float* o = &out.m[0][0];
    for (int i = 0; i < 16; ++i) o[i] = inv[i] * det;
    return out;
}

/// Inverse of a matrix known to contain only rotation, uniform-or-not scale and translation.
inline Mat4 inverseAffine(const Mat4& mat) { return inverse(mat); }

inline Mat4 perspective(float fovY, float aspect, float zNear, float zFar) {
    float f = 1.0f / std::tan(fovY * 0.5f);
    Mat4 r = Mat4::zero();
    r.m[0][0] = f / aspect;
    r.m[1][1] = f;
    r.m[2][2] = (zFar + zNear) / (zNear - zFar);
    r.m[2][3] = -1.0f;
    r.m[3][2] = (2.0f * zFar * zNear) / (zNear - zFar);
    return r;
}

inline Mat4 orthographic(float l, float r_, float b, float t, float zn, float zf) {
    Mat4 r = Mat4::identity();
    r.m[0][0] = 2.0f / (r_ - l);
    r.m[1][1] = 2.0f / (t - b);
    r.m[2][2] = -2.0f / (zf - zn);
    r.m[3][0] = -(r_ + l) / (r_ - l);
    r.m[3][1] = -(t + b) / (t - b);
    r.m[3][2] = -(zf + zn) / (zf - zn);
    return r;
}

inline Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
    Vec3 f = normalize(center - eye);
    Vec3 s = normalize(cross(f, up));
    Vec3 u = cross(s, f);
    Mat4 r = Mat4::identity();
    r.m[0][0] = s.x; r.m[1][0] = s.y; r.m[2][0] = s.z;
    r.m[0][1] = u.x; r.m[1][1] = u.y; r.m[2][1] = u.z;
    r.m[0][2] = -f.x; r.m[1][2] = -f.y; r.m[2][2] = -f.z;
    r.m[3][0] = -dot(s, eye);
    r.m[3][1] = -dot(u, eye);
    r.m[3][2] = dot(f, eye);
    return r;
}

struct AABB {
    Vec3 min{std::numeric_limits<float>::max()};
    Vec3 max{-std::numeric_limits<float>::max()};

    bool valid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    Vec3 center() const { return (min + max) * 0.5f; }
    Vec3 extent() const { return (max - min) * 0.5f; }
    void expand(const Vec3& p) { min = vmin(min, p); max = vmax(max, p); }
    void expand(const AABB& b) { min = vmin(min, b.min); max = vmax(max, b.max); }
    float radius() const { return length(extent()); }
};

inline bool overlaps(const AABB& a, const AABB& b) {
    return a.min.x <= b.max.x && a.max.x >= b.min.x &&
           a.min.y <= b.max.y && a.max.y >= b.min.y &&
           a.min.z <= b.max.z && a.max.z >= b.min.z;
}

/// Maps a local-space AABB through an affine transform, keeping it axis aligned.
inline AABB transformAABB(const AABB& b, const Mat4& t) {
    Vec3 c = b.center();
    Vec3 e = b.extent();
    Vec3 nc{t.m[0][0] * c.x + t.m[1][0] * c.y + t.m[2][0] * c.z + t.m[3][0],
            t.m[0][1] * c.x + t.m[1][1] * c.y + t.m[2][1] * c.z + t.m[3][1],
            t.m[0][2] * c.x + t.m[1][2] * c.y + t.m[2][2] * c.z + t.m[3][2]};
    Vec3 ne{std::fabs(t.m[0][0]) * e.x + std::fabs(t.m[1][0]) * e.y + std::fabs(t.m[2][0]) * e.z,
            std::fabs(t.m[0][1]) * e.x + std::fabs(t.m[1][1]) * e.y + std::fabs(t.m[2][1]) * e.z,
            std::fabs(t.m[0][2]) * e.x + std::fabs(t.m[1][2]) * e.y + std::fabs(t.m[2][2]) * e.z};
    AABB out;
    out.min = nc - ne;
    out.max = nc + ne;
    return out;
}

/// Plane in the form dot(n, p) + d = 0 with a normalized normal.
struct Plane {
    Vec3 n{0, 1, 0};
    float d = 0;
    float distance(const Vec3& p) const { return dot(n, p) + d; }
};

struct Frustum {
    Plane planes[6];
};

/// Gribb-Hartmann plane extraction from a combined view-projection matrix.
inline Frustum extractFrustum(const Mat4& vp) {
    auto row = [&](int j) { return Vec4{vp.m[0][j], vp.m[1][j], vp.m[2][j], vp.m[3][j]}; };
    Vec4 r0 = row(0), r1 = row(1), r2 = row(2), r3 = row(3);
    Vec4 raw[6] = {
        {r3.x + r0.x, r3.y + r0.y, r3.z + r0.z, r3.w + r0.w},
        {r3.x - r0.x, r3.y - r0.y, r3.z - r0.z, r3.w - r0.w},
        {r3.x + r1.x, r3.y + r1.y, r3.z + r1.z, r3.w + r1.w},
        {r3.x - r1.x, r3.y - r1.y, r3.z - r1.z, r3.w - r1.w},
        {r3.x + r2.x, r3.y + r2.y, r3.z + r2.z, r3.w + r2.w},
        {r3.x - r2.x, r3.y - r2.y, r3.z - r2.z, r3.w - r2.w},
    };
    Frustum f;
    for (int i = 0; i < 6; ++i) {
        float l = length(raw[i].xyz());
        if (l <= 1e-20f) l = 1.0f;
        f.planes[i].n = raw[i].xyz() / l;
        f.planes[i].d = raw[i].w / l;
    }
    return f;
}

/// Conservative center/extent frustum test; false means definitely outside.
inline bool frustumIntersectsAABB(const Frustum& f, const Vec3& center, const Vec3& extent) {
    for (int i = 0; i < 6; ++i) {
        const Plane& p = f.planes[i];
        float r = extent.x * std::fabs(p.n.x) + extent.y * std::fabs(p.n.y) + extent.z * std::fabs(p.n.z);
        if (dot(p.n, center) + p.d < -r) return false;
    }
    return true;
}

inline bool frustumIntersectsSphere(const Frustum& f, const Vec3& center, float radius) {
    for (int i = 0; i < 6; ++i)
        if (f.planes[i].distance(center) < -radius) return false;
    return true;
}

}  // namespace skein
