#include "core/math.hpp"
#include "test.hpp"

#include <random>

using namespace skein;

namespace {

Mat4 randomAffine(std::mt19937& rng) {
    std::uniform_real_distribution<float> pos(-10.0f, 10.0f);
    std::uniform_real_distribution<float> scale(0.2f, 3.0f);
    std::uniform_real_distribution<float> angle(-PI, PI);
    Quat q = Quat::euler(angle(rng), angle(rng), angle(rng));
    return composeTRS(Vec3{pos(rng), pos(rng), pos(rng)}, q, Vec3{scale(rng), scale(rng), scale(rng)});
}

/// Reference frustum test: a box is outside only if every corner sits behind a
/// single plane. Slower and slightly less conservative than the center/extent
/// form, but unambiguous.
bool referenceOutside(const Frustum& f, const AABB& box) {
    for (int p = 0; p < 6; ++p) {
        bool allBehind = true;
        for (int c = 0; c < 8 && allBehind; ++c) {
            Vec3 corner{(c & 1) ? box.max.x : box.min.x, (c & 2) ? box.max.y : box.min.y,
                        (c & 4) ? box.max.z : box.min.z};
            if (f.planes[p].distance(corner) >= 0) allBehind = false;
        }
        if (allBehind) return true;
    }
    return false;
}

}  // namespace

TEST(mat4_inverse_round_trips) {
    std::mt19937 rng(7);
    for (int i = 0; i < 64; ++i) {
        Mat4 m = randomAffine(rng);
        Mat4 id = m * inverse(m);
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r) CHECK_NEAR(id.m[c][r], c == r ? 1.0f : 0.0f, 1e-3);
    }
}

TEST(quaternion_rotation_matches_matrix) {
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> a(-PI, PI);
    std::uniform_real_distribution<float> c(-5.0f, 5.0f);
    for (int i = 0; i < 64; ++i) {
        Quat q = Quat::euler(a(rng), a(rng), a(rng));
        Vec3 v{c(rng), c(rng), c(rng)};
        Vec3 byQuat = rotate(q, v);
        Vec4 byMat = toMat4(q) * Vec4{v, 1.0f};
        CHECK_NEAR(byQuat.x, byMat.x, 1e-4);
        CHECK_NEAR(byQuat.y, byMat.y, 1e-4);
        CHECK_NEAR(byQuat.z, byMat.z, 1e-4);
    }
}

TEST(quaternion_rotation_preserves_length) {
    Quat q = Quat::axisAngle(Vec3{1, 2, 3}, 1.234f);
    Vec3 v{0.3f, -2.0f, 5.0f};
    CHECK_NEAR(length(rotate(q, v)), length(v), 1e-4);
}

TEST(transform_aabb_contains_transformed_corners) {
    std::mt19937 rng(23);
    std::uniform_real_distribution<float> s(-2.0f, 2.0f);
    for (int i = 0; i < 64; ++i) {
        AABB local;
        for (int k = 0; k < 4; ++k) local.expand(Vec3{s(rng), s(rng), s(rng)});
        Mat4 m = randomAffine(rng);
        AABB world = transformAABB(local, m);
        for (int c = 0; c < 8; ++c) {
            Vec3 corner{(c & 1) ? local.max.x : local.min.x, (c & 2) ? local.max.y : local.min.y,
                        (c & 4) ? local.max.z : local.min.z};
            Vec4 p = m * Vec4{corner, 1.0f};
            CHECK(p.x >= world.min.x - 1e-3f && p.x <= world.max.x + 1e-3f);
            CHECK(p.y >= world.min.y - 1e-3f && p.y <= world.max.y + 1e-3f);
            CHECK(p.z >= world.min.z - 1e-3f && p.z <= world.max.z + 1e-3f);
        }
    }
}

TEST(frustum_never_culls_a_visible_box) {
    Mat4 proj = perspective(radians(60.0f), 16.0f / 9.0f, 0.1f, 200.0f);
    Mat4 view = lookAt(Vec3{0, 10, 30}, Vec3{0, 0, 0}, Vec3{0, 1, 0});
    Frustum f = extractFrustum(proj * view);

    std::mt19937 rng(31);
    std::uniform_real_distribution<float> pos(-150.0f, 150.0f);
    std::uniform_real_distribution<float> ext(0.1f, 6.0f);
    int inside = 0, outside = 0;
    for (int i = 0; i < 20000; ++i) {
        Vec3 c{pos(rng), pos(rng), pos(rng)};
        Vec3 e{ext(rng), ext(rng), ext(rng)};
        AABB box;
        box.min = c - e;
        box.max = c + e;
        bool kept = frustumIntersectsAABB(f, c, e);
        if (kept)
            ++inside;
        else
            ++outside;
        if (!kept) CHECK(referenceOutside(f, box));
    }
    CHECK(inside > 0);
    CHECK(outside > inside);
}

TEST(frustum_sphere_test_agrees_with_box_test) {
    Mat4 vp = perspective(radians(45.0f), 1.0f, 0.5f, 100.0f) *
              lookAt(Vec3{5, 5, 5}, Vec3{0, 0, 0}, Vec3{0, 1, 0});
    Frustum f = extractFrustum(vp);
    std::mt19937 rng(41);
    std::uniform_real_distribution<float> pos(-80.0f, 80.0f);
    for (int i = 0; i < 20000; ++i) {
        Vec3 c{pos(rng), pos(rng), pos(rng)};
        Vec3 e{1.0f, 1.0f, 1.0f};
        if (frustumIntersectsAABB(f, c, e)) CHECK(frustumIntersectsSphere(f, c, length(e)));
    }
}

TEST(frustum_classification_agrees_with_the_per_corner_verdict) {
    Mat4 vp = perspective(radians(60.0f), 1.6f, 0.5f, 120.0f) *
              lookAt(Vec3{6, 9, 30}, Vec3{0, 0, 0}, Vec3{0, 1, 0});
    Frustum f = extractFrustum(vp);
    std::mt19937 rng(913);
    std::uniform_real_distribution<float> pos(-70.0f, 70.0f);
    std::uniform_real_distribution<float> size(0.5f, 9.0f);
    int inside = 0, outside = 0, straddling = 0;
    for (int i = 0; i < 20000; ++i) {
        Vec3 c{pos(rng), pos(rng), pos(rng)};
        Vec3 e{size(rng), size(rng), size(rng)};
        FrustumFit fit = frustumClassifyAABB(f, c, e);
        // Inside must mean no corner can be outside, which is what lets a
        // cluster skip every per-object test; Outside must agree with the plain
        // AABB test, or culling would drop a visible object.
        if (fit == FrustumFit::Inside) {
            ++inside;
            for (int corner = 0; corner < 8; ++corner) {
                Vec3 p{c.x + ((corner & 1) ? e.x : -e.x), c.y + ((corner & 2) ? e.y : -e.y),
                       c.z + ((corner & 4) ? e.z : -e.z)};
                CHECK(frustumIntersectsAABB(f, p, Vec3{0, 0, 0}));
            }
        } else if (fit == FrustumFit::Outside) {
            ++outside;
            CHECK(!frustumIntersectsAABB(f, c, e));
        } else {
            ++straddling;
            CHECK(frustumIntersectsAABB(f, c, e));
        }
    }
    CHECK(inside > 0);
    CHECK(outside > 0);
    CHECK(straddling > 0);
}

TEST(perspective_maps_near_and_far_to_clip_range) {
    Mat4 p = perspective(radians(70.0f), 1.5f, 0.25f, 500.0f);
    Vec4 nearPoint = p * Vec4{Vec3{0, 0, -0.25f}, 1.0f};
    Vec4 farPoint = p * Vec4{Vec3{0, 0, -500.0f}, 1.0f};
    CHECK_NEAR(nearPoint.z / nearPoint.w, -1.0, 1e-4);
    CHECK_NEAR(farPoint.z / farPoint.w, 1.0, 1e-4);
}

TEST(look_at_places_camera_at_origin_of_view_space) {
    Vec3 eye{3, 4, 5};
    Mat4 v = lookAt(eye, Vec3{0, 0, 0}, Vec3{0, 1, 0});
    Vec4 p = v * Vec4{eye, 1.0f};
    CHECK_NEAR(p.x, 0.0, 1e-4);
    CHECK_NEAR(p.y, 0.0, 1e-4);
    CHECK_NEAR(p.z, 0.0, 1e-4);
}

TEST(compose_trs_matches_explicit_multiply) {
    Vec3 t{1, -2, 3};
    Quat q = Quat::euler(0.4f, -1.2f, 0.7f);
    Vec3 s{2.0f, 0.5f, 1.5f};
    Mat4 composed = composeTRS(t, q, s);
    Mat4 explicitly = translation(t) * toMat4(q) * scaling(s);
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) CHECK_NEAR(composed.m[c][r], explicitly.m[c][r], 1e-5);
}
