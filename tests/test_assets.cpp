#include "assets/mesh.hpp"
#include "test.hpp"

#include <cstdio>
#include <fstream>
#include <set>

#include "core/jobs.hpp"

using namespace skein;

namespace {

const char* QUAD_WITH_NORMALS = R"(
# a unit quad
v -1 -1 0
v  1 -1 0
v  1  1 0
v -1  1 0
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn 0 0 1
f 1/1/1 2/2/1 3/3/1 4/4/1
)";

const char* TRIANGLE_NEGATIVE_INDICES = R"(
v 0 0 0
v 1 0 0
v 0 1 0
f -3 -2 -1
)";

std::string writeTemp(const std::string& name, const std::string& contents) {
    std::string path = std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") + "/" + name;
    std::ofstream out(path);
    out << contents;
    return path;
}

}  // namespace

TEST(obj_quads_are_triangulated_and_vertices_welded) {
    MeshData mesh;
    std::string error;
    CHECK(parseObj(QUAD_WITH_NORMALS, mesh, error));
    CHECK_EQ(mesh.vertices.size(), size_t{4});
    CHECK_EQ(mesh.indices.size(), size_t{6});
    for (const Vertex& v : mesh.vertices) CHECK_NEAR(v.normal.z, 1.0, 1e-6);
    CHECK_NEAR(mesh.bounds.min.x, -1.0, 1e-6);
    CHECK_NEAR(mesh.bounds.max.y, 1.0, 1e-6);
}

TEST(obj_negative_indices_reference_from_the_end) {
    MeshData mesh;
    std::string error;
    CHECK(parseObj(TRIANGLE_NEGATIVE_INDICES, mesh, error));
    CHECK_EQ(mesh.indices.size(), size_t{3});
    CHECK_NEAR(mesh.vertices[mesh.indices[1]].position.x, 1.0, 1e-6);
    CHECK_NEAR(mesh.vertices[mesh.indices[2]].position.y, 1.0, 1e-6);
}

TEST(obj_without_normals_gets_generated_ones) {
    MeshData mesh;
    std::string error;
    CHECK(parseObj("v 0 0 0\nv 1 0 0\nv 0 0 -1\nf 1 2 3\n", mesh, error));
    CHECK_NEAR(length(mesh.vertices[0].normal), 1.0, 1e-5);
    CHECK_NEAR(std::fabs(mesh.vertices[0].normal.y), 1.0, 1e-5);
}

TEST(obj_shared_vertices_are_not_duplicated_across_faces) {
    MeshData mesh;
    std::string error;
    CHECK(parseObj("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nf 1 2 3\nf 1 3 4\n", mesh, error));
    CHECK_EQ(mesh.vertices.size(), size_t{4});
    CHECK_EQ(mesh.indices.size(), size_t{6});
}

TEST(obj_rejects_malformed_input) {
    MeshData mesh;
    std::string error;
    CHECK(!parseObj("v 0 0\nf 1 1 1\n", mesh, error));
    CHECK(!error.empty());

    error.clear();
    CHECK(!parseObj("v 0 0 0\nf 1 99 3\n", mesh, error));
    CHECK(error.find("out of range") != std::string::npos);

    error.clear();
    CHECK(!parseObj("# nothing here\n", mesh, error));
}

TEST(obj_face_slash_forms_all_parse) {
    MeshData a, b, c;
    std::string error;
    const char* header = "v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nvt 1 0\nvt 0 1\nvn 0 0 1\n";
    CHECK(parseObj(std::string(header) + "f 1 2 3\n", a, error));
    CHECK(parseObj(std::string(header) + "f 1/1 2/2 3/3\n", b, error));
    CHECK(parseObj(std::string(header) + "f 1//1 2//1 3//1\n", c, error));
    CHECK_EQ(a.indices.size(), size_t{3});
    CHECK_EQ(b.indices.size(), size_t{3});
    CHECK_EQ(c.indices.size(), size_t{3});
    CHECK_NEAR(c.vertices[0].normal.z, 1.0, 1e-6);
    CHECK_NEAR(b.vertices[1].uv.x, 1.0, 1e-6);
}

TEST(primitive_meshes_are_closed_and_correctly_bounded) {
    MeshData cube = primitives::cube(2.0f);
    CHECK_NEAR(cube.bounds.min.x, -1.0, 1e-5);
    CHECK_NEAR(cube.bounds.max.z, 1.0, 1e-5);
    CHECK_EQ(cube.indices.size(), size_t{36});

    MeshData sphere = primitives::sphere(1.5f, 16, 12);
    for (const Vertex& v : sphere.vertices) {
        CHECK_NEAR(length(v.position), 1.5, 1e-4);
        CHECK_NEAR(dot(normalize(v.position), v.normal), 1.0, 1e-4);
    }
    CHECK_NEAR(sphere.bounds.max.y, 1.5, 1e-4);
}

TEST(primitive_winding_is_consistent_and_outward) {
    MeshData cube = primitives::cube(1.0f);
    for (size_t i = 0; i + 2 < cube.indices.size(); i += 3) {
        const Vec3& a = cube.vertices[cube.indices[i]].position;
        const Vec3& b = cube.vertices[cube.indices[i + 1]].position;
        const Vec3& c = cube.vertices[cube.indices[i + 2]].position;
        Vec3 faceNormal = normalize(cross(b - a, c - a));
        Vec3 stored = cube.vertices[cube.indices[i]].normal;
        CHECK_NEAR(dot(faceNormal, stored), 1.0, 1e-4);
        CHECK(dot(faceNormal, (a + b + c) / 3.0f) > 0.0f);
    }
}

TEST(batch_loading_registers_meshes_in_a_stable_order) {
    std::vector<std::string> paths;
    for (int i = 0; i < 8; ++i)
        paths.push_back(writeTemp("skein_test_mesh_" + std::to_string(i) + ".obj",
                                  "v 0 0 0\nv " + std::to_string(i + 1) + " 0 0\nv 0 1 0\nf 1 2 3\n"));
    paths.push_back(writeTemp("skein_test_bad.obj", "not an obj\n"));
    paths.push_back("/definitely/not/here.obj");

    JobSystem jobs(4);
    Assets assets;
    std::vector<std::string> errors;
    std::vector<uint32_t> ids = assets.loadObjBatch(paths, &jobs, &errors);

    CHECK_EQ(ids.size(), size_t{8});
    CHECK_EQ(errors.size(), size_t{2});
    for (int i = 0; i < 8; ++i) {
        uint32_t id = assets.findMesh("skein_test_mesh_" + std::to_string(i));
        CHECK(id != ~0u);
        CHECK_NEAR(assets.mesh(id).bounds.max.x, static_cast<double>(i + 1), 1e-5);
    }
    for (const std::string& p : paths) std::remove(p.c_str());
}

TEST(asset_handles_are_dense_and_reusable_by_name) {
    Assets assets;
    uint32_t cube = assets.addMesh(primitives::cube());
    Material m;
    m.name = "brass";
    m.albedo = Vec3{0.7f, 0.6f, 0.2f};
    uint32_t brass = assets.addMaterial(m);

    CHECK_EQ(assets.findMesh("cube"), cube);
    CHECK_EQ(assets.findMaterial("brass"), brass);
    CHECK_EQ(assets.findMesh("missing"), ~0u);
    CHECK(assets.bytesUsed() > 0);
}
