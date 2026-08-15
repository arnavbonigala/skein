#include "app/demo_scene.hpp"

#include <cstdio>
#include <random>

#include "core/jobs.hpp"
#include "core/profiler.hpp"

namespace skein {
namespace {

Material makeMaterial(const char* name, Vec3 albedo, float roughness, float metallic, Vec3 emissive = {0, 0, 0}) {
    Material m;
    m.name = name;
    m.albedo = albedo;
    m.roughness = roughness;
    m.metallic = metallic;
    m.emissive = emissive;
    return m;
}

}  // namespace

void Demo::build(const DemoConfig& cfg, JobSystem* jobs) {
    SKEIN_PROFILE("demo/build");
    config = cfg;
    fieldMin_ = Vec3{-cfg.fieldExtent, 0.0f, -cfg.fieldExtent};
    fieldMax_ = Vec3{cfg.fieldExtent, cfg.fieldHeight, cfg.fieldExtent};

    meshIds_.clear();
    meshIds_.push_back(assets.addMesh(primitives::cube(1.0f)));
    meshIds_.push_back(assets.addMesh(primitives::sphere(0.5f, 20, 14)));
    meshIds_.push_back(assets.addMesh(primitives::cone(0.5f, 1.2f, 18)));
    std::vector<std::string> objPaths = cfg.objPaths;
    if (objPaths.empty()) {
        objPaths.push_back(std::string(SKEIN_ASSET_DIR) + "/meshes/geosphere.obj");
        objPaths.push_back(std::string(SKEIN_ASSET_DIR) + "/meshes/pylon.obj");
    }
    std::vector<std::string> objErrors;
    for (uint32_t id : assets.loadObjBatch(objPaths, jobs, &objErrors)) meshIds_.push_back(id);
    for (const std::string& e : objErrors) std::fprintf(stderr, "[assets] %s\n", e.c_str());

    MeshData slab = primitives::cube(1.0f);
    slab.name = "slab";
    for (Vertex& v : slab.vertices) v.position = v.position * Vec3{1.6f, 0.35f, 1.6f};
    slab.recomputeBounds();
    meshIds_.push_back(assets.addMesh(std::move(slab)));

    assets.addMaterial(makeMaterial("steel", Vec3{0.62f, 0.66f, 0.72f}, 0.32f, 0.85f));
    assets.addMaterial(makeMaterial("copper", Vec3{0.85f, 0.45f, 0.22f}, 0.38f, 0.9f));
    assets.addMaterial(makeMaterial("slate", Vec3{0.24f, 0.26f, 0.30f}, 0.75f, 0.0f));
    assets.addMaterial(makeMaterial("moss", Vec3{0.28f, 0.42f, 0.24f}, 0.85f, 0.0f));
    assets.addMaterial(makeMaterial("sand", Vec3{0.76f, 0.68f, 0.48f}, 0.7f, 0.0f));
    assets.addMaterial(makeMaterial("ember", Vec3{0.9f, 0.35f, 0.15f}, 0.5f, 0.0f, Vec3{0.7f, 0.22f, 0.06f}));

    scene.world.reserve(static_cast<size_t>(cfg.entityCount) + 64);

    std::mt19937 rng(cfg.seed);
    std::uniform_real_distribution<float> planar(-cfg.fieldExtent, cfg.fieldExtent);
    std::uniform_real_distribution<float> vertical(1.0f, cfg.fieldHeight);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    std::uniform_real_distribution<float> signedUnit(-1.0f, 1.0f);

    std::vector<Entity> renderables;
    renderables.reserve(static_cast<size_t>(cfg.renderableCount));

    const float pileExtent = cfg.fieldExtent * 0.14f;
    std::uniform_real_distribution<float> pilePlanar(-pileExtent, pileExtent);
    std::uniform_real_distribution<float> pileVertical(1.0f, cfg.fieldHeight * 0.55f);

    // Renderables are spread across the whole entity range rather than taken
    // from the front, so the visible set covers both the dense simulated pile
    // and the wide scattered field instead of sitting entirely inside one.
    const int renderStride = std::max(1, cfg.entityCount / std::max(1, cfg.renderableCount));
    int rendered = 0;

    for (int i = 0; i < cfg.entityCount; ++i) {
        Transform t;
        const bool simulated = i < cfg.colliderCount;
        const bool visible = (i % renderStride) == 0 && rendered < cfg.renderableCount;
        t.position = simulated ? Vec3{pilePlanar(rng), pileVertical(rng), pilePlanar(rng)}
                               : Vec3{planar(rng), vertical(rng), planar(rng)};
        float s = 0.6f + unit(rng) * 1.6f;
        t.scale = Vec3{s, s * (0.7f + unit(rng) * 0.7f), s};
        t.rotation = Quat::euler(signedUnit(rng) * PI, signedUnit(rng) * PI, 0.0f);
        Entity e = scene.create(t);

        scene.world.add<Velocity>(
            e, Velocity{Vec3{signedUnit(rng) * 3.0f, signedUnit(rng) * 2.0f, signedUnit(rng) * 3.0f},
                        Vec3{signedUnit(rng) * 0.8f, signedUnit(rng) * 1.2f, signedUnit(rng) * 0.8f}});

        if (visible) {
            ++rendered;
            uint32_t mesh = meshIds_[static_cast<size_t>(rng() % meshIds_.size())];
            uint32_t material = static_cast<uint32_t>(rng() % assets.materialCount());
            scene.world.add<Renderable>(e, Renderable{mesh, material, 1, 0});

            CullBounds cb;
            const AABB& b = assets.mesh(mesh).bounds;
            cb.localCenter = b.center();
            cb.localExtent = b.extent();
            cb.center = cb.localCenter;
            cb.extent = cb.localExtent;
            cb.radius = length(cb.extent);
            scene.world.add<CullBounds>(e, cb);
            renderables.push_back(e);
        }

        if (simulated) {
            Collider c;
            bool box = (rng() % 3) == 0;
            c.kind = box ? static_cast<uint32_t>(ColliderKind::Box) : static_cast<uint32_t>(ColliderKind::Sphere);
            c.radius = 0.5f;
            c.halfExtents = Vec3{0.5f, 0.5f, 0.5f};
            c.invMass = 1.0f;
            c.restitution = 0.25f + unit(rng) * 0.4f;
            scene.world.add<Collider>(e, c);
        }
    }

    for (int i = 0; i < cfg.hierarchyChildren && !renderables.empty(); ++i) {
        Entity parent = renderables[static_cast<size_t>(rng()) % renderables.size()];
        Transform t;
        t.position = Vec3{signedUnit(rng) * 2.0f, 1.2f + unit(rng), signedUnit(rng) * 2.0f};
        t.scale = Vec3{0.35f, 0.35f, 0.35f};
        Entity child = scene.create(t, parent);
        uint32_t mesh = meshIds_[static_cast<size_t>(rng() % meshIds_.size())];
        scene.world.add<Renderable>(child, Renderable{mesh, static_cast<uint32_t>(rng() % assets.materialCount()), 1, 0});
        CullBounds cb;
        const AABB& b = assets.mesh(mesh).bounds;
        cb.localCenter = b.center();
        cb.localExtent = b.extent();
        cb.center = cb.localCenter;
        cb.extent = cb.localExtent;
        cb.radius = length(cb.extent);
        scene.world.add<CullBounds>(child, cb);
    }

    Transform sunTransform;
    Entity sun = scene.create(sunTransform);
    Light sunLight;
    sunLight.kind = static_cast<uint32_t>(LightKind::Directional);
    sunLight.direction = normalize(Vec3{-0.4f, -0.85f, -0.35f});
    sunLight.color = Vec3{1.0f, 0.95f, 0.86f};
    sunLight.intensity = 2.4f;
    scene.world.add<Light>(sun, sunLight);

    for (int i = 0; i < cfg.pointLights; ++i) {
        Transform t;
        t.position = Vec3{planar(rng) * 0.5f, 3.0f + unit(rng) * 20.0f, planar(rng) * 0.5f};
        Entity e = scene.create(t);
        Light l;
        l.kind = static_cast<uint32_t>(LightKind::Point);
        l.color = Vec3{0.4f + unit(rng) * 0.6f, 0.35f + unit(rng) * 0.5f, 0.3f + unit(rng) * 0.7f};
        l.intensity = 6.0f + unit(rng) * 10.0f;
        l.range = 26.0f + unit(rng) * 24.0f;
        scene.world.add<Light>(e, l);
        scene.world.add<Velocity>(e, Velocity{Vec3{signedUnit(rng) * 4.0f, 0, signedUnit(rng) * 4.0f}, Vec3{}});
    }

    physics.settings.boundsMin = fieldMin_;
    physics.settings.boundsMax = fieldMax_;
    // Measured on this scene: the scan walks a cell's entries contiguously, so
    // fewer, fatter cells beat more, tighter ones until about here.
    physics.settings.cellSize = 4.0f;

    script.bind(&scene, &assets);
    scene.updateTransforms(jobs);
}

bool Demo::loadScript(const std::string& path, std::string& error) {
    return script.doFile(path, error);
}

void Demo::stepKinematics(float dt, JobSystem* jobs) {
    SKEIN_PROFILE("ecs/kinematics");
    Pool<Velocity>& velocities = scene.world.pool<Velocity>();
    Pool<Transform>& transforms = scene.world.pool<Transform>();
    Pool<Collider>& colliders = scene.world.pool<Collider>();

    const Vec3 lo = fieldMin_;
    const Vec3 hi = fieldMax_;

    auto body = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            Entity e = velocities.dense[i];
            if (colliders.contains(e)) continue;
            Transform* t = transforms.tryGet(e);
            if (!t) continue;
            Velocity& v = velocities.data[i];
            t->position += v.linear * dt;
            for (int axis = 0; axis < 3; ++axis) {
                if (t->position[axis] < lo[axis]) {
                    t->position[axis] = lo[axis];
                    v.linear[axis] = std::fabs(v.linear[axis]);
                } else if (t->position[axis] > hi[axis]) {
                    t->position[axis] = hi[axis];
                    v.linear[axis] = -std::fabs(v.linear[axis]);
                }
            }
            if (length2(v.angular) > 0.0f) {
                Quat spin = Quat::axisAngle(v.angular, length(v.angular) * dt);
                t->rotation = normalize(spin * t->rotation);
            }
        }
    };

    size_t n = velocities.dense.size();
    if (jobs && n >= 8192)
        jobs->parallelFor(n, 4096, body);
    else
        body(0, n);
}

void Demo::update(float dt, JobSystem* jobs) {
    SKEIN_PROFILE("demo/update");
    if (config.runScripts) script.update(dt);
    stepKinematics(dt, jobs);
    physics.step(scene, dt, jobs);
    scene.updateTransforms(jobs);
}

size_t Demo::bytesUsed() const {
    return scene.world.bytesUsed() + assets.bytesUsed() + physics.bytesUsed() + script.memoryBytes();
}

}  // namespace skein
