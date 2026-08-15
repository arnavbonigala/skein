#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>
#include <cmath>
#include <string>
#include <tuple>
#include <thread>
#include <vector>

#include "app/demo_scene.hpp"
#include "core/jobs.hpp"
#include "core/profiler.hpp"
#include "render/render_list.hpp"
#include "scene/serialize.hpp"

using namespace skein;

namespace {

struct Timing {
    double median = 0;
    double best = 0;
    double worst = 0;
};

/// Runs `fn` `iterations` times after `warmup` untimed passes and reports the
/// median, which is far more stable than a mean on a laptop.
template <typename Fn>
Timing measure(int warmup, int iterations, Fn&& fn) {
    for (int i = 0; i < warmup; ++i) fn();
    std::vector<double> samples;
    samples.reserve(static_cast<size_t>(iterations));
    for (int i = 0; i < iterations; ++i) {
        Clock::time_point start = Clock::now();
        fn();
        samples.push_back(millisSince(start));
    }
    std::sort(samples.begin(), samples.end());
    Timing t;
    t.median = samples[samples.size() / 2];
    t.best = samples.front();
    t.worst = samples.back();
    return t;
}

void heading(const char* title) { std::printf("\n\033[1m%s\033[0m\n", title); }

void row(const char* label, double ms, const std::string& note = {}) {
    std::printf("  %-38s %9.3f ms   %s\n", label, ms, note.c_str());
}

/// Same column layout as `row` but for facts that are not a duration.
void fact(const char* label, const std::string& note) {
    std::printf("  %-38s %9s      %s\n", label, "", note.c_str());
}

std::string format(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    return buf;
}

std::string perEntity(double ms, size_t n) {
    return format("%.1f ns/entity", ms * 1e6 / static_cast<double>(n));
}

std::string speedup(double baselineMs, double ms) { return format("%.2fx", baselineMs / ms); }

std::string bytes(size_t n) {
    double mb = static_cast<double>(n) / (1024.0 * 1024.0);
    return mb < 1.0 ? format("%.0f KB", static_cast<double>(n) / 1024.0) : format("%.1f MB", mb);
}

Mat4 benchViewProj() {
    return perspective(radians(65.0f), 16.0f / 9.0f, 0.2f, 600.0f) *
           lookAt(Vec3{0, 22, 120}, Vec3{0, 6, 0}, Vec3{0, 1, 0});
}

/// The classic scene-graph shape the ECS is being compared against: one
/// polymorphic object per entity, allocated individually and visited through a
/// pointer array.
struct SceneNode {
    virtual ~SceneNode() = default;
    virtual void update(float dt) = 0;
    Vec3 position;
    Quat rotation;
    Vec3 scale{1, 1, 1};
    Vec3 linear;
    Vec3 angular;
    Mat4 world;
    char name[32] = {};
    uint32_t mesh = 0;
    uint32_t material = 0;
    bool visible = true;
};

struct MovingNode final : SceneNode {
    void update(float dt) override {
        position += linear * dt;
        if (length2(angular) > 0.0f) rotation = normalize(Quat::axisAngle(angular, length(angular) * dt) * rotation);
    }
};

/// Same fields, same work, but stored contiguously and called directly.
struct FlatNode {
    Vec3 position;
    Quat rotation;
    Vec3 scale{1, 1, 1};
    Vec3 linear;
    Vec3 angular;
    Mat4 world;
    char name[32] = {};
    uint32_t mesh = 0;
    uint32_t material = 0;
    bool visible = true;

    void update(float dt) {
        position += linear * dt;
        if (length2(angular) > 0.0f) rotation = normalize(Quat::axisAngle(angular, length(angular) * dt) * rotation);
    }
};

}  // namespace

int main(int argc, char** argv) {
    DemoConfig config;
    config.runScripts = false;
    int iterations = 40;
    bool runSweep = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--entities") == 0 && i + 1 < argc) config.entityCount = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--renderables") == 0 && i + 1 < argc) config.renderableCount = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--colliders") == 0 && i + 1 < argc) config.colliderCount = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--iterations") == 0 && i + 1 < argc) iterations = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--sweep") == 0) runSweep = true;
    }

    const unsigned hw = std::thread::hardware_concurrency();
    JobSystem jobs;

    std::printf("\033[1mskein benchmark\033[0m\n");
    std::printf("  hardware threads      %u\n", hw);
    std::printf("  job system threads    %d (%d workers + caller)\n", jobs.threadCount(), jobs.workerCount());
    std::printf("  entities              %d\n", config.entityCount);
    std::printf("  renderables           %d\n", config.renderableCount);
    std::printf("  physics bodies        %d\n", config.colliderCount);

    Demo demo;
    Clock::time_point buildStart = Clock::now();
    demo.build(config, &jobs);
    double buildMs = millisSince(buildStart);

    heading("scene construction");
    row("build world", buildMs,
        std::to_string(demo.scene.world.aliveCount()) + " entities, " + perEntity(buildMs, demo.scene.world.aliveCount()));
    fact("resident memory", bytes(demo.bytesUsed()));
    fact("assets", format("%zu meshes, %zu materials, %zu hierarchy levels", demo.assets.meshCount(),
                          demo.assets.materialCount(), demo.scene.depthLevels()));

    const size_t entities = demo.scene.world.aliveCount();
    const size_t kinematicCount = demo.scene.world.pool<Velocity>().size();

    heading("ecs iteration (position + rotation integrate)");
    Timing ecsSerial = measure(3, iterations, [&] { demo.stepKinematics(1.0f / 60.0f, nullptr); });
    Timing ecsParallel = measure(3, iterations, [&] { demo.stepKinematics(1.0f / 60.0f, &jobs); });
    row("single threaded", ecsSerial.median, perEntity(ecsSerial.median, kinematicCount));
    row("job system", ecsParallel.median,
        perEntity(ecsParallel.median, kinematicCount) + "  " + speedup(ecsSerial.median, ecsParallel.median));

    heading("transform hierarchy");
    Timing xformSerial = measure(3, iterations, [&] { demo.scene.updateTransforms(nullptr); });
    Timing xformParallel = measure(3, iterations, [&] { demo.scene.updateTransforms(&jobs); });
    row("single threaded", xformSerial.median, perEntity(xformSerial.median, entities));
    row("job system", xformParallel.median,
        perEntity(xformParallel.median, entities) + "  " + speedup(xformSerial.median, xformParallel.median));

    heading("physics step");
    Timing physicsSerial = measure(3, iterations / 2, [&] { demo.physics.step(demo.scene, 1.0f / 60.0f, nullptr); });
    Timing physicsParallel = measure(3, iterations / 2, [&] { demo.physics.step(demo.scene, 1.0f / 60.0f, &jobs); });
    const PhysicsStats& ps = demo.physics.stats();
    row("single threaded", physicsSerial.median, perEntity(physicsSerial.median, ps.bodies));
    row("job system", physicsParallel.median,
        perEntity(physicsParallel.median, ps.bodies) + "  " + speedup(physicsSerial.median, physicsParallel.median));
    fact("broadphase", format("%u bodies, %llu pairs tested, %u contacts, %u occupied cells", ps.bodies,
                              static_cast<unsigned long long>(ps.pairsTested), ps.contacts, ps.gridCells));
    fact("pair funnel", format("%llu tested, %llu within reach, %llu dropped as another cell's, %u contacts",
                               static_cast<unsigned long long>(ps.pairsTested),
                               static_cast<unsigned long long>(ps.nearPairs),
                               static_cast<unsigned long long>(ps.duplicatePairs), ps.contacts));
    {
        double allPairs = 0.5 * static_cast<double>(ps.bodies) * static_cast<double>(ps.bodies - 1);
        fact("broadphase pruning",
             format("%.0fx fewer than all-pairs (%.2e)",
                    allPairs / std::max<double>(static_cast<double>(ps.pairsTested), 1.0), allPairs));
    }

    heading("stack convergence (32 columns of 8 spheres)");
    {
        // A stack is the case a sequential-impulse solver is worst at: the
        // reaction has to travel one contact per iteration, so a cold solver
        // needs as many iterations as the stack is tall. Reusing last frame's
        // impulse removes that dependency.
        const float radius = 0.5f;
        auto column = [&](bool warm, int iters) {
            Scene scene;
            PhysicsWorld world;
            world.settings.boundsMin = Vec3{-20, 0, -20};
            world.settings.boundsMax = Vec3{20, 40, 20};
            world.settings.restitutionFloor = 0.0f;
            world.settings.allowSleep = false;
            world.settings.warmStart = warm;
            world.settings.solverIterations = iters;
            std::vector<Entity> tops;
            for (int c = 0; c < 32; ++c) {
                float x = static_cast<float>(c % 8) * 3.0f - 12.0f;
                float z = static_cast<float>(c / 8) * 3.0f - 6.0f;
                for (int i = 0; i < 8; ++i) {
                    Transform t;
                    t.position = Vec3{x, radius + static_cast<float>(i) * 1.02f, z};
                    Entity e = scene.create(t);
                    scene.world.add<Velocity>(e, Velocity{});
                    Collider col;
                    col.radius = radius;
                    col.restitution = 0.0f;
                    scene.world.add<Collider>(e, col);
                    if (i == 7) tops.push_back(e);
                }
            }
            for (int i = 0; i < 600; ++i) world.step(scene, 1.0f / 60.0f, &jobs);
            double height = 0;
            for (Entity e : tops) height += scene.world.tryGet<Transform>(e)->position.y;
            Timing t = measure(2, 20, [&] { world.step(scene, 1.0f / 60.0f, &jobs); });
            return std::make_pair(t.median, height / static_cast<double>(tops.size()));
        };
        const double ideal = radius + 7.0 * 2.0 * radius;
        for (int iters : {2, 4, 8, 16}) {
            auto [coldMs, coldTop] = column(false, iters);
            auto [warmMs, warmTop] = column(true, iters);
            row(format("%2d iterations, cold", iters).c_str(), coldMs,
                format("top sphere at %.2f of %.2f (%.0f%% of the stack standing)", coldTop, ideal,
                       100.0 * (coldTop - radius) / (ideal - radius)));
            row(format("%2d iterations, warm started", iters).c_str(), warmMs,
                format("top sphere at %.2f of %.2f (%.0f%% standing)", warmTop, ideal,
                       100.0 * (warmTop - radius) / (ideal - radius)));
        }
    }

    heading("sleeping bodies");
    {
        // Each configuration gets its own world: continuing one run from
        // another's end state would compare two different piles.
        // Timed single threaded, because sleeping is a work reduction and
        // thread contention swamps it in the parallel path.
        auto settleRun = [&](const DemoConfig& base, int steps, bool allowSleep) {
            Demo d;
            d.build(base, &jobs);
            d.physics.settings.allowSleep = allowSleep;
            for (int i = 0; i < steps; ++i) d.physics.step(d.scene, 1.0f / 60.0f, &jobs);
            Timing t = measure(5, 40, [&] { d.physics.step(d.scene, 1.0f / 60.0f, nullptr); });
            return std::make_pair(t.median, d.physics.stats());
        };

        auto pileRun = [&](int bodies, int steps, bool allowSleep) {
            Scene scene;
            PhysicsWorld world;
            world.settings.boundsMin = Vec3{-15, 0, -15};
            world.settings.boundsMax = Vec3{15, 60, 15};
            world.settings.restitutionFloor = 0.0f;
            world.settings.allowSleep = allowSleep;
            std::mt19937 rng(7);
            std::uniform_real_distribution<float> u(-13.0f, 13.0f);
            for (int i = 0; i < bodies; ++i) {
                Transform t;
                t.position = Vec3{u(rng), 1.0f + static_cast<float>(i) * 0.01f, u(rng)};
                Entity e = scene.create(t);
                scene.world.add<Velocity>(e, Velocity{});
                Collider c;
                c.radius = 0.5f;
                c.restitution = 0.0f;
                scene.world.add<Collider>(e, c);
            }
            for (int i = 0; i < steps; ++i) world.step(scene, 1.0f / 60.0f, &jobs);
            Timing t = measure(5, 40, [&] { world.step(scene, 1.0f / 60.0f, nullptr); });
            return std::make_pair(t.median, world.stats());
        };

        auto [pileOff, pileOffStats] = pileRun(4000, 1800, false);
        auto [pileOn, pileOnStats] = pileRun(4000, 1800, true);
        row("4k in a box, 30 s to settle, sleeping off", pileOff,
            format("%u awake, %u contacts", pileOffStats.awake, pileOffStats.contacts));
        row("4k in a box, 30 s to settle, sleeping on", pileOn,
            format("%u of %u awake, %u contacts, ", pileOnStats.awake, pileOnStats.bodies, pileOnStats.contacts) +
                speedup(pileOff, pileOn));

        DemoConfig churn = config;
        churn.runScripts = false;
        auto [churnOn, churnOnStats] = settleRun(churn, 600, true);
        auto [churnOff, churnOffStats] = settleRun(churn, 600, false);
        row("30k demo pile, still churning, off", churnOff,
            format("%u awake, %u contacts", churnOffStats.awake, churnOffStats.contacts));
        row("30k demo pile, still churning, on", churnOn,
            format("%u of %u awake, %u contacts, ", churnOnStats.awake, churnOnStats.bodies, churnOnStats.contacts) +
                speedup(churnOff, churnOn));
    }

    heading("fast bodies against a thin wall");
    {
        // Speed, not body count, is what a fixed step misses: a body that
        // crosses the wall between two tests is never tested against it.
        auto volley = [&](int maxSubsteps) {
            Scene scene;
            PhysicsWorld world;
            world.settings.useBounds = false;
            world.settings.gravity = Vec3{0, 0, 0};
            world.settings.maxSubsteps = maxSubsteps;
            Transform wallT;
            Entity wall = scene.create(wallT);
            Collider wc;
            wc.kind = static_cast<uint32_t>(ColliderKind::Box);
            wc.halfExtents = Vec3{40.0f, 0.25f, 40.0f};
            wc.invMass = 0.0f;
            scene.world.add<Collider>(wall, wc);
            scene.world.add<Velocity>(wall, Velocity{});

            std::vector<Entity> shots;
            for (int i = 0; i < 400; ++i) {
                float speed = 20.0f + static_cast<float>(i) * 0.5f;
                Transform t;
                t.position = Vec3{static_cast<float>(i % 20) * 2.0f - 20.0f,
                                  8.0f, static_cast<float>(i / 20) * 2.0f - 20.0f};
                Entity e = scene.create(t);
                scene.world.add<Velocity>(e, Velocity{Vec3{0, -speed, 0}, Vec3{0, 0, 0}});
                Collider c;
                c.radius = 0.15f;
                c.restitution = 0.0f;
                scene.world.add<Collider>(e, c);
                shots.push_back(e);
            }
            uint32_t worstSplit = 0;
            Timing t = measure(2, 60, [&] {
                worstSplit = std::max(worstSplit, world.step(scene, 1.0f / 60.0f, &jobs).substeps);
            });
            int through = 0;
            for (Entity e : shots)
                if (scene.world.tryGet<Transform>(e)->position.y < -1.0f) ++through;
            return std::make_tuple(t.median, through, worstSplit);
        };
        for (int cap : {1, 4, 16}) {
            auto [ms, through, split] = volley(cap);
            row(format("400 shots, 20 to 220 m/s, cap %2d", cap).c_str(), ms,
                format("%3d of 400 passed through the wall, up to %u substeps", through, split));
        }
        fact("why the tightest cap is not the slowest",
             "a shot that stops at the wall stops needing splits, while every one "
             "that escapes keeps moving fast enough to split every later step");
    }

    heading("frustum culling and batching");
    demo.scene.updateTransforms(&jobs);
    Frustum frustum = extractFrustum(benchViewProj());
    CullSystem culler;
    RenderList list;

    Timing cullSerial = measure(3, iterations, [&] { culler.build(demo.scene, frustum, 6, list, nullptr, true); });
    uint32_t visible = list.visible;
    uint32_t candidates = list.totalCandidates;
    uint32_t batches = list.drawCalls();

    Timing cullParallel = measure(3, iterations, [&] { culler.build(demo.scene, frustum, 6, list, &jobs, true); });
    Timing cullOff = measure(3, iterations, [&] { culler.build(demo.scene, frustum, 6, list, &jobs, false); });

    row("cull + batch, single threaded", cullSerial.median, perEntity(cullSerial.median, candidates));
    row("cull + batch, job system", cullParallel.median,
        perEntity(cullParallel.median, candidates) + "  " + speedup(cullSerial.median, cullParallel.median));
    row("batch only, culling disabled", cullOff.median, format("%u objects submitted", list.visible));
    fact("culling effectiveness", format("%u of %u kept, %.1f%% rejected", visible, candidates,
                                         100.0 * (1.0 - static_cast<double>(visible) / candidates)));
    fact("instanced batching", format("%u draw calls for %u objects, %.0f instances per call", batches, visible,
                                      static_cast<double>(visible) / std::max(batches, 1u)));

    heading("full frame update (ecs + physics + hierarchy)");
    Timing frameSerial = measure(3, iterations / 2, [&] { demo.update(1.0f / 60.0f, nullptr); });
    Timing frameParallel = measure(3, iterations / 2, [&] { demo.update(1.0f / 60.0f, &jobs); });
    row("single threaded", frameSerial.median, format("%.0f fps ceiling", 1000.0 / frameSerial.median));
    row("job system", frameParallel.median,
        format("%.0f fps ceiling, ", 1000.0 / frameParallel.median) + speedup(frameSerial.median, frameParallel.median));

    heading("spatial clustering vs flat culling");
    {
        CullSystem flat;
        flat.useClusters = false;
        RenderList flatList;
        Timing flatTime = measure(3, iterations, [&] { flat.build(demo.scene, frustum, 6, flatList, &jobs, true); });

        CullSystem clusteredCuller;
        clusteredCuller.sortSpatially(demo.scene);
        demo.scene.updateTransforms(&jobs);
        RenderList clusteredList;
        Timing sortedTime =
            measure(3, iterations, [&] { clusteredCuller.build(demo.scene, frustum, 6, clusteredList, &jobs, true); });
        const CullStats& cs = clusteredCuller.stats();

        row("flat, one test per object", flatTime.median, perEntity(flatTime.median, flatList.totalCandidates));
        row("clustered over morton order", sortedTime.median,
            perEntity(sortedTime.median, clusteredList.totalCandidates) + "  " +
                speedup(flatTime.median, sortedTime.median));
        fact("agreement", format("%u visible both ways", clusteredList.visible) +
                              (clusteredList.visible == flatList.visible ? "" : "  MISMATCH"));
        fact("clusters", format("%u of %u objects, %u outside, %u inside, %u straddling", cs.clusters,
                                clusteredList.totalCandidates, cs.clustersOutside, cs.clustersInside,
                                cs.clustersStraddling));
        fact("subclusters", format("%u of 16 objects each, %u outside, %u inside, %u straddling",
                                   cs.subclusters, cs.subclustersOutside, cs.subclustersInside,
                                   cs.subclustersStraddling));
        fact("objects individually tested",
             format("%u of %u, %.1f%% resolved by their cluster", cs.objectsTested, clusteredList.totalCandidates,
                    100.0 * (1.0 - static_cast<double>(cs.objectsTested) / clusteredList.totalCandidates)));
    }

    heading("frame phase breakdown (job system)");
    {
        Profiler& profiler = Profiler::instance();
        profiler.reset();
        for (int i = 0; i < iterations; ++i) {
            profiler.beginFrame();
            demo.update(1.0f / 60.0f, &jobs);
            culler.build(demo.scene, frustum, 6, list, &jobs, true);
            profiler.endFrame();
        }
        std::vector<ZoneStats> zones = profiler.stats();
        std::sort(zones.begin(), zones.end(), [](const ZoneStats& a, const ZoneStats& b) { return a.avgMs > b.avgMs; });
        for (const ZoneStats& z : zones) {
            if (z.avgMs < 0.005) continue;
            std::printf("  %-38s %9.3f ms   p95 %6.3f  max %6.3f  %u calls/frame\n", z.name.c_str(), z.avgMs, z.p95Ms,
                        z.maxMs, z.calls);
        }
    }

    heading("job system scaling (full frame update)");
    for (int workers : {0, 1, 2, 3, 5, 7, 9, 11, 15}) {
        if (workers > static_cast<int>(hw) - 1) break;
        JobSystem pool(workers);
        Timing t = measure(2, std::max(iterations / 4, 4), [&] { demo.update(1.0f / 60.0f, workers ? &pool : nullptr); });
        char label[64];
        std::snprintf(label, sizeof(label), "%d thread%s", workers + 1, workers == 0 ? "" : "s");
        row(label, t.median, speedup(frameSerial.median, t.median));
    }

    heading("serialization");
    std::vector<uint8_t> blob;
    Timing saveTime = measure(1, 8, [&] { blob = serializeScene(demo.scene); });
    Scene loaded;
    std::string error;
    Timing loadTime = measure(1, 8, [&] { deserializeScene(loaded, blob.data(), blob.size(), error); });
    const double blobMb = static_cast<double>(blob.size()) / (1024.0 * 1024.0);
    row("serialize scene", saveTime.median, format("%s, %.1f GB/s", bytes(blob.size()).c_str(),
                                                   blobMb / (saveTime.median / 1000.0) / 1024.0));
    row("deserialize scene", loadTime.median,
        format("%zu entities restored, %.1f GB/s", loaded.world.aliveCount(),
               blobMb / (loadTime.median / 1000.0) / 1024.0));

    heading("lua scripting");
    {
        Demo scripted;
        DemoConfig small = config;
        small.entityCount = 4000;
        small.renderableCount = 2000;
        small.colliderCount = 1000;
        small.hierarchyChildren = 0;
        scripted.build(small, &jobs);
        std::string scriptError;
        const char* source = R"(
            local n = 5000
            for i = 1, n do
              local e = skein.spawn{position = {i % 50, 5, i // 50}}
              skein.on_update(e, function(self, dt)
                local x, y, z = skein.position(self)
                skein.set_position(self, x, y + dt, z)
              end)
            end
        )";
        if (!scripted.script.doString(source, "=bench", scriptError))
            std::printf("  lua setup failed: %s\n", scriptError.c_str());
        size_t scriptedCount = scripted.script.scriptedEntities();
        Timing luaTime = measure(3, iterations, [&] { scripted.script.update(1.0f / 60.0f); });
        row("per-entity callbacks", luaTime.median,
            format("%zu scripts, ", scriptedCount) + perEntity(luaTime.median, scriptedCount));
        fact("lua heap", bytes(scripted.script.memoryBytes()));
    }

    heading("memory layout: ecs pools vs an object graph");
    {
        const size_t layoutCount = 200000;
        std::mt19937 rng(9001);
        std::uniform_real_distribution<float> u(-1.0f, 1.0f);

        std::vector<Vec3> pos(layoutCount), lin(layoutCount), ang(layoutCount);
        std::vector<Quat> rot(layoutCount);
        for (size_t i = 0; i < layoutCount; ++i) {
            pos[i] = Vec3{u(rng) * 100, u(rng) * 100, u(rng) * 100};
            lin[i] = Vec3{u(rng), u(rng), u(rng)};
            ang[i] = Vec3{u(rng) * 0.5f, u(rng) * 0.5f, u(rng) * 0.5f};
            rot[i] = Quat::euler(u(rng), u(rng), u(rng));
        }

        std::vector<FlatNode> flat(layoutCount);
        std::vector<std::unique_ptr<SceneNode>> owned;
        std::vector<SceneNode*> nodes(layoutCount);
        owned.reserve(layoutCount);
        for (size_t i = 0; i < layoutCount; ++i) {
            flat[i].position = pos[i]; flat[i].rotation = rot[i];
            flat[i].linear = lin[i]; flat[i].angular = ang[i];
            auto node = std::make_unique<MovingNode>();
            node->position = pos[i]; node->rotation = rot[i];
            node->linear = lin[i]; node->angular = ang[i];
            nodes[i] = node.get();
            owned.push_back(std::move(node));
        }
        // Long-lived scenes do not keep their allocations in creation order, so
        // shuffling the pointer array is the honest version of this comparison.
        std::shuffle(nodes.begin(), nodes.end(), rng);

        std::vector<Vec3> soaPos = pos, soaLin = lin, soaAng = ang;
        std::vector<Quat> soaRot = rot;

        Timing virtualTime = measure(2, iterations, [&] {
            for (SceneNode* n : nodes) n->update(1.0f / 60.0f);
        });
        Timing flatTime = measure(2, iterations, [&] {
            for (FlatNode& n : flat) n.update(1.0f / 60.0f);
        });
        Timing soaTime = measure(2, iterations, [&] {
            const float dt = 1.0f / 60.0f;
            for (size_t i = 0; i < layoutCount; ++i) {
                soaPos[i] += soaLin[i] * dt;
                if (length2(soaAng[i]) > 0.0f)
                    soaRot[i] = normalize(Quat::axisAngle(soaAng[i], length(soaAng[i]) * dt) * soaRot[i]);
            }
        });

        row("virtual call, pointer per object", virtualTime.median,
            perEntity(virtualTime.median, layoutCount) + format("  %zu B/object", sizeof(MovingNode)));
        row("contiguous objects, direct call", flatTime.median,
            perEntity(flatTime.median, layoutCount) + format("  %zu B/object, ", sizeof(FlatNode)) +
                speedup(virtualTime.median, flatTime.median));
        row("component arrays (what skein does)", soaTime.median,
            perEntity(soaTime.median, layoutCount) +
                format("  %zu B touched, ", sizeof(Vec3) * 3 + sizeof(Quat)) +
                speedup(virtualTime.median, soaTime.median));
        // Touching two fields instead of all of them is where the layouts
        // actually diverge: the object walk still drags whole cache lines.
        Timing flatPos = measure(2, iterations, [&] {
            const float dt = 1.0f / 60.0f;
            for (FlatNode& n : flat) n.position += n.linear * dt;
        });
        Timing soaPosTime = measure(2, iterations, [&] {
            const float dt = 1.0f / 60.0f;
            for (size_t i = 0; i < layoutCount; ++i) soaPos[i] += soaLin[i] * dt;
        });
        row("position only, contiguous objects", flatPos.median, perEntity(flatPos.median, layoutCount));
        row("position only, component arrays", soaPosTime.median,
            perEntity(soaPosTime.median, layoutCount) + "  " + speedup(flatPos.median, soaPosTime.median));
        fact("bytes walked per pass",
             format("%s vs %s", bytes(layoutCount * sizeof(MovingNode)).c_str(),
                    bytes(layoutCount * (sizeof(Vec3) * 3 + sizeof(Quat))).c_str()));
    }

    heading("scaling sweep (density held constant, job system)");
    if (runSweep) {
        std::printf("  %-38s %9s   %10s %10s %10s %10s\n", "entities", "build", "ecs", "hierarchy", "cull flat",
                    "cull morton");
        for (int count : {25000, 100000, 250000, 500000, 1000000}) {
            DemoConfig sweep = config;
            sweep.entityCount = count;
            sweep.renderableCount = count / 3;
            sweep.colliderCount = std::min(count / 4, 20000);
            sweep.hierarchyChildren = count / 10;
            sweep.pointLights = 8;
            sweep.runScripts = false;
            // Density is what culling and the broadphase actually respond to,
            // so the field grows with the cube root of the population.
            sweep.fieldExtent = 200.0f * std::cbrt(static_cast<float>(count) / 100000.0f);
            sweep.fieldHeight = 90.0f * std::cbrt(static_cast<float>(count) / 100000.0f);

            Demo d;
            Clock::time_point t0 = Clock::now();
            d.build(sweep, &jobs);
            double build = millisSince(t0);

            const int reps = count >= 500000 ? 6 : 12;
            Timing ecs = measure(2, reps, [&] { d.stepKinematics(1.0f / 60.0f, &jobs); });
            Timing xf = measure(2, reps, [&] { d.scene.updateTransforms(&jobs); });

            Mat4 sweepVp = perspective(radians(65.0f), 16.0f / 9.0f, 0.2f, 600.0f) *
                           lookAt(Vec3{0, sweep.fieldHeight * 0.25f, sweep.fieldExtent * 0.6f}, Vec3{0, 0, 0},
                                  Vec3{0, 1, 0});
            Frustum sweepFrustum = extractFrustum(sweepVp);

            CullSystem flat;
            flat.useClusters = false;
            RenderList flatList;
            Timing flatT = measure(2, reps, [&] { flat.build(d.scene, sweepFrustum, 6, flatList, &jobs, true); });

            CullSystem morton;
            morton.sortSpatially(d.scene);
            RenderList mortonList;
            Timing mortonT = measure(2, reps, [&] { morton.build(d.scene, sweepFrustum, 6, mortonList, &jobs, true); });

            std::printf("  %-38s %8.1fms %9.3fms %9.3fms %9.3fms %9.3fms  %s, %u/%u visible\n",
                        format("%d", count).c_str(), build, ecs.median, xf.median, flatT.median, mortonT.median,
                        speedup(flatT.median, mortonT.median).c_str(), mortonList.visible, mortonList.totalCandidates);
            if (mortonList.visible != flatList.visible) std::printf("  MISMATCH at %d entities\n", count);
        }
    } else {
        fact("skipped", "pass --sweep to run the 25k to 1M scaling sweep");
    }

    heading("cull cost across camera coverage");
    {
        CullSystem flat;
        flat.useClusters = false;
        CullSystem morton;
        morton.sortSpatially(demo.scene);
        RenderList flatList, mortonList;
        std::printf("  %-38s %9s %11s %9s %8s\n", "camera", "flat", "morton", "speedup", "visible");
        struct Shot {
            const char* label;
            Vec3 eye;
            Vec3 at;
            float fov;
        };
        const Shot shots[] = {
            {"tight, looking away", {0, 24, 260}, {0, 24, 600}, 35.0f},
            {"narrow fov into the field", {0, 24, 220}, {0, 6, 0}, 25.0f},
            {"default view", {0, 22, 120}, {0, 6, 0}, 65.0f},
            {"wide fov, centred", {0, 30, 60}, {0, 6, 0}, 100.0f},
            {"inside the field, looking down", {0, 80, 0}, {0, 0, 0}, 110.0f},
            {"far back, whole field in view", {0, 120, 520}, {0, 6, 0}, 70.0f},
        };
        for (const Shot& shot : shots) {
            Frustum f = extractFrustum(perspective(radians(shot.fov), 16.0f / 9.0f, 0.2f, 1400.0f) *
                                       lookAt(shot.eye, shot.at, Vec3{0, 1, 0}));
            Timing ft = measure(3, iterations, [&] { flat.build(demo.scene, f, 6, flatList, &jobs, true); });
            Timing mt = measure(3, iterations, [&] { morton.build(demo.scene, f, 6, mortonList, &jobs, true); });
            std::printf("  %-38s %7.3fms %9.3fms %8s %7.1f%%%s\n", shot.label, ft.median, mt.median,
                        speedup(ft.median, mt.median).c_str(),
                        100.0 * mortonList.visible / std::max(mortonList.totalCandidates, 1u),
                        mortonList.visible == flatList.visible ? "" : "  MISMATCH");
        }
    }

    heading("order decay under sustained motion");
    {
        Demo moving;
        DemoConfig mc = config;
        mc.runScripts = false;
        moving.build(mc, &jobs);

        // Both runs measure decay every 30 frames; only the maintained one is
        // allowed to act on it, so the two rows differ in exactly one thing.
        auto run = [&](bool maintain) {
            CullSystem c;
            RenderList l;
            if (!maintain) c.resortThreshold = 1e9f;
            c.sortSpatially(moving.scene);
            std::vector<double> samples;
            double decay = 1.0;
            for (int frame = 0; frame < 800; ++frame) {
                moving.update(1.0f / 60.0f, &jobs);
                if ((frame % 30) == 0) c.maintain(moving.scene);
                Clock::time_point t0 = Clock::now();
                c.build(moving.scene, frustum, 6, l, &jobs, true);
                samples.push_back(millisSince(t0));
                decay = c.spreadBaseline() > 0 ? c.clusterSpread() / c.spreadBaseline() : 1.0;
            }
            std::sort(samples.begin(), samples.end());
            return std::make_tuple(samples[samples.size() / 2], decay, c.stats().sorts, c.stats().objectsTested);
        };

        auto [staleMs, staleDecay, staleSorts, staleTested] = run(false);
        auto [freshMs, freshDecay, freshSorts, freshTested] = run(true);
        row("sorted once, 800 frames of motion", staleMs,
            format("decay %.2fx, %u sorts, %u objects tested", staleDecay, staleSorts, staleTested));
        row("maintained every 30 frames", freshMs,
            format("decay %.2fx, %u sorts, %u objects tested, ", freshDecay, freshSorts, freshTested) +
                speedup(staleMs, freshMs));
    }

    heading("memory");
    fact("ecs pools", bytes(demo.scene.world.bytesUsed()));
    fact("mesh + material assets", bytes(demo.assets.bytesUsed()));
    fact("physics working set", bytes(demo.physics.bytesUsed()));
    fact("culling working set", bytes(culler.bytesUsed()));
    std::printf("\n");
    return 0;
}
