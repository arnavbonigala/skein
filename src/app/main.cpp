#include <cstdio>
#include <cstring>
#include <string>

#include "app/camera.hpp"
#include "app/demo_scene.hpp"
#include "core/jobs.hpp"
#include "core/profiler.hpp"
#include "render/debug_draw.hpp"
#include "render/gl.hpp"
#include "render/renderer.hpp"
#include "scene/serialize.hpp"

using namespace skein;

namespace {

struct AppState {
    Camera camera;
    bool mouseCaptured = true;
    bool firstMouse = true;
    double lastX = 0, lastY = 0;
    bool paused = false;
    bool showBounds = false;
    bool freezeCullCamera = false;
    Mat4 frozenViewProj = Mat4::identity();
    Renderer* renderer = nullptr;
    Demo* demo = nullptr;
    JobSystem* jobs = nullptr;
    bool vsync = true;
};

AppState g;

void printHelp() {
    std::printf(
        "\nskein controls\n"
        "  W A S D / Space / Ctrl   fly            Shift  boost\n"
        "  Tab    release or capture the mouse     Esc    quit\n"
        "  F      frustum culling on or off        I      instanced batching on or off\n"
        "  L      shadows on or off                B      draw cull bounds\n"
        "  C      freeze the culling camera        P      dump the frame profile\n"
        "  V      vsync                            Space bar pause is O\n"
        "  F5/F9  save or load scene.skn\n\n");
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action != GLFW_PRESS) return;
    switch (key) {
        case GLFW_KEY_ESCAPE:
            glfwSetWindowShouldClose(window, GLFW_TRUE);
            break;
        case GLFW_KEY_TAB:
            g.mouseCaptured = !g.mouseCaptured;
            glfwSetInputMode(window, GLFW_CURSOR, g.mouseCaptured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
            g.firstMouse = true;
            break;
        case GLFW_KEY_F:
            g.renderer->options.frustumCulling = !g.renderer->options.frustumCulling;
            std::printf("frustum culling %s\n", g.renderer->options.frustumCulling ? "on" : "off");
            break;
        case GLFW_KEY_I:
            g.renderer->options.instancing = !g.renderer->options.instancing;
            std::printf("instanced batching %s\n", g.renderer->options.instancing ? "on" : "off");
            break;
        case GLFW_KEY_L:
            g.renderer->options.shadows = !g.renderer->options.shadows;
            std::printf("shadows %s\n", g.renderer->options.shadows ? "on" : "off");
            break;
        case GLFW_KEY_B:
            g.showBounds = !g.showBounds;
            break;
        case GLFW_KEY_C:
            g.freezeCullCamera = !g.freezeCullCamera;
            std::printf("culling camera %s\n", g.freezeCullCamera ? "frozen" : "live");
            break;
        case GLFW_KEY_O:
            g.paused = !g.paused;
            break;
        case GLFW_KEY_P:
            std::printf("\n%s\n", Profiler::instance().report().c_str());
            break;
        case GLFW_KEY_V:
            g.vsync = !g.vsync;
            glfwSwapInterval(g.vsync ? 1 : 0);
            std::printf("vsync %s\n", g.vsync ? "on" : "off");
            break;
        case GLFW_KEY_F5: {
            std::string error;
            if (saveScene(g.demo->scene, "scene.skn", error))
                std::printf("saved scene.skn\n");
            else
                std::printf("save failed: %s\n", error.c_str());
            break;
        }
        case GLFW_KEY_F9: {
            std::string error;
            if (loadScene(g.demo->scene, "scene.skn", error)) {
                g.demo->scene.updateTransforms(g.jobs);
                std::printf("loaded scene.skn (%zu entities)\n", g.demo->scene.world.aliveCount());
            } else {
                std::printf("load failed: %s\n", error.c_str());
            }
            break;
        }
        default:
            break;
    }
}

struct CaptureConfig {
    const char* label;
    bool culling;
    bool instancing;
};

/// One measured configuration of the render path, averaged over the sampled frames.
struct CaptureResult {
    double frameMs = 0, cpuMs = 0, gpuMs = 0, cullMs = 0, submitMs = 0;
    double drawCalls = 0, visible = 0, candidates = 0, triangles = 0;
};

void cursorCallback(GLFWwindow*, double x, double y) {
    if (!g.mouseCaptured) return;
    if (g.firstMouse) {
        g.lastX = x;
        g.lastY = y;
        g.firstMouse = false;
        return;
    }
    float dx = static_cast<float>(x - g.lastX) * 0.0022f;
    float dy = static_cast<float>(g.lastY - y) * 0.0022f;
    g.lastX = x;
    g.lastY = y;
    g.camera.look(dx, dy);
}

void handleMovement(GLFWwindow* window, float dt) {
    Vec3 delta{0, 0, 0};
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) delta.z += 1;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) delta.z -= 1;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) delta.x += 1;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) delta.x -= 1;
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) delta.y += 1;
    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) delta.y -= 1;
    float boost = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ? 4.0f : 1.0f;
    g.camera.move(delta, dt, boost);
}

}  // namespace

int main(int argc, char** argv) {
    DemoConfig config;
    int captureFrames = 0;
    std::string scriptPath = std::string(SKEIN_ASSET_DIR) + "/scripts/demo.lua";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--entities") == 0 && i + 1 < argc)
            config.entityCount = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--renderables") == 0 && i + 1 < argc)
            config.renderableCount = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--colliders") == 0 && i + 1 < argc)
            config.colliderCount = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--no-script") == 0)
            config.runScripts = false;
        else if (std::strcmp(argv[i], "--script") == 0 && i + 1 < argc)
            scriptPath = argv[++i];
        else if (std::strcmp(argv[i], "--capture") == 0 && i + 1 < argc)
            captureFrames = std::atoi(argv[++i]);
    }

    if (!glfwInit()) {
        std::fprintf(stderr, "glfwInit failed\n");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    GLFWwindow* window = glfwCreateWindow(1600, 900, "skein", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "failed to create a GL 4.1 core window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetCursorPosCallback(window, cursorCallback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glEnable(GL_MULTISAMPLE);

    std::printf("gl %s | %s\n", glGetString(GL_VERSION), glGetString(GL_RENDERER));

    JobSystem jobs;
    Renderer renderer;
    std::string error;
    if (!renderer.init(error)) {
        std::fprintf(stderr, "renderer init failed: %s\n", error.c_str());
        return 1;
    }

    Demo demo;
    Clock::time_point buildStart = Clock::now();
    demo.build(config, &jobs);
    double buildMs = millisSince(buildStart);
    renderer.uploadAssets(demo.assets);

    if (config.runScripts && !demo.loadScript(scriptPath, error))
        std::fprintf(stderr, "script: %s\n", error.c_str());

    DebugDraw debug;
    if (!debug.init(error)) std::fprintf(stderr, "debug draw: %s\n", error.c_str());

    g.renderer = &renderer;
    g.demo = &demo;
    g.jobs = &jobs;

    std::printf("built %zu entities in %.1f ms across %d threads\n", demo.scene.world.aliveCount(), buildMs,
                jobs.threadCount());

    if (captureFrames > 0) {
        glfwSwapInterval(0);
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        const CaptureConfig configs[] = {
            {"instanced + culled", true, true},
            {"instanced, no culling", false, true},
            {"one draw per object, culled", true, false},
            {"one draw per object, no culling", false, false},
        };
        const int warmup = 20;
        std::vector<CaptureResult> results;

        for (const CaptureConfig& c : configs) {
            renderer.options.frustumCulling = c.culling;
            renderer.options.instancing = c.instancing;
            CaptureResult r;
            int sampled = 0;
            for (int frame = 0; frame < captureFrames + warmup && !glfwWindowShouldClose(window); ++frame) {
                Clock::time_point frameStart = Clock::now();
                glfwPollEvents();
                demo.update(1.0f / 60.0f, &jobs);
                int width = 0, height = 0;
                glfwGetFramebufferSize(window, &width, &height);
                renderer.resize(width, height);
                renderer.render(demo.scene, demo.assets, g.camera.view(),
                                g.camera.projection(static_cast<float>(width) / static_cast<float>(height)),
                                g.camera.position, &jobs);
                glfwSwapBuffers(window);
                if (frame < warmup) continue;
                const RenderStats& s = renderer.stats();
                r.frameMs += millisSince(frameStart);
                r.cpuMs += s.cpuMs;
                r.gpuMs += s.gpuMs;
                r.cullMs += s.cullMs;
                r.submitMs += s.submitMs;
                r.drawCalls += s.drawCalls + s.shadowDrawCalls;
                r.visible += s.visible;
                r.candidates += s.candidates;
                r.triangles += static_cast<double>(s.triangles);
                ++sampled;
            }
            double n = sampled > 0 ? sampled : 1;
            r.frameMs /= n; r.cpuMs /= n; r.gpuMs /= n; r.cullMs /= n; r.submitMs /= n;
            r.drawCalls /= n; r.visible /= n; r.candidates /= n; r.triangles /= n;
            results.push_back(r);
            std::printf("%-32s frame %6.2f ms (%5.1f fps)  render cpu %6.2f  gpu %6.2f  cull %5.2f  submit %6.2f  "
                        "%7.0f draws  %6.0f/%6.0f visible  %.2fM tris\n",
                        c.label, r.frameMs, 1000.0 / r.frameMs, r.cpuMs, r.gpuMs, r.cullMs, r.submitMs, r.drawCalls,
                        r.visible, r.candidates, r.triangles * 1e-6);
        }
        glfwDestroyWindow(window);
        glfwTerminate();
        return 0;
    }

    printHelp();

    double lastTime = glfwGetTime();
    double titleTimer = 0;

    while (!glfwWindowShouldClose(window)) {
        Profiler::instance().beginFrame();
        double now = glfwGetTime();
        float dt = static_cast<float>(std::min(now - lastTime, 0.1));
        lastTime = now;

        glfwPollEvents();
        handleMovement(window, dt);

        if (!g.paused) demo.update(dt, &jobs);

        int width = 0, height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        renderer.resize(width, height);

        Mat4 view = g.camera.view();
        Mat4 projection = g.camera.projection(static_cast<float>(width) / static_cast<float>(height));

        if (!g.freezeCullCamera) g.frozenViewProj = projection * view;

        renderer.render(demo.scene, demo.assets, view, projection, g.camera.position, &jobs);

        if (g.showBounds || g.freezeCullCamera) {
            if (g.freezeCullCamera) debug.frustum(g.frozenViewProj, Vec3{1.0f, 0.85f, 0.2f});
            if (g.showBounds) {
                int drawn = 0;
                forEach<CullBounds>(demo.scene.world, [&](Entity, CullBounds& cb) {
                    if (drawn++ >= 3000) return;
                    debug.box(cb.center, cb.extent, Vec3{0.2f, 0.9f, 0.5f});
                });
            }
            glDisable(GL_DEPTH_TEST);
            debug.flush(projection * view);
            glEnable(GL_DEPTH_TEST);
        }

        glfwSwapBuffers(window);
        Profiler::instance().endFrame();

        titleTimer += dt;
        if (titleTimer > 0.25) {
            titleTimer = 0;
            const RenderStats& s = renderer.stats();
            const PhysicsStats& p = demo.physics.stats();
            char title[512];
            std::snprintf(title, sizeof(title),
                          "skein | %.2f ms cpu (%.1f fps) | gpu %.2f ms | %u/%u visible | %u draws | "
                          "%.1fM tris | %u bodies %u contacts | %.0f MB",
                          Profiler::instance().avgFrameMs(),
                          Profiler::instance().avgFrameMs() > 0 ? 1000.0 / Profiler::instance().avgFrameMs() : 0.0,
                          s.gpuMs, s.visible, s.candidates, s.drawCalls + s.shadowDrawCalls,
                          static_cast<double>(s.triangles) * 1e-6, p.bodies, p.contacts,
                          static_cast<double>(demo.bytesUsed() + s.gpuBytes) / (1024.0 * 1024.0));
            glfwSetWindowTitle(window, title);
        }
    }

    std::printf("\n%s\n", Profiler::instance().report().c_str());
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
