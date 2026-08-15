# Skein

A data-oriented 3D engine written from scratch in C++20: sparse-set ECS, a
work-stealing job system, an OpenGL 4.1 forward renderer with instanced
batching and shadows, a hashed-grid physics world, binary serialization and
embedded Lua scripting.

Every number below is measured on the machine described in
[Measurements](#measurements), by the two benchmark harnesses in this repo. No
figure is estimated.

```
Entity
  ↓
┌──────────────────────────────┐
│ ECS                          │
│ Transform[]  Renderable[]    │
│ Velocity[]   Collider[]      │
│ CullBounds[] Light[]         │
└──────────────┬───────────────┘
       ┌───────┼─────────┐
       ↓       ↓         ↓
    Render   Physics   Scripts
```

## Build

Requires CMake 3.20, a C++20 compiler, Lua and (for the interactive demo) GLFW.

```sh
brew install cmake lua glfw
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/skein_tests     # 59 tests
./build/skein_bench     # headless CPU benchmark
./build/skein_demo      # interactive window
```

`skein_demo --capture 200` runs the render-path sweep instead of opening an
interactive session.

## What is in here

**ECS** — sparse-set storage: each component type owns a dense array plus a
sparse entity-to-index map, so iteration is linear over packed memory and
removal is swap-and-pop. Entities are 64-bit handles with a generation counter
in the high word, so a stale handle never resolves to a recycled slot. Pools
are registered by stable string name behind a type-erased interface, which is
what lets serialization walk them generically.

**Transform hierarchy** — no recursion and no pointer chasing. Depths are
computed once with memoized upward walks and counting-sorted into a flat order,
so each depth level is a contiguous span whose parents are all already final.
That makes every level trivially parallel: levels of 512 entities or more are
split across the job system.

**Job system** — one deque per worker, LIFO on the owner's end and FIFO for
thieves. The submitting thread participates rather than blocking, so nested
`parallelFor` calls make progress instead of deadlocking.

**Renderer** — OpenGL 4.1 core against the macOS system framework, so no
loader library. Culling produces a counting-sorted run of `(mesh, material)`
batches; every visible object's model matrix is streamed into one orphaned
instance buffer per frame, with the shadow pass appended after the main pass so
there is a single upload. `glDrawElementsInstancedBaseInstance` is 4.2 and
unavailable here, so batches re-point the four mat4 instance attributes at a
byte offset before drawing. Lighting is Blinn-Phong with a metallic tint, a
2048² PCF shadow map for the sun and up to 32 point lights selected per frame
by distance.

**Culling** — Gribb-Hartmann plane extraction, a sphere prepass and a
center/extent AABB test. World-space bounds live inside the same `CullBounds`
component the transform pass refreshes, so culling touches one contiguous
array.

**Physics** — SoA gather/scatter around a hashed uniform grid built with a
counting sort, a 27-cell neighbour scan that verifies real cell coordinates to
reject hash collisions, and a sequential-impulse solver with Baumgarte
positional correction. A test asserts the grid finds exactly the same contact
set as an O(n²) reference over 3000 bodies.

**Assets** — a hand-written OBJ parser (all four face index forms, negative
indices, polygon fan triangulation, vertex welding, generated normals) plus
generated primitives. Batch loads parse concurrently and register in a
deterministic order.

**Serialization** — a versioned binary format that writes each registered pool
verbatim. Entity ids, including generations and holes, survive a round trip;
unknown or size-mismatched pools are skipped rather than treated as fatal.

**Scripting** — Lua 5.5 embedded through the C API. Scripts spawn, destroy,
move, reparent, light and collide entities through the same component pools the
C++ systems read, so a scripted entity is culled, lit and simulated exactly
like a native one. Per-entity callbacks are held in the registry; a callback
that errors is unbound and reported instead of killing the frame.

## Measurements

Apple M3, 8 hardware threads, macOS 26.6, Apple clang `-O2`, OpenGL 4.1 (Metal
90.5). CPU figures are medians over 40 runs from `skein_bench`; render figures
are means over 200 frames from `skein_demo --capture 200`.

Both harnesses build the same world from `Demo::build`: **112,025 entities**,
100,000 of them integrating position and rotation every frame, **37,000
renderable**, 30,000 rigid bodies, 25 lights, 6 meshes, 6 materials, 31.7 MB
resident. The interactive demo also runs `assets/scripts/demo.lua` on top,
which adds 316 scripted entities — 112,341 entities and 37,316 renderables in
the render table below.

### ECS and simulation, single threaded vs the job system

| Pass | 1 thread | 8 threads | Speedup |
|---|---|---|---|
| ECS iteration (100k integrate) | 0.862 ms — 8.6 ns/entity | 0.270 ms — 2.7 ns/entity | 3.19x |
| Transform hierarchy (112k) | 0.767 ms — 6.8 ns/entity | 0.332 ms — 3.0 ns/entity | 2.31x |
| Physics step (30k bodies) | 7.461 ms — 249 ns/body | 2.035 ms — 68 ns/body | 3.67x |
| Cull + batch (37k candidates) | 0.443 ms — 12.0 ns/object | 0.326 ms — 8.8 ns/object | 1.36x |
| **Full simulation frame** | **8.959 ms (112 fps)** | **2.685 ms (372 fps)** | **3.34x** |

Thread scaling on the full simulation frame:

| Threads | 1 | 2 | 3 | 4 | 6 | 8 |
|---|---|---|---|---|---|---|
| ms | 9.31 | 5.37 | 4.12 | 3.56 | 3.16 | 3.20 |
| speedup | 0.96x | 1.67x | 2.18x | 2.52x | 2.84x | 2.80x |

Scaling flattens past six threads because the M3's four efficiency cores are
roughly a third the throughput of its performance cores, and the remaining
serial work is the physics solver's iteration barrier.

### Culling effectiveness

37,000 candidates against a 65° frustum: **20,141 kept, 16,859 rejected
(45.6%)** in 0.33 ms. The broadphase is the same story at a different scale —
30,000 bodies produce 22,375 candidate pairs instead of the 4.5×10⁸ an
all-pairs test would need, **20,111x fewer**.

### Rendering, before and after batching

Same scene, same camera, same frame count, only the render path changed. GPU
time is a `GL_TIME_ELAPSED` query read one frame late so it never stalls.

| Path | Frame | Render CPU | GPU | Submit | Draw calls | Objects |
|---|---|---|---|---|---|---|
| Instanced + culled | **6.61 ms (151 fps)** | 2.45 ms | 4.10 ms | 1.47 ms | 72 | 17,976 |
| Instanced, no culling | 7.83 ms (128 fps) | 3.51 ms | 4.39 ms | 1.79 ms | 72 | 37,316 |
| One draw per object, culled | 16.66 ms (60 fps) | 11.42 ms | 11.30 ms | 10.77 ms | 22,577 | 19,103 |
| One draw per object, no culling | 24.19 ms (41 fps) | 19.88 ms | 19.12 ms | 19.26 ms | 40,822 | 37,316 |

Batching the same 37,316 objects collapses 40,822 draw calls into 72 and cuts
submission from 19.26 ms to 1.79 ms — **10.8x** — which takes the frame from
41 fps to 128 fps. Once batched, the GPU stops tracking CPU submission cost and
the frame becomes GPU-bound, which is where culling starts paying: it removes
about 45% of the objects for another 1.2 ms.

### Serialization, scripting, memory

| | |
|---|---|
| Serialize 112,025 entities | 1.35 ms → 21.0 MB (15.2 GB/s) |
| Deserialize | 4.17 ms (4.9 GB/s), entity ids preserved exactly |
| Lua per-entity callbacks | 0.383 ms for 5,000 scripts — 76.5 ns/callback |
| ECS pools / physics / culling / Lua heap | 31.6 MB / 2.6 MB / 737 KB / 385 KB |

## Demo controls

`W A S D`, `Space`/`Ctrl` fly, `Shift` boosts, `Tab` releases the mouse.
`F` toggles frustum culling, `I` instanced batching, `L` shadows, `B` draws
cull bounds, `C` freezes the culling camera so you can fly outside it and watch
objects drop out, `O` pauses, `P` dumps the frame profile, `V` toggles vsync,
`F5`/`F9` save and load `scene.skn`.

## Tests

59 tests, no framework. They cover the parts where being wrong is quiet: the
hashed grid must return exactly the brute-force contact set, culling must keep
exactly the objects a brute-force frustum test keeps over 20,000 objects, a
2,000-entity hierarchy with destroyed parents must survive a serialization
round trip, threaded transform updates must match single-threaded ones bit for
bit, and `parallelFor` must cover every index exactly once under nesting.
