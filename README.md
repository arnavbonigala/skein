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
./build/skein_tests     # 63 tests
./build/skein_bench     # headless CPU benchmark
./build/skein_bench --sweep   # plus the 25k to 1M scaling sweep
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
center/extent AABB test, but the per-object test is the fallback rather than
the main path. See [Cluster culling](#cluster-culling-over-morton-ordered-pools).

**Physics broadphase** — bodies are inserted into every grid cell they overlap
and the cell size follows the *typical* body rather than the largest one, so a
single oversized collider no longer coarsens the whole grid. Pairs are deduped
by reporting only from the lowest cell two bodies share, compared through an
exact packed 64-bit cell key so cells that collide in the hash are still told
apart. The solver colours the contact graph greedily (64 colours, one bitmask
per body) and runs each colour in parallel; a test asserts the parallel result
is bitwise identical to the serial one over 90 steps of a 4,000-body pile.

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
renderable**, 30,000 rigid bodies piled densely enough to generate ~45,000
contacts a step, 25 lights, 6 meshes, 6 materials, 31.4 MB resident. The
interactive demo also runs `assets/scripts/demo.lua` on top, which adds 316
scripted entities — 112,341 entities and 37,316 renderables in the render
table below.

### ECS and simulation, single threaded vs the job system

| Pass | 1 thread | 8 threads | Speedup |
|---|---|---|---|
| ECS iteration (100k integrate) | 0.700 ms — 7.0 ns/entity | 0.303 ms — 3.0 ns/entity | 2.31x |
| Transform hierarchy (112k) | 0.782 ms — 7.0 ns/entity | 0.374 ms — 3.3 ns/entity | 2.09x |
| Physics step (30k bodies, 45k contacts) | 22.33 ms — 744 ns/body | 7.10 ms — 237 ns/body | 3.15x |
| Cull + batch (37k candidates) | 0.606 ms — 16.4 ns/object | 0.333 ms — 9.0 ns/object | 1.82x |
| **Full simulation frame** | **23.64 ms (42 fps)** | **8.41 ms (119 fps)** | **2.81x** |

Thread scaling on the full simulation frame:

| Threads | 1 | 2 | 3 | 4 | 6 | 8 |
|---|---|---|---|---|---|---|
| ms | 30.39 | 17.23 | 13.58 | 12.41 | 12.38 | 10.77 |
| speedup | 0.78x | 1.37x | 1.74x | 1.91x | 1.91x | 2.19x |

Scaling flattens past four threads because the M3's four efficiency cores are
roughly a third the throughput of its performance cores, and because the
coloured solver has to finish each colour before starting the next.

Where the frame actually goes, from the built-in profiler at 8 threads:

| Zone | avg | p95 |
|---|---|---|
| physics/narrowphase | 5.11 ms | 6.29 ms |
| physics/solve | 1.41 ms | 1.70 ms |
| physics/broadphase | 1.16 ms | 1.41 ms |
| scene/updateTransforms | 0.76 ms | 1.23 ms |
| physics/gather | 0.52 ms | 0.60 ms |
| render/cull | 0.32 ms | 0.45 ms |
| physics/color | 0.31 ms | 0.38 ms |
| ecs/kinematics | 0.27 ms | 0.40 ms |
| render/clusterBounds | 0.04 ms | 0.07 ms |
| render/batchSort | 0.04 ms | 0.04 ms |

### Cluster culling over Morton-ordered pools

The sparse-set ECS lets a pool be permuted freely — the sparse map absorbs the
move, so every handle stays valid. Skein exploits that: the `Renderable`,
`CullBounds` and `WorldTransform` pools are periodically sorted into Morton
(Z-order) sequence, which makes neighbours in the array neighbours in space.
Culling then runs over clusters of 128 consecutive entries and sub-clusters of
16, each with a precomputed AABB. `frustumClassifyAABB` returns Outside,
Inside or Straddling, so a whole cluster resolves with one test and only a
straddling sub-cluster falls back to per-object tests. Because the pools are
aligned to the same order, the cull loop and the instance fill read by index
instead of through the sparse map.

37,000 candidates, same camera, same visible set both ways:

| | Flat, one test per object | Clustered over Morton order |
|---|---|---|
| Cull + batch | 0.316 ms — 8.5 ns/object | **0.231 ms — 6.3 ns/object (1.37x)** |
| Objects individually tested | 37,000 | **4,576 (87.6% resolved by their cluster)** |
| Visible | 25,224 | 25,224 |

Cluster verdicts for that frame: 290 clusters — 56 outside, 155 inside, 79
straddling; the straddling ones break into 2,313 sub-clusters of which 162 are
outside and 184 fully inside.

Objects drift out of Morton order as they move, so `CullSystem::maintain()`
rebuilds the cluster bounds, compares the mean cluster size against what it was
right after the last sort, and re-sorts only once that ratio crosses a
threshold. Over 800 frames of continuous motion:

| | Median cull | Order decay | Re-sorts | Objects tested |
|---|---|---|---|---|
| Sorted once | 0.303 ms | 2.18x | 1 | 16,384 |
| Maintained every 30 frames | **0.278 ms** | 1.16x | 2 | **7,120** |

The win depends on how much of the scene is on screen — clustering has nothing
to skip when everything is visible-adjacent, and nothing to gain when a cheap
sphere test already rejects everything:

| Camera | Visible | Flat | Clustered |
|---|---|---|---|
| Tight, looking away | 0.0% | 0.051 ms | 0.050 ms (1.04x) |
| Narrow fov into the field | 51.5% | 0.201 ms | 0.174 ms (1.15x) |
| Default view | 68.2% | 0.244 ms | 0.188 ms (1.30x) |
| Inside the field, looking down | 92.9% | 0.335 ms | 0.309 ms (1.08x) |
| Far back, whole field in view | 100.0% | 0.368 ms | 0.261 ms (1.41x) |

### Scaling to a million entities

`skein_bench --sweep` rebuilds the world at five sizes with density held
constant (the field grows with the cube root of the population), so the numbers
compare like for like:

| Entities | Build | ECS iterate | Hierarchy | Cull flat | Cull clustered |
|---|---|---|---|---|---|
| 25,000 | 3.2 ms | 0.165 ms | 0.126 ms | 0.131 ms | 0.112 ms (1.17x) |
| 100,000 | 13.2 ms | 0.228 ms | 0.281 ms | 0.337 ms | 0.254 ms (1.33x) |
| 250,000 | 32.0 ms | 0.570 ms | 1.018 ms | 0.996 ms | 0.456 ms (2.19x) |
| 500,000 | 65.2 ms | 1.142 ms | 2.293 ms | 1.985 ms | 1.151 ms (1.72x) |
| 1,000,000 | 133.1 ms | 2.117 ms | 5.515 ms | 4.122 ms | 2.847 ms (1.45x) |

At a million entities the ECS integrate pass is 2.1 ns per entity and 433,333
renderables cull and batch in 2.8 ms, with the clustered and flat paths
agreeing on the visible set at every size.

### Culling and broadphase effectiveness

37,000 candidates against a 65° frustum: **25,230 kept, 11,770 rejected
(31.8%)** in 0.33 ms. The broadphase is the same story at a different scale —
30,000 densely piled bodies produce 1.59M candidate pairs instead of the
4.5×10⁸ an all-pairs test would need, **284x fewer**, and 44,991 real contacts
out of those.

### Rendering, before and after batching

Same scene, same camera, same frame count, only the render path changed. GPU
time is a `GL_TIME_ELAPSED` query read one frame late so it never stalls.
Frame time includes the full simulation step, which is ~8 ms of it.

| Path | Frame | Render CPU | GPU | Submit | Draw calls | Objects |
|---|---|---|---|---|---|---|
| Instanced + culled | **17.19 ms (58 fps)** | 1.12 ms | 9.32 ms | 0.31 ms | 72 | 22,844 |
| Instanced, no culling | 20.81 ms (48 fps) | 1.91 ms | 11.58 ms | 0.55 ms | 72 | 37,316 |
| One draw per object, culled | 31.67 ms (32 fps) | 15.44 ms | 16.10 ms | 14.02 ms | 24,050 | 20,095 |
| One draw per object, no culling | 38.97 ms (26 fps) | 25.69 ms | 26.45 ms | 24.29 ms | 40,771 | 37,316 |

Batching the same 37,316 objects collapses 40,771 draw calls into 72 and cuts
submission from 24.29 ms to 0.55 ms — **44x** — which takes the frame from 26
to 48 fps. Once batched the frame is GPU-bound, and culling removes another
39% of the objects for 3.6 ms more.

### Serialization, scripting, memory

| | |
|---|---|
| Serialize 112,025 entities | 1.51 ms → 21.0 MB (13.6 GB/s) |
| Deserialize | 4.04 ms (5.1 GB/s), entity ids preserved exactly |
| Lua per-entity callbacks | 0.415 ms for 5,000 scripts — 83.1 ns/callback |
| ECS pools / physics / culling / Lua heap | 27.8 MB / 25.7 MB / 917 KB / 385 KB |

## Demo controls

`W A S D`, `Space`/`Ctrl` fly, `Shift` boosts, `Tab` releases the mouse.
`F` toggles frustum culling, `M` cluster culling (the title bar shows how many
objects still needed their own test), `I` instanced batching, `L` shadows, `B` draws
cull bounds, `C` freezes the culling camera so you can fly outside it and watch
objects drop out, `O` pauses, `P` dumps the frame profile, `V` toggles vsync,
`F5`/`F9` save and load `scene.skn`.

## Tests

63 tests, no framework. They cover the parts where being wrong is quiet: the
hashed grid must return exactly the brute-force contact set even when collider
sizes vary 70x, the coloured parallel solver must land bitwise on the serial
result, clustered culling must keep exactly the objects the flat path keeps and
must re-sort only once motion has actually loosened the order, a
2,000-entity hierarchy with destroyed parents must survive a serialization
round trip, threaded transform updates must match single-threaded ones bit for
bit, and `parallelFor` must cover every index exactly once under nesting.
