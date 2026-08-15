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
./build/skein_tests     # 66 tests
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
single oversized collider no longer coarsens the whole grid. Entries are
counting-sorted into hash buckets and then sorted within each bucket by an
exact packed 64-bit cell key, which turns every bucket into contiguous runs of
one cell; the scan walks those runs and tests pairs directly instead of
rehashing each body's cells and filtering foreign entries out of the bucket
(narrowphase 5.11 ms → 1.98 ms on the benchmark scene). Pairs are deduped by
reporting only from the lowest cell two bodies share. The solver colours the contact graph greedily (64 colours, one bitmask
per body) and runs each colour in parallel; a test asserts the parallel result
is bitwise identical to the serial one over 90 steps of a 4,000-body pile.

**Physics** — SoA gather/scatter around a hashed uniform grid built with a
counting sort, a cell-run scan that verifies real cell coordinates to reject
hash collisions, and a warm-started sequential-impulse solver: each contact
carries an accumulated normal impulse keyed by entity pair and cached across
frames, so a resting stack starts every frame already holding itself up. On top
of that sit Coulomb friction bounded by that accumulated impulse, restitution
captured as a bias before the first iteration, world bounds solved as a
constraint rather than clamped afterwards, split-impulse positional correction
that never feeds real velocity, and body sleeping. A test asserts the grid finds
exactly the same contact set as an O(n²) reference over 3000 bodies, and another
that an eight-sphere column still stands after ten seconds.

Warm starting is what makes the rest work. Cancelling each contact's approach
velocity from scratch every frame cannot hold a stack up — the reaction has to
travel down the column one contact per iteration, so at two iterations a column
sinks through itself and the depth push papers over the result. Reusing last
frame's impulse converges the same column in two iterations that otherwise
needed thirty-two, which in turn is what lets a pile go still enough to sleep.

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
| ECS iteration (100k integrate) | 0.696 ms — 7.0 ns/entity | 0.235 ms — 2.3 ns/entity | 2.96x |
| Transform hierarchy (112k) | 0.753 ms — 6.7 ns/entity | 0.313 ms — 2.8 ns/entity | 2.41x |
| Physics step (30k bodies, 45k contacts) | 12.19 ms — 406 ns/body | 4.25 ms — 142 ns/body | 2.87x |
| Cull + batch (37k candidates) | 0.594 ms — 16.1 ns/object | 0.318 ms — 8.6 ns/object | 1.87x |
| **Full simulation frame** | **13.52 ms (74 fps)** | **5.47 ms (183 fps)** | **2.47x** |

Thread scaling on the full simulation frame:

| Threads | 1 | 2 | 3 | 4 | 6 | 8 |
|---|---|---|---|---|---|---|
| ms | 17.68 | 10.80 | 8.54 | 7.02 | 7.27 | 7.00 |
| speedup | 0.76x | 1.25x | 1.58x | 1.93x | 1.86x | 1.93x |

Scaling flattens past four threads because the M3's four efficiency cores are
roughly a third the throughput of its performance cores, and because the
coloured solver has to finish each colour before starting the next.

Where the frame actually goes, from the built-in profiler at 8 threads:

| Zone | avg | p95 |
|---|---|---|
| physics/narrowphase | 1.98 ms | 2.37 ms |
| physics/solve | 1.47 ms | 1.76 ms |
| physics/broadphase | 0.92 ms | 1.00 ms |
| scene/updateTransforms | 0.65 ms | 0.77 ms |
| physics/gather | 0.49 ms | 0.52 ms |
| physics/color | 0.28 ms | 0.33 ms |
| render/cull | 0.27 ms | 0.33 ms |
| ecs/kinematics | 0.25 ms | 0.33 ms |
| render/clusterBounds | 0.04 ms | 0.07 ms |
| render/batchSort | 0.03 ms | 0.04 ms |

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
| Cull + batch | 0.315 ms — 8.5 ns/object | **0.254 ms — 6.9 ns/object (1.24x)** |
| Objects individually tested | 37,000 | **4,608 (87.5% resolved by their cluster)** |
| Visible | 25,224 | 25,224 |

Cluster verdicts for that frame: 290 clusters — 55 outside, 154 inside, 81
straddling; the straddling ones break into 2,313 sub-clusters of which 167 are
outside and 193 fully inside.

Objects drift out of Morton order as they move, so `CullSystem::maintain()`
rebuilds the cluster bounds, compares the mean cluster size against what it was
right after the last sort, and re-sorts only once that ratio crosses a
threshold. Over 800 frames of continuous motion:

| | Median cull | Order decay | Re-sorts | Objects tested |
|---|---|---|---|---|
| Sorted once | 0.286 ms | 2.18x | 1 | 16,432 |
| Maintained every 30 frames | **0.250 ms** | 1.16x | 2 | **7,040** |

The win depends on how much of the scene is on screen — clustering has nothing
to skip when everything is visible-adjacent, and nothing to gain when a cheap
sphere test already rejects everything:

| Camera | Visible | Flat | Clustered |
|---|---|---|---|
| Tight, looking away | 0.0% | 0.041 ms | 0.049 ms (0.83x) |
| Narrow fov into the field | 51.5% | 0.203 ms | 0.184 ms (1.10x) |
| Default view | 68.2% | 0.307 ms | 0.207 ms (1.48x) |
| Inside the field, looking down | 92.9% | 0.314 ms | 0.246 ms (1.27x) |
| Far back, whole field in view | 100.0% | 0.345 ms | 0.272 ms (1.27x) |

When nothing is on screen the sphere prepass already rejects everything and the
cluster tests are pure overhead, which is the one case clustering loses.

### What the layout is actually worth

The same integrate over 200,000 objects, three ways: one polymorphic object per
entity reached through a shuffled pointer array (the classic scene graph), the
same objects stored contiguously and called directly, and loose component
arrays the way the ECS stores them.

| | Time | Per object | |
|---|---|---|---|
| Virtual call, pointer per object | 5.555 ms | 27.8 ns | 184 B/object |
| Contiguous objects, direct call | 1.335 ms | 6.7 ns | **4.16x** |
| Component arrays | 1.348 ms | 6.7 ns | **4.12x** |
| Position only, contiguous objects | 0.560 ms | 2.8 ns | |
| Position only, component arrays | **0.075 ms** | **0.4 ns** | **7.48x** |

Most of the first 4.1x is dispatch and pointer chasing, not layout — with the
whole object touched, contiguous AoS keeps up with component arrays because the
quaternion math dominates. Layout only separates them when a pass touches part
of an object: reading two fields walks 9.9 MB out of component arrays instead of
dragging 35.1 MB of cache lines, and that is worth 7.5x.

### Scaling to a million entities

`skein_bench --sweep` rebuilds the world at five sizes with density held
constant (the field grows with the cube root of the population), so the numbers
compare like for like:

| Entities | Build | ECS iterate | Hierarchy | Cull flat | Cull clustered |
|---|---|---|---|---|---|
| 25,000 | 3.4 ms | 0.147 ms | 0.098 ms | 0.103 ms | 0.100 ms (1.03x) |
| 100,000 | 12.2 ms | 0.201 ms | 0.355 ms | 0.327 ms | 0.259 ms (1.26x) |
| 250,000 | 30.6 ms | 0.521 ms | 0.991 ms | 0.843 ms | 0.537 ms (1.57x) |
| 500,000 | 62.3 ms | 1.100 ms | 2.576 ms | 2.014 ms | 1.349 ms (1.49x) |
| 1,000,000 | 134.0 ms | 2.166 ms | 5.739 ms | 4.439 ms | 2.380 ms (1.86x) |

At a million entities the ECS integrate pass is 2.1 ns per entity and 433,333
renderables cull and batch in 2.4 ms, with the clustered and flat paths
agreeing on the visible set at every size.

### What warm starting buys a stack

Thirty-two columns of eight spheres, dropped and left for ten seconds, then
measured by where the top sphere ended up. A perfect stack puts it at 7.50.

| Solver iterations | Cold | Warm started |
|---|---|---|
| 2 | 3.68 — 45% of the stack standing | 7.25 — **96%** |
| 4 | 6.14 — 81% | 7.35 — **98%** |
| 8 | 7.24 — 96% | 7.41 — **99%** |
| 16 | 7.39 — 98% | 7.43 — **99%** |

A cold solver needs eight iterations to reach what warm starting reaches in
two, because the reaction holding a column up travels one contact per
iteration. Reusing each contact's accumulated impulse removes that dependency
on stack height, and it is what makes sleeping possible at all: a pile that
never converges never goes still enough to sleep.

### Culling and broadphase effectiveness

37,000 candidates against a 65° frustum: **25,230 kept, 11,770 rejected
(31.8%)** in 0.32 ms. The broadphase is the same story at a different scale —
30,000 densely piled bodies produce 1.88M candidate pairs instead of the
4.5×10⁸ an all-pairs test would need, **239x fewer**, and 44,971 real contacts
out of those.

### Rendering, before and after batching

Same scene, same camera, same frame count, only the render path changed. The
world is settled for 240 frames and then held still, so the four rows differ in
exactly one thing. GPU time is a `GL_TIME_ELAPSED` query read a frame late;
figures are medians over 200 frames with vsync off.

| Path | CPU frame | GPU | Ceiling | Render CPU | Submit | Draw calls | Objects |
|---|---|---|---|---|---|---|---|
| Instanced + culled | **1.35 ms** | 5.33 ms | **188 fps** | 0.98 ms | 0.22 ms | 72 | 22,915 |
| Instanced, no culling | 2.36 ms | 6.40 ms | 156 fps | 1.84 ms | 0.32 ms | 72 | 37,316 |
| One draw per object, culled | 16.57 ms | 14.07 ms | 60 fps | 13.76 ms | 12.92 ms | 26,579 | 22,915 |
| One draw per object, no culling | 23.83 ms | 21.50 ms | 42 fps | 22.83 ms | 21.75 ms | 40,980 | 37,316 |

Batching the same 37,316 objects collapses 40,980 draw calls into 72 and cuts
submission from 21.75 ms to 0.32 ms — **68x** — taking the CPU frame from 23.83
to 2.36 ms and the ceiling from 42 to 156 fps. Once batched the frame is
GPU-bound, and culling removes another 39% of the objects for 1.1 ms of GPU
time on top.

### Serialization, scripting, memory

| | |
|---|---|
| Serialize 112,025 entities | 1.85 ms → 21.0 MB (11.1 GB/s) |
| Deserialize | 4.66 ms (4.4 GB/s), entity ids preserved exactly |
| Lua per-entity callbacks | 0.393 ms for 5,000 scripts — 78.6 ns/callback |
| ECS pools / physics / culling / Lua heap | 27.8 MB / 22.0 MB / 917 KB / 385 KB |

## Demo controls

`W A S D`, `Space`/`Ctrl` fly, `Shift` boosts, `Tab` releases the mouse.
`F` toggles frustum culling, `M` cluster culling (the title bar shows how many
objects still needed their own test), `I` instanced batching, `L` shadows, `B` draws
cull bounds, `C` freezes the culling camera so you can fly outside it and watch
objects drop out, `O` pauses, `P` dumps the frame profile, `V` toggles vsync,
`F5`/`F9` save and load `scene.skn`.

## Tests

66 tests, no framework. They cover the parts where being wrong is quiet: the
hashed grid must return exactly the brute-force contact set even when collider
sizes vary 70x, the coloured parallel solver must land bitwise on the serial
result, clustered culling must keep exactly the objects the flat path keeps and
must re-sort only once motion has actually loosened the order, a
2,000-entity hierarchy with destroyed parents must survive a serialization
round trip, threaded transform updates must match single-threaded ones bit for
bit, and `parallelFor` must cover every index exactly once under nesting.
