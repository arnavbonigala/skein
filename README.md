# Skein

A data-oriented 3D engine written from scratch in C++20 — sparse-set ECS,
work-stealing job system, OpenGL 4.1 forward renderer with instanced batching and
shadows, a hashed-grid rigid-body world with a parallel warm-started solver,
binary serialization and embedded Lua.

**One million entities. 30,000 rigid bodies at 105,000 contacts a step. 37,000
renderables in 72 draw calls.** Every number in this repo is measured by the two
benchmark harnesses in it — none is estimated.

| | |
|---|---|
| [BENCHMARKS.md](BENCHMARKS.md) | every table, the method, the A/B protocol |
| [ARCHITECTURE.md](ARCHITECTURE.md) | ECS internals, frame order, jobs, culling, renderer |
| [PHYSICS.md](PHYSICS.md) | grid, narrowphase, colouring, solver, joints, sleeping |
| [SCRIPTING.md](SCRIPTING.md) | the Lua API |

```
                        ┌──────────────────────────────────┐
   Entity ─────────────▶│  ECS — sparse set, packed pools  │
   64-bit handle,       │  Transform[]   Renderable[]      │
   generation-tagged    │  Velocity[]    Collider[]        │
                        │  CullBounds[]  Light[]  Joint[]  │
                        └────┬───────────────┬────────┬────┘
                             │               │        │
             ┌───────────────┘               │        └──────────────┐
             ▼                               ▼                       ▼
   ┌───────────────────┐        ┌─────────────────────┐    ┌──────────────┐
   │ RENDER            │        │ PHYSICS             │    │ SCRIPTS      │
   │ morton sort       │        │ hashed grid         │    │ Lua 5.5      │
   │ cluster cull      │        │ 15-axis SAT         │    │ same pools   │
   │ batch + instance  │        │ coloured solver     │    │ as C++       │
   └───────────────────┘        └─────────────────────┘    └──────────────┘
             │                               │
             └───────────────┬───────────────┘
                             ▼
                    work-stealing job system
                    (0.7 µs dispatch floor)
```

## Build

Requires CMake 3.20, a C++20 compiler, Lua, and GLFW for the interactive demo.

```sh
brew install cmake lua glfw
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

./build/skein_tests            # 101 tests
./build/skein_bench            # headless CPU benchmark
./build/skein_bench --sweep    # plus the 25k → 1M scaling sweep
./build/skein_demo             # interactive window
./build/skein_demo --capture 200   # render-path sweep instead of a window
```

## Headline numbers

| | |
|---|---|
| Entities culled and batched | **1,000,000** — 433k renderables in 2.5 ms |
| Rigid bodies | **30,000** at ~105,000 contacts a step |
| Draw calls for 37,580 objects | **72** — submission 20.69 ms → 0.25 ms (**83x**) |
| Raycast into 30,000 bodies | **0.55 µs** (1.81 M rays/s) — **145x** brute force |
| Sleeping a settled 4,000-body pile | **28.3x** cheaper a step |
| Rope stretch, warm started | **0.2 mm** against 116.7 mm — **580x** for 9% time |
| OBJ parse | **663 MB/s**, 9.3 M triangles/s |
| Serialize 112,025 entities | **1.89 ms** (11.1 GB/s) |
| Job dispatch, 64 chunks | **0.7 µs** — same as 2 chunks |

Measured on an Apple M3 under load; every figure is a ceiling rather than a best
case. Full tables, method and A/B protocol in [BENCHMARKS.md](BENCHMARKS.md).

## Subsystems

| | |
|---|---|
| **ECS** | Sparse set — dense array plus entity→index map per type. Linear iteration, swap-and-pop removal, 64-bit handles with a generation counter so a stale handle never resolves to a recycled slot. Pools registered by stable name behind a type-erased interface, which is what lets serialization walk them generically. |
| **Hierarchy** | No recursion, no pointer chasing. Depths are memoized and counting-sorted into a flat order, so each level is a contiguous span whose parents are already final — and therefore trivially parallel. |
| **Jobs** | One deque per worker, LIFO for the owner and FIFO for thieves. The submitter participates rather than blocking, so nested `parallelFor` makes progress instead of deadlocking. One task per *worker*, not per chunk, with chunks handed out through an atomic cursor. |
| **Renderer** | OpenGL 4.1 core against the macOS system framework — no loader library. Counting-sorted `(mesh, material)` batches, one orphaned instance buffer per frame with the shadow pass appended so there is a single upload. Blinn-Phong with a metallic tint, 2048² PCF sun shadows, 32 point lights chosen per frame by distance. |
| **Culling** | Gribb-Hartmann planes, sphere prepass, center/extent AABB — with the per-object test as the *fallback*. See [Cluster culling](#cluster-culling) below. |
| **Physics** | Hashed uniform grid, 15-axis SAT with Sutherland-Hodgman clipping, warm-started sequential impulses solved over a coloured contact graph in four substeps of two iterations. See [The solver](#the-solver) below. |
| **Joints** | Distance constraints solved by the same accumulated-impulse machinery as contacts, with XPBD compliance exposed as metres of stretch per newton-second. Length error routes through the split-impulse channel, not as a bias inside the carried impulse — a bias there replays last frame's correction on top of this frame's and a hanging chain climbs above its own anchor. |
| **Assets** | Hand-written OBJ parser — all four face index forms, negative indices, fan triangulation, welding, generated normals. Walked in place rather than through `getline`/`sscanf`, welded through an open-addressed table. Batch loads parse concurrently and register deterministically. |
| **Serialization** | Versioned binary format writing each registered pool verbatim. Entity ids, generations and holes survive a round trip; unknown or size-mismatched pools are skipped rather than fatal. |
| **Scripting** | Lua 5.5 through the C API, driving the same pools the C++ systems read — a scripted entity is culled, lit and simulated exactly like a native one. `skein.raycast`, `skein.overlap_sphere`, `skein.joint`/`unjoint`. A callback that errors is unbound and reported instead of killing the frame. |

## Cluster culling

A sparse set lets a pool be permuted freely — the sparse map absorbs the move, so
every handle stays valid. Skein spends that freedom: `Renderable`, `CullBounds`
and `WorldTransform` are sorted into Morton order, which makes neighbours in the
array neighbours in space.

```
   128 objects        one AABB test
   ────────────  ──▶  ┌──────────┐
   cluster            │ Outside  │ ──▶ skip 128
                      │ Inside   │ ──▶ keep 128
                      │ Straddle │ ──▶ 8 sub-clusters of 16
                      └──────────┘        └─▶ per-object test
```

| | Flat | Clustered |
|---|---|---|
| Cull + batch, 37k candidates | 0.339 ms | **0.222 ms (1.53x)** |
| Objects individually tested | 37,000 | **4,752** |

Across six cameras — from an empty view to the whole field on screen — no camera
drives more than a fifth of the objects down to their own test. Clustering wins
in five of the six; the exception is the empty view, where the sphere prepass
already rejects everything in one compare. At a million entities it still
resolves **88.5%** of objects by cluster.

Order decays as objects move, so `maintain()` compares mean cluster size against
what it was right after the last sort and re-sorts only once that ratio crosses a
threshold: **4,912** objects individually tested over 800 frames of motion
against 11,472 for a pool sorted once.

## The solver

Four substeps of two iterations, not one pass of eight. A substep integrates
velocity, sweeps the contacts, then integrates position — so every sweep after
the first answers where the pair is *now*.

```
  narrowphase ──▶ colour ──▶ ┌─ substep ×4 ────────────────────┐
  (once a step)   (once)     │  integrate v                    │
                             │  prepare  (re-derive depth from │
                             │            the anchors, turned) │
                             │  solve ×2 ──▶ colour 0..N in    │
                             │              parallel           │
                             │  integrate x                    │
                             └─────────────────────────────────┘
                                        ──▶ restitution, once
```

Each contact keeps the arms it was born with rather than a world-space point, so
turning them by however far each body has turned since re-derives live depth for
free. Restitution lands once, after the substeps: mixed into them it fights its
own output, because the substep answering a bounce sees the body leaving and
winds the impulse back down.

**Two layout decisions carry the solve**, which is 81% of the step:

| | |
|---|---|
| Contacts | The counting sort that assigns colours moves the *contacts themselves* rather than building an index array, so eight sweeps read a stride instead of a gather — worth 9% of the whole step. |
| Bodies | A contact reaches two bodies at indices no ordering can make contiguous. Velocity, angular velocity, inverse mass, inverse inertia and the six numbers of the world inverse-inertia tensor live in one 64-byte `SolverBody` — exactly one cache line, asserted at compile time. Two lines pulled per contact instead of ten. |

The graph is coloured greedily into 64 colours with one bitmask per body — 42 of
them on the benchmark scene, none left over to solve serially. A test asserts the
parallel result is **bitwise identical** to the serial one over 90 steps of a
4,000-body pile.

### What it buys

| Behaviour | Mechanism |
|---|---|
| A stack of eight turned boxes stands | Four substeps × two iterations; nothing below six sweeps stands at all |
| A resting pile sleeps | Warm starting converges in 2 iterations what cold needs 32 for |
| A 220 m/s shot stops at a 0.5 m wall | Speculative contacts — negative depth read as a bound on approach, not an overlap to push out |
| An 80 m/s ball at restitution 0.8 rebounds above 40 m/s | Fastest approach seen while apart is cached next to the impulse and spent on the frame they meet |
| A box rests on a face instead of rocking | Four-point manifold keeping the widest quad, each point named by the feature that made it so its impulse finds it again |
| A slab tumbles end over end, not about its length | Real inertia tensor, rotated into the world once a step |
| A skidding ball rolls, then stops | Rolling resistance — at curved contacts only, since a box face is already held by four points friction acts at |
| A column does not walk itself over | The positional pass translates and never rotates |
| Ice on rubber lands between ice and rubber | Per-collider friction, geometric mean of the pair |

## Demo controls

| | | | |
|---|---|---|---|
| `W A S D` | fly | `F` | frustum culling |
| `Space` / `Ctrl` | up / down | `M` | cluster culling |
| `Shift` | boost | `I` | instanced batching |
| `Tab` | release mouse | `L` | shadows |
| `O` | pause | `B` | draw cull bounds |
| `P` | dump frame profile | `C` | freeze the culling camera and fly outside it |
| `V` | vsync | `F5` / `F9` | save / load `scene.skn` |

## Tests

101 tests, no framework. They cover the places where being wrong is quiet:

- The hashed grid returns **exactly** the brute-force contact set, even with collider sizes varying 70x
- The coloured parallel solver lands **bitwise** on the serial result
- Eight spheres, and sixteen columns of eight turned boxes, are still standing after ten seconds
- A sleeping pile is no more interpenetrated than a waking one, and wakes when a script throws it somewhere else
- A bullet **does** go through a wall when speculation and substepping are both off, and does not when either is on
- A body crossing twelve times its own radius in a step does not end up on the far side
- An off-centre hit spins the body; a hit through the centres does not
- A box set down on a face rests on four contacts, not balanced on one
- A slab tumbles end over end at least 5x more readily than it spins about its length
- A skidding ball rolls, then stops, without the motion running backwards
- A slab slides furthest on ice, least on rubber, in between on one of each
- A chain hangs at rest length rather than stretching or climbing — and still does after a save/load round trip
- Cutting one rope does not disturb the rope hanging next to it
- Clustered culling keeps exactly what the flat path keeps, and re-sorts only once motion has actually loosened the order
- Spheres, turned boxes, ropes, sleeping, bounds and the job system all run together for twenty seconds without a body leaving the box, a quaternion leaving the unit sphere, or a number leaving the reals
- A 2,000-entity hierarchy with destroyed parents survives a serialization round trip
- Threaded transform updates match single-threaded ones bit for bit
- `parallelFor` covers every index exactly once under nesting
