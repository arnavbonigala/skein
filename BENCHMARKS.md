# Benchmarks

Every figure here is measured by the two harnesses in this repo. None is estimated.

```sh
./build/skein_bench                # CPU tables
./build/skein_bench --sweep        # plus the 25k → 1M scaling sweep
./build/skein_demo --capture 200   # render tables
```

## Method

| | |
|---|---|
| Machine | Apple M3, 8 hardware threads, macOS 26.6 |
| Build | Apple clang `-O2`, OpenGL 4.1 (Metal 90.5) |
| CPU figures | medians over 40 runs, `skein_bench`, one session |
| Render figures | means over 200 frames, `skein_demo --capture 200` |
| Load average during the CPU run | 3.46 |

**Preemption filter.** Every timed sample reads `ru_nivcsw` on either side of
itself and is discarded if the scheduler preempted the thread mid-measurement, so
a loaded machine costs coverage rather than accuracy. This run took 3,506 samples
and dropped 2,831 of them.

**Core-milliseconds.** Headline rows also report the CPU time the process
actually burned, summed over its threads, because wall time on a shared machine
measures the machine:

```
                          drift across runs of this session
  wall time, full frame   28 ms ──────────────────────── 119 ms   4.3x
  core-ms, physics step   83  ────── 100                          1.2x
```

Core-ms is the more honest of the two, but it is not an invariant — contention
inflates CPU time through the cache and memory bus as well. Compare two builds by
it *within one session, back to back*. Every A/B below was measured that way,
alternating binaries in one loop so the drift lands on both.

**Every timing here is a ceiling, not a best case.** No run was cherry-picked for
speed; the published run is the one whose parts agree with each other. An idle
machine measures faster.

## The scene

Both harnesses build the same world from `Demo::build`:

| | |
|---|---|
| Entities | 112,025 (100,000 integrating position + rotation per frame) |
| Renderable | 37,000 |
| Rigid bodies | 30,000, piled densely enough for **105,436 contacts** a step |
| Lights / meshes / materials | 25 / 6 / 6 |
| Resident | 31.8 MB |

The interactive demo adds 24 ropes of ten jointed spheres and runs
`assets/scripts/demo.lua`, for 112,605 entities and 37,580 renderables in the
render table.

## ECS and simulation

| Pass | 1 thread | 8 threads | Speedup |
|---|---|---|---|
| ECS iteration (100k integrate) | 0.91 ms — 0.83 core-ms | 0.23 ms — 1.50 core-ms | 4.03x |
| Transform hierarchy (112k) | 0.75 ms — 0.77 core-ms | 0.34 ms — 2.09 core-ms | 2.19x |
| Physics step (30k bodies, 105k contacts) | 82.3 ms — 82.7 core-ms | 29.8 ms — 160.9 core-ms | **2.76x** |
| Cull + batch (37k candidates) | 0.60 ms — 0.60 core-ms | 0.36 ms — 1.67 core-ms | 1.65x |
| **Full simulation frame** | **80.6 ms — 80.6 core-ms** | **27.8 ms — 138.9 core-ms** | **2.90x** |

The core-ms columns price the parallelism: the physics step does 1.95x the work
to run 2.76x faster. The transform hierarchy does 2.7x the work for 2.2x, because
its levels are shallow enough that dispatch is a real fraction of the pass.

Frame-to-frame spread, same run:

| | best | median | p99 | worst |
|---|---|---|---|---|
| Single threaded | 77.68 ms | 80.57 ms | 84.19 ms (+4%) | 101.58 ms |
| Job system | 26.22 ms | 27.77 ms | 38.96 ms (+40%) | 68.47 ms |

### Thread scaling

| Threads | 1 | 2 | 3 | 4 | 6 | 8 |
|---|---|---|---|---|---|---|
| ms | 80.66 | 42.90 | 33.14 | 28.08 | 27.87 | 28.54 |
| speedup | 1.00x | 1.88x | 2.43x | 2.87x | 2.89x | 2.83x |

Sampled round robin, one frame per thread count per pass, rather than one thread
count at a time — measured in blocks the table records when the load spiked
rather than a scaling curve.

Two things flatten it past four threads: the M3's four efficiency cores are
roughly a third the throughput of its performance cores, and the coloured solver
must finish each colour before starting the next — **42 colours across 8 sweeps
is 336 barriers a step**.

### Where the frame goes

At 8 threads, from the built-in profiler:

| Zone | avg | p95 | calls/frame |
|---|---|---|---|
| physics/step | 31.52 ms | 46.37 ms | 1 |
| ├ physics/solve | 25.48 ms | 41.01 ms | 1 |
| ├ physics/narrowphase | 3.44 ms | 4.14 ms | 1 |
| ├ physics/impulseCache | 1.52 ms | 1.61 ms | 1 |
| ├ physics/prepare | 1.48 ms | 1.90 ms | 4 |
| ├ physics/broadphase | 1.11 ms | 1.35 ms | 1 |
| ├ physics/integrate | 0.94 ms | 1.15 ms | 8 |
| ├ physics/color | 0.83 ms | 1.03 ms | 1 |
| ├ physics/impulseStore | 0.71 ms | 0.96 ms | 1 |
| ├ physics/gather | 0.48 ms | 0.51 ms | 1 |
| ├ physics/scatter | 0.14 ms | 0.19 ms | 1 |
| └ physics/sleep | 0.05 ms | 0.06 ms | 1 |
| scene/updateTransforms | 0.76 ms | 1.06 ms | 1 |
| render/cull | 0.31 ms | 0.43 ms | 1 |
| ecs/kinematics | 0.28 ms | 0.39 ms | 1 |
| render/batchSort | 0.04 ms | 0.04 ms | 1 |
| render/clusterBounds | 0.04 ms | 0.06 ms | 1 |

The solve is 81% of the step and everything else is rounding, which is what eight
sweeps over 105,000 contacts should look like.

### What rotation costs

| | Step | |
|---|---|---|
| Linear contacts only | 19.32 ms | no contact torque, no orientation integration |
| Angular contacts | 26.18 ms | **+35.5%** |

## Data layout

The same integrate over 200,000 objects, three ways: one polymorphic object per
entity through a shuffled pointer array (the classic scene graph), the same
objects contiguous and called directly, and loose component arrays.

| | Time | Per object | |
|---|---|---|---|
| Virtual call, pointer per object | 6.123 ms | 30.6 ns | 184 B/object |
| Contiguous objects, direct call | 1.359 ms | 6.8 ns | **4.50x** |
| Component arrays | 1.359 ms | 6.8 ns | **4.51x** |
| Position only, contiguous objects | 0.623 ms | 3.1 ns | |
| Position only, component arrays | **0.074 ms** | **0.4 ns** | **8.37x** |

Most of the first 4.5x is dispatch and pointer chasing, not layout — with the
whole object touched, contiguous AoS keeps up with component arrays because the
quaternion math dominates.

Layout separates them only when a pass touches *part* of an object: reading two
fields walks **9.9 MB** out of component arrays instead of dragging **35.1 MB**
of cache lines.

## Cluster culling

37,000 candidates, same camera, same visible set both ways:

| | Flat, one test per object | Clustered over Morton order |
|---|---|---|
| Cull + batch | 0.339 ms — 9.2 ns/object | **0.222 ms — 6.0 ns/object (1.53x)** |
| Objects individually tested | 37,000 | **4,752 (87.2% resolved by their cluster)** |
| Visible | 25,224 | 25,224 |

Cluster verdicts for that frame: 290 clusters — 56 outside, 154 inside, 80
straddling; the straddling ones break into 2,313 sub-clusters, 155 outside and
188 fully inside.

### Across cameras

A cluster pays when it lands wholly in or wholly out of the frustum, and a camera
can be chosen to make that likely. Six of them, from an empty view to the whole
field on screen:

| Camera | Flat | Clustered | | Visible | Tested |
|---|---|---|---|---|---|
| Tight, looking away | 0.035 ms | 0.037 ms | 0.97x | 0.0% | 0.0% |
| Narrow fov into the field | 0.221 ms | 0.184 ms | 1.20x | 51.5% | 19.0% |
| Default view | 0.281 ms | 0.236 ms | 1.19x | 68.2% | 12.2% |
| Wide fov, centred | 0.293 ms | 0.184 ms | **1.59x** | 64.0% | 13.9% |
| Inside the field, looking down | 0.388 ms | 0.341 ms | 1.14x | 92.9% | 18.6% |
| Far back, whole field in view | 0.397 ms | 0.270 ms | **1.47x** | 100.0% | 0.0% |

The last column is why it holds: no camera drives more than a fifth of the
objects down to their own test. The hardest case is the one that looks easiest —
standing inside the field with almost everything visible, where the clusters
surrounding the camera are the likeliest to straddle — and it still wins there.

The one row it does not win is the empty view, where the sphere prepass already
rejects everything in a single compare and the cluster tests have nothing left to
save.

### Order maintenance

Objects drift out of Morton order as they move. `CullSystem::maintain()` rebuilds
the cluster bounds, compares mean cluster size against what it was right after
the last sort, and re-sorts only once that ratio crosses a threshold. Over 800
frames of continuous motion:

| | Median cull | Order decay | Re-sorts | Objects tested |
|---|---|---|---|---|
| Sorted once | 0.306 ms | 1.78x | 1 | 11,472 |
| Maintained every 30 frames | 0.309 ms | **1.02x** | 2 | **4,912** |

Two re-sorts over 800 frames hold the order at 1.02x of freshly sorted and cut
individually tested objects by 57%.

## Scaling to a million entities

`skein_bench --sweep` rebuilds the world at five sizes with density held constant
— the field grows with the cube root of the population — so the rows compare like
for like. (Its own run; the sweep is skipped by default.)

| Entities | Build | ECS iterate | Hierarchy | Cull flat | Cull clustered |
|---|---|---|---|---|---|
| 25,000 | 3.4 ms | 0.106 ms | 0.118 ms | 0.137 ms | 0.084 ms (1.62x) |
| 100,000 | 11.7 ms | 0.180 ms | 0.287 ms | 0.368 ms | 0.282 ms (1.30x) |
| 250,000 | 32.8 ms | 0.573 ms | 0.947 ms | 0.981 ms | 0.535 ms (1.83x) |
| 500,000 | 61.8 ms | 1.227 ms | 2.430 ms | 2.205 ms | 1.305 ms (1.69x) |
| 1,000,000 | 135.7 ms | 2.392 ms | 5.004 ms | 4.244 ms | 2.547 ms (1.67x) |

At a million entities the ECS integrate pass is **2.4 ns per entity** and 433,333
renderables cull and batch in **2.5 ms**, with the clustered and flat paths
agreeing on the visible set at every size.

Clustering holds as the population grows rather than degrading into it: the sweep
builds 3,386 clusters at a million entities, 807 of them straddling, and 49,792
of 433,333 objects — **11.5%** — need a frustum test of their own.

## Broadphase funnel

30,000 densely piled bodies produce 1.88M candidate pairs against the 4.5×10⁸ an
all-pairs test needs — **276x fewer**. What happens to those 1.88M:

```
  1,881,821  share a cell           ─┐  reads only the sorted entry array
    248,239  within reach (13.2%)   ─┤  one distance compare
    106,795  this cell's to report  ─┤  six ints from the body's cell range
    105,436  real contacts          ─┘  full sphere/box narrowphase
```

87% of the work is rejected before touching a body array at all, which is why an
entry carries its own position and reach rather than an index to look one up.

Almost everything surviving all three filters is a real contact — what a grid
sized to the *typical* body rather than the largest one buys.

Frustum culling is the same story at a different scale: 37,000 candidates against
a 65° frustum leaves **25,230 kept, 11,770 rejected (31.8%)** in 0.35 ms, in **36
draw calls** at 701 instances each.

## Warm starting

Thirty-two columns of eight spheres, dropped and left for ten seconds. A perfect
stack puts the top sphere at 7.50.

| Budget | Cold | Warm started |
|---|---|---|
| 1 substep × 2 iterations | 4.34 — 0 of 32 columns intact | 5.32 — 0 of 32 |
| 2 substeps × 2 iterations | 6.38 — 0 of 32 | **7.09 — 32 of 32** |
| 4 substeps × 2 iterations (default) | 7.45 — 32 of 32 | 7.46 — 32 of 32 |

The middle row is the result: at the same budget, warm starting is the difference
between every column standing and none. Mean height hides that — 6.38 of 7.50
sounds like a stack leaning, and it is thirty-two stacks that came down and piled
up — which is why the intact count sits next to it.

The top row is the other half. At one substep the column comes down at two, four,
eight and sixteen iterations alike: the reaction travels one contact per sweep,
so warm starting removes the dependency on *height* but not the one on time.

## Substeps versus iterations

Sixteen columns of eight turned boxes, every mix, sorted by what it costs. Rows
sharing a sweep count are the same solver work spent two ways.

| Mix | Sweeps | Cost | Top box | Lean |
|---|---|---|---|---|
| 1 × 1 | 1 | 0.119 ms | 0.62 of 7.50 (2%) | 101.2° |
| 1 × 2 | 2 | 0.125 ms | 0.56 (1%) | 92.1° |
| 2 × 1 | 2 | 0.137 ms | 0.69 (3%) | 95.1° |
| 1 × 3 | 3 | 0.138 ms | 0.60 (1%) | 106.5° |
| 3 × 1 | 3 | 0.183 ms | 0.68 (3%) | 122.2° |
| 4 × 1 | 4 | 0.198 ms | 4.18 (53%) | 42.6° |
| 2 × 2 | 4 | 0.201 ms | 1.15 (9%) | 79.2° |
| 2 × 3 | 6 | 0.243 ms | 7.02 (93%) | 9.6° |
| 3 × 2 | 6 | 0.252 ms | 6.69 (88%) | 15.3° |
| **4 × 2 (default)** | **8** | **0.315 ms** | **7.48 (100%)** | **1.3°** |
| 3 × 3 | 9 | 0.337 ms | 7.48 (100%) | 1.0° |
| 4 × 3 | 12 | 0.430 ms | 7.48 (100%) | 0.7° |

Nothing below six sweeps stands. Six very nearly does either way round. Four
substeps of two iterations is the cheapest mix that actually stands, which is why
it is the default.

Substeps buy fresh geometry, iterations buy news travelling down the column, and
a stack needs both. Axis-aligned, the same 512 contacts stand at 4 × 1 for
0.161 ms.

## What the solver leaves behind

Speed is half of a solver; the other half is how much of the pile is still inside
itself when the motion stops. 2,000 spheres dropped into a 24 m box, left for
20 s, then every one of the 2M pairs checked directly:

| Solver | Step | Worst overlap | Mean | Fastest body |
|---|---|---|---|---|
| 1 iteration, warm | 2.07 ms | 5.3% of a radius | 1.1% | 0.453 m/s |
| 2 iterations, warm (default) | 3.06 ms | **1.8%** | 0.9% | 0.244 m/s |
| 4 iterations, warm | 5.08 ms | 1.3% | 0.8% | 0.404 m/s |
| 8 iterations, warm | 9.60 ms | **1.1%** | **0.8%** | **0.174 m/s** |
| 2 iterations, cold | 2.85 ms | 2.2% | 0.9% | 0.370 m/s |

Nothing is deeply interpenetrated. What is left is a floor around 1% that
iterations buy back very slowly, because it is a deep column the positional pass
unwinds one contact per iteration.

Warm starting costs the pile 0.4% of a radius here, because four substeps
re-derive the contact geometry often enough to do most of that job themselves. It
earns its keep where the chain is long — see the ropes below.

## Joints

Four hundred twelve-link ropes, 4,800 joints. The top joint carries eleven links
of load and hears about it one joint per sweep.

| | Step cost | Stretch at the free end |
|---|---|---|
| Warm started | 1.555 ms | **0.2 mm** |
| Solved from nothing each step | 1.424 ms | 116.7 mm |

**580x less stretch for 9% more time.**

## Fast bodies against a thin wall

400 spheres of radius 0.15 fired at 20–220 m/s at a 0.5 m static slab. At the top
speed that is 3.7 m of travel in a 1/60 s step.

| Contact test | Substep cap | Step cost | Splits used | Passed through |
|---|---|---|---|---|
| discrete | 1 | 0.058 ms | 1 | 277 of 400 |
| discrete | 4 | 0.495 ms | 4 | 68 of 400 |
| discrete | 16 | 0.184 ms | 13 | **0 of 400** |
| speculative | 1 | 0.161 ms | 1 | **0 of 400** |
| speculative | 4 (default) | 0.175 ms | 2 | **0 of 400** |

A discrete test has to split the step thirteen ways to catch every shot.
Speculative contacts catch all of them without splitting it at all.

The tightest discrete cap is not the slowest: a shot that stops at the wall stops
needing splits, while every shot that escapes keeps moving fast enough to split
every later step.

Widening is what keeps it cheap. A body slower than its own radius per step is
not widened at all, so the 30,000-body field tests **3.1% fewer** pairs with
speculative contacts on than off, and the whole feature costs 5.3% of the step.

## Sleeping

Timed single threaded, because the saving is work, not parallelism.

| Scene | Sleeping off | Sleeping on | |
|---|---|---|---|
| 4,000 in a box, 30 s to settle | 8.54 ms — 4,000 awake, 10,713 contacts | **0.30 ms** — 0 awake, 0 contacts | **28.3x** |
| 30,000 demo field, still churning | 87.74 ms — 30,000 awake, 87,387 contacts | 86.30 ms — 25,728 awake, 84,376 contacts | 1.02x |

The second row is the point of the first: a pile that never settles pays only the
bookkeeping for a feature it cannot use.

## Queries

| | |
|---|---|
| 4,096 rays into 30,000 bodies | 2.27 ms — **0.55 µs/ray, 1.81 M rays/s**, 1,440 hit |
| Same rays, every body tested | 80.12 µs/ray — **145x slower** |
| 1,024 sphere overlaps, radius 3 | 8.78 µs/query, 48,509 bodies returned |

## Rendering

Same scene, same camera, same frame count, only the render path changed. The
world is settled for 240 frames then held still, so the four rows differ in
exactly one thing. GPU time is a `GL_TIME_ELAPSED` query read a frame late;
medians over 200 frames, vsync off.

| Path | CPU frame | GPU | Ceiling | Render CPU | Submit | Draw calls | Objects |
|---|---|---|---|---|---|---|---|
| Instanced + culled | **1.26 ms** | 5.39 ms | **185 fps** | 0.93 ms | 0.20 ms | 72 | 23,094 |
| Instanced, no culling | 1.58 ms | 6.36 ms | 157 fps | 1.16 ms | 0.25 ms | 72 | 37,580 |
| One draw per object, culled | 16.66 ms | 14.04 ms | 60 fps | 14.13 ms | 13.21 ms | 26,430 | 23,094 |
| One draw per object, no culling | 22.99 ms | 20.69 ms | 44 fps | 21.96 ms | 20.69 ms | 40,916 | 37,580 |

Batching the same 37,580 objects collapses 40,916 draw calls into 72 and cuts
submission from 20.69 ms to 0.25 ms — **83x** — taking the CPU frame from 22.99
to 1.58 ms and the ceiling from 44 to 157 fps.

Once batched the frame is GPU-bound, and culling removes another 39% of the
objects for 1.0 ms of GPU time on top.

## Assets, serialization, scripting, memory

| | |
|---|---|
| OBJ parse | **663 MB/s** — 16.5 MB, 230,400 triangles in 24.8 ms (**9.3 M tri/s**) |
| Vertex welding | 460,800 corners → 115,921 vertices (4.0x reuse) |
| Serialize 112,025 entities | **1.89 ms** → 21.4 MB (11.1 GB/s) |
| Deserialize | 4.05 ms (5.2 GB/s), entity ids preserved exactly |
| Lua per-entity callbacks | 0.391 ms for 5,000 scripts — **78.2 ns/callback** |
| Job dispatch, 2 / 8 / 64 chunks | 0.9 µs / 0.4 µs / **0.7 µs** |
| Memory: ECS / physics / culling / Lua / assets | 28.1 MB / 55.7 MB / 917 KB / 384 KB / 50 KB |

The dispatch row is the one that matters for the solver: a 64-chunk pass costs
the same as a 2-chunk one, because a parallel pass queues one task per *worker*
and hands chunks out through an atomic cursor. The coloured solver pays that
floor once per colour per iteration — 336 times a step.

## Comparing two builds

```sh
./build/skein_bench --save before.txt
# change something, rebuild
./build/skein_bench --compare before.txt
```

Every timed row grows a `+x% wall +y% work` column. Read the work column.
Compared against itself on a loaded machine the benchmark reports about 3% on the
physics step — the noise floor a real change has to clear.
