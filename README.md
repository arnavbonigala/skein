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
./build/skein_tests     # 97 tests
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
`parallelFor` calls make progress instead of deadlocking. A parallel pass
queues one task per worker, not one per chunk, and the chunks are handed out
through an atomic cursor: the queue mutex and the `std::function` allocation
are paid once per thread instead of once per chunk, which holds a 64-chunk
dispatch at **0.8 µs** — the same cost as a 2-chunk one. That floor matters
because the coloured solver pays it once per colour per iteration.

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

A discrete contact test misses whatever crosses a collider between two tests.
Rather than splitting the step until nothing moves that far, a moving body is
widened in the broadphase by the part of its motion its own extent does not
already cover, and a contact found across the remaining gap carries a negative
depth that the solver treats as a bound on approach instead of an overlap to
push out — so the body lands on the surface rather than inside it or past it.
Restitution is carried across that gap rather than measured inside it: the
constraint stops the body at the surface before a bounce could be sized from
its velocity, so the fastest approach seen while the pair was apart is cached
next to the impulse and spent on the frame they meet — an 80 m/s ball at
restitution 0.8 rebounds above 40 m/s instead of barely leaving the floor.
See [Fast bodies against a thin wall](#fast-bodies-against-a-thin-wall). The
step still splits itself, capped by `maxSubsteps` (4 by default, 1 to disable),
but only for motion past a whole grid cell in one step, where widening one body
would smear it across the grid.

The same grid answers ray queries: `PhysicsWorld::raycast` walks it cell by cell
and stops as soon as the nearest hit lies inside the cell it is walking, since a
body is registered in every cell it overlaps and a closer hit would have been
found in a cell already visited. 4,096 rays into the 30,000-body field cost
**0.37 µs each** (2.74 M rays/s) against 74.2 µs for testing every body — 203x.
A test checks 400 random rays against an O(n) reference. `overlapSphere`
answers area queries off the same grid, and both are bound into Lua as
`skein.raycast` and `skein.overlap_sphere`.

Contacts are solved at the point the pair touches, not through the two centres,
so an impulse applied at arm's length spins the body it lands on and the
orientation it produces goes back onto the `Transform`. The effective mass of a
contact therefore includes how much the pair resists being rotated there, and
friction resolves along the direction the surfaces are actually sliding in.
Inertia is a real tensor: the diagonal for the body's own shape, rotated into
the world once a step and applied as a symmetric matrix, so a long slab tumbles
end over end far more readily than it spins about its length. A test asserts
exactly that, at better than five to one.

Sliding friction cannot slow a ball that is already rolling: its contact point
is stationary, so there is nothing left for friction to act on and a pile of
spheres jostles forever without ever going still. Contacts and walls therefore
spend a bounded fraction of their normal impulse against the pair's relative
spin, which is what lets a rolling body come to rest and the pile sleep at all.
A test drives a ball across the floor and checks it stops skidding, starts
rolling, and eventually stops — without the motion ever running backwards.
Friction is per collider and a pair rubs at the geometric mean of the two, so
ice on rubber lands between ice on ice and rubber on rubber instead of at
whichever coefficient was picked for the whole world.

A turned box is met as the box it is. Box pairs separate along the least-overlap
axis of a fifteen-axis separating-axis test, and the face that separates them is
clipped against the other box's nearest face, so the pair is held by the corners
of the contact patch rather than by a single point — one point cannot stop a
flat face from rocking about it. Which four points survive matters as much as
finding them: dropping the shallowest leaves the choice to rounding noise on a
patch whose points are all at the same depth, and the manifold then changes
every frame, so the four that make the widest quad are kept instead. Each point
is named by the feature that produced it — an incident corner, or the edge cut
by one of the reference face's side planes — which is how its accumulated
impulse finds it again next frame however far the pair has crept. A box whose
rotation is identity keeps the cheap axis-aligned path, which reports the
corners of the rectangle the two faces share rather than a point in the middle
of it, and pairs the three world axes already separate never reach the fifteen.

The positional pass deliberately does not rotate. A box resting on one deep
corner is separated fastest by turning it, and letting the push do that works
for one box — but the push is a correction with no momentum behind it, and a
column fed a few degrees of it per frame leans further every frame until it
goes over. Rolling resistance is refused at flat contacts for the same reason:
a ball needs it because its contact point is stationary and sliding friction
never reaches it, while a box face is already held by the four points friction
acts at, so spending it there only torques the stack. Neither is enough alone.
A column of eight boxes stands only with both removed.

**Joints** — a `Joint` component holds two bodies a fixed distance apart between
an anchor on each, solved by the same accumulated-impulse machinery as contacts
and warm started the same way. Length error is not folded into that impulse as a
bias term: the impulse is carried between frames, so a bias inside it replays
last frame's correction on top of this frame's and a hanging chain climbs above
its own anchor within five seconds. The error goes through the split-impulse
channel the contacts already use instead, and the chain then holds its rest
length exactly. Compliance is exposed as metres of stretch per newton-second,
folded in XPBD-style as mass the constraint does not have, so a rope that should
sag is a number rather than a retune. Four hundred twelve-link ropes are where
warm starting earns its keep hardest: the top joint carries eleven links of load
and only hears about it one joint per sweep, so solving from nothing each step
leaves the free end **116.7 mm** below rest against **0.2 mm** warm started, for
6% less time.

**Substeps** — the step is solved in four substeps of two iterations rather than
in one pass of eight. A substep integrates velocity, sweeps the contacts, and
integrates position, so every sweep after the first answers where the pair is
now rather than where the narrowphase left it; each contact re-derives its depth
from the anchors it was born with, turned by however far each body has turned
since. Substeps and iterations cost exactly the same per sweep, so the benchmark runs
the whole matrix of them against sixteen columns of eight turned boxes and
sorts it by what each mix costs. Rows sharing a sweep count are the same solver
work spent two ways and do not land in the same place: at four sweeps, four
substeps of one iteration leaves the top box 53% of the way up, two of two
leaves it at 9%, and one of four at 1%. Nothing below six sweeps stands at all;
six very nearly does either way round; four substeps of two iterations is the
cheapest mix that actually stands, which is why it is the default. Substeps buy
fresh geometry, iterations buy news travelling down the column, and a stack
needs both. Restitution is applied once, after the substeps: mixed into them it
fights its own output, since the substep that answers a bounce sees the body
leaving and winds the impulse back down to stop it.

**Assets** — a hand-written OBJ parser (all four face index forms, negative
indices, polygon fan triangulation, vertex welding, generated normals) plus
generated primitives. Batch loads parse concurrently and register in a
deterministic order. The text is walked in place rather than through
`getline`/`sscanf`, and corners are welded through an open-addressed table
instead of an `unordered_map`, which is worth **700 MB/s** against the 130 MB/s
the stream version managed: a 16.5 MB, 230,400 triangle mesh parses in 23.6 ms
(9.8 M triangles/s, 460,800 corners welded down to 115,921 vertices).

**Serialization** — a versioned binary format that writes each registered pool
verbatim. Entity ids, including generations and holes, survive a round trip;
unknown or size-mismatched pools are skipped rather than treated as fatal.

**Scripting** — Lua 5.5 embedded through the C API. Scripts spawn, destroy,
move, reparent, light and collide entities through the same component pools the
C++ systems read, so a scripted entity is culled, lit and simulated exactly
like a native one, and `skein.raycast` queries the same physics grid the
solver uses. Per-entity callbacks are held in the registry; a callback
that errors is unbound and reported instead of killing the frame.

## Measurements

Apple M3, 8 hardware threads, macOS 26.6, Apple clang `-O2`, OpenGL 4.1 (Metal
90.5). CPU figures are medians over 40 runs from `skein_bench`; render figures
are means over 200 frames from `skein_demo --capture 200`. Every timed sample
checks `ru_nivcsw` on either side of itself and is discarded if the scheduler
preempted the thread mid-measurement, so a loaded machine costs coverage rather
than accuracy; the run behind these tables kept 639 of 3,438 samples and the
header prints the drop rate and the load average alongside them.

Both harnesses build the same world from `Demo::build`: **112,025 entities**,
100,000 of them integrating position and rotation every frame, **37,000
renderable**, 30,000 rigid bodies piled densely enough to generate ~60,000
contacts a step, 25 lights, 6 meshes, 6 materials, 31.6 MB resident. The
interactive demo also runs `assets/scripts/demo.lua` on top, which adds 316
scripted entities — 112,341 entities and 37,316 renderables in the render
table below.

### ECS and simulation, single threaded vs the job system

| Pass | 1 thread | 8 threads | Speedup |
|---|---|---|---|
| ECS iteration (100k integrate) | 0.679 ms — 6.8 ns/entity | 0.176 ms — 1.8 ns/entity | 3.86x |
| Transform hierarchy (112k) | 0.756 ms — 6.8 ns/entity | 0.334 ms — 3.0 ns/entity | 2.26x |
| Physics step (30k bodies, 60k contacts) | 17.13 ms — 571 ns/body | 6.23 ms — 208 ns/body | 2.75x |
| Cull + batch (37k candidates) | 0.600 ms — 16.2 ns/object | 0.376 ms — 10.2 ns/object | 1.60x |
| **Full simulation frame** | **19.60 ms (51 fps)** | **7.20 ms (139 fps)** | **2.72x** |

Thread scaling on the full simulation frame:

| Threads | 1 | 2 | 3 | 4 | 6 | 8 |
|---|---|---|---|---|---|---|
| ms | 17.96 | 10.22 | 7.78 | 6.41 | 6.95 | 7.00 |
| speedup | 1.09x | 1.92x | 2.52x | 3.06x | 2.82x | 2.80x |

Scaling flattens past four threads because the M3's four efficiency cores are
roughly a third the throughput of its performance cores, and because the
coloured solver has to finish each colour before starting the next.

Where the frame actually goes, from the built-in profiler at 8 threads:

| Zone | avg | p95 |
|---|---|---|
| physics/solve | 2.79 ms | 3.07 ms |
| physics/narrowphase | 1.77 ms | 2.01 ms |
| physics/broadphase | 1.04 ms | 1.13 ms |
| scene/updateTransforms | 0.69 ms | 0.83 ms |
| physics/impulseCache | 0.36 ms | 0.49 ms |
| render/cull | 0.30 ms | 0.39 ms |
| physics/gather | 0.27 ms | 0.32 ms |
| ecs/kinematics | 0.26 ms | 0.37 ms |
| physics/impulseStore | 0.26 ms | 0.32 ms |
| physics/color | 0.23 ms | 0.24 ms |
| physics/scatter | 0.10 ms | 0.12 ms |
| physics/integrate | 0.04 ms | 0.08 ms |
| physics/sleep | 0.04 ms | 0.08 ms |
| render/clusterBounds | 0.04 ms | 0.06 ms |
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
| Cull + batch | 0.330 ms — 8.9 ns/object | **0.239 ms — 6.4 ns/object (1.39x)** |
| Objects individually tested | 37,000 | **4,736 (87.2% resolved by their cluster)** |
| Visible | 25,224 | 25,224 |

Cluster verdicts for that frame: 290 clusters — 55 outside, 154 inside, 81
straddling; the straddling ones break into 2,313 sub-clusters of which 171 are
outside and 181 fully inside.

Objects drift out of Morton order as they move, so `CullSystem::maintain()`
rebuilds the cluster bounds, compares the mean cluster size against what it was
right after the last sort, and re-sorts only once that ratio crosses a
threshold. Over 800 frames of continuous motion:

| | Median cull | Order decay | Re-sorts | Objects tested |
|---|---|---|---|---|
| Sorted once | 0.310 ms | 1.71x | 1 | 10,864 |
| Maintained every 30 frames | 0.291 ms | **1.01x** | 2 | **4,960** |

The win depends on how much of the scene is on screen — clustering has nothing
to skip when everything is visible-adjacent, and nothing to gain when a cheap
sphere test already rejects everything:

| Camera | Visible | Flat | Clustered |
|---|---|---|---|
| Tight, looking away | 0.0% | 0.039 ms | 0.041 ms (0.94x) |
| Narrow fov into the field | 51.5% | 0.226 ms | 0.170 ms (1.32x) |
| Default view | 68.2% | 0.286 ms | 0.204 ms (1.40x) |
| Inside the field, looking down | 92.9% | 0.328 ms | 0.263 ms (1.25x) |
| Far back, whole field in view | 100.0% | 0.370 ms | 0.289 ms (1.28x) |

When nothing is on screen the sphere prepass already rejects everything and the
cluster tests are pure overhead, which is the one case clustering loses.

### What the layout is actually worth

The same integrate over 200,000 objects, three ways: one polymorphic object per
entity reached through a shuffled pointer array (the classic scene graph), the
same objects stored contiguously and called directly, and loose component
arrays the way the ECS stores them.

| | Time | Per object | |
|---|---|---|---|
| Virtual call, pointer per object | 4.820 ms | 24.1 ns | 184 B/object |
| Contiguous objects, direct call | 1.359 ms | 6.8 ns | **3.55x** |
| Component arrays | 1.357 ms | 6.8 ns | **3.55x** |
| Position only, contiguous objects | 0.562 ms | 2.8 ns | |
| Position only, component arrays | **0.075 ms** | **0.4 ns** | **7.51x** |

Most of the first 3.6x is dispatch and pointer chasing, not layout — with the
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
| 25,000 | 3.4 ms | 0.106 ms | 0.118 ms | 0.137 ms | 0.084 ms (1.62x) |
| 100,000 | 11.7 ms | 0.180 ms | 0.287 ms | 0.368 ms | 0.282 ms (1.30x) |
| 250,000 | 32.8 ms | 0.573 ms | 0.947 ms | 0.981 ms | 0.535 ms (1.83x) |
| 500,000 | 61.8 ms | 1.227 ms | 2.430 ms | 2.205 ms | 1.305 ms (1.69x) |
| 1,000,000 | 135.7 ms | 2.392 ms | 5.004 ms | 4.244 ms | 2.547 ms (1.67x) |

At a million entities the ECS integrate pass is 2.4 ns per entity and 433,333
renderables cull and batch in 2.5 ms, with the clustered and flat paths
agreeing on the visible set at every size.

### What warm starting buys a stack

Thirty-two columns of eight spheres, dropped and left for ten seconds, then
measured by where the top sphere ended up. A perfect stack puts it at 7.50.

| Solver iterations | Cold | Warm started |
|---|---|---|
| 2 | 3.76 — 47% of the stack standing | 7.34 — **98%** |
| 4 | 6.28 — 83% | 7.41 — **99%** |
| 8 | 7.34 — 98% | 7.43 — **99%** |
| 16 | 7.43 — 99% | 7.45 — **99%** |

A cold solver needs eight iterations to reach what warm starting reaches in
two, because the reaction holding a column up travels one contact per
iteration. Reusing each contact's accumulated impulse removes that dependency
on stack height, and it is what makes sleeping possible at all: a pile that
never converges never goes still enough to sleep.

### Sleeping

A body that has been slow for 0.6 s stops integrating, stops solving and stops
being scanned; a cell where every entry is asleep is skipped whole. It wakes
when something moving faster than the sleep threshold touches it, or when
anything is pushed into it deeper than 5 cm. Two sleepers cannot push each
other apart, so an overlap that appears exactly as the second one falls asleep
would otherwise be permanent; every sixteenth frame the both-asleep pairs are
measured again and woken if they are deep, which is what allows the positional
correction to be as strong as it is. Timed single threaded, because the saving
is work, not parallelism:

| Scene | Sleeping off | Sleeping on | |
|---|---|---|---|
| 4,000 in a box, 30 s to settle | 1.82 ms — 4,000 awake, 11,725 contacts | **0.24 ms** — 18 awake, 76 contacts | **7.48x** |
| 30,000 demo field, still churning | 18.35 ms — 30,000 awake | 17.96 ms — 22,626 awake | 1.02x |

The second row is the point of the first: a pile that never settles pays only
the bookkeeping for a feature it cannot use.

### What the solver leaves behind

Speed is half of a solver; the other half is how much of the pile is still
inside itself when the motion stops. 2,000 spheres dropped into a 24 m box and
left for 20 s, then every one of the 2M pairs checked directly:

| Solver | Step | Worst overlap | Mean overlap | Fastest body |
|---|---|---|---|---|
| 1 iteration, warm | 0.57 ms | 25.0% of a radius | 3.6% | 0.196 m/s |
| 2 iterations, warm | 0.71 ms | 16.9% | 2.3% | 0.185 m/s |
| 4 iterations, warm | 0.93 ms | 15.2% | 1.7% | **0.144 m/s** |
| 8 iterations, warm | 1.71 ms | **10.9%** | **1.4%** | 0.162 m/s |
| 2 iterations, cold | 0.65 ms | **170.0%** | 5.1% | 0.567 m/s |

The last row is the one that matters: without warm starting, two iterations
leave a pair fully inside each other and the pile still crawling at half a
metre a second. Iterations past two buy the worst case back slowly, because it
is a deep column the positional pass only unwinds one contact per iteration.

### Fast bodies against a thin wall

400 spheres of radius 0.15 fired at 20 to 220 m/s at a 0.5 m thick static slab,
which at the top speed is 3.7 m of travel in a 1/60 s step:

| Contact test | Substep cap | Step cost | Passed through |
|---|---|---|---|
| discrete | 1 | 0.028 ms | 292 of 400 |
| discrete | 4 | 0.158 ms | 85 of 400 |
| discrete | 16 | 0.078 ms | **0 of 400** |
| speculative | 1 | 0.072 ms | **0 of 400** |
| speculative | 4 (the default) | 0.080 ms | **0 of 400** |

A discrete test has to split the step sixteen ways to catch every shot.
Speculative contacts catch all of them without splitting it at all: a moving
body is registered in the broadphase as wide as the part of its motion its own
extent does not already cover, so a pair that will meet during the step is
already sharing a cell when the scan runs, and the contact found across the gap
carries a *negative* depth the solver reads as a bound on how far the pair may
approach rather than an overlap to push apart. The shot lands on the surface
instead of inside it or past it.

The widening is what keeps this free: a body slower than its own radius per step
is not widened at all, so the 30,000-body benchmark field tests 1% *fewer* pairs
with speculative contacts on than with them off. Splitting is still there for
motion past a whole grid cell in one step, where the inflation would smear one
body across the grid.

The tightest discrete cap is not the slowest, either: a shot that stops at the
wall stops needing splits, while every shot that escapes keeps moving fast
enough to split every later step.

### Culling and broadphase effectiveness

37,000 candidates against a 65° frustum: **25,230 kept, 11,770 rejected
(31.8%)** in 0.35 ms. The broadphase is the same story at a different scale —
30,000 densely piled bodies produce 1.99M candidate pairs instead of the
4.5×10⁸ an all-pairs test would need, **226x fewer**. What happens to those
1.99M is the whole design:

| Stage | Pairs | Cost of the test |
|---|---|---|
| Tested in a shared cell | 1,993,119 | reads only the sorted entry array |
| Within each other's reach | 257,427 (12.9%) | one distance compare |
| Not another cell's to report | 106,481 | six ints from the body's cell range |
| Real contacts | 61,892 | full sphere/box narrowphase |

87% of the work is rejected before touching a body array at all, which is why
the entry carries its own position and reach instead of an index to look one
up.

### Rendering, before and after batching

Same scene, same camera, same frame count, only the render path changed. The
world is settled for 240 frames and then held still, so the four rows differ in
exactly one thing. GPU time is a `GL_TIME_ELAPSED` query read a frame late;
figures are medians over 200 frames with vsync off.

| Path | CPU frame | GPU | Ceiling | Render CPU | Submit | Draw calls | Objects |
|---|---|---|---|---|---|---|---|
| Instanced + culled | **1.30 ms** | 5.29 ms | **189 fps** | 0.95 ms | 0.20 ms | 72 | 22,929 |
| Instanced, no culling | 1.55 ms | 6.34 ms | 158 fps | 1.06 ms | 0.22 ms | 72 | 37,316 |
| One draw per object, culled | 16.81 ms | 13.32 ms | 60 fps | 13.63 ms | 12.75 ms | 26,101 | 22,929 |
| One draw per object, no culling | 22.38 ms | 20.22 ms | 45 fps | 21.59 ms | 20.50 ms | 40,488 | 37,316 |

Batching the same 37,316 objects collapses 40,488 draw calls into 72 and cuts
submission from 20.50 ms to 0.22 ms — **93x** — taking the CPU frame from 22.38
to 1.55 ms and the ceiling from 45 to 158 fps. Once batched the frame is
GPU-bound, and culling removes another 39% of the objects for 1.1 ms of GPU
time on top.

### Serialization, scripting, memory

| | |
|---|---|
| Serialize 112,025 entities | 1.72 ms → 21.3 MB (12.1 GB/s) |
| Deserialize | 4.24 ms (4.9 GB/s), entity ids preserved exactly |
| Lua per-entity callbacks | 0.413 ms for 5,000 scripts — 82.7 ns/callback |
| ECS pools / physics / culling / Lua heap | 28.0 MB / 28.6 MB / 917 KB / 385 KB |

## Demo controls

`W A S D`, `Space`/`Ctrl` fly, `Shift` boosts, `Tab` releases the mouse.
`F` toggles frustum culling, `M` cluster culling (the title bar shows how many
objects still needed their own test), `I` instanced batching, `L` shadows, `B` draws
cull bounds, `C` freezes the culling camera so you can fly outside it and watch
objects drop out, `O` pauses, `P` dumps the frame profile, `V` toggles vsync,
`F5`/`F9` save and load `scene.skn`.

## Tests

97 tests, no framework. They cover the parts where being wrong is quiet: the
hashed grid must return exactly the brute-force contact set even when collider
sizes vary 70x, the coloured parallel solver must land bitwise on the serial
result, a stack of eight spheres must still be standing after ten seconds, a
pile that has gone to sleep must be no more interpenetrated than one that has
not and must wake when a script throws it somewhere else, a body crossing
twelve times its own radius in one step must not end up on the far side of a
wall, an off-centre hit must spin the body it lands on while a hit through the
centres must not, a box set down on a face must rest on four contacts rather
than balance on one and must slide upright when pushed, sixteen columns of
eight turned boxes must all still be standing after ten seconds, a slab must tumble end
over end at least five times more readily than it spins about its own length,
a skidding ball must end up rolling and then stop, clustered
culling must keep exactly the objects the flat path keeps and
must re-sort only once motion has actually loosened the order, a slab must
slide furthest on ice, least on rubber and in between on one of each, a chain of joints must hang at its rest length rather than
stretch or climb and must still do so after being saved and loaded, a
2,000-entity hierarchy with destroyed parents must survive a serialization
round trip, threaded transform updates must match single-threaded ones bit for
bit, and `parallelFor` must cover every index exactly once under nesting.
