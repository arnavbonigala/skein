# Architecture

How the pieces fit, and why they are shaped that way. Numbers live in
[BENCHMARKS.md](BENCHMARKS.md); the solver has its own document in
[PHYSICS.md](PHYSICS.md).

## Layout

| Directory | Contents |
|---|---|
| `src/core` | `Vec3`/`Quat`/`Mat4`, job system, profiler |
| `src/ecs` | Sparse-set `World`, `Pool<T>`, type-erased pool registry |
| `src/scene` | Components, `Scene`, transform hierarchy, serialization, particles |
| `src/render` | Frustum + cluster culling, render list, GL renderer, post chain, particles, overlay, recorder |
| `src/physics` | Broadphase, narrowphase, solver, queries |
| `src/assets` | Mesh storage, primitives, OBJ parser, distance field fonts |
| `src/script` | Lua host and bindings |
| `src/app` | Demo scene construction, window, main loop |
| `bench`, `tests` | The two harnesses |
| `third_party` | stb_truetype, the one vendored file |

## The ECS

Each component type owns two arrays and a map:

```
  Pool<Transform>

  sparse[]   entity index ──▶ dense slot        (or NONE)
  dense[]    slot ──▶ Entity                    packed, no holes
  data[]     slot ──▶ Transform                 packed, no holes
                      └── iteration walks this straight through
```

| Operation | Cost | Note |
|---|---|---|
| `get` / `tryGet` | O(1) | one indirection through `sparse` |
| `add` | O(1) amortised | push onto `dense` + `data` |
| `remove` | O(1) | swap-and-pop, patch the moved entity's `sparse` |
| Iterate | O(n) linear | contiguous, prefetchable |
| Permute | O(n) | rewrite `sparse`; every handle stays valid |

An `Entity` is 64 bits: index in the low word, generation in the high word. A
handle to a destroyed entity fails `alive()` rather than resolving to whatever
was recycled into the slot.

**Permutation is a feature, not an accident.** Because `sparse` absorbs the move,
a pool can be reordered freely at runtime. Cluster culling spends that on Morton
order; the physics solver spends it on colour order.

Pools register by stable string name behind a type-erased interface, which is
what lets serialization walk them without knowing their types.

## Frame order

```
   ┌─────────────────────────────────────────────────────┐
   │ scripts        Lua per-entity callbacks             │
   ├─────────────────────────────────────────────────────┤
   │ kinematics     integrate Velocity into Transform    │
   ├─────────────────────────────────────────────────────┤
   │ physics        gather → broadphase → narrowphase →  │
   │                colour → 4 × (solve) → scatter       │
   ├─────────────────────────────────────────────────────┤
   │ hierarchy      Transform → WorldTransform, by depth │
   ├─────────────────────────────────────────────────────┤
   │ cull           frustum + clusters → visible list    │
   ├─────────────────────────────────────────────────────┤
   │ batch          counting sort by (mesh, material)    │
   ├─────────────────────────────────────────────────────┤
   │ render         one instance upload, shadow + main   │
   └─────────────────────────────────────────────────────┘
```

Physics runs before the hierarchy so bodies that are also children land in the
right place the same frame. Culling runs after the hierarchy because it reads
world bounds.

## Transform hierarchy

No recursion, no pointer chasing. Depths are computed once with memoized upward
walks, then counting-sorted into a flat order:

```
  depth 0  │████████████████████│  parents already final (none)
  depth 1  │████████████│           parents all in depth 0
  depth 2  │█████│                  parents all in depth 0..1
           └── each level is one contiguous span
```

Every level is trivially parallel because its parents are complete before it
starts. Levels of 512 entities or more are split across the job system; smaller
ones are not worth the dispatch.

## Job system

```
   worker 0     worker 1     worker 2     ...
   ┌────────┐   ┌────────┐   ┌────────┐
   │ deque  │   │ deque  │   │ deque  │
   └───┬────┘   └───┬────┘   └───┬────┘
    LIFO│  ▲     LIFO│           │
   owner│  └── FIFO ─┴── thief ──┘
```

| Decision | Why |
|---|---|
| LIFO for the owner | the task just pushed is the one still in cache |
| FIFO for thieves | steal the oldest, likeliest to spawn more work |
| Submitter participates | nested `parallelFor` makes progress instead of deadlocking |
| One task per **worker**, not per chunk | the queue mutex and `std::function` allocation are paid once per thread |
| Chunks handed out by an atomic cursor | load balances without more queue traffic |

That last pair is why a 64-chunk dispatch costs the same as a 2-chunk one —
**0.7 µs**. The floor matters because the coloured solver pays it once per colour
per iteration, 336 times a step.

## Culling

```
  Renderable[]  ┐
  CullBounds[]  ├─ Morton-sorted together, so index i means the
  WorldTransform┘  same object in all three and neighbours in the
                   array are neighbours in space

  cluster of 128 ──▶ frustumClassifyAABB ──▶ Outside  ── skip 128
                                             Inside   ── keep 128
                                             Straddle ── 8 sub-clusters of 16
                                                          └─▶ same three-way test
                                                              └─▶ per-object
```

The per-object test — sphere prepass, then center/extent AABB — is the fallback,
reached by 13% of objects on the benchmark scene.

Because the three pools share an order, the cull loop and the instance fill read
by index rather than through the sparse map.

**Re-sorting is measured, not scheduled.** Objects drift out of Morton order as
they move, so `maintain()` rebuilds the cluster bounds, compares mean cluster
half-diagonal against what it was right after the last sort, and re-sorts only
once that ratio crosses a threshold. Two re-sorts over 800 frames of continuous
motion hold the order at 1.02x of fresh.

## Renderer

OpenGL 4.1 core against the macOS system framework — no loader library.

```
  visible list ──▶ counting sort by (mesh, material) ──▶ runs of identical state
                                                          │
       ┌──────────────────────────────────────────────────┘
       ▼
  one orphaned instance buffer per frame
  [ main pass matrices ][ shadow pass matrices ]   ← single upload
       │                      │
       ▼                      ▼
  draw batches           draw batches
```

| Constraint | Response |
|---|---|
| `glDrawElementsInstancedBaseInstance` is GL 4.2 | batches re-point the four mat4 instance attributes at a byte offset before drawing |
| Buffer stalls on rewrite | orphan the whole buffer each frame rather than sub-updating |
| Two passes, one buffer | shadow matrices appended after main, so there is a single upload |

Lighting is Blinn-Phong with a metallic tint, a 2048² PCF shadow map for the sun,
and up to 32 point lights chosen per frame by distance.

Each instance's tint rides in the bottom row of its model matrix. An affine
matrix always has (0, 0, 0, 1) there, so the vertex shader reads the colour out
and puts the row back: per-instance colour costs no extra bandwidth, no extra
attribute and no change to batching. The shadow map follows the point the camera
looks at and snaps to its own texels, so shadows do not shimmer as the camera
moves.

Stylised looks are options on the same shader. `toon` blends diffuse into two
bands and the shadow edge into a hard step, `rim` adds a sunlit edge, and
`skyGradient` draws a zenith-to-horizon sky after opaque geometry, at the far
plane, so it only shades pixels nothing covered. Fog fades distant geometry to
that sky in the view direction, so hills melt into the horizon. A material's
`wind` sways vertices by the square of their height above the mesh origin. A material's `ink` goes out
in the scene alpha, and the outline pass takes the smallest value under its
kernel, so grass and flowers can stay unlined while the ground around them is
still inked.

### Post-processing

```
  scene ──▶ RGBA16F, 4x MSAA ──▶ resolve colour + depth
                                     │
          ┌──────────────┬───────────┼──────────────┐
          ▼              ▼           ▼              ▼
     SSAO, half res   bloom: prefilter, 6-level   depth for
     16 samples,      13-tap down, tent up        outlines
     depth-aware blur        │                      │
          └──────────────────┴─────────┬────────────┘
                                       ▼
            composite: AO, exposure, bloom, outline, ACES, saturation,
            contrast, vignette ──▶ RGBA8 ──▶ FXAA ──▶ final target
```

The final target has its own resolution, independent of the window, and is what
`VideoRecorder` reads. `present()` scales it into the window. Outlines are the
Laplacian of reciprocal depth, which is exactly zero across any flat surface, so
silhouettes and creases are inked and a floor at a grazing angle is not.

| GPU time, showcase scene | 1080p | 4K |
|---|---|---|
| whole frame, every pass on | 3.8 ms | 15.1 ms |
| SSAO | | 2.3 ms |
| bloom | | 2.3 ms |
| 4x MSAA over 1x | | 3.8 ms |
| FXAA | | 1.1 ms |

The recorder reads through a ring of three pixel buffer objects, so frame N is
copied off the GPU while frame N + 1 renders and nothing waits on a readback.

### Particles and overlay

Particles draw inside the scene pass, so they are lit, fogged, outlined and
bloomed like everything else. Their buffer is filled before the shadow pass
starts. Apple's GL runs on Metal, and a buffer write in the middle of a pass
ends the pass: the 4x MSAA colour and depth targets are stored and reloaded,
which cost a fixed 4 ms at 1080p even for one particle. Uploading first brought
30,000 particles down to noise.

The overlay draws after post-processing into the final target, in pixels from
the top-left. Text comes from a 48 px signed distance field, so a 200 px title
stays sharp and an outline is one more threshold in the same shader.

## Serialization

A versioned binary format writing each registered pool verbatim.

| Property | Behaviour |
|---|---|
| Entity ids | preserved exactly, including generations and holes |
| Unknown pool name | skipped, not fatal |
| Size-mismatched pool | skipped, not fatal |
| Component contents | raw bytes — a new field serializes for free |

That last row is why the solver's carried joint impulse rides on the `Joint`
component: a rope reloads still holding its own weight, with no code in the
serializer that knows about ropes.

## Scripting

Lua 5.5 through the C API. Scripts drive the same pools the C++ systems read, so
a scripted entity is culled, lit and simulated exactly like a native one. See
[SCRIPTING.md](SCRIPTING.md) for the API.

Per-entity callbacks are held in the Lua registry by reference. A callback that
raises is unbound and reported rather than killing the frame.

## Testing strategy

138 tests, no framework — a `TEST` macro, a `CHECK`, and a main that runs them.

The rule for what earns a test: it must be something that can be **quietly**
wrong. Not "does `add` return the component" but:

| Test shape | Example |
|---|---|
| Differential | the hashed grid returns exactly the brute-force contact set |
| Bitwise | the coloured parallel solve equals the serial one, bit for bit |
| Behavioural | sixteen columns of turned boxes are still standing after ten seconds |
| Negative control | a bullet **does** go through the wall when the feature meant to stop it is off |
| Round trip | a hierarchy with destroyed parents survives save and load |
| Soak | everything at once for twenty seconds with no NaN, no escape, no drift off the unit quaternion |

The negative controls are the ones that catch feature flags that silently do
nothing.
