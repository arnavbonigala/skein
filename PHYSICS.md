# Physics

The rigid-body world, from grid to solver. Numbers in
[BENCHMARKS.md](BENCHMARKS.md); the surrounding engine in
[ARCHITECTURE.md](ARCHITECTURE.md).

## A step

```
  gather        pools ──▶ SoA arrays, one 64-byte SolverBody per body
     ▼
  broadphase    hashed uniform grid, counting sort, cell-run scan
     ▼
  narrowphase   sphere/sphere, sphere/box, 15-axis SAT + clipping
     ▼
  colour        greedy graph colouring, contacts moved into colour order
     ▼
  ┌─ substep ×4 ──────────────────────────────────────────┐
  │   integrate v                                         │
  │   prepare      arms turned, live depth, normal mass    │
  │   solve ×2     colour 0..N, each colour in parallel    │
  │   integrate x                                          │
  └────────────────────────────────────────────────────────┘
     ▼
  restitution   once, after the substeps
     ▼
  sleep, scatter
```

## Broadphase

A hashed uniform grid, built with a counting sort so the per-step cost is linear.

| Decision | Consequence |
|---|---|
| Cell size follows the **typical** body, not the largest | one oversized collider cannot coarsen the whole grid |
| A body registers in **every** cell it overlaps | no missed pairs at cell boundaries; raycast can stop early |
| Entries counting-sorted into hash buckets | linear, no per-frame allocation |
| Then sorted **within** each bucket by exact packed 64-bit cell key | every bucket becomes contiguous runs of one real cell |
| Entry carries its own position and reach | 87% of pairs are rejected without touching a body array |
| A pair is reported only from the **lowest** cell the two share | dedup without a hash set |

The within-bucket sort is what makes the scan cheap: it walks contiguous runs and
tests pairs directly, rather than rehashing each body's cells and filtering
foreign entries out of the bucket.

```
   1,881,821  share a cell           ─┐  reads only the sorted entry array
     248,239  within reach (13.2%)   ─┤  one distance compare
     106,795  this cell's to report  ─┤  six ints from the body's cell range
     105,436  real contacts          ─┘  full narrowphase
```

Almost everything surviving all three filters is a real contact.

### Speculative contacts

A discrete test misses whatever crosses a collider between two tests. Rather than
splitting the step until nothing moves that far:

```
   body moving right at v·dt
   ┌───┐- - - - - - -▷              widened by the part of its motion
   │   │            :               its own extent does not already cover
   └───┘. . . . . . .┘
                      ┃ wall
   contact found across the gap carries NEGATIVE depth
   solver reads it as "you may close this much and no more"
```

| | |
|---|---|
| Body slower than its own radius per step | not widened at all |
| Cost on the 30,000-body field | **3.1% fewer** pairs tested than discrete, 5.3% of the step |
| 400 shots at 20–220 m/s into a 0.5 m slab | 0 through, without splitting the step |

Restitution is carried *across* the gap rather than measured inside it. The
constraint stops the body at the surface before a bounce could be sized from its
velocity, so the fastest approach seen while the pair was still apart is cached
next to the impulse and spent on the frame they meet — an 80 m/s ball at
restitution 0.8 rebounds above 40 m/s instead of barely leaving the floor.

Step splitting still exists, capped by `maxSubsteps`, but only for motion past a
whole grid cell in one step — where widening one body would smear it across the
grid.

## Narrowphase

| Pair | Test |
|---|---|
| Sphere / sphere | centre distance, one contact |
| Sphere / box | closest point on the box, one contact |
| Box / box, axis-aligned | overlap rectangle corners — cheap path |
| Box / box, turned | 15-axis SAT, least-overlap axis, Sutherland-Hodgman clip |

A box whose rotation is near identity takes the axis-aligned path either way, so
`rotatedBoxes` only costs what the scene actually turns. Pairs the three world
axes already separate never reach the fifteen.

### Manifold points

One contact point cannot stop a flat face from rocking about it, so a box pair is
held by the corners of the clipped contact patch.

**Which four survive matters as much as finding them.** On a patch whose points
sit at the same depth, dropping the shallowest leaves the choice to rounding
noise and the manifold changes every frame. Skein keeps the four that make the
widest quad instead.

Each point is named by the feature that produced it — an incident corner, or the
edge cut by a specific side plane of the reference face — which is how its
accumulated impulse finds it again next frame however far the pair has crept.

## Contact graph colouring

Contacts sharing a body cannot be solved at the same time. Greedy colouring with
one `uint64_t` bitmask per body assigns each contact the lowest colour neither of
its bodies has used.

```
   colour 0  ████████████████████   no two contacts share a body
   colour 1  ██████████████         solved after 0, in parallel within
   colour 2  █████████              ...
   ...
   42 colours, 2,510 contacts each, 0 left over
```

Each colour runs in parallel; the barrier between colours is why the dispatch
floor matters. A test asserts the parallel result is **bitwise identical** to the
serial one over 90 steps of a 4,000-body pile.

### The contacts move, the index array does not exist

The counting sort that assigns colours physically permutes the contact array
rather than producing a permutation to read through.

| | Per contact, per sweep |
|---|---|
| Through an index array | 5–6 scattered loads |
| In colour order | one prefetchable stride |

Paid once, read eight times a step. **Worth 9% of the whole step.**

### The bodies move to the solver

The other half cannot be sorted away: a contact reaches two bodies at indices no
ordering makes contiguous. So the body data was packed instead.

```
   SolverBody — 64 bytes, exactly one cache line (asserted at compile time)

   ┌────────────┬─────────┬────────────┬────────────┬──────────────────┬─────┐
   │ velocity   │ invMass │ angular    │ invInertia │ world inv-inertia│ pad │
   │ 3 × f32    │ f32     │ 3 × f32    │ f32        │ 6 × f32          │     │
   └────────────┴─────────┴────────────┴────────────┴──────────────────┴─────┘
```

Five arrays touched at the same scattered index would be five cache lines per
body, ten per contact. As one record it is two.

## The solve

### Substeps and iterations are not interchangeable

A substep integrates velocity, sweeps the contacts, then integrates position.
An iteration only sweeps. They cost the same per sweep and buy different things:

| | Buys |
|---|---|
| Substeps | fresh geometry — every sweep answers where the pair is *now* |
| Iterations | news travelling down the column, one contact per sweep |

At four sweeps of a turned-box stack: 4 × 1 leaves the top box 53% of the way up,
2 × 2 leaves it at 9%, 1 × 4 at 1%. Nothing below six sweeps stands. **4 × 2 is
the cheapest mix that stands**, which is why it is the default.

### Live contacts

Contacts are found once per step, then re-derived every substep. Each contact
stores the arms from each body's centre at the moment it was found — not a
world-space point:

```
   found:      A ●───rA───◆───rB───● B
   two substeps later, both bodies have turned:
               A ●──rA'──◆'
                          ╲rB'
                           ● B      depth re-derived from rA', rB'
```

Turning the arms by `spinDelta` costs two quaternion rotations and follows the
surface, instead of leaving the contact where the narrowphase put it.

### Per contact

| Term | Treatment |
|---|---|
| Normal | accumulated impulse, clamped against zero, cached across frames by entity pair |
| Friction | Coulomb, accumulated as a vector in the contact plane, clamped against the normal impulse |
| Friction coefficient | per collider; a pair rubs at the geometric mean of the two |
| Tangent mass | taken along the direction the pair is actually sliding — the two in-plane axes do not resist a spin equally |
| Rolling resistance | bounded fraction of the normal impulse against relative spin, **curved contacts only** |
| Depth | split impulse — a separate pseudo-velocity integrated into position and dropped |
| Restitution | a bias captured once, after the substeps |
| Effective mass | includes how much the pair resists being rotated at that point |
| Inertia | real tensor — body-space diagonal rotated into the world once a step |

### Three deliberate refusals

| Refused | Why |
|---|---|
| Positional pass rotating | a box on one deep corner separates fastest by turning, but the push has no momentum behind it, and a column fed a few degrees per frame leans further every frame until it goes over |
| Rolling resistance at flat contacts | a box face is already held by four points friction acts at; spending it there torques the stack instead of settling it |
| Restitution inside the substeps | the substep answering a bounce sees the body leaving and winds the impulse back down to stop it |

Neither of the first two is sufficient alone. A column of eight boxes stands only
with both removed.

## Warm starting

Each contact's accumulated normal impulse is keyed by entity pair and manifold
point id, cached in an open-addressed table, and replayed before the first sweep.

Cancelling approach velocity from scratch cannot hold a stack up: the reaction
travels down the column one contact per iteration, so at two iterations a column
sinks through itself and the depth push papers over the result.

| Budget | Cold | Warm |
|---|---|---|
| 2 substeps × 2 iterations | 0 of 32 columns intact | **32 of 32** |

Where it matters most is the longest chain — 400 twelve-link ropes hang with
**0.2 mm** of stretch warm started against 116.7 mm from nothing.

## Joints

A `Joint` holds two bodies a fixed distance apart between an anchor on each,
solved by the same accumulated-impulse machinery as contacts.

| | |
|---|---|
| Compliance | XPBD-style, folded in as mass the constraint does not have — metres of stretch per newton-second, and exactly rigid at zero |
| Length error | routed through the split-impulse channel |
| Carried impulse | stored **on the component**, so it survives pool reordering and saves with the scene |
| Sleeping | one end waking wakes the other; a chain half asleep is solved against a body that never integrates the result |

**Why the length error is not a bias inside the impulse.** The impulse is carried
between frames, so a bias term inside it replays last frame's correction on top
of this frame's. A hanging chain fed that climbs above its own anchor within five
seconds.

**Why the impulse lives on the component.** A sparse set does not keep positions
stable — removing one joint swaps the last entry into the hole. Indexed by
position, cutting one rope hands every other rope's load to the wrong joint.

## Sleeping

A body slower than `sleepSpeed` for `sleepTime` stops integrating, stops solving,
and stops being scanned. A grid cell where every entry is asleep is skipped
whole.

| Wakes on | |
|---|---|
| Contact from something above the sleep threshold | immediate |
| Anything pushed into it deeper than 5 cm | immediate |
| Both-asleep pair found overlapping | every 16th frame |

That last row exists because two sleepers cannot push each other apart: an
overlap appearing exactly as the second one falls asleep would otherwise be
permanent. Re-measuring both-asleep pairs periodically is what allows the
positional correction to be as strong as it is.

`sleepSpeed` must sit above `gravity * dt`, or a body resting on another never
falls below it — gravity re-accelerates it every step and the contact impulse
cancels it right back.

## Queries

Both answer from the grid the last step built.

| | |
|---|---|
| `raycast` | walks the grid cell by cell, stops as soon as the nearest hit lies inside the cell being walked |
| `overlapSphere` | appends every collider overlapping the sphere |

The early-out is sound because a body is registered in *every* cell it overlaps,
so a closer hit would already have been found in a cell already visited.

**0.55 µs per ray** into 30,000 bodies — 145x an all-body test.

## Settings

| Setting | Default | |
|---|---|---|
| `solverSubsteps` | 4 | velocity solves per step, each re-deriving geometry |
| `solverIterations` | 2 | sweeps per substep |
| `maxSubsteps` | 4 | cap on splitting the step for fast bodies; 1 disables |
| `warmStart` | true | off is the naive solver, kept so the benchmark can price it |
| `speculativeContacts` | true | off is the naive discrete test |
| `rotatedBoxes` | true | off falls back to axis-aligned extents |
| `angularContacts` | true | off is the purely linear solver |
| `allowSleep` | true | |
| `sleepSpeed` / `sleepTime` | 0.3 / 0.6 s | |
| `cellSize` | 2.0 | a floor; the grid widens itself to fit the largest collider |
| `restitutionFloor` | 0.35 | |
| `friction` | 0.4 | **world bounds only** — body pairs use their colliders |
| `gravity`, `linearDamping`, `angularDamping` | | |
| `useBounds`, `boundsMin`, `boundsMax` | | bounds solved as a constraint, not clamped afterwards |

Every `false`-able flag above is there because the benchmark measures both sides
of it, and there is a test asserting the feature actually does nothing when it is
off.
