# Scripting

Lua 5.5 embedded through the C API. Everything below drives the same component
pools the C++ systems read — a scripted entity is culled, lit, simulated and
serialized exactly like one spawned from C++. There is no scripting-side mirror
of the scene to keep in sync.

Engine in [ARCHITECTURE.md](ARCHITECTURE.md), solver in [PHYSICS.md](PHYSICS.md).

## Entities

| Function | Returns | |
|---|---|---|
| `skein.spawn(desc)` | entity | see [Spawn descriptor](#spawn-descriptor) |
| `skein.destroy(e)` | | |
| `skein.alive(e)` | bool | false for a handle to a recycled slot |
| `skein.entity_count()` | int | live entities |

### Spawn descriptor

Every field is optional. An empty `skein.spawn{}` makes a bare transform.

| Field | Type | Effect |
|---|---|---|
| `position` | `{x, y, z}` or `{1, 2, 3}` | |
| `scale` | `{x, y, z}` | |
| `yaw`, `pitch` | number | radians |
| `mesh` | string or int | name or id — **adds `Renderable` + `CullBounds`**, bounds taken from the mesh |
| `material` | int | id from `skein.material` or `skein.add_material` |
| `velocity` | `{x, y, z}` | adds `Velocity` |
| `collider` | table | adds `Collider`, and `Velocity` if absent |

Collider sub-table:

| Field | Default | |
|---|---|---|
| `kind` | `"sphere"` | `"sphere"` or `"box"` |
| `radius` | 0.5 | |
| `half_extents` | `{0.5, 0.5, 0.5}` | boxes |
| `inv_mass` | 1.0 | **0 makes it static** |
| `restitution` | 0.5 | |
| `friction` | 0.4 | a pair rubs at the geometric mean of the two |

An unknown mesh name is an error, and the half-built entity is destroyed before
it raises.

```lua
local ball = skein.spawn{
  position = {0, 20, 0},
  mesh = "geosphere",
  material = skein.material("copper"),
  collider = { radius = 0.6, restitution = 0.7, friction = 0.1 },
}
```

## Transform

| Function | |
|---|---|
| `skein.position(e)` | returns `x, y, z` — three values, not a table |
| `skein.set_position(e, x, y, z)` | |
| `skein.set_rotation(e, pitch, yaw, roll)` | radians |
| `skein.set_scale(e, s)` | uniform |
| `skein.set_parent(e, parent)` | |

Setting a position on a body with a collider teleports it — the solver picks the
new position up next step, but its velocity is untouched.

## Physics

| Function | |
|---|---|
| `skein.velocity(e)` | returns `x, y, z` |
| `skein.set_velocity(e, x, y, z)` | wakes the body |
| `skein.joint(a, b, opts)` | ties two bodies; the joint lives on `a` |
| `skein.unjoint(a)` | cuts it |
| `skein.raycast(ox, oy, oz, dx, dy, dz [, max])` | `entity, distance, nx, ny, nz` — or nothing on a miss |
| `skein.overlap_sphere(x, y, z, r)` | array of entities |

Joint options:

| Field | Default | |
|---|---|---|
| `length` | 1.0 | 0 pins the two together |
| `compliance` | 0.0 | metres of stretch per newton-second; 0 is rigid |
| `anchor_a`, `anchor_b` | `{0,0,0}` | attachment point in each body's own space |

One entity carries one joint. Joining `a` to something else replaces the
existing joint rather than adding a second.

```lua
-- a rope hanging from a static anchor
local previous = skein.spawn{ position = {0, 20, 0}, collider = { inv_mass = 0 } }
for i = 1, 10 do
  local link = skein.spawn{ position = {0, 20 - i, 0}, collider = { radius = 0.2 } }
  skein.joint(link, previous, { length = 1.0 })
  previous = link
end
```

Both queries answer from the grid the **last step** built, which is the right
answer for a script running before physics and one frame stale for one running
after.

```lua
local hit, dist, nx, ny, nz = skein.raycast(0, 50, 0, 0, -1, 0, 100)
if hit then skein.set_position(marker, 0, 50 - dist, 0) end
```

## Rendering

| Function | |
|---|---|
| `skein.mesh(name)` | mesh id, or nothing if unknown |
| `skein.material(name)` | material id, or nothing if unknown |
| `skein.add_material(desc)` | new material id |
| `skein.set_material(e, id)` | |
| `skein.set_visible(e, bool)` | skips it in culling without destroying it |

Material descriptor: `name`, `albedo`, `emissive`, `roughness`, `metallic`.

```lua
local glow = skein.add_material{
  name = "glow", albedo = {1.0, 0.4, 0.1},
  emissive = {0.9, 0.3, 0.05}, roughness = 0.4, metallic = 0.1,
}
```

## Per-entity callbacks

| Function | |
|---|---|
| `skein.on_update(e, fn)` | `fn(entity, dt)` each frame |
| `skein.time()` | seconds since start |
| `skein.log(...)` | |

The callback is held in the Lua registry by reference and runs before physics.

**A callback that raises is unbound and reported, not fatal.** One broken script
stops being called; the frame continues and every other script keeps running.

```lua
skein.on_update(beacon, function(self, dt)
  angle = angle + 0.4 * dt
  skein.set_position(self, math.cos(angle) * 40, 8 + math.sin(skein.time()) * 2, math.sin(angle) * 40)
  skein.set_rotation(self, 0, angle * 2, 0)
end)
```

Cost: **78.2 ns per callback** — 0.391 ms for 5,000 of them.

## Notes

| | |
|---|---|
| Vectors | passed and returned as loose numbers, not tables — no allocation per call |
| `{x=,y=,z=}` and `{1,2,3}` | both accepted for input vectors |
| A dead entity handle | every function returns nothing rather than raising |
| Lua heap | 384 KB in the benchmark's 5,000-script scene |
| Load order | `skein` is registered before any script runs |

`assets/scripts/demo.lua` is a working example: 96 orbiting emissive beacons on
`on_update`, plus a tower of physics bodies the script drops in and tops up as
they scatter.
