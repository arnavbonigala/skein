-- Scripted layer on top of the C++ systems. Everything here goes through the
-- same component pools the renderer and physics read, so scripted entities are
-- culled, lit and simulated exactly like the ones spawned from C++.

local BEACON_COUNT = 96
local RING_RADIUS = 46.0
local beacons = {}

local ember = skein.material("ember") or 0
local copper = skein.material("copper") or 0

for i = 1, BEACON_COUNT do
  local angle = (i / BEACON_COUNT) * math.pi * 2.0
  local radius = RING_RADIUS + (i % 7) * 3.0
  local e = skein.spawn{
    position = {math.cos(angle) * radius, 6.0 + (i % 5) * 2.5, math.sin(angle) * radius},
    scale = {1.4, 1.4, 1.4},
    mesh = "geosphere",
    material = ember,
  }
  beacons[#beacons + 1] = { entity = e, angle = angle, radius = radius, speed = 0.25 + (i % 9) * 0.045 }

  skein.on_update(e, function(self, dt)
    local b = beacons[i]
    b.angle = b.angle + b.speed * dt
    local bob = math.sin(skein.time() * 1.7 + i) * 2.4
    skein.set_position(self,
      math.cos(b.angle) * b.radius,
      8.0 + (i % 5) * 2.5 + bob,
      math.sin(b.angle) * b.radius)
    skein.set_rotation(self, 0.0, b.angle * 2.0, 0.0)
  end)
end

-- A tower of physics bodies the scripts drop in, then top up as they scatter.
local TOWER_TARGET = 220
local dropped = {}

local function drop(i)
  local e = skein.spawn{
    position = {(i % 9) * 1.3 - 5.2, 60.0 + (i % 13) * 1.6, (i // 9 % 9) * 1.3 - 5.2},
    scale = {1.0, 1.0, 1.0},
    mesh = "pylon",
    material = copper,
    collider = { kind = "sphere", radius = 0.62, restitution = 0.45 },
  }
  dropped[#dropped + 1] = e
  return e
end

for i = 1, TOWER_TARGET do drop(i) end

local reseed = 0
local frames = 0

function on_frame(dt)
  frames = frames + 1
  reseed = reseed + dt

  -- Recycle bodies that have settled so the pile keeps moving.
  if reseed > 2.0 then
    reseed = 0
    local recycled = 0
    for index = #dropped, 1, -1 do
      local e = dropped[index]
      if not skein.alive(e) then
        table.remove(dropped, index)
      else
        local _, y, _ = skein.position(e)
        if y and y < 1.5 and recycled < 24 then
          skein.set_position(e, (index % 9) * 1.3 - 5.2, 70.0, (index // 9 % 9) * 1.3 - 5.2)
          skein.set_velocity(e, 0.0, -2.0, 0.0)
          recycled = recycled + 1
        end
      end
    end
  end

  if frames == 1 then
    skein.log(string.format("scripted %d beacons and %d falling bodies out of %d entities",
      BEACON_COUNT, #dropped, skein.entity_count()))
  end
end
