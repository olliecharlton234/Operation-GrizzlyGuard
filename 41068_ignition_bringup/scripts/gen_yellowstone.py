#!/usr/bin/env python3
"""Generate worlds/yellowstone.sdf - Canyon Village, Yellowstone.

Deterministic (fixed seed), so re-running reproduces the same world. Edit the
constants below and re-run rather than hand-editing 120 tree <include> blocks.

Layout, south -> north, with the husky spawning at (0, 0) in the forest:

    y  -40 ..  70   dense lodgepole forest (the husky spawns at 0,0 inside it)
    y   70 ..  92   campground clearing, loop road off the spawn axis
    y   92 .. 104   ragged treeline: broken edge with fingers and clearings
    y      110      fence + service track: the human-made "line"
    y  120 .. 132   RIVER - hard barrier, single bridge at x = +20
    y  135 .. 168   Hayden Valley grassland
    y  170 .. 209   Canyon Village

Playable area is 150 x 250 m (x -75..75, y -40..210); beyond it sits a cheap
visual surround and half-buried ellipsoid mountains.

Two constraints drive the geometry:
  * Nav2 runs inflation_radius 2.0 against a 1x1 m footprint, so every drivable
    corridor here is >= 10 m wide - streets, the bridge deck and the fence gate.
  * Gazebo has no water simulation and a flat water plane is invisible to the
    2D lidar, so the river is enforced by 1.2 m embankments (taller than the
    husky's 0.845 m scan plane), broken only at the bridge.
"""
import math
import random

SEED = 41068
rng = random.Random(SEED)

X_MIN, X_MAX = -75.0, 75.0
Y_MIN, Y_MAX = -40.0, 235.0   # lengthened 2026-09-14; the extra 25 m went to the valley

# The husky spawns at (0, 0) and drives north, so the forest has to be deep
# *north* of the spawn - an earlier cut put the clearing 28 m ahead and you
# could see straight from the spawn to the village.
FOREST_N = 58.0          # north edge of the dense forest
CAMP_S, CAMP_N = 58.0, 78.0
TREELINE_S, TREELINE_N = 78.0, 90.0
FENCE_Y = 96.0
TRACK_Y = 99.5
RIVER_S, RIVER_N = 106.0, 126.0   # 20 m wide, widened from 12
RIVER_MID = 116.0
BRIDGE_X = 20.0
BRIDGE_HALF = 6.5        # 13 m clear between railings
CAMP_ROAD_X = -18.0      # off the spawn axis on purpose - see camp_road below
CAMP_ROAD_S, CAMP_ROAD_N = 52.0, 96.0
VALLEY_S, VALLEY_N = 131.0, 190.0   # 59 m of open grassland, up from 33
VILLAGE_S = 192.0
STREET_Y = 208.0
CENTER_Y = (Y_MIN + Y_MAX) / 2      # everything ring-shaped is centred here

# A large lake at the foot of the western range. gen_mountains.py flattens the
# mesh over the same footprint, so the water sits in a basin rather than on a
# hillside - keep the two in step if you move it.
LAKE_X, LAKE_Y = -300.0, 165.0
LAKE_RX, LAKE_RY = 135.0, 92.0

PINE = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine%20tree'
OAK = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak%20tree'

# Measured from the cached Fuel meshes (the 0.0254 in->m node matrix applies):
# pine is 3.17 m across the canopy and 5.08 m tall, and its collision is the
# full mesh, so at the 0.845 m scan plane a pine is a ~3.2 m wide obstacle.
PINE_R = 1.6
TREE_MIN_SPACING = 5.5   # leaves ~2.3 m of free gap between canopies

out = []
add = out.append


def lerp3(a, b, t):
    t = max(0.0, min(1.0, t))
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


# Canopy colour by how far outside the playable area a tree sits: the green
# forest gives way to olive, then gold, then the muted brown-grey of the slopes.
# These are LINEAR values. Ignition's ogre2 renders <diffuse> as linear colour
# and displays it sRGB-encoded, so a linear 0.13 shows up as a mid-tone ~0.40,
# not the near-black it looks like here. Two earlier passes came out as pale
# sage balloons next to the near-black Fuel pines for exactly this reason.
# Rule of thumb: displayed ~= 1.055 * linear**(1/2.4) - 0.055.
FOLIAGE_STOPS = [(0.0, (0.013, 0.030, 0.015)),
                 (80.0, (0.026, 0.046, 0.018)),
                 (160.0, (0.115, 0.078, 0.020)),
                 (260.0, (0.095, 0.068, 0.048))]


def foliage_colour(d):
    for (d0, c0), (d1, c1) in zip(FOLIAGE_STOPS, FOLIAGE_STOPS[1:]):
        if d <= d1:
            return lerp3(c0, c1, (d - d0) / (d1 - d0) if d1 > d0 else 0.0)
    return FOLIAGE_STOPS[-1][1]


def edge_distance(x, y):
    """How far (x, y) lies outside the playable rectangle; 0 if inside."""
    dx = max(X_MIN - x, 0.0, x - X_MAX)
    dy = max(Y_MIN - y, 0.0, y - Y_MAX)
    return math.hypot(dx, dy)


def pose(x, y, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return f'{x:.2f} {y:.2f} {z:.2f} {roll:.4f} {pitch:.4f} {yaw:.4f}'


def box_visual(name, sx, sy, sz, x, y, z, rgb, yaw=0.0, roll=0.0, amb=0.35, emis=0.0):
    # Ambient is held well below diffuse: with the scene ambient up where the
    # textured ground needs it, ambient==diffuse bleaches every solid-colour
    # box to the same cream.
    # EXCEPT far from the origin. ogre2's directional-light shadow map only
    # covers so much ground, and everything beyond it is treated as shadowed, so
    # the outer surround bands rendered as a huge black plain between the grass
    # and the mountains. Those pass amb=1.0 and are then lit the same either way.
    r, g, b = rgb
    ar, ag, ab = r * amb, g * amb, b * amb
    er, eg, eb = r * emis, g * emis, b * emis
    return f'''      <visual name="{name}">
        <pose>{pose(x, y, z, roll=roll, yaw=yaw)}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry>
        <material>
          <ambient>{ar:.3f} {ag:.3f} {ab:.3f} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <emissive>{er:.3f} {eg:.3f} {eb:.3f} 1</emissive>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>'''


def tile_visual(name, size, x, y, texture, tint=(0.62, 0.62, 0.62)):
    # The tint multiplies the albedo map, so one grass texture can read as damp
    # forest floor in the south and dry golden valley grass in the north.
    # Hayden Valley is tawny in life, not green.
    r, g, b = tint
    return f'''      <visual name="{name}">
        <pose>{pose(x, y, 0.01)}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>
        <material>
          <ambient>{r * 0.72:.3f} {g * 0.72:.3f} {b * 0.72:.3f} 1</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
          <pbr><metal><albedo_map>{texture}</albedo_map></metal></pbr>
        </material>
      </visual>'''


def ground_tint(cy):
    # The tint MULTIPLIES a green grass texture, so a "gold" tint whose green
    # channel is still high just yields olive. To read as dry grass the red
    # channel has to come out at or above the green after the multiply, which
    # means pulling green and blue right down rather than pushing red up.
    if cy < TREELINE_N:
        return (0.40, 0.58, 0.36)       # forest floor, cool green
    if cy < VILLAGE_S - 4:
        return (1.00, 0.55, 0.13)       # Hayden Valley, dry tawny gold
    return (0.85, 0.62, 0.34)           # dusty ground around the village


def ellipsoid_visual(name, rx, ry, rz, x, y, z, rgb, amb=0.35, emis=0.0):
    r, g, b = rgb
    return f'''      <visual name="{name}">
        <pose>{pose(x, y, z)}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><ellipsoid><radii>{rx:.2f} {ry:.2f} {rz:.2f}</radii></ellipsoid></geometry>
        <material>
          <ambient>{r * amb:.3f} {g * amb:.3f} {b * amb:.3f} 1</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
          <emissive>{r * emis:.3f} {g * emis:.3f} {b * emis:.3f} 1</emissive>
          <specular>0.02 0.02 0.02 1</specular>
        </material>
      </visual>'''


def cylinder_visual(name, radius, length, x, y, z, rgb):
    r, g, b = rgb
    return f'''      <visual name="{name}">
        <pose>{pose(x, y, z)}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><cylinder><radius>{radius:.2f}</radius><length>{length:.2f}</length></cylinder></geometry>
        <material>
          <ambient>{r * 0.35:.3f} {g * 0.35:.3f} {b * 0.35:.3f} 1</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
        </material>
      </visual>'''


def include(uri, name, x, y, z=0.0, yaw=0.0):
    return f'''    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <pose>{pose(x, y, z, yaw=yaw)}</pose>
    </include>'''


# ---------------------------------------------------------------- world header
add(f'''<?xml version="1.0"?>
<!-- GENERATED by scripts/gen_yellowstone.py (seed {SEED}) - edit that, not this file. -->
<sdf version="1.8">
  <world name="yellowstone">

    <!-- Server systems (Physics, Sensors, SceneBroadcaster, UserCommands) come
         from config/ignition_server.config via IGN_GAZEBO_SERVER_CONFIG_PATH,
         so no <plugin> blocks are needed here. -->

    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>0 0 40 0 -0 0</pose>
      <diffuse>0.9 0.88 0.82 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>2000</range>
        <constant>0.9</constant>
        <linear>0.005</linear>
        <quadratic>0.0005</quadratic>
      </attenuation>
      <direction>-0.4 0.25 -0.88</direction>
    </light>
    <gravity>0 0 -9.81</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <physics name="default_physics" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4</ambient>
      <background>0.55 0.68 0.85</background>
      <shadows>1</shadows>
    </scene>
    <spherical_coordinates>
      <latitude_deg>44.7358</latitude_deg>
      <longitude_deg>-110.4942</longitude_deg>
      <elevation>2400.0</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>
''')

# -------------------------------------------------------------------- terrain
# One model, one link: a single big collision box plus many visuals. Tiling the
# ground as separate <include> models would cost ~60 extra entities for nothing.
terrain = []
terrain.append(f'''    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <pose>0 {CENTER_Y} -0.1 0 0 0</pose>
          <geometry><box><size>2600 2600 0.2</size></box></geometry>
        </collision>''')

# cheap visual surround beyond the playable area
# Concentric bands running out toward the mountains, so the ground changes
# colour with distance instead of being one flat green to the horizon. Drawn
# outermost-first at slightly increasing z; the playable tiles (z 0.01) cover
# the innermost of them.
for _i, (_size, _z, _rgb) in enumerate([
        (2600, -0.038, (0.115, 0.088, 0.058)),  # runs out past the mountains
        (1600, -0.034, (0.125, 0.095, 0.062)),  # red rock apron at the mountain feet
        (1150, -0.030, (0.150, 0.115, 0.058)),  # dry ochre grassland
        (820,  -0.026, (0.140, 0.120, 0.055)),  # tawny scrub
        (560,  -0.022, (0.125, 0.135, 0.065)),  # olive, greening toward the valley
        (380,  -0.018, (0.140, 0.170, 0.082))]):  # green, meeting the playable edge
    # The far bands lie outside ogre2's directional-light shadow map, so they get
    # no sun at all and only ambient x scene-ambient (0.4) reaches them - which
    # rendered the ground between the grass and the mountains as a black plain.
    # An emissive floor keeps them at their intended colour whether lit or not.
    far = _size > 500
    terrain.append(box_visual(f'surround_{_i}', _size, _size, 0.02, 0, CENTER_Y, _z, _rgb,
                              amb=1.0 if far else 0.55, emis=0.55 if far else 0.0))

# One texture for the whole ground: forest_plane's forest_overhead.png is an
# aerial photograph, so neighbouring tiles do not match and the seams read as
# hard squares at eye level. Forest character comes from the trees instead.
GRASS_TEX = 'model://grass_plane/materials/textures/empty_grass.png'
TILE = 25.0
ti = 0
y = Y_MIN
while y < Y_MAX:
    x = X_MIN
    while x < X_MAX:
        cy = y + TILE / 2
        cx = x + TILE / 2
        terrain.append(tile_visual(f'ground_{ti}', TILE, cx, cy, GRASS_TEX, ground_tint(cy)))
        ti += 1
        x += TILE
    y += TILE

# campground clearing reads as trodden dirt, not forest floor
terrain.append(box_visual('camp_ground', 60, 24, 0.02, 0, (CAMP_S + CAMP_N) / 2, 0.02,
                          (0.34, 0.29, 0.21)))
# service track along the fence, and the campground loop road
terrain.append(box_visual('service_track', 150, 8, 0.02, 0, TRACK_Y, 0.02, (0.34, 0.30, 0.25)))
# The camp road is deliberately off the spawn axis. Centred on x = 0 it opened a
# straight sightline (and a straight drive) from the spawn all the way to the
# river, which defeats the point of starting the robot inside the forest.
terrain.append(box_visual('camp_road', 12, CAMP_ROAD_N - CAMP_ROAD_S, 0.02, CAMP_ROAD_X,
                          (CAMP_ROAD_S + CAMP_ROAD_N) / 2, 0.02, (0.34, 0.30, 0.25)))
# River water. This must sit ABOVE the ground tiles (z 0.01): drawn below them
# it is hidden completely and the river simply is not there.
terrain.append(box_visual('river_water', 150, RIVER_N - RIVER_S, 0.02, 0,
                          (RIVER_S + RIVER_N) / 2, 0.05, (0.16, 0.31, 0.40)))
# bridge deck, flush with the ground so the husky simply drives across
terrain.append(box_visual('bridge_deck', 2 * BRIDGE_HALF, 28, 0.02, BRIDGE_X, RIVER_MID, 0.08,
                          (0.38, 0.27, 0.17)))
# village streets: 12 m wide, which clears the inflation_radius 2.0 rule
terrain.append(box_visual('street_main', 130, 12, 0.02, 0, STREET_Y, 0.02, (0.33, 0.33, 0.35)))
terrain.append(box_visual('street_cross', 12, 44, 0.02, BRIDGE_X, 210, 0.02, (0.33, 0.33, 0.35)))
terrain.append(box_visual('parking', 30, 18, 0.02, -52, 216, 0.02, (0.29, 0.29, 0.31)))
terrain.append('      </link>\n    </model>')
add('\n'.join(terrain))

# --------------------------------------------------------------- river banks
# 1.2 m embankments: taller than the husky's 0.845 m scan plane, so the 2D lidar
# actually returns off them. Broken only at the bridge, which is what forces
# Nav2 to route to the crossing instead of driving straight at the goal.
banks = ['    <model name="river_banks">\n      <static>true</static>\n      <link name="link">']
for tag, by in (('s', RIVER_S - 0.6), ('n', RIVER_N + 0.6)):
    segs = [(X_MIN, BRIDGE_X - BRIDGE_HALF), (BRIDGE_X + BRIDGE_HALF, X_MAX)]
    for i, (x0, x1) in enumerate(segs):
        w = x1 - x0
        cx = (x0 + x1) / 2
        banks.append(f'''        <collision name="bank_{tag}{i}_col">
          <pose>{pose(cx, by, 0.6)}</pose>
          <geometry><box><size>{w:.2f} 1.6 1.2</size></box></geometry>
        </collision>''')
        banks.append(box_visual(f'bank_{tag}{i}', w, 1.6, 1.2, cx, by, 0.6, (0.38, 0.33, 0.25)))
banks.append('      </link>\n    </model>')
add('\n'.join(banks))

# ------------------------------------------------------------------- bridge
bridge = ['    <model name="bridge">\n      <static>true</static>\n      <link name="link">']
for i, sx in enumerate((-1, 1)):
    rx = BRIDGE_X + sx * BRIDGE_HALF
    bridge.append(f'''        <collision name="rail{i}_col">
          <pose>{pose(rx, RIVER_MID, 0.45)}</pose>
          <geometry><box><size>0.3 28 0.9</size></box></geometry>
        </collision>''')
    bridge.append(box_visual(f'rail{i}', 0.3, 28, 0.9, rx, RIVER_MID, 0.45, (0.35, 0.26, 0.18)))
bridge.append('      </link>\n    </model>')
add('\n'.join(bridge))

# -------------------------------------------------------- fence + gate at bridge
fence = ['    <model name="boundary_fence">\n      <static>true</static>\n      <link name="link">']
for i, (x0, x1) in enumerate([(X_MIN, BRIDGE_X - BRIDGE_HALF), (BRIDGE_X + BRIDGE_HALF, X_MAX)]):
    w = x1 - x0
    cx = (x0 + x1) / 2
    fence.append(f'''        <collision name="rail_{i}_col">
          <pose>{pose(cx, FENCE_Y, 0.95)}</pose>
          <geometry><box><size>{w:.2f} 0.12 0.25</size></box></geometry>
        </collision>''')
    fence.append(box_visual(f'rail_{i}', w, 0.12, 0.18, cx, FENCE_Y, 0.95, (0.32, 0.24, 0.16)))
    fence.append(box_visual(f'rail_{i}_low', w, 0.12, 0.16, cx, FENCE_Y, 0.55, (0.32, 0.24, 0.16)))
    px = x0 + 2
    p = 0
    while px < x1:
        fence.append(box_visual(f'post_{i}_{p}', 0.18, 0.18, 1.15, px, FENCE_Y, 0.575,
                                (0.3, 0.22, 0.15)))
        px += 4.0
        p += 1
fence.append('      </link>\n    </model>')
add('\n'.join(fence))

# ------------------------------------------------------------------- village
buildings = [
    # name,             cx,    cy,   sx,   sy,   sz,  colour
    ('visitor_centre', -22.0, 223.0, 34.0, 16.0, 7.0, (0.46, 0.40, 0.33)),
    ('lodge',           34.0, 222.0, 26.0, 13.0, 6.5, (0.34, 0.24, 0.16)),
    ('general_store',  -30.0, 197.0, 16.0, 10.0, 5.0, (0.44, 0.36, 0.27)),
    ('ranger_station',  -56.0, 197.0, 12.0,  9.0, 4.5, (0.30, 0.33, 0.29)),
]
for i in range(6):
    buildings.append((f'cabin_{i}', -4.0 + i * 12.0, 197.0, 8.0, 6.0, 3.6, (0.32, 0.22, 0.14)))
for name, cx, cy, sx, sy, sz, rgb in buildings:
    b = [f'    <model name="{name}">\n      <static>true</static>\n      <link name="link">']
    b.append(f'''        <collision name="col">
          <pose>{pose(0, 0, sz / 2)}</pose>
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>''')
    b.append(box_visual('walls', sx, sy, sz, 0, 0, sz / 2, rgb))
    # Gable roof: two slabs tilted about X meeting at the ridge. Flat-box roofs
    # made every building read as the same anonymous crate.
    a = 0.45
    ridge = (sy / 2) * math.tan(a)
    slab = (sy / 2) / math.cos(a)
    for sgn, tag in ((1, 'n'), (-1, 's')):
        b.append(box_visual(f'roof_{tag}', sx + 1.2, slab, 0.22,
                            0, sgn * sy / 4, sz + ridge / 2, (0.24, 0.22, 0.21),
                            roll=-sgn * a))
    b.append(f'      </link>\n      <pose>{pose(cx, cy)}</pose>\n    </model>')
    add('\n'.join(b))

# ----------------------------------------------------------------- mountains
# Half-buried ellipsoids outside the playable area. Billboards were rejected
# here: they do not parallax over a 250 m drive.
add('    <!-- Mountains: half-buried ellipsoids, visual only, outside the playable area. -->')
# Kept far out: at ~120 m these filled the sky from the village as featureless
# grey walls. Pushed to 250-450 m they read as a range on the horizon instead.
# A single generated heightfield mesh, built by scripts/gen_mountains.py.
# This replaced six clusters of overlapping ellipsoids: at that size an ellipsoid
# reads as a smooth golf ball however you stack them - no ridgelines, no peaks.
# The mesh is flat across the playable area and only lifts beyond d = 230 m, so
# it cannot intrude on the 150 x 250 m rectangle. Visual only.
add('    <!-- Mountain horizon: models/mountain_range, generated. -->')
add(include('model://mountain_range', 'mountain_range', 0.0, 0.0))

# ----------------------------------------------------------------- the lake
# A large water body at the foot of the western range. Built from overlapping
# flattened ellipsoids so the outline is not an obvious oval, with a pale shore
# ring underneath. Everything out here is past the directional light's shadow
# map, hence the emissive floor - see the surround bands above.
lake = ['    <model name="lake">', '      <static>true</static>', '      <link name="link">']
SHORE = (0.150, 0.132, 0.098)
WATER = (0.038, 0.082, 0.115)
lobes = [(0.0, 0.0, 1.00, 1.00), (-0.46, 0.30, 0.62, 0.58),
         (0.52, -0.26, 0.55, 0.62), (0.16, 0.40, 0.50, 0.44)]
# Each lobe gets its own z. Coplanar lobes z-fight along every overlap, which
# made the lake read as a handful of separate puddles instead of one body.
for i, (ox, oy, fx, fy) in enumerate(lobes):
    lx = LAKE_X + ox * LAKE_RX
    ly = LAKE_Y + oy * LAKE_RY
    # Kept very flat on purpose: a domed lobe shades differently across its
    # curve, so overlapping domes show every seam. At rz ~0.1 every lobe's
    # surface normal is straight up and the overlaps become invisible.
    lake.append(ellipsoid_visual(f'shore{i}', LAKE_RX * fx + 6, LAKE_RY * fy + 6, 0.10,
                                 lx, ly, 0.10 + 0.006 * i, SHORE, amb=1.0, emis=0.5))
for i, (ox, oy, fx, fy) in enumerate(lobes):
    lx = LAKE_X + ox * LAKE_RX
    ly = LAKE_Y + oy * LAKE_RY
    lake.append(ellipsoid_visual(f'water{i}', LAKE_RX * fx, LAKE_RY * fy, 0.12,
                                 lx, ly, 0.26 + 0.006 * i, WATER, amb=1.0, emis=0.5))
lake.append('      </link>')
lake.append('    </model>')
add('    <!-- Large lake at the foot of the western range. -->')
add('\n'.join(lake))


def in_lake(x, y, margin=1.5):
    for ox, oy, fx, fy in lobes:
        lx, ly = LAKE_X + ox * LAKE_RX, LAKE_Y + oy * LAKE_RY
        if ((x - lx) / (LAKE_RX * fx * margin)) ** 2 + \
           ((y - ly) / (LAKE_RY * fy * margin)) ** 2 < 1.0:
            return True
    return False


# ------------------------------------------------------------ canyon bluffs
# Layered rock benches standing on the flat ground in front of the mountains.
# This is canyon country and the place is called Canyon Village; without them
# everything outside the playable area was one unbroken green.
bluffs = ['    <model name="canyon_bluffs">', '      <static>true</static>',
          '      <link name="link">']
STRATA = [(0.088, 0.060, 0.038),    # buff
          (0.115, 0.055, 0.030),    # rust
          (0.070, 0.052, 0.036),    # shadowed band
          (0.125, 0.100, 0.068),    # pale caprock
          (0.098, 0.062, 0.040)]    # buff again
bi = 0
for side in (1, -1):
    yy = -30.0
    while yy < 250.0:
        bx = side * rng.uniform(125.0, 205.0)
        by = yy + rng.uniform(0.0, 30.0)
        if in_lake(bx, by, margin=1.9):
            yy += rng.uniform(55.0, 95.0)
            continue
        w = rng.uniform(34.0, 76.0)
        dp = rng.uniform(26.0, 52.0)
        layers = rng.randint(3, 5)
        z = 0.0
        yaw0 = rng.uniform(-0.5, 0.5)
        for k in range(layers):
            hgt = rng.uniform(3.5, 8.0)
            shrink = 1.0 - 0.13 * k
            # each bench is nudged off-centre; stacking them concentrically made
            # every bluff look like a wedding cake
            ox = rng.uniform(-0.10, 0.10) * w
            oy = rng.uniform(-0.10, 0.10) * dp
            bluffs.append(box_visual(f'bluff{bi}_{k}', w * shrink, dp * shrink, hgt,
                                     bx + ox, by + oy, z + hgt / 2,
                                     STRATA[(k + bi) % len(STRATA)],
                                     yaw=yaw0 + rng.uniform(-0.12, 0.12),
                                     amb=1.0, emis=0.28))
            z += hgt
        bi += 1
        yy += rng.uniform(55.0, 95.0)
bluffs.append('      </link>')
bluffs.append('    </model>')
add(f'    <!-- {bi} layered canyon bluffs standing off the playable area. -->')
add('\n'.join(bluffs))

# --------------------------------------------------------------------- trees
# Building footprints, so the scatter does not drop a pine through a roof.
FOOTPRINTS = [(cx, cy, sx, sy) for _n, cx, cy, sx, sy, _sz, _c in buildings]

placed = []


def free(x, y, spacing=TREE_MIN_SPACING):
    if math.hypot(x, y) < 8.0:                       # keep the spawn clear
        return False
    if CAMP_S - 2 < y < CAMP_N + 2 and abs(x) < 32:  # campground clearing
        return False
    if abs(x - CAMP_ROAD_X) < 7 and CAMP_ROAD_S - 1 < y < CAMP_ROAD_N:   # camp road
        return False
    if FENCE_Y - 6 < y < TRACK_Y + 6:                # fence + service track
        return False
    if RIVER_S - 8 < y < RIVER_N + 8:                # river corridor
        return False
    if abs(x - BRIDGE_X) < 12 and 86 < y < 198:      # the bridge approach
        return False
    if abs(y - STREET_Y) < 10:                       # main street
        return False
    for bx, by, bsx, bsy in FOOTPRINTS:              # village buildings
        if abs(x - bx) < bsx / 2 + 3 and abs(y - by) < bsy / 2 + 3:
            return False
    for px, py in placed:
        if math.hypot(x - px, y - py) < spacing:
            return False
    return True


def scatter(n, x0, x1, y0, y1, tries=4000):
    got = []
    for _ in range(tries):
        if len(got) >= n:
            break
        x = rng.uniform(x0, x1)
        y = rng.uniform(y0, y1)
        if free(x, y):
            placed.append((x, y))
            got.append((x, y))
    return got

trees = []
# dense lodgepole forest
# Tree budget. 191 total also loads fine here (7.9 s startup, RTF ~0.94 headless
# against 2.6 s for the 55-tree simple_trees world), so this number is the agreed
# 120-150 cap rather than a measured limit - raise it if you want a denser stand.
for x, y in scatter(104, X_MIN + 2, X_MAX - 2, Y_MIN + 2, FOREST_N):
    trees.append(('pine', x, y))
# ragged treeline: fingers of forest pushing north into clearings, so the edge
# is broken rather than a straight row
finger_xs = [-62, -44, -20, 4, 34, 55]
for fx in finger_xs:
    reach = rng.uniform(4.0, 12.0)
    got = 0
    for _ in range(60):
        if got >= 5:            # cap per finger, to hold the total tree budget
            break
        x = rng.gauss(fx, 7.0)
        y = rng.uniform(TREELINE_S - 6, TREELINE_S + reach)
        if X_MIN + 2 < x < X_MAX - 2 and free(x, y):
            placed.append((x, y))
            trees.append(('pine', x, y))
            got += 1
# a few stragglers out in the valley and around the village
for x, y in scatter(10, X_MIN + 5, X_MAX - 5, VALLEY_S + 4, VALLEY_N - 4, tries=3000):
    trees.append(('pine', x, y))
for x, y in scatter(6, -62, 62, VILLAGE_S - 2, Y_MAX - 4, tries=3000):
    trees.append(('oak', x, y))

npine = sum(1 for t in trees if t[0] == 'pine')
noak = len(trees) - npine
add(f'    <!-- {len(trees)} trees: {npine} lodgepole pine, {noak} broadleaf near the village.')
add('         Pine collision is the full mesh, so each is a ~3.2 m obstacle at the')
add(f'         scan plane; min spacing {TREE_MIN_SPACING} m keeps gaps navigable. -->')
pi = oi = 0
for kind, x, y in trees:
    if kind == 'pine':
        add(include(PINE, f'pine{pi}', x, y, yaw=rng.uniform(0, 6.28)))
        pi += 1
    else:
        add(include(OAK, f'oak{oi}', x, y, yaw=rng.uniform(0, 6.28)))
        oi += 1

# ------------------------------------------------ taper past the playable edge
# Real trees thinning out past the boundary, so the forest does not stop dead in
# a straight line at x = +-75.
taper = []
for _ in range(8000):
    if len(taper) >= 52:
        break
    tx = rng.uniform(X_MIN - 75, X_MAX + 75)
    ty = rng.uniform(Y_MIN - 75, Y_MAX + 75)
    d = edge_distance(tx, ty)
    if not 3.0 < d < 68.0:
        continue
    if rng.random() > 1.0 - d / 74.0:          # density falls off with distance
        continue
    if any(math.hypot(tx - px, ty - py) < 9.0 for px, py in placed):
        continue
    placed.append((tx, ty))
    taper.append((tx, ty))
add(f'    <!-- {len(taper)} trees thinning out past the playable boundary. -->')
for i, (tx, ty) in enumerate(taper):
    add(include(PINE, f'pine_taper{i}', tx, ty, yaw=rng.uniform(0, 6.28)))

# ------------------------------------------------------------- distant grove
# Everything past the taper is inline geometry in a SINGLE model: an ellipsoid
# canopy on a cylinder trunk. Hundreds more Fuel <include> blocks would be
# hundreds more entities; this is one. It also takes an explicit colour, which a
# Fuel include cannot, and that is what carries the green -> olive -> gold shift
# as the ground rises toward the mountains.
grove = ['    <model name="distant_grove">',
         '      <static>true</static>',
         '      <link name="link">']
gplaced = []
for _ in range(30000):
    if len(gplaced) >= 200:
        break
    gx = rng.uniform(-440, 440)
    gy = rng.uniform(85 - 440, 85 + 440)
    d = edge_distance(gx, gy)
    if not 16.0 < d < 138.0:   # stay clear of the now-closer rising ground
        continue
    if rng.random() > math.exp(-d / 165.0):
        continue
    if in_lake(gx, gy, margin=1.25):
        continue
    if any(math.hypot(gx - px, gy - py) < 15.0 for px, py in gplaced):
        continue
    gplaced.append((gx, gy))
    i = len(gplaced) - 1
    # Matched to the real pine (3.17 m across, 5.08 m tall) - at twice that
    # width they read as balloons parked beside the Fuel trees.
    rxy = rng.uniform(1.15, 1.75)
    rz = rng.uniform(3.4, 5.4)
    rgb = foliage_colour(d)
    bark = lerp3((0.022, 0.014, 0.009), (0.045, 0.038, 0.030), min(1.0, d / 260.0))
    grove.append(cylinder_visual(f'trunk{i}', 0.22, 2.2, gx, gy, 1.1, bark))
    grove.append(ellipsoid_visual(f'canopy{i}', rxy, rxy, rz, gx, gy, 1.9 + rz * 0.80, rgb))
grove.append('      </link>')
grove.append('    </model>')
add(f'    <!-- distant grove: {len(gplaced)} trees, one entity, colour by distance. -->')
add('\n'.join(grove))

# Billboard forest walls were removed on 2026-09-14. They sat right on the
# playable boundary at x = +-78 and, now that real trees taper past the edge and
# the distant grove fills in behind them, they read as a flat dark hedge rather
# than depth. To bring them back, re-add forest_wall includes along y = Y_MIN - 3
# and x = +-78 at z = 2 with rpy 1.5708 0 {0, +-1.5708}.

# --------------------------------------------------------------------- bears
add('    <!-- It is Yellowstone. -->')
# Right next to the husky spawn, facing it. The nose of the mesh points -Y,
# hence the yaw. Kept 5.4 m out so it is an obstacle to see,
# not one parked on top of the robot.
add(include('model://bear', 'bear_spawn', 4.5, 3.0, yaw=-0.983))
add(include('model://bear', 'bear_meadow', -34.0, 150.0, yaw=1.1))
add(include('model://bear', 'bear_treeline', -6.0, 84.0, yaw=-2.4))

add('  </world>\n</sdf>')

path = '/home/mahak/41068_ws/src/41068_ignition_bringup/worlds/yellowstone.sdf'
with open(path, 'w') as f:
    f.write('\n'.join(out) + '\n')
print(f'wrote {path}')
print(f'  playable trees: {npine} pine + {noak} oak = {len(trees)}')
print(f'  taper pines beyond the edge: {len(taper)}')
print(f'  distant grove: {len(gplaced)} trees in 1 entity')
print(f'  ground tiles: {ti}')
