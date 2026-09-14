#!/usr/bin/env python3
"""Generate models/mountain_range/ - a real mountain horizon for yellowstone.sdf.

Replaces six overlapping-ellipsoid "mountains". Ellipsoids at that size read as
smooth golf balls or rounded hills however you cluster them: no ridgelines, no
peaks, no valleys. This builds an actual heightfield mesh instead.

The surface is a ring: flat at zero across the playable area, rising through a
smoothstep falloff to ridged multifractal peaks far out, so the mountains never
intrude on the 150 x 250 m playable rectangle. Visual only - no collision.

Triangles are split into three Collada geometries by height, each with its own
material (treeline / rock / snow), which is how one mesh gets three colours.

Colours are LINEAR: Ignition renders <diffuse> as linear and displays it
sRGB-encoded, so these look far too dark in the file and are correct on screen.
Everything is tinted toward the sky, because distant terrain reads lighter and
bluer, not darker.

Run:  python3 scripts/gen_mountains.py
"""
import math
import os
import numpy as np

SEED = 41068
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   '..', 'models', 'mountain_range')

# playable rectangle these must stay clear of (matches gen_yellowstone.py)
X_MIN, X_MAX = -75.0, 75.0
Y_MIN, Y_MAX = -40.0, 235.0

EXTENT = 900.0          # half-width of the generated ring, centred on the world
STEP = 14.0             # grid spacing in metres
RISE_START = 155.0      # distance from the playable edge where ground starts to lift
RISE_FULL = 430.0       # ...and where the range reaches full height
PEAK = 210.0            # maximum peak height

# Lake basin: the mesh is held flat over this footprint so the water body in
# gen_yellowstone.py sits in a basin at the mountain foot rather than on a
# slope. Keep these in step with LAKE_* there.
LAKE_X, LAKE_Y = -300.0, 165.0
LAKE_RX, LAKE_RY = 135.0, 92.0
LAKE_MARGIN = 1.55        # how far past the water the ground stays flat

SNOW_Z = 132.0
# Kept low on purpose. At 58 m the treeline band was thick enough that the
# range's lower slopes formed one continuous dark wall along the horizon - and
# since we mostly view their shadowed west faces, it read as a black stripe
# under the peaks rather than as forested foothills.
TREELINE_Z = 25.0

rng = np.random.default_rng(SEED)


def value_noise(shape, freq, gen):
    """Smoothed value noise on a grid, bilinear with a smoothstep fade."""
    gh, gw = int(freq) + 2, int(freq) + 2
    lattice = gen.random((gh, gw))
    ys = np.linspace(0, gh - 1.001, shape[0])
    xs = np.linspace(0, gw - 1.001, shape[1])
    y0 = ys.astype(int)[:, None]
    x0 = xs.astype(int)[None, :]
    fy = (ys - ys.astype(int))[:, None]
    fx = (xs - xs.astype(int))[None, :]
    fy = fy * fy * (3 - 2 * fy)      # smoothstep
    fx = fx * fx * (3 - 2 * fx)
    v00 = lattice[y0, x0]
    v10 = lattice[y0 + 1, x0]
    v01 = lattice[y0, x0 + 1]
    v11 = lattice[y0 + 1, x0 + 1]
    return (v00 * (1 - fy) * (1 - fx) + v10 * fy * (1 - fx)
            + v01 * (1 - fy) * fx + v11 * fy * fx)


def ridged(shape, octaves=6, base_freq=3.0):
    """Ridged multifractal: sharp crests instead of rolling blobs."""
    total = np.zeros(shape)
    amp, freq, norm = 1.0, base_freq, 0.0
    for _ in range(octaves):
        n = value_noise(shape, freq, rng)
        r = 1.0 - np.abs(2.0 * n - 1.0)      # fold into ridges
        total += (r ** 1.7) * amp
        norm += amp
        amp *= 0.5
        freq *= 2.0
    return total / norm


def build():
    n = int(2 * EXTENT / STEP) + 1
    ax = np.linspace(-EXTENT, EXTENT, n)
    ay = np.linspace(-EXTENT, EXTENT, n)
    gx, gy = np.meshgrid(ax, ay)
    wy = gy + (Y_MIN + Y_MAX) / 2             # world centre of the playable area

    dx = np.maximum.reduce([X_MIN - gx, np.zeros_like(gx), gx - X_MAX])
    dy = np.maximum.reduce([Y_MIN - wy, np.zeros_like(wy), wy - Y_MAX])
    d = np.hypot(dx, dy)

    t = np.clip((d - RISE_START) / (RISE_FULL - RISE_START), 0.0, 1.0)
    t = t * t * (3 - 2 * t)

    h = ridged((n, n)) * (t ** 1.25) * PEAK
    # a second, slower band so some directions carry higher ranges than others
    h *= 0.55 + 0.85 * value_noise((n, n), 2.0, np.random.default_rng(SEED + 7))
    h[d < RISE_START] = 0.0

    # Flatten the lake basin, with a soft rim so the slope does not step.
    lr = np.hypot((gx - LAKE_X) / (LAKE_RX * LAKE_MARGIN),
                  (wy - LAKE_Y) / (LAKE_RY * LAKE_MARGIN))
    basin = np.clip((lr - 0.62) / 0.38, 0.0, 1.0)
    h *= basin * basin * (3 - 2 * basin)

    # Taper to zero at the outer edge of the grid too. Without this the ridged
    # noise simply stops at +-EXTENT at whatever height it happens to be, leaving
    # a cliff all the way round; through a gap between near ranges that far edge
    # shows up as a thin straight dark streak on the horizon.
    edge = np.minimum(EXTENT - np.abs(gx), EXTENT - np.abs(gy))
    fade = np.clip(edge / 160.0, 0.0, 1.0)
    h *= fade * fade * (3 - 2 * fade)
    return gx, wy, h, n


def normals(gx, gy, h, n):
    nz = np.zeros((n, n, 3))
    dzdx = np.gradient(h, STEP, axis=1)
    dzdy = np.gradient(h, STEP, axis=0)
    nz[..., 0] = -dzdx
    nz[..., 1] = -dzdy
    nz[..., 2] = 1.0
    nz /= np.linalg.norm(nz, axis=2, keepdims=True)
    return nz


def src(sid, vals, params):
    flat = vals.reshape(-1)
    return (f'      <source id="{sid}">\n'
            f'        <float_array id="{sid}-array" count="{flat.size}">'
            + ' '.join(f'{v:.2f}' for v in flat) +
            '</float_array>\n'
            '        <technique_common>\n'
            f'          <accessor source="#{sid}-array" count="{vals.shape[0]}" stride="{len(params)}">\n'
            + '\n'.join(f'            <param name="{p}" type="float"/>' for p in params) +
            '\n          </accessor>\n'
            '        </technique_common>\n'
            '      </source>')


def main():
    gx, gy, h, n = build()
    nrm = normals(gx, gy, h, n)

    verts = np.stack([gx, gy - 85.0, h], axis=2).reshape(-1, 3)
    norms = nrm.reshape(-1, 3)

    # triangles, skipping the flat interior entirely
    tris = {'treeline': [], 'rock': [], 'snow': []}
    idx = lambda r, c: r * n + c
    for r in range(n - 1):
        for c in range(n - 1):
            quad = (h[r, c], h[r + 1, c], h[r, c + 1], h[r + 1, c + 1])
            if max(quad) <= 0.01:
                continue                      # flat ground: emit nothing
            top = max(quad)
            band = 'snow' if top > SNOW_Z else ('rock' if top > TREELINE_Z else 'treeline')
            a, b, cc, dd = idx(r, c), idx(r + 1, c), idx(r, c + 1), idx(r + 1, c + 1)
            tris[band] += [(a, b, cc), (cc, b, dd)]

    # LINEAR, and deliberately hazed toward the sky colour (0.55 0.68 0.85).
    # A first pass at half these values rendered the range near-black: we mostly
    # see its shadowed faces, since the sun points -0.4 0.25 -0.88, and distant
    # terrain that is darker than the sky reads as a hole cut in it.
    mats = {
        'treeline': (0.150, 0.175, 0.185),
        'rock': (0.205, 0.225, 0.275),
        'snow': (0.570, 0.615, 0.690),
    }
    geos, nodes, effects, materials = [], [], [], []
    for band, faces in tris.items():
        if not faces:
            continue
        p = ' '.join(f'{i} {i}' for f in faces for i in f)
        geos.append(f'''    <geometry id="{band}-geo" name="{band}-geo">
      <mesh>
{src(f'{band}-pos', verts, 'XYZ')}
{src(f'{band}-nrm', norms, 'XYZ')}
        <vertices id="{band}-verts">
          <input semantic="POSITION" source="#{band}-pos"/>
        </vertices>
        <triangles count="{len(faces)}" material="{band}-mat">
          <input semantic="VERTEX" source="#{band}-verts" offset="0"/>
          <input semantic="NORMAL" source="#{band}-nrm" offset="1"/>
          <p>{p}</p>
        </triangles>
      </mesh>
    </geometry>''')
        r, g, b = mats[band]
        effects.append(f'''    <effect id="{band}-fx">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <emission><color>0 0 0 1</color></emission>
            <ambient><color>{r:.4f} {g:.4f} {b:.4f} 1</color></ambient>
            <diffuse><color>{r:.4f} {g:.4f} {b:.4f} 1</color></diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>''')
        materials.append(f'    <material id="{band}-mat" name="{band}-mat">'
                         f'<instance_effect url="#{band}-fx"/></material>')
        nodes.append(f'''      <node id="{band}-node" name="{band}-node" type="NODE">
        <instance_geometry url="#{band}-geo">
          <bind_material><technique_common>
            <instance_material symbol="{band}-mat" target="#{band}-mat"/>
          </technique_common></bind_material>
        </instance_geometry>
      </node>''')

    dae = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>gen_mountains.py</authoring_tool></contributor>
    <unit meter="1" name="meter"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_effects>
{chr(10).join(effects)}
  </library_effects>
  <library_materials>
{chr(10).join(materials)}
  </library_materials>
  <library_geometries>
{chr(10).join(geos)}
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="scene" name="scene">
{chr(10).join(nodes)}
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
'''
    os.makedirs(os.path.join(OUT, 'meshes'), exist_ok=True)
    with open(os.path.join(OUT, 'meshes', 'range.dae'), 'w') as f:
        f.write(dae)
    with open(os.path.join(OUT, 'model.sdf'), 'w') as f:
        f.write('''<?xml version="1.0" ?>
<sdf version="1.8">
  <!-- Mountain horizon for the yellowstone world. Visual only: it is scenery,
       200 m tall and hundreds of metres out, and giving it collision would cost
       a large mesh in the physics broadphase for nothing. Generated by
       scripts/gen_mountains.py - regenerate rather than editing the mesh. -->
  <model name="mountain_range">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry>
          <mesh><uri>model://mountain_range/meshes/range.dae</uri></mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
''')
    with open(os.path.join(OUT, 'model.config'), 'w') as f:
        f.write('''<?xml version="1.0"?>
<model>
  <name>mountain_range</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>
    Ridged-multifractal mountain horizon surrounding the yellowstone world.
    Flat across the playable area, rising to snow-capped peaks beyond it.
    Generated by scripts/gen_mountains.py (seed 41068).
  </description>
</model>
''')
    total = sum(len(v) for v in tris.values())
    size = os.path.getsize(os.path.join(OUT, 'meshes', 'range.dae'))
    print(f'grid {n}x{n} at {STEP} m, peak {h.max():.0f} m')
    for band, faces in tris.items():
        print(f'  {band:9s} {len(faces):7d} triangles')
    print(f'  total    {total:7d} triangles, range.dae {size/1e6:.1f} MB')


if __name__ == '__main__':
    main()
