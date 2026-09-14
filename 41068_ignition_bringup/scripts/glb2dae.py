#!/usr/bin/env python3
"""Dependency-free GLB -> Collada 1.4.1 converter for Gazebo Fortress.

Fortress ships ignition-common4, whose mesh loaders are Collada/OBJ/STL only, so
downloaded .glb assets must be converted. Handles the two traps:
  1. glTF is Y-up, SDF is Z-up  -> bake (x, y, z) -> (x, -z, y), declare Z_UP.
     That matrix has determinant +1, so triangle winding is preserved as-is.
  2. Collada resolves <init_from> relative to the .dae file, not the model root,
     so the texture reference is written as ../materials/textures/<name>.png.
Also flips the V texcoord: glTF's UV origin is the image's top-left, Collada's
is bottom-left.
"""
import json, os, struct, sys

F = '{:.7g}'.format
CT = {5120: ('b', 1), 5121: ('B', 1), 5122: ('h', 2),
      5123: ('H', 2), 5125: ('I', 4), 5126: ('f', 4)}
NCOMP = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}


def load_glb(path):
    raw = open(path, 'rb').read()
    magic, _, length = struct.unpack_from('<III', raw, 0)
    if magic != 0x46546C67:
        sys.exit(f'{path}: not a GLB')
    off, chunks = 12, {}
    while off < length:
        clen, ctype = struct.unpack_from('<II', raw, off)
        chunks[struct.pack('<I', ctype).decode('ascii').strip(' \x00')] = (off + 8, clen)
        off += 8 + clen
    jo, jl = chunks['JSON']
    bo, bl = chunks['BIN']
    return json.loads(raw[jo:jo + jl]), raw[bo:bo + bl]


def read_accessor(G, blob, idx):
    """Return a list of per-element tuples, honouring byteStride."""
    acc = G['accessors'][idx]
    view = G['bufferViews'][acc['bufferView']]
    fmt, size = CT[acc['componentType']]
    n = NCOMP[acc['type']]
    base = view.get('byteOffset', 0) + acc.get('byteOffset', 0)
    stride = view.get('byteStride') or size * n
    out = []
    for i in range(acc['count']):
        out.append(struct.unpack_from('<' + fmt * n, blob, base + i * stride))
    return out


def src(sid, name, vals, params):
    """A Collada <source>: flat float array + accessor naming its components."""
    flat = [c for v in vals for c in v]
    stride = len(params)
    return f'''      <source id="{sid}">
        <float_array id="{sid}-array" count="{len(flat)}">{' '.join(F(x) for x in flat)}</float_array>
        <technique_common>
          <accessor source="#{sid}-array" count="{len(vals)}" stride="{stride}">
{chr(10).join(f'            <param name="{p}" type="float"/>' for p in params)}
          </accessor>
        </technique_common>
      </source>'''


def convert(glb, out_dae, out_png, texref):
    G, blob = load_glb(glb)
    prim = G['meshes'][0]['primitives'][0]
    if prim.get('mode', 4) != 4:
        sys.exit('only triangle primitives (mode 4) supported')
    if any('matrix' in n or 'rotation' in n or 'scale' in n or 'translation' in n
           for n in G['nodes']):
        sys.exit('node transforms present - not handled')

    pos = read_accessor(G, blob, prim['attributes']['POSITION'])
    nrm = read_accessor(G, blob, prim['attributes']['NORMAL'])
    uv = read_accessor(G, blob, prim['attributes']['TEXCOORD_0'])
    idx = [i[0] for i in read_accessor(G, blob, prim['indices'])]

    # Trap 1: Y-up -> Z-up.
    pos = [(x, -z, y) for (x, y, z) in pos]
    nrm = [(x, -z, y) for (x, y, z) in nrm]
    # glTF UV origin is top-left, Collada's is bottom-left.
    uv = [(u, 1.0 - v) for (u, v) in uv]

    # Embedded PNG -> a real file beside the mesh.
    img = G['images'][0]
    view = G['bufferViews'][img['bufferView']]
    o = view.get('byteOffset', 0)
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    with open(out_png, 'wb') as f:
        f.write(blob[o:o + view['byteLength']])

    ntri = len(idx) // 3
    p = ' '.join(f'{i} {i} {i}' for i in idx)   # VERTEX/NORMAL/TEXCOORD share indices
    dae = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>glb2dae.py</authoring_tool></contributor>
    <unit meter="1" name="meter"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_images>
    <image id="tex-img" name="tex-img">
      <init_from>{texref}</init_from>
    </image>
  </library_images>
  <library_effects>
    <effect id="mat-fx">
      <profile_COMMON>
        <newparam sid="tex-surface">
          <surface type="2D"><init_from>tex-img</init_from></surface>
        </newparam>
        <newparam sid="tex-sampler">
          <sampler2D><source>tex-surface</source></sampler2D>
        </newparam>
        <technique sid="common">
          <phong>
            <emission><color>0 0 0 1</color></emission>
            <ambient><color>0.3 0.3 0.3 1</color></ambient>
            <diffuse><texture texture="tex-sampler" texcoord="UVSET0"/></diffuse>
            <specular><color>0 0 0 1</color></specular>
            <shininess><float>1</float></shininess>
          </phong>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="mat" name="mat"><instance_effect url="#mat-fx"/></material>
  </library_materials>
  <library_geometries>
    <geometry id="geo" name="geo">
      <mesh>
{src('geo-pos', 'position', pos, 'XYZ')}
{src('geo-nrm', 'normal', nrm, 'XYZ')}
{src('geo-uv', 'uv', uv, 'ST')}
        <vertices id="geo-verts">
          <input semantic="POSITION" source="#geo-pos"/>
        </vertices>
        <triangles count="{ntri}" material="mat">
          <input semantic="VERTEX" source="#geo-verts" offset="0"/>
          <input semantic="NORMAL" source="#geo-nrm" offset="1"/>
          <input semantic="TEXCOORD" source="#geo-uv" offset="2" set="0"/>
          <p>{p}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="scene" name="scene">
      <node id="node" name="node" type="NODE">
        <instance_geometry url="#geo">
          <bind_material>
            <technique_common>
              <instance_material symbol="mat" target="#mat">
                <bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/>
              </instance_material>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
'''
    os.makedirs(os.path.dirname(out_dae), exist_ok=True)
    open(out_dae, 'w').write(dae)

    lo = [min(v[i] for v in pos) for i in range(3)]
    hi = [max(v[i] for v in pos) for i in range(3)]
    print(f'{len(pos)} verts, {ntri} tris, texture {view["byteLength"]} bytes')
    print('raw Z-up extents  X %.4f  Y %.4f  Z %.4f' % tuple(hi[i]-lo[i] for i in range(3)))
    print('raw min           X %.4f  Y %.4f  Z %.4f' % tuple(lo))
    for s in (0.1,):
        print(f'at scale {s}:    X %.4f  Y %.4f  Z %.4f  (min z %.4f)'
              % (*[(hi[i]-lo[i])*s for i in range(3)], lo[2]*s))


if __name__ == '__main__':
    convert(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
