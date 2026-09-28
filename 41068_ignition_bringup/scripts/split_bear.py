#!/usr/bin/env python3
"""Split models/bear into a body and four legs -> models/bear_walk/.

The Poly bear mesh is a rigid shell with no skeleton, so it cannot be animated
as an SDF <actor>. This cuts it into five independent pieces instead, each its
own model, so a driver node can pose them separately and swing the legs through
a gait. Crude compared to real rigging - the hips do not deform - but it gives
actual leg movement with no rigging and no physics.

Split rule, in raw mesh units (the model is displayed at scale 0.1):
  a triangle is a LEG if its centroid is below z = 4.2 and |y| > 1.2,
  assigned by sign(x) (left/right) and sign(y) (front/rear).
Everything else is the body. The four leg clusters are cleanly separated -
there are no vertices at all between y = -1.17 and y = +1.52 below z = 3.

Each leg's geometry is translated so its hip sits at the origin, so posing the
model rotates the leg about the hip rather than about the bear's centre.
"""
import os
import xml.etree.ElementTree as ET

import numpy as np

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models', 'bear')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models', 'bear_walk')
LEG_TOP, LEG_Y_GAP = 4.2, 1.2
HIP_Z = 4.2


def read(path):
    r = ET.parse(path).getroot()
    def arr(sid, stride):
        t = r.find(f".//c:source[@id='{sid}']/c:float_array", NS).text
        return np.array([float(v) for v in t.split()]).reshape(-1, stride)
    p = r.find('.//c:triangles/c:p', NS).text.split()
    idx = np.array([int(i) for i in p]).reshape(-1, 3, 3)   # tri x corner x (v,n,uv)
    return arr('geo-pos', 3), arr('geo-nrm', 3), arr('geo-uv', 2), idx


def source(sid, vals, params):
    flat = vals.reshape(-1)
    return (f'      <source id="{sid}">\n'
            f'        <float_array id="{sid}-array" count="{flat.size}">'
            + ' '.join(f'{v:.6g}' for v in flat) + '</float_array>\n'
            '        <technique_common>\n'
            f'          <accessor source="#{sid}-array" count="{len(vals)}" stride="{len(params)}">\n'
            + '\n'.join(f'            <param name="{p}" type="float"/>' for p in params)
            + '\n          </accessor>\n        </technique_common>\n      </source>')


def write_dae(path, pos, nrm, uv, faces, texref):
    p = ' '.join(f'{a} {b} {c}' for f in faces for a, b, c in f)
    dae = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis>
    <contributor><authoring_tool>split_bear.py</authoring_tool></contributor></asset>
  <library_images><image id="tex-img"><init_from>{texref}</init_from></image></library_images>
  <library_effects><effect id="mat-fx"><profile_COMMON>
    <newparam sid="s"><surface type="2D"><init_from>tex-img</init_from></surface></newparam>
    <newparam sid="sm"><sampler2D><source>s</source></sampler2D></newparam>
    <technique sid="common"><phong>
      <emission><color>0 0 0 1</color></emission>
      <ambient><color>0.3 0.3 0.3 1</color></ambient>
      <diffuse><texture texture="sm" texcoord="UVSET0"/></diffuse>
      <specular><color>0 0 0 1</color></specular>
    </phong></technique>
  </profile_COMMON></effect></library_effects>
  <library_materials><material id="mat"><instance_effect url="#mat-fx"/></material></library_materials>
  <library_geometries><geometry id="geo"><mesh>
{source('geo-pos', pos, 'XYZ')}
{source('geo-nrm', nrm, 'XYZ')}
{source('geo-uv', uv, 'ST')}
    <vertices id="geo-verts"><input semantic="POSITION" source="#geo-pos"/></vertices>
    <triangles count="{len(faces)}" material="mat">
      <input semantic="VERTEX" source="#geo-verts" offset="0"/>
      <input semantic="NORMAL" source="#geo-nrm" offset="1"/>
      <input semantic="TEXCOORD" source="#geo-uv" offset="2" set="0"/>
      <p>{p}</p>
    </triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="scene"><node id="n" type="NODE">
    <instance_geometry url="#geo"><bind_material><technique_common>
      <instance_material symbol="mat" target="#mat">
        <bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/>
      </instance_material></technique_common></bind_material></instance_geometry>
  </node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
'''
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, 'w').write(dae)


def main():
    pos, nrm, uv, idx = read(os.path.join(SRC, 'meshes', 'bear.dae'))
    cent = pos[idx[:, :, 0]].mean(axis=1)

    is_leg = (cent[:, 2] < LEG_TOP) & (np.abs(cent[:, 1]) > LEG_Y_GAP)
    parts = {'body': ~is_leg}
    for tag, sx, sy in (('lf', -1, -1), ('rf', 1, -1), ('lr', -1, 1), ('rr', 1, 1)):
        parts[tag] = is_leg & (np.sign(cent[:, 0]) == sx) & (np.sign(cent[:, 1]) == sy)

    hips = {}
    for name, mask in parts.items():
        faces = idx[mask]
        if not len(faces):
            print(f'  {name}: EMPTY - skipped')
            continue
        # offset so the hip (top centre of the leg) sits at the model origin
        if name == 'body':
            off = np.zeros(3)
        else:
            v = pos[faces[:, :, 0].reshape(-1)]
            off = np.array([v[:, 0].mean(), v[:, 1].mean(), HIP_Z])
            hips[name] = off * 0.1          # record in display scale
        write_dae(os.path.join(OUT, 'meshes', f'{name}.dae'),
                  pos - off, nrm, uv, faces, '../materials/textures/bear.png')
        print(f'  {name:4s}: {len(faces):3d} triangles, hip offset {off.round(2)}')

    os.makedirs(os.path.join(OUT, 'materials', 'textures'), exist_ok=True)
    import shutil
    shutil.copy(os.path.join(SRC, 'materials', 'textures', 'bear.png'),
                os.path.join(OUT, 'materials', 'textures', 'bear.png'))

    for name in parts:
        body = name == 'body'
        col = ('''      <collision name="collision">
        <pose>0 -0.18 0.51 0 0 0</pose>
        <geometry><box><size>0.67 1.55 1.0</size></box></geometry>
      </collision>
''' if body else '')
        open(os.path.join(OUT, f'model_{name}.sdf'), 'w').write(f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="bear_walk_{name}">
    <static>true</static>
    <link name="link">
{col}      <visual name="visual">
        <geometry>
          <mesh><uri>model://bear_walk/meshes/{name}.dae</uri><scale>0.1 0.1 0.1</scale></mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
''')
    open(os.path.join(OUT, 'hips.txt'), 'w').write(
        '\n'.join(f'{k} {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}' for k, v in hips.items()) + '\n')
    print('  hip positions (display scale) written to models/bear_walk/hips.txt')


if __name__ == '__main__':
    main()
