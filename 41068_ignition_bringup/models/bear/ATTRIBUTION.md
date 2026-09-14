# Attribution

The mesh and texture in this model are third-party assets used under a licence
that **requires attribution**. Keep this file with the model.

- **Model:** "Black bear"
- **Source:** https://poly.pizza/m/56ym_pyVnel
- **Original creator:** "Poly by Google" (Google Poly archive)
- **Licence:** CC-BY 3.0 — https://creativecommons.org/licenses/by/3.0/

## Changes made

Converted from the distributed `.glb` to Collada 1.4.1, because Gazebo Fortress
ships ignition-common4, which has no glTF loader. The conversion baked the
glTF Y-up basis to SDF's Z-up as `(x, y, z) -> (x, -z, y)`, flipped the V
texcoord (glTF's UV origin is top-left, Collada's bottom-left), and unpacked the
embedded PNG to `materials/textures/bear.png`. Geometry is otherwise unmodified:
1334 vertices, 666 triangles. Displayed at `<scale>0.1</scale>`.
