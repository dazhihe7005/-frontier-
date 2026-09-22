# Task 1 irregular goaf

This is the canonical Task 1 environment shell. Its nominal mean dimensions
are 24 m long, 8 m wide and 4 m high. The cross-section is a deterministic,
perturbed half ellipse rather than a box. The entrance at `x=-2 m` is open and
the end at `x=22 m` is closed.

`model.sdf` uses the same OBJ for rendering, MID360 ray intersection and
collision. Regenerate it with:

```bash
./scripts/generate_irregular_goaf_mesh.py \
  models/goaf_rock_24x8x4/meshes/goaf_rock_inner.obj
```
