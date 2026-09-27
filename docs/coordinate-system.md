# Coordinate-system contract

All neutral geometry and NAVM vertices use `skyrim-world-z-up-v1`: Skyrim world units, with X and Y on the horizontal plane and Z increasing upward. The tool does not convert a cell-local coordinate into a separate export space; parsed NAVM positions and transformed reference vertices are emitted in the same world-space convention.

NIF vertices begin in NIF-local space. The extractor resolves NIF parent-node transforms, then applies the neutral reference transform described below.

OBJ is written without axis remapping: `v x y z` is Skyrim world X/Y/Z. A viewer that assumes Y-up may need its own import-axis conversion. Future glTF exports must document any required conversion separately; none exists today.

Use a tolerance of 0.01 world units for synthetic fixture comparisons and 0.1 world units for visual alignment checks, allowing for float serialization and viewer display precision. Larger systematic offsets are evidence of missing transforms or coverage, not a reason to silently relax analysis thresholds.
# Scene-transform implementation

Milestone 3 uses one neutral affine transform representation for extracted geometry. Vectors are column vectors. The generic `FromEulerXYZ` constructor applies X, then Y, then Z (`Rz * Ry * Rx`), after uniform scale; translation is applied last. Placed Skyrim references use `FromSkyrimReference` instead: it matches the game's `NiMatrix3::SetEulerAnglesXYZ` matrix for the `REFR`/`ACHR` `DATA` angles in radians. In particular, a positive reference Z angle rotates model-local +X toward world -Y. A child node's world transform is `parent * child`. NIF parent-node transforms are resolved into the NIF-local vertices before the reference transform is applied.

## Exterior LAND terrain

Exterior terrain uses the `LAND` record's `VHGT` stream. The grid has 33 samples along each axis, so one exterior cell produces 32×32 quads (2,048 triangles). Sample `(x, y)` is positioned at:

```text
worldX = cellX * 4096 + x * 128
worldY = cellY * 4096 + y * 128
worldZ = VHGT height
```

`VHGT` starts with a float height offset and has one signed-byte delta for each of its 1,089 samples. Both are in eight-world-unit height units. The stored LAND grid is row-major: sample `(x, y)` is read at `y * 33 + x`, matching the scene's world-space X/Y basis. Terrain triangles retain the owning `LAND` FormID, exterior cell coordinate, and lower-left grid sample in JSON provenance. This covers collision-relevant height surfaces only: terrain textures and visual LOD remain out of scope.
