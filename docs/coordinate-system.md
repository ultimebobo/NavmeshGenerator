# Coordinate-system contract

All neutral geometry and NAVM vertices use `skyrim-world-z-up-v1`: Skyrim world units, with X and Y on the horizontal plane and Z increasing upward. The tool does not convert a cell-local coordinate into a separate export space; parsed NAVM positions and transformed reference vertices are emitted in the same world-space convention.

NIF vertices begin in NIF-local space. The extractor resolves NIF parent-node transforms, then applies the neutral reference transform described below.

OBJ is written without axis remapping: `v x y z` is Skyrim world X/Y/Z. A viewer that assumes Y-up may need its own import-axis conversion. Future glTF exports must document any required conversion separately; none exists today.

Use a tolerance of 0.01 world units for synthetic fixture comparisons and 0.1 world units for visual alignment checks, allowing for float serialization and viewer display precision. Larger systematic offsets are evidence of missing transforms or coverage, not a reason to silently relax analysis thresholds.
# Scene-transform implementation

Milestone 3 uses one neutral affine transform implementation for all extracted visual geometry. Vectors are column vectors. A local transform with Euler angles `(x, y, z)` in radians applies rotations in this order: X, then Y, then Z (`Rz * Ry * Rx`), after uniform scale; translation is applied last. A child node's world transform is `parent * child`. NIF parent-node transforms are resolved into the NIF-local vertices before this neutral reference transform is applied.

## Exterior LAND terrain

Exterior terrain uses the `LAND` record's `VHGT` stream. The grid has 33 samples along each axis, so one exterior cell produces 32×32 quads (2,048 triangles). Sample `(x, y)` is positioned at:

```text
worldX = cellX * 4096 + x * 128
worldY = cellY * 4096 + y * 128
worldZ = VHGT height
```

`VHGT` starts with the first sample's float height. Its remaining 1,088 signed-byte deltas are accumulated across a row, with the first sample of each subsequent row continuing from the sample directly above; each delta is eight world units. Terrain triangles retain the owning `LAND` FormID, exterior cell coordinate, and lower-left grid sample in JSON provenance. This covers collision-relevant height surfaces only: terrain textures and visual LOD remain out of scope.
