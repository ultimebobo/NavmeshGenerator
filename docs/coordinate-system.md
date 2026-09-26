# Coordinate-system contract

All neutral geometry and NAVM vertices use `skyrim-world-z-up-v1`: Skyrim world units, with X and Y on the horizontal plane and Z increasing upward. The tool does not convert a cell-local coordinate into a separate export space; parsed NAVM positions and transformed reference vertices are emitted in the same world-space convention.

NIF vertices begin in NIF-local space. The current extractor applies the reference scale, then its existing Euler rotation implementation, then reference translation: `world = translation + rotate(rotationRadians, local * scale)`. Parent-node transforms are not yet extracted; NIF visual geometry is therefore coverage-limited and this is reported in export metadata. Rotation order is implementation-defined for this POC and must not be treated as a cross-tool transform contract until milestone 3 defines and tests it.

OBJ is written without axis remapping: `v x y z` is Skyrim world X/Y/Z. A viewer that assumes Y-up may need its own import-axis conversion. Future glTF exports must document any required conversion separately; none exists today.

Use a tolerance of 0.01 world units for synthetic fixture comparisons and 0.1 world units for visual alignment checks, allowing for float serialization and viewer display precision. Larger systematic offsets are evidence of missing transforms or coverage, not a reason to silently relax analysis thresholds.
