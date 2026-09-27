# Candidate surface algorithm spike (milestone 8)

## Evidence and decision

The local exterior benchmark is the existing `output/default` export
for CELL `00008EA2` at (26, 25). Its saved `geometry.json` identifies the
winning LAND and collision sources and contains 98,558 source triangles:
2,048 LAND, 17,748 collision, and 78,762 render fallback. The run reports
306 missing models among 643 references with model paths. These counts were
read from the saved export on 2026-09-27; the asset bytes are local and are not
committed. The benchmark manifest still has no configured interior or bridge
cell, so coordinate alignment for those cases is **unverified**. The roadmap's
real interior/exterior decision gate is therefore not claimed as passed.

At 16 Skyrim units per horizontal sample, a single 4,096-unit cell needs
65,536 heightfield columns; at 8 units it needs 262,144. Multiple layers per
column are necessary for overpasses. The current evidence has only 19,796
authoritative terrain/collision triangles, so a voxel grid would allocate
many columns without evidence and would rasterize exact LAND/bridge edges.
Its fixed resolution also makes a narrow passage change with cell placement.

An end-to-end read-only run against the installed vanilla `Skyrim.esm` CELL
`00008EA2` at (26, 25), using `--terrain-only --generate-candidate` and
`human@1.0.0`, completed in 8.45 seconds on this development machine. The
decoded LAND supplied 2,048 triangles. The candidate accepted 1,639,
rejected 409 for slope, and emitted 1,639 polygons in 89 regions with valid
index/adjacency/contour topology. The combined GLB has a separate Candidate
NAVM layer and 6,089 displayed triangles. The source LAND bounds are
X 106,496–110,592 and Y 102,400–106,496; candidate vertices are emitted in
that same world coordinate system. This run measures the whole parser/export
path, not just generation, and excludes collision by request. Its fragmented
regions are a review signal, not evidence of playable navigation.

Choose **polygon-based construction** for this milestone. Filter original
terrain/collision triangles, weld edge coordinates with a fixed tolerance,
form regions through traversable shared edges, inset exposed edges by agent
radius, then triangulate the clipped polygons. This retains vertical layers,
exact source coordinates, and a direct source-triangle audit trail. Runtime
and memory scaling are tied primarily to source triangle and edge counts.
The measured source distribution above is evidence for that choice, not a
performance result for the new generator. Re-run this comparison with real
interior and bridge scenes before treating generated candidates as production
navigation.

## Scope and limitations

Generation is opt-in and emits a neutral JSON candidate plus OBJ and GLB
inspection views. The JSON includes the profile name/version, all parameter
values, source-triangle provenance (including LAND sample coordinates),
regions, contours, polygon neighbors, and topology findings.
`polygons[].source_triangle` and `regions[].source_triangles` join to
`source_triangles[].input_index`; each `geometry_source` joins to
`geometry_sources[]`. It does not assign a Bethesda FormID or serialize NVNM.
The candidate is reproducible for identical scene geometry, source order, and
profile. Triangle order from the resolved load order is part of the input.

Only LAND and supported Havok collision are eligible by default. Render
fallback remains diagnostic evidence and is never silently promoted to
walkable ground or an authoritative obstruction. Unsupported collision,
missing meshes, and incomplete LAND can leave real walkable areas absent.
Clearance uses vertical triangle intersections at triangle vertices, edge
midpoints, and centroid. Steep collision triangles crossing a floor
triangle's interior are obstructions. These discrete tests cannot prove
continuous capsule clearance or door semantics. The agent-radius inset
addresses exposed region edges, but complex corners and very thin triangles
may be removed. Exterior selection without neighbors uses triangle centroids
within the selected cell, and the border policy preserves its outer edges.
With `--neighboring-cell-radius`, the generator includes extracted neighbor
cells and welds matching border edges into the same region when their height
difference fits the step profile. It does not invent a link across unavailable
neighbors or geometrically clip triangles that cross the selection border.
