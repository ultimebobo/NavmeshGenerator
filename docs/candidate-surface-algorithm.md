# Candidate surface algorithm spike (milestone 8)

## Current generator

The application now uses Recast Navigation for candidate generation. The
polygon-based measurements and decisions below document the previous spike and
remain useful as comparison evidence; they do not describe the active run path.
The Recast adapter consumes supported terrain and collision, converts Skyrim
world Z-up to Recast Y-up, and runs Recast's voxel rasterization, walkability
filters, radius erosion, monotone region partition, contour construction, and
polygon mesh construction. It triangulates Recast polygons for the neutral
candidate JSON/OBJ and combined GLB. The horizontal voxel cell size is at least
4 Skyrim units and increases for wide extracted areas so neither grid axis
exceeds roughly 2048 columns. Vertical cell height is 2 world units. This
resolves 12-unit stair treads in a single cell. A three-cell-wide footprint
uses about 6-unit horizontal voxels, although collision extending outside
those cells can make the extracted bounds and voxels larger. Larger extracted
areas may still lose narrow stairs. The
default `human@1.1.0` profile allows a 28-unit climb; `human@1.0.0` remains
available with its original 18-unit climb for historical comparisons. A local
Riverwood03 `WalkwayStairs15` collision sample has tread rises near 25 units.
With the old climb, Recast split the isolated stair collision into five
regions. The new profile joined it into one region, including when the test
uses the wider extracted scene bounds. The slope and radius settings were
unchanged: the walkable tread faces are flat and the normal radius fits. A
full CLI rerun of Riverwood03 with one neighboring-cell ring found three
placements of that stair model. With `human@1.0.0`, each placement's candidate
polygons appeared in two connected regions. With `human@1.1.0`, polygons from
all three placements joined one connected region. These are local game-data
measurements; the source assets are not committed.

Source-triangle provenance is recovered by the closest source height at each
generated triangle's XY centroid. This is an approximate audit join after
voxelization. Recast's smoothing and erosion can remove small supported areas;
output is still an inspection candidate and is not Bethesda NAVM serialization.
The profile's agent dimensions, clearance, slope, step height, radius, and
minimum region area feed the Recast build. Weld tolerance, contour tolerance,
and cell-border policy remain in the versioned profile for historical output
compatibility but do not control this voxel-based build.

A Windows UI terrain-only run on a local exterior CELL on 2026-09-29 produced
516 Recast candidate triangles in two connected regions. Its candidate topology
validated and `scene.glb` contained a `Candidate NAVM` object with 516 triangles.
The named CELL is reproduction data rather than a special-case rule.

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

The local Riverwood CELL reproduction uses the five supported collision
placements beneath the existing elevated walkway NAVM. Before the region and
inset fixes, that walkway had no candidate polygons. In a 2026-09-28 run with
one neighboring-cell ring and `human@1.0.0`, the candidate had 83 walkway
polygons in one region; all 18 existing walkway NAVM polygon centroids lay on
those candidate polygons at the same height. Candidate topology was valid.
This checks coverage of one known structure, not navigation quality across all
bridges or stairs. The local asset cache supplied 755 models; 222 referenced
models remained unavailable in that run.

A 2026-09-28 terrain-only rerun of the local exterior benchmark with the
interior simplifier and `human@1.0.0` had 2,202 polygons after filtering and
edge splitting, then 1,984 after simplification (218 fewer, 9.9%). It kept
four connected regions and passed candidate topology validation. This
resolved load order selected an exterior CELL override, so its counts differ
from the earlier vanilla run above. A synthetic flat grid of 72 source
triangles produces fewer than 36 candidate polygons with identical area;
peak, opening, stacked-level, and ripple fixtures check the guarded cases.

Choose **polygon-based construction** for this milestone. Filter original
terrain/collision triangles, canonicalize near-coincident placed-mesh seams
within 0.25 world units, split partial edges at compatible T junctions,
form regions through traversable shared edges, inset exposed edges by agent
radius, split partial edges created by that inset, then triangulate the clipped
polygons. This retains vertical layers
and a direct source-triangle audit trail; seam vertices may move by at most
0.25 world units. After region filtering, interior vertices of convex,
connected fans are removed when all contributing source triangles and the
replacement triangles stay within half a profile-bounded height tolerance
of the same fitted plane. The total vertical difference is at most the
smaller of 16 world units or the profile's step height. The
replacement triangles must still satisfy the profile slope, clearance, and
wall-obstruction checks. Fan boundaries, including obstacle openings, cell
borders, and distinct height levels, are retained exactly. This reduces
surface detail without relaxing
source eligibility or create links. Runtime
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
`polygons[].source_triangles` and `regions[].source_triangles` join to
`source_triangles[].input_index`; each `geometry_source` joins to
`geometry_sources[]`. `polygons[].source_triangle` remains a primary source
index for simple joins; a simplified polygon can cover several input triangles,
all of which appear in its `source_triangles` list. It does not assign a
Bethesda FormID or serialize NVNM.
The candidate is reproducible for identical scene geometry, source order, and
profile. Triangle order from the resolved load order is part of the input.
Placed, enabled DOOR references in the selected worldspace are identified as
exits before filtering regions. A door anchors only a nearby polygon on the
same vertical level. A region also survives when it reaches any exterior cell
grid border, including a triangle that crosses the border without an edge
lying exactly on it. Substantial collision regions also survive when their
walkable area is at least the larger of 16,384 square world units or 64 squared
agent radii. They carry a warning because a route to a door or border has not
been established. Smaller isolated regions are removed and counted as
`rejected_unreachable` or `rejected_small_region`. If there is no identified
exit or reachable border, the largest connected region is retained for review
and a warning is emitted. JSON records exits, their matched region, and each
region's border/exit evidence. These are candidate reachability checks, not
Bethesda door portal links.

Only LAND and supported Havok collision are eligible by default. Render
fallback remains diagnostic evidence and is never silently promoted to
walkable ground or an authoritative obstruction. Unsupported collision,
missing meshes, and incomplete LAND can leave real walkable areas absent.
Clearance uses vertical triangle intersections at triangle vertices, edge
midpoints, and centroid. A steep wall from another collision placement that
traverses a floor triangle from side to side is an obstruction. Rails and
posts in the same placed mesh do not discard the entire floor triangle;
local obstacle trimming is still incomplete. These discrete tests cannot prove
continuous capsule clearance or door semantics. The agent-radius inset
addresses exposed region edges, but complex corners and very thin triangles
may be removed. Exterior selection without neighbors uses triangle centroids
within the selected cell, and the border policy preserves its outer edges.
With `--neighboring-cell-radius`, the generator includes extracted neighbor
cells and welds matching border edges into the same region when their height
difference fits the step profile. Candidate surfaces are clipped to the outer
bounds of the extracted exterior area, including triangles that merely cross
those bounds; a missing neighbor never becomes an invented link.
Independent surfaces on different levels remain separate unless a traversable
step or ramp provides a shared edge; matching XY coordinates alone do not join
stacked floors. Supported collision missing from a walkway can still leave it
uncovered, and unsupported crossings require inspection rather than a guessed
bridge.
