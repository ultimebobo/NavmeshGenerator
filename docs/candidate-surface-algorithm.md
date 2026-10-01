# Candidate surface algorithm spike (milestone 8)

## Current generator

The application now uses Recast Navigation for candidate generation. The
polygon-based measurements and decisions below document the previous spike and
remain useful as comparison evidence; they do not describe the active run path.
The Recast adapter consumes supported terrain and collision, clips it to the
selected exterior CELL bounds, converts Skyrim world Z-up to Recast Y-up, and runs Recast's voxel rasterization, walkability
filters, radius erosion, configurable region partitioning, contour construction,
and polygon mesh construction. Region partitioning defaults to watershed; the
CLI and Windows UI also expose monotone and layer partitioning. The selected
strategy is recorded in `candidate-navm.json`. It triangulates Recast polygons for the neutral
candidate JSON/OBJ and combined GLB. Horizontal voxel size grows with the
extracted area to limit grid dimensions. Finer voxels resolve narrow stair treads,
while larger extracted areas can lose them. The human profile supplies the
traversable climb and other movement limits. Contour edges are not subdivided
by a maximum length. Contours allow two horizontal voxels of simplification
error, and Recast emits triangles directly for the neutral mesh. Disconnected
regions below the profile's minimum area are removed. Watershed and monotone
partitioning can merge small adjacent regions; layer partitioning does not use
the merge threshold. Recast converts the area thresholds to horizontal voxel
cells. This filters small orphan surfaces before polygon construction.
Long contour edges can still produce thin triangles on irregular boundaries,
so scene exports need inspection. On a synthetic jagged corridor, these settings
produced 39 triangles instead of 50 with the 1.3-voxel, six-vertex-polygon
configuration; the worst normalized triangle quality rose from 0.16 to 0.42.
This comparison is a geometry fixture, not a claim about every game scene.

A synthetic 16-step straight staircase with 12-unit
treads and 24-unit rises produces one connected region and two output triangles
from 32 source triangles. With a 12-voxel maximum contour edge, it produced ten
output triangles. A flat square of two source triangles produces two output
triangles instead of 62. Both fixtures pass candidate topology validation.

With the old climb, Recast split the isolated stair collision into five
regions. The 28-unit setting joined it into one region, including when the test
uses the wider extracted scene bounds. The slope and radius settings were
unchanged: the walkable tread faces are flat and the normal radius fits. A
full CLI rerun of Riverwood03 with one neighboring-cell ring found three
placements of that stair model. With the historical 18-unit climb, each placement's candidate polygons
appeared in two connected regions. With the tested 28-unit climb, polygons
from all three placements joined one connected region. These are local game-data
measurements; the source assets are not committed.

After polygon construction, the active generator keeps only connected components
that reach an enabled placed DOOR or the border of the target exterior
CELL. Door matching checks the closest point on a polygon at a compatible
height and records the matched candidate triangle. Border matching allows for
the walkable inset created by agent-radius erosion. When a resolved load order
provides an adjacent NAVM, the selected-cell candidate joins a near-coincident
full boundary edge to the neighbor's border edge at a compatible height. The
joined triangles and reciprocal target appear in `border_links`. Border proximity
alone does not anchor a region. Authored edges may deviate slightly from the
nominal CELL boundary within the shared border tolerance.
Stitching preserves their exact endpoints, including bounded portal extensions
beyond the selected CELL, so reciprocal edges remain coincident. Other candidate
vertices stay inside the selected CELL. Regions without a matched border portal
or door are removed after stitching. Unanchored components are removed from the
candidate mesh, JSON, OBJ, and GLB; the statistics count their rejected polygons. An interior scene without
a reachable door and an exterior scene without a reachable door or border yield
an empty candidate and a warning. Entrance positions are shown as orange markers
in the candidate GLB layer, including entrances that did not match a polygon.

Source-triangle provenance is recovered by the closest source height at each
generated triangle's XY centroid. This is an approximate audit join after
voxelization. Recast's smoothing and erosion can remove small supported areas;
output is still an inspection candidate and is not Bethesda NAVM serialization.
The fixed settings for agent dimensions, clearance, slope, step height, radius, and
minimum region area feed the Recast build. Weld tolerance, contour tolerance,
and cell-border policy remain in the exported settings for output
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
an 18-unit climb, completed in 8.45 seconds on this development machine. The
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
one neighboring-cell ring and the historical 18-unit climb, the candidate
had 83 walkway polygons in one region; all 18 existing walkway NAVM polygon centroids lay on
those candidate polygons at the same height. Candidate topology was valid.
This checks coverage of one known structure, not navigation quality across all
bridges or stairs. The local asset cache supplied 755 models; 222 referenced
models remained unavailable in that run.

A 2026-09-28 terrain-only rerun of the local exterior benchmark with the
interior simplifier and the historical 18-unit climb had 2,202 polygons
after filtering and edge splitting, then 1,984 after simplification (218 fewer, 9.9%). It kept
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
replacement triangles stay within half a step-height-bounded height tolerance
of the same fitted plane. The total vertical difference is at most the
smaller of 16 world units or the fixed step height. The
replacement triangles must still satisfy the fixed slope, clearance, and
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
inspection views. The JSON includes the fixed human settings and all parameter
values, source-triangle provenance (including LAND sample coordinates),
regions, contours, polygon neighbors, and topology findings.
`polygons[].source_triangles` and `regions[].source_triangles` join to
`source_triangles[].input_index`; each `geometry_source` joins to
`geometry_sources[]`. `polygons[].source_triangle` remains a primary source
index for simple joins; a simplified polygon can cover several input triangles,
all of which appear in its `source_triangles` list. It does not assign a
Bethesda FormID or serialize NVNM.
The candidate is reproducible for identical scene geometry, source order, and
fixed settings. Triangle order from the resolved load order is part of the input.
Placed, enabled DOOR references in the selected worldspace are identified as
exits before filtering regions. A door anchors only a nearby polygon on the
same vertical level. A region also survives when it reaches the extracted
exterior cell grid border, including a triangle that crosses the border without
an edge lying exactly on it. Other regions are removed and counted as
`rejected_unreachable`. If no region reaches an entrance or border, the mesh is
empty and a warning is emitted. JSON records exits, their matched region, and
their matched polygon, along with each surviving region's border/exit evidence.
For a selected exterior cell with a resolved load order, `border_links` records
the neighboring NAVM target of each fully matched edge. These are candidate
reachability checks; the guarded writer serializes matched NAVM door and border
portals separately.

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

## Batch target boundaries

Plugin and load-order scopes reuse this generator per target CELL, with
neighboring source geometry and persistent placements provided by the impact
index. The candidate bounds remain the target CELL bounds. Generated borders
must be reconciled to reciprocal generated triangle identities before combined
plugin serialization. See [batch rebuilding](batch-rebuilding.md).
