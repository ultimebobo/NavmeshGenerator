# Candidate surface algorithm spike (milestone 8)

## Current generator

The application now uses Recast Navigation for candidate generation. The
polygon-based measurements and decisions below document the previous spike and
remain useful as comparison evidence; they do not describe the active run path.
The Recast adapter consumes supported terrain and collision, clips input to a supported halo around the
selected exterior CELL, converts Skyrim world Z-up to Recast Y-up, and runs Recast's voxel rasterization, walkability
filters, radius erosion, configurable region partitioning, contour construction,
polygon mesh construction, and floor-height detail sampling. Vertical collision
faces remain rasterization input as obstructions, even though their horizontal
projection has no area.
The Skyrim extraction layer tags assets in the landscape rock directory as
obstacle-only collision, normalizing case, separators and the optional meshes
prefix. The neutral generator consumes this tag without interpreting model
names. Every tagged face contributes a solid raster span but cannot become
a walkable floor, including through coincident terrain or low-obstacle promotion.
The policy leaves architectural stone and terrain eligible for navigation.
Candidate source evidence records the obstacle tag, and obstacle triangles are
counted in `rejected_obstruction`.

Operable DOOR references do not supply permanent collision or render geometry:
their closed model pose would obstruct an intended portal. Extraction records
them as excluded while preserving reference metadata for exit matching. Static
door frames, thresholds and adjoining floors remain eligible geometry.

Before ledge rejection, standing-clearance filtering and radius erosion, the
adapter repairs narrow unsupported raster seams between collision pieces. Both
endpoints must be supported standing floors within the profile's climb limit.
The unsupported width is bounded by the agent radius at the effective voxel
resolution. Inserted columns use the higher endpoint's floor and require clear
standing volume; nonwalkable solids reaching the crossing height block repair.
Proposals inspect the unchanged heightfield, so repairs cannot chain across a
wider void. Ordinary walkability filters still apply to the completed field.
Candidate warnings report repaired columns. This automatic stage uses the same
movement and voxel settings in single-cell and batch generation.

Region partitioning defaults to watershed; the
CLI and Windows UI also expose monotone and layer partitioning. The selected
strategy is recorded in `candidate-navm.json`. It exports Recast height-detail triangles for the neutral
candidate JSON/OBJ and combined GLB. Horizontal voxel size grows with the
extracted area to limit grid dimensions. Finer voxels resolve narrow stair treads,
while larger extracted areas can lose them. The navigation profile supplies
traversable climb and other movement limits. The shared CLI/UI advanced settings
configure requested horizontal and vertical voxel sizes, contour simplification
error, optional maximum edge length and merge-area multiplier. Contour error is
an upper bound: if simplification collapses a surviving voxel region or produces
inverted or nonconvex coarse polygons, construction retries the complete contour
set at finer tolerance until every retained region has a consistent mesh.
Coarse polygons must have the expected Recast winding and cannot reuse a directed
edge on the same floor. Neighboring regions share one simplification pass, preserving their
common interfaces without overlapping coarse and fine boundaries.
Recast can collapse a watershed region enclosed by one other region even at zero
contour error. If refinement cannot produce consistent polygons for every retained region, the adapter
repartitions the already-retained spans with layer partitioning and rebuilds the
complete contour set. Recovery preserves the initial walkability and island
filtering decisions, and reports its use in candidate warnings; the recorded
partitioning setting remains the requested strategy. Unrepresentable recovered
regions still fail generation.
Supported geometry must fit Recast's packed vertical span range at the requested
vertical voxel size. The adapter rejects unrepresentable extents before
rasterization can clamp distant solids into artificial floors. Cached batch
candidates undergo the same scene-height validation without rerunning Recast.
Missing retained-region contours remain failures. Batch generation logs cell-local
failures and continues without replacing those cells' authored geometry.
Recast merges convex contour polygons before height sampling and triangulates the
result for the neutral mesh. Disconnected
regions below the profile's minimum area are removed. Watershed and monotone
partitioning can merge small adjacent regions; layer partitioning does not use
the merge threshold. Recast converts the area thresholds to horizontal voxel
cells. This filters small orphan surfaces before polygon construction.
The compact heightfield stays alive through detail construction. Sampling
spacing scales with horizontal voxel size. Height approximation error is bounded
by the larger of the movement profile's climb and the vertical voxel size.
This lets straight stairs form compact ramp surfaces without reproducing every
riser; the voxel filters enforce climb, clearance and obstacle exclusion before
simplification. Turns, landings and nonplanar terrain add height samples where
needed. Detail vertices use world coordinates and patch-local triangle indices;
conversion restores Skyrim axes, removes Recast's additional height offset, and joins shared
patch vertices before adjacency and reachability filtering. Landings and stair
interiors therefore retain floor evidence beyond the contour corners. Long
contour edges and detail sampling limits still require scene inspection.

Automated checks verify coverage at matching floor heights, shared-edge
reachability, clearance rejection and separation of stacked levels. Geometry,
parameters and expectations live in the test builders; exported scenes and
reports provide inspection evidence. Coverage and route connectivity are checked
separately from topology, since a valid mesh can still omit intended routes.
Coarse voxels can erase valid treads before detail sampling; climb is rounded
down to whole vertical voxels. Candidate warnings report quantization and
adaptive horizontal resolution instead of silently presenting requested values
as the effective constraints.

After polygon construction and CELL clipping, single-cell generation retains only
shared-edge components reaching a matched door or a boundary edge on the selected
exterior CELL. Door matching selects the closest compatible triangle within the
horizontal and vertical reach derived from the movement profile and voxel sizes.
Large roofs and stone tops are discarded when they have no such path, regardless
of their area. A surface merely near a border, or touching it at one vertex, does
not qualify. Interiors without a matched door produce empty candidates. Filtering
preserves source evidence and remaps polygon, neighbor, region, door and vertex
indices; removed triangles are counted in `rejected_unreachable`. Exterior input
includes a halo sized for radius erosion and neighboring voxels; the resulting
mesh is clipped to the selected CELL after Recast construction. This preserves
walkable seams where supported terrain or collision continues across the border.

With an adjacent NAVM, stitching matches complete authored boundary edges at
compatible heights. Selected-cell authored external links also define required
crossings, resolved through neighboring return entries or a geometric edge match.
Both single-cell and batch linking repair complete authored partitions spanning
multiple generated fans, including when there is no selected-cell authored NAVM.
The neighboring authored floor bounds repair depth in that case. Matching repeats
after successful repairs so endpoint alignment can expose earlier partitions.
New authored height bends can use supported interior samples with step-bounded
height displacement. If continuous floor triangulation cannot preserve the slope
envelope, a narrow border strip can retain authored heights and join the supporting
floor at a climb-compatible internal edge. The floor keeps its sampled heights;
both levels retain their slopes. Cavity growth preserves these step interfaces.
When direct subdivision cannot retain a crossing, stitching replaces a connected
boundary cavity with a triangulation constrained by the complete neighboring edge
and the untouched interior rim. Its repair depth follows the authored floor
triangle, with height matching accounting for authored endpoint drift. Dynamic
programming considers alternative diagonals and bounds replacement slopes by the
configured limit or the existing generated cavity's slope envelope, whichever is
greater; required crossings also retain the selected authored floor's envelope.
Terminal chains can end short of a portal or turn into the cell: endpoint projection
and the authored floor's repair bounds select the complete cavity.
Near-coincident endpoints align through their incident fans before cavity
construction. Removed seam height detail can become a bounded interior sample
when rim-only triangulation cannot preserve the floor's slope envelope. Cavity
growth includes incident triangles reaching the repair band through a vertex;
their centroids need not be inside it. Source contributors, region membership, door anchors, and other portals
are remapped transactionally. Missing required crossings fail final validation
and prevent that candidate's plugin export, including when no candidate floor survives.
The shared Cell runner supplies generation input to identify authored crossings
onto excluded obstacle collision. When no compatible generated boundary floor
survives, positive upward collision support at both interior anchor samples closes
that crossing with a warning. Render fallback, untagged collision and missing source
evidence cannot waive a required crossing. Plugin export removes its incoming links.
At shared CELL corners, separate authored endpoint heights join through internal
step edges. Strip samples move into supported floor within the strip inset and
share a slope-bounded plane; the interior floor retains its sampled heights.
Single-cell generation fails export on invalid topology. Plugin and load-order
batches skip cell-local generation/topology failures and preserve those cells as
authored neighbors. Complete successful candidates are reconciled together;
invalid global seam topology stops export.
Compatible collinear generated subdivisions are coalesced
by retriangulating their incident fans while preserving the interior rim and
horizontal footprint. Mixed regions or flags, existing portal triangles, invalid
topology, and nonwalkable replacements prevent coalescing.
A containing generated edge can be split into smaller
segments to match authored partitions, retaining its remaining triangles and
updating region, provenance, and door joins. Near-coincident endpoints are aligned
to avoid collapsed connector triangles. Subdivision commits only when every
existing portal survives; incompatible proposals leave the candidate unchanged.
Extensions obey distance, step, slope,
and welding constraints. Inward authored offsets trim generated fans; outward
offsets add connectors. Matched endpoints preserve the exact authored edge,
including bounded deviations from the nominal CELL border. Other generated
vertices stay inside the CELL. Terminal seam endpoints extend through their
complete incident fans when movement and topology permit, preserving shared
interior vertices. Short corner edges select their nearest CELL side, and one
triangle can consume portals on multiple sides. Unmatched CELL seam wedges
retract through interior fans, preserving their floor plane and shared interior
edges. Unpaired border vertices move inward with the agent footprint and weld
tolerance bounding the displacement. Matched endpoints remain pinned. Each
nonplanar fan can try individual floor-centroid directions when its average would
increase the slope envelope. Fan slope comparisons allow floating-point roundoff.
Every remaining border vertex must equal a neighboring portal endpoint.
Final shared-edge components survive only with a real neighboring portal
or matched door. Region, source, door, vertex and portal indices are compacted;
contours are rebuilt from the final boundaries. Stitch validation requires exact
reversed endpoint equality, unique consuming edges and complete coverage of every
remaining CELL seam. No portal is invented across unsupported gaps.
Entrance positions appear as orange markers, including unmatched entrances.
See the [glossary](glossary.md) for the Creation Kit border-bar interpretation
and the distinction between neighboring geometry and saved return links.

Plugin and load-order scopes retain all surviving Recast floor components, including
isolated interiors and exterior borders awaiting generated neighbors. Only untouched
cells supply authored border edges; batch stitching provisionally preserves unpaired seams
and unanchored components. The complete generated set supplies common
seam partitions, compatible shared heights and reciprocal generated triangle indices.
Shared corner heights are planned together; established portal endpoints remain pinned.
Linking repeats until corner welding exposes no new compatible shared intervals.
Different authored corner heights use separate vertices and climb-compatible internal
edges. Thin float-coordinate strips use boundary triangulation when an interior
centroid cannot be represented.
Triangle subdivision preserves regions, flags, source joins and door anchors. Exact
XY edge matching and mutual nearest-height adjacency distinguish thin edges and
stacked floors; contour tracing follows incident triangle adjacency.

`--skip-existing-navmesh` protects cells with winning NAVM records. Other selected
cells provisionally retain every walkable component and link to each other's generated mesh,
including when the plugin starts without navmesh. Valid empty targets remain completed
replacements when source geometry contains no supported walkable floor.

Complete-batch reachability then joins shared-edge components through reciprocal
generated portals. Matched doors and untouched authored neighbors anchor accessible
networks. Each contiguous selected exterior area and each interior also retains
its largest network by horizontal area. Remaining isolated roofs and other floors
are removed regardless of their area or the number of CELLs they span. Open borders,
generated portals alone and vertex-only contact do not anchor islands. Separate
selected areas and worldspaces retain independent primary networks. Filtering runs
for fresh and cached candidates, remapping geometry, evidence, regions, doors and
both generated portal destinations before rebuilding contours and validating topology.
Removed triangles contribute to `rejected_unreachable`.

Recast height-detail patches are checked before the compact heightfield is released.
An overlapping patch is retriangulated with its original boundary and floor samples;
valid patches keep their triangulation. Exact sampled endpoints remain shared with
neighboring patches. Repair does not reuse authored geometry. The repaired detail
edges are checked again; unresolved overlaps fail generation. Coarse partition
validation precedes detail sampling because normalizing an inverted coarse face
can put it on the same side of a shared edge as its neighbor. Those coplanar
triangles overlap in horizontal area even when a scene viewer displays a continuous floor.

Source-triangle provenance is recovered by the closest source height at each
generated triangle's XY centroid. This is an approximate audit join after
voxelization. Recast's smoothing and erosion can remove small supported areas;
output is still an inspection candidate and is not Bethesda NAVM serialization.
Agent dimensions, clearance, slope, step height, radius and minimum region area
feed the Recast build through the shared navigation profile. Voxel/contour
controls come from `RecastSettings` and are retained in candidate JSON and cache
fingerprints. Weld tolerance controls later geometry/border matching; profile
contour tolerance and cell-border policy remain export evidence and do not
control voxel construction. See [advanced generation settings](operator-workflow.md#advanced-generation-settings).

A Windows UI terrain-only run on a local exterior CELL on 2026-09-29 produced
516 Recast candidate triangles in two connected regions. Its candidate topology
validated and `scene.glb` contained a `Candidate NAVM` object with 516 triangles.
The named CELL is reproduction data rather than a special-case rule.

## Evidence and decision

The shared application paths classify the final candidate after authored-border
reshaping. `TagCandidateTriangles` compares world-space centroids against the
effective water plane and closest overlapping authored floor; water and preferred
path bits are independent. Unmarked authored surfaces constrain nearest-height
matching, preventing lower route markings from leaking onto upper surfaces.
Classification changes flags only and is optional. It is approximate at boundaries
and does not infer preference where authored route intent is unavailable.


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
Placed, enabled DOOR references are matched to nearby generated polygons at
compatible heights. Every retained component must reach a matched entrance or a
CELL boundary edge. For exterior targets with
a resolved load order, `border_links` records neighboring NAVM targets for complete
matched edges. The guarded writer serializes matched door and border portals.

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
