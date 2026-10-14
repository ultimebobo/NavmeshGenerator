# Architecture

This project is intentionally split into processing stages so the analysis logic stays independent from Skyrim runtime code.

The build targets the Windows application, neutral core, and tests. Runtime game APIs are outside the processing pipeline.

The application-owned `app/cell_scene` accumulator joins Cell-mode inspection and
batch extraction into an optional complete-selection scene. It deduplicates by
source identity, original triangle and exact world-space positions, rebases vertex
and provenance indices, and retains authored NAVMs and exits by Form ID. The neutral
scene exporter accepts borrowed finalized candidates with owning CELL identities,
so generated seam destinations resolve without allocating plugin NAVM identities.
This keeps extraction/orchestration in the app and serialization in the core.
Requested scene storage is owned for the run, independently of cache retention and
generation admission budgets. Scene and sidecar failures stop plugin publishing.

The `core/io/file_path` helper prepares native filesystem paths without reading
game data. Candidate cache publication, reads and audit pinning use it at their
I/O boundaries. The app-owned gzip opener shares that preparation between
private candidate evidence and streaming compressed JSON exports, preserving
cache identities while supporting long Windows local and UNC paths.

## 1. Plugin parsing

The parser layer is responsible for reading a Bethesda plugin file directly from disk.

- Input: `.esm`, `.esp`, or `.esl` file
- Responsibility: identify records, skip unsupported record types, and surface the subset needed for cell/navmesh inspection
- Output: a minimal record stream or normalized plugin index

The current POC keeps this intentionally narrow and reports unsupported record layouts explicitly rather than pretending the parser understands the whole format.

## 2. Skyrim extraction

Model extraction owns the furniture and fire navigation policies. Supported
furniture collision is obstacle-only. Recognized fire effects derive closed
exclusion bounds from transformed fuel or flame meshes; decorative billboard
extents are ignored. The neutral scene identifies this geometry as
`NavigationExclusion`, retains its asset/reference provenance and obstacle tag,
and exports it in the collision inspection layer. Recast consumes the geometry
without interpreting Skyrim record classes or asset names. Collision-impact
selection compares exclusion geometry as well as supported Havok triangles,
and the candidate pipeline revision invalidates results built with other policies.

Exterior terrain extraction follows explicit winning WRLD parent-LAND inheritance
when the child CELL has no owned LAND. It uses matching coordinates and retains
actual LAND provenance; child records take precedence, and missing, deleted or
cyclic parents supply no floor. References and authored NAVMs retain their owning
worldspace identities.

The Skyrim-owned `parser/base_object_swapper` module discovers effective loose
BOS configs, resolves identities and precedence, and applies runtime placement
alternatives after plugin resolution and before spatial indexing. Encoded record
histories remain immutable. Collision impact selection compares original and
effective placements, including configurations without plugin edits. The app
owns snapshot reporting and checks config coherence before publishing output.
See [Base Object Swapper inputs](base-object-swapper.md) for the compatibility contract.
The record index includes BOS replacement classes independently of extraction
eligibility. Ordinary placements of those additional classes remain excluded;
effective runtime placements can supply their models without enabling unrelated
inventory clutter in the scene.

Uncertain runtime outcomes cross the neutral boundary as `PlacementEnvelope`.
Model extraction encloses supported collision in filled world bounds on
`GeometrySource::solidBounds`. The core rasterizer fills those obstruction
volumes, and analysis excludes their surfaces from floor evidence. No BOS syntax
or Skyrim runtime policy enters the neutral algorithm layer.

Winning exterior CELL water flags and heights resolve into `Cell::waterHeight`.
Default heights use winning WRLD water data and master-aware parent-world
inheritance; invalid, cyclic or unresolved data supplies no water plane. Both engine default-height
sentinel forms request world water rather than becoming literal elevations.
The range interpretation follows [CommonLibSSE-NG](https://github.com/CharmedBaryon/CommonLibSSE-NG/blob/main/src/RE/T/TESObjectCELL.cpp).
The neutral `core/navmesh/triangle_tagging` module classifies final generated
triangles using that plane and winning authored floor markings. App cell and
batch paths invoke it after border reshaping; cache inputs include all tagging
dependencies. The serializer retains flags and inspection exports expose them.


The extraction layer translates raw data into a neutral project model.

- Input: plugin record stream
- Responsibility: map records into cell, reference, and navmesh structures
- Output: cell metadata, worldspace/cell coordinates, references, and NAVM geometry

This layer should remain the only place that knows about Bethesda record conventions.

## 3. Neutral geometry model

The model layer does not depend on CommonLibSSE or runtime types.

- Types: `Vec3`, `AABB`, `Triangle`, `Mesh`, `Reference`, `Cell`, `NavMesh`, `NavPolygon`
- Responsibility: hold clean geometry and navmesh data in a portable representation
- Output: JSON and OBJ exports for inspection

## 4. Analysis and validation

The analysis layer is intentionally small but useful.

- Vertex/polygon counts
- Connected components
- Degenerate polygon detection
- Bounding box computation
- Area statistics
- Validation findings for out-of-range indices

The goal is to make a real downstream generation pass possible without hard-coding Skyrim-specific assumptions into the algorithmic layer.

## 5. Candidate generation

`core/navmesh/recast_contours` owns complete contour-set refinement and verifies
coverage of retained voxel regions and coarse polygon winding, convexity and directed
edge consistency before height sampling. When watershed contours or polygons remain invalid,
it repartitions the retained floor with layer partitioning and reports recovery.
It preserves initial island filtering and rebuilds all shared interfaces together.
The Recast adapter rejects supported geometry outside its packed vertical span
range before rasterization, and applies the same check to cached batch scenes.
Batch workers distinguish cell-local generation failures from infrastructure
failures. Skipped generation targets retain authored NAVM and rejoin border
matching as untouched neighbors. Recovery includes existing authored portal
destinations from the resolved snapshot in its neighbor validation. Missing authored
NAVM adds no constraint and leaves open borders intact; global refinement and writer checks remain fatal.

`core::CandidateGenerator` is the replaceable boundary between extracted scene
geometry and a neutral candidate navmesh. The shared CLI/Windows run path uses
`RecastCandidateGenerator` from the Recast Navigation submodule. It accepts only
terrain and supported collision triangles, converts Skyrim Z-up world positions
to Recast Y-up coordinates, rasterizes a supported exterior halo around the target CELL,
repairs radius-bounded unsupported seams between climb-compatible standing floors,
filters walkable spans, erodes them by
agent radius, then builds regions, contours, a polygon mesh and floor-height
detail triangles. Authoritative vertical collision faces remain rasterization
input as obstructions. Skyrim extraction tags landscape rock assets as obstacle-only
sources; the neutral rasterizer retains their solids but excludes their tops even
when low-obstacle promotion or terrain overlap could make them walkable. Raster
seam repair checks solid obstruction and clearance against the unchanged field
before proposing crossings, preventing repairs from chaining across wider voids.
Skyrim extraction excludes operable door models while retaining door metadata
and separate static architectural geometry. Height
detail samples the surviving compact heightfield with movement-bounded error;
convex contour polygons merge before sampling. Contour construction refines the
requested error when a retained voxel region would collapse. Every region uses
the same refinement pass so shared boundaries partition the compact spans without overlap. Final output remains triangles;
shared detail-patch vertices are joined before neutral adjacency is built.
`core/navmesh/detail_triangulation` repairs overlapping sampled patches through
boundary ear clipping and incremental sample insertion. It preserves the original
hull and sampled floor heights; the Recast adapter owns patch decoding and replacement.
`core/navmesh/surface_simplification` removes coplanar interior samples and
straight exposed boundary samples from the clipped, unlinked mesh. It retains
the complete floor footprint and rim, pins CELL seam samples, respects contour
edge limits, and refuses ambiguous topology or mixed flags. Immutable original
triangle contributors preserve exact floor heights at this generator stage.
The Recast adapter runs cleanup before assigning source, region, door or portal indices. Bounded passes
of disjoint fan replacements keep geometry editing separate from Skyrim I/O and
connection remapping. CLI and desktop calls share the same generator, and the
candidate cache compatibility revision includes this stage.
The same module performs transactional candidate cleanup after authored border
repair: it prunes exposed triangles and trims outward fans inside the immutable
boundary band derived from the agent footprint. Convex capsule coverage proves
the entire removed footprint stays in that band; shared-edge route guards retain
passages, and cumulative area loss is limited per original connected floor.
Fan retriangulation bounds height error against immutable triangle contributors
by the vertical voxel size; replacement slopes preserve the original envelope
or movement limit. The exact generator pass leaves this approximation budget
unspent. It preserves portal and door triangles,
remaps evidence and region membership, and rebuilds topology before publication.
Single-cell orchestration invokes it after island filtering; batch workers invoke
it before cache publication and generated seam reconciliation. Generated-to-generated
triangle identities are never changed independently by this cell-local pass.
Detail triangles are clipped to the target CELL in the project's neutral model for JSON, OBJ, and the
`Candidate NAVM` GLB layer. Eligible generated geometry can enter the guarded
plugin writer. The application then matches selected exterior boundary edges to
adjacent NAVM edges in the resolved load order and records reciprocal targets.
The shared authored-border tolerance allows small deviations from nominal CELL
bounds only at matched portal endpoints. Both generation and serialization
preserve those endpoints and keep other generated vertices inside the target.
`core/navmesh/authored_border_edits` owns transactional validation and local
height relaxation of bounded authored-neighbor fan changes. Border matching
plans these only when a required crossing remains missing and nearby authoritative
floor corroborates its adjusted endpoints. Bounded offsets must improve floor
agreement when they exceed direct climb matching. Input geometry stays immutable; candidates
carry original-position checked edits through private caches and inspection JSON.
The writer merges plans, protects unrelated portals and rebuilds edited neighbor
spatial grids while preserving triangle, vertex, cover and door identities.

Single-cell and batch generation supply the selected cell's authored NAVMs to stitching.
Resolved reciprocal edges identify required exterior crossings. A constrained
boundary cavity repairs crossings that direct subdivision cannot retain, keeping
its interior rim, other portals, source joins, and door anchors. Repair depth comes
from the authored floor triangle; height matching accounts for authored endpoint
drift. Dynamic programming chooses a triangulation within the greater of the
configured walking slope and the existing cavity's generated slope envelope.
Incident-fan endpoint alignment uses the same slope-envelope rule and pins saved
portal endpoints. Cavity preparation aligns near-coincident endpoints and admits
incident floor triangles whose vertices reach the authored repair band. When the
unchanged rim needs seam height detail, a bounded interior sample supports the
triangulation while preserving the complete portal. Unpaired nonplanar fans try
individual floor-centroid directions, with a roundoff allowance in slope comparisons.
Missing required crossings invalidate the final candidate;
seam retraction cannot silently remove them.
Required cavities also accommodate the selected authored floor's slope envelope.
Endpoint projection handles terminal chains and boundaries turning into the cell.
Shared corner height offsets use separate vertices and slope-bounded strips joined
to the unchanged floor through climb-compatible internal edges.
The Cell runner passes the generation scene to neutral border stitching. Positive
excluded-collision support at the authored interior anchor can close a crossing
with a warning when no compatible boundary floor survives. Missing or unsupported
evidence cannot waive a crossing. The writer clears incoming links to closed edges.
Single-cell generation provisionally retains every surviving walkable component.
Shared reachability filtering follows CELL clipping, door matching and authored
border stitching, retaining the largest connected floor plus components reaching
a matched door or real neighboring portal. Uncovered cells and interiors without
doors can retain a floor. Filtering preserves source joins while compacting
polygon, neighbor, region, door and vertex indices; open borders alone do not
provide access anchors.
Authored partition repair runs before deferred batch linking as well as single-cell
filtering. It repeats direct subdivision and connected cavity repairs, using
neighboring floor depth when no selected-cell crossing supplies bounds. Supported
interior samples accommodate authored height bends. Continuous floor repair
precedes border strips whose authored heights connect to the unchanged supporting
floor through climb-compatible interior edges. The neutral candidate stitcher
owns these edits; neighbor NAVM geometry and serialization remain separate.
Stitching can subdivide a containing
generated boundary edge to match smaller authored border edges without removing
the remaining geometry. Collinear generated subdivisions can be coalesced through
validated boundary-fan retriangulation, preserving the interior rim and remapping
source, region, contour, door, and portal indices. Inward authored offsets trim
generated fans; outward offsets add connectors. Connections retain full authored endpoints and obey
distance, height, slope, and welding constraints.
Short corner seams select their nearest CELL side. Terminal endpoints can extend
through their entire incident fans, preserving shared interior vertices and
existing portal endpoints. Unmatched seam wedges retract through interior fans
without changing shared interior edges. Final adjacency determines the surviving
components. Unpaired border vertices move into their incident fans, with inward
movement bounded by the agent footprint and weld tolerance; matched endpoints
stay pinned. Every remaining border vertex equals a neighboring portal endpoint.
Region, door, source and contour evidence is rebuilt or remapped.
Stitch validation requires exact reversed neighboring endpoints, unique consuming
edges, and a portal on every remaining CELL seam. Candidate cache fingerprints
include the stitching implementation revision.
The [glossary](glossary.md) explains these terms and why single-cell geometry
replacement can require neighboring NAVM connection overrides.

The previous polygon-based `GenerateCandidate` function remains for historical
fixture comparisons. Application runs use the Recast implementation.

## 6. NAVM override serialization

The plugin writer is separate from candidate generation. It reuses the winning
NAVM source plugin's master order and group hierarchy, writes an override for
every existing NAVM in the selected cell, and verifies each with the direct
reader. Generated geometry occupies the largest original NAVM; the others get
empty geometry. Parent CELL and worldspace records remain in the load order.
New NAVMs serialize a non-null PathingCell type tag so Creation Kit consumes the
location fields before the geometry arrays; read-back checks each new identity's tag.
Retained source group ancestors are rebased from the winning plugin's master
indices. New child groups use the resolved CELL identity converted directly to
output-master indices, independently of its load-order index.
The writer always emits an ESP and sets its ESL flag when the override-only
records and master table fit the light format. Matched door triangles are
serialized in the generated NAVM. Matched exterior borders add external portals
to the generated NAVM and reciprocal portals to adjacent NAVM overrides. Authored
portals into replaced triangle spaces are removed from every incoming NAVM,
including records without matched borders. Retained external indices are remapped,
then matched borders receive fresh reciprocal portals to generated triangles.
Read-back checks every consuming portal and its emitted destination's triangle
range. Adjacent geometry remains unchanged. Other authored links, cover
data, NAVI, and REFR XNDP references remain outside this
writer's supported remapping.

Cell and Plugin scopes can instead copy the selected source with generated
navigation. `skyrim/parser/plugin_copy` validates the complete raw record/group
envelope, preserves encoded unrelated records and TES4 metadata, merges NAVMs
by identity and group placement, and updates group sizes, HEDR accounting, and
ONAM override registration for master-flagged copies or existing ONAM tables.
It inspects every source record type for allocation safety without expanding
the extraction reader's index. The writer rebases generated references through
the unchanged source master order and implicit self slot. Dependencies outside
that table are rejected, and source full/light flags constrain new identities.
The resulting copy keeps its original filename and resources and replaces the
source when installed. NAVI and other unsupported connection metadata remain
subject to the writer's existing limitations.

## Design boundary

The critical architecture boundary is:

Skyrim plugin data
  -> extraction
  -> neutral model
  -> analysis and replaceable candidate generator (currently Recast)
  -> guarded NAVM override serialization

This keeps the codebase testable, portable, and independent from a live Skyrim process.

## Source responsibilities and readability

`tools/render_media.py` presents exported scene geometry with shared depth
testing, captions and interior cutaways. It reads CELL selection metadata for
image counts independently of camera mode. `tools/remake_showcase.py` owns local
showcase orchestration: tracked recipes select cells and presentation, the normal
application CLI produces candidate scenes, and saved manifests permit image
rerendering with run evidence retained. Generation reuses the saved live
mod-manager source with its active overrides; scene attribution joins exported
identities to winning records and referenced loose-asset providers. Neither tool changes generation policy;
their workflow and count contracts are in [scene image rendering](render-media.md).

`ui/windows_ui` hosts a dark Dear ImGui workspace on Win32 and DirectX, using the
bundled framework core and matching upstream backends. Drawing, folder browsing,
INI persistence and graphics stay on the UI thread. A joined background worker
invokes the shared runner; mutex-protected progress and atomic cancellation keep
rendering responsive and closing waits for a safe completion boundary.
`ui/options_model` prepares selected mod-manager desktop actions independently of rendering,
clearing hidden selectors and resolving scope-dependent generation/export flags.
It can be exercised by automated tests without opening the desktop.

`app::ModManager` carries the persisted desktop source choice, defaulting to MO2.
The desktop passes only the selected source to `app::Run`; CLI source flags are
validated before I/O to reject Vortex mixed with MO2 or developer inputs.
`skyrim/vortex/vortex_importer` reads the active deployed Data directory and
game application plugin files independently of Vortex's internal database.
It preserves SSE activation/order from `plugins.txt`, prepends installed native
masters and installed `Skyrim.ccc` entries, and resolves case-insensitive physical
plugin names. Deployed loose models override base and active-plugin archives;
archive priority follows the resolved plugin order. The importer emits a report
before record resolution, and rechecks plugin-list content plus deployed file
membership, sizes and modification times before processing cells. Missing active
plugins or invalid lists fail rather than producing a partial load order.
Its physical winners use the existing `ModelAssetSources` extraction boundary.
Data does not expose staging-mod ownership or overwritten loose providers, so
automatic selection cannot identify loose-only replacements; explicit Cell
selection supplies those targets. Custom INI archive lists are outside this input
contract. MO2 retains its independent virtual-file importer and catalog cache.

The MO2 importer treats profile and enabled-mod names, decoded configuration
paths, and serialized asset-catalog paths as UTF-8. It converts that text
explicitly into native filesystem paths independently of the Windows ANSI
code page. Catalogs, input reports, and MO2 path diagnostics retain UTF-8 text;
cache reads preserve the same physical asset winners as a fresh import.

`app::Options::cellSelection` carries an explicit list independently of single-cell
selectors and automatic impact selection. `ParseCellSelection` shares delimiter
handling between CLI, desktop validation and persisted list encoding. Cell
lists resolve all identifiers uniquely against the winning load order and
deduplicate by CELL Form ID before processing. `ResolveCellSelection` performs
this resolution independently of generation. Inspection reuses the resolved
snapshot and the single-cell processing pipeline, with separate directories for
multiple targets. `UsesBatchGeneration` dispatches generation lists and Cell copy
exports to supplier extraction, seam stitching, skip policies and the combined
writer. `Options::copySourcePlugin` supplies the active copy template in Cell
mode; its edit history does not expand targets.

`NavigationProfile` and `RecastSettings` carry movement and raster/contour controls
through shared options to cell and batch generation. Validation precedes input
resolution and cache lookup; candidate fingerprints and persisted audit data
include the settings, and candidate JSON retains them as reproducibility evidence.

`app::Run` is the common CLI and Windows entry point. It validates and resolves
inputs, then dispatches a cell inspection or an affected-cell rebuild. The
application helpers in `app/geometry_pipeline` compose extracted world-space
geometry and its provenance.
Single-cell orchestration selects scene cells independently of conservative
model suppliers; distant suppliers contribute intersecting model geometry while
terrain and authored NAVM stay within the scene neighborhood.
`app/batch_runner` owns batch orchestration,
bounded geometry reuse, per-cell evidence, and combined writer dispatch.
The plugin readers decode authored door and consumed external-edge tables into
neutral NAVM connection evidence. Load-order assembly resolves connection
identities using each winning plugin's master table. The shared runner gathers
exit markers independently of generation. `core/scene/scene_exporter` owns
ordered inspection groups, selected-cell ownership, face materials, and solid
connection bars along consuming triangle edges; it does not infer portals from geometric proximity.
`cli/inspection_report` owns inspection OBJ, JSON, HTML, and console reporting;
it consumes the runner's results without choosing generation policy.

The Recast adapter separates input filtering and coordinate conversion, source
indexing, voxel configuration, Recast resource ownership, neutral mesh
conversion, output clipping, adjacency, provenance joins, region discovery, and
exit matching into named internal helpers. Mesh compaction rebases
vertices through one shared operation. Analysis separates surface support from
edge topology, coincident-vertex checks, projected overlaps, and repair evidence.
These stages retain their processing order and output contracts.

Stateless stages use plain functions and neutral value types. The existing
`CandidateGenerator` interface remains the extension point for generation
implementations; individual algorithms do not need speculative class hierarchies.
Long orchestration and binary-format routines explain their stages and index,
ownership, coordinate, and failure invariants where keeping the sequence
together makes it easier to review. The repository formatting configuration and
[agent guidelines](../AGENTS.md) define the shared readability and SOLID rules.

## Shared ownership, asset reuse, and candidate storage

`core/io/SharedBytes` owns immutable payload ranges shared by raw/decoded records
and subrecords; incremental resolver ownership avoids retaining all source payload
tables. `core/io/ContentHash` supplies SHA-256 cache dependency identities.
`ResolvedLoadOrder` owns lookup indexes for FormID and CELL-to-LAND records.
Navigation-equivalence comparison belongs to resolution; affected-cell indexing
consumes its compact change evidence while preserving historical placements.

`skyrim/extraction/asset_cache` resolves versioned archive-provider snapshots and
evicts only owned generated files. `skyrim/extraction/bsa_archive` owns native
BSA directory parsing, bounded zlib/LZ4 decoding, archive precedence, reliable
negative caching, and atomic publication of requested NIFs. `BsaModelExtractor`
loads directory tables lazily and retains them for the geometry-cache lifetime;
its calls are serialized by the model cache. Changed-archive impact selection
uses the same reader without decompressing payloads. Indexes are run-scoped,
while extracted model snapshots retain cross-run reuse. The Python reference
reader under `tools/` supports differential tests and benchmarks. MO2 catalogs
share the cache root and preserve virtual winner priority. `ModelGeometryCache` retains
immutable decoded and placed geometry under one byte budget. `GeometryExtraction`
and `TerrainExtraction` keep support geometry solely in their scene meshes.

`app/batch_generation` owns one isolated all-walkable Recast task, validation,
untouched-neighbor stitching and compact evidence spooling. `core/navmesh/batch_stitching`
is independent of plugin I/O: it plans seam intersections and corner heights across
all generated candidates, refines triangles while preserving joins, revisits
intervals made reachable by corner welding, and adds exact
reciprocal links addressed by opaque CELL keys. It retains unanchored floors and
never substitutes authored meshes from rebuilding targets. `core/navmesh/candidate_reachability`
then evaluates the complete floor graph through shared edges and reciprocal generated
portals. It retains door/authored-neighbor access and the largest floor network in
each contiguous exterior selection or interior, removing isolated roofs even when
they span CELL seams. It owns stable geometry/evidence compaction and reciprocal
destination remapping, contour rebuilding and final topology validation. This stage
runs after linking for both generated and cached candidates; cache entries continue
to store complete pre-link floors. Single-cell orchestration also generates all walkable
components and applies this reachability stage after authored border matching.
Immediate border matching retracts unmatched seams while deferring component
filtering, allowing uncovered cells and interiors without doors to keep their
largest floor. Invalid topology or
refinement prevents export. Authored border subdivision commits only when all
consumed portals survive. `app/candidate_cache` stores candidates before generated
seam refinement with bounded reads and a versioned dependency fingerprint. The
cache/pipeline revision defines compatibility across executable rebuilds; it must
change when successful generation semantics become incompatible. Compact
source joins remain in memory for subsequent splitting; inspection evidence is
loaded from pinned gzip audits. `app/candidate_artifacts` streams public JSON into
gzip. `app/batch_runner` owns selection, sampling, admission, ordered checkpoints,
complete-set reconciliation, artifact policy and combined writing. The writer
allocates all primary NAVM identities before resolving generated CELL destinations.
See [performance settings and limits](performance-improvements.md).

## Affected-cell batch rebuilding

The resolver retains compact placement evidence for every record origin and
builds a FormID lookup index. `CellImpactIndex` indexes exterior cells by
worldspace and coordinates, buckets persistent references by physical position,
and follows changed base records to their placed uses. Persistent worldspace
containers stay outside batch generation destinations and authored border
constraints even when they carry placeholder coordinates. Plugin/load-order scopes
provide geometry suppliers and conservative record-impact discovery for diagnostics.
`skyrim/extraction/collision_impact` selects batch regeneration targets by comparing
historical supported collision triangles, terrain heights and effective water.
It reconstructs placement state and base models at plugin cutoffs, uses the shared
NIF cache, and queries exact horizontal triangle/cell intersections. Visual-only,
NAVM-only and collision-equivalent edits do not expand the regeneration scope.
Added terrain and collision select uncovered cells, including new worldspaces
and submerged heightfields. Water-only transitions require supported winning
terrain or exact placed collision coverage because water is a tagging input.
NAVM presence is independent of impact selection. Selection diagnostics
distinguish these contributions and worldspaces. Missing model paths from a
completed provider search supply no collision and are reported with a warning.
Unreadable models and incomplete archive searches stop selection.
Geometry suppliers and neighboring connection overrides remain independent of
regeneration targets. The shared
runner resolves inputs once, extracts in target order, and admits independent
generation tasks within worker and estimated-byte limits. Bounded terrain,
model-local and transformed-placement caches reuse geometry. Candidate audit
evidence is compacted and spooled to private gzip data; compact joins remain available
for seam subdivision while input scene triangles are released. The batch writer serializes all replacements together and
verifies reciprocal generated triangle targets before finalizing one ESP.
Selection and output contracts are in [batch rebuilding](batch-rebuilding.md).

The shared runner applies the optional uncovered-cell policy before geometry
extraction. `CellsWithExistingNavmesh` indexes winning record ownership rather
than decoded polygon counts, so unsupported and empty records protect their
cells. The writer allocates plugin-owned identities for uncovered cells, derives
child placement from the winning CELL, and validates both new and overridden
NAVMs through the same read-back path. Authored neighboring geometry is retained
when reciprocal portal links are added.
