# Architecture

This project is intentionally split into offline processing stages so the analysis logic stays independent from Skyrim runtime code.

The build targets the offline Windows application, neutral core, and tests. Runtime game APIs are outside the processing pipeline.

## 1. Plugin parsing

The parser layer is responsible for reading a Bethesda plugin file directly from disk.

- Input: `.esm`, `.esp`, or `.esl` file
- Responsibility: identify records, skip unsupported record types, and surface the subset needed for cell/navmesh inspection
- Output: a minimal record stream or normalized plugin index

The current POC keeps this intentionally narrow and reports unsupported record layouts explicitly rather than pretending the parser understands the whole format.

## 2. Skyrim extraction

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

`core::CandidateGenerator` is the replaceable boundary between extracted scene
geometry and a neutral candidate navmesh. The shared CLI/Windows run path uses
`RecastCandidateGenerator` from the Recast Navigation submodule. It accepts only
terrain and supported collision triangles, converts Skyrim Z-up world positions
to Recast Y-up coordinates, rasterizes a supported exterior halo around the target CELL,
filters walkable spans, erodes them by
agent radius, then builds regions, contours, and a polygon mesh. Recast polygons
are triangulated and clipped to the target CELL in the project's neutral model for JSON, OBJ, and the
`Candidate NAVM` GLB layer. Eligible generated geometry can enter the guarded
plugin writer. The application then matches selected exterior boundary edges to
adjacent NAVM edges in the resolved load order and records reciprocal targets.
The shared authored-border tolerance allows small deviations from nominal CELL
bounds only at matched portal endpoints. Both generation and serialization
preserve those endpoints and keep other generated vertices inside the target.
Every valid generated component is retained. Stitching can subdivide a containing
generated boundary edge to match smaller authored border edges without removing
the remaining geometry. Collinear generated subdivisions can be coalesced through
validated boundary-fan retriangulation, preserving the interior rim and remapping
source, region, contour, door, and portal indices. Inward authored offsets trim
generated fans; outward offsets add connectors. Connections retain full authored endpoints and obey
distance, height, slope, and welding constraints.
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

Plugin-scope authoring can instead copy the selected source with generated
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
  -> offline extraction
  -> neutral model
  -> analysis and replaceable candidate generator (currently Recast)
  -> guarded NAVM override serialization

This keeps the codebase testable, portable, and independent from a live Skyrim process.

## Source responsibilities and readability

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
evicts only owned generated files. The project-owned `tools/bsa_index.py` boundary
indexes archive metadata and reads requested entry ranges. MO2 catalogs share the
cache root and preserve virtual winner priority. `ModelGeometryCache` retains
immutable decoded and placed geometry under one byte budget. `GeometryExtraction`
and `TerrainExtraction` keep support geometry solely in their scene meshes.

`app/batch_generation` owns one isolated Recast task, candidate validation,
authored stitching, evidence compaction, and spooling. `app/candidate_cache` owns
the versioned private gzip layout and dependency fingerprint, bounded reads,
and cache compatibility checks. `app/candidate_artifacts` streams public JSON into
gzip while preserving its schema. `app/batch_runner` owns sampling, admission,
ordered collection/checkpoints, global border reconciliation, export policy, and
combined writing. Active audit pins survive cache eviction until export completes.
See [performance settings and limits](performance-improvements.md).

## Affected-cell batch rebuilding

The resolver retains compact placement evidence for every record origin and
builds a FormID lookup index. `CellImpactIndex` indexes exterior cells by
worldspace and coordinates, buckets persistent references by physical position,
and follows changed base records to their placed uses. Plugin/load-order scopes
select conservative affected targets and model-bound influence halos. The shared
runner resolves inputs once, extracts in target order, and admits independent
generation tasks within worker and estimated-byte limits. Bounded terrain,
model-local and transformed-placement caches reuse geometry. Candidate audit
evidence is compacted, spooled to private gzip data, and released from RAM. The batch writer serializes all replacements together and
verifies reciprocal generated triangle targets before finalizing one ESP.
Selection and output contracts are in [batch rebuilding](batch-rebuilding.md).

The shared runner applies the optional uncovered-cell policy before geometry
extraction. `CellsWithExistingNavmesh` indexes winning record ownership rather
than decoded polygon counts, so unsupported and empty records protect their
cells. The writer allocates plugin-owned identities for uncovered cells, derives
child placement from the winning CELL, and validates both new and overridden
NAVMs through the same read-back path. Authored neighboring geometry is retained
when reciprocal portal links are added.
