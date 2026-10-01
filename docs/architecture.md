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
to Recast Y-up coordinates, clips exterior input to the target CELL bounds independently of its geometry suppliers,
rasterizes and filters walkable spans, erodes them by
agent radius, then builds regions, contours, and a polygon mesh. Recast polygons
are triangulated into the project's neutral model for JSON, OBJ, and the
`Candidate NAVM` GLB layer. Eligible generated geometry can enter the guarded
plugin writer. The application then matches selected exterior boundary edges to
adjacent NAVM edges in the resolved load order and records reciprocal targets.
The shared authored-border tolerance allows small deviations from nominal CELL
bounds only at matched portal endpoints. Both generation and serialization
preserve those endpoints and keep other generated vertices inside the target.
Regions without a matched border portal or door are removed from this candidate.

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
portals targeting any replaced secondary NAVM are redirected to the generated
primary NAVM. Adjacent geometry remains unchanged. Other authored links, cover
data, NAVI, and REFR XNDP references remain outside this
writer's supported remapping.

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
`cli/inspection_report` owns inspection OBJ, JSON, HTML, and console reporting;
it consumes the runner's results without choosing generation policy.

The Recast adapter separates input filtering and coordinate conversion, source
indexing, voxel configuration, Recast resource ownership, neutral mesh
conversion, adjacency, provenance joins, region discovery, exit matching, and
reachability filtering into named internal helpers. Mesh compaction rebases
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

## Affected-cell batch rebuilding

The resolver retains compact placement evidence for every record origin and
builds a FormID lookup index. `CellImpactIndex` indexes exterior cells by
worldspace and coordinates, buckets persistent references by physical position,
and follows changed base records to their placed uses. Plugin/load-order scopes
select conservative affected targets and model-bound influence halos. The shared
runner resolves inputs once, generates one CELL at a time and reuses a bounded
LRU cache of source-cell geometry. Candidate evidence is compacted after each
scene is released. The batch writer serializes all replacements together and
verifies reciprocal generated triangle targets before finalizing one ESP.
Selection and output contracts are in [batch rebuilding](batch-rebuilding.md).
