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
to Recast Y-up coordinates, clips exterior input to the extracted CELL bounds,
rasterizes and filters walkable spans, erodes them by
agent radius, then builds regions, contours, and a polygon mesh. Recast polygons
are triangulated into the project's neutral model for JSON, OBJ, and the
`Candidate NAVM` GLB layer. Eligible generated geometry can enter the guarded
plugin writer. The application then matches selected exterior boundary edges to
adjacent NAVM edges in the resolved load order and records reciprocal targets.
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
to the generated NAVM and reciprocal portals to adjacent NAVM overrides. Other
authored links, cover data, NAVI, and REFR XNDP references remain outside this
writer's supported remapping.

## Design boundary

The critical architecture boundary is:

Skyrim plugin data
  -> offline extraction
  -> neutral model
  -> analysis and replaceable candidate generator (currently Recast)
  -> guarded NAVM override serialization

This keeps the codebase testable, portable, and independent from a live Skyrim process.
