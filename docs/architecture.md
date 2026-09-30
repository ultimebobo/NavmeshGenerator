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
to Recast Y-up coordinates, rasterizes and filters walkable spans, erodes them by
agent radius, then builds regions, contours, and a polygon mesh. Recast polygons
are triangulated into the project's neutral model for JSON, OBJ, and the
`Candidate NAVM` GLB layer. Generated geometry remains an inspection candidate;
it is not encoded as a Bethesda NAVM record.

The previous polygon-based `GenerateCandidate` function remains for historical
fixture comparisons. Application runs use the Recast implementation.

## 6. Future NAVM serialization

The next milestone is not runtime integration but a future serialization layer that can convert neutral navmesh data back into a Bethesda-compatible record layout when the generation step is ready.

This project intentionally keeps the serialization boundary separate from the geometry and analysis logic so automatic generation can be implemented as a later stage rather than a hidden side effect of the parser.

## Design boundary

The critical architecture boundary is:

Skyrim plugin data
  -> offline extraction
  -> neutral model
  -> analysis and replaceable candidate generator (currently Recast)
  -> repair planning
  -> future NAVM serialization

This keeps the codebase testable, portable, and independent from a live Skyrim process.
