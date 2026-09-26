# Skyrim Navmesh Generator Roadmap

This is the durable implementation plan for an **offline** Skyrim SE/AE navmesh tool. It covers both intended products:

1. create navmeshes for new/large world projects, such as Beyond Skyrim and Skyblivion;
2. diagnose and repair navigation broken by an installed mod load order.

The tool never requires Skyrim or SKSE to be running, never edits source plugins in place, and does not claim a repair is safe until it has been validated.

The user-facing input is an existing **Mod Organizer 2 instance and profile**, not a hand-authored load-order file. See [the MO2 operator workflow](operator-workflow.md) for the command contract required at every milestone.

## Project contract

The end-to-end pipeline is:

```text
declared load order
  -> resolve masters, overrides, and world/cell ownership
  -> build navigation scene (terrain + collision + references)
  -> inspect existing NAVM and diagnose discrepancies
  -> generate a constrained candidate surface
  -> plan either a repair or a new NAVM
  -> serialize an override-only output plugin
  -> validate in data tools and in game
```

The neutral model is the boundary between Bethesda-specific readers/writers and geometry/navigation algorithms. It must contain source provenance, coordinate-system metadata, and confidence/coverage information so that the repair tool can explain every result.

## Current baseline (September 2026)

The repository has an experimental but useful read-only single-plugin pipeline:

- direct parsing of a subset of `CELL`, `REFR`/`ACHR`, and `NAVM` records;
- extraction of model paths and transforms, and NIF visual-mesh export through Nifly;
- NAVM OBJ/JSON export plus basic validation and statistics;
- an experimental per-polygon support query against extracted NIF triangles;
- `analysis.obj` and `navmesh_diagnostics.html`, including source-NIF provenance for selected support triangles.

Its limitations are decisive for later work:

- a single plugin is parsed in isolation: masters, overrides, ESL/light FormIDs, and a real load order are not resolved;
- the NAVM decoder is deliberately narrow and must not yet be treated as a complete read/write representation;
- it uses visual NIF geometry, not authoritative Havok collision;
- `LAND` terrain is absent, which means the ground is absent for exterior cells;
- association classifications are diagnostic hypotheses, not repair instructions;
- the existing `navmesh-generator` SKSE target is legacy/experimental and is outside the offline product path.

## Shared definitions of done

Every implementation step must:

- keep the offline binary buildable and tests passing;
- add unit tests for new neutral logic and a fixture/integration test when possible;
- report unsupported or incomplete data explicitly instead of silently approximating it;
- preserve source provenance (plugin, FormID, record type, and geometry source) in machine-readable output;
- document CLI/output changes in `README.md` and any format decisions in `docs/`;
- avoid modifying input ESM/ESP/ESL files.

Use small legally redistributable fixtures or fixture builders in the repository. Tests that use local game data must be opt-in via a configured data directory and skip cleanly when it is absent.

## Milestones and implementation order

### 0. Establish reproducible evidence and contracts

**Purpose:** Make results comparable before changing parsing or geometry.

Implement:

- a versioned JSON schema/metadata block for every export (tool version, input plugins, selected cell, coordinates, source coverage, warnings);
- a `fixtures/` strategy: synthetic binary/NIF fixtures for unit tests and a documented optional local-game integration configuration;
- a benchmark manifest of representative interior, exterior, bridge/overpass, and mod-conflict cells (references only; no game assets committed);
- a coordinate-system document that defines Skyrim world coordinates, NIF local coordinates, transform order, units, OBJ/glTF export axes, and expected tolerances.

Accept when: a run can be reproduced from an input manifest; exports identify their source data and coordinate convention; the test suite runs without a game install.

### 1. Make input a resolved load order, not an isolated plugin

**Purpose:** Both creation and repair depend on what the game actually loads.

Implement:

- a `--mo2 <instance-or-portable-root> --profile <profile-name>` input model. Read the selected profile's existing `modlist.txt`, `plugins.txt`, and `loadorder.txt`, plus MO2's configured game/mod/profile/overwrite paths; do not ask the user to create or edit a manifest;
- construct a read-only virtual file map from base-game `Data`, enabled MO2 mods in their recorded priority, and the profile's `Overwrite` content. Report the exact physical source selected for every plugin and loose asset;
- take an immutable snapshot of the profile inputs for each run (paths, timestamps, hashes, active plugins, and mod priority), and fail clearly if they change during processing;
- retain `--data` and `--load-order` only as explicitly documented developer/test overrides, never as the normal operator workflow;
- TES4 master parsing, file identity, full/light FormID resolution, and winning-override resolution;
- cell/worldspace indexing that preserves persistent, temporary, and exterior-cell groups;
- diagnostics for missing masters, cycles, duplicate files, unresolved FormIDs, and unsupported record variants;
- a parser-library evaluation spike with written findings. Retain the direct parser only if its verified coverage and licensing fit the required records; otherwise isolate and adopt a mature offline parser behind a reader interface.

Accept when: `--mo2` and `--profile` import a real existing profile without manual files; a small fixture load order resolves a base cell, a reference override, and a NAVM override exactly as expected; outputs name the winning record, origin chain, and physical file selected through MO2's virtual-file rules.

### 2. Build a loss-aware Bethesda record model

**Purpose:** Reading and writing safely requires more information than the current triangle-only NAVM representation.

Implement:

- record/subrecord reader primitives with bounds checks, compressed-record support, and structured unsupported-data errors;
- a `PluginRecord`/`RecordOrigin` layer that preserves unknown subrecords and byte ranges where required for future round trips;
- full models for the subset required by this roadmap: `WRLD`, `CELL`, `LAND`, `REFR`, `ACHR`, base objects/models, `NAVM`, and relevant door/link records;
- a NAVM format study against documented or independently verified samples. Represent all fields needed for faithful preservation, not just vertices and triangles.

Accept when: known record fixtures round-trip through the reader without losing recognized metadata, and unsupported NAVM versions fail with an actionable diagnostic.

### 3. Build a provenanced scene graph and transform system

**Purpose:** A geometry/navmesh mismatch must be traceable to a record, mesh, and transform.

Implement:

- neutral `Scene`, `SceneNode`, `GeometrySource`, `Material/CollisionClass`, and per-triangle provenance types;
- a single tested transform implementation (local NIF transform, parent-node transforms, reference scale/rotation/translation) with explicit Euler convention;
- base-object/model lookup through resolved records, including references from masters;
- deterministic scene filtering by cell bounds, relevant neighboring exterior cells, reference type, and configurable radius;
- source coverage accounting: found, excluded, missing, unreadable, and unsupported geometry.

Accept when: transform fixtures verify translation, rotation, non-unit scale, and parent transforms; each exported triangle can be traced back to its plugin record and model path.

### 4. Add exterior terrain as first-class ground geometry

**Purpose:** Exterior NAVM cannot be evaluated or generated without `LAND` ground.

Implement:

- decode exterior `LAND` height data and cell coordinates into a regular terrain grid/triangles in Skyrim world space;
- account for LAND inheritance/override semantics and missing terrain records;
- include terrain bounds and per-triangle provenance (`LAND`, cell, sample coordinates);
- expose a terrain-only export/diagnostic mode and terrain coverage statistics;
- explicitly defer visual LOD and textures: only collision-relevant height surfaces are in scope.

Accept when: an exterior benchmark exports terrain aligned with existing NAVM and terrain triangle provenance; a missing LAND record is reported rather than replaced with a flat plane.

### 5. Replace visual meshes with navigation-relevant collision geometry

**Purpose:** Render meshes can be decorative, displaced, or non-walkable; collision is the authority for navigation.

Implement:

- inspect NIF/Havok structures and choose the smallest reliable collision extraction path for Skyrim SE assets;
- extract collision shapes (including transforms), triangulate supported primitive/packed shapes, and label the collision type;
- provide an explicit, configurable fallback to render meshes only where collision is unavailable, with reduced confidence;
- add policy filters for non-solid, effect, furniture, animated, and ignored reference classes;
- keep render geometry optionally available solely for visual comparison.

Accept when: collision geometry is separately counted/exported, a collision-versus-render fixture proves the correct source is selected, and every support result says whether it came from terrain, collision, or a low-confidence fallback.

### 6. Make scene inspection trustworthy and practical

**Purpose:** Developers must be able to see an entire scene without manually combining many OBJ files.

Implement:

- one combined scene export with named layers/groups: existing NAVM, terrain, collision, render fallback, and diagnostic markers;
- a color-capable format (prefer glTF/GLB; retain OBJ for simple compatibility) with stable material colors and per-object names/provenance map;
- a standalone report that links classifications to source triangles and groups them by source/coverage status;
- spatial indexing and streaming/culling so a large exterior selection does not require all assets in memory;
- command options for cell bounds, neighboring-cell radius, geometry layers, and output detail.

Suggested viewer colors: existing NAVM cyan; terrain brown/green; collision gray; render fallback purple; supported diagnostic green; floating orange; buried red; unknown gray.

Accept when: one GLB can be opened in Online 3D Viewer and clearly shows all requested layers in alignment; a large benchmark completes within a documented memory/time budget.

### 7. Turn association diagnostics into reliable discrepancy detection

**Purpose:** Determine what is actually wrong before any repair changes a NAVM.

Implement:

- replace centroid-only sampling with multi-point polygon coverage and edge-aware tests;
- query terrain/collision through an acceleration structure and use source-priority rules;
- classify each polygon as supported, floating, buried, too steep, blocked, out-of-coverage, or ambiguous;
- calculate confidence from source type, sample agreement, distance, slope, and scene coverage;
- detect connected-component defects, invalid adjacency, overlaps, gaps, duplicate vertices, off-mesh islands, and cross-cell border issues;
- emit a stable repair-candidate report that contains evidence and **never writes a plugin**.

Accept when: the benchmark cases produce expected classifications, including bridge-over-terrain and missing-geometry cases; ambiguous/out-of-coverage polygons are not presented as defects.

### 8. Generate a candidate walkable surface

**Purpose:** This is the shared core for generating new NAVM and proposing repairs.

Implement:

- define navigation parameters as named versioned profiles: agent radius/height, max slope, step height, clearance, cell-border policy, and simplification tolerances;
- build a walkable triangle set from terrain/collision, including slope, clearance, and obstruction tests;
- voxel/heightfield or polygon-based region construction (choose after a documented spike using real benchmark scenes);
- contour extraction, region filtering, polygonization/triangulation, adjacency construction, and border stitching;
- preserve generation provenance/parameters per region and export the result only as a neutral candidate NAVM.

Accept when: candidate NAVM has valid topology, respects test obstacles/slopes, aligns visually with the scene, and is deterministic for a fixed input/profile.

### 9. Plan conservative repair operations

**Purpose:** Repair should change only defective parts and preserve intentional authored navigation.

Implement:

- compare existing and candidate NAVM spatially and semantically;
- define repair operations: retain, delete invalid polygon, move/rebuild local region, add missing region, repair adjacency, and flag manual review;
- require high confidence and bounded affected regions for automatic proposals; default uncertain cases to manual review;
- protect authored features requiring preservation until explicitly supported (doors, cover, preferred paths, navmesh metadata, links, island data);
- output a reviewable repair plan (JSON + colored scene) and an approval token/manifest required before writing.

Accept when: fixture cases show minimal local changes, intentionally differing NAVM stays untouched, and no operation is proposed for low-confidence/out-of-coverage geometry.

### 10. Serialize safe override-only plugins

**Purpose:** Writing is the highest-risk operation and comes after read/generate/repair are proven.

Implement:

- create a new ESP/ESL output only; never overwrite input files;
- write required TES4 header/master dependencies and only changed winning records;
- serialize verified NAVM records while preserving every required field and links that are not deliberately changed;
- validate FormIDs, group placement, compression, record sizes, and master dependencies before output;
- provide `--dry-run`, `--output`, and a post-write read-back verification that compares intended neutral NAVM to the emitted record.

Accept when: a generated override is recognized by independent tooling, read-back matches the planned result, and game validation on a disposable profile confirms no navmesh load errors.

### 11. Validate, scale, and release

**Purpose:** Make the tool dependable for team-sized projects and mod lists.

Implement:

- automated regression suite covering parsers, terrain, collision, generation, repairs, and writer read-back;
- golden reports/GLB metadata for legal fixtures plus optional local-game benchmarks;
- fuzz/malformed-input testing for record parsing and NIF/Havok decoding;
- performance tests for representative interiors, exterior cells, and large mod lists;
- command-line help, reproducible run manifests, compatibility matrix, failure taxonomy, and operator guide;
- staged user testing: report-only, candidate-only, review-plan, disposable write test, then limited production use.

Accept when: documented supported cases meet the performance/accuracy budget, failure modes are actionable, and release candidates pass independent tool and in-game validation.

## Product paths after the shared foundation

| Capability | New-world generation | Mod-list repair |
| --- | --- | --- |
| Input | Project's declared masters and new cells | Exact user load order, including overrides |
| Existing NAVM | May be absent | Is the baseline to preserve or repair |
| Core operation | Candidate generation over selected cells/regions | Discrepancy detection, then local repair planning |
| Safety default | Export neutral candidate first | Report-only; manual review for ambiguity |
| Writer output | New/override NAVM plugin | Minimal patch plugin with exact masters |

## Decision gates

Do not advance past these gates merely because code compiles:

1. **Before generation:** prove coordinate alignment with real terrain and collision in at least one interior and one exterior benchmark.
2. **Before repair planning:** prove that known intentional differences are not flagged as defects.
3. **Before writing:** independently confirm the NAVM reader understands every field the writer must preserve.
4. **Before broad use:** validate output in xEdit/other independent readers and in a disposable Skyrim profile.

## Copy-ready Codex prompts

Run these in order. Each prompt is deliberately narrow; it authorizes one reviewable change set.

### Prompt 0

> Read `docs/roadmap.md`, then implement milestone 0 only. Do not change parsing or generation behavior. Add reproducibility metadata/schema, a legal fixture strategy, optional local-game integration-test configuration, and coordinate-system documentation. Preserve existing user changes, add tests, run the relevant build/tests, and report modified files plus any assumptions.

### Prompt 1

> Read `docs/roadmap.md` and implement milestone 1 only: resolved-load-order input and winning-override resolution. First write the parser-library evaluation findings in `docs/`. Keep the parser behind a small interface. Support fixture-tested masters, full/light FormIDs, cell/worldspace indexing, and actionable missing-master diagnostics. Do not implement terrain, collision, generation, or writing.

### Prompt 2

> Read `docs/roadmap.md` and implement milestone 2 only. Strengthen loss-aware reading for the records listed there and investigate/document NAVM field layout using verified samples. Preserve unknown data when required for a future round trip. Add malformed-input tests. Do not write plugins.

### Prompt 3

> Read `docs/roadmap.md` and implement milestone 3 only: a neutral provenanced scene graph and one tested transform system. Make every triangle traceable to its input record/model and account for coverage failures. Do not add terrain or collision extraction yet.

### Prompt 4

> Read `docs/roadmap.md` and implement milestone 4 only: exterior `LAND` terrain decoding and world-space terrain triangles with provenance. Add benchmark documentation and tests for height/coordinate alignment. Do not substitute a flat ground surface for missing terrain.

### Prompt 5

> Read `docs/roadmap.md` and implement milestone 5 only: extract and triangulate the smallest reliable subset of Skyrim collision geometry. Keep render meshes as an explicitly lower-confidence fallback and report source type on every triangle. Add a fixture that demonstrates collision selection over render geometry.

### Prompt 6

> Read `docs/roadmap.md` and implement milestone 6 only. Export one combined, color-layered GLB scene plus provenance metadata, retaining OBJ compatibility. Add scalable scene selection/indexing and document how to inspect it in Online 3D Viewer. Do not change repair classifications.

### Prompt 7

> Read `docs/roadmap.md` and implement milestone 7 only: reliable discrepancy detection with multi-point coverage sampling, source priority, confidence, and explicit ambiguous/out-of-coverage states. Emit report-only repair candidates; do not modify NAVM records or generate replacements.

### Prompt 8

> Read `docs/roadmap.md` and implement milestone 8 only. First document and justify the candidate-surface algorithm choice with benchmark evidence. Then generate deterministic neutral candidate NAVM with versioned navigation profiles, topology validation, and visual export. Do not serialize a plugin.

### Prompt 9

> Read `docs/roadmap.md` and implement milestone 9 only: conservative, reviewable repair planning based on existing versus candidate NAVM. Require high confidence for automatic proposals and preserve unsupported authored features. Output a repair plan; do not write a plugin.

### Prompt 10

> Read `docs/roadmap.md` and implement milestone 10 only: safe override-only plugin serialization with dry-run, exact masters, read-back verification, and output validation. Never modify source plugins. Add tests proving unchanged fields are preserved.

### Prompt 11

> Read `docs/roadmap.md` and implement milestone 11 only: regression/performance/fuzz coverage, operator documentation, compatibility matrix, and staged validation workflow. Do not expand record support unless a failing test proves it is required.

## Immediate next instruction

Start with the MO2-import completion prompt in [the operator workflow](operator-workflow.md). The existing milestone-1 implementation is a useful parser/resolution spike, but it is not complete for the operator workflow until it imports a selected MO2 profile directly.
