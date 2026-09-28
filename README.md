# Navmesh Generator

Offline experimental Skyrim navmesh analysis tooling.

Run `navmesh-offline.exe` without arguments to open the Windows desktop UI. It exposes the main command-line options, including a **Generate candidate NAVM** checkbox and versioned navigation-profile selector, stores the most recently used values in `%LOCALAPPDATA%\\NavmeshGenerator\\config.ini`, shows hover help through the circular `?` controls, and reports elapsed time when a run finishes.

## What this is

This project is an offline, read-only prototype for inspecting Skyrim plugin data without requiring an active Skyrim runtime or SKSE. The focus is on parsing plugin records directly, extracting cell and navmesh information from a plugin file, and turning that into a neutral geometry + navmesh model that can be analyzed and exported for inspection.

## What it currently does

- Builds a neutral, CommonLib-free C++ core for geometry and navmesh data.
- Builds a neutral scene graph for terrain, authoritative packed Havok collision, and explicitly low-confidence render fallbacks. Every emitted triangle carries reference/base-record/model provenance and its source type.
- Reads the roadmap record subset (`WRLD`, `CELL`, `LAND`, `REFR`/`ACHR`, base-model records, and `NAVM`) through a loss-aware record layer: byte ranges, compressed source bytes, decoded subrecords, and unknown fields are retained for future round trips.
- Produces JSON diagnostics, compatible OBJ exports, and one color-layered GLB scene for navmesh polygon inspection.
- Computes basic navmesh statistics such as vertex counts, polygon area range, connected components, and degenerate polygons.
- Includes automated tests for neutral geometry and navmesh analysis.

## Example

```powershell
xmake build navmesh-offline
./build/windows/x64/releasedbg/navmesh-offline.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --cell-formid 00027D1C --output ./output
```

The tool prints the resolved cell and also accepts an editor ID or exterior cell coordinate pair:

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --editor-id KilkreathRuins03 --output ./output
```

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --cell-x 10 --cell-y -5 --output ./output
```

Use `--list-cells` to discover available form IDs, editor IDs, and exterior coordinates:

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --list-cells --output "D:/NavmeshWork/cells"
```

## Mod Organizer 2 input

The intended normal workflow is to select an existing Mod Organizer 2 instance and profile. The tool will read the profile's existing mod priority, active plugins, and load order without asking you to create or edit a manifest:

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --list-cells --output "D:/NavmeshWork/cells"
```

The importer writes `input-report.json` before listing or extraction. It records the profile snapshot hash, enabled-mod priority, active plugin winners, and loose-file winners selected from game `Data`, enabled mods, and `Overwrite`. The `--data` plus `--load-order` route remains an internal developer/test interface only.

If an existing MO2 profile's configured mod storage was moved, use the read-only `--mods-dir` recovery override. It changes only where enabled mod folders are looked up; it never edits MO2 configuration or profile files:

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --mo2 "C:/Users/baudo/AppData/Local/ModOrganizer/Mod Authoring" --profile "My Actual Profile" --mods-dir "D:/Dev/Skyrim Mods/Mods" --list-cells --output "D:/NavmeshWork/cells"
```

Split MO2 layouts are supported: when `ModOrganizer.ini` has a relative `base_directory`, profile/mod/overwrite paths resolve from that directory (for example, a sibling `MODS` tree), while `gamePath` resolves to the configured stock game directory.

The direct reader currently resolves its fixture-tested full/light identities and override chains, and reports missing masters/cycles/duplicates. Its deliberately narrow coverage is documented in [the parser evaluation](docs/parser-library-evaluation.md).

## Output

The CLI writes files into the target output directory:

- `report.json` — cell metadata, references, navmesh summaries, and validation findings.
- `navmesh.obj` — exported navmesh polygon geometry for inspection in Blender or MeshLab.
- `geometry.obj` — exported selected support geometry: packed Havok collision when available for a reference, otherwise a render fallback, plus decoded exterior `LAND` terrain.
- `geometry.json` — machine-readable triangle provenance plus geometry-coverage failures. Every triangle has `sourceType` (`terrain`, `collision`, or `render_fallback`), `collisionType`, and confidence. Terrain triangles also name their winning `LAND` record, exterior cell, and lower-left VHGT sample coordinate.
- `scene.glb` — one combined, color-layered scene with named Existing NAVM, terrain, collision, render-fallback, and diagnostic-marker objects. `scene.glb.provenance.json` maps each named object to its source record/model and preserves run metadata; `scene.glb.metadata.json` is the standard metadata sidecar.
- `scene-report.html` — standalone classification report grouped by support-source and geometry-coverage status. Its support-triangle indices join `analysis.json` to `geometry.json`; this report does not change classifications.
- `analysis.json` — stable, report-only discrepancy evidence: seven-point polygon coverage, selected source type and confidence, explicit `ambiguous`/`out_of_coverage` states, topology findings, and manual-review repair candidates. It never contains replacement NAVM geometry or a plugin write instruction.
- With `--generate-candidate`, `candidate-navm.json` contains a neutral candidate NAVM with versioned profile parameters, triangle and region provenance, identified door exits, border reachability, contours, adjacency, statistics, and topology validation. `candidate-navm.obj` is a simple mesh view, and `scene.glb` gains a blue-green Candidate NAVM layer beside the existing scene layers. These files do not contain plugin records.
- `input-report.json` — MO2 profile snapshot and virtual-file winners, emitted first for every MO2 run.
- `load-order.json` — when using `--load-order`, every winning record with its plugin and ordered origin chain.

Every JSON export contains a versioned `metadata` block. OBJ and HTML exports have an adjacent `<export>.metadata.json` sidecar. The metadata identifies the tool version, input plugin, selected cell, coordinate convention, source coverage, and known limitations. Its schema is [docs/schemas/export-metadata.schema.json](docs/schemas/export-metadata.schema.json); coordinate details are in [docs/coordinate-system.md](docs/coordinate-system.md).

The combined-scene workflow, material legend, provenance joins, and Online 3D Viewer instructions are in [docs/scene-inspection.md](docs/scene-inspection.md). For scalable exterior scenes, use `--neighboring-cell-radius <cells>`, `--scene-bounds <minX> <minY> <maxX> <maxY>`, `--geometry-layers <navmesh,terrain,collision,render,diagnostics>`, and `--output-detail <full|summary>`. The Windows UI exposes **Neighboring cells** with a default of `1`; set it to `0` for the selected CELL only. `--export-scene <path>` changes the default `scene.glb` location. Neighboring cells are appended one at a time and bounds filtering uses a coarse world-space scene grid before GLB buffers are created.

To generate an inspection candidate from the selected cell, add `--generate-candidate --navigation-profile human@1.0.0`. The other built-in profile is `small@1.0.0`; unknown names or versions fail explicitly. Profiles specify agent radius and height, slope, step, vertical clearance, cell-border policy, weld tolerance, minimum region area, and contour simplification tolerance. Candidate construction uses terrain and supported collision only. It filters slopes and obstructions, joins close placed-mesh seams and partial edges (including edges split by the radius inset), and preserves separate vertical levels unless a traversable ramp or step joins them. Regions reaching an exterior border or enabled door are retained. Substantial supported collision regions also remain as inspection candidates with a warning when their connectivity is unverified; smaller isolated fragments are removed. If no region meets those conditions, the largest connected region is kept with a warning. The result is triangulated, contoured, and topology-checked before success. See [the algorithm choice and benchmark evidence](docs/candidate-surface-algorithm.md) for scope and limits. There is no plugin serialization option.

Record the command inputs using [docs/run-manifest.example.json](docs/run-manifest.example.json) before a benchmark. The current CLI does not consume this file; map its fields to the existing command-line flags so milestone 0 does not alter parser input behavior.

## Reproducible fixtures and optional game checks

The repository contains only synthetic, redistributable fixture builders; it does not include Skyrim assets. See [fixtures/README.md](fixtures/README.md) for the policy and [docs/benchmarks.json](docs/benchmarks.json) for the local benchmark reference manifest. To opt into the read-only local-game parser smoke test, configure `SKYRIM_DATA_DIR` as described in [tests/integration/README.md](tests/integration/README.md). Without it, the complete test suite still runs and skips that check.

## Current limitations

- This is not an SKSE runtime plugin and does not inspect a running game session.
- The direct parser is intentionally small and targets the common Bethesda plugin structure, not the entire plugin ecosystem.
- Compressed indexed records are zlib-decoded with declared-size and boundary checks while their original bytes remain retained. Malformed records and unknown NAVM versions are explicit diagnostics, never best-effort geometry.
- Geometry extraction resolves MO2's winning loose NIFs from enabled mods and Overwrite, then caches requested meshes from enabled mod and game BSAs on demand. This includes meshes supplied by a separate resources mod. The supported collision subset is reachable `bhkMoppBvTreeShape`/`bhkListShape` wrappers containing packed strips, `bhkNiTriStripsShape`, or Skyrim SE `bhkCompressedMeshShape` chunks; their triangle vertices receive the Havok rigid-body transform, then reference scale, Skyrim's placed-reference rotation matrix from the `DATA` angles (radians), and translation. A positive reference Z angle rotates model-local +X toward world -Y. Other Havok primitives are reported as unsupported rather than guessed. Render meshes are retained only as 0.35-confidence fallback when no supported collision exists. Winning placed records flagged initially disabled or deleted are excluded from both geometry layers and reported in coverage. Effects, furniture, actors, and path-marked animated/FX references are excluded by policy.
- Install the BSA bridge dependencies with `python -m pip install -r tools/requirements.txt` before extracting archived assets. If Python is not on PATH, set `NAVMESH_PYTHON` to the Python executable.
- Archived assets are cached under `<output>/.bsa-cache`; delete that directory to rebuild the cache.
- Exterior `LAND` decoding supports the collision-relevant VHGT height grid only: 33×33 samples per cell, 128-unit spacing, and world origin `(cellX * 4096, cellY * 4096)`. It intentionally excludes visual LOD and texture layers. Use `--terrain-only` with a resolved load order to export only that terrain diagnostic surface. Missing or malformed `LAND` records are reported and produce no replacement plane.
- Some Skyrim record variants and non-standard modded data layouts may still be rejected or reported as unsupported.
- Discrepancy detection is deliberately conservative. Collision support has priority over terrain, terrain has priority over render fallback, and only sufficiently consistent samples can classify a polygon. Ambiguous or out-of-coverage polygons are displayed as limitations rather than defects. Repair candidates are report-only/manual-review evidence; the tool does not modify NAVM records or write replacement plugins.

The verified NVNM prefix, preservation policy, and known trailing-layout limits are documented in [docs/navm-format-study.md](docs/navm-format-study.md). This milestone remains read-only and does not serialize plugins.

## Next milestone

The next roadmap milestone is:

```text
candidate NAVM -> conservative repair planning
```

not runtime integration.

See [docs/architecture.md](docs/architecture.md) for the intended pipeline and separation between parsing, extraction, neutral model, analysis, and future NAVM serialization.

## Source documentation

The C++ API reference is generated with Doxygen. Install Doxygen and run
`doxygen Doxyfile` from the repository root, then open
`build/doxygen/html/index.html`. The source guide is in [docs/api.md](docs/api.md).
The generated site covers project-owned `src/` code and linked project guides;
it does not include vendored dependencies.
