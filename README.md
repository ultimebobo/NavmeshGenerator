# Navmesh Generator

Offline experimental Skyrim navmesh analysis and guarded override generation tooling.

Run `navmesh-offline.exe` without arguments to open the Windows desktop UI. It exposes the main command-line options, including **Generate candidate NAVM** and **Write plugin**, stores the most recently used values in `%LOCALAPPDATA%\\NavmeshGenerator\\config.ini`, shows hover help through the circular `?` controls, and reports elapsed time when a run finishes. Candidate generation uses [Recast Navigation](https://github.com/recastnavigation/recastnavigation) from the `lib/recastnavigation` submodule; clone with `git submodule update --init --recursive` before building.

## What this is

This project processes Skyrim plugin data offline without requiring an active Skyrim runtime or SKSE. It reads plugin records, builds a neutral geometry and navmesh model, and can write NAVM overrides into a new ESP, ESL-flagged when eligible.

The desktop build uses the Nifly and Recast Navigation submodules. Archived asset extraction uses the BSAFileExtractor submodule. The project has no runtime plugin target.

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

## Rebuild a plugin or load order

Use the persisted **Rebuild scope** selector in the Windows UI, or add one of
these options to an MO2/profile command:

```powershell
navmesh-offline.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-plugin "<active plugin.esp>" --generate-plugin --output "<new output folder>"
navmesh-offline.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-load-order --generate-plugin --output "<new output folder>"
```

Plugin scope finds cells possibly affected by that plugin's edit history;
load-order scope considers changes after the first active baseline plugin and
MO2 model replacements. Selection includes historical moved/deleted placements,
changed base-object uses, persistent references and exterior impact halos.
Generation uses winning geometry from the full load order and neighboring cells,
while each generated NAVM stays clipped to its own target CELL. Oversized model
bounds extend the geometry suppliers and impact footprint; unknown bounds use
conservative worldspace coverage.

The load order is parsed once, targets run in spatial order, and a bounded geometry
cache reuses neighboring extraction. Candidates go under `cells/<FormID>/`, with
selection, skips and cache statistics in `batch-report.json`. Add
`--generate-plugin` to write one verified `generated-navmesh.esp`; omit it for
candidate inspection. Cells without existing NAVM or a nonempty candidate are
reported as skipped. Incompatible generated border partitions stop patch writing.
See [batch rebuilding](docs/batch-rebuilding.md) for the baseline contract,
outputs, caching and writer limitations.

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
- `navmesh-counts.txt` — the number of original navmesh polygons in the selected cell and the number of generated navmesh polygons. The same two counts are printed by the CLI and shown in the desktop UI when the run finishes.
- `navmesh.obj` — exported navmesh polygon geometry for inspection in Blender or MeshLab.
- `geometry.obj` — exported selected support geometry: packed Havok collision when available for a reference, otherwise a render fallback, plus decoded exterior `LAND` terrain.
- `geometry.json` — machine-readable triangle provenance plus geometry-coverage failures. Every triangle has `sourceType` (`terrain`, `collision`, or `render_fallback`), `collisionType`, and confidence. Terrain triangles also name their winning `LAND` record, exterior cell, and lower-left VHGT sample coordinate.
- `scene.glb` — one combined, color-layered scene with named Existing NAVM, terrain, collision, render-fallback, and diagnostic-marker objects. `scene.glb.provenance.json` maps each named object to its source record/model and preserves run metadata; `scene.glb.metadata.json` is the standard metadata sidecar.
- `scene-report.html` — standalone classification report grouped by support-source and geometry-coverage status. Its support-triangle indices join `analysis.json` to `geometry.json`; this report does not change classifications.
- `analysis.json` — stable, report-only discrepancy evidence: seven-point polygon coverage, selected source type and confidence, explicit `ambiguous`/`out_of_coverage` states, topology findings, and manual-review repair candidates. It never contains replacement NAVM geometry or a plugin write instruction.
- With `--generate-candidate`, Recast creates a neutral candidate NAVM from supported terrain and collision. `candidate-navm.json` contains profile parameters, approximate source-triangle provenance, regions, matched door triangles, reciprocal border-link matches, adjacency, statistics, and topology validation. `candidate-navm.obj` is a mesh view, and `scene.glb` gains a blue-green Candidate NAVM layer beside the existing scene layers. These files do not contain plugin records.
- `input-report.json` — MO2 profile snapshot and virtual-file winners, emitted first for every MO2 run.
- `load-order.json` — when using `--load-order`, every winning record with its plugin and ordered origin chain.
- With `--generate-plugin`, `generated-navmesh.esp` overrides the selected cell's existing NAVM records. The ESP receives the ESL flag when its override-only records fit a light plugin. This option also generates candidate exports. The writer refuses to replace an existing output file and verifies every NAVM by reading it back.

Every JSON export contains a versioned `metadata` block. OBJ and HTML exports have an adjacent `<export>.metadata.json` sidecar. The metadata identifies the tool version, input plugin, selected cell, coordinate convention, source coverage, and known limitations. Its schema is [docs/schemas/export-metadata.schema.json](docs/schemas/export-metadata.schema.json); coordinate details are in [docs/coordinate-system.md](docs/coordinate-system.md).

The combined-scene workflow, material legend, provenance joins, and Online 3D Viewer instructions are in [docs/scene-inspection.md](docs/scene-inspection.md). For scalable exterior scenes, use `--neighboring-cell-radius <cells>`, `--scene-bounds <minX> <minY> <maxX> <maxY>`, `--geometry-layers <navmesh,terrain,collision,render,diagnostics>`, and `--output-detail <full|summary>`. The Windows UI exposes **Neighboring cells**. Analysis uses the requested radius; generation always includes adjacent geometry and clips output to the selected CELL. `--export-scene <path>` changes the default `scene.glb` location. Neighboring cells are appended one at a time and bounds filtering uses a coarse world-space scene grid before GLB buffers are created.

To generate an inspection candidate from the selected cell, add `--generate-candidate`. Candidate generation uses the human navigation profile. Recast rasterizes supported terrain and collision, filters slopes and clearance, erodes walkable spans by the profile's agent radius, and creates polygonal regions. Region partitioning defaults to watershed; choose `--partitioning-algorithm monotone` or `--partitioning-algorithm layers` to use Recast's other strategies. The Windows UI provides the same selector and saves it with the other options. The selected strategy is recorded in `candidate-navm.json`. Voxel resolution adapts to the extracted area's size. Contours allow two horizontal voxels of simplification error without forced edge subdivision, and Recast builds triangles directly to avoid long triangle fans from larger polygons. Regions below the profile's minimum region area are removed; watershed and monotone can merge small adjacent regions, while layers does not use the merge threshold. Candidate output is clipped to the selected exterior CELL independently of its neighboring geometry suppliers. Only connected components reaching an enabled DOOR entrance or the extracted exterior cell border appear in the candidate. With no reachable anchor, the candidate is empty and carries a warning. With a resolved load order and a selected exterior cell, matching adjacent NAVM edges are joined to the candidate border and recorded in `border_links`; regions without a matched border portal or door are removed. Candidate triangles and orange entrance markers appear in the scene GLB. Source-triangle joins are approximate because voxelization does not retain input triangle IDs. See [the current algorithm and historical comparison](docs/candidate-surface-algorithm.md) for scope and limits.

To write a plugin, use a resolved MO2 profile or the developer load-order route, select a cell or batch rebuild scope, and add `--generate-plugin`. The desktop UI has the same **Write plugin** option; neighboring geometry remains available during generation. The writer puts the generated geometry in the selected cell's largest existing NAVM and replaces its other existing NAVMs with empty overrides. It retains their group hierarchy and includes the source plugins and their masters as dependencies. Parent CELL and worldspace records remain supplied by the load order. Install the generated ESP after its source plugins. It is ESL-flagged when its master table fits the light format, even when a source is a regular ESP. Matched entrances become NAVM door links, and matched cell-border edges become reciprocal external links in overrides of the adjacent NAVMs. A border link requires compatible full edges. Border-only regions without a matching adjacent NAVM edge are removed from the candidate before plugin export. Other authored connections, cover data, NAVI, and teleport-door XNDP references are not rebuilt; those can still prevent NPC navigation. Inspect the plugin in independent tooling and validate navigation on a disposable game profile before use.

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
- Some Skyrim record variants and non-standard modded data layouts may still be rejected or reported as unsupported. The writer does not copy localized parent records into the generated plugin.
- Discrepancy detection is deliberately conservative. Collision support has priority over terrain, terrain has priority over render fallback, and only sufficiently consistent samples can classify a polygon. Ambiguous or out-of-coverage polygons are displayed as limitations rather than defects. Repair candidates remain report-only/manual-review evidence.

The verified NVNM layout and writer acceptance rules are documented in [docs/navm-format-study.md](docs/navm-format-study.md).

## Next milestone

The next roadmap milestone is:

```text
candidate NAVM -> conservative repair planning
```

not runtime integration.

See [docs/architecture.md](docs/architecture.md) for the pipeline and separation between parsing, extraction, neutral model, analysis, and guarded NAVM serialization.

## Source documentation

Project-owned C++ follows the repository `.clang-format` and the readability and
SOLID rules in [AGENTS.md](AGENTS.md). Prefer functions with a coherent
responsibility and explain non-obvious algorithm stages and invariants. The
[architecture guide](docs/architecture.md) maps the runner, geometry composition,
inspection reporting, generation, and analysis responsibilities.

To check formatting from PowerShell with clang-format available on PATH:

```powershell
clang-format --dry-run --Werror (rg --files src -g '*.cpp' -g '*.h')
```

Use a debug build to run the assert-based C++ regression suite, so its checks and
fixture setup execute:

```powershell
xmake f -m debug
xmake build navmesh-tests
xmake run navmesh-tests
```

The C++ API reference is generated with Doxygen. Install Doxygen and run
`doxygen Doxyfile` from the repository root, then open
`build/doxygen/html/index.html`. The source guide is in [docs/api.md](docs/api.md).
The generated site covers project-owned `src/` code and linked project guides;
it does not include vendored dependencies.
