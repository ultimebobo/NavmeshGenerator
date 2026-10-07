# Command-line and export reference

This guide covers current commands, input resolution, exports, and implementation limits. For the desktop quick start, see [the README](../README.md). For build and dependency setup, see [development](development.md).

## Example

Launch the executable without switches for the dark Dear ImGui desktop workspace.
Select an existing MO2 folder and profile, choose a rebuild scope, and choose an
output folder. **Cell** shows only the selected Form ID or editor ID input;
**Plugin** shows the affected plugin and **Copy selected plugin**. Batch output
and estimation appear for Plugin and Load order scopes. **List cells to file**
is a separate action that writes `cells.json` without requiring a target.

**Advanced settings** starts collapsed. It contains numerical analysis and
performance controls, plus Recast agent, voxel, contour and region inputs when
generation is enabled. Choices are persisted in the local application settings;
hidden target/export choices are excluded from the run. The desktop uses MO2
input and standard artifact paths. Direct plugin/Data/manifest input, custom
OBJ paths, exterior coordinates, diagnostic HTML and terrain-only switches
remain CLI developer/inspection options. See the
[desktop workflow](operator-workflow.md#desktop-workspace) and
[Recast settings](operator-workflow.md#advanced-generation-settings).

```powershell
./build/windows/x64/releasedbg/NavmeshGenerator.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --cell-formid 00027D1C --output ./output
```

These paths and cell selections are examples; use identifiers from your own
profile's cell catalog. The tool prints the resolved cell and also accepts an
editor ID or exterior cell coordinate pair:

```powershell
./build/windows/x64/releasedbg/NavmeshGenerator.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --editor-id KilkreathRuins03 --output ./output
```

```powershell
./build/windows/x64/releasedbg/NavmeshGenerator.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --cell-x 10 --cell-y -5 --output ./output
```

Use `--list-cells` to discover available form IDs, editor IDs, and exterior coordinates:

```powershell
./build/windows/x64/releasedbg/NavmeshGenerator.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --list-cells --output "D:/NavmeshWork/cells"
```

## Rebuild a plugin or load order

Use the persisted **Rebuild scope** selector in the Windows UI, or add one of
these options to an MO2/profile command:

```powershell
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-plugin "<active plugin.esp>" --generate-plugin --output "<new output folder>"
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-load-order --generate-plugin --output "<new output folder>"
```

Plugin scope finds cells with terrain, water or collision changes in that plugin's edit history;
load-order scope considers changes after the first active baseline plugin and
MO2 model replacements. Selection includes historical moved/deleted placements,
changed base-object uses and persistent references. Visual-only assets and NAVM-only
edits do not select rebuilding targets. Actual changed collision triangles determine
exterior coverage; unchanged collision and metadata do not enlarge the scope.
New terrain and collision select uncovered cells, including new worldspaces and
submerged heightfields. Water-only changes require supported winning terrain or
exact model collision coverage; empty water planes do not become targets.
Existing NAVM is not required for generation in any scope.
Generation uses winning geometry from the full load order and neighboring cells,
while each generated NAVM stays clipped to its own target CELL. Oversized model
bounds extend geometry suppliers. Neighboring-cell radius controls generation
inputs without adding unchanged cells to the regeneration scope. Required model
evidence that cannot be read stops collision selection with a diagnostic.

The load order is resolved once, navigation-equivalent overrides are excluded,
and bounded caches reuse terrain, models, placements, and generated candidates.
`batch-report.json` checkpoints selection, timings, skips, and cache statistics.
Add `--generate-plugin` to write one verified `generated-navmesh.esp`; its automatic
batch output retains reports and the plugin. Choose `--batch-output compact` for
gzip candidate JSON or `--batch-output full` for candidate JSON/OBJ beneath
`cells/<FormID>/` and the complete winning-record report. Omit plugin writing for
candidate inspection. `--estimate-only` samples the selected scope before a full
rebuild and caches the sampled work. Shared cache location and disk retention,
working-memory admission, and worker settings are exposed and persisted in the
desktop UI; see [performance improvements](performance-improvements.md).
Every selected live batch cell generates navigation, including cells with no NAVM.
Workers retain every surviving walkable component until all targets are ready;
shared generated seams then receive matching partitions and reciprocal links.
Only untouched neighbors provide authored border constraints. Valid empty targets
clear replaced navigation where no supported floor survives. Topology and linking
failures stop export with a CELL-specific diagnostic.
See [batch rebuilding](batch-rebuilding.md) for the baseline contract,
outputs, caching and writer limitations.

For mod authoring, select **Plugin** rebuild scope and enable **Copy selected
plugin** in the Windows UI, or use `--copy-plugin`:

```powershell
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-plugin "<active plugin.esp>" --copy-plugin --output "<new output folder>"
```

This option enables plugin writing and creates `<output>/<selected filename>`
with generated navigation and the selected plugin's other records. It preserves
the filename, extension, TES4 flags, master order, and encoded non-NAVM records,
including unknown and compressed records. New NAVM identities avoid all existing
source identities and must fit the source's full/light format. Existing outputs
and input plugins are never overwritten. The UI saves this setting.

Use the exported copy **in place of** the selected plugin, with its original
assets and localization resources. It is a replacement plugin, while the
default `--generate-plugin` export remains a separate NAVM-only patch. Copy mode
requires Plugin scope and rejects generated references to plugins outside the
source's existing master table. Winning geometry still comes from the full
resolved load order. NAVI, XNDP, cover, and unmatched authored-link limitations
also apply to the copy; retained records do not imply rebuilt navigation data.

Enable **Skip cells with existing navmesh** in the Windows UI, or add
`--skip-existing-navmesh` to a generation command, to fill uncovered selected
cells when explicitly enabled. The setting is optional and saved
with the other UI options. It applies to Cell, Plugin, and Load order scopes and
requires resolved MO2 or load-order input. Any winning NAVM record protects its
cell, including empty, unsupported, and deleted records. Batch reports identify
these cells as `skipped_existing_navm`; a skipped single cell produces
`generation-report.json` and completes without extraction or generation.

```powershell
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-plugin "<active plugin.esp>" --generate-plugin --skip-existing-navmesh --output "<new output folder>"
```

Uncovered cells receive new plugin-owned NAVM records. Matched borders can add
reciprocal links to authored neighbors while retaining their vertices and
triangles. Batch scopes also link new cells to each other's generated meshes and
retain unanchored walkable floors. In Cell scope, unmatched seam wedges retract,
components without a portal or door are removed, and empty candidates are skipped.

## Mod Organizer 2 input

The intended normal workflow is to select an existing Mod Organizer 2 instance and profile. The tool will read the profile's existing mod priority, active plugins, and load order without asking you to create or edit a manifest:

```powershell
./build/windows/x64/releasedbg/NavmeshGenerator.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --list-cells --output "D:/NavmeshWork/cells"
```

The importer writes `input-report.json` before listing or extraction. It records the profile snapshot hash, enabled-mod priority, active plugin winners, and loose-file winners selected from game `Data`, enabled mods, and `Overwrite`. The `--data` plus `--load-order` route remains an internal developer/test interface only.

If an existing MO2 profile's configured mod storage was moved, use the read-only `--mods-dir` recovery override. It changes only where enabled mod folders are looked up; it never edits MO2 configuration or profile files:

```powershell
./build/windows/x64/releasedbg/NavmeshGenerator.exe --mo2 "C:/Users/baudo/AppData/Local/ModOrganizer/Mod Authoring" --profile "My Actual Profile" --mods-dir "D:/Dev/Skyrim Mods/Mods" --list-cells --output "D:/NavmeshWork/cells"
```

Split MO2 layouts are supported: when `ModOrganizer.ini` has a relative `base_directory`, profile/mod/overwrite paths resolve from that directory (for example, a sibling `MODS` tree), while `gamePath` resolves to the configured stock game directory.

The direct reader currently resolves its fixture-tested full/light identities and override chains, and reports missing masters/cycles/duplicates. Its deliberately narrow coverage is documented in [the parser evaluation](parser-library-evaluation.md).

## Output

The CLI writes files into the target output directory:

- `report.json` — cell metadata, references, navmesh summaries, and validation findings.
- `navmesh-counts.txt` — the number of original navmesh polygons in the selected cell and the number of generated navmesh polygons. The same two counts are printed by the CLI and shown in the desktop UI when the run finishes.
- `navmesh.obj` — exported navmesh polygon geometry for inspection in Blender or MeshLab.
- `geometry.obj` — exported selected support geometry: packed Havok collision when available for a reference, otherwise a render fallback, plus decoded exterior `LAND` terrain.
- `geometry.json` — machine-readable triangle provenance plus geometry-coverage failures. Every triangle has `sourceType` (`terrain`, `collision`, or `render_fallback`), `collisionType`, and confidence. Terrain triangles also name their winning `LAND` record, exterior cell, and lower-left VHGT sample coordinate.
- `scene.glb` — one combined scene ordered with the current cell’s original NAVM in its own group, neighboring and candidate NAVMs, green connection bars along linked triangle edges and exit markers, then terrain, collision, and render geometry. Door-linked triangles use a dedicated orange material; classification centroid markers are omitted. `scene.glb.provenance.json` maps each named object to its source record/model and preserves run metadata; `scene.glb.metadata.json` is the standard metadata sidecar.
- `scene-report.html` — standalone classification report grouped by support-source and geometry-coverage status. Its support-triangle indices join `analysis.json` to `geometry.json`; this report does not change classifications.
- `analysis.json` — stable, report-only discrepancy evidence: seven-point polygon coverage, selected source type and confidence, explicit `ambiguous`/`out_of_coverage` states, topology findings, and manual-review repair candidates. It never contains replacement NAVM geometry or a plugin write instruction.
- With `--generate-candidate`, Recast creates a neutral candidate NAVM from supported terrain and collision. Convex patch merging and movement-bounded height detail simplify stairs and terrain; landscape rock collision blocks movement without supplying walkable tops. Contour simplification automatically refines if a retained voxel region would collapse. `candidate-navm.json` contains profile parameters, approximate source-triangle provenance, obstacle tags, regions, matched door triangles, reciprocal border-link matches, adjacency, statistics, and topology validation. `candidate-navm.obj` is a mesh view, and `scene.glb` gains a blue-green Candidate NAVM layer beside the existing scene layers. These files do not contain plugin records.
- `input-report.json` — MO2 profile snapshot, emitted first for every MO2 run. Compact/plugin-only batches reference a shared loose-asset catalog; full inspection includes every winner.
- `load-order.json` — full inspection of resolved inputs: every winning record with its plugin and ordered origin chain. Compact/plugin-only batches omit this table.
- With `--generate-plugin`, `generated-navmesh.esp` overrides existing NAVM records or allocates new records for uncovered cells. The ESP receives the ESL flag when its master table and new identities fit a light plugin. Single-cell writing also generates candidate exports; batch writing follows `--batch-output`. The writer refuses to replace an existing output file and verifies every NAVM by reading it back.
- With `--copy-plugin` in Plugin scope, the selected plugin's original filename is used for a complete copy with generated NAVMs. Source flags and master indices are preserved. `batch-report.json` records `copy_plugin`; when no eligible navigation is generated, no plugin is written.

Every JSON export contains a versioned `metadata` block. OBJ and HTML exports have an adjacent `<export>.metadata.json` sidecar. The metadata identifies the tool version, input plugin, selected cell, coordinate convention, source coverage, and known limitations. Its schema is [docs/schemas/export-metadata.schema.json](schemas/export-metadata.schema.json); coordinate details are in [docs/coordinate-system.md](coordinate-system.md).

The combined-scene workflow, material legend, provenance joins, and Online 3D Viewer instructions are in [docs/scene-inspection.md](scene-inspection.md). For scalable exterior scenes, use `--neighboring-cell-radius <cells>`, `--scene-bounds <minX> <minY> <maxX> <maxY>`, `--geometry-layers <navmesh,terrain,collision,render,diagnostics>`, and `--output-detail <full|summary>`. The `diagnostics` scene layer shows only exit markers, including cave entrances identified by teleport metadata or authored NAVM door associations. The Windows UI exposes **Neighboring cells**. Analysis uses the requested radius; generation always includes adjacent geometry and clips output to the selected CELL. Terrain and authored NAVM stay within the scene neighborhood. Distant model suppliers contribute only support and display triangles whose bounds intersect that neighborhood. Terrain-only extraction visits only the scene cells. `--export-scene <path>` changes the default `scene.glb` location. Neighboring cells are appended one at a time and bounds filtering uses a coarse world-space scene grid before GLB buffers are created.

To generate an inspection candidate from the selected cell, add `--generate-candidate`. Candidate generation uses the human navigation profile. Recast rasterizes supported terrain and collision, filters slopes and clearance, erodes walkable spans by the profile's agent radius, and creates polygonal regions. Region partitioning defaults to watershed; choose `--partitioning-algorithm monotone` or `--partitioning-algorithm layers` to use Recast's other strategies. The Windows UI provides the same selector and saves it with the other options. The selected strategy is recorded in `candidate-navm.json`. Voxel resolution adapts to the extracted area's size. Contours use the configured simplification error and edge subdivision settings, and Recast builds triangles directly to avoid long triangle fans from larger polygons. Regions below the profile's minimum region area are removed; watershed and monotone can merge small adjacent regions, while layers does not use the merge threshold. Candidate output is clipped to the selected exterior CELL independently of its neighboring geometry suppliers, with bounded extensions at matched authored border portal endpoints. Generation provisionally retains shared-edge components reaching a matched door or the selected exterior CELL border. Final stitching retains only components reaching a real neighboring portal or matched door. Unreachable roof and stone-top islands are removed regardless of area; surfaces merely near a border do not qualify. Interiors without a matched door produce empty candidates. Filtering preserves source evidence and remaps geometry and door indices, with discarded triangles recorded in `rejected_unreachable`. Exterior rasterization includes a supported neighboring halo before the generated mesh is clipped to the selected CELL, so radius erosion does not treat the CELL seam as an obstacle. With a resolved load order and a selected exterior cell, matching adjacent NAVM edges are joined to the candidate border and recorded in `border_links`. Stitching can coalesce compatible collinear generated subdivisions and split containing edges to match the neighbor's complete edges. Terminal seam endpoints extend through their incident fans, and unmatched seam wedges retract into the CELL. Every remaining seam must have a unique portal with exact reversed neighboring endpoints. Inward offsets trim generated fans and outward offsets use bounded extensions; distance, height, slope, and topology constrain the result. Candidate triangles and orange entrance markers appear in the scene GLB. Source-triangle joins are approximate because voxelization does not retain input triangle IDs. See [the current algorithm and historical comparison](candidate-surface-algorithm.md) for scope and limits.

To write a plugin, use a resolved MO2 profile or the developer load-order route, select a cell or batch rebuild scope, and add `--generate-plugin`. The desktop UI has the same **Write plugin** option; neighboring geometry remains available during generation. The writer puts the generated geometry in the selected cell's largest existing NAVM and replaces its other existing NAVMs with empty overrides. It retains their group hierarchy and includes the source plugins and their masters as dependencies. Parent CELL and worldspace records remain supplied by the load order. Install the generated ESP after its source plugins. It is ESL-flagged when its master table and newly allocated identities fit the light format, even when a source is a regular ESP. Matched entrances become NAVM door links, and matched cell-border edges become reciprocal external links in overrides of the adjacent NAVMs. In Cell scope, each portal matches a complete authored edge; containing generated edges can be subdivided to fit it. Small authored deviations from the nominal CELL boundary are accepted within a shared tolerance; matched endpoints preserve the exact neighboring edge, and other generated vertices stay inside the CELL. Every authored NAVM with an incoming portal to replaced geometry receives an override. Those entries are removed and retained portal indices are remapped before matched borders receive fresh reciprocal links to generated triangles. Unmatched incoming edges remain open boundaries. Read-back validation checks portal indices and emitted destination triangle ranges. Cell-scope components without real neighboring portals or matched doors are removed.
Batch scopes retain all surviving floors and join generated neighbors after all cells are ready. Empty single-cell plugin candidates complete with skipped_empty_candidate in generation-report.json and no patch. Scene previews replace authored bars involving the original cell NAVMs with generated bars. Other authored connections, cover data, NAVI, and teleport-door XNDP references are not rebuilt; those can still prevent NPC navigation. Inspect the plugin in independent tooling and validate navigation on a disposable game profile before use.

Record the command inputs using [docs/run-manifest.example.json](run-manifest.example.json) before a benchmark. The current CLI does not consume this file; map its fields to the existing command-line flags so milestone 0 does not alter parser input behavior.

For measured plugin-rebuild costs and prioritized ways to reduce runtime and disk
usage, see [implemented improvements](performance-improvements.md) and the
[before/after snapshot](performance-improvements-measurements.json). The
[original analysis](performance-analysis.md) preserves the baseline investigation.

## Generated triangle tagging

Generation enables water and preferred-path tagging by default in Cell, Plugin
and Load order scopes. Use `--no-triangle-tagging` to disable both. The saved
desktop setting is **Tag water and preferred path triangles** in Advanced settings.

Water detection compares each final triangle's centroid with the effective
exterior CELL water surface. The parser requires the CELL water flag and resolves
explicit heights or default heights from winning worldspace water data, including
parent-world inheritance. Invalid heights supply no plane. Interiors and the
direct single-plugin reader have no worldspace-plane inference; explicit exterior
heights remain usable in the direct reader. Without a supported plane, matching
authored water markings supply evidence.

Preferred paths come from the closest overlapping winning authored triangle at
a compatible height. Unmarked floors participate in matching to keep preference
from transferring between stacked surfaces. Matching tolerance follows the
movement profile and vertical voxel resolution. Deleted or malformed authored
triangles are ignored; equally close contradictory markings are conservative.
This preserves authored route intent; it does not invent routes in areas without
authored preference evidence. Centroid classification is approximate near shorelines
and preference boundaries; it does not split generated triangles at those boundaries.

Candidate JSON records `triangle_tagging` and per-polygon `flags`, `water`, and
`preferred_path`. Plugins serialize both independent bits, which may coexist.
Disabling tagging leaves geometry and connection flags intact. Cache fingerprints
include the switch, effective water height, and winning selected-cell NAVMs.

## Reproducible fixtures and optional game checks

The repository contains only synthetic, redistributable fixture builders; it does not include Skyrim assets. See [fixtures/README.md](../fixtures/README.md) for the policy and [docs/benchmarks.json](benchmarks.json) for the local benchmark reference manifest. To opt into the read-only local-game parser smoke test, configure `SKYRIM_DATA_DIR` as described in [tests/integration/README.md](../tests/integration/README.md). Without it, the complete test suite still runs and skips that check.

## Current limitations

- This is not an SKSE runtime plugin and does not inspect a running game session.
- The direct parser is intentionally small and targets the common Bethesda plugin structure, not the entire plugin ecosystem.
- Compressed indexed records are zlib-decoded with declared-size and boundary checks while their original bytes remain retained. Malformed records and unknown NAVM versions are explicit diagnostics, never best-effort geometry.
- Geometry extraction resolves MO2's winning loose NIFs from enabled mods and Overwrite, then caches requested meshes from enabled mod and game BSAs on demand. This includes meshes supplied by a separate resources mod. The supported collision subset is reachable `bhkMoppBvTreeShape`/`bhkListShape` wrappers containing packed strips, `bhkNiTriStripsShape`, or Skyrim SE `bhkCompressedMeshShape` chunks; their triangle vertices receive the Havok rigid-body transform, then reference scale, Skyrim's placed-reference rotation matrix from the `DATA` angles (radians), and translation. A positive reference Z angle rotates model-local +X toward world -Y. Other Havok primitives are reported as unsupported rather than guessed. Render meshes are retained only as lower-confidence fallback when no supported collision exists. Winning placed records flagged initially disabled or deleted are excluded from both geometry layers and reported in coverage. Effects, furniture, actors, and path-marked animated/FX references are excluded by policy.
- Archived NIFs are read by the native BSA reader and cached on demand. The application needs no Python interpreter, extractor script, or project working directory for archived assets. Source builds resolve zlib and LZ4 through xmake; see [BSA performance](bsa-performance.md).
- Archived assets and private candidate evidence use a shared cache, configurable with `--asset-cache` and bounded by `--cache-budget-mib`. Existing legacy output caches remain untouched. See [cache lifetime and output modes](performance-improvements.md).
- Exterior `LAND` decoding supports the collision-relevant VHGT height grid only, positioned in Skyrim world coordinates. It intentionally excludes visual LOD and texture layers. Use `--terrain-only` with a resolved load order to export only that terrain diagnostic surface. Missing or malformed `LAND` records are reported and produce no replacement plane.
- Some Skyrim record variants and non-standard modded data layouts may still be rejected or reported as unsupported. NAVM-only patches leave localized parent records in the load order; plugin copies preserve source records and require the original localization resources.
- Discrepancy detection is deliberately conservative. Collision support has priority over terrain, terrain has priority over render fallback, and only sufficiently consistent samples can classify a polygon. Ambiguous or out-of-coverage polygons are displayed as limitations rather than defects. Repair candidates remain report-only/manual-review evidence.

The verified NVNM layout and writer acceptance rules are documented in [docs/navm-format-study.md](navm-format-study.md).

