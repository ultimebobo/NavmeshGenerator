# Command-line and export reference

This guide covers current commands, input resolution, exports, and implementation limits. For the desktop quick start, see [the README](../README.md). For build and dependency setup, see [development](development.md).

## Example

Launch the executable without switches for the dark Dear ImGui desktop workspace.
Select MO2 and its existing folder/profile, or Vortex and its deployed Skyrim
installation folder, then choose a rebuild scope and an
output folder. **Cell** shows a multiline **Cells** field accepting Form IDs
and editor IDs. Its **Copy selected plugin** option reveals **Plugin to copy**
with a filename input and Browse button. **Plugin** shows the affected plugin and
**Copy selected plugin**. Generation lists and affected-cell scopes expose batch
output and estimation. **List cells to file** writes `cells.json` without requiring
a target.

**Advanced settings** starts collapsed. It contains numerical analysis and
performance controls, plus Recast agent, voxel, contour and region inputs when
generation is enabled. Choices are persisted in the local application settings;
hidden target/export choices are excluded from the run. The desktop uses the selected
mod manager and standard artifact paths. MO2 is the initial default; the radio
button choice is saved. Direct plugin/Data/manifest input, custom
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

BOS configurations are discovered automatically from effective Data, including
MO2 configuration-only mods and overwrite or Vortex deployed files. Both base
replacements and placement properties contribute to geometry. Load order scope
includes their collision impacts, and Cell scope uses effective placements.
Random outcomes use conservative filled obstruction volumes and cannot supply
floors. `bos-input.json` records winners and alternatives; keep those configs
installed with the output patch. See [BOS inputs](base-object-swapper.md) for
precedence, units and offline limitations.

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
Final reachability removes isolated floor networks, including roofs spanning CELL
seams. Matched doors and authored neighbors provide access; each contiguous
selected exterior area or interior also retains its largest connected floor network.
Only untouched neighbors provide authored border constraints. Valid empty targets
clear replaced navigation where no supported floor survives. Cell-local generation
and topology failures are logged as `skipped_generation_failed` and leave
authored geometry intact while other targets continue. `complete_with_skips` and
`generation_failed_cells` identify such completion in `batch-report.json`.
Global linking and writer failures still stop export.
The patch filename gains a numeric suffix if its default name is already active
in the load order, so an earlier generated plugin can remain a valid dependency.
Selected persistent worldspace containers are logged as `skipped_persistent_cell`;
their placed objects remain geometry suppliers for ordinary terrain cells.
See [batch rebuilding](batch-rebuilding.md) for the baseline contract,
outputs, caching and writer limitations.

For mod authoring, select **Plugin** rebuild scope and enable **Copy selected
plugin** in the Windows UI, or use `--copy-plugin`:

```powershell
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-plugin "<active plugin.esp>" --copy-plugin --output "<new output folder>"
```

For a chosen set of cells, use **Cell** mode and paste identifiers into **Cells**. The CLI accepts `--cells` with whitespace, comma or semicolon
separators, and repeated lists are combined:

```powershell
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --cells "<cell Form ID>,<cell editor ID>" --copy-plugin-source "<active plugin.esp>" --copy-plugin --output "<new output folder>"
```

Each identifier must uniquely match a resolved hexadecimal Form ID or editor ID
from `cells.json`; aliases are deduplicated before processing. These cells need
not be affected by the copied plugin. `--copy-plugin-source` names the copy
template without discovering additional targets or changing Cell scope. Without
copying, omit the template filename. Use `--generate-plugin` for a navigation patch or
`--generate-candidate` for generated previews. Without either generation option,
Cell mode inspects the list and writes multiple results beneath `cells/<FormID>/`.
A single inspected cell retains the standard output paths. Unknown or ambiguous
identifiers fail selection before processing. Single-cell selectors,
`--rebuild-plugin`, `--rebuild-load-order` and `--list-cells` cannot be combined
with `--cells`. Batch skip, estimation, output and resource settings apply to
multi-cell generation and copy exports.

Add `--make-scene` to a Cell-mode command to write one combined `<output>/scene.glb`
with adjacent provenance and metadata JSON. It works with `--cells`, single-cell
selectors, candidate generation and plugin copies, including `plugin_only` output.
Generated meshes use their finalized seams, and shared source geometry is included
once. For example:

```powershell
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --cells "<cell identifiers>" --make-scene --output "<output folder>"
```

This flag does not enable generation. It requires Cell scope and cannot combine
listing or cost estimation. Single-cell inspection emits its standard scene
automatically. Independent interiors/worldspaces keep native coordinates and can
overlap in the scene. Layer and provenance detail selections apply to the combined
scene; custom export paths and bounds remain single-cell controls.

This option enables plugin writing and creates `<output>/<selected filename>`
with generated navigation and the selected plugin's other records. It preserves
the filename, extension, TES4 flags, master order, and encoded non-NAVM records,
including unknown and compressed records. New NAVM identities avoid all existing
source identities and must fit the source's full/light format. Existing outputs
and input plugins are never overwritten. The UI saves this setting.

Use the exported copy **in place of** the selected plugin, with its original
assets and localization resources. It is a replacement plugin, while the
default `--generate-plugin` export remains a separate navigation patch. Copy mode
requires Cell or Plugin scope and rejects generated references to plugins outside the
source's existing master table. Winning geometry still comes from the full
resolved load order. Both exports finalize navigation metadata and affected door
references, then validate their effective graph. Unsupported remapping stops
export; see the [finalization contract](navm-format-study.md#cover-and-door-finalization).

Enable **Skip cells with existing navmesh** in the Windows UI, or add
`--skip-existing-navmesh` to a generation command, to fill uncovered selected
cells when explicitly enabled. The setting is optional and saved
with the other UI options. It applies to Cell, Plugin, and Load order scopes and
requires resolved MO2, Vortex or load-order input. Any winning NAVM record protects its
cell, including empty, unsupported, and deleted records. Batch reports identify
these cells as `skipped_existing_navm`; a skipped single cell produces
`generation-report.json` and completes without extraction or generation.

```powershell
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-plugin "<active plugin.esp>" --generate-plugin --skip-existing-navmesh --output "<new output folder>"
```

Uncovered cells receive new plugin-owned NAVM records. Matched borders can add
reciprocal links to authored neighbors while retaining their vertices and
triangles. Batch scopes also link new cells to each other's generated meshes. All scopes
retain the largest floor network in each contiguous selected area or interior,
plus routes reaching matched doors or authored neighbors. Other isolated islands
are removed after linking, including when their roofs span CELL seams.
In single-cell generation, unmatched seam wedges retract and the largest connected
floor survives even without authored navigation or doors. Candidates with no
surviving walkable floor are skipped when writing a single-cell patch.

## Mod Organizer 2 input

The intended normal workflow is to select an existing Mod Organizer 2 instance and profile. The tool will read the profile's existing mod priority, active plugins, and load order without asking you to create or edit a manifest:

```powershell
./build/windows/x64/releasedbg/NavmeshGenerator.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --list-cells --output "D:/NavmeshWork/cells"
```

The importer writes `input-report.json` before listing or extraction. It records the profile snapshot hash, enabled-mod priority, active plugin winners, and loose-file winners selected from game `Data`, enabled mods, and `Overwrite`. The `--data` plus `--load-order` route remains an internal developer/test interface only.

Mod storage is resolved automatically from `ModOrganizer.ini`: `mod_directory`
takes precedence, with `mods_directory` and the instance layout as fallbacks.
The desktop uses this configured path. The read-only `--mods-dir` option is a
developer/test escape hatch that overrides physical mod lookup without editing
MO2 configuration or profile files.

Split MO2 layouts are supported: when `ModOrganizer.ini` has a relative `base_directory`, profile/mod/overwrite paths resolve from that directory (for example, a sibling `MODS` tree), while `gamePath` resolves to the configured stock game directory.

The direct reader currently resolves its fixture-tested full/light identities and override chains, and reports missing masters/cycles/duplicates. File-local FormID prefixes index the containing plugin's master table; indices at or beyond its end name that plugin, matching [xEdit's identity conversion](https://github.com/TES5Edit/TES5Edit/blob/dev-4.1.6/Core/wbImplementation.pas). An unresolved-reference diagnostic names the containing file, the reference's hexadecimal source-local FormID, the containing record's source-local and load-order identities, and the missing master; it does not mean the target object belongs to that file or is absent from it. Null or unresolved base identities supply no base object. Its deliberately narrow coverage is documented in [the parser evaluation](parser-library-evaluation.md).

## Vortex input

Activate the intended profile in Vortex and deploy its mods before running.
Choose **Vortex** in the desktop input card and select the Skyrim installation
folder containing Data. The CLI equivalent is:

```powershell
NavmeshGenerator.exe --vortex "<Skyrim installation>" --list-cells --output "<output folder>"
NavmeshGenerator.exe --vortex "<Skyrim installation>" --rebuild-plugin "<active plugin.esp>" --generate-plugin --output "<new output folder>"
```

The active game's `plugins.txt` is read from the standard Skyrim SE local
application data directory by default. For another store edition or a custom
location, use **Game application data folder** in the desktop, or add
`--vortex-plugins "<folder containing active plugins.txt>"`. The installation
folder and plugin directory must describe the same active game. Vortex's game
application data folder action can locate this directory. No profile name,
saved-profile directory or staging path is required. Vortex input cannot be
combined with MO2/profile or developer plugin/Data/manifest/mod-directory inputs.

SSE's `plugins.txt` supplies both activation markers and plugin ordering.
Installed native masters and installed `Skyrim.ccc` entries load first even if
omitted from that list. Plugin names are resolved case insensitively against
deployed Data. Loose model files are deployed winners; base-game BSAs load
before archives attached to active plugins in plugin order. Inactive plugin
archives are excluded. Custom INI archive lists are not imported.

Every Vortex run writes `input-report.json` before record resolution, with the
source directories, active plugin paths, archive order, loose model winners,
snapshot identity, diagnostics and limitations. Missing lists, undeployed active
plugins, duplicates and invalid names fail import. The runner rejects changes
to the plugin-list contents or deployed file membership, size or modification
times observed during load-order resolution.

Cell, Plugin and Load order operations use the same generation and writer path
as MO2. Deployed Data does not retain staging-mod ownership or overwritten loose
providers. Automatic impact selection therefore cannot attribute loose-only
replacements to a selected mod; choose their cells explicitly with Cell scope.
The tool reads the current deployment and does not switch or deploy Vortex profiles.

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
- With `--generate-candidate`, Recast creates a neutral candidate NAVM from supported terrain and collision. Convex patch merging and movement-bounded height detail simplify stairs and terrain; landscape rock collision blocks movement without supplying walkable tops. Contour simplification automatically refines if a retained voxel region would collapse. If watershed contours remain incomplete at zero error, generation recovers with layer regions over the already-retained spans and records a candidate warning; the partitioning setting remains the requested strategy. `candidate-navm.json` contains profile parameters, approximate source-triangle provenance, obstacle tags, regions, matched door triangles, reciprocal border-link matches, adjacency, statistics, and topology validation. `candidate-navm.obj` is a mesh view, and `scene.glb` gains a blue-green Candidate NAVM layer beside the existing scene layers. These files do not contain plugin records.
- `input-report.json` — selected mod-manager snapshot, emitted first for MO2 and Vortex runs. Compact/plugin-only batches omit the full loose-winner table; MO2 reports reference its shared loose-asset catalog.
- `load-order.json` — full inspection of resolved inputs: every winning record with its plugin and ordered origin chain. Compact/plugin-only batches omit this table.
- With `--generate-plugin`, `generated-navmesh.esp` overrides existing NAVM records or allocates new records for uncovered cells. The ESP receives the ESL flag when its master table and new identities fit a light plugin. Single-cell writing also generates candidate exports; batch writing follows `--batch-output`. The writer refuses to replace an existing output file and verifies every NAVM by reading it back.
- With `--copy-plugin` in Cell or Plugin scope, the selected plugin's original filename is used for a complete copy with generated NAVMs. Source flags and master indices are preserved. `batch-report.json` records `copy_plugin`; when no eligible navigation is generated, no plugin is written.

Every JSON export contains a versioned `metadata` block. OBJ and HTML exports have an adjacent `<export>.metadata.json` sidecar. The metadata identifies the tool version, input plugin, selected cell, coordinate convention, source coverage, and known limitations. Its schema is [docs/schemas/export-metadata.schema.json](schemas/export-metadata.schema.json); coordinate details are in [docs/coordinate-system.md](coordinate-system.md).

The combined-scene workflow, material legend, provenance joins, and Online 3D Viewer instructions are in [docs/scene-inspection.md](scene-inspection.md). For scalable exterior scenes, use `--neighboring-cell-radius <cells>`, `--scene-bounds <minX> <minY> <maxX> <maxY>`, `--geometry-layers <navmesh,terrain,collision,render,diagnostics>`, and `--output-detail <full|summary>`. The `diagnostics` scene layer shows only exit markers, including cave entrances identified by teleport metadata or authored NAVM door associations. The Windows UI exposes **Neighboring cells**. Analysis uses the requested radius; generation always includes adjacent geometry and clips output to the selected CELL. Terrain and authored NAVM stay within the scene neighborhood. Distant model suppliers contribute only support and display triangles whose bounds intersect that neighborhood. Terrain-only extraction visits only the scene cells. `--export-scene <path>` changes the default `scene.glb` location. Neighboring cells are appended one at a time and bounds filtering uses a coarse world-space scene grid before GLB buffers are created.

To generate an inspection candidate from the selected cell, add `--generate-candidate`. Candidate generation uses the human navigation profile. Recast rasterizes supported terrain and collision, filters slopes and clearance, erodes walkable spans by the profile's agent radius, and creates polygonal regions. Region partitioning defaults to watershed; choose `--partitioning-algorithm monotone` or `--partitioning-algorithm layers` to use Recast's other strategies. The Windows UI provides the same selector and saves it with the other options. The selected strategy is recorded in `candidate-navm.json`. Voxel resolution adapts to the extracted area's size. Contours use the configured simplification error and edge subdivision settings, and Recast builds triangles directly to avoid long triangle fans from larger polygons. Regions below the profile's minimum region area are removed; watershed and monotone can merge small adjacent regions, while layers does not use the merge threshold. Candidate output is clipped to the selected exterior CELL independently of its neighboring geometry suppliers, with bounded extensions at matched authored border portal endpoints. Generation provisionally retains every surviving walkable component. After authored border matching, shared reachability retains the largest connected floor by horizontal area, plus components reaching a real neighboring portal or matched door. Other disconnected roof and floor islands are removed; open borders and vertex-only contact do not supply access anchors. Interiors without a matched door can retain their largest floor. Filtering preserves source evidence and remaps geometry and door indices, with discarded triangles recorded in `rejected_unreachable`. Exterior rasterization includes a supported neighboring halo before the generated mesh is clipped to the selected CELL, so radius erosion does not treat the CELL seam as an obstacle. With a resolved load order and a selected exterior cell, matching adjacent NAVM edges are joined to the candidate border and recorded in `border_links`. Stitching can coalesce compatible collinear generated subdivisions and split containing edges to match the neighbor's complete edges. Terminal seam endpoints extend through their incident fans, and unmatched seam wedges retract into the CELL. Every remaining seam must have a unique portal with exact reversed neighboring endpoints. Inward offsets trim generated fans and outward offsets use bounded extensions; distance, height, slope, and topology constrain the result. Candidate triangles and orange entrance markers appear in the scene GLB. Source-triangle joins are approximate because voxelization does not retain input triangle IDs. See [the current algorithm and historical comparison](candidate-surface-algorithm.md) for scope and limits.

To write a plugin, use a resolved MO2 profile, active Vortex deployment or the developer load-order route, select a cell or batch rebuild scope, and add `--generate-plugin`. The desktop UI has the same **Write plugin** option; neighboring geometry remains available during generation. The writer puts the generated geometry in the selected cell's largest existing NAVM and replaces its other existing NAVMs with empty overrides. An uncovered cell receives a new plugin-owned NAVM in its temporary child group. It retains their group hierarchy and includes the source plugins and their masters as dependencies. Parent CELL and worldspace records remain supplied by the load order. Install the generated ESP after its source plugins. It is ESL-flagged when its master table and newly allocated identities fit the light format, even when a source is a regular ESP. Matched entrances become NAVM door links, and matched cell-border edges become reciprocal external links in overrides of the adjacent NAVMs. In Cell scope, each portal matches a complete authored edge; containing generated edges can be subdivided to fit it. Small authored deviations from the nominal CELL boundary are accepted within a shared tolerance; matched endpoints preserve the exact neighboring edge, and other generated vertices stay inside the CELL. Every authored NAVM with an incoming portal to replaced geometry receives an override. Those entries are removed and retained portal indices are remapped before matched borders receive fresh reciprocal links to generated triangles. Unmatched incoming edges remain open boundaries. Read-back validation checks portal indices and emitted destination triangle ranges. Single-cell generation applies the [candidate reachability policy](candidate-surface-algorithm.md): exterior entrances identify playable networks, with a primary-floor fallback for doorless selections and interiors.
Batch scopes provisionally retain all surviving floors and join generated neighbors after all cells are ready, then remove disconnected islands through complete-set reachability. Empty single-cell plugin candidates complete with skipped_empty_candidate in generation-report.json and no patch. Scene previews replace authored bars involving the original cell NAVMs with generated bars. NAVI, cover and reciprocal door XNDP associations are finalized automatically. The writer validates the effective navigation graph before publishing the ESP and refuses unsupported remapping. Load the patch after its input plugins; regenerate it when those inputs change. See the [finalization contract](navm-format-study.md#cover-and-door-finalization).

Record the command inputs using [docs/run-manifest.example.json](run-manifest.example.json) before a benchmark. The current CLI does not consume this file; map its fields to the existing command-line flags so milestone 0 does not alter parser input behavior.

For measured plugin-rebuild costs and prioritized ways to reduce runtime and disk
usage, see [implemented improvements](performance-improvements.md) and the
[before/after snapshot](performance-improvements-measurements.json). The
[original analysis](performance-analysis.md) preserves the baseline investigation.

## Cell border repair

Cell-scope border repair preserves required authored connections through terminal
boundary cavities and shared corner height offsets. Required cavities accommodate
the selected authored floor's slope envelope. A crossing onto explicitly excluded
obstacle collision closes with a warning only when no compatible generated boundary
floor survives and both authored interior anchor samples have positive collision
support. Missing models or unsupported geometry do not waive a crossing. The patch
removes incoming links to closed edges; inspect candidate warnings during review.

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
- Geometry extraction resolves MO2's winning loose NIFs from enabled mods and Overwrite, then caches requested meshes from enabled mod and game BSAs on demand. This includes meshes supplied by a separate resources mod. The supported collision subset is reachable `bhkMoppBvTreeShape`/`bhkListShape` wrappers containing packed strips, `bhkNiTriStripsShape`, Skyrim SE `bhkCompressedMeshShape` chunks, and closed `bhkConvexVerticesShape` vertex hulls (including convex-list wrappers); their triangle vertices receive the Havok rigid-body transform, then reference scale, Skyrim's placed-reference rotation matrix from the `DATA` angles (radians), and translation. A positive reference Z angle rotates model-local +X toward world -Y. Other Havok primitives are reported as unsupported rather than guessed. Render meshes are retained only as lower-confidence fallback when no supported collision exists. Winning placed records flagged initially disabled or deleted are excluded from both geometry layers and reported in coverage. Furniture, containers and static clutter retain supported collision as obstacles without supplying walkable floors. Recognized fire effects contribute separate navigation exclusions from fuel or flame geometry, including in batch extraction. Decorative effects, actors, operable doors and other path-marked animated/FX references are excluded by policy. See [the extraction and generation contracts](candidate-surface-algorithm.md).
- Archived NIFs are read by the native BSA reader and cached on demand. The application needs no Python interpreter, extractor script, or project working directory for archived assets. Source builds resolve zlib and LZ4 through xmake; see [BSA performance](bsa-performance.md).
- Archived assets and private candidate evidence use a shared cache, configurable with `--asset-cache` and bounded by `--cache-budget-mib`. Existing legacy output caches remain untouched. See [cache lifetime and output modes](performance-improvements.md).
- Exterior `LAND` decoding supports the collision-relevant VHGT height grid only, positioned in Skyrim world coordinates. It intentionally excludes visual LOD and texture layers. Use `--terrain-only` with a resolved load order to export only that terrain diagnostic surface. Missing or malformed `LAND` records are reported and produce no replacement plane.
- Some Skyrim record variants and non-standard modded data layouts may still be rejected or reported as unsupported. Navigation patches leave localized parent records in the load order; plugin copies preserve source records and require the original localization resources.
- Discrepancy detection is deliberately conservative. Collision support has priority over terrain, terrain has priority over render fallback, and only sufficiently consistent samples can classify a polygon. Ambiguous or out-of-coverage polygons are displayed as limitations rather than defects. Repair candidates remain report-only/manual-review evidence.

The verified NVNM layout and writer acceptance rules are documented in [docs/navm-format-study.md](navm-format-study.md).
