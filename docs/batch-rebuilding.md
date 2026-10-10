# Rebuilding affected cells

For terminology and the distinction between regenerated cell geometry and
neighboring connection updates, see the [navmesh glossary](glossary.md).

The shared application runner supports **Cell**, **Plugin**, and **Load order**
scopes. Cell accepts one or multiple explicit cells. The Windows UI persists the
scope and affected-plugin filename with the other settings. Affected-cell scopes
require a resolved MO2 profile or the developer load-order manifest. They generate candidates;
**Write plugin** / `--generate-plugin` additionally writes a combined patch.
Automatic batch output retains full inspection without writing, and reports plus
the plugin when writing. **Batch output** / `--batch-output` selects full,
compressed, or plugin-only artifacts explicitly.

In Cell scope, **Make scene** / `--make-scene` adds a combined `scene.glb` and its
sidecars regardless of batch output policy. Geometry is deduplicated across
overlapping neighborhoods; generated candidates are displayed after final seam
reconciliation, alongside authored navigation from skipped cells. This is a
complete-selection export and excludes cost estimation. Retaining requested scene
geometry adds memory beyond the generation admission budget. Scene write failures
stop publishing the plugin.

For authoring, **Copy selected plugin** / `--copy-plugin` enables writing and
uses the selected file as the output template. The UI persists the
option and exposes it in Cell and Plugin scopes. The result is written
under the source filename in the output folder and is intended to replace the selected
plugin while retaining its assets. It preserves encoded unrelated records,
TES4 flags, and existing master indices. Header accounting is updated; master
copies and copies with ONAM tables register generated overrides there while
retaining existing entries. Generated NAVMs replace source NAVMs by
identity or are inserted into their cell groups. New identities are allocated
after the source's allocation cursor and all source-owned record IDs, including
record types outside the parser's extraction index.

Copy mode cannot introduce dependencies outside the selected plugin's existing
master table, because arbitrary retained payloads cannot safely be rebased.
Generated references may name the selected plugin itself or its masters; other
owners stop export. New NAVMs must fit the source plugin's existing full/light
format. The copy retains its extension and flags rather than selecting a patch
format. Source plugins and existing output files are never overwritten. Every
generated or modified NAVM is read back before the copy is finalized.

NAVM-only patches use a filename distinct from every active input. A numeric
suffix is added when the default generated filename is already active, allowing
the patch to depend on earlier generated navigation without a self dependency.

To rebuild a chosen set rather than discover affected cells, use **Cell** mode
in the desktop and paste Form IDs or editor IDs into **Cells**. The
multiline list accepts whitespace, commas and semicolons and retains its entries
between sessions. Enable **Copy selected plugin** to show **Plugin to copy**, which
accepts an active filename or the Browse button and is required when copying.
The equivalent CLI is:

```powershell
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --cells "<cell identifier>,<another cell identifier>" --copy-plugin-source "<active plugin.esp>" --copy-plugin --output "<new output folder>"
```

`--cells` can be repeated. Each token is a resolved hexadecimal Form ID (with an
optional `0x` prefix) or a case-insensitive editor ID from `cells.json`. Every token
must resolve uniquely before any cell is processed. Multiple tokens naming the
same CELL are combined, and processing follows resolved CELL order. Explicit
selection includes unchanged cells and cells outside the chosen plugin's edit
history; `--copy-plugin-source` supplies the copy template in Cell mode.
`--rebuild-plugin` selects automatic affected-cell rebuilding and cannot combine
with `--cells`. Without copy mode, `--generate-plugin` writes a combined NAVM patch,
and `--generate-candidate` without writing produces candidate inspection artifacts.
Without generation, multiple cells receive the standard inspection exports beneath
`cells/<resolved FormID>/`; one cell uses the ordinary output paths. The input
snapshot is shared across these inspections. Single-cell selectors, affected-cell
rebuilding and listing cannot be combined with `--cells`.

Explicit targets use the shared batch generation, skip-existing, seam repair,
estimation and export policies. Neighboring cells supply geometry and may receive
reciprocal connection updates while retaining their authored geometry. Copy mode's
master-table and format constraints still apply to every generated reference.
`batch-report.json` identifies explicit generation as scope `cell`; impact-selection
counters do not report automatic impacts because this scope does not scan for
changed geometry.

```powershell
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-plugin "<active plugin.esp>" --copy-plugin --output "<new output folder>"
```

```powershell
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-plugin "<active plugin.esp>" --generate-plugin --output "<new output folder>"
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-load-order --generate-plugin --output "<new output folder>"
```

`--rebuild-plugin` accepts an active ESP, ESM, or ESL filename, case insensitive.
It identifies edits contributed by that plugin anywhere in a record's override
history. Geometry and NAVM always come from the complete winning load order,
including overrides after the selected plugin.
With MO2, loose model replacements from the selected plugin's mod folder and
archives named for that plugin also contribute conservative asset impacts.

`--rebuild-load-order` treats the first active input plugin as the baseline and
all subsequent plugins as potential changes. This includes additions from
master-flagged mods and official add-ons, conservatively. A baseline-only input
has no plugin edits to rebuild. The MO2 route also includes cells using replaced
loose model assets. Enabled mod archives can replace models without any record
edit; their winning NIF names are indexed to select affected uses. Index failures
conservatively include all model-bearing reference locations.

## Impact selection and geometry inputs

Selection compares supported terrain heights, effective water inputs and placed
collision across the selected record transitions. CELL ownership/display fields,
worldspace metadata, LAND color/texture edits and NAVM-only edits do not select
regeneration targets. Changed base records compare collision at their placed uses,
including untouched references. Render-only models, effects, excluded actor/furniture
classes and unchanged collision triangles do not expand selection.
Moved, disabled or deleted colliders select their changed historical and winning locations. Exterior
positions, including negative coordinates, determine physical cell ownership;
persistent references stored in a distant parent CELL are bucketed by their
world position. Worldspace identities keep overlapping coordinate grids separate.
Interiors remain independent targets.

Persistent worldspace CELL containers can carry placeholder grid coordinates.
They supply placed geometry through physical bucketing and are reported as
`skipped_persistent_cell` when selected; they do not receive generated NAVM or
authored border constraints. The ordinary terrain CELL at that position remains
eligible. Cost samples exclude the containers.

Exterior impact uses the horizontal projection of changed collision triangles,
including their historical rotation and scale. Height-only model extent does not
expand horizontal coverage. Triangle/cell intersection excludes empty corners of
triangle bounds. Terrain changes select their owning CELL; no whole-cell impact
halo is added. `--neighboring-cell-radius` controls geometry suppliers without
expanding regeneration targets. Models absent after a completed loose-asset and
archive search supply no collision. Selection continues with a warning and lists
unique missing paths in `selection_missing_models` in `batch-report.json`.
Unreadable models and incomplete archive searches stop selection.
Model replacements compare winning collision against the preceding available asset
providers; added model paths have no prior collider. Archive-index failures compare
all model uses rather than select every model's bounding sphere.

New terrain and collision select cells without requiring authored NAVM. This
includes new worldspaces and submerged LAND heightfields: seabed terrain remains
a collision input, and generation can classify its triangles as water. Large
plugins can therefore contain much more supported terrain than their visible
landmass or authored navigation covers. These rules apply equally to Plugin
and Load order scope.

Water changes independently select cells with supported winning terrain or placed
collision. Water classifies generated triangles and does not supply a floor.
Supplier bounds discover candidate models, but exact collision triangles establish
their CELL coverage. Cells already selected by terrain or collision use their
winning water data. Empty water grids do not become targets from water metadata
alone. NAVM presence does not determine terrain, water or collision eligibility.

Generation loads neighboring geometry, including references whose model bounds
reach the target from farther away. Missing model bounds conservatively include
their source cells across that worldspace. Every candidate is clipped to its
own exterior CELL; geometry suppliers do not enlarge the generated area.
This separation also applies to single-cell generation. Increasing the geometry
halo is compatible with plugin writing.
For single-cell inspection scenes, terrain and authored NAVM are limited to the
scene neighborhood. Distant suppliers contribute only model triangles whose
world-space bounds intersect that neighborhood, including display-only render
geometry. Their source cells do not expand the scene's terrain or NAVM coverage.

## Outputs and failure policy

`batch-report.json` identifies the selection, completion/failure state, cell
statuses, polygon totals, extraction count, geometry-cache reuse, and whether
`copy_plugin` was selected. It also checkpoints per-cell timing and supplier
counts, selection diagnostics, archive I/O, worker admission, cache reuse, and
terminal output bytes. Selection counters distinguish unique terrain, water and
collision targets, water owners without supported geometry, and
selected counts by worldspace. Categories overlap when multiple inputs affect a
cell; ignored contributions can still be selected by another input.
`full` output writes candidate JSON/OBJ beneath
`cells/<resolved FormID>/`, OBJ metadata, and the complete winning-record table.
`compact` writes streaming gzip candidate JSON and references shared input
catalogs. `plugin_only` retains reports and the requested plugin. `auto` selects
plugin-only output for writing or estimates, and full output for inspection.
Source-triangle and geometry-source evidence are compacted together. Batch runs
omit the large per-cell scene and discrepancy exports.

Every selected live cell generates a candidate, including cells without an existing
NAVM. With `--skip-existing-navmesh`, any winning NAVM record protects its CELL and
is reported as `skipped_existing_navm` before extraction. Empty, unsupported and
deleted NAVM records also protect their identities when that option is enabled.
The Windows UI exposes and persists **Skip cells with existing navmesh**.
New NAVM child groups use the CELL's identity in the output master table;
unrelated earlier load-order plugins do not affect their placement.
Deleted CELLs are skipped. Valid empty candidates are completed replacements when
no supported walkable floor survives; they clear replaced authored geometry rather
than inventing a floor. Cell-local Recast generation, height-range and topology
failures are logged as `skipped_generation_failed`, with the CELL identity, error
and validation findings in `batch-report.json`. These cells produce no replacement
NAVM; their authored geometry stays intact. Other cells continue generating and
the verified plugin can complete. Reports use `complete_with_skips` when generation
failures occurred and count them in `generation_failed_cells`; the completion
message also reports that count. Input, extraction, allocation exceptions, evidence-storage,
global seam refinement and writer failures still stop publishing.

Generation and border linking are separate stages. Workers retain all surviving
walkable components without requiring authored portals or door anchors. Authored
NAVMs from other rebuilding targets never constrain their geometry. Borders into
untouched cells are matched to authored edges while preserving those endpoints.
Targets skipped after generation failure become untouched neighbors for border
matching, including reciprocal connection updates when required.
Recovery validates the complete set of established authored portal destinations
together with the skipped neighbors. A skipped neighbor without authored NAVM
adds no border constraint; open seams remain subject to complete-set reachability.
After every target is ready, `core/navmesh/batch_stitching` intersects neighboring
candidate seam partitions, splits triangles with stable evidence and door joins,
and welds compatible heights within movement limits. Corner endpoints are planned
together. Linking revisits intervals made compatible by corner welding while
keeping existing portal endpoints pinned; different authored corner heights remain pinned and connect through
climb-compatible internal edges. Missing or unreachable
neighbor floors leave unpaired seams open. Compatible shared edges
receive exact reversed endpoints and unique reciprocal generated triangle targets.

After linking, `core/navmesh/candidate_reachability` floods shared-edge adjacency
and reciprocal generated portals across the complete successful set. It retains
floor networks reaching matched doors or untouched authored neighbors, together
with the largest network by horizontal area in each contiguous exterior selection
and each interior. Separate selections and worldspaces are evaluated independently.
Generated portals and open CELL borders alone do not provide access: an isolated
roof spanning multiple CELLs is removed as one component. Vertex-only contact does
not connect floors. Filtering compacts geometry, source joins, regions, doors and
both portal destinations, rebuilds contours and validates final topology.
Removed triangles appear in `rejected_unreachable`; empty targets remain completed
replacements. Cached candidates are filtered after linking on every run.

The combined plugin overrides each target's existing NAVMs. Generated geometry
occupies its largest source NAVM and the others become empty. Uncovered targets
receive new plugin-owned NAVMs in their winning CELL hierarchy. The writer allocates
all primary identities together, then resolves generated CELL destinations to those
identities, including pairs where neither cell had authored navmesh. Untouched
neighbors preserve their vertices and triangles and receive reciprocal portal
overrides where required. Existing incoming links to replaced triangles are cleared
and matched borders receive fresh return links. Every emitted NAVM is read back
before the temporary plugin is finalized. Existing output plugins are refused.
The patch is light-flagged only when its dependencies and allocated identities fit
that format. Copy mode retains the selected source format and dependency limits.

The existing writer limitations still apply: NAVI, teleport-door XNDP, cover and
unmatched authored links are not rebuilt.

## Performance and verification

[Performance improvements](performance-improvements.md) describes persisted CLI/UI
settings for output policy, shared disk cache, estimated working memory, workers,
and cost preflight, with [measured results](performance-improvements-measurements.json).
The [original cost analysis](performance-analysis.md) retains the baseline.

The load order and MO2 asset winners are resolved once per run. Shared immutable
payload ranges avoid subrecord duplication, while plugin record ownership moves
into winners incrementally. LAND uses a direct CELL index. Targets run in spatial
order; winning placements are bounds-filtered before NIF extraction, and bounded
model-local/placement caches reuse geometry. Independent generation tasks obey
worker and estimated-byte admission limits. Compact candidate audit data is spooled
to gzip; compact joins remain resident for seam refinement. Border reconciliation and guarded plugin writing
remain global ordered stages. Input plugin records are read by range.

`--estimate-only` samples eligible interior/exterior and density strata and stops
before writing a plugin. Its report distinguishes the full selected/eligible scope
from sampled/completed work and gives a heuristic remaining-time range. Cached
sampled candidates can be reused by a later generation run.

Candidate keys include the ordered generation inputs and a versioned pipeline
compatibility identity. Rebuilding a compatible executable preserves reuse.
Cache entries keyed by a legacy executable hash need explicit verified migration;
the checkpoint alone does not make an arbitrary candidate safe to reuse.

Run the core tests and the synthetic CLI integration tests after building:

```powershell
xmake f -m debug
xmake build navmesh-tests
xmake run navmesh-tests
xmake f -m releasedbg
xmake build NavmeshGenerator
python tools/test_batch_rebuild.py ./build/windows/x64/releasedbg/NavmeshGenerator.exe
```

Fixtures exercise LAND edits without CELL overrides, plugin and load-order
selection, geometry-cache reuse, single-cell clipping with neighboring input,
combined patch serialization, complete shared-floor seam coverage and reciprocal
generated borders, historical moves,
persistent placements, oversized models and invalid selections. The Windows
executable is built; validation invokes only the CLI.
