# Rebuilding affected cells

The shared application runner supports **Cell**, **Plugin**, and **Load order**
scopes. The Windows UI persists the scope and affected-plugin filename with the
other settings. Plugin and load-order rebuilding require a resolved MO2 profile
or the developer load-order manifest. They generate candidates;
**Write plugin** / `--generate-plugin` additionally writes a combined patch.
Automatic batch output retains full inspection without writing, and reports plus
the plugin when writing. **Batch output** / `--batch-output` selects full,
compressed, or plugin-only artifacts explicitly.

For authoring, **Copy selected plugin** / `--copy-plugin` enables writing and
uses the selected Plugin-scope file as the output template. The UI persists the
option and rejects it in Cell or Load order scope. The result is written under
the source filename in the output folder and is intended to replace the selected
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

```powershell
navmesh-offline.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-plugin "<active plugin.esp>" --copy-plugin --output "<new output folder>"
```

```powershell
navmesh-offline.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-plugin "<active plugin.esp>" --generate-plugin --output "<new output folder>"
navmesh-offline.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-load-order --generate-plugin --output "<new output folder>"
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

The index examines navigation-changing CELL, LAND, NAVM, placed-reference and
worldspace edits. Equivalent navigation inputs, display names, editor IDs,
worldspace map/height-summary metadata and LAND color/texture edits are excluded.
Unknown fields and ownership changes remain conservative dependencies.
Changed base records select their placed uses, including untouched references.
Moved or deleted references select historical and winning locations. Exterior
positions, including negative coordinates, determine physical cell ownership;
persistent references stored in a distant parent CELL are bucketed by their
world position. Worldspace identities keep overlapping coordinate grids separate.
Interiors remain independent targets.

An exterior target includes an adjacent impact halo, extended by the requested
`--neighboring-cell-radius`. Placed-object OBND bounds and historical reference
scales conservatively extend the footprint for oversized models. The bound uses
a rotation-independent enclosing sphere. Missing model bounds select the whole
worldspace because a finite influence radius cannot be justified.

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
terminal output bytes. `full` output writes candidate JSON/OBJ beneath
`cells/<resolved FormID>/`, OBJ metadata, and the complete winning-record table.
`compact` writes streaming gzip candidate JSON and references shared input
catalogs. `plugin_only` retains reports and the requested plugin. `auto` selects
plugin-only output for writing or estimates, and full output for inspection.
Source-triangle and geometry-source evidence are compacted together. Batch runs
omit the large per-cell scene and discrepancy exports.

Without `--skip-existing-navmesh`, cells without an existing NAVM are reported
as skipped. With that option, any winning NAVM record protects its CELL and is
reported as `skipped_existing_navm` before geometry extraction. Empty,
unsupported, and deleted NAVM records also protect their authored identities.
The Windows UI exposes and persists **Skip cells with existing navmesh**.
Uncovered selected cells generate candidates and receive new plugin-owned NAVM
identities when writing a patch. New records use the winning CELL's hierarchy
and its temporary child group. The report records `skip_existing_navmesh`.
Deleted cells and empty candidates are explicitly reported as skipped.
Unsupported candidates or source layouts stop the batch with an error. A plugin
is written only after all eligible candidates have been generated. When no
eligible replacements exist, the report is produced without a patch.

The combined ESP overrides each eligible cell's existing NAVMs. Generated
geometry occupies that cell's largest source NAVM and the others become empty.
Generated-to-generated borders are redirected to generated triangle indices
and must have matching full edges and reciprocal targets. Incompatible border
partitions fail the batch before a final ESP is created. Adjacent cells outside
the rebuilding set keep their geometry and receive reciprocal portal overrides
where required. Existing output plugins are refused. Every emitted NAVM is read
back before the temporary file is finalized.

In uncovered-cell mode, authored neighbor vertices and triangles are preserved;
matched borders may add reciprocal portal links to them. New CELL navigation
retains border-reaching regions without an authored portal requirement. Unmatched
borders remain unlinked, including between newly covered cells, and require
independent connection review. The generated ESP is light-flagged only when its
master table and newly allocated identities fit the light format.

The existing writer limitations still apply: NAVI, teleport-door XNDP, cover and
unmatched authored links are not rebuilt. Inspect the patch independently before
using it in a disposable game profile.

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
to gzip and released from memory. Border reconciliation and guarded plugin writing
remain global ordered stages. Input plugin records are read by range.

`--estimate-only` samples eligible interior/exterior and density strata and stops
before writing a plugin. Its report distinguishes the full selected/eligible scope
from sampled/completed work and gives a heuristic remaining-time range. Cached
sampled candidates can be reused by a later generation run.

Run the core tests and the synthetic CLI integration tests after building:

```powershell
xmake build navmesh-offline navmesh-tests
./build/windows/x64/releasedbg/navmesh-tests.exe
python tools/test_batch_rebuild.py ./build/windows/x64/releasedbg/navmesh-offline.exe
```

Fixtures exercise LAND edits without CELL overrides, plugin and load-order
selection, geometry-cache reuse, single-cell clipping with neighboring input,
combined patch serialization, reciprocal generated borders, historical moves,
persistent placements, oversized models and invalid selections. The Windows
executable is built; validation invokes only the CLI.
