# Rebuilding affected cells

The shared application runner supports **Cell**, **Plugin**, and **Load order**
scopes. The Windows UI persists the scope and affected-plugin filename with the
other settings. Plugin and load-order rebuilding require a resolved MO2 profile
or the developer load-order manifest. They generate inspection candidates;
**Write plugin** / `--generate-plugin` additionally writes a combined patch.

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
edit; without an archive-content impact index, their presence conservatively
includes all model-bearing reference locations.

## Impact selection and geometry inputs

The index examines CELL, LAND, NAVM, placed-reference and worldspace edits.
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
statuses, polygon totals, extraction count and geometry-cache reuse. Each
processed target has candidate JSON/OBJ beneath `cells/<resolved FormID>/`.
Candidate source evidence is compacted to the provenance entries used by its
polygons. JSON exports contain run metadata, and candidate OBJs have metadata
sidecars. Batch runs omit the large per-cell scene and discrepancy exports.

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

The load order and MO2 asset winners are resolved once per run. FormID lookups
use a hash index; the spatial index uses worldspace/coordinate keys and preserves
compact placement history instead of historical raw record payloads. Exterior
targets run in spatial order, and the geometry cache uses least-recently-used
eviction with entry and triangle-volume limits. Candidate geometry and compact
audit evidence remain available for combined serialization; extracted scene
meshes do not accumulate for the entire load order.

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
