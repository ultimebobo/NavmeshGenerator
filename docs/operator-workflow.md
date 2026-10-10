# Mod Organizer 2 operator workflow

For a desktop quick start, see [the README](../README.md). Current CLI examples
and detailed export descriptions are in the [command-line reference](command-line.md).

This document defines the user-facing command contract. You select an existing Mod Organizer 2 (MO2) instance and one of its existing profiles. You do **not** create plugin manifests, copy assets, deploy mods, edit `plugins.txt`, or change MO2's virtual file system.

Analysis commands read the input profile without changing it. The guarded plugin-generation option writes a new plugin only beneath the `--output` directory supplied by the user.

For mod authors, select **Plugin** scope, enter the active filename in **Affected
plugin**, and enable **Copy selected plugin**. The saved option enables plugin
writing and exports a copy under the original filename with generated NAVMs and
the plugin's other encoded records. The CLI equivalent is `--rebuild-plugin
"<active plugin.esp>" --copy-plugin`. Use this copy in place of the source with
the original assets and localization resources. It preserves source flags and
master order; generated references outside that master table and new identities
that exceed the source's full/light format stop export. Existing output files
and source plugins are never overwritten. The default **Write plugin** export
still produces a separate NAVM-only patch. See [batch rebuilding](batch-rebuilding.md).

For explicit cells, use **Cell** mode, fill **Cells**, enable **Copy selected
plugin**, and enter the active filename in **Plugin to copy**. The CLI accepts
`--cells "<cell identifiers>" --copy-plugin --copy-plugin-source "<active plugin.esp>"`.
Inspection without generation writes separate results beneath `cells/<FormID>/`
for multiple cells and uses the usual output paths for one cell.

## Desktop workspace

Starting without command-line switches opens a dark Dear ImGui workspace with
input-profile, target and output cards. Folder inputs have native browse buttons;
the layout scrolls while progress and actions remain visible. The title bar uses
the Windows dark appearance where supported. Keyboard navigation and DPI scaling
are enabled.

Select an MO2 folder, an existing profile name and an output folder. **List cells
to file** exports `cells.json`; it is an action rather than a saved run mode and
does not require a selected cell or valid generation settings. Cell rows are
written only to the catalog, including for CLI `--list-cells` runs.

**Rebuild scope** controls which settings appear:

- **Cell** shows a multiline **Cells** field. Paste Form IDs or editor IDs from
  `cells.json`, one per line or separated by commas or semicolons. The identifier
  count helps review pasted input, and **Clear list** removes it. Aliases are
  combined; unknown or ambiguous identifiers stop processing. Leave generation
  disabled to inspect the selected cells. Enable **Copy selected plugin** to show
  **Plugin to copy**, where you enter or browse to an active plugin filename.
  The generated NAVMs are written in that copy. The source field is required only
  when copying and is saved independently of the affected-plugin selection.
- **Plugin** shows **Affected plugin** and **Copy selected plugin**. Copy mode
  enables writing through the shared runner.
- **Load order** selects affected cells without a cell identifier or affected
  plugin. Affected-cell scopes and Cell generation lists expose cost estimation,
  output policy and workers; Cell inspection retains its analysis controls.

**Skip cells with existing navmesh** appears when generation is enabled. The
plugin-only output policy is available when batch writing or estimation is
selected. Settings retain their saved values when hidden, but inactive selectors
and dependent flags are cleared before invoking `app::Run`.

In **Cell** scope, enable **Make scene** to write one `scene.glb` for the complete
selection. It combines terrain, collision, display models, authored NAVMs and any
finalized generated NAVMs, with shared neighborhood geometry emitted once. It
works during inspection, candidate generation and plugin-copy exports, independently
of the batch output policy. The choice is saved. Cost estimation is disabled while
making a scene because the scene requires the whole selection. Cells retain their
native coordinates, so unrelated interiors or worldspaces may overlap.

**Advanced settings** starts folded each time the workspace opens. It contains
numerical analysis thresholds, neighboring-cell radius, disk/working budgets,
batch workers, optional moved-mod recovery and cache folders. Recast settings
appear only for generation; analysis thresholds appear only for Cell scope.
The desktop persists choices in the existing local `NavmeshGenerator/config.ini`
and validates relevant values before running. Developer input routes, custom OBJ
paths, exterior coordinates, terrain-only and diagnostic HTML controls are CLI
options.

## Advanced generation settings

**Tag water and preferred path triangles** enables generated classification and
is saved with the other desktop choices. Water uses supported exterior water
levels, with authored water markings as a fallback. Preferred paths follow
nearby winning authored markings on a matching floor. Disable the checkbox to
omit both tags; the CLI equivalent is `--no-triangle-tagging`.


The UI and CLI pass the same navigation profile and Recast settings to Cell,
Plugin and Load order generation. Defaults come from `NavigationProfile` and
`RecastSettings`; numeric constraints are checked by `ValidateRecastSettings`.
Distances use Skyrim world units, region area uses square world units, and slope
uses degrees. Values must be finite and conversions must fit Recast's voxel
counts and packed vertical spans. Horizontal voxel size is a requested minimum;
the generator increases it for large scenes to bound the grid.

**Reset** restores all advanced numerical inputs from the shared application
defaults and saves them, including currently hidden fields. It covers generation,
analysis, neighboring-cell radius, resource budgets and worker count. Paths,
target and output selections, switches and region partitioning stay selected.

| CLI input | Advanced setting | Meaning |
| --- | --- | --- |
| `--agent-radius` | Agent radius | Horizontal agent footprint. |
| `--agent-height` | Agent height | Standing height. |
| `--agent-clearance` | Agent clearance | Required headroom, combined with standing height. |
| `--agent-step-height` | Step height | Traversable climb. |
| `--agent-max-slope` | Walkable slope | Generation slope limit, separate from analysis slope. |
| `--minimum-region-area` | Minimum region area | Disconnected-island cutoff. |
| `--weld-tolerance` | Weld tolerance | Positive output/border matching distance tolerance. |
| `--recast-cell-size` | Horizontal voxel size | Requested minimum raster width. |
| `--recast-cell-height` | Vertical voxel size | Raster height. |
| `--recast-simplification-error` | Contour simplification error | Maximum deviation in horizontal voxels. |
| `--recast-max-edge-length` | Maximum contour edge length | Subdivision distance; zero disables it. |
| `--recast-merge-area-multiplier` | Region merge multiplier | Merge area relative to the minimum area. |
| `--partitioning-algorithm` | Region partitioning | Watershed, monotone or layers. |

Layer partitioning does not use the merge-area multiplier, so that input is
hidden for Layers. Requested settings appear in candidate JSON. Cached candidates
include these settings in their fingerprint; changing them invalidates reuse.

For incomplete stairs, use **Reset** to restore the current numerical defaults,
then verify the climb limit against the actual risers. A climb limit below the
risers disconnects the flight; reachability filtering can then remove its
unanchored portion. Coarse horizontal voxels can combine
several treads, and coarse vertical voxels round floor heights and the climb
limit. Recast rounds the climb limit down to a whole number of vertical voxels;
candidate warnings report the effective limit when it differs from the requested
value. Warnings also identify adaptive horizontal resolution. Use a climb limit
that fits the actual risers, rather than allowing larger obstacles to become
traversable. Agent radius still trims exposed edges, so complete coverage does
not mean navigation reaches the outer edge of every tread.
The default movement profile accommodates taller and slightly uneven stair
risers while retaining normal standing clearance and agent-radius erosion.

The generator samples height detail after building contours so landings and
stairs follow the surviving walkable floor with movement-bounded approximation.
Straight flights can form compact ramps. Convex patch merging and contour
simplification reduce triangle density; smaller **Contour simplification error**
values retain closer obstacle outlines, and larger values reduce boundary detail.
The generator automatically refines contours that would erase a retained voxel
region. Use **Reset** to adopt current defaults in an existing saved configuration.
Landscape rock collision contributes obstacles without walkable tops; architectural
stone and terrain remain eligible. This cannot recover geometry that
was lost during voxelization, lacks supported collision, or fails clearance.

## MO2 command input

```powershell
NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --output "D:\NavmeshWork\run-001" <operation>
```

`--mo2` is either the portable MO2 root or the selected MO2 instance directory. `--profile` is the profile name visible in MO2. The importer reads, without changing:

- MO2's configured paths and game instance information;
- `profiles\<profile>\modlist.txt` to determine enabled mods and priority;
- `profiles\<profile>\plugins.txt` and `loadorder.txt` to determine enabled plugins and plugin order;
- base-game `Data`, enabled mod directories, and `Overwrite` to reproduce MO2's loose-file winners.

The first output for every MO2 operation is `input-report.json`. Compact/plugin-only batches retain winner counts and a shared loose-asset catalog path; full inspection includes every virtual winner. It identifies the profile, profile files read, active plugins, enabled mods, physical paths chosen for every input file, and a snapshot hash. It must warn about missing files, duplicate plugins, unsupported archive sources, or profile changes observed during the run.

MO2's profile files are the source of truth. `--data`, `--load-order`, `--mods-dir`, and `--profiles-dir` may exist only as explicitly marked developer/test escape hatches; they are not examples for normal use.

## What each milestone must let an operator do

The commands below are the target command contract. A command is introduced only in its milestone; later milestones keep the same MO2 input prefix.

| Milestone | Command an operator runs | Existing inputs required | Result written under `--output` |
| --- | --- | --- | --- |
| 0 — reproducibility | `NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --inspect-input --output "D:\NavmeshWork\m0"` | Existing MO2 instance and profile | `input-report.json`, `run-manifest.json`, metadata schema version, and a clear list of inputs/limitations. No game data is changed. |
| 1 — profile/load order | `NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --list-cells --output "D:\NavmeshWork\m1"` | Same MO2 profile; no manually made manifest | `input-report.json` plus `cells.json`: active plugin order, winning cell records, origins, FormID diagnostics, and cells available for later commands. |
| 2 — record/NAVM inspection | `NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --inspect-record 00012ABC --output "D:\NavmeshWork\m2"` | A FormID copied from the tool's own `cells.json`/report, or `--cell-x` and `--cell-y` | `record.json`, recognized NAVM/cell fields, raw/origin metadata, and explicit unsupported-field diagnostics. |
| 3 — scene/provenance | `NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-formid 00012ABC --export-scene --layers render --output "D:\NavmeshWork\m3"` | A cell identifier returned by milestone 1 | `scene.json` and render-geometry export; every triangle identifies the selected MO2 file, plugin record, model, and transforms. |
| 4 — terrain | `NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-x 10 --cell-y -5 --export-terrain --output "D:\NavmeshWork\m4"` | An exterior cell coordinate returned by milestone 1 | `terrain.json` and terrain geometry export with `LAND`/cell provenance, or an explicit `LAND` coverage failure. |
| 5 — collision | `NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-formid 00012ABC --export-scene --layers terrain,collision,render-fallback --output "D:\NavmeshWork\m5"` | A selected cell and the active MO2 assets | Scene export plus coverage report distinguishing terrain, collision, and lower-confidence render fallback. |
| 6 — combined visualization | `NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-formid 00012ABC --geometry-layers navmesh,terrain,collision,render --output "D:\NavmeshWork\m6"` | A selected cell | One color-layered `scene.glb`, `scene.glb.provenance.json`, `scene-report.html`, and `input-report.json`; it opens directly in Online 3D Viewer. |
| 7 — diagnose existing NAVM | `NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-formid 00012ABC --diagnose-navmesh --output "D:\NavmeshWork\m7"` | A selected cell containing NAVM | `diagnosis.json` and diagnostic scene: supported, floating, buried, too-steep, blocked, out-of-coverage, and ambiguous polygons with confidence/evidence. No plugin is written. |
| 8 — candidate NAVM | `NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-x 10 --cell-y -5 --generate-candidate --output "D:\NavmeshWork\m8"` | A selected cell/region using fixed human candidate settings | `candidate-navmesh.json` and colored scene export. This is neutral data only, not a plugin. |
| 9 — repair plan | `NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-formid 00012ABC --plan-repair --output "D:\NavmeshWork\m9"` | Existing NAVM plus scene/candidate data produced in the same run | `repair-plan.json` and a review scene. It says retain/change/manual-review and includes evidence; it does not write a plugin. |
| 10 — patch plugin | `NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --apply-approved-plan "D:\NavmeshWork\m9\repair-plan.json" --output "D:\NavmeshWork\m10"` | An unchanged profile snapshot and a plan explicitly approved by the user | A new patch plugin only in the output directory, read-back validation report, and installation instructions. Input plugins and MO2 files remain untouched. |
| 11 — release validation | `NavmeshGenerator.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --validate-benchmarks --output "D:\NavmeshWork\m11"` | Existing MO2 profile; configured benchmark selections from earlier tool output | `validation-summary.json`, performance results, and pass/fail evidence for the supported workflow. |

`00012ABC`, `10`, and `-5` are placeholders. The operator obtains real cell IDs and coordinates from milestone 1's `cells.json` or by using `--list-cells`; they are not expected to discover or edit files manually.

For a current analysis run, `navmesh-counts.txt` records two counts: original
navmesh polygons in the selected CELL and generated navmesh polygons. The CLI
prints these counts, and the desktop UI shows the same summary after completion.

Select a cell and use `--generate-plugin`. Neighboring geometry remains available while generation is clipped to the selected CELL. Plugin and Load order scopes rebuild affected cells with the same input profile; see [batch rebuilding](batch-rebuilding.md). The generated ESP is placed under `--output` and every NAVM override is read back before success is reported. The desktop UI exposes **Write plugin**; the ESP is ESL-flagged when the patch fits the light format. Place it after its source ESPs. All existing NAVMs in the selected cell receive overrides: the generated geometry goes into the largest source NAVM and the rest become empty. Matched door triangles and reciprocal links to adjacent border NAVMs are serialized. Only components with a path through shared polygon edges to a matched door or a real neighboring portal survive final stitching. Interiors without a matched door produce empty candidates. Unmatched seam wedges retract into the CELL. Every remaining seam uses the exact neighboring edge; empty candidates are skipped without writing a patch. The [glossary](glossary.md) explains border matching and neighboring connection overrides. Other authored links, cover data, NAVI, and REFR XNDP references are not rebuilt. Check the resulting pathing in independent tooling and on a disposable game profile before using the plugin.

For generation that fills uncovered cells, enable **Skip cells with existing
navmesh** or pass `--skip-existing-navmesh` with `--generate-candidate` or
`--generate-plugin`. It works in all rebuild scopes with resolved input. Winning
NAVM records, even empty, unsupported, or deleted ones, protect their CELL from
generation. A skipped single-cell run completes with `generation-report.json`;
batch runs record `skipped_existing_navm`. Uncovered cells can receive new NAVM
records. Matched borders may add reciprocal links to authored neighbors while
retaining their geometry. Unmatched seams retract into the CELL, and components
without a real neighboring portal or matched door are removed; see
[batch rebuilding](batch-rebuilding.md).

## Immediate correction to milestone 1

The current milestone-1 parser/resolver is useful internal work, but its normal command takes a manually prepared load-order manifest. That does not meet this product's input contract.

Before milestone 2, complete milestone 1 with the following instruction:

> Read `docs/roadmap.md` and `docs/operator-workflow.md`. Complete milestone 1 by adding a read-only MO2 profile importer. The normal command must be `--mo2 <instance-or-portable-root> --profile <existing-profile>`, and it must derive active plugins, plugin load order, enabled-mod priority, base-game Data, selected physical plugin paths, and loose-asset winners from MO2's existing configuration/profile files. Emit `input-report.json` before cell listing or extraction, including a profile snapshot hash and actionable missing/unsupported diagnostics. Keep the existing manual `--load-order` route only as a developer/test override. Add tests using synthetic MO2 directory/profile fixtures; do not ask the user to create a manifest or modify MO2/game files. Update README examples to use MO2. Do not begin milestone 2.

## Validation without manual input preparation

For every milestone, the operator runs that milestone's command against their actual MO2 profile and checks the named output. The test harness may use synthetic MO2 directories for repeatability, but the product itself must always show exactly which profile and virtual-file inputs it consumed.

The high-risk gates remain: real exterior terrain before generation; real collision before trusting support classification; visual scene alignment before repair planning; independent-tool and disposable-profile checks before using a written patch.

## Batch cost and storage settings

Use **Estimate batch cost** / `--estimate-only` with Cell generation lists, Cell copy exports, or Plugin and Load order scopes
to sample real generation before starting the complete job. Sampled candidates
are cached for reuse. **Batch output**, **Shared asset cache**, cache disk/working
budgets, and generation workers are persisted by the Windows UI and passed to
the common runner. Automatic plugin writing omits large inspection exports; full
or compact output can be selected explicitly. See [performance improvements](performance-improvements.md)
for cache lifetime, admission limits, report fields, and measurement scope.
