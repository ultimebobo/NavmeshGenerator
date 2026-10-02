# Mod Organizer 2 operator workflow

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

## One input model for the whole product

```powershell
navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --output "D:\NavmeshWork\run-001" <operation>
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
| 0 — reproducibility | `navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --inspect-input --output "D:\NavmeshWork\m0"` | Existing MO2 instance and profile | `input-report.json`, `run-manifest.json`, metadata schema version, and a clear list of inputs/limitations. No game data is changed. |
| 1 — profile/load order | `navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --list-cells --output "D:\NavmeshWork\m1"` | Same MO2 profile; no manually made manifest | `input-report.json` plus `cells.json`: active plugin order, winning cell records, origins, FormID diagnostics, and cells available for later commands. |
| 2 — record/NAVM inspection | `navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --inspect-record 00012ABC --output "D:\NavmeshWork\m2"` | A FormID copied from the tool's own `cells.json`/report, or `--cell-x` and `--cell-y` | `record.json`, recognized NAVM/cell fields, raw/origin metadata, and explicit unsupported-field diagnostics. |
| 3 — scene/provenance | `navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-formid 00012ABC --export-scene --layers render --output "D:\NavmeshWork\m3"` | A cell identifier returned by milestone 1 | `scene.json` and render-geometry export; every triangle identifies the selected MO2 file, plugin record, model, and transforms. |
| 4 — terrain | `navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-x 10 --cell-y -5 --export-terrain --output "D:\NavmeshWork\m4"` | An exterior cell coordinate returned by milestone 1 | `terrain.json` and terrain geometry export with `LAND`/cell provenance, or an explicit `LAND` coverage failure. |
| 5 — collision | `navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-formid 00012ABC --export-scene --layers terrain,collision,render-fallback --output "D:\NavmeshWork\m5"` | A selected cell and the active MO2 assets | Scene export plus coverage report distinguishing terrain, collision, and lower-confidence render fallback. |
| 6 — combined visualization | `navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-formid 00012ABC --geometry-layers navmesh,terrain,collision,render --output "D:\NavmeshWork\m6"` | A selected cell | One color-layered `scene.glb`, `scene.glb.provenance.json`, `scene-report.html`, and `input-report.json`; it opens directly in Online 3D Viewer. |
| 7 — diagnose existing NAVM | `navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-formid 00012ABC --diagnose-navmesh --output "D:\NavmeshWork\m7"` | A selected cell containing NAVM | `diagnosis.json` and diagnostic scene: supported, floating, buried, too-steep, blocked, out-of-coverage, and ambiguous polygons with confidence/evidence. No plugin is written. |
| 8 — candidate NAVM | `navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-x 10 --cell-y -5 --generate-candidate --output "D:\NavmeshWork\m8"` | A selected cell/region using fixed human candidate settings | `candidate-navmesh.json` and colored scene export. This is neutral data only, not a plugin. |
| 9 — repair plan | `navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --cell-formid 00012ABC --plan-repair --output "D:\NavmeshWork\m9"` | Existing NAVM plus scene/candidate data produced in the same run | `repair-plan.json` and a review scene. It says retain/change/manual-review and includes evidence; it does not write a plugin. |
| 10 — patch plugin | `navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --apply-approved-plan "D:\NavmeshWork\m9\repair-plan.json" --output "D:\NavmeshWork\m10"` | An unchanged profile snapshot and a plan explicitly approved by the user | A new patch plugin only in the output directory, read-back validation report, and installation instructions. Input plugins and MO2 files remain untouched. |
| 11 — release validation | `navmesh-offline.exe --mo2 "D:\Modding\MO2\Skyrim SE" --profile "My Actual Profile" --validate-benchmarks --output "D:\NavmeshWork\m11"` | Existing MO2 profile; configured benchmark selections from earlier tool output | `validation-summary.json`, performance results, and pass/fail evidence for the supported workflow. |

`00012ABC`, `10`, and `-5` are placeholders. The operator obtains real cell IDs and coordinates from milestone 1's `cells.json` or by using `--list-cells`; they are not expected to discover or edit files manually.

For a current analysis run, `navmesh-counts.txt` records two counts: original
navmesh polygons in the selected CELL and generated navmesh polygons. The CLI
prints these counts, and the desktop UI shows the same summary after completion.

Select a cell and use `--generate-plugin`. Neighboring geometry remains available while generation is clipped to the selected CELL. Plugin and Load order scopes rebuild affected cells with the same input profile; see [batch rebuilding](batch-rebuilding.md). The generated ESP is placed under `--output` and every NAVM override is read back before success is reported. The desktop UI exposes **Write plugin**; the ESP is ESL-flagged when the patch fits the light format. Place it after its source ESPs. All existing NAVMs in the selected cell receive overrides: the generated geometry goes into the largest source NAVM and the rest become empty. Matched door triangles and reciprocal links to adjacent border NAVMs are serialized. Candidate regions without either connection are removed before export. Other authored links, cover data, NAVI, and REFR XNDP references are not rebuilt. Check the resulting pathing in independent tooling and on a disposable game profile before using the plugin.

For generation that fills uncovered cells, enable **Skip cells with existing
navmesh** or pass `--skip-existing-navmesh` with `--generate-candidate` or
`--generate-plugin`. It works in all rebuild scopes with resolved input. Winning
NAVM records, even empty, unsupported, or deleted ones, protect their CELL from
generation. A skipped single-cell run completes with `generation-report.json`;
batch runs record `skipped_existing_navm`. Uncovered cells can receive new NAVM
records. Matched borders may add reciprocal links to authored neighbors while
retaining their geometry. Unmatched borders, including between new cells, remain
unlinked; see [batch rebuilding](batch-rebuilding.md).

## Immediate correction to milestone 1

The current milestone-1 parser/resolver is useful internal work, but its normal command takes a manually prepared load-order manifest. That does not meet this product's input contract.

Before milestone 2, complete milestone 1 with the following instruction:

> Read `docs/roadmap.md` and `docs/operator-workflow.md`. Complete milestone 1 by adding a read-only MO2 profile importer. The normal command must be `--mo2 <instance-or-portable-root> --profile <existing-profile>`, and it must derive active plugins, plugin load order, enabled-mod priority, base-game Data, selected physical plugin paths, and loose-asset winners from MO2's existing configuration/profile files. Emit `input-report.json` before cell listing or extraction, including a profile snapshot hash and actionable missing/unsupported diagnostics. Keep the existing manual `--load-order` route only as a developer/test override. Add tests using synthetic MO2 directory/profile fixtures; do not ask the user to create a manifest or modify MO2/game files. Update README examples to use MO2. Do not begin milestone 2.

## Validation without manual input preparation

For every milestone, the operator runs that milestone's command against their actual MO2 profile and checks the named output. The test harness may use synthetic MO2 directories for repeatability, but the product itself must always show exactly which profile and virtual-file inputs it consumed.

The high-risk gates remain: real exterior terrain before generation; real collision before trusting support classification; visual scene alignment before repair planning; independent-tool and disposable-profile checks before using a written patch.

## Batch cost and storage settings

Use **Estimate batch cost** / `--estimate-only` with a Plugin or Load order scope
to sample real generation before starting the complete job. Sampled candidates
are cached for reuse. **Batch output**, **Shared asset cache**, cache disk/working
budgets, and generation workers are persisted by the Windows UI and passed to
the common runner. Automatic plugin writing omits large inspection exports; full
or compact output can be selected explicitly. See [performance improvements](performance-improvements.md)
for cache lifetime, admission limits, report fields, and measurement scope.
