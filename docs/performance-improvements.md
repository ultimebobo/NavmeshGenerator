# Faster plugin rebuilds with less disk output

Plugin rebuilds filter navigation changes before scheduling generation, reuse
indexed assets and decoded geometry, and make inspection exports optional.
The shared CLI and Windows runner use the same settings and validation.

## Operator settings

| CLI option | Windows setting | Behavior |
| --- | --- | --- |
| `--batch-output auto` | Batch output | Plugin writing and cost estimation retain reports and the requested plugin; candidate inspection retains full exports. |
| `--batch-output plugin_only` | Batch output | Retain run/input reports and the verified plugin; requires plugin writing or cost estimation. |
| `--batch-output compact` | Batch output | Add streaming gzip candidate JSON, without OBJ or the full winning-record table. |
| `--batch-output full` | Batch output | Include candidate JSON/OBJ and metadata, the winning-record table, and full MO2 asset winners. |
| `--asset-cache <directory>` | Shared asset cache | Reuse generated assets across output directories. An empty setting uses the system temporary cache. |
| `--cache-budget-mib <MiB>` | Cache disk budget | Evict generated cache files by last use. Zero disables retention after a run. |
| `--working-memory-mib <MiB>` | Working memory budget | Divide estimated working storage between decoded geometry reuse and admitted generation tasks. |
| `--workers <count>` | Generation workers | Bound independent Recast tasks; extraction, checkpoint publication, border reconciliation, and writing remain ordered. |
| `--estimate-only` | Estimate batch cost | Select the full target set and generate a stratified sample, cache the work, and finish before writing a plugin. |

Numeric defaults and accepted limits come from `app::Options` and shared run
validation. The dark desktop persists every setting and provides a scrolling
workspace with numerical controls in the initially folded **Advanced settings**.
Single-cell inspection retains its existing exports; batch output policy applies
to Plugin and Load order scopes.

Run an estimate using the same inputs and generation settings as the intended
rebuild, then run generation into a fresh output directory:

```powershell
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-plugin "<active plugin.esp>" --generate-plugin --estimate-only --output "<estimate folder>"
NavmeshGenerator.exe --mo2 "<MO2 instance>" --profile "<existing profile>" --rebuild-plugin "<active plugin.esp>" --generate-plugin --output "<new output folder>"
```

The estimate samples interior cells, exterior cells with authored neighbors, and
other exterior cells. Within each group it samples low, middle, and high reference
and authored-polygon density. `selected_cells` and `eligible_cells` describe the
full scope; `sampled_cells` and `completed_cells` describe the work actually done.
The remaining-time range is a heuristic derived from elapsed time per completed
cell, excludes final export/writing, and cannot guarantee a completion time for
heterogeneous scenes. Cached samples can be reused by the later rebuild.

## Selection and extraction

Each resolved origin records whether its navigation inputs differ from the
preceding version. Display names and editor IDs, worldspace map/height-summary
data, and LAND color/texture data do not trigger generation. LAND height samples,
reference placement and ownership, base-object geometry, navigation records,
deletion/disable flags, and unknown fields remain conservative dependencies.
Resolved identities are compared where raw plugin-local IDs differ. Historical
placements and base bounds remain available for move/deletion impact selection.

Archive model indexes identify the winning changed NIF names, excluding loose
winners. An index failure retains the conservative archive-impact fallback.
Oversized or unknown model bounds retain conservative coverage. Geometry suppliers
use winning bounds, and placements whose conservative world-space sphere misses
the target are excluded before requesting or decoding a NIF. Unknown bounds and
direct model overrides remain included.

LAND records are indexed by CELL. A shared model cache reuses immutable decoded
model-local geometry and transformed placements, keyed by provider revision,
extraction policy, and transform. Generation materializes collision support and
coverage without building render display meshes. Inspection still builds display
geometry. Support geometry has one authoritative mesh, with ordered provenance
joins during assembly. Recast intermediates are released after their last use.

## Disk and cache lifetime

BSA reading indexes directory/file tables and seeks only to requested winning
payloads. It supports desktop Skyrim archive revisions, filename prefixes,
per-entry compression toggles, zlib, and LZ4. Requests are deduplicated and missing
models are remembered only after reliable archive searches. Unreadable winning
entries do not fall back to lower-priority assets. The Python helper requires the
packages in `tools/requirements.txt`; `NAVMESH_PYTHON` can select its interpreter.

Provider paths, size, modification time, priority, and cache schema identify
archive snapshots. MO2 loose catalogs also include profile/root identities and
directory membership revisions. Input reports reference these shared catalogs
instead of duplicating every winner in compact or plugin-only runs.

Candidate fingerprints include actual ordered support geometry and provenance,
generation settings, exits, bounds, authored neighboring geometry/portals, and
the running executable identity. Incompatible or corrupt cache entries trigger
generation. Both source-triangle and geometry-source evidence tables are
compacted and written to private gzip cache data. Inspection audits are pinned
through reconciliation/export and staging links or copies are removed on exit.

Eviction only touches recognized generated cache files inside the cache root.
Game inputs and operator-owned exports are excluded. Ownership/statistics markers
and active audit pins are outside the retention budget; active work can temporarily
exceed it. Separate-volume inspection staging may require a compressed copy.
Legacy caches in old output directories are retained; they are not migrated or
deleted automatically. Requested plugin copies still contain the complete source.

The working-memory setting controls cache retention and admission estimates,
rather than enforcing a process RAM cap. Resolved records and compact candidate
meshes required for global border validation/writing remain resident; unusually
large vertical-span workloads can exceed the admission estimate. A task exceeding
the estimated work budget runs alone. Plugin source records are read by byte range;
complete source buffers are required only for byte-preserving plugin copies.

## Reports and verification

`batch-report.json` checkpoints each completed target and records input identity
and diagnostics, selection counts by record type, equivalent-record exclusions,
base-object and asset uses, supplier/reference/model counts, extraction and
generation timings, cache hits, archive metadata/payload bytes read, peak admitted
workers/estimated bytes, and terminal output bytes grouped by extension.
Selection categories can overlap. Output byte totals cover the output directory,
excluding the checkpoint itself and private staging; use a fresh directory for
comparable measurements. Archive counters describe the shared snapshot's observed
helper activity during the run; concurrent processes using the same snapshot can
contribute to these counters.

Combined border reconciliation and writer read-back remain mandatory. Synthetic
regressions verify unchanged plugin bytes across output modes and worker counts,
cache invalidation by neighboring navigation changes, semantic selection,
preflight reuse, archive precedence/compression, cache corruption, and eviction
that preserves foreign files. Validation builds the desktop executable and invokes
only CLI workflows, with assertions enabled in the native tests.

[The before/after measurement snapshot](performance-improvements-measurements.json)
contains local resolution, selection, archive I/O, Recast, and synthetic export
measurements. [The original cost analysis](performance-analysis.md) preserves the
baseline investigation. These stage measurements do not establish a complete
plugin rebuild speedup. Navigation-profile coarsening and tiled interior generation
require separate correctness work; partition strategies remain explicit choices.
