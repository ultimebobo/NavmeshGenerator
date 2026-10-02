# Plugin generation cost analysis

This document preserves the pre-implementation cost investigation and its
historical measurements. Current behavior, implemented optimizations, operator
settings and limits are described in [performance improvements](performance-improvements.md),
with a separate [before/after snapshot](performance-improvements-measurements.json).
Descriptions of existing code and proposed changes below refer to the baseline
investigation, rather than the current runner.

The largest opportunity is to reduce the work scheduled before optimizing
Recast. A plugin rebuild can expand to a whole worldspace, assemble geometry
from distant supplier cells, repeatedly decode the same models, and write
inspection evidence for every candidate. These costs compound.

The operator clarified that the storage concern is **disk space**. Disk output
and archive I/O are therefore the primary resource concerns below; RAM findings
are included because memory pressure can also slow generation.

## Evidence and limits

[The measurement snapshot](performance-analysis-measurements.json) records the
local reproduction, units, observed counts, timings, and output sizes. Numerical
results belong to that snapshot rather than being configuration promises.
Its reproduction plugin and cells are test data, not proposed special cases.

The analysis used the active plugin paths from the saved MO2 input report,
the current optimized desktop build, a read-only native probe, and a streaming
plugin-envelope inventory. It reproduced target selection without extracting
models or generating a plugin. The probe deliberately suppressed the selected
plugin's WRLD selection origin for a diagnostic comparison; this was not a
production behavior change or a validated replacement selection policy. Asset
impacts were omitted from both selections. The baseline count agrees with the
saved batch report.

The native probe also compared ordinary terrain extraction with the same
extractor given only the indexed LAND records for each sampled cell. The indexed
comparison includes copying those records into a temporary input. Vertices and
triangle indices matched byte for byte. This checks geometry, not the complete
reporting and failure contract of a future implementation.

Recast measurements use a synthetic flat exterior surface and fresh processes
for each partition strategy. They isolate generation from model extraction,
serialization, MO2 import, and border stitching. They do not establish an
end-to-end plugin speedup or equivalent navigation on complex scenes.

The existing output directory contains artifacts from multiple runs, including
a scene backup. Its inventory is not the disk footprint of the cancelled batch.
There is no measured complete Winterhold rebuild, no stage timing from the
cancelled run, and no reliable final output-size or completion-time estimate.
Probe sources and raw results are retained locally under
`output/performance-analysis/`; no source plugins or existing exports were edited.

## Priorities

| Priority | Change | Runtime benefit | Disk benefit | Main constraint |
| --- | --- | --- | --- | --- |
| Highest | Select cells from navigation-relevant changes | Avoid unnecessary generation and extraction | Avoid unnecessary candidates and model requests | Preserve historical moves, deletions, dependencies, and unknown-field fallback |
| Highest | Select intersecting placed references; cache decoded models | Avoid repeated model decoding, transforms, and cell-cache churn | Request fewer archived models | Correct worldspace, transforms, asset winners, and conservative bounds |
| High | Make diagnostic exports an explicit output policy | Avoid formatting unnecessary JSON and OBJ | Omit full load-order and inspection exports when only a plugin is needed | Keep required status, provenance policy, and writer verification |
| High | Index archive entries and read requested ranges | Avoid repeatedly reading whole archives | Reuse bounded extraction storage across runs | Preserve archive priority and cache invalidation |
| High | Index LAND records by CELL | Replace repeated full-record scans with direct lookup | None directly | Keep LAND order, inherited VHGT, and missing-data diagnostics |
| Medium | Compact evidence and stream finished work | Reduce allocations, copying, and paging | Reduce duplicate provenance and support restart | Reconcile borders before final export and verify the entire patch |
| Medium | Tune Recast partitioning and allocation lifetimes | Reduce generation-stage work | None directly | Preserve acceptable mesh quality and border partitions |
| Later | Reuse unchanged candidates; add bounded parallelism | Avoid repeated builds and use spare CPU | Avoid duplicate run artifacts | Complete dependency fingerprints and memory-aware scheduling |

## Target selection is much broader than the edited area

`CellImpactIndex::AffectedCells` in `src/skyrim/parser/affected_cells.cpp`
selects every CELL belonging to a worldspace when the selected plugin occurs in
that WRLD's origin chain. It does not ask which fields changed or whether they
influence navigation. The local reproduction overrides the shared worldspace.
The probe's WRLD-only counterfactual removes most selected cells and most cells
with decoded NAVM. This is the strongest measured opportunity to reduce both
runtime and output count.

The inventory found actual WRLD subrecord differences as well as removed
subrecords; the record must not simply be assumed identical. Compare versions
using explicit dependencies of the supported navigation pipeline. Exclude only
changes proven irrelevant, keep changes to geometry-relevant state, and retain
conservative selection for unclassified fields. Record selection reasons in the
batch report so a worldspace-wide expansion is visible before extraction.

The same issue exists at smaller scales: CELL metadata, LAND texture-only edits,
and base-record changes can select navigation even when the inputs used by
generation have not changed. A navigation-input fingerprint should cover
placement, scale, enable/deletion state, model winner, collision, LAND heights,
doors, and authored navigation/border inputs. It must retain the affected old
and new locations of moved or removed placements.

Missing historical model bounds deliberately expand impacts conservatively.
Do not replace that fallback with an arbitrary radius. Recover trustworthy model
bounds where possible, identify unresolved bounds explicitly, and keep broad
coverage when it cannot be established safely.

## Geometry suppliers multiply the work

`GeometryNeighbors` adds supplier **cells**, then `RunBatch` extracts all their
references and LAND before `PrepareRecastInput` clips supported triangles to the
target. The sampled exterior target has substantially more suppliers than the
cell cache can retain. This can defeat reuse even when consecutive targets are
nearby: repeated traversal of a supplier set larger than the cache can evict
entries before their next use. Increasing the cache alone also increases RAM.

Index individual placed-reference bounds and query their intersection with the
generation area before requesting assets or transforming triangles. Separate
the historical footprint used for impact selection from the winning footprint
used for generation. The current shared `Footprint` considers historical base
bounds even when constructing winning geometry suppliers. Preserve conservative
unknown-bound behavior and keep neighbor context until the generation boundary
contract has been reviewed; input clipping and seam quality are separate concerns.

`ExtractGeometry` calls `LoadNif` for each reference. Repeated placements of one
physical asset therefore repeat NIF parsing and collision/render extraction.
Cache immutable model-local collision geometry by resolved asset identity and
version, then apply each reference's transform. Cache missing and unsupported
results within the input snapshot as well. Use a byte budget rather than an
unbounded model table. The sampled reference-to-model ratio demonstrates reuse
potential; it is not an end-to-end speedup estimate.

Batch generation does not consume render fallback as navigation. Nevertheless,
`LoadNif` expands render shapes and `ExtractGeometry` transforms and retains
supplemental render geometry. Add a generation extraction policy that materializes
supported collision and required coverage information without building display
meshes. This avoids derived render work; Nifly may still need to parse the NIF
container. Keep the existing inspection path and unsupported-collision reporting.

`ExtractTerrain` in `src/skyrim/extraction/terrain_extractor.cpp` scans every
resolved record for each extracted supplier cell. Build an ordered CELL-to-LAND
index once. The sampled indexed extraction was dramatically faster with matching
geometry, although this stage alone cannot account for a whole plugin's cost.

## Disk output is dominated by diagnostics and cache policy

`app::Run` writes `load-order.json` for every resolved run before batch dispatch.
This reports the complete winning record table, including records unrelated to
the selected plugin. The existing reproduction's file is large compared with its
input plugin. `WriteCandidateJson` and `WriteCandidateObj` also run for every
completed batch candidate, including plugin-writing runs and empty candidates.

Provide explicit plugin-only, compact-evidence, and full-inspection output
policies. Plugin-only runs should retain the batch summary, input snapshot,
necessary warnings, and verified plugin while making large diagnostic exports
optional. Compact mode can use compressed evidence and omit redundant OBJ views.
Compression alone reduces stored bytes but still spends time producing evidence;
not producing optional artifacts is the stronger improvement.

The existing `--output-detail summary` controls GLB provenance in the single-cell
path. It does not suppress batch candidate JSON/OBJ, the complete load-order
report, model extraction, or the BSA cache. Scene bounds are rejected in batch
mode. These settings therefore do not solve plugin rebuild storage cost.

Batch evidence compaction remaps used source triangles, but moves the **entire**
geometry-source table into each result. `WriteCandidateJson` then serializes
every source entry, including neighboring and display-only sources unused by the
candidate. Compact that table too, remapping both triangle provenance and region
geometry-source joins. Retain enough metadata to explain incomplete coverage.

`ExtractBsaModels` stores extracted NIFs under the output folder and leaves them
there. A new output directory creates another extraction cache. The MO2 cache
similarly retains snapshot-specific loose-asset tables. Use a shared, versioned
asset cache with a storage budget, explicit retention policy, and run manifests
that refer to it. Never delete input assets or operator-owned exports. Cache
identity must include the winning loose/archive provider and its version; logical
path or existing-file checks alone are insufficient across profile changes.

Plugin-copy mode intentionally includes the original plugin's other records.
That requested output should remain complete; the opportunities are auxiliary
artifacts, extraction storage, and repeated temporary copies.

## Archive reading can dominate elapsed time

Each uncached source-cell extraction can launch `extract_bsa_models.py`. The
script walks archives in priority order and uses `BSAArchive.parse_file`, whose
implementation reads the entire archive into memory. Requests for a missing model
can keep the request set nonempty through all archives. This causes archive
reads and metadata parsing to repeat across source cells, even when only a few
small assets are needed. It increases I/O and RAM; reading an archive is not itself
additional allocated disk space.

Resolve requested models against an archive-content index once per snapshot,
then seek to the winning entries and decompress only their payloads. This also
allows precise archive asset impacts in place of selecting every model-bearing
location when an archive might contain replacements. Deduplicate requests and
apply reference exclusions before extraction. Implement the boundary in
project-owned code without editing vendored dependencies. Preserve filename
prefixes, per-entry compression rules, archive precedence, and error diagnostics.

## RAM findings that can affect runtime

The reader retains original payload bytes, decoded payload bytes, encoded
subrecords, and subrecord data independently. `ResolveLoadOrder` retains every
source plugin's records while producing winners and copies each record in its
resolution loop. The native measurements show substantial resident memory even
before geometry extraction. Store one backing payload with ranges/views, retain
unknown bytes losslessly, move records when ownership transfers, and resolve
payloads incrementally after collecting master metadata. Decode NAVM and LAND
on demand where the required ownership and skip policies permit it.

`GeometryExtraction` owns both `mesh` and `scene.mesh`. Cached entries are
deep-copied into temporary extractions. `AppendGeometry` also copies the entire
growing aggregate mesh into `scene.mesh` after each append. With many similarly
sized suppliers, repeated aggregate synchronization has quadratic copy volume
in supplier count. Use one authoritative mesh, immutable cache views, and a
single ordered assembly/compaction stage. Retain stable provenance remapping.

Completed candidates and evidence remain in `RunBatch::results` until border
reconciliation and serialization. Spool compact candidates into private staging
storage while retaining border descriptors and counts, then reconcile and stream
records into a temporary plugin. This trades RAM for staging disk; it only serves
the disk objective when staging is bounded, compact, and cleaned after successful
finalization. Writer read-back and reciprocal border validation must remain intact.

The Recast adapter retains raw heightfield, compact heightfield, and contours
through polygon construction. Release each intermediate after its last use.
The writer also reopens complete source plugins into buffers even though record
ranges are known; mapped or range-based input can avoid those additional copies.

## Recast and parallelism come after eliminating repeated work

Monotone partitioning is already available in both the CLI and persisted Windows
selector. The synthetic measurements favor it, but complex floors and border
partitions need validation before adopting it for a rebuild. The
[upstream Recast example](https://github.com/recastnavigation/recastnavigation/blob/main/RecastDemo/Source/Sample_TileMesh.cpp)
describes its speed advantage and possible long, thin polygons. It does not reduce
inspection output or model-cache disk usage.

Horizontal raster cost is proportional to rasterized area divided by the square
of horizontal voxel size, with additional costs for vertical spans. The current
exterior adapter clips input to the target CELL, so it does not voxelize the whole
supplier neighborhood. Coarsening resolution changes navigation detail and is
not the first optimization to make. Tiling large interiors needs overlap, global
reachability, stable seam geometry, and region thresholds independent of tile
layout; it cannot be introduced as an interchangeable implementation detail.

Parallelize independent target work only after removing archive rereads, model
redecoding, and excess copying. Limit workers by a shared byte budget and isolate
Recast allocations. Extraction currently uses shared request/cache manifest
filenames, and border reconciliation and final writing are global stages. They
must be made safe before parallel extraction; uncontrolled workers can increase
I/O contention, RAM, and temporary disk usage.

## Implementation and verification sequence

Start with stage timers, selection-reason counters, supplier/reference/model
counts, archive bytes read, model-cache hits, and bytes written by artifact type.
Preflight the target set and use a stratified sample of affected exterior,
interior, dense, and border cells to estimate completion cost with an explicit
uncertainty range. Checkpoint progress currently writes at start and terminal
states, so the saved running report does not establish how far generation got.

Then implement semantic impact selection, direct LAND lookup, model-local cache,
reference-level supplier filtering, and generation extraction as separate changes.
Add output policy and evidence compaction next, followed by indexed archive reads
and bounded shared cache retention. Measure again before changing partitioning,
adding incremental candidate reuse, or adding workers. Candidate fingerprints must
include neighboring inputs, authored portals, settings, and tool/schema versions;
changed borders must invalidate dependent neighbors.

For behavior-preserving work, compare generated geometry, ordered provenance,
diagnostics, skips, cancellation, topology, reciprocal borders, and writer
read-back. For semantic selection changes, use synthetic fixtures for irrelevant
WRLD/CELL metadata, real geometry changes, historical moves/deletions, changed
base objects, oversized/unknown bounds, and overlapping worldspaces. Keep fixture
names location independent. Any new operator option must be exposed, persisted,
validated, and passed through both CLI and Windows entry points to `app::Run`.
Build the desktop executable and invoke only CLI workflows for verification.
