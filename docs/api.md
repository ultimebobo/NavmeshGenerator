# Navmesh Generator API

This is the source reference for the project's C++ code. The public entry points
are in `src`; the `lib` and `tools` trees contain third-party code and are outside
this reference.

## Code map

- `navmesh::core::StitchCandidateBorders`: transactional authored portal repair,
  including terminal boundary chains, authored floor slope envelopes and shared
  corner height offsets. Optional generation-scene evidence identifies crossings
  onto excluded obstacle collision; only positive interior support together with
  an absent compatible boundary floor can close such a crossing with a warning.
  Unresolved required crossings remain validation failures.

- `navmesh::core::RemoveCandidateIslands`: complete-set floor reachability after
  generated seam linking. Retains matched-door/authored-neighbor access and the
  largest network in each contiguous exterior selection or interior; removes
  isolated floor networks even across CELL seams. Compacts source, geometry,
  region, door and reciprocal portal indices, then rebuilds contours and topology.
  `CandidateReachabilityTarget` supplies borrowed candidates and neutral ownership keys.

- `navmesh::app::detail::CellScene`: optional complete Cell-selection scene
  accumulation, source-triangle deduplication, authored navigation and exit joins,
  and checked GLB/sidecar export. Storage retains native Skyrim coordinates.
- `navmesh::core::SceneCandidate`: borrowed finalized candidate with an owning
  CELL identity, used to resolve generated connections without plugin allocation.
  `SceneExportResult::written` confirms GLB and provenance output success.
  Combined scene metadata records its borrowed `selectedCells` selection.

- `navmesh::core::detail::BuildRetainedRegionContours`: Recast contour coverage
  and shared-interface refinement, with layer recovery over already-retained
  spans when watershed contours remain incomplete. Recovery adds a warning.
- `navmesh::core::detail::BuildRetainedRegionMesh`: coarse polygon construction
  with clockwise convex floor footprints and consistent directed edges. Refines
  the complete contour partition and permits watershed layer recovery over retained spans.
  `HasConsistentRegionMesh` checks the coarse mesh without modifying it.

- `navmesh::core::ValidateRecastSceneHeightRange`: validates supported,
  generation-clipped scene heights against Recast's packed raster span range;
  used before cached batch candidates can be accepted.
- `navmesh::app::detail::BuildBatchCandidate`: cell-local generation failures
  return `skipped_generation_failed` without a candidate or audit. Allocation exceptions and
  evidence failures return fatal `failed` results. Border reconciliation treats
  skipped generation targets as untouched authored neighbors.

- `navmesh::core::TagCandidateTriangles`: optional centroid-based water detection
  and closest-floor authored water/preferred-path transfer after border reshaping.
  It preserves geometry and connection bits; `WaterFlag` and `PreferredPathFlag`
  name the independent Skyrim NVNM classification bits.
- `navmesh::core::Cell::waterHeight`: supported effective exterior water-surface Z
  supplied by the Skyrim parser, including winning worldspace inheritance.

- `navmesh::core`: neutral geometry, world and navmesh data, scene provenance,
  candidate generation, and exports.
- `navmesh::core::WriteCombinedGlb`: ordered scene groups, separate selected-cell
  originals, door-linked face materials, exit markers, and authored/generated
  connection bars along recorded portal edges. Selection omits bars without both displayed endpoint triangles.
  Candidate display suppresses authored links involving the selected cell's original NAVMs.
- `navmesh::skyrim::DecodeNavmeshConnections`: bounded inspection decoding
  of consumed external entries and door associations, retaining raw plugin bytes.
- `navmesh::core::SharedBytes`: immutable shared byte ownership and bounded slices
  for parser payload/subrecord data without repeated copies.
- `navmesh::core::ContentHash`: incremental SHA-256 identities for cache dependencies.
- `navmesh::skyrim::ModelGeometryCache`: byte-budgeted model-local and
  transformed-placement reuse with extraction-policy/provider revision keys.
- `navmesh::skyrim::BsaModelExtractor`: run-scoped lazy archive indexes,
  bounded selective zlib/LZ4 decoding, winning changed-model names, negative
  caching, and atomic extracted-file publication without a Python runtime.
- `navmesh::skyrim` asset-cache helpers: shared archive snapshot identity,
  winning changed-model lookup, and eviction confined to generated files.
- `navmesh::app::detail::BuildBatchCandidate`: one isolated worker's all-walkable
  generation, untouched-neighbor stitching, validation, compact evidence and audit cache.
- `navmesh::app::detail::ReconcileBatchBorders`: dispatches complete generated candidates
  to neutral batch seam refinement using winning CELL ownership.
- `navmesh::core::StitchGeneratedCandidates`, `GeneratedCellCandidate`: common seam
  partitions, shared corner heights, stable provenance/door joins and exact reciprocal
  portals between generated CELLs, including targets without authored NAVM.
- `navmesh::core::TriangulateDetailSamples`: constrained boundary triangulation and
  insertion of floor-height samples when a Recast detail patch overlaps itself;
  preserves hull segments and every sample position without authored geometry.
- `navmesh::core::RefreshCandidateTopology`: rebuilds internal adjacency, region areas
  and boundary loops after geometry refinement without changing polygon identities.
- `navmesh::app::detail` candidate-cache/artifact helpers: input fingerprints
  with explicit pipeline compatibility across executable rebuilds,
  bounded private gzip reads/writes and streaming public gzip JSON.
- `navmesh::core::CandidateGenerator`: interchangeable scene-to-candidate
  interface. `RecastCandidateGenerator` is the application implementation and
  links against the Recast Navigation submodule under `lib/recastnavigation`.
  It clips output after exterior-halo rasterization and retains only shared-edge
  components reaching a matched door or an exterior boundary edge under Anchored retention;
  AllWalkable retains all surviving floor components for batch linking. Vertical
  collision faces contribute obstruction evidence; height-detail triangles
  follow the compact heightfield with movement-bounded error. Convex contour
  patches merge before sampling; collapsed regions trigger contour refinement.
  `GeometrySource::navigationObstacle` excludes tagged solids from walkable floors
  while retaining obstruction evidence. Skyrim extraction tags landscape rock assets.
  Candidate warnings identify adaptive voxel width and climb quantization.
- `navmesh::core::StitchCandidateBorders`: matches complete neighboring edges and
  reconciles authored partitions before deferred batch linking, including cells
  without authored crossings. Supported interior samples and climb-compatible
  border strips preserve floor slope limits and authored endpoint heights.
  Selected-cell authored NAVMs supply required crossing constraints. Missing
  crossings are repaired by constrained cavity retriangulation with stable source,
  region, door, and portal joins; unresolved crossings invalidate the candidate.
  Near-coincident endpoints align through complete fans, and bounded interior
  height samples preserve the unchanged rim's slope envelope when needed.
  It retains only components connected to real portals or matched doors and retracts
  unmatched CELL seam wedges while preserving shared interior edges. Border
  preparation coalesces compatible generated boundary subdivisions and splits
  containing edges with consistent region, source, contour, door,
  and portal joins. Inward offsets trim the candidate; outward offsets extend it.
  Terminal endpoint alignment updates complete incident fans. Final validation
  checks exact reversed endpoint equality, unique portals and complete seam coverage.
  Deferred batch mode preserves unlinked borders and unanchored components while
  matching only untouched authored neighbors.
- `navmesh::core::AuthoredBorderTolerance`: shared world-unit bound for authored
  portal drift from exterior CELL borders, used by stitching and the guarded writer.
- `navmesh::skyrim`: plugin/load-order reading, guarded NAVM override
  writing, Mod Organizer 2 input, and terrain/model extraction.
- `navmesh::analysis`: spatial queries and navmesh discrepancy analysis.
- `navmesh::validation`: cell validation findings.
- `navmesh::app`: command-line options and the shared application runner.
- `navmesh::app::ParseCellSelection`, `Options::cellSelection`, `UsesBatchGeneration`:
  shared Cell-mode list parsing and coordinated generation dispatch. Identifiers are Form
  IDs or editor IDs; unknown or ambiguous matches fail before generation, and
  aliases deduplicate by resolved CELL identity.
- `navmesh::ui::RunWindowsUi`: dark Dear ImGui desktop host, persisted MO2
  controls, background shared-run execution, progress and safe cancellation.
- `navmesh::ui::PrepareDesktopOptions`: rendering-independent contextual option
  preparation for cell lists, plugin, load-order and catalog-export actions.
- `navmesh::ui::ResetAdvancedNumericalOptions`: restores shared numerical
  defaults throughout the desktop draft, including hidden controls; the desktop
  persists the result through its ordinary settings writer.
- `navmesh::core::RecastSettings`, `NavigationProfile` and `ValidateRecastSettings`:
  shared movement, voxel, contour and region controls with finite/range validation.
- `navmesh::app::detail::ResolveCellSelection`: unique Form ID/editor ID resolution
  against a shared snapshot, reused by Cell inspection and coordinated generation.
- `navmesh::app::detail::RunBatch`: affected or explicitly selected cell orchestration with bounded
  geometry reuse and combined plugin writer dispatch.
- `navmesh::app::detail` geometry-pipeline helpers: world-space extraction
  composition, bounds filtering of support and display geometry, and analysis
  provenance joins.
- `navmesh::cli` inspection-report functions: inspection OBJ/JSON/HTML exports,
  cell and load-order listings, and console analysis summaries.
- `navmesh::skyrim::CellImpactIndex`: affected-cell discovery, historical
  reference footprints, worldspace coordinate indexing and geometry suppliers.
- `navmesh::skyrim::SelectCollisionAffectedCells`, `CollisionImpactInput`:
  terrain/water and collision-based batch target selection. Historical placement
  and base-model states are compared through shared extraction; only changed
  collision triangles select exterior footprints. Missing models supply no
  collision and are recorded for operator warnings; unreadable models and
  incomplete archive searches stop selection. The spatial index supplies horizontal bounds queries while exact
  triangle/cell intersection excludes empty corners and vertical-only extent.
  New terrain and collision can select uncovered cells, including new worldspaces
  and submerged terrain. Water-only impacts require supported winning geometry.
  `ImpactSelectionStatistics` records unique overlapping input contributions and
  water owners without supported geometry.
- `navmesh::skyrim::WriteNavmeshOverrides`: combined batch serialization
  of existing overrides and new plugin-owned NAVMs, with generated-to-generated
  and authored-neighbor reciprocal portal checks.
- `navmesh::skyrim::detail::PluginCopy`, `PreparePluginCopy`, and
  `MergePluginCopy`: complete raw-envelope validation, identity allocation limits,
  and NAVM insertion into a byte-preserving source copy. `WriteNavmeshOverrides`
  accepts an optional active filename for this authoring export and verifies
  modified NAVMs against the source's unchanged master table and full/light flags.
- `navmesh::skyrim::CellsWithExistingNavmesh`: winning NAVM ownership for
  the shared runner's optional uncovered-cell generation policy, independent of
  whether polygon geometry can be decoded.

Build, dependency setup, formatting, and regression commands are in the
[development guide](development.md). Current CLI inputs and exports are in the
[command-line reference](command-line.md).

The data flow and design constraints are described in
[Architecture](architecture.md), [Coordinate system](coordinate-system.md),
[Candidate surface algorithm](candidate-surface-algorithm.md),
[Operator workflow](operator-workflow.md), and [Batch rebuilding](batch-rebuilding.md).
The [glossary](glossary.md) explains border connections and NAVM terminology.
The generated namespaces, classes, files, and functions are available from the
navigation pane.

## Generate locally

Install Doxygen, then run this command from the repository root:

```powershell
doxygen Doxyfile
```

Open `build/doxygen/html/index.html` in a browser. Generated files stay under
`build/` and are ignored by Git. Doxygen is needed only to produce the reference,
not to build the application.

Internal generation and analysis stages are named by their responsibility and
document their non-obvious assumptions beside the implementation. The
[architecture guide](architecture.md) describes those boundaries; the
[agent guidelines](../AGENTS.md) and repository `.clang-format` govern readability
and proportionate use of SOLID for future changes.
