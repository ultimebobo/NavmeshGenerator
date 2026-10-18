# Navmesh Generator API

This is the source reference for the project's C++ code. The public entry points
are in `src`; vendored code under `lib` and Python utilities under `tools` are outside
this reference. The showcase utilities and renderer are described in the
[scene image rendering guide](render-media.md).

## Code map

- `navmesh::core::TriangulateConvexHull`: closed, outward-facing hull triangles
  indexing finite input points without moving them; rejects degenerate or
  unrepresentable hulls. Used by Skyrim convex collision extraction.
- `navmesh::core::SimplifyGeneratedSurface`: conservative cleanup of an unlinked
  generated floor. Retriangulates nearly planar incident fans and removes straight
  exposed boundary samples without moving the rim or filling openings. Bounds
  cumulative vertical approximation against immutable input triangle contributors
  and preserves the slope envelope. Pins exterior CELL seams, respects the contour edge limit,
  preserves mixed-flag interfaces, and resets neighbors for subsequent rebuilding.
- `navmesh::core::SimplifyCandidateSurface`, `SurfaceSimplificationSettings`:
  bounded exposed-triangle pruning and fan cleanup after authored border
  repair. Pins door/portal triangles and CELL seams; transactionally remaps
  contributors, regions and connection indices. Uses immutable boundary-band
  coverage, shared-edge route guards and per-floor area budgets for conservative
  inward trimming, including uneven collision outlines. Fan approximation uses
  the vertical voxel budget left unspent by exact generator cleanup.
  Rejects independent edits once
  generated-to-generated portal identities require coordinated remapping.
- `navmesh::skyrim::DiscoverBaseObjectSwaps`, `ApplyBaseObjectSwaps`:
  effective loose configuration snapshots, BOS identity/precedence resolution
  and runtime alternatives separated from encoded record history.
  `BaseObjectSwapSnapshotMatches` guards publication; `WriteBaseObjectSwapReport`
  exports winner identity and placement evidence.
- `navmesh::core::PlacementEnvelope`, `GeometrySource::solidBounds`:
  model-placement uncertainty in Skyrim world units and filled world obstruction
  volumes. Extraction encloses possible collision; generation fills interiors
  and excludes their roofs from support. Analysis uses `TriangleSource::obstacleOnly`
  to exclude speculative floor hits. See [the BOS contract](base-object-swapper.md).

- `navmesh::core::StitchCandidateBorders`: transactional authored portal repair,
  including terminal boundary chains, authored floor slope envelopes and shared
  corner height offsets. Optional generation-scene evidence identifies crossings
  onto excluded obstacle collision; only positive interior support together with
  an absent compatible boundary floor can close such a crossing with a warning.
  Unresolved required crossings remain validation failures.

- `navmesh::core::ApplyAuthoredBorderVertexEdits` and
  `CompleteAuthoredBorderFanEdits`: neutral validation and bounded local height
  relaxation for adjacent authored boundary fans. Preserve vertex and triangle
  identities, reject stale or conflicting plans, maintain each incident face's
  winding and slope envelope, and leave pinned portals fixed. Candidates retain
  these edits for cache reuse, audit and guarded reciprocal export.
- `navmesh::skyrim::ExtractTerrain`: decode owned LAND, or explicitly inherited
  coordinate-matched parent LAND, with actual source provenance and cycle checks.

- `navmesh::core::RemoveCandidateIslands`: complete-set floor reachability after
  authored border matching or generated seam linking. Retains matched doors and required crossings into
  untouched navigation; doorless selections and interiors also retain a primary floor. Removes
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
  from CELL/WRLD data and closest-floor authored preferred-path transfer after border reshaping.
  It preserves geometry and connection bits; `WaterFlag` and `PreferredPathFlag`
  name the independent Skyrim NVNM classification bits.
- `navmesh::core::Cell::waterHeight`: supported effective CELL water-surface Z
  supplied by the Skyrim parser, including winning worldspace inheritance.

- `navmesh::core`: neutral geometry, world and navmesh data, scene provenance,
  candidate generation, and exports.
- `navmesh::core::GeometrySourceType::NavigationExclusion`: asset-derived
  obstacle geometry that Recast rasterizes without admitting walkable floors;
  extraction retains source provenance separately from Havok and visual meshes.
- `navmesh::core::WriteCombinedGlb`: ordered scene groups, separate selected-cell
  originals, door-linked face materials, exit markers, and authored/generated
  connection bars along recorded portal edges. Selection omits bars without both displayed endpoint triangles.
  Candidate display suppresses authored links involving the selected cell's original NAVMs.
- `navmesh::skyrim::DecodeNavmeshConnections`: bounded inspection decoding
  of consumed external entries and door associations, retaining raw plugin bytes.
- `navmesh::core::SharedBytes`: immutable shared byte ownership and bounded slices
  for parser payload/subrecord data without repeated copies.
- `navmesh::core::ContentHash`: incremental SHA-256 identities for cache dependencies.
- `navmesh::core::NativeFilePath`: native I/O path preparation, including
  normalized Windows extended local and UNC paths for candidate cache files.
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
  AllWalkable retains all surviving floor components for subsequent linking and
  reachability filtering in both single-cell and batch generation. Vertical
  collision faces contribute obstruction evidence; height-detail triangles
  follow the compact heightfield with movement-bounded error. Convex contour
  patches merge before sampling; collapsed regions trigger contour refinement.
  `GeometrySource::navigationObstacle` excludes tagged solids from walkable floors
  while retaining obstruction evidence. Skyrim extraction tags landscape rock assets.
  Unsupported raster seams can be repaired within the agent radius between
  climb-compatible standing floors, subject to solid obstruction and clearance.
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
  Immediate linking defaults to retaining only components connected to real
  portals or matched doors. Deferred reachability keeps all components for the
  shared island filter while immediate linking still retracts
  unmatched CELL seam wedges while preserving shared interior edges. Border
  preparation coalesces compatible generated boundary subdivisions and splits
  containing edges with consistent region, source, contour, door,
  and portal joins. Inward offsets trim the candidate; outward offsets extend it.
  Terminal endpoint alignment updates complete incident fans. Final validation
  checks exact reversed endpoint equality, unique portals and complete seam coverage.
  Deferred batch mode preserves unlinked borders and unanchored components while
  matching only untouched authored neighbors.
- `navmesh::core::AuthoredBorderTolerance`: shared world-unit bound for incidental authored
  portal drift. Explicit reciprocal crossings may use the profile's bounded border halo.
- `navmesh::skyrim`: plugin/load-order reading, guarded NAVM override
  writing, Mod Organizer 2 and Vortex input, and terrain/model extraction.
- `navmesh::skyrim::ImportVortexDeployment`, `VortexInput`, `VortexSnapshotMatches`:
  read-only active deployment import, case-insensitive plugin and model winners,
  ordered active archives, input reporting and coherence checks. No Vortex
  database or staging-folder changes are required.
- `navmesh::app::ModManager`: persisted desktop source choice with MO2 as default.
- `navmesh::analysis`: spatial queries and navmesh discrepancy analysis.
- `navmesh::validation`: cell validation findings.
- `navmesh::app`: command-line options and the shared application runner.
- `navmesh::app::ParseCellSelection`, `Options::cellSelection`, `UsesBatchGeneration`:
  shared Cell-mode list parsing and coordinated generation dispatch. Identifiers are Form
  IDs or editor IDs; unknown or ambiguous matches fail before generation, and
  aliases deduplicate by resolved CELL identity.
- `navmesh::ui::RunWindowsUi`: dark Dear ImGui desktop host, persisted manager selection and source
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
  and authored-neighbor reciprocal portal checks, cover, NAVI and reciprocal XNDP.
- `navmesh::core::CaptureCoverGeometry`, `BuildNavmeshCover`: physical wall evidence
  retained through seam refinement and final cover-slot encoding without triangle renumbering.
- `navmesh::skyrim::detail::RebuildNavmeshMetadata`, `RewriteReferenceNavigation`:
  auxiliary constraint remapping and complete winning-reference preservation with FormID rebasing.
- `navmesh::skyrim::detail::ValidateNavigationGraph`: exported grid, cover and connection
  checks against the effective load order, including unchanged destinations.
- `navmesh::skyrim::LoadOrderSnapshotMatches`: read-only rejection of changed plugin inputs.
- `navmesh::skyrim::detail::BuildNavmeshInfo`: NAVI payload construction from final
  output-local NVNM geometry, retaining unrelated source-copy navigation entries.
- `navmesh::skyrim::detail::PluginCopy`, `PreparePluginCopy`, and
  `MergePluginCopy`: complete raw-envelope validation, identity allocation limits,
  and navigation-record insertion into a byte-preserving source copy. `WriteNavmeshOverrides`
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
