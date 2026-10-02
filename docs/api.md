# Navmesh Generator API

This is the source reference for the project's C++ code. The public entry points
are in `src`; the `lib` and `tools` trees contain third-party code and are outside
this reference.

## Code map

- `navmesh::core`: neutral geometry, world and navmesh data, scene provenance,
  candidate generation, and exports.
- `navmesh::core::CandidateGenerator`: interchangeable scene-to-candidate
  interface. `RecastCandidateGenerator` is the application implementation and
  links against the Recast Navigation submodule under `lib/recastnavigation`.
- `navmesh::core::AuthoredBorderTolerance`: shared world-unit bound for authored
  portal drift from exterior CELL borders, used by stitching and the guarded writer.
- `navmesh::skyrim::offline`: plugin/load-order reading, guarded NAVM override
  writing, Mod Organizer 2 input, and terrain/model extraction.
- `navmesh::analysis`: spatial queries and navmesh discrepancy analysis.
- `navmesh::validation`: cell validation findings.
- `navmesh::app`: command-line options and the shared application runner.
- `navmesh::app::detail::RunBatch`: affected-cell orchestration with bounded
  geometry reuse and combined plugin writer dispatch.
- `navmesh::app::detail` geometry-pipeline helpers: world-space extraction
  composition, bounds filtering of support and display geometry, and analysis
  provenance joins.
- `navmesh::cli` inspection-report functions: inspection OBJ/JSON/HTML exports,
  cell and load-order listings, and console analysis summaries.
- `navmesh::skyrim::offline::CellImpactIndex`: affected-cell discovery, historical
  reference footprints, worldspace coordinate indexing and geometry suppliers.
- `navmesh::skyrim::offline::WriteNavmeshOverrides`: combined batch serialization
  of existing overrides and new plugin-owned NAVMs, with generated-to-generated
  and authored-neighbor reciprocal portal checks.
- `navmesh::skyrim::offline::CellsWithExistingNavmesh`: winning NAVM ownership for
  the shared runner's optional uncovered-cell generation policy, independent of
  whether polygon geometry can be decoded.

The data flow and design constraints are described in
[Architecture](architecture.md), [Coordinate system](coordinate-system.md),
[Candidate surface algorithm](candidate-surface-algorithm.md),
[Operator workflow](operator-workflow.md), and [Batch rebuilding](batch-rebuilding.md).
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
