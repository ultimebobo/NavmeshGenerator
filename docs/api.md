# Navmesh Generator API

This is the source reference for the project's C++ code. The public entry points
are in `src`; the `lib` and `tools` trees contain third-party code and are outside
this reference.

## Code map

- `navmesh::core`: neutral geometry, world and navmesh data, scene provenance,
  candidate generation, and exports.
- `navmesh::skyrim::offline`: plugin/load-order reading, Mod Organizer 2 input,
  and terrain/model extraction.
- `navmesh::analysis`: spatial queries and navmesh discrepancy analysis.
- `navmesh::validation`: cell validation findings.
- `navmesh::app`: command-line options and the shared application runner.

The data flow and design constraints are described in
[Architecture](architecture.md), [Coordinate system](coordinate-system.md),
[Candidate surface algorithm](candidate-surface-algorithm.md), and
[Operator workflow](operator-workflow.md). The generated namespaces, classes,
files, and functions are available from the navigation pane.

## Generate locally

Install Doxygen, then run this command from the repository root:

```powershell
doxygen Doxyfile
```

Open `build/doxygen/html/index.html` in a browser. Generated files stay under
`build/` and are ignored by Git. Doxygen is needed only to produce the reference,
not to build the application.
