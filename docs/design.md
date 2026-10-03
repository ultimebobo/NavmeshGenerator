# Task: Build the first POC for Skyrim Navmesh Automation

We are developing a tool to investigate whether Skyrim navmesh generation/repair can be meaningfully automated.

## Critical clarification

This is a **modding/data-processing tool**.

Do NOT build an SKSE runtime plugin.
Do NOT inspect the navmesh from inside a running Skyrim process.
Do NOT depend on Skyrim being launched.

The purpose of this project is to process Skyrim's mod/plugin data directly on disk and eventually generate/repair NAVM records.

The first POC should produce a tangible artifact that demonstrates that we can read a Skyrim cell, understand its existing NAVM, and extract the surrounding world geometry needed for future automated generation.

---

# Product direction

The eventual pipeline is:

```
Skyrim load order
       |
       v
parse ESM/ESP/ESL
       |
       v
identify cells/worldspaces
       |
       v
extract terrain + static/reference geometry
       |
       v
construct neutral geometry representation
       |
       v
generate candidate walkable surface
       |
       v
compare against existing NAVM
       |
       v
repair/regenerate affected regions
       |
       v
write a minimal compatible plugin
```

The POC only needs to implement the first part of this pipeline.

---

# POC objective

Given:

* a Skyrim Special Edition/AE Data directory
* a plugin such as Skyrim.esm or a small test ESP
* a target cell

produce a report and geometry export describing:

1. the cell
2. its references
3. its existing NAVM
4. NAVM vertices/polygons/connectivity where accessible
5. static/reference geometry relevant to navigation
6. terrain information where accessible
7. bounding boxes
8. enough information to visualize the result externally

The success criterion is:

> We can point the tool at a real Skyrim cell and obtain a useful representation of the existing navigation and surrounding geometry.

Do not attempt automatic navmesh generation yet.

---

# Repository architecture

Keep the project independent of Skyrim runtime execution.

Use a structure approximately like:

```
src/
  app/
  skyrim/
    records/
    parser/
    extraction/
  model/
    geometry/
    navigation/
    world/
  analysis/
  output/

tests/

tools/

docs/
```

The important architectural boundary is:

```
Skyrim plugin data
      |
      v
Skyrim extraction layer
      |
      v
neutral data model
      |
      v
analysis algorithms
```

The `model` layer must NOT depend on CommonLibSSE-NG or Skyrim runtime types.

---

# Technology choice

Use the most appropriate existing Skyrim plugin parsing library rather than forcing CommonLibSSE-NG into the problem.

Investigate existing options first.

In particular evaluate:

* xEdit/libxEdit ecosystem
* zlib/ESP/ESM parsing implementations
* CommonLibSSE-NG only if it is genuinely useful for reading plugin files
* existing open-source Skyrim record parsers

The tool must ultimately be capable of reading ESM/ESP/ESL files directly.

If an existing mature parser can save substantial effort, use it rather than reimplementing the binary plugin format.

Document the choice and why it was made.

---

# Important: do not fake the POC

Do not create mock NAVM data and call the POC complete.

The tool must consume actual Skyrim data.

If a particular piece of data cannot yet be extracted, explicitly report that limitation.

A smaller real POC is preferable to a large simulated implementation.

---

# First concrete target

Support one known cell first.

Make the CLI something approximately like:

```
skyrim-nav-poc \
    --data "D:/Games/Skyrim Special Edition/Data" \
    --plugin Skyrim.esm \
    --cell <cell identifier> \
    --output ./output
```

Choose a practical way to identify a cell.

If useful, also support:

```
--worldspace Tamriel
--cell-x 10
--cell-y -5
```

or another sensible Skyrim-native representation.

---

# Output

Produce at least:

```
output/
  report.json
  navmesh.json
  geometry.json
```

The exact format is up to you.

The JSON should contain enough information to inspect:

### Cell

* form ID
* editor ID if available
* worldspace
* cell coordinates
* bounds

### References

For each relevant reference:

* form ID
* base form
* editor ID/name where available
* position
* rotation
* scale
* bounds if available

### NAVM

Extract as much as the parser/API reliably provides:

* NAVM form ID
* vertices
* polygons
* polygon vertex indices
* adjacency/connectivity
* flags
* relevant metadata

### Geometry

Represent geometry using a neutral format:

```
Triangle {
    Vec3 a;
    Vec3 b;
    Vec3 c;
}
```

Also provide:

* source reference ID
* source type
* transform
* bounds

If complete collision extraction is not yet possible, implement the best reliable geometry source available and clearly document what is missing.

---

# Visual proof

A POC should produce something that can be inspected.

If practical, implement an export compatible with an existing visualization format, for example:

* OBJ for geometry
* OBJ/PLY for NAVM polygons
* glTF if convenient

For example:

```
output/
  geometry.obj
  navmesh.obj
  report.json
```

The goal is to be able to open the output in Blender/MeshLab/etc. and visually confirm:

```
Skyrim geometry
      +
existing navmesh
```

are spatially aligned.

If producing a direct visualization is easier, a tiny standalone viewer is also acceptable, but do not spend significant time building a GUI.

---

# Coordinate systems

Be extremely careful about Skyrim coordinate systems.

Document:

* Skyrim coordinates
* plugin/worldspace coordinates
* exported coordinate system
* any axis/sign conversions

Make the output visually verifiable.

A technically correct parser with incorrect transforms is not a successful POC.

---

# NAVM analysis

Add a basic analysis pass.

At minimum calculate:

* polygon count
* vertex count
* connected components
* bounding box
* polygon area statistics
* min/max/average polygon area
* isolated polygons/components
* degenerate polygons
* duplicate vertices if detectable
* basic adjacency statistics

Produce these in `report.json` and human-readable CLI output.

Example:

```
Cell: 0000xxxx
Worldspace: Tamriel

NAVM:
  vertices:          1842
  polygons:           421
  connected regions:    2
  isolated polygons:    3
  degenerate polygons:  0

Geometry:
  triangles:         9231
  references:         147
```

---

# Geometry extraction

Do NOT attempt to solve every Skyrim geometry case.

For the first POC, explicitly target:

1. static references with usable collision/mesh data
2. terrain if reasonably accessible

Prioritize correctness over coverage.

If NIF/Havok collision extraction is a substantial blocker, identify the exact blocker and implement the simplest useful geometry source first.

The POC should still demonstrate a real path from:

```
plugin -> reference -> transform -> geometry
```

for at least some real static objects.

---

# Testing

Add automated tests for the neutral model and analysis layer.

Also add at least one integration test using a real Skyrim fixture if licensing/repository constraints permit.

Do not commit copyrighted Skyrim game assets to the repository.

Instead support a configurable test-data path, e.g.:

```
SKYRIM_DATA_DIR=/path/to/skyrim/Data
```

Tests should skip gracefully when the local Skyrim installation is unavailable.

---

# Dependency policy

Do not introduce large dependencies without first checking whether an existing dependency in the repository already solves the problem.

Keep dependencies isolated.

The eventual navigation-generation algorithm should be able to operate without Skyrim-specific types.

---

# Documentation

Update README.md with:

## What this is

Experimental Skyrim navmesh analysis/generation tooling.

## What it currently does

Be precise and only claim functionality that actually works.

## Example

Show a real command line invocation.

## Output

Explain the generated files.

## Current limitations

Explicitly list unsupported geometry/reference types.

## Next milestone

The next milestone after this POC is:

```
geometry -> walkability analysis -> candidate navmesh
```

not runtime integration.

Also create:

```
docs/architecture.md
```

describing the intended pipeline and separation between:

* plugin parsing
* Skyrim extraction
* neutral geometry
* navigation algorithms
* analysis
* future NAVM serialization

---

# Engineering requirements

1. The project must build from a clean checkout.
2. Avoid hard-coded paths.
3. Avoid hard-coded FormIDs except in tests/examples.
4. Do not require Skyrim to run.
5. Do not require SKSE.
6. Do not modify the user's Skyrim installation.
7. Do not write changes back into original ESM/ESP files.
8. Do not implement generation yet.
9. Do not pretend unsupported data is supported.
10. Prefer a small working implementation over incomplete abstractions.

---

# Definition of done

The first iteration is complete only when:

* [ ] The repository builds from a clean checkout.
* [ ] The tool runs against an actual Skyrim installation.
* [ ] It reads an actual ESM/ESP/ESL file.
* [ ] It identifies a real cell.
* [ ] It extracts an actual existing NAVM.
* [ ] NAVM polygons/vertices can be exported or otherwise inspected.
* [ ] At least some real world/reference geometry can be extracted.
* [ ] Geometry and NAVM use a consistent coordinate system.
* [ ] A real cell can be exported to inspect visually.
* [ ] A JSON diagnostic report is produced.
* [ ] Basic NAVM statistics/validation are produced.
* [ ] Tests exist for the neutral representation and analysis.
* [ ] README and architecture documentation are updated.

Do not move on to automatic generation until this definition of done is genuinely satisfied.

---

# Final instruction

Work directly in the repository.

First inspect the existing code and repository structure.

Then inspect available/open-source Skyrim parsing approaches and determine the most appropriate approach.

Implement the smallest end-to-end vertical slice that satisfies the definition of done.

Do not spend the iteration designing future systems that aren't required for the POC.

At the end, provide a concise summary of:

1. what was implemented
2. how to build it
3. the exact command to run it against Skyrim
4. what real Skyrim data was successfully extracted
5. what remains blocked
6. the recommended next experiment
