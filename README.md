# Navmesh Generator

Offline experimental Skyrim navmesh analysis tooling.

## What this is

This project is an offline, read-only prototype for inspecting Skyrim plugin data without requiring an active Skyrim runtime or SKSE. The focus is on parsing plugin records directly, extracting cell and navmesh information from a plugin file, and turning that into a neutral geometry + navmesh model that can be analyzed and exported for inspection.

## What it currently does

- Builds a neutral, CommonLib-free C++ core for geometry and navmesh data.
- Extracts basic cell and reference information from Skyrim plugin records in a direct offline pass.
- Parses a subset of `CELL`, `REFR`/`ACHR`, and `NAVM` record payloads when present.
- Produces JSON diagnostics and OBJ exports for navmesh polygon inspection.
- Computes basic navmesh statistics such as vertex counts, polygon area range, connected components, and degenerate polygons.
- Includes automated tests for neutral geometry and navmesh analysis.

## Example

```powershell
xmake build navmesh-offline
./build/windows/x64/releasedbg/navmesh-offline.exe --plugin "D:/Games/Skyrim Special Edition/Data/Skyrim.esm" --cell-formid 00027D1C --output ./output
```

The tool prints the resolved cell and also accepts an editor ID or exterior cell coordinate pair:

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --plugin "D:/Games/Skyrim Special Edition/Data/Skyrim.esm" --editor-id KilkreathRuins03 --output ./output
```

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --plugin "D:/Games/Skyrim Special Edition/Data/Skyrim.esm" --cell-x 10 --cell-y -5 --output ./output
```

Use `--list-cells` to discover available form IDs, editor IDs, and exterior coordinates:

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --plugin "D:/Games/Skyrim Special Edition/Data/Skyrim.esm" --list-cells
```

## Output

The CLI writes files into the target output directory:

- `report.json` — cell metadata, references, navmesh summaries, and validation findings.
- `navmesh.obj` — exported navmesh polygon geometry for inspection in Blender or MeshLab.
- `geometry.obj` — exported neutral mesh/geometry derived from the parsed records.

## Current limitations

- This is not an SKSE runtime plugin and does not inspect a running game session.
- The direct parser is intentionally small and targets the common Bethesda plugin structure, not the entire plugin ecosystem.
- Geometry extraction supports loose NIF files and uses the `tools/BSAFileExtractor` submodule to cache requested BSA-backed assets on demand. Collision/Havok extraction is not complete.
- Install the BSA bridge dependencies with `python -m pip install -r tools/requirements.txt` before extracting archived assets.
- Archived assets are cached under `<output>/.bsa-cache`; delete that directory to rebuild the cache.
- Terrain extraction is not implemented beyond any simple geometry that is explicitly included in the parsed records.
- Some Skyrim record variants and non-standard modded data layouts may still be rejected or reported as unsupported.

## Next milestone

The next milestone after this POC is:

```text
geometry -> walkability analysis -> candidate navmesh
```

not runtime integration.

See [docs/architecture.md](docs/architecture.md) for the intended pipeline and separation between parsing, extraction, neutral model, analysis, and future NAVM serialization.
