# Navmesh Generator

Offline experimental Skyrim navmesh analysis tooling.

## What this is

This project is an offline, read-only prototype for inspecting Skyrim plugin data without requiring an active Skyrim runtime or SKSE. The focus is on parsing plugin records directly, extracting cell and navmesh information from a plugin file, and turning that into a neutral geometry + navmesh model that can be analyzed and exported for inspection.

## What it currently does

- Builds a neutral, CommonLib-free C++ core for geometry and navmesh data.
- Extracts basic cell and reference information from Skyrim plugin records in a direct offline pass.
- Reads the roadmap record subset (`WRLD`, `CELL`, `LAND`, `REFR`/`ACHR`, base-model records, and `NAVM`) through a loss-aware record layer: byte ranges, compressed source bytes, decoded subrecords, and unknown fields are retained for future round trips.
- Produces JSON diagnostics and OBJ exports for navmesh polygon inspection.
- Computes basic navmesh statistics such as vertex counts, polygon area range, connected components, and degenerate polygons.
- Includes automated tests for neutral geometry and navmesh analysis.

## Example

```powershell
xmake build navmesh-offline
./build/windows/x64/releasedbg/navmesh-offline.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --cell-formid 00027D1C --output ./output
```

The tool prints the resolved cell and also accepts an editor ID or exterior cell coordinate pair:

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --editor-id KilkreathRuins03 --output ./output
```

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --cell-x 10 --cell-y -5 --output ./output
```

Use `--list-cells` to discover available form IDs, editor IDs, and exterior coordinates:

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --list-cells --output "D:/NavmeshWork/cells"
```

## Mod Organizer 2 input

The intended normal workflow is to select an existing Mod Organizer 2 instance and profile. The tool will read the profile's existing mod priority, active plugins, and load order without asking you to create or edit a manifest:

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --mo2 "D:/Modding/MO2/Skyrim SE" --profile "My Actual Profile" --list-cells --output "D:/NavmeshWork/cells"
```

The importer writes `input-report.json` before listing or extraction. It records the profile snapshot hash, enabled-mod priority, active plugin winners, and loose-file winners selected from game `Data`, enabled mods, and `Overwrite`. The `--data` plus `--load-order` route remains an internal developer/test interface only.

If an existing MO2 profile's configured mod storage was moved, use the read-only `--mods-dir` recovery override. It changes only where enabled mod folders are looked up; it never edits MO2 configuration or profile files:

```powershell
./build/windows/x64/releasedbg/navmesh-offline.exe --mo2 "C:/Users/baudo/AppData/Local/ModOrganizer/Mod Authoring" --profile "My Actual Profile" --mods-dir "D:/Dev/Skyrim Mods/Mods" --list-cells --output "D:/NavmeshWork/cells"
```

Split MO2 layouts are supported: when `ModOrganizer.ini` has a relative `base_directory`, profile/mod/overwrite paths resolve from that directory (for example, a sibling `MODS` tree), while `gamePath` resolves to the configured stock game directory.

The direct reader currently resolves its fixture-tested full/light identities and override chains, and reports missing masters/cycles/duplicates. Its deliberately narrow coverage is documented in [the parser evaluation](docs/parser-library-evaluation.md).

## Output

The CLI writes files into the target output directory:

- `report.json` — cell metadata, references, navmesh summaries, and validation findings.
- `navmesh.obj` — exported navmesh polygon geometry for inspection in Blender or MeshLab.
- `geometry.obj` — exported neutral mesh/geometry derived from the parsed records.
- `input-report.json` — MO2 profile snapshot and virtual-file winners, emitted first for every MO2 run.
- `load-order.json` — when using `--load-order`, every winning record with its plugin and ordered origin chain.

Every JSON export contains a versioned `metadata` block. OBJ and HTML exports have an adjacent `<export>.metadata.json` sidecar. The metadata identifies the tool version, input plugin, selected cell, coordinate convention, source coverage, and known limitations. Its schema is [docs/schemas/export-metadata.schema.json](docs/schemas/export-metadata.schema.json); coordinate details are in [docs/coordinate-system.md](docs/coordinate-system.md).

Record the command inputs using [docs/run-manifest.example.json](docs/run-manifest.example.json) before a benchmark. The current CLI does not consume this file; map its fields to the existing command-line flags so milestone 0 does not alter parser input behavior.

## Reproducible fixtures and optional game checks

The repository contains only synthetic, redistributable fixture builders; it does not include Skyrim assets. See [fixtures/README.md](fixtures/README.md) for the policy and [docs/benchmarks.json](docs/benchmarks.json) for the local benchmark reference manifest. To opt into the read-only local-game parser smoke test, configure `SKYRIM_DATA_DIR` as described in [tests/integration/README.md](tests/integration/README.md). Without it, the complete test suite still runs and skips that check.

## Current limitations

- This is not an SKSE runtime plugin and does not inspect a running game session.
- The direct parser is intentionally small and targets the common Bethesda plugin structure, not the entire plugin ecosystem.
- Compressed indexed records are zlib-decoded with declared-size and boundary checks while their original bytes remain retained. Malformed records and unknown NAVM versions are explicit diagnostics, never best-effort geometry.
- Geometry extraction supports loose NIF files and uses the `tools/BSAFileExtractor` submodule to cache requested BSA-backed assets on demand. Collision/Havok extraction is not complete.
- Install the BSA bridge dependencies with `python -m pip install -r tools/requirements.txt` before extracting archived assets.
- Archived assets are cached under `<output>/.bsa-cache`; delete that directory to rebuild the cache.
- Terrain extraction is not implemented beyond any simple geometry that is explicitly included in the parsed records.
- Some Skyrim record variants and non-standard modded data layouts may still be rejected or reported as unsupported.

The verified NVNM prefix, preservation policy, and known trailing-layout limits are documented in [docs/navm-format-study.md](docs/navm-format-study.md). This milestone remains read-only and does not serialize plugins.

## Next milestone

The next milestone after this POC is:

```text
geometry -> walkability analysis -> candidate navmesh
```

not runtime integration.

See [docs/architecture.md](docs/architecture.md) for the intended pipeline and separation between parsing, extraction, neutral model, analysis, and future NAVM serialization.
