# Exterior terrain benchmark

The `exterior` entry in `benchmarks.json` is an opt-in local-game benchmark; no Skyrim assets are committed. Select a known exterior cell with existing NAVM and run the normal resolved-load-order command with `--terrain-only`.

Confirm that the exported bounds start at `(cellX * 4096, cellY * 4096)` and end 4096 world units later on each horizontal axis. Check several nearby NAVM vertices against the terrain surface; use a 0.1-unit visual/alignment tolerance. Inspect `geometry.json` to ensure each terrain triangle records `reference.recordType: "LAND"` and a `terrain` object containing the cell, LAND FormID, and VHGT sample.

The synthetic `TestExteriorLandTerrain` test provides deterministic coverage for height-delta reconstruction, coordinates, 2,048-triangle grid topology, provenance, and the required missing-LAND behavior. A missing record is coverage failure, never a flat ground surface.
