# Combined scene inspection

Milestone 6 writes one binary glTF scene, `scene.glb`, for each analysis run.
It uses named objects and stable materials so it can be inspected without
manually joining the OBJ exports:

| Layer/object name | Color | Meaning |
| --- | --- | --- |
| `Existing NAVM` | cyan | decoded NAVM polygons |
| `Terrain` | brown-green | decoded `LAND` height surface |
| `Collision` | gray | packed Havok support geometry |
| `Render fallback` | purple | visual geometry: low-confidence support where collision was absent, plus display-only render meshes retained beside authoritative collision |
| `Diagnostic: supported` | green | existing classifier result |
| `Diagnostic: floating` | orange | existing classifier result |
| `Diagnostic: buried` | red | existing classifier result |
| `Diagnostic: ...` | gray | unsupported, too-steep, or unknown result |

The GLB contains the geometry and colors. Its adjacent
`scene.glb.provenance.json` contains the reproducibility metadata, named GLB
objects, and source record/model provenance. `analysis.json` links each NAVM
classification to its selected world-triangle index; `geometry.json` resolves
that index to the complete triangle provenance. This is intentionally a report
join, not a new repair classification.

Every requested layer is emitted as a top-level GLB group, even when it has no
child mesh. An empty `Terrain`, `Collision`, or `Existing NAVM` group therefore
means the selected input did not provide that source; consult the metadata
warnings and coverage report rather than treating it as fallback geometry.

When a model has supported collision and a render mesh, collision remains the
only geometry used for navmesh analysis. Its render mesh is additionally placed
in `Render fallback` for visual comparison in the 3D viewer; it is display-only
and cannot affect classifications.

`scene-report.html` is emitted alongside the scene. It groups classifications
by selected support source and all extracted sources by coverage status, then
shows the source-triangle index used for each sampled row. It is a compact
human-readable view of the same `analysis.json` → `geometry.json` join.

## Online 3D Viewer

Open [Online 3D Viewer](https://3dviewer.net/), then drag `scene.glb` onto the
page (or use its Open button). The viewer supports binary GLB and locally opened
models are processed in the browser according to its
[user manual](https://www.3dviewer.net/info/index.html). Expand **Meshes** to
select a named layer/object, use **Materials** to confirm the color legend, and
enable edges from the settings panel when comparing NAVM boundaries to support
geometry. Keep `scene.glb.provenance.json`, `analysis.json`, and
`geometry.json` beside the exported scene for provenance review; they are not
files the viewer needs to load.

For a large exterior scene, limit work before export:

```powershell
navmesh-offline.exe ... --neighboring-cell-radius 1 `
  --scene-bounds 40960 -20480 49152 -12288 `
  --geometry-layers navmesh,terrain,collision,diagnostics `
  --output-detail summary
```

`--neighboring-cell-radius` streams matching exterior cells around the selected
cell into the combined scene. `--scene-bounds minX minY maxX maxY` uses the
scene grid to cull geometry outside that world-space rectangle. `--geometry-layers`
accepts comma-separated `navmesh`, `terrain`, `collision`, `render`, and
`diagnostics`; `--output-detail summary` trims the scene provenance report to a
summary selection record (the existing JSON exports remain the full audit data).
