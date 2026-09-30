# Combined scene inspection

Milestone 6 writes one binary glTF scene, `scene.glb`, for each analysis run.
It uses named objects and stable materials so it can be inspected without
manually joining the OBJ exports:

| Layer/object name | Color | Meaning |
| --- | --- | --- |
| `Existing NAVM ...: unclassified` | cyan | decoded NAVM polygons without an analysis result, including neighboring cells |
| `Terrain` | brown-green | decoded `LAND` height surface |
| `Collision` | gray | packed Havok support geometry |
| `Render fallback` | purple | visual geometry: low-confidence support where collision was absent, plus display-only render meshes retained beside authoritative collision |
| `Existing NAVM ...: supported` / `Diagnostic: supported` | green | supported polygons and their centroid markers |
| `Existing NAVM ...: floating` / `Diagnostic: floating` | orange | floating polygons and markers |
| `Existing NAVM ...: buried` / `Diagnostic: buried` | red | buried polygons and markers |
| `Existing NAVM ...: too_steep` / `Diagnostic: too_steep` | yellow | polygons with overly steep support |
| `Existing NAVM ...: blocked` / `Diagnostic: blocked` | magenta | blocked polygons |
| `Existing NAVM ...: out_of_coverage` / `Diagnostic: out_of_coverage` | blue | polygons outside extracted support coverage |
| `Existing NAVM ...: ambiguous` / `Diagnostic: ambiguous` | violet | polygons with conflicting support evidence |
| `Existing NAVM ...: unsupported` / `Diagnostic: unsupported` | dark gray | unsupported or unknown classification |
| `Candidate NAVM` | blue-green | generated navigation connected to an entrance or exterior cell border |
| `Entrance ...` | orange | enabled placed DOOR position; scene provenance records its reference ID and matched candidate region |

Each analyzed NAVM is split into named objects by polygon classification under
the `Existing NAVM` layer. The GLB contains the geometry and colors. Its adjacent
`scene.glb.provenance.json` contains the reproducibility metadata, named GLB
objects, and source record/model provenance. `analysis.json` links each NAVM
classification to its selected world-triangle index; `geometry.json` resolves
that index to the complete triangle provenance. This is intentionally a report
join, not a new repair classification.

Every requested layer is emitted as a top-level GLB group, even when it has no
child mesh. An empty `Terrain`, `Collision`, or `Existing NAVM` group therefore
means the selected input did not provide that source; consult the metadata
warnings and coverage report rather than treating it as fallback geometry.

MO2 scene extraction uses the profile's winning loose meshes and enabled mod
archives. A CELL records the origin of each reference, but its NIF may be in
another enabled mod. For an exterior structure whose reference origin falls in
a neighboring CELL, use `--neighboring-cell-radius 1` in the CLI or set
**Neighboring cells** to `1` in the Windows UI (the UI default). Set it to `0`
for an exact single-CELL scene.

Placed references whose winning plugin record is initially disabled or deleted
are omitted from collision and render geometry. They remain in `geometry.json`
coverage as `excluded`, with the reason and winning reference provenance. This
uses the winning record's flags, so a later plugin can enable or disable an
earlier placement without a height-based scene filter. An initially disabled
reference that a quest enables at runtime is outside the default offline scene.

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
