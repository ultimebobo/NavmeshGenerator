# Combined scene inspection

Milestone 6 writes one binary glTF scene, `scene.glb`, for each analysis run.
It uses named objects and stable materials so it can be inspected without
manually joining the OBJ exports:

The scene tree and flattened mesh list put navigation first: `Original NAVM
(current cell)`, `Neighboring NAVM`, `Candidate NAVM` when generated, `NAVM
links`, and `Doors`, followed by `Terrain`, `Collision`, and `Render fallback`.
The `navmesh` layer selection includes both original and neighboring authored
meshes and their connection bars. Each authored NAVM is split by face color
inside its owning group.

| Layer/object name | Color | Meaning |
| --- | --- | --- |
| `Existing NAVM ...: unclassified` | cyan | decoded authored polygons without analysis evidence |
| `Existing NAVM ...: supported` | green | supported polygons |
| `Existing NAVM ...: floating` | amber | floating polygons |
| `Existing NAVM ...: buried` | red | buried polygons |
| `Existing NAVM ...: too_steep` | yellow | polygons with overly steep support |
| `Existing NAVM ...: blocked` | magenta | blocked polygons |
| `Existing NAVM ...: out_of_coverage` | blue | polygons outside extracted support coverage |
| `Existing NAVM ...: ambiguous` | violet | polygons with conflicting support evidence |
| `Existing NAVM ...: unsupported` | dark gray | unsupported or unknown classification |
| `Existing NAVM ...: door_linked` / `Candidate NAVM: door_linked` | orange | triangles associated with an exit; this material takes priority over the analysis face color |
| `Candidate NAVM` | blue-green | generated navigation connected to an entrance or matched exterior portal |
| `Candidate NAVM: water` | blue | generated triangles classified as water |
| `Candidate NAVM: preferred_path` | yellow | generated triangles following authored preferred routes |
| `Candidate NAVM: water_preferred_path` | teal | generated triangles with both independent tags |
| `Authored link ...` / `Candidate link ...` | bright green | solid bars running along the recorded edge between connected triangles, lifted above the surface |
| `Door ...` | orange | enabled exit position, including physical doors and invisible cave entrances |
| `Terrain` | brown-green | decoded `LAND` height surface |
| `Collision` | gray | packed Havok support geometry |
| `Render fallback` | purple | visual geometry, including display-only meshes retained beside authoritative collision |

Here a **door** means an exit to another area, including another worldspace or
an interior. Scene markers use placed teleport destinations (`XTEL`) or authored
NAVM door associations; a door model alone does not identify an exit. Markers
appear during inspection as well as generation, and disabled or deleted
placements are excluded. The `diagnostics` layer selection controls these door
markers. Polygon classifications remain available as face colors and in reports;
the scene emits no classification centroid markers.

Authored connection bars come from consuming external NAVM edges, while
candidate bars use the generated border-link matches. Reciprocal authored
connections share a single bar. Bars follow the consuming triangle’s edge endpoints,
including its slope and authored border drift. A bar requires both linked triangles to
be displayed; unresolved destinations, invalid indices, and connections cut by
scene bounds are omitted. Nearby unlinked edges do not produce bars.
When a candidate is displayed, authored bars involving the working cell's
original NAVMs are suppressed in both directions. Green bars at its seams follow
the generated polygons and exact neighboring endpoints. Authored bars between
unchanged neighboring NAVMs remain visible; inspection without a candidate keeps
the authored connections.

The GLB contains geometry, groups, and colors. Its adjacent
`scene.glb.provenance.json` records the exported groups in display order, named
objects, source record/model provenance, and connection endpoint identities.
`analysis.json` links each NAVM classification to its selected world-triangle
index; `geometry.json` resolves that index to complete triangle provenance.
Door coloring does not change the analysis classification or repair evidence.
Generated water/preferred classifications take color priority over door-linked
faces; the door marker and candidate JSON door association remain available.

Every requested layer is emitted as a top-level GLB group, even when it has no
child mesh. An empty `Terrain`, `Collision`, or `Original NAVM (current cell)` group therefore
means the selected input did not provide that source; consult the metadata
warnings and coverage report rather than treating it as fallback geometry.

MO2 scene extraction uses the profile's winning loose meshes and enabled mod
archives. A CELL records the origin of each reference, but its NIF may be in
another enabled mod. For an exterior structure whose reference origin falls in
a neighboring CELL, use `--neighboring-cell-radius 1` in the CLI or set
**Neighboring cells** to `1` in the Windows UI (the UI default). Set it to `0`
for an exact single-CELL inspection without candidate generation.

Placed references whose winning plugin record is initially disabled or deleted
are omitted from collision and render geometry. They remain in `geometry.json`
coverage as `excluded`, with the reason and winning reference provenance. This
uses the winning record's flags, so a later plugin can enable or disable an
earlier placement without a height-based scene filter. An initially disabled
reference that a quest enables at runtime is outside the default scene.

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
NavmeshGenerator.exe ... --neighboring-cell-radius 1 `
  --scene-bounds 40960 -20480 49152 -12288 `
  --geometry-layers navmesh,terrain,collision,diagnostics `
  --output-detail summary
```

`--neighboring-cell-radius` selects the exterior scene neighborhood around the
target. Terrain and authored NAVM come only from those cells. Model-bound and
unknown-bound searches can examine distant geometry suppliers, but retain only
their support and display triangles whose world-space bounds intersect the
neighborhood; they do not add the supplier cells' terrain or NAVM. Persistent
references are assigned by physical position. Terrain-only extraction visits
only the neighborhood. Generation includes the adjacent geometry ring and
keeps the candidate clipped to the target CELL.
`--scene-bounds minX minY maxX maxY` uses the
scene grid to cull geometry outside that world-space rectangle. `--geometry-layers`
accepts comma-separated `navmesh`, `terrain`, `collision`, `render`, and
`diagnostics`; `--output-detail summary` trims the scene provenance report to a
summary selection record (the existing JSON exports remain the full audit data).
