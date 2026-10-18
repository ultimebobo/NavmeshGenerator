# Rendering scene images

`tools/render_media.py` renders NavmeshGenerator GLB exports with shared depth
testing for navigation and supporting geometry. The same command accepts
application exports and synthetic test scenes. Cameras, lighting, colors and
captions affect presentation. Interior cutaways clip supporting surfaces for
display; navigation vertices and the source exports stay unchanged.

## Requirements

The renderer requires Python, NumPy and Pillow:

```powershell
python -m pip install numpy Pillow
```

These packages are not requirements for running NavmeshGenerator itself. Build
instructions are in the [development guide](development.md).

## Export and render

Create a scene using the application's [normal export workflow](command-line.md)
or the shared [fixture workflow](../fixtures/README.md). Render any exported GLB:

```powershell
python tools/render_media.py --scene <scene.glb> --output ./output/images
```

The output contains `scene.png` and `render-manifest.json`. Images show the number
of selected cells from the adjacent metadata, deduplicated by CELL Form ID.
The manifest records source paths, hashes, cell counts and image filenames. Candidate navigation is teal,
water-tagged navigation is blue even when also preferred-path tagged, and supporting collision is gray. Geometry can occlude navigation behind it;
coplanar navigation remains visible without shifting its vertices.

Optional presentation controls apply to every scene:

- `--title` sets the image caption.
- `--navigation "Original NAVM (current cell)"` selects authored navigation.
- `--bounds <xmin> <ymin> <xmax> <ymax>` selects a world-coordinate crop.
- `--azimuth` and `--elevation` set camera angles in degrees.
- `--view auto|interior|exterior` selects automatic cell detection or an explicit
  presentation mode.
- `--cut-height <Z>` sets the interior supporting-geometry cut in Skyrim world Z
  units. It implies interior presentation in automatic mode.
- `--cell-count <count>` supplies a verified displayed-cell count for exports
  without metadata or for a crop. The value must be positive.

Cell counts describe the selected CELLs, excluding neighboring geometry suppliers.
Without metadata or an override, captions state that the cell count is unavailable.
Metadata counts on cropped images are labeled as the selection before cropping;
the sidecar cannot determine which CELLs remain in an arbitrary crop. Explicit
camera modes still read metadata for counts and record its hash in the manifest.

Fixture names, dimensions, probe expectations and measured outcomes belong in
builders and generated reports, rather than renderer presets or separate guides.
The reader accepts the application's untransformed world-coordinate triangle
exports; it is not a general glTF importer. Images support inspection and do not
verify NPC behavior in game.

## Interior cutaways

The default automatic view reads `<scene.glb>.metadata.json`. A single selected
interior or a combined selection containing only interiors uses a slightly
inclined cutaway so floor heights and stairs remain visible. Missing metadata,
exterior selections, and mixed interior/exterior selections use the exterior
presentation. Keep the metadata beside the GLB when
moving an application export. Use `--view interior` for an interior scene without
metadata, or `--view exterior` to display its complete supporting geometry.

The automatic cut sits above the highest authored or generated navigation vertex
in the selected world-XY bounds, with a presentation margin defined by the
renderer. Terrain and collision above that plane are omitted. Crossing triangles
are clipped at the plane, retaining their lower portions and surface normals.
Camera framing uses the retained support and both navigation layers. Navigation
is never clipped, and retained geometry still shares its depth buffer with
navigation. Furniture, stair treads and floors below the cut can occlude routes.

Both comparison panels use the same cut and camera, even when one navigation
layer is empty. An interior selection without navigation requires `--cut-height`
to identify the plane. Use a height override to inspect a lower floor in a
building with overlapping levels; the automatic cut does not remove ceilings or
upper floors below its plane. Explicit `--elevation` also applies to interiors
to adjust the incline or select a top-down view. Cut heights, bounds and camera
angles must be finite, and `--cut-height` cannot be combined with `--view exterior`.

Render an interior using the same export and rendering commands as other cells:

```powershell
python tools/render_media.py --scene <interior-scene.glb> --output ./output/images
python tools/render_media.py --comparison <interior-scene.glb> --output ./output/comparison `
  --attribution "<winning game plugins and mod edits>" --view interior
```

Images label the cutaway presentation. `render-manifest.json` records each image's
effective view, cut height and camera, plus the path and hash of metadata used for
automatic detection. Comparison counts still describe the selected exported
navigation, including any faces hidden by retained geometry.

## Authored/generated comparison

Use `--comparison` for any region. Select cells from the same worldspace, generate
navigation with `--make-scene`, and pass the resulting combined scene to the
renderer:

```powershell
python tools/render_media.py --comparison <scene.glb> --output ./output/images `
  --title "<region>: authored vs generated" --attribution "<winning game plugins and mod edits>" `
  --filename <region>-navmesh-comparison.png --azimuth <degrees> --elevation <degrees>
```

Both panels use identical framing, lighting and retained supporting geometry.
Captions count the triangle primitives in `Original NAVM (current cell)` and `Candidate
NAVM`, across all material groups. Neighboring navigation and connection bars
are excluded; occluded navigation is included. With `--bounds`, counts and
display use the same world-XY centroid selection. Counts describe the exported
selection, not a whole hold or worldspace, and do not measure navigation quality.

`--attribution` is required for generic comparisons. Resolve authored NAVM IDs
and selected CELLs through `load-order.json`, and inspect scene provenance for
supporting geometry. Name third-party plugins touching the scene in the image;
an active plugin alone does not establish that it edited those cells. Consult
the input report for asset replacements. Use `--note` to disclose skipped cells,
generation failures or other case-specific qualifications. Review run logs and
candidate topology before presenting an image as a successful generation.

`render-manifest.json` records source hashes, comparison counts and presentation
settings alongside the images. Keep exported game assets and full local run
evidence in the ignored output folder; reviewed stills may be copied to
`docs/images/`.

For the Riverwood comparison used in the README, export the village's official
game-plugin scene as described in the command-line reference, then run:

```powershell
python tools/render_media.py --riverwood <scene.glb> --output ./output/images
```

This produces `riverwood-navmesh-comparison.png` using matching cameras and depth
testing for both panels. After reviewing it, copy the image to
`docs/images/riverwood-navmesh-comparison.png`. Game-derived scenes and assets
stay in the ignored output directory.

## Remake the program showcase

`tools/remake_showcase.py` generates scene previews and authored/generated images
for Riverwood, Whiterun, Whiterun Dragonsreach, Riften, Winterhold, Dawnstar,
Dragon Bridge, Falkreath, Morthal and Rorikstead. Its tracked recipes in
`tools/showcase-scenes.json` define the CELL selections, crops and cameras.
Dragonsreach uses an interior cutaway. List the current selections with:

```powershell
python tools/remake_showcase.py --list
```

Build the desktop executable, install the renderer requirements, and remake the
images from an existing MO2 profile:

```powershell
python tools/remake_showcase.py --mo2 "<MO2 instance>" --profile "<existing profile>"
```

For a deployed Vortex setup, replace `--mo2` and `--profile` with
`--vortex "<Skyrim installation>"`. Use `--vortex-plugins` when the active
plugin list lives in a custom directory. `--exe` chooses a different desktop
build and `--asset-cache` chooses the generator's shared cache.

The selected source supplies its complete active plugin order and winning model
assets, including mod overrides. The helper records that live source in
`showcase-manifest.json`. Regenerate scenes and images from its current inputs
on subsequent runs with:

```powershell
python tools/remake_showcase.py
python tools/remake_showcase.py --only winterhold
```

Change `--mo2` and `--profile`, or `--vortex`, to select another source. Generation
rereads the selected profile or deployment; the stored source is a location to
read, rather than a filtered copy of its plugin list. Captions automatically name
winning scene record plugins and referenced loose model replacements. The helper
audits selected CELLs, authored NAVMs and supporting record identities through
the export provenance, resolved load order and input report. An unrelated active
plugin alone is not attribution evidence. Each saved scene retains those winners,
the input snapshot identity and report hashes. `--attribution` supplies an optional
caption override without replacing the audit evidence.

The default output is `output/showcase/`. Open `gallery.html` to review the images.
`--output` selects another output root. Each generation uses a fresh folder under
`runs/`, retains exact CLI commands and logs, and disables experimental triangle
tagging. It generates candidate previews through the normal Cell workflow and
does not need plugin writing. Skipped generation, invalid candidate topology and
enabled tagging stop image creation for that scene. Review source coverage and
visible routes before using a successful render as showcase material.

Repeat rendering from the saved GLBs without regenerating navigation:

```powershell
python tools/remake_showcase.py --render-only
python tools/remake_showcase.py --render-only --only riverwood whiterundragonsreach
```

`--only` also limits generation runs. `--render-only` uses the scene paths and
attribution in `showcase-manifest.json`; use the same `--output` as the generation
run. Presentation changes in the recipes apply when rerendering, while changed
CELL selections require fresh generation. `--presets` accepts a separate recipe
file for another showcase. The helper checks the scene's metadata selection
against the recipe before supplying its displayed cell count. Exterior recipes
frame the complete selected region; their counts exclude neighboring suppliers.
An optional `note` in a recipe or saved scene entry supplies case-specific
qualifications for the image caption, such as a visible generation artifact.
`--render-only` keeps the exported input snapshot, including its mod overrides;
it does not read changed profile/deployment inputs. Mod-manager input switches
cannot be combined with it. Use generation to incorporate updated mods.

Every image directory retains a render manifest with source hashes, cell and
triangle counts, crop and camera settings. Reviewed PNGs can be copied to
`docs/images/`; keep the GLBs, game assets and local run evidence under `output/`.

## Inspect generation and export failures

Earlier local showcase audits retained their commands, profile snapshot, logs, candidate JSON,
emitted plugins, scene provenance and render manifests beneath
`output/nexus-media/`. Their `validation.json` joins image counts to candidate and
plugin counts and records reciprocal generated-portal checks. The generated
`validation-report.md` contains the measured results and reproduction evidence.
Reviewed region comparisons are available in `docs/images/`.

These checks distinguish mesh generation from tagging and reporting. Tagged
diagnostic renders preserve blue water classifications from the exported scene;
classification follows supported CELL/WRLD water data as described in the
[triangle-tagging contract](command-line.md#generated-triangle-tagging).

For geometry showcases, disable experimental water and preferred-path tagging
with `--no-triangle-tagging` on the generator command. Verify `triangle_tagging`
and polygon flags in every candidate export, and state that tagging is disabled
in the image caption. Tagged diagnostic probes should remain separate from the
showcase run evidence.

Catalog exports can also contain invalid JSON when localized `FULL` bytes are
treated as names and copied without complete escaping. Use verified editor IDs
and Form IDs for reproduction and preserve the original malformed export.
Combined-scene metadata currently leaves input-plugin and source-coverage
aggregation incomplete; use the input report, object provenance and per-cell
candidate evidence to audit those sources.

Passing topology, reciprocal-portal and plugin read-back checks does not prove
route completeness, correct triangle tags or NPC movement. Review visible gaps
and unsupported model coverage as separate findings.

## Verify rendering

Run the synthetic occlusion and interior cutaway regressions:

```powershell
python tests/test_media_renderer.py
python tests/test_showcase.py
```
