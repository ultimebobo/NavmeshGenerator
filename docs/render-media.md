# Rendering scene images

`tools/render_media.py` renders NavmeshGenerator GLB exports with shared depth
testing for navigation and supporting geometry. The same command accepts
application exports and synthetic test scenes. Cameras, lighting, colors and
captions affect presentation; exported triangle positions stay unchanged.

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

The output contains `scene.png` and `render-manifest.json`. The manifest records
source paths, hashes and image filenames. Candidate navigation is teal and
supporting collision is gray. Geometry can occlude navigation behind it;
coplanar navigation remains visible without shifting its vertices.

Optional presentation controls apply to every scene:

- `--title` sets the image caption.
- `--navigation "Original NAVM (current cell)"` selects authored navigation.
- `--bounds <xmin> <ymin> <xmax> <ymax>` selects a world-coordinate crop.
- `--azimuth` and `--elevation` set camera angles in degrees.

Fixture names, dimensions, probe expectations and measured outcomes belong in
builders and generated reports, rather than renderer presets or separate guides.
The reader accepts the application's untransformed world-coordinate triangle
exports; it is not a general glTF importer. Images support inspection and do not
verify NPC behavior in game.

## Authored/generated comparison

Use `--comparison` for any region. Select cells from the same worldspace, generate
navigation with `--make-scene`, and pass the resulting combined scene to the
renderer:

```powershell
python tools/render_media.py --comparison <scene.glb> --output ./output/images `
  --title "<region>: authored vs generated" --attribution "<winning game plugins and mod edits>" `
  --filename <region>-navmesh-comparison.png --azimuth <degrees> --elevation <degrees>
```

Both panels use identical framing, lighting and supporting geometry. Captions
count the triangle primitives in `Original NAVM (current cell)` and `Candidate
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

## Inspect generation and export failures

The showcase runs retain their commands, profile snapshot, logs, candidate JSON,
emitted plugins, scene provenance and render manifests beneath
`output/nexus-media/`. `validation.json` joins image counts to candidate and
plugin counts and records reciprocal generated-portal checks. The generated
`validation-report.md` contains the measured results and reproduction evidence.
Reviewed region comparisons are available in `docs/images/`.

These checks distinguish mesh generation from tagging and reporting. Extreme
positive CELL `XCLW` heights can be accepted as finite water planes, marking land
triangles as water in candidate JSON and emitted NAVM. Presentation colors do
not show those flags; disclose the tagging issue in affected comparison captions.
The intended interpretation of unusual water values requires independent format
review before changing parsing policy.

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

Run the synthetic occlusion regressions:

```powershell
python tests/test_media_renderer.py
```
