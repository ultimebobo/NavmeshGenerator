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

For the Riverwood comparison used in the README, export the village's official
game-plugin scene as described in the command-line reference, then run:

```powershell
python tools/render_media.py --riverwood <scene.glb> --output ./output/images
```

This produces `riverwood-navmesh-comparison.png` using matching cameras and depth
testing for both panels. After reviewing it, copy the image to
`docs/images/riverwood-navmesh-comparison.png`. Game-derived scenes and assets
stay in the ignored output directory.

## Verify rendering

Run the synthetic occlusion regressions:

```powershell
python tests/test_media_renderer.py
```
