# Rendering scene images

`tools/render_media.py` renders still images from NavmeshGenerator's GLB exports.
It provides presets for the Riverwood authored/generated comparison and the
synthetic stairs-and-bridges fixture. Geometry is read directly from exported
triangle positions and indices; the renderer adds cameras, lighting, display
colors and captions without changing the source geometry.

## Requirements

Build the desktop application and assertion-enabled test executable using the
[development guide](development.md). The optional renderer requires Python,
NumPy and Pillow:

```powershell
python -m pip install numpy Pillow
```

These packages are not requirements for running NavmeshGenerator itself.

## Export the scenes

For a default-versus-generated comparison, create a local text file listing your
installed official game plugins in load order, one path per line. Exclude mod
plugins and generated patches. Export the selected Riverwood village cells:

```powershell
./build/windows/x64/releasedbg/NavmeshGenerator.exe `
  --data "<Skyrim Data directory>" `
  --load-order "<official plugins list>" `
  --cells "Riverwood03,Riverwood02,Riverwood01,Riverwood05,Riverwood04,Riverwood" `
  --generate-candidate --make-scene --batch-output compact `
  --geometry-layers navmesh,terrain,collision --output-detail summary `
  --asset-cache ./output/render-media/asset-cache `
  --output ./output/render-media/riverwood
```

This uses the current generation defaults and reconciles the selected cells
together. Review the resulting `batch-report.json` for failures and diagnostics.
Keep game-derived GLB exports and extracted assets in the ignored output directory.

Export the project-authored fixture with the assertion-enabled test executable:

```powershell
./build/windows/x64/debug/navmesh-tests.exe `
  --export-navigation-fixture ./output/render-media/navigation-obstacles
```

The builder in `tests/navigation_obstacle_tests.cpp` checks coverage and
connectivity before writing the fixture. It contains synthetic solids and no game
assets. See the [fixture guide](../fixtures/README.md) for the expected routes.

## Render the images

Run the renderer with both exported scenes and an output directory:

```powershell
python tools/render_media.py `
  --riverwood ./output/render-media/riverwood/scene.glb `
  --fixture ./output/render-media/navigation-obstacles/scene.glb `
  --output ./output/render-media/images
```

The output contains:

- `riverwood-navmesh-comparison.png`: authored navigation in amber and generated
  navigation in teal, with matching cameras and gray terrain/collision.
- `stairs-and-bridges.png`: the complete synthetic fixture.
- `bridge-and-underpass.png`: stair access to a bridge above a separate road.
- `corner-descent.png`: uneven stairs leading beneath a crossing deck.
- `render-manifest.json`: source GLB paths, hashes and image filenames.

The comparison unifies navigation tag and analysis colors to show geometry.
Neighboring NAVMs and connection bars are hidden. All images share depth testing
between navigation, terrain and collision: decks, walls and individual stair
treads occlude navigation behind them. A small numerical tolerance displays
coplanar faces without shifting their positions. These are diagnostic renders;
they do not verify NPC behavior in game.

The reader accepts the application's untransformed world-coordinate triangle
exports. It is not a general glTF importer. Camera and crop presets belong to
this rendering script, not to the generation algorithm.

To refresh the README comparison after reviewing the render:

```powershell
Copy-Item ./output/render-media/images/riverwood-navmesh-comparison.png ./docs/images/riverwood-navmesh-comparison.png
```

## Verify rendering

Run the synthetic occlusion regressions:

```powershell
python tests/test_media_renderer.py
```

The checks verify that collision hides lower navigation in overhead and oblique
views while coplanar and exposed navigation remain visible.
