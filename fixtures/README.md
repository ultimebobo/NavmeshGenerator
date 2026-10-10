# Legal fixture strategy

No Bethesda game assets, plugins, or extracted NIFs are committed here. Unit tests construct the small NIF triangle they require at runtime through Nifly, and future binary-plugin fixtures must be generated from documented synthetic record builders. Keep each builder and expected result in source control so its bytes are reviewable and redistributable.

`docs/benchmarks.json` names representative local-game cells only. It is a reference manifest, not test data; users configure their own legally installed Data directory. Golden output derived from game assets must remain local unless it has been independently reduced to lawful synthetic data.

Add fixture files only when their license, provenance, and regeneration path are documented in this directory.

## Triangle tagging

`tests/triangle_tagging_tests.cpp` constructs overlapping marked and unmarked
floors, stacked surfaces, contradictory markings and malformed authored faces.
`tools/test_batch_rebuild.py` creates exterior water and preferred-route fixtures,
checks water-height inheritance, and independently reads exported NVNM flags and
GLB groups. Cell and batch paths exercise enabled and disabled classification and
cache invalidation. No location names or game assets enter these fixtures.

## Disconnected floors

`tests/batch_generation_tests.cpp` constructs stacked floors and a roof spanning
neighboring CELLs. It checks complete-set island removal, stable source and door
joins, reciprocal portal remapping, empty replacements, separate selected areas,
worldspace ownership and upper landings reached by ramps. Vertex-only contact
does not retain an isolated surface. Recast generation exercises upper-platform
removal after fresh and cached generation with every partitioning strategy.
`tools/test_batch_rebuild.py` supplies a synthetic steep-sided LAND plateau,
checks its removal through the CLI and independently reads exported plugin portals.
These builders contain no game assets or location-specific rules.

## Stairs and overpasses

`tests/navigation_obstacle_tests.cpp` builds a synthetic collision scene from
closed boxes. It is project-authored, redistributable under the repository
license, and contains no game assets. The ordinary C++ suite checks tread and
landing coverage, shared-edge reachability, separate stacked levels, pier
obstructions, insufficient headroom, a corner descent with uneven risers leading
under a bridge, and oversized risers with each supported
region partitioning strategy. It uses the current navigation defaults. Additional
regressions check minimal straight-flight triangulation, obstacle-only solids
within climb reach, a connected route around their blocked footprint, and the
same solids remaining walkable when their obstacle tag is absent. Extraction
checks rock-directory classification without treating stone stairs as rocks.

Build the assertion-enabled test executable as described in the
[development guide](../docs/development.md), then export the visual fixture:

```powershell
./build/windows/x64/debug/navmesh-tests.exe --export-navigation-fixture ./output/navigation-obstacles
```

Open `output/navigation-obstacles/scene.glb` in a compatible viewer. Named
collision objects identify narrow stairs, diagonal stairs, switchback stairs,
oversized steps, a bridge over a road, a low overpass, and a bridge reached by
stairs, plus a corner descent beneath a bridge. Toggle `Collision`,
`Candidate NAVM`, and `Doors` to compare the support with the generated surface.
`candidate-navm.json` records the settings,
connectivity and source evidence; `scene.glb.provenance.json` records object
names. The output is regenerated from the same builder used by the automated
checks, so it also serves as a visual regression scene.

Expected: traversable stairs cover every central tread and connect both
landings; bridge decks follow their own level; an open underpass remains
traversable around its pier; the low underpass has no path beneath the deck;
oversized steps do not connect to the lower entrance. A bridge with stair
access connects its deck to that entrance while the road beneath stays on a
separate component. Radius erosion intentionally leaves clearance at exposed
edges. Run only these checks with `navmesh-tests.exe --obstacles-only`.

The corner descent represents a street at an exterior boundary, taller and
slightly uneven steps, a lower landing and the road beneath a crossing deck.
The tests also generate it with CELL bounds and no doors: the street's actual
border edges must retain the entire lower route. A deliberately insufficient
climb reproduces removal of the disconnected descent. This fixture uses only
project-authored solids and has no location-specific generation rule.

The Recast stair regression can use a local Riverwood03 geometry export without
committing Bethesda assets. Run a candidate export of that CELL with supported
collision, then invoke `navmesh-tests.exe --local-stair-obj <geometry.obj>` on
the resulting OBJ. The test reads all available `WalkwayStairs15` placements,
checks that the fixed human settings connect each one's landings at
both isolated and broad scene resolutions. The ordinary local test run uses
`output/riverwood03-recast-repro/geometry.obj` when present. CI can run the
same test when a lawfully
supplied local export is available; no extracted game geometry is stored in
source control.

`TestPackedCollisionPreferredOverRenderFixture` in `tests/core_tests.cpp` is the milestone-5 collision fixture. It generates a NIF containing a decorative render triangle at Z=0 and an attached `hkPackedNiTriStripsData` collision triangle at Z=7 (plus a rigid-body Z translation of 2). The expected extracted triangle is collision at Z=9, proving that render geometry does not win when supported collision exists.
