# Local-game integration checks

## Playable world without authored navigation

Create a project-authored exterior fixture with the standard-library Python tool:

```powershell
python tools/create_test_world.py --output ./output/test-world
```

The output contains `NavmeshTestWorld.esp`, a `fixture.json` manifest listing the
cells, teleport commands and shared borders, and a developer `load-order.txt`.
The ESP adds an independent worldspace with continuous textured LAND terrain,
flat ground and gentle slopes across adjacent cells in both horizontal directions.
Each terrain cell has a named editor ID and a base-game teleport marker above
the ground. A separate persistent world container has no terrain. There are no
NAVM or NAVI records. The plugin references installed `Skyrim.esm` records for
the texture, climate and markers; no game records or assets are copied.
Use `--help` for the current grid defaults, limits and `--cell-name` option.
The destination must be new; the tool refuses existing paths.

Install and enable the ESP through your mod manager. With the default cell name,
enter `coc myTestCell` in Skyrim's console. To create navigation, select the
fixture plugin in NavmeshGenerator's **Plugin** rebuild scope, generate a patch,
then install and enable that patch after the fixture. Alternatively, test
directly through the developer CLI:

```powershell
./build/windows/x64/release/NavmeshGenerator.exe --data "<installed Skyrim Data directory>" --load-order ./output/test-world/load-order.txt --rebuild-plugin NavmeshTestWorld.esp --terrain-only --generate-plugin --output ./output/test-world-navmesh
```

Enable both plugins in game and use the same teleport command. Bring a follower
and cross the shared borders listed in the manifest, including a shared corner,
to check pathing between generated cells. Compare with the fixture alone. The
terrain is finite; stay within its listed cells. Game pathfinding remains a
manual check; successful plugin serialization does not verify engine behavior.

The automated fixture regression checks binary group placement, terrain continuity,
teleport positions, input validation and overwrite refusal. Supplying the desktop
executable also checks generation and reciprocal links across every shared border,
using a synthetic master without launching the UI or game:

```powershell
python tools/test_create_test_world.py ./build/windows/x64/release/NavmeshGenerator.exe
```

## Optional installed-master parser check

Set `SKYRIM_DATA_DIR` to the directory containing a legally installed `Skyrim.esm`, then run:

```powershell
$env:SKYRIM_DATA_DIR = 'D:/Games/Skyrim Special Edition/Data'
xmake run navmesh-tests
```

The test reads only the configured `Skyrim.esm`, checks that the lightweight parser can enumerate at least one cell, and makes no writes to the game directory. When the variable is absent, or `Skyrim.esm` is unavailable, it reports a skip and still exits successfully.
