# Local-game integration checks

## Playable world without authored navigation

Create a project-authored exterior fixture with the standard-library Python tool:

```powershell
python tools/create_test_world.py --output ./output/test-world
```

The output contains `NavmeshTestWorld.esp`, a `fixture.json` manifest listing the
cells, teleport commands, shared borders and door route, and a developer `load-order.txt`.
The ESP adds an independent worldspace with continuous textured LAND terrain,
flat ground and gentle slopes across adjacent cells in both horizontal directions.
Each terrain cell has a named editor ID and a base-game teleport marker above
the ground. A large vanilla wooden load door in a freestanding stone surround
connects point A near the origin spawn to point B in an isolated terrain worldspace,
with a return door. The door is the
only route between these worldspaces. Each world's persistent container has no
terrain, uses exterior placeholder coordinates and holds its persistent door reference.
Each surround is a static reference in its terrain cell. There are no
NAVM or NAVI records. The plugin references installed `Skyrim.esm` records for
the texture, climate, markers, door and surround models; no game records or assets are copied.
The manifest's `door_route` identifies each endpoint's reference, cell and worldspace,
door position and rotation, and local arrival transform. Positions use Skyrim world
coordinates and rotations use radians; each door's XTEL targets its partner's arrival.
Use `--help` for the current grid defaults, limits and `--cell-name` option. The grid
size applies to the main world; the isolated destination is added separately.
The destination must be new; the tool refuses existing paths.

Install and enable the ESP through your mod manager and enter the teleport command
printed by the tool in Skyrim's console. To create navigation, select the
fixture plugin in NavmeshGenerator's **Plugin** rebuild scope, generate a patch,
then install and enable that patch after the fixture. Alternatively, test
directly through the developer CLI:

```powershell
./build/windows/x64/release/NavmeshGenerator.exe --data "<installed Skyrim Data directory>" --load-order ./output/test-world/load-order.txt --rebuild-plugin NavmeshTestWorld.esp --generate-plugin --output ./output/test-world-navmesh
```

Enable both plugins in game and use the same teleport command. Start with a recruited
follower beside you at point A. Activate the wooden door in the stone arch ahead of
the spawn, wait at point B, and confirm the follower arrives and resumes walking
after you. Turn around and use
the return door to repeat the check toward A. Do not move the follower with console
commands or follower-mod catch-up teleports during this check. Compare with the fixture
alone in a separate test session. Also cross the shared borders listed in the manifest,
including a shared corner, to check pathing between generated cells. The
terrain is finite; stay within its listed cells. Game pathfinding remains a
manual check; successful plugin serialization does not verify engine behavior.

The automated fixture regression checks binary group placement, exterior container
coordinates, static surround placement, terrain continuity, teleport positions,
reciprocal door destinations, input validation and overwrite refusal.
Supplying the desktop executable also checks generation and reciprocal links across every
shared border, matching both door endpoints to generated triangles and independently
decoding the emitted NAVM door tables and reciprocal REFR XNDP associations. It uses a
synthetic master without launching the UI or game:

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
