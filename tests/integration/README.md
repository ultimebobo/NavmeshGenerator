# Optional local-game integration test

Set `SKYRIM_DATA_DIR` to the directory containing a legally installed `Skyrim.esm`, then run:

```powershell
$env:SKYRIM_DATA_DIR = 'D:/Games/Skyrim Special Edition/Data'
xmake run navmesh-tests
```

The test reads only the configured `Skyrim.esm`, checks that the lightweight parser can enumerate at least one cell, and makes no writes to the game directory. When the variable is absent, or `Skyrim.esm` is unavailable, it reports a skip and still exits successfully.
