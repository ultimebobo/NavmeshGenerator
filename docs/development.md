# Building and developing NavmeshGenerator

For desktop usage, see [the README](../README.md). Commands, exports, and detailed
coverage limits are in the [command-line reference](command-line.md).

## Build the desktop executable

Install xmake and a Visual Studio C++ toolchain. From the repository root,
initialize the dependency submodules and build the application:

```powershell
git submodule update --init --recursive
xmake f -m releasedbg
xmake build NavmeshGenerator
```

The Windows x64 build produces
`build/windows/x64/releasedbg/NavmeshGenerator.exe`. Launch it without arguments
for the desktop workspace, or pass command-line options for scripted operations.
The application uses Nifly for model reading, Recast Navigation for generation,
and the bundled Dear ImGui core with matching Windows/DirectX backends for its UI.
xmake resolves the compression and model-library packages declared by the build.

## Archived asset setup

The application reads requested BSA models with its in-process C++ reader and
reuses a shared disk cache. xmake resolves the native zlib and LZ4 dependencies;
no Python interpreter or extractor script is needed at runtime. See
[performance and cache settings](performance-improvements.md) and the
[BSA performance investigation](bsa-performance.md).

Python and the packages in `tools/requirements.txt` are needed only for the
Python reference-reader regressions and comparative benchmarks:

```powershell
python -m pip install -r tools/requirements.txt
```

## Automated checks

The C++ regression suite uses assertions for checks and fixture setup. Run it in
debug mode so assertions are enabled:

```powershell
xmake f -m debug
xmake build navmesh-tests
xmake run navmesh-tests
```

Run the CLI regressions against the desktop binary without opening its UI:

```powershell
python tools/test_batch_rebuild.py ./build/windows/x64/releasedbg/NavmeshGenerator.exe
python tools/test_bsa_index.py
xmake build navmesh-bsa-probe
python tools/test_native_bsa.py ./build/windows/x64/releasedbg/navmesh-bsa-probe.exe
```

Fixtures are synthetic and redistributable; no Skyrim assets belong in the
repository. See the [fixture policy](../fixtures/README.md) and
[optional local-game checks](../tests/integration/README.md). Restore the desired
build mode with `xmake f -m releasedbg` after running debug tests.

Synthetic scene fixtures share the selection, testing and export commands in the
[fixture workflow](../fixtures/README.md). Discover available fixtures through
the test executable; their builders and generated reports define their details.

## Source documentation and formatting

Follow [AGENTS.md](../AGENTS.md) and the repository `.clang-format`. Restrict
formatting to changed project-owned source and tests, preserving include order.
With clang-format available on PATH, check project source from PowerShell:

```powershell
clang-format --dry-run --Werror (rg --files src -g '*.cpp' -g '*.h')
```

The [architecture guide](architecture.md) maps module responsibilities. The
[API guide](api.md) is the entry point for the generated C++ reference. With
Doxygen installed, generate it from the repository root:

```powershell
doxygen Doxyfile
```

Open `build/doxygen/html/index.html`. Generated documentation stays beneath
`build/` and excludes vendored dependencies.
