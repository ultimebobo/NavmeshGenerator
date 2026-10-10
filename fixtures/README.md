# Fixture policy and workflow

Fixtures are interchangeable test inputs. Keep their geometry, parameters and
expected behavior in their builders under `tests/`; generated scenes and reports
carry the inspection evidence. Adding a fixture does not require a separate
guide or changes to product documentation.

Commit only project-authored, redistributable inputs with a reviewable generation
path and documented provenance. No Bethesda assets, extracted NIFs or plugins
belong in source control. Game-derived exports stay local under `output/`.
`docs/benchmarks.json` is a reference manifest, not fixture data.

Build the assertion-enabled test executable using the
[development guide](../docs/development.md). Discover and select a scene fixture:

```powershell
./build/windows/x64/debug/navmesh-tests.exe --list-fixtures
./build/windows/x64/debug/navmesh-tests.exe --fixture <name>
./build/windows/x64/debug/navmesh-tests.exe --export-fixture <name> ./output/fixture
```

Open an exported `scene.glb` in a compatible viewer, or render it using the shared
[scene renderer](../docs/render-media.md). Collision, candidate navigation and
entrance markers use the same layers across fixtures. Candidate JSON records
generation settings and evidence; provenance sidecars identify source objects.
Additional reports are emitted alongside the scene when the builder supplies
coverage probes or parameter comparisons. Inspect diagnostics and connectivity
as well as topology; completing an export does not imply every route succeeded.

Register new scene builders in the test executable's fixture catalog with their
test and export functions. Use the same selector and GLB rendering workflow;
fixture-specific details stay with the builder and its output.
