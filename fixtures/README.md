# Legal fixture strategy

No Bethesda game assets, plugins, or extracted NIFs are committed here. Unit tests construct the small NIF triangle they require at runtime through Nifly, and future binary-plugin fixtures must be generated from documented synthetic record builders. Keep each builder and expected result in source control so its bytes are reviewable and redistributable.

`docs/benchmarks.json` names representative local-game cells only. It is a reference manifest, not test data; users configure their own legally installed Data directory. Golden output derived from game assets must remain local unless it has been independently reduced to lawful synthetic data.

Add fixture files only when their license, provenance, and regeneration path are documented in this directory.

The Recast stair regression can use a local Riverwood03 geometry export without
committing Bethesda assets. Run a candidate export of that CELL with supported
collision, then invoke `navmesh-tests.exe --local-stair-obj <geometry.obj>` on
the resulting OBJ. The test reads all available `WalkwayStairs15` placements,
checks that `human@1.1.0` connects each one's landings at both isolated and
broad scene resolutions, and confirms that the legacy 18-unit profile splits
each one. The ordinary local test run uses
`output/riverwood03-recast-repro/geometry.obj` when present. CI can run the
same test when a lawfully
supplied local export is available; no extracted game geometry is stored in
source control.

`TestPackedCollisionPreferredOverRenderFixture` in `tests/core_tests.cpp` is the milestone-5 collision fixture. It generates a NIF containing a decorative render triangle at Z=0 and an attached `hkPackedNiTriStripsData` collision triangle at Z=7 (plus a rigid-body Z translation of 2). The expected extracted triangle is collision at Z=9, proving that render geometry does not win when supported collision exists.
