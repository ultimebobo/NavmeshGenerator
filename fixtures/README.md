# Legal fixture strategy

No Bethesda game assets, plugins, or extracted NIFs are committed here. Unit tests construct the small NIF triangle they require at runtime through Nifly, and future binary-plugin fixtures must be generated from documented synthetic record builders. Keep each builder and expected result in source control so its bytes are reviewable and redistributable.

`docs/benchmarks.json` names representative local-game cells only. It is a reference manifest, not test data; users configure their own legally installed Data directory. Golden output derived from game assets must remain local unless it has been independently reduced to lawful synthetic data.

Add fixture files only when their license, provenance, and regeneration path are documented in this directory.
