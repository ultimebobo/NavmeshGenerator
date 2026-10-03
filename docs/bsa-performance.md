# BSA performance investigation

The application uses an in-process C++ extractor with run-scoped directory
indexes and the shared extracted-model disk cache. This removes helper-process
startup and repeated JSON index parsing from incremental geometry extraction.
Both CLI and desktop operations reach this implementation through `app::Run`;
there is no operator setting to select a backend.

## Findings and scope

The Python reference helper already seeks to requested winning entry ranges.
Whole-archive extraction and loading unrelated payloads therefore do not explain
its remaining overhead. Repeated small requests incur interpreter startup,
directory-index loading, manifest publication, and negative-cache reads. Native
extraction retains its directory indexes and missing-model set across those
requests. It also handles changed-archive impact selection directly.

[The measurement snapshot](bsa-performance-measurements.json) compares repeated
individual requests, one bulk request, metadata indexing, payload decompression,
decoded-file writes, and cached requests against installed desktop archives.
The incremental gain is much larger than the bulk gain: it depends on how often
new models appear across cells. All sampled payload bytes and the complete set
of winning model names are checked against the Python reader.

These are archive-stage measurements, not whole-plugin rebuild timings. NIF
decoding, placed geometry, resolution, Recast, and plugin writing are excluded.
The operating-system file cache is not flushed. Python timings include helper
startup; native timings exclude the development probe's startup, matching the
production in-process path. The JSON records samples, medians, counters, and
comparison ratios rather than embedding configuration values here.

## Direct archive loading versus disk extraction

Nifly accepts an input stream, so decoding an archive payload directly is
technically possible. It would avoid publishing and reopening extracted NIFs,
but must still seek and decompress compressed entries. The benchmark measures
decoded-file writes separately to expose this remaining cost.

The implemented path retains selective disk extraction because it preserves
cross-run reuse, existing cache-budget behavior, and the geometry reader's path
identity. A wholly memory-only path would decompress again after process exit
or decoded-geometry eviction. Keeping the shared cache while introducing a
direct-stream first-load path is a possible additional optimization, with
separate accounting and cache-lifetime work. It is not included in these
measured native-extractor gains.

## Compatibility and failure policy

The native reader supports named desktop Skyrim BSA revisions, filename
prefixes, archive-wide compression with per-entry toggles, zlib streams, and
LZ4 frames. It reads directory metadata and requested winning NIF payloads only.
Directory indexes remain resident for a geometry-cache session; extracted
models and reliable missing names remain reusable in versioned provider
snapshots. Archive inputs must remain unchanged during a run.

An unreadable winning entry never substitutes a lower-priority model. Failed
searches do not publish missing names, and failed changed-model indexing retains
the caller's conservative impact fallback. Payload/table bounds, declared decoded
sizes, stream completion, trailing data, and unsafe model paths are validated.
Complete models, missing lists, and statistics are published atomically. Cache
eviction remains confined to owned generated files.

## Reproduce

Build the desktop executable, assertion-enabled tests, and development probe as
described in [development](development.md). The probe is not an application
backend option or a desktop workflow. Its source is under `tools/`; it uses the
same production native reader.

```powershell
xmake f -m releasedbg
xmake build NavmeshGenerator navmesh-tests navmesh-bsa-probe
python -m pip install -r tools/requirements.txt
python tools/test_native_bsa.py ./build/windows/x64/releasedbg/navmesh-bsa-probe.exe
python tools/benchmark_bsa.py --data "<installed game Data>" --probe ./build/windows/x64/releasedbg/navmesh-bsa-probe.exe --output ./build/bsa-measurements.json
```

The benchmark writes sampled assets only into a temporary directory and removes
them on completion. Inputs are opened read-only, and the report retains counters
and a sample digest. Synthetic tests cover versions, compression, prefixes,
precedence, missing-model/index reuse, padding, corrupt/truncated input, unsafe
paths, declared sizes, trailing data, actual NIF geometry loading, provider
revision changes, loose-winner exclusions, and decoded reuse after disk eviction.
