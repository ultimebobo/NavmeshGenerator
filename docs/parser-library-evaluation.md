# Milestone 1 parser-library evaluation

## Decision

Milestone 1 retains the repository's direct, read-only parser behind the small `IPluginReader` interface. `DirectPluginReader` is intentionally restricted to TES4 headers, group traversal, record identity, master declarations, and the cell fields required for indexing. It does not claim full record decoding or writing coverage.

## Options evaluated

| Option | Fit for a C++ tool | Decision |
| --- | --- | --- |
| xEdit/libxEdit | The most mature record-aware ecosystem, but it is a Delphi-oriented toolchain and would introduce a heavyweight external-process/integration boundary for this C++ binary. | Do not embed for this milestone; retain as an independent validation tool before any writer milestone. |
| CommonLibSSE-NG | Excellent runtime/game type definitions, but it is not an ESM/ESP/ESL file reader and would couple the path to runtime assumptions. | Not suitable. |
| Existing lightweight C++ parsers | Most are partial, obsolete, or carry uncertain coverage/licensing for SE/AE compressed records and light plugins. | No dependency adopted without a maintained, license-compatible candidate and fixture proof. |
| Direct parser | Already in-tree, GPL-compatible with this project, deterministic, and sufficient for a narrow, fixture-tested identity/index spike. | Retained behind `IPluginReader`. |

## Coverage and limits

The direct reader verifies TES4 master declarations, full and light FormID identity, record override chains, and `WRLD`/`CELL` group context (including persistent and temporary cell groups). It intentionally reports compressed payloads as unsupported for metadata extraction. It does not yet decode arbitrary compressed records, resolve every FormID-bearing subrecord, or serialize plugins.

The interface is the replacement seam: a future mature reader must return the same record identity, master list, light-file status, cell/worldspace context, and diagnostics. Re-evaluate a library before milestone 2, especially if fixture coverage reveals record variants this narrow reader cannot safely represent.
