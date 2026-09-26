# NAVM / NVNM format study (Milestone 2)

## Scope and evidence

This is a reader study, not a writer specification.  It was checked against the
repository's synthetic `NAVM` fixture (`TestLossAwareRecordReader`) and against
two independently maintained descriptions of the runtime/plugin ecosystem:

- [Wrye Bash's Skyrim record definitions](https://github.com/wrye-bash/wrye-bash/blob/dev/Mopy/bash/game/skyrim/records.py), which identifies `NAVM` as carrying `NVNM` plus `ONAM`, `PNAM`, and `NNAM` subrecords.
- [CommonLibSSE-NG's runtime layout](https://ng.commonlib.dev/_b_s_navmesh_8h_source.html), which corroborates 12-byte vertices, 16-byte triangles, external-edge information, door portals, cover edges, and grid data.
- [BethesdaLibrary's plugin format notes](https://github.com/BadDogSkyrim/BethesdaLibrary/blob/main/docs/file-formats/plugins.md), which verifies the `0x00040000` compressed-record flag and the `XXXX` extended-subrecord convention required for large NVNM payloads.

The fixture has NVNM version 12, one vertex, one triangle, and an arbitrary
unknown `ZZZZ` subrecord.  The test proves the reader's offsets/counts and exact
unknown-subrecord retention.  It does **not** claim that the synthetic trailing
sections cover all data authored by the Creation Kit.  Local-game samples remain
an opt-in validation input and are not committed to the repository.

## Implemented, verified prefix

`NAVM` is an ordinary plugin record.  Its `NVNM` subrecord is decoded as follows,
with all offsets relative to the start of NVNM data:

| Offset | Field | Status |
| ---: | --- | --- |
| `0x00` | little-endian NVNM version | Read; only version 12's prefix is marked supported. |
| `0x04..0x0f` | NVNM prefix/header fields | Preserved verbatim; semantics intentionally not asserted. |
| `0x10` | `uint32` vertex count | Bounds checked. |
| `0x14` | `vertexCount` × `{ float x, float y, float z }` | Read as a 12-byte array. |
| after vertices | `uint32` triangle count | Bounds checked. |
| after triangle count | `triangleCount` × 16-byte triangle | Read/preserved.  The runtime definition supports three `uint16` vertex indices, three `uint16` neighbours, `uint16` triangle flags, and `uint16` traversal flags. |
| remaining NVNM bytes | links, doors, cover/grid/other version-specific content | Kept verbatim and exposed as `trailingData`; not interpreted. |

The reader does not reconstruct the trailing layout from guesses.  A future
writer must either preserve the original NVNM byte stream exactly or extend this
study with independently checked sample coverage for every modified section.

## Preservation and failure policy

- Every indexed roadmap record keeps its header/file byte ranges, exact on-disk
  payload, decoded payload, and ordered encoded subrecords.  Unknown subrecords,
  duplicate subrecords, ordering, and `XXXX` encodings are therefore available
  for a future byte-faithful round trip.
- Compressed records retain both their compressed source bytes and the decoded
  stream.  The reader rejects a missing size prefix, unsafe declared expansion,
  zlib failure, or decoded-size mismatch with a structured diagnostic.
- Invalid record/group sizes, truncated subrecord headers, dangling `XXXX`, and
  overrun NVNM arrays are malformed input, not partial successes.
- An unrecognised NVNM version is retained but reported as `UnsupportedVersion`;
  no geometry derived from its layout should be trusted.

No plugin serialization is implemented in this milestone.
