# NAVM / NVNM format study

## Scope and evidence

The reader was checked against the
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
| `0x04..0x07` | NVNM location field | Read as raw bytes and preserved by the writer. |
| `0x08..0x0b` | Worldspace FormID | Preserved by the reader; the writer encodes the selected worldspace or zero for an interior. |
| `0x0c..0x0f` | Exterior grid Y/X or interior CELL FormID | Preserved by the reader; the writer encodes the selected CELL. |
| `0x10` | `uint32` vertex count | Bounds checked. |
| `0x14` | `vertexCount` × `{ float x, float y, float z }` | Read as a 12-byte array. |
| after vertices | `uint32` triangle count | Bounds checked. |
| after triangle count | `triangleCount` × 16-byte triangle | Read/preserved.  The runtime definition supports three `uint16` vertex indices, three `uint16` neighbours, `uint16` triangle flags, and `uint16` traversal flags. |
| remaining NVNM bytes | links, doors, cover/grid/other version-specific content | Kept verbatim and exposed as `trailingData`; not interpreted. |

The reader retains trailing bytes. The writer creates fresh external, door,
cover, and grid sections for generated geometry. A matched border edge adds a
portal entry and a reciprocal portal in an adjacent NAVM override. A matched
entrance adds a door triangle referencing the placed door. Other authored
connection sections are not carried into generated geometry.

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

## Guarded override writer

The writer accepts a resolved load order and the selected cell's existing NAVMs.
It copies their group placement and record headers, while leaving parent CELL and
worldspace records in their source plugins. Its TES4 master list contains each
NAVM source plugin and its dependencies in load order; record and group FormIDs
are rebased into that table. It places generated world-space vertices and triangles in the
largest original NAVM and writes empty geometry into the other NAVM overrides.
Generated geometry has matched external and door links, an empty cover section,
and a rebuilt spatial grid. Adjacent NAVM overrides retain their geometry and
authored sections while appending reciprocal external portals. The source plugin
is never modified. The output is always an ESP; its ESL
flag is set when the override-only records and master table fit the light format,
including when a dependency is a regular ESP. All NAVMs are
read back and compared with the serialized geometry.

The generated candidate must be nonempty, topologically valid, and stay within
the selected exterior cell when applicable. A cell with no existing NAVM or
unresolved source dependencies is rejected. Additional authored
NAVM subrecords are rejected because they may contain geometry references.
Unmatched authored external and door connections, cover data, NAVI, and REFR
XNDP references are not rebuilt. They can still break navigation between NAVMs,
doors, or cells. Independent-tool and disposable-profile game validation are
required before an output plugin is used.
Localized source plugins work because no localized parent records are copied.

The generated trailing section layout follows
[OpenMW's NAVM loader](https://gitlab.com/OpenMW/openmw/-/raw/master/components/esm4/loadnavm.cpp).

## Combined affected-cell overrides

`WriteNavmeshOverrides` accepts multiple cell/candidate pairs from the same
resolved snapshot and writes one ESP with a shared master table. Each cell keeps
its own primary NAVM identity. Generated-to-generated links must target matching
edges in the generated primary meshes and have reciprocal generated targets;
replaced secondary NAVMs cannot serve as portal destinations. Authored neighbor
records shared by several replacements are serialized once. Read-back checks
include every cell's geometry, door table and portal entries. Any failure occurs
before the temporary plugin is finalized. See [batch rebuilding](batch-rebuilding.md).
