# NAVM / NVNM format study

See the [glossary and cell-border explanation](glossary.md) for plain-language
definitions of authored NAVMs, primary/secondary triangle numbering, portals,
and reciprocal links, including why neighboring connection overrides are needed.

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
| `0x04..0x07` | PathingCell type CRC | Preserved for authored overrides; fresh NAVMs write the required non-null PathingCell tag. |
| `0x08..0x0b` | Worldspace FormID | Preserved by the reader; the writer encodes the selected worldspace or zero for an interior. |
| `0x0c..0x0f` | Exterior grid Y/X or interior CELL FormID | Preserved by the reader; the writer encodes the selected CELL. |
| `0x10` | `uint32` vertex count | Bounds checked. |
| `0x14` | `vertexCount` × `{ float x, float y, float z }` | Read as a 12-byte array. |
| after vertices | `uint32` triangle count | Bounds checked. |
| after triangle count | `triangleCount` × 16-byte triangle | Read/preserved.  The runtime definition supports three `uint16` vertex indices, three `uint16` neighbours, `uint16` triangle flags, and `uint16` traversal flags. |
| remaining NVNM bytes | links, doors, cover/grid/other version-specific content | Kept verbatim as `trailingData`; bounds-checked external and door tables also provide scene inspection evidence. Cover/grid and later content remain uninterpreted by the reader. |

The reader retains trailing bytes. Inspection decodes only external entries consumed by flagged triangle edges and valid door associations, resolving their FormIDs through the winning plugin’s master table. Malformed connection tables yield no scene connection evidence; their original bytes remain intact. The writer creates fresh external, door,
cover, and grid sections for generated geometry. A matched border edge adds a
portal entry and a reciprocal portal in an adjacent NAVM override. A matched
entrance adds a door triangle referencing the placed door. Other authored
connection sections are not carried into generated geometry.

## Preservation and failure policy

Generated classification uses the preferred-path and water flag bits defined in
[xEdit's shared NAVM flags](https://github.com/TES5Edit/TES5Edit/blob/dev-4.1.6/Core/wbDefinitionsCommon.pas).
These are independent of external-edge and traversal fields. Optional classification
runs before serialization; read-back comparison checks the complete triangle flags.
See [generated triangle tagging](command-line.md#generated-triangle-tagging).


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
Generated geometry has matched external and door links, rebuilt cover metadata,
and a rebuilt spatial grid. Adjacent NAVM overrides retain their geometry and
authored sections while rebuilding external portals. Every authored NAVM with
an incoming link to replaced geometry receives an override, even without a
matched candidate border. Entries targeting replaced primary or secondary
triangle spaces are removed, consuming edges are cleared, and retained external
indices are remapped. Matched borders receive reciprocal links to generated
triangles; unmatched edges remain open boundaries. Unrelated portals are retained,
and an attempted match on an edge linked to unrelated geometry is rejected.
Small authored border deviations are preserved at the matched portal endpoints,
using the same border tolerance as candidate stitching; other candidate vertices
must stay within the selected exterior CELL. The source plugin is never modified.
The output is always an ESP; its ESL
flag is set when newly allocated identities and the master table fit the light format,
including when a dependency is a regular ESP. All NAVMs are
read back and compared with the serialized geometry. Every consuming external
edge is checked against its table, and emitted destinations must contain the
referenced triangle.

Every live NAVM in a selected CELL must have supported geometry, including cells
that also contain supported NAVMs. An unknown live NAVM layout in a file whose
master/self table can reference a replaced identity also rejects export: its
incoming triangle dependencies cannot be ruled out safely.

The generated candidate must be topologically valid and stay within
the selected exterior cell when applicable, apart from bounded extensions at
matched authored border portal endpoints. Unresolved source dependencies are
rejected. An uncovered CELL receives a new plugin-owned NAVM identity, a fresh
NVNM location prefix, and placement in the winning CELL's temporary child group.
The fresh location prefix includes the PathingCell type CRC before its worldspace
and CELL fields. Creation Kit uses that tag to instantiate the location object;
a null tag leaves those fields unread and misaligns the geometry arrays. Writer
read-back verifies the tag on every newly allocated NAVM, for patches and source
copies as well as interior and exterior cells.
The master table includes the CELL source and its dependencies; parent CELL
payloads are not copied. Persistent door groups are preceded by partial WRLD and CELL
declarations that establish their loading context while inheriting the original parent data.
New identities are allocated consecutively above the
reserved local range, and TES4 HEDR records the next available identity. The ESP
uses the light flag only when those identities and its master table fit the
light format. Existing unsupported NAVM records cannot be treated as uncovered.
ONAM base-object dependencies are rebased, and PNAM/NNAM connector constraints
are remapped to nearby generated vertices. Unknown auxiliary fields, contradictory
constraints and constraints without a representable vertex reject export.
Authored crossings require matched generated portals or explicit obstacle-floor
closure evidence. Reciprocal intra-cell ledges are superseded when nearby generated
thresholds have a shared-edge route. Ledges without that replacement route and
enable/disable portals reject regeneration. Malformed input or a source edit during
generation also stops export; no final ESP is published on a validation failure.
Install a patch after its input plugins and regenerate it after changing that
load-order snapshot. Unrelated later overrides can supersede its navigation.

The optional Plugin-scope copy export retains all unrelated source records,
including localized payloads and TES4 localization flags. It keeps the source
filename so the original external assets and localization resources remain
applicable; those resources are not duplicated by the exporter. The source's
master order and implicit self index remain unchanged. Generated references
outside that table are rejected, and allocated identities must fit the source's
existing full/light format. Replaced NAVMs, affected group sizes, HEDR accounting,
and ONAM registration change. Master-flagged copies register generated overrides
in ONAM, retaining existing entries; an existing ONAM table is also extended in
other copies. This follows [xEdit's ONAM description](https://github.com/TES5Edit/TES5Edit/blob/dev-4.1.6/whatsnew.md),
which identifies the table as overridden records in temporary CELL child groups.
NAVI entries for emitted NAVMs are rebuilt; unrelated source NAVI entries are retained.
Affected door references are merged under their winning group placement.

### Cover and door finalization

Cover is derived from physical collision walls alongside final unlinked edges,
with coincident authored cover retained when applicable. Visual fallback and
navigation-exclusion effects supply no cover. Portal edges remain traversable;
triangle rotation moves cover into the format's available edge slots without
renumbering triangles, and all edge-dependent indices rotate together. Cover
tables are rebuilt from the resulting flags. Matching authored creature
restrictions are retained independently of optional surface classification.

Each generated door portal marks its triangle and receives a reciprocal REFR
XNDP association. Initially disabled doors remain eligible because gameplay can
enable them. An authored live door without a reachable generated triangle stops
export. Obsolete associations are removed from other emitted NAVMs. The writer
copies the winning reference, preserving scripts, locks, ownership, enable
parents, flags and other supported fields while rebasing their embedded FormIDs.
XNDP uses its authored field position or is inserted before the final DATA placement field.
Unknown reference layouts fail rather than being copied with incorrect indices.
Localized reference names require their original filename and string resources.

Read-back checks geometry, cover and grid indices, reciprocal portals and both
directions of door associations against unchanged input destinations as well as
emitted records. Untouched authored portal offsets retain their original relationship
and endpoint coordinates; new or moved edges must agree with their reciprocal edge.
This verifies the exported record graph; runtime movement and
AI behavior still require game testing. The layouts follow
[xEdit's Skyrim record definitions](https://github.com/TES5Edit/TES5Edit/blob/dev-4.1.6/Core/wbDefinitionsTES5.pas).

A winning generated plugin must win for the complete navigation patch, including
neighbor NAVMs and door references. NAVM identities are separate records: winning
one mesh does not suppress other meshes in the same CELL or repair incoming
triangle references by itself. These structural checks reduce conflict hazards;
they do not certify engine stability, saved-game pathing state, dynamic geometry,
or quest-specific movement on regenerated floors. Generate from the complete
active input snapshot and repeat generation when that snapshot changes.

### Navigation information map

Every export includes NAVI derived from the final serialized NAVMs, including
changed authored neighbors and empty secondary overrides. The writer reuses the
input NAVI identity through the output master table; master-free synthetic inputs
allocate a navigation-map identity. NVMI contains area-weighted world-space
locations, preferred-surface fractions, unique consumed edge destinations,
preferred edge destinations, door CRC/reference pairs, and the NAVM's PathingCell
location. Isolated nonempty meshes also carry bounds, triangles, and vertices.
The layout follows [xEdit's Skyrim NAVI definition](https://github.com/TES5Edit/TES5Edit/blob/dev-4.1.6/Core/wbDefinitionsTES5.pas).
Source copies preserve unrelated NVMI entries and deleted-mesh registrations,
removing registrations for regenerated meshes. The optional NVPP precomputed-path
cache is emitted empty so it cannot retain stale routes. NAVI is read back and
compared before the plugin is finalized. It participates in the navigation graph
validation described above.

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

`--skip-existing-navmesh` selects uncovered targets and protects all cells owning
winning NAVM records. Generation retains the largest connected floor in each
selected area or interior, plus components reaching matched doors or real
authored-neighbor portals. Single cells without authored navigation or doors can
receive new NAVMs. Matched borders receive reciprocal authored-neighbor
links without replacing that neighbor's geometry. Unmatched generated seams
retract into the target CELL; single-cell candidates without surviving walkable
floor are skipped. The unsupported connection limitations above apply to new
identities as well.
