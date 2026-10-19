# Base Object Swapper inputs

NavmeshGenerator automatically imports Base Object Swapper (BOS) configurations
for inspection and generation. The offline interpretation targets the
[audited BOS source release](https://github.com/powerof3/BaseObjectSwapper/tree/v3.5.0).
BOS replaces existing references' base objects and properties during game
initialization; these changes can exist entirely in INI files without ESP
placement overrides.
See the [BOS documentation](https://www.nexusmods.com/skyrimspecialedition/mods/60805).

## Discovery and snapshot

Discovery reads loose files at the top level of effective Data. It follows BOS's
case-sensitive `.ini` extension and `_SWAP` filename substring checks, followed
by lexical filename ordering. It does not recurse into subdirectories or read
configuration files from BSAs. See the upstream
[configuration discovery](https://github.com/powerof3/CLibUtil/blob/master/include/CLIBUtil/distribution.hpp).

MO2 discovery includes game Data, enabled mods in provider priority, and overwrite.
Mods containing only configurations participate without an associated plugin.
Same-name winners resolve before BOS ordering. Vortex discovery uses deployed
Data; direct Data and manifest inputs use their supplied Data directory.
Configuration contents are retained for the run. Winner paths, membership and
contents form a fingerprint, checked again before plugin publication. A changed
snapshot stops publication and requires a new run.

There is no separate enable option. CLI and desktop use the same application
path. Listing cells remains a metadata operation and does not import geometry
overlays. Terrain-only runs retain the BOS input audit but omit object collision.

## Rule interpretation

The importer resolves editor IDs, `0x`-prefixed resolved hexadecimal FormIDs, and
plugin-qualified local FormIDs against active full and light plugin identities.
Owning-plugin prefixes are independent of the plugin supplying the winning
override. The indexed bound-object types include static, movable, furniture,
door, activator, inventory and leveled-item bases.

Record indexing and geometry eligibility are separate. Ordinary plugin placements
of the additional inventory, leveled-item, light, hazard and idle-marker classes
do not supply scene geometry. Their models become eligible when a BOS rule
actually changes that placement, subject to the existing effect, actor
and door exclusions. Unrelated configurations do not enable ordinary clutter.
Without effective BOS changes, supported geometry remains the same as for the
static, movable-static, activator, tree, flora, furniture and container extraction subset.
Furniture and containers remain obstacle-only; persistent fire exclusions use
the same envelopes for uncertain placements and cannot be fitted to a single hearth pose.

`Forms` selects base swaps, while unconditional `Properties` and `Transforms`
select property rules. Other unconditional sections select reference swaps;
conditional sections other than `Forms` select base properties, as in BOS.
Source/replacement sets follow BOS's unique assignment
branch; invalid undersized replacement sets are skipped with a diagnostic.
Repeated sections are grouped in first-occurrence order. Reference swaps precede
conditional base swaps and unconditional base swaps. Section names are trimmed,
text after a section's closing bracket is ignored, and entries before any named
section use reference-swap semantics, matching BOS's SimpleIni interpretation.
Repeated sections retain their first spelling, and duplicate case-insensitive
keys retain their first position. INI values after `=` are ignored. Whitespace
inside identity fields remains significant; physical INI line endings preserve
source line evidence.
Valid properties attached to the selected swap precede separate property rules;
otherwise reference properties precede conditional base properties and general base properties.
Each selected property bundle stands alone rather than merging fields from
different rules. Unconditional candidates are visited in reverse insertion
order, while conditional groups use reverse condition-key order. These contracts
follow the upstream [rule manager](https://github.com/powerof3/BaseObjectSwapper/blob/master/src/Manager.cpp).

Positions use Skyrim world units. Absolute and relative component transforms
follow BOS; rotations are expressed in degrees in the INI and converted to the
project's radians. `scaleA` assigns the runtime integer percentage and `scale`
multiplies that percentage, including integer quantization. Conversions exceeding
the runtime integer field use its entire representable scale envelope. The importer follows
BOS interpretation even when a writer supplies an apparent scale factor to
`scaleA`; it does not guess corrections to the writer's intent. See upstream
[property application](https://github.com/powerof3/BaseObjectSwapper/blob/master/src/ObjectProperties.cpp).

Initially-disabled and deleted flags control effective reference visibility.
Other flag changes retain possible collision as conservative obstruction because
their runtime collision effects are outside the offline visibility model.
Base replacements refresh model path and classification. Encoded plugin winners
and override histories remain unchanged; runtime placement evidence is stored
separately with the contributing configuration and line.
Chance values above the always-pass threshold pass unconditionally; negative
values never pass. A random replacement list containing an unresolved numeric
identity or a known non-bound form retains lower-priority swap outcomes, because
the runtime lookup can fail before selecting a replacement. Known forms outside
the supported condition classes do not match filters.

## Conservative geometry

Deterministic rules extract the resulting model collision normally. Chance
rules, random replacement sets and transform ranges do not stop generation.
The importer retains every plausible outcome and its fallthrough, without
depending on BOS's reference, location or session RNG seeds. Leveled replacements
retain reachable base objects and the unchanged outcome instead of evaluating
player level or encounter-zone state.

CELL and worldspace conditions resolve from indexed ownership, including the
immediate parent worldspace. Positive filters are alternatives; any matching
negative filter excludes the rule. Location ancestry, region membership and
runtime keywords are treated as unresolved conditions. Both applicable and
inapplicable paths remain possible, with a conservative diagnostic. See
[upstream condition evaluation](https://github.com/powerof3/BaseObjectSwapper/blob/master/src/ConditionalData.cpp).

For uncertain placements, supported model-local collision is enclosed across
the position range and maximum scale. Variable rotations use an enclosing
sphere about the reference origin. The resulting world boxes are filled
obstruction volumes during rasterization, including their interiors; their
roofs never supply navigation floors. Analysis likewise excludes these roofs
from support evidence. This policy can block space that would be usable in an
individual game session, and cannot create a reliable floor on a random platform.

Conservatism is limited to indexed references and supported, available model
collision. Missing models follow the existing warning policy; unreadable
collision evidence stops affected-cell selection. MergeMapper identities,
dynamic/save-only references and later script-driven changes are not emulated.
Malformed or unrepresentable entries stop input processing with the winning
source path and line. Unavailable source or replacement identities produce
diagnostics and should be reviewed before using output. Invalid entries and
malformed recognized properties are rejected explicitly. Property names are
case-sensitive; unknown names are ignored, while recognized properties in the
same bundle still apply, matching BOS. This stricter failure policy for malformed
inputs, supported record/collision coverage,
conservative outcome envelopes and unavailable runtime state are deliberate
limits of offline interpretation; full runtime equivalence is not claimed.

## Selection, caches and outputs

Load order scope includes BOS changes even with a baseline-only plugin list.
Collision differences between encoded placements and effective alternatives
select both old and new footprints, including the interior of continuous sweeps.
Cell scope uses effective placements and imports neighboring suppliers for moved
or uncertain collision. Plugin scope includes BOS changes when that plugin
contributes to the reference or original base history; use Load order scope to
include configuration-only changes independently of plugin attribution.

`bos-input.json` records the input fingerprint, winning config paths,
reference/cell identities, effective alternatives, transforms, envelopes and
diagnostics. Geometry and scene provenance carry contributing config/line
information. Candidate cache identities include geometry provenance and filled
volumes, with a pipeline compatibility revision for the envelope policy.

Output remains a NAVM plugin. Keep the same BOS configurations installed with
the generated navigation and regenerate when they change; the plugin does not
bake BOS placement edits into REFR overrides. Copy-plugin output uses the same
effective geometry and existing dependency checks.

Synthetic C++ and CLI regressions cover INI enumeration, identity syntax,
precedence, transforms, chance bounds, failed-lookup fallthrough, full/light IDs,
removal, movement, conservative volumes, selection, cache identity and discovery
through direct Data, MO2 configurations/overwrite and Vortex deployed files.
