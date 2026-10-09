# NavmeshGenerator

NavmeshGenerator helps Skyrim Special Edition and Anniversary Edition modders
inspect and rebuild navmeshes: the walkable areas that NPCs use to move through
the world. It uses your existing Mod Organizer 2 profile and can export a patch
or a copy of a selected plugin with regenerated navigation.

The tool is experimental. Review generated navigation before using it in a
regular playthrough.

## Before you start

You need Windows, a Skyrim SE/AE installation, and an existing Mod Organizer 2
instance with a configured profile. Have the mods you want to work with enabled
in that profile.

Start with `NavmeshGenerator.exe`. Archived models are read by the native
application and cached on demand. For source builds, follow the
[setup guide](docs/development.md).

## Get started

1. Open `NavmeshGenerator.exe` to launch the desktop window.
2. Choose your **MO2 folder** and enter your existing **Profile** name.
3. Choose a fresh **Output folder** for the results.
4. Select a **Rebuild scope**: **Cell** works on one or multiple areas, identified
   by Form IDs or editor IDs; **Plugin** rebuilds areas affected by a selected active plugin;
   **Load order** rebuilds areas affected by changes across your active mods.
5. Choose the output you want, then run the operation.

If you need a cell identifier, use **List cells to file** to export the available
cells to `cells.json`. Hover over the `?` controls for help. Your settings are
saved for the next session.

## Choose what to produce

- **Inspect a cell:** select Cell scope and leave generation and plugin writing
  disabled to export the existing navigation and surrounding geometry.
- **Preview generated navigation:** enable **Generate candidate NAVM** to inspect
  a proposed navmesh before writing a plugin. NAVM is Skyrim's navmesh record.
- **Make a scene from multiple cells:** select **Cell**, enter the identifiers in
  **Cells**, and enable **Make scene**. One `scene.glb` in the output folder combines
  their geometry and NAVMs, including generated NAVMs when generation is enabled.
  This also works with **Copy selected plugin**.
- **Create a patch:** enable **Write plugin** to export `generated-navmesh.esp`.
  A numeric suffix is added when that filename is already active in the load order.
- **Update your own mod:** select Plugin scope and enable **Copy selected plugin**
  to export a copy with regenerated navigation and the plugin's other records.
- **Rebuild chosen cells in your mod:** select **Cell** and paste cell Form IDs or
  editor IDs into **Cells**, one per line. Commas and semicolons also work. Enable
  **Copy selected plugin** and choose the active filename in **Plugin to copy**
  to write their NAVMs into the exported copy. The list is saved between sessions;
  aliases are combined and unknown or ambiguous identifiers stop processing.
- **Fill areas without navmeshes:** enable **Skip cells with existing navmesh**
  during generation to preserve areas that already have navigation.

For larger Plugin or Load order runs, use the estimation option to check the
expected workload. **Advanced settings** contains generation and performance
controls; start with the defaults unless you need to adjust them.

Generation uses simplified contours and movement-bounded height detail to keep
triangle counts closer to authored navigation. Straight stair flights can become
compact ramps while landings, turns and obstructions retain the detail they need.
Landscape rock collision blocks movement without creating walkable rock tops.
Generation also tags submerged triangles as water and preserves nearby authored
preferred paths on matching floors. Disable **Tag water and preferred path
triangles** in Advanced settings to omit these tags. Candidate JSON and scene
colors expose the classifications for review; exported plugins retain the tags.
Use **Reset** in Advanced settings to adopt current numerical defaults if your
desktop configuration already has saved values. Smaller contour error follows
obstacle outlines more closely and can increase the triangle count.
If contour refinement cannot represent a retained watershed region, generation
recovers with layer regions over the retained floor and records a warning.
Scenes exceeding Recast's vertical raster range are rejected before generation.
Plugin and load-order batches log cell generation failures and continue with the
remaining cells, preserving skipped cells' authored NAVM. Check `batch-report.json`
for `skipped_generation_failed` entries and `complete_with_skips` completion.
Persistent worldspace containers supply placed objects but are skipped as NAVM
destinations, including when their placeholder coordinates overlap a terrain cell.

## Review and use the results

The output folder contains the files for your chosen operation. Cell inspection
and candidate previews include `scene.glb`, which you can open in a compatible
3D viewer or Blender to compare navigation with the surrounding world.
`navmesh-counts.txt` gives the original and generated polygon counts, and reports
explain warnings, skipped areas, and export results. Batch output depends on the
selected output policy.

Plugin and Load order scopes rebuild cells with changed supported terrain,
water inputs, or placed collision. Visual effects, render-only models and
NAVM-only edits do not select regeneration targets. Collision footprints use
transformed triangles, so tall model bounds do not spread vertically into the
horizontal cell grid. Neighboring geometry supplies input without enlarging the
rebuild scope; neighboring NAVMs can receive reciprocal connection updates while
retaining their authored geometry.
New terrain and collision remain eligible without existing navmesh, including
new worldspaces and submerged terrain. Water-only changes select cells with
supported terrain or model collision; an empty water plane does not supply a floor.
Missing NIFs warn and supply no collision; the batch report lists their paths.
Unreadable models and incomplete archive searches stop selection.

Generated border portals use the neighboring NAVM's exact edge endpoints.
Border repair aligns nearby endpoints and keeps interior floor-height detail
when needed to preserve the surrounding slopes, including with finer contours.
In Cell scope, existing exterior crossings are required constraints:
regeneration retains their locations and destinations. Boundary cavities are
retriangulated when a direct match cannot retain a crossing; unresolved required
crossings invalidate the candidate. Cell-scope plugin export stops on an invalid
candidate. Plugin and Load order batches regenerate every selected live cell,
including cells without existing NAVM. All walkable candidates are retained until neighboring
candidates are available. Shared seams receive matching partitions and reciprocal
links; only untouched cells supply authored border constraints. The dedicated
skip-existing option protects authored cells. Cell-local generation and topology
failures are logged and skipped, retaining their authored geometry while other
cells complete. Recovery preserves links to untouched neighbors; cells without
authored NAVM leave open borders and supported walkable floors intact.
Global seam and writer validation failures stop export.
Compatible executable rebuilds reuse completed candidate caches through versioned
pipeline identities; legacy executable-hash entries require verified migration.

Install a generated patch through MO2 and load it after the plugins it depends
on. A **Copy selected plugin** export replaces the selected plugin; use it with
the original mod's assets and language files.

Input plugins and MO2 profile files are left untouched. Existing output plugins
are not overwritten, so choose a new output folder when repeating an export.
New NAVMs for cells without authored navigation include the PathingCell type tag
required by Creation Kit, and plugin verification checks that tag before finalizing.

## Current limitations

Some collision shapes and modded data are unsupported. Areas without enough
usable geometry may be skipped or produce incomplete navigation. Door and
cell-border connections also need review: a successful export does not guarantee
that NPCs can reach every intended area.

Check generated plugins in the Creation Kit or other inspection tools, then
test NPC movement in a separate game profile before adopting them.

## Further help

- [Desktop workflow and settings](docs/operator-workflow.md#desktop-workspace)
- [Viewing exported scenes](docs/scene-inspection.md)
- [Navmesh terms and cell connections](docs/glossary.md)
- [Command-line options, output files, and detailed limitations](docs/command-line.md)
- [Build, dependencies, and development](docs/development.md)
- [Architecture](docs/architecture.md) and [C++ API reference](docs/api.md)

## Acknowledgments

Thanks to [Sw4T's BSAFileExtractor](https://github.com/Sw4T/BSAFileExtractor)
and [Stephen Bunn's bethesda-structs](https://github.com/stephen-bunn/bethesda-structs)
for helping establish the project's initial BSA extraction workflow and archive
format understanding. The current native reader and standalone Python reference
tool operate independently of these projects.
