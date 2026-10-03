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
4. Select a **Rebuild scope**: **Cell** works on one area, identified by its Form
   ID or editor ID; **Plugin** rebuilds areas affected by a selected active plugin;
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
- **Create a patch:** enable **Write plugin** to export `generated-navmesh.esp`.
- **Update your own mod:** select Plugin scope and enable **Copy selected plugin**
  to export a copy with regenerated navigation and the plugin's other records.
- **Fill areas without navmeshes:** enable **Skip cells with existing navmesh**
  during generation to preserve areas that already have navigation.

For larger Plugin or Load order runs, use the estimation option to check the
expected workload. **Advanced settings** contains generation and performance
controls; start with the defaults unless you need to adjust them.

## Review and use the results

The output folder contains the files for your chosen operation. Cell inspection
and candidate previews include `scene.glb`, which you can open in a compatible
3D viewer or Blender to compare navigation with the surrounding world.
`navmesh-counts.txt` gives the original and generated polygon counts, and reports
explain warnings, skipped areas, and export results. Batch output depends on the
selected output policy.

Install a generated patch through MO2 and load it after the plugins it depends
on. A **Copy selected plugin** export replaces the selected plugin; use it with
the original mod's assets and language files.

Input plugins and MO2 profile files are left untouched. Existing output plugins
are not overwritten, so choose a new output folder when repeating an export.

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
