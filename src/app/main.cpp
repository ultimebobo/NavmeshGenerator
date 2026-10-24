#include "analysis/navmesh_analysis.h"
#include "core/io/file_path.h"
#include "app/batch_runner.h"
#include "app/asset_failures.h"
#include "app/cell_scene.h"
#include "app/geometry_pipeline.h"
#include "skyrim/extraction/asset_cache.h"
#include "app/run.h"
#include "cli/inspection_report.h"
#include "cli/json_report.h"
#include "core/navmesh/candidate.h"
#include "core/navmesh/candidate_reachability.h"
#include "core/navmesh/generator.h"
#include "core/navmesh/surface_simplification.h"
#include "core/navmesh/triangle_tagging.h"
#include "core/navmesh/cover.h"
#include "core/scene/scene_exporter.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/extraction/terrain_extractor.h"
#include "skyrim/mo2/mo2_importer.h"
#include "skyrim/vortex/vortex_importer.h"
#include "skyrim/parser/affected_cells.h"
#include "skyrim/parser/base_object_swapper.h"
#include "skyrim/parser/plugin_writer.h"
#include "validation/validation.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace
{
    /// Serialize native paths as UTF-8 for mod-manager diagnostics and progress text.
    std::string PathUtf8(const std::filesystem::path &path)
    {
        const auto text = path.generic_u8string();
        return {reinterpret_cast<const char *>(text.data()), text.size()};
    }

    /// Scene markers identify exits from teleport metadata or authored door associations,
    /// independently of whether this run generated a candidate. Cave exits need no visible model.
    [[nodiscard]] std::vector<navmesh::core::CandidateExit> CollectSceneDoors(
        const std::vector<navmesh::core::Cell> &cells, const std::vector<navmesh::core::NavMesh> &meshes,
        const navmesh::core::CandidateNavMesh *candidate)
    {
        std::set<std::uint32_t> authoredDoors;
        for (const auto &mesh : meshes)
        {
            for (const auto &link : mesh.doorLinks)
            {
                authoredDoors.insert(link.referenceId);
            }
        }
        std::map<std::uint32_t, navmesh::core::CandidateExit> doors;
        for (const auto &cell : cells)
        {
            for (const auto &reference : cell.references)
            {
                if (!reference.deleted && !reference.initiallyDisabled &&
                    (reference.teleportExit || authoredDoors.contains(reference.id)))
                {
                    doors.try_emplace(reference.id,
                                      navmesh::core::CandidateExit{
                                          .referenceId = reference.id,
                                          .position = reference.teleportArrival.value_or(reference.position)});
                }
            }
        }
        if (candidate)
        {
            for (const auto &exit : candidate->exits)
            {
                if (doors.contains(exit.referenceId))
                {
                    doors[exit.referenceId] = exit;
                }
            }
        }
        std::vector<navmesh::core::CandidateExit> result;
        for (const auto &[id, door] : doors)
        {
            result.push_back(door);
        }
        return result;
    }

    [[nodiscard]] bool EqualsIgnoreCase(const std::string &left, const std::string &right)
    {
        return left.size() == right.size() &&
               std::equal(left.begin(), left.end(), right.begin(),
                          [](unsigned char a, unsigned char b) { return std::tolower(a) == std::tolower(b); });
    }

    [[nodiscard]] bool ContainsIgnoreCase(const std::string &text, const std::string &fragment)
    {
        return std::search(text.begin(), text.end(), fragment.begin(), fragment.end(),
                           [](unsigned char a, unsigned char b)
                           { return std::tolower(a) == std::tolower(b); }) != text.end();
    }

    /// Publish the same terminal reason to console and desktop consumers without changing progress.
    int ReportFailure(const navmesh::app::ProgressCallback &progress, int percent, std::string message, int code)
    {
        while (!message.empty() && (message.back() == '\n' || message.back() == '\r'))
        {
            message.pop_back();
        }
        std::cerr << message << '\n';
        if (progress)
        {
            progress(percent, message);
        }
        return code;
    }

    /// Name the destination and practical checks even when a serializer cannot provide an OS error.
    std::string WriteFailure(const std::filesystem::path &path)
    {
        return std::format(
            "Cannot write '{}'. Check that its parent folder exists, that you have write permission, "
            "and that the drive has free space. Close applications locking the file or choose another output folder.",
            PathUtf8(std::filesystem::absolute(path)));
    }

    /// Validate run-mode combinations and normalize generation prerequisites before performing I/O.
    int ValidateRunOptions(navmesh::app::Options &options, const navmesh::app::ProgressCallback &progress)
    {
        using namespace navmesh::app;
        const auto fail = [&](const std::string &message, int code)
        { return ReportFailure(progress, 0, message, code); };
        if (options.mo2.empty() && options.vortex.empty() && options.plugin.empty() && options.loadOrder.empty())
        {
            return fail("Usage: NavmeshGenerator --mo2 <instance-or-portable-root> --profile <existing-profile> "
                        "[--mods-dir <moved-mods-root>] [--list-cells] [--cell-formid <hex> | --cells <identifiers> | "
                        "--rebuild-plugin <active "
                        "filename> | --rebuild-load-order] [--generate-plugin] [--no-auto-split] [--copy-plugin] "
                        "[--skip-existing-navmesh] "
                        "[--no-triangle-tagging] [--cell-error-policy <continue|stop>] "
                        "[--batch-output <auto|full|compact|plugin_only>] [--asset-cache <dir>] "
                        "[--cache-budget-mib <MiB>] [--working-memory-mib <MiB>] [--workers <count>] [--estimate-only] "
                        "--output "
                        "<dir>\nVortex input: --vortex <Skyrim-installation> [--vortex-plugins "
                        "<game-application-data-folder>] "
                        "<operation> --output <dir>\nDeveloper/test override: "
                        "--data <Data> --load-order <plugins.txt>.\n",
                        1);
        }
        if ((!options.vortex.empty() &&
             (!options.mo2.empty() || !options.plugin.empty() || !options.loadOrder.empty() || !options.data.empty() ||
              !options.profile.empty() || !options.modsDirectory.empty())) ||
            (!options.vortexPlugins.empty() && options.vortex.empty()))
        {
            return fail("--vortex selects deployed game input and cannot combine MO2 or developer input options; "
                        "--vortex-plugins requires --vortex.\n",
                        1);
        }
        if (!options.mo2.empty() && options.profile.empty())
        {
            return fail("--mo2 requires --profile naming an existing MO2 profile.\n", 1);
        }
        if (options.generatePlugin &&
            (options.listCells || (options.mo2.empty() && options.vortex.empty() && options.loadOrder.empty())))
        {
            return fail("Plugin generation requires resolved mod-manager/load-order input and a rebuild selection.\n",
                        1);
        }
        if (options.copyPlugin && (options.rebuildScope == RebuildScope::LoadOrder || !options.generatePlugin ||
                                   (options.rebuildScope == RebuildScope::Cell ? options.copySourcePlugin.empty()
                                                                               : options.affectedPlugin.empty())))
        {
            return fail("--copy-plugin requires Cell or Plugin scope and an active copy source: "
                        "--copy-plugin-source for Cell scope or --rebuild-plugin for Plugin scope.\n",
                        1);
        }
        if (!options.copySourcePlugin.empty() && (!options.copyPlugin || options.rebuildScope != RebuildScope::Cell))
        {
            return fail("--copy-plugin-source requires --copy-plugin in Cell scope.\n", 1);
        }
        if (options.copyPlugin && options.rebuildScope == RebuildScope::Cell && options.cellSelection.empty() &&
            !options.cellFormId && options.editorId.empty() && !(options.cellX && options.cellY))
        {
            return fail("Cell copy exports require explicit cell identifiers or complete exterior coordinates.\n", 1);
        }
        if (!options.cellSelection.empty() &&
            (options.rebuildScope != RebuildScope::Cell || ParseCellSelection(options.cellSelection).empty() ||
             options.listCells || !options.cell.empty() || options.cellFormId || !options.editorId.empty() ||
             options.cellX || options.cellY || !options.worldspace.empty()))
        {
            return fail("--cells requires a nonempty Cell-mode list and cannot combine other selectors or listing.\n",
                        1);
        }
        const bool batchGeneration = UsesBatchGeneration(options);
        if (options.makeScene &&
            (options.rebuildScope == RebuildScope::LoadOrder || options.listCells || options.estimateOnly))
        {
            return fail(
                "--make-scene requires Cell or small Plugin scope and cannot combine listing or cost estimation.\n", 1);
        }
        if (options.rebuildScope != RebuildScope::Cell)
        {
            if (options.mo2.empty() && options.vortex.empty() && options.loadOrder.empty())
            {
                return fail("Batch rebuilding requires MO2, Vortex or --load-order input.\n", 1);
            }
            if (options.listCells || !options.cell.empty() || options.cellFormId || !options.editorId.empty() ||
                options.cellX || options.cellY || !options.exportGeometry.empty() || !options.exportAnalysis.empty() ||
                !options.exportScene.empty() || options.sceneBounds)
            {
                return fail("Batch rebuilding cannot combine cell selectors, listing, scene bounds, or custom export "
                            "paths.\n",
                            1);
            }
            if (options.rebuildScope == RebuildScope::Plugin && options.affectedPlugin.empty())
            {
                return fail("Select an active plugin filename.\n", 1);
            }
            options.generateCandidate = true;
        }
        if (options.skipExistingNavmesh &&
            (options.listCells || (!options.generateCandidate && !options.generatePlugin) ||
             (options.mo2.empty() && options.vortex.empty() && options.loadOrder.empty())))
        {
            return fail("--skip-existing-navmesh requires generation with resolved mod-manager/load-order input.\n", 1);
        }
        if (options.estimateOnly && (!batchGeneration || options.listCells))
        {
            return fail("--estimate-only requires multi-cell generation, Cell copy, Plugin or Load order scope.\n", 1);
        }
        if (options.batchOutput != "auto" && options.batchOutput != "full" && options.batchOutput != "compact" &&
            options.batchOutput != "plugin_only")
        {
            return fail("--batch-output must be auto, full, compact, or plugin_only.\n", 1);
        }
        if (options.cellErrorPolicy != "continue" && options.cellErrorPolicy != "stop")
        {
            return fail("Cell error policy / --cell-error-policy must be continue or stop.\n", 1);
        }
        if (options.workers == 0 || options.workers > 64)
        {
            return fail(std::format("Invalid Workers / --workers: {}. Enter a whole number from 1 through 64 in "
                                    "Advanced settings or the CLI option.",
                                    options.workers),
                        1);
        }
        if (options.workingMemoryMiB < 64 || options.workingMemoryMiB > 1048576)
        {
            return fail(std::format("Invalid Working memory / --working-memory-mib: {}. Enter a value from 64 through "
                                    "1048576 MiB in Advanced settings or the CLI option.",
                                    options.workingMemoryMiB),
                        1);
        }
        if (options.cacheBudgetMiB > 1048576)
        {
            return fail(std::format("Invalid Cache disk budget / --cache-budget-mib: {}. Enter a value from 0 through "
                                    "1048576 MiB in Advanced settings or the CLI option.",
                                    options.cacheBudgetMiB),
                        1);
        }
        if (options.batchOutput == "plugin_only" && !options.listCells &&
            ((!options.generatePlugin && !options.estimateOnly) || !batchGeneration))
        {
            return fail("plugin_only output requires batch plugin generation or cost estimation.\n", 1);
        }
        if (options.batchOutput == "auto")
        {
            options.batchOutput =
                (options.generatePlugin || options.estimateOnly) && batchGeneration ? "plugin_only" : "full";
        }
        if ((!options.cellSelection.empty() || options.copyPlugin) &&
            (batchGeneration || ParseCellSelection(options.cellSelection).size() > 1) &&
            (!options.exportGeometry.empty() || !options.exportAnalysis.empty() || !options.exportScene.empty() ||
             options.sceneBounds))
        {
            return fail("Multiple-cell and copy exports cannot combine scene bounds or custom export paths.\n", 1);
        }
        if (options.neighboringCellRadius < 0)
        {
            return fail(
                std::format(
                    "Invalid Neighboring cells / --neighboring-cell-radius: {}. Enter a nonnegative whole number.",
                    options.neighboringCellRadius),
                1);
        }
        if (options.generatePlugin)
        {
            options.generateCandidate = true;
        }

        if (options.generateCandidate)
        {
            navmesh::core::ValidateRecastSettings(options.navigationProfile, options.recastSettings);
        }
        if (!std::isfinite(options.surfaceSearchRadius) || options.surfaceSearchRadius <= 0)
        {
            return fail(std::format("Invalid Surface search radius / --surface-search-radius: {}. Enter a finite "
                                    "positive distance in Skyrim units in Advanced settings or the CLI option.",
                                    options.surfaceSearchRadius),
                        1);
        }
        if (!std::isfinite(options.maxSupportDistance) || options.maxSupportDistance < 0)
        {
            return fail(std::format("Invalid Maximum support distance / --max-support-distance: {}. Enter a finite "
                                    "nonnegative distance in Skyrim units in Advanced settings or the CLI option.",
                                    options.maxSupportDistance),
                        1);
        }
        if (!std::isfinite(options.maxSlope) || options.maxSlope < 0 || options.maxSlope >= 90)
        {
            return fail(std::format("Invalid Analysis slope / --max-slope: {}. Enter a finite angle from 0 up to, but "
                                    "excluding, 90 degrees in Advanced settings or the CLI option.",
                                    options.maxSlope),
                        1);
        }

        return 0;
    }

} // namespace

/// Run the shared pipeline; the public boundary annotates exceptions with the current processing stage.
static int RunWithDiagnostics(const navmesh::app::Options &input, const navmesh::app::ProgressCallback &progress,
                              const navmesh::app::CancellationCallback &cancelled,
                              const navmesh::app::RunEventCallback &events)
{
    using namespace navmesh::app;
    auto options = input;
    int lastPercent{};
    const auto fail = [&](const std::string &message, int code = 2)
    { return ReportFailure(progress, lastPercent, message, code); };
    std::size_t activeCellIndex{}, inspectionCellCount{1};
    bool processingCellList{};
    const auto update = [&](int percent, std::string_view status)
    {
        if (progress)
        {
            if (processingCellList && inspectionCellCount > 1)
            {
                const auto inspectionSpan = options.makeScene ? 65 : 70;
                const auto overallPercent =
                    30 + static_cast<int>((inspectionSpan * activeCellIndex + inspectionSpan * percent / 100) /
                                          inspectionCellCount);
                lastPercent = overallPercent;
                progress(overallPercent,
                         std::format("CELL {}/{}: {}", activeCellIndex + 1, inspectionCellCount, status));
            }
            else
            {
                lastPercent = percent;
                progress(percent, status);
            }
        }
    };
    const auto wasCancelled = [&] { return cancelled && cancelled(); };
    update(0, "Validating inputs");
    if (const auto status = ValidateRunOptions(options, progress); status != 0)
    {
        return status;
    }

    const auto inputStarted = std::chrono::steady_clock::now();
    // Resolve one coherent input snapshot shared by listing, single-cell, and batch processing.
    std::optional<navmesh::skyrim::ResolvedLoadOrder> resolved;
    std::optional<navmesh::skyrim::BaseObjectSwapInput> bosInputs;
    std::optional<navmesh::skyrim::Mo2ProfileInput> mo2Input;
    std::optional<navmesh::skyrim::VortexInput> vortexInput;
    navmesh::skyrim::ModelAssetSources modelAssets;
    bool inputReportWritten{};
    const auto importFailure = [&](std::string_view manager, const auto &diagnostics)
    {
        std::string reason;
        for (const auto &diagnostic : diagnostics)
        {
            reason += std::format("\n{}: {}", diagnostic.plugin, diagnostic.message);
        }
        return std::format(
            "{} import found no usable active plugin inputs.{}\nCheck the selected manager folder, active profile and "
            "enabled plugins, then rerun.{}",
            manager, reason,
            inputReportWritten
                ? std::format(" Input report: '{}'.",
                              PathUtf8(std::filesystem::absolute(options.output / "input-report.json")))
                : " The input report could not be saved; check output-folder permissions and free space.");
    };
    const auto sharedCacheRoot = options.assetCache.empty()
                                     ? std::filesystem::temp_directory_path() / "NavmeshGenerator" / "assets"
                                     : options.assetCache;
    if (!options.mo2.empty())
    {
        update(5, "Preparing output folder");
        std::filesystem::create_directories(options.output);
        update(5, "Reading MO2 profile");
        std::cerr << "Importing MO2 profile and virtual-file winners...\n";
        if (!options.modsDirectory.empty())
        {
            std::cerr << "Using --mods-dir override: " << PathUtf8(options.modsDirectory) << "\n";
        }
        try
        {
            mo2Input = navmesh::skyrim::ImportMo2Profile(
                options.mo2, options.profile,
                options.modsDirectory.empty() ? std::nullopt : std::optional(options.modsDirectory),
                sharedCacheRoot / ".mo2");
        }
        catch (const std::exception &error)
        {
            mo2Input = navmesh::skyrim::Mo2ProfileInput{.instanceRoot = options.mo2, .profile = options.profile};
            mo2Input->diagnostics.push_back({navmesh::skyrim::DiagnosticKind::InvalidPlugin, PathUtf8(options.mo2),
                                             std::string("MO2 import failed: ") + error.what()});
        }
        if (!(inputReportWritten = navmesh::skyrim::WriteInputReport(
                  options.output / "input-report.json", *mo2Input,
                  options.listCells || !UsesBatchGeneration(options) || options.batchOutput == "full")))
        {
            std::cerr << WriteFailure(options.output / "input-report.json") << '\n';
        }
        update(15, mo2Input->looseAssetCacheUsed ? "MO2 loose-asset cache loaded" : "MO2 loose-asset cache created");
        std::cerr << (mo2Input->looseAssetCacheUsed ? "Using" : "Created") << " MO2 loose-asset cache in "
                  << PathUtf8(sharedCacheRoot / ".mo2") << "\n";
        std::cerr << "Effective MO2 mods directory: " << PathUtf8(mo2Input->modsDirectory) << "\n";
        for (const auto &diagnostic : mo2Input->diagnostics)
        {
            std::cerr << diagnostic.plugin << ": " << diagnostic.message << "\n";
        }
        if (mo2Input->pluginPaths.empty())
        {
            return fail(importFailure("MO2", mo2Input->diagnostics));
        }
        for (const auto &file : mo2Input->looseAssetWinners)
        {
            if (navmesh::core::PathFromUtf8(file.logicalPath).extension() == ".nif")
            {
                modelAssets.looseModels.emplace(file.logicalPath, file.physicalPath);
            }
        }
        modelAssets.archives = std::move(mo2Input->archivePaths);
        modelAssets.looseRoots = std::move(mo2Input->looseAssetRoots);
        mo2Input->looseAssetWinners.clear();
        mo2Input->looseAssetWinners.shrink_to_fit();
        mo2Input->enabledMods.clear();
        mo2Input->enabledMods.shrink_to_fit();
        options.data = mo2Input->gameData;
        update(20, "Resolving active plugin load order");
        std::cerr << "Resolving worldspace and cell records from " << mo2Input->pluginPaths.size()
                  << " active plugins...\n";
        try
        {
            resolved = navmesh::skyrim::ResolveLoadOrder(
                {.dataDirectory = options.data,
                 .plugins = mo2Input->pluginPaths,
                 .indexReferencesAndNavmeshes = !options.listCells,
                 .progress = [&](std::size_t completed, std::size_t total, const std::filesystem::path &plugin)
                 {
                     const auto percent = total == 0 ? 30 : 20 + static_cast<int>((10 * completed) / total);
                     update(percent, completed == total
                                         ? "Load order records resolved"
                                         : std::format("Resolving plugin {} of {}: {}", completed + 1, total,
                                                       navmesh::core::PathToUtf8(plugin.filename())));
                 }});
        }
        catch (const std::exception &error)
        {
            return fail(std::format("Load-order resolution failed: {}. Check the named plugin and its masters in your "
                                    "mod manager. Imported inputs: '{}'.",
                                    error.what(),
                                    inputReportWritten
                                        ? PathUtf8(std::filesystem::absolute(options.output / "input-report.json"))
                                        : "input report unavailable; check output permissions"));
        }
    }
    else if (!options.vortex.empty())
    {
        update(5, "Preparing output folder");
        std::filesystem::create_directories(options.output);
        update(5, "Reading Vortex deployment");
        std::cerr << "Importing active Vortex deployment...\n";
        vortexInput = navmesh::skyrim::ImportVortexDeployment(options.vortex, options.vortexPlugins);
        if (!(inputReportWritten = navmesh::skyrim::WriteInputReport(
                  options.output / "input-report.json", *vortexInput,
                  options.listCells || !UsesBatchGeneration(options) || options.batchOutput == "full")))
        {
            return fail(WriteFailure(options.output / "input-report.json"));
        }
        for (const auto &diagnostic : vortexInput->diagnostics)
        {
            std::cerr << diagnostic.plugin << ": " << diagnostic.message << "\n";
        }
        if (vortexInput->pluginPaths.empty())
        {
            return fail(importFailure("Vortex", vortexInput->diagnostics));
        }
        options.data = vortexInput->gameData;
        modelAssets = std::move(vortexInput->assets);
        std::cerr << "Using deployed Data: " << navmesh::core::PathToUtf8(options.data) << "\n";
        update(20, "Resolving active plugin load order");
        try
        {
            resolved = navmesh::skyrim::ResolveLoadOrder(
                {.dataDirectory = options.data,
                 .plugins = vortexInput->pluginPaths,
                 .indexReferencesAndNavmeshes = !options.listCells,
                 .progress = [&](std::size_t completed, std::size_t total, const std::filesystem::path &plugin)
                 {
                     const auto percent = total == 0 ? 30 : 20 + static_cast<int>((10 * completed) / total);
                     update(percent, completed == total
                                         ? "Load order records resolved"
                                         : std::format("Resolving plugin {} of {}: {}", completed + 1, total,
                                                       navmesh::core::PathToUtf8(plugin.filename())));
                 }});
        }
        catch (const std::exception &error)
        {
            return fail(std::format("Load-order resolution failed: {}. Check the named plugin and its masters in "
                                    "Vortex, then deploy again. Imported inputs: '{}'.",
                                    error.what(),
                                    inputReportWritten
                                        ? PathUtf8(std::filesystem::absolute(options.output / "input-report.json"))
                                        : "input report unavailable; check output permissions"));
        }
    }
    else if (!options.loadOrder.empty())
    {
        try
        {
            resolved =
                navmesh::skyrim::ResolveLoadOrder({.dataDirectory = options.data,
                                                   .plugins = navmesh::skyrim::ReadLoadOrderManifest(options.loadOrder),
                                                   .indexReferencesAndNavmeshes = !options.listCells});
        }
        catch (const std::exception &error)
        {
            return fail(std::format(
                "Cannot resolve load-order manifest '{}': {}. Check the listed plugin paths and required masters.",
                PathUtf8(options.loadOrder), error.what()));
        }
    }
    else if (!options.listCells && !options.plugin.empty())
    {
        // Standalone previews need the same resolved CELL ownership, terrain
        // halo and neighboring NAVMs as the shared load-order generation path.
        if (options.data.empty())
        {
            options.data = std::filesystem::absolute(options.plugin).parent_path();
        }
        try
        {
            bosInputs = navmesh::skyrim::DiscoverBaseObjectSwaps({options.data});
            if (options.generateCandidate || !options.cellSelection.empty() || !bosInputs->configs.empty())
            {
                resolved = navmesh::skyrim::ResolveLoadOrder(
                    {.dataDirectory = options.data, .plugins = {std::filesystem::absolute(options.plugin)}});
            }
        }
        catch (const std::exception &error)
        {
            return fail(std::format("Cannot resolve preview plugin '{}': {}. Check the plugin path and required "
                                    "masters; use mod-manager input for a complete load order.",
                                    PathUtf8(options.plugin), error.what()));
        }
    }
    if (resolved)
    {
        std::size_t unsupported{};
        for (const auto &diagnostic : resolved->diagnostics)
        {
            if (diagnostic.kind == navmesh::skyrim::DiagnosticKind::UnsupportedRecord)
            {
                ++unsupported;
                continue;
            }
            std::cerr << diagnostic.plugin << ": " << diagnostic.message << "\n";
        }
        if (unsupported != 0)
        {
            std::cerr << unsupported
                      << " unsupported record variants were recorded in input-report.json/load-order.json.\n";
        }
    }
    if (resolved)
    {
        std::string errors;
        for (const auto &diagnostic : resolved->diagnostics)
        {
            if (diagnostic.kind == navmesh::skyrim::DiagnosticKind::MissingMaster ||
                diagnostic.kind == navmesh::skyrim::DiagnosticKind::Cycle ||
                diagnostic.kind == navmesh::skyrim::DiagnosticKind::InvalidPlugin)
            {
                errors += std::format("\n{}: {}", diagnostic.plugin, diagnostic.message);
            }
        }
        if (!errors.empty())
        {
            return fail("Cannot resolve the active load order:" + errors +
                        "\nEnable or install the named masters and correct invalid plugins or dependency cycles in "
                        "your mod manager, then rerun. Vortex users must deploy after changes.");
        }
    }
    if (mo2Input && !navmesh::skyrim::ProfileSnapshotMatches(*mo2Input))
    {
        return fail("MO2 profile inputs changed while resolving the load order; rerun so the snapshot is coherent.\n",
                    2);
    }
    if (vortexInput && !navmesh::skyrim::VortexSnapshotMatches(*vortexInput))
    {
        return fail("Vortex deployment or active plugin inputs changed while resolving the load order; rerun after "
                    "deployment finishes.\n",
                    2);
    }
    update(30, resolved ? "Load order resolved" : "Plugin input ready");
    if (wasCancelled())
    {
        return 3;
    }

    // Listing ends before geometry extraction or any candidate/plugin generation.
    if (options.listCells)
    {
        if (resolved)
        {
            std::filesystem::create_directories(options.output);
            if (!navmesh::cli::WriteLoadOrderJson(options.output / "load-order.json", *resolved))
            {
                return fail(WriteFailure(options.output / "load-order.json"));
            }
        }
        const auto cells = resolved ? resolved->cells : navmesh::skyrim::ListCells(options.plugin);
        if (!options.output.empty())
        {
            std::filesystem::create_directories(options.output);
            if (!navmesh::cli::WriteCellsJson(options.output / "cells.json", cells))
            {
                return fail(WriteFailure(options.output / "cells.json"));
            }
        }
        if (cells.empty())
        {
            return fail(
                "No CELL records were found in the selected inputs. Check that the intended plugins are enabled in the "
                "selected profile; for Vortex, deploy before rerunning List cells / --list-cells.");
        }
        update(100, "Cell list complete");
        return 0;
    }

    std::filesystem::create_directories(options.output);
    if (resolved)
    {
        try
        {
            // Effective Data roots include MO2's enabled providers and overwrite; deployed/direct inputs use Data itself.
            const auto roots = modelAssets.looseRoots.empty() ? std::vector<std::filesystem::path>{options.data}
                                                              : modelAssets.looseRoots;
            if (!bosInputs)
            {
                bosInputs = navmesh::skyrim::DiscoverBaseObjectSwaps(roots);
            }
            const auto diagnostics = navmesh::skyrim::ApplyBaseObjectSwaps(*resolved, *bosInputs);
            for (const auto &diagnostic : diagnostics)
            {
                std::cerr << diagnostic << '\n';
            }
            if (!navmesh::skyrim::WriteBaseObjectSwapReport(options.output / "bos-input.json", *bosInputs, *resolved,
                                                            diagnostics))
            {
                return fail(WriteFailure(options.output / "bos-input.json"));
            }
            std::cerr << std::format("BOS: {} configs, {} effective reference changes.\n", bosInputs->configs.size(),
                                     resolved->runtimePlacements.size());
        }
        catch (const std::exception &error)
        {
            return fail(
                std::format("BOS input resolution failed: {}. Check the named Base Object Swapper configuration "
                            "in the active Data files, then rerun.",
                            error.what()));
        }
    }
    if (resolved && (!UsesBatchGeneration(options) || options.batchOutput == "full"))
    {
        if (!navmesh::cli::WriteLoadOrderJson(options.output / "load-order.json", *resolved))
        {
            return fail(WriteFailure(options.output / "load-order.json"));
        }
    }
    const auto runBatch = [&]()
    {
        if (resolved->plugins.empty())
        {
            return fail("The load order contains no active plugin inputs. Enable the intended plugins in the selected "
                        "profile and rerun; deploy Vortex changes first.",
                        2);
        }
        std::vector<std::filesystem::path> paths;
        if (mo2Input)
        {
            paths = mo2Input->pluginPaths;
        }
        else if (vortexInput)
        {
            paths = vortexInput->pluginPaths;
        }
        else if (!options.loadOrder.empty())
        {
            for (auto path : navmesh::skyrim::ReadLoadOrderManifest(options.loadOrder))
            {
                paths.push_back(path.is_absolute() ? path : options.data / path);
            }
        }
        else
        {
            paths.push_back(std::filesystem::absolute(options.plugin));
        }
        return detail::RunBatch(options, *resolved, (mo2Input || vortexInput) ? &modelAssets : nullptr, paths, progress,
                                cancelled,
                                std::chrono::duration<double>(std::chrono::steady_clock::now() - inputStarted).count(),
                                bosInputs ? &*bosInputs : nullptr, events);
    };
    if (UsesBatchGeneration(options) && (options.rebuildScope != RebuildScope::Cell || !options.cellSelection.empty()))
    {
        return runBatch();
    }
    std::vector<const navmesh::core::Cell *> selectedCells;
    if (!options.cellSelection.empty())
    {
        try
        {
            selectedCells = detail::ResolveCellSelection(*resolved, options.cellSelection);
        }
        catch (const std::runtime_error &error)
        {
            return fail(std::format("{} Use List cells / --list-cells to export cells.json, then copy an exact CELL "
                                    "Form ID or editor ID into Cells / --cells.",
                                    error.what()));
        }
    }
    auto cell = resolved ? std::optional<navmesh::core::Cell>{}
                         : navmesh::skyrim::LoadCell(options.plugin, options.cell, options.worldspace, options.cellX,
                                                     options.cellY, options.cellFormId, options.editorId);
    if (!selectedCells.empty())
    {
        cell = *selectedCells.front();
    }
    else if (resolved)
    {
        for (const auto &candidate : resolved->cells)
        {
            const auto formMatch = options.cellFormId && candidate.id == *options.cellFormId;
            const auto editorMatch =
                !options.editorId.empty() && EqualsIgnoreCase(candidate.editorId, options.editorId);
            const auto cellNameMatch = !options.cell.empty() && (ContainsIgnoreCase(candidate.editorId, options.cell) ||
                                                                 ContainsIgnoreCase(candidate.name, options.cell));
            const auto coordinateMatch = candidate.exteriorCoordinates && options.cellX && options.cellY &&
                                         (*candidate.exteriorCoordinates)[0] == *options.cellX &&
                                         (*candidate.exteriorCoordinates)[1] == *options.cellY;
            if ((!options.cellFormId && options.editorId.empty() && options.cell.empty() && !options.cellX &&
                 !options.cellY) ||
                formMatch || editorMatch || cellNameMatch || coordinateMatch)
            {
                cell = candidate;
                break;
            }
        }
    }
    if (!cell)
    {
        std::string selection;
        if (!options.editorId.empty())
        {
            selection += std::format(" editor ID '{}'", options.editorId);
        }
        if (options.cellFormId)
        {
            selection += std::format(" Form ID {:08X}", *options.cellFormId);
        }
        if (!options.cell.empty())
        {
            selection += std::format(" name '{}'", options.cell);
        }
        if (options.cellX || options.cellY)
        {
            selection += std::format(" exterior coordinates ({}, {})",
                                     options.cellX ? std::to_string(*options.cellX) : "missing X",
                                     options.cellY ? std::to_string(*options.cellY) : "missing Y");
        }
        return fail(std::format("No matching Skyrim CELL was found for{} in {}. Use List cells / --list-cells to "
                                "export cells.json, then copy an exact CELL Form ID or editor ID into Cells / --cells. "
                                "Exterior selection requires both --cell-x and --cell-y.",
                                selection.empty() ? " the selected input" : selection,
                                resolved ? "the active load order" : PathUtf8(options.plugin)));
    }
    if (UsesBatchGeneration(options))
    {
        options.cellSelection = std::format("{:08X}", cell->id);
        return runBatch();
    }
    detail::CellScene combinedScene;
    // The CELL pipeline shares the resolved snapshot; inspection lists vary only the target and artifact directory.
    const auto processCell = [&]() -> int
    {
        update(45, "Cell resolved");
        const auto failedCell =
            [&](std::string_view status, const std::string &reason, const detail::AssetFailureCatalog *assets = nullptr)
        {
            std::ofstream failureReport(options.output / "generation-report.json", std::ios::trunc);
            failureReport << std::format("{{\"form_id\":\"{:08X}\",\"status\":\"{}\",\"cell_error_policy\":\"{}\","
                                         "\"error\":\"{}\",\"asset_failures\":",
                                         cell->id, status, options.cellErrorPolicy,
                                         navmesh::reproducibility::EscapeJson(reason));
            if (assets)
            {
                assets->WriteJson(failureReport);
            }
            else
            {
                failureReport << "[]";
            }
            failureReport << "}\n";
            failureReport.close();
            if (!failureReport)
            {
                return fail(WriteFailure(options.output / "generation-report.json"));
            }
            if (options.cellErrorPolicy == "stop")
            {
                return fail(reason);
            }
            update(100, "Completed with skips. " + reason + " See generation-report.json.");
            return 0;
        };
        if (wasCancelled())
        {
            return 3;
        }
        if (options.skipExistingNavmesh && navmesh::skyrim::CellsWithExistingNavmesh(*resolved).contains(cell->id))
        {
            const auto status = std::format("Skipped CELL {:08X}: existing NAVM records preserved.", cell->id);
            std::ofstream report(options.output / "generation-report.json", std::ios::trunc);
            const navmesh::reproducibility::ExportMetadata metadata{
                .selectedCell = &*cell,
                .warnings = {"Generation skipped because the selected CELL owns winning NAVM records."}};
            report << "{\n  \"metadata\": " << navmesh::reproducibility::ToJson(metadata, "    ") << ",\n";
            report << std::format("  \"form_id\":\"{:08X}\",\"status\":\"skipped_existing_navm\"\n}}\n", cell->id);
            if (!report)
            {
                return fail(WriteFailure(options.output / "generation-report.json"));
            }
            std::cout << status << '\n';
            update(100, status);
            return 0;
        }
        if (resolved)
        {
            if (const auto *winning = resolved->FindWinning(cell->id))
            {
                std::cout << "Winning CELL: " << winning->winning.plugin << "\nOverride chain:";
                for (const auto &origin : winning->origins)
                {
                    std::cout << " " << origin.plugin;
                }
                std::cout << "\n";
            }
        }

        // Neighbor cells supply world-space geometry; generation still targets the selected cell.
        std::vector<navmesh::core::Cell> sceneCells{*cell};
        std::set<std::uint32_t> sceneCellIds{cell->id};
        std::optional<navmesh::core::AABB> neighborhoodBounds;
        if (resolved)
        {
            const navmesh::skyrim::CellImpactIndex index(*resolved);
            const auto radius =
                options.generateCandidate ? std::max(1, options.neighboringCellRadius) : options.neighboringCellRadius;
            const auto neighbors = index.Neighbors(*cell, radius);
            for (const auto *neighbor : neighbors)
            {
                sceneCellIds.insert(neighbor->id);
            }
            if (cell->exteriorCoordinates)
            {
                const auto [x, y] = *cell->exteriorCoordinates;
                neighborhoodBounds =
                    navmesh::core::AABB{.min = {static_cast<float>((static_cast<double>(x) - radius) * 4096.0),
                                                static_cast<float>((static_cast<double>(y) - radius) * 4096.0),
                                                std::numeric_limits<float>::lowest()},
                                        .max = {static_cast<float>((static_cast<double>(x) + radius + 1) * 4096.0),
                                                static_cast<float>((static_cast<double>(y) + radius + 1) * 4096.0),
                                                std::numeric_limits<float>::max()}};
            }
            // Geometry suppliers may lie beyond the scene neighborhood. Their model
            // triangles can contribute, but their LAND and NAVM belong to other cells.
            sceneCells.clear();
            for (const auto *source : options.terrainOnly ? neighbors : index.GeometryNeighbors(*cell, radius))
            {
                auto geometryCell = index.GeometryCell(*source, neighborhoodBounds);
                if (sceneCellIds.contains(source->id))
                {
                    geometryCell.navMeshes = source->navMeshes;
                }
                sceneCells.push_back(std::move(geometryCell));
            }
        }
        navmesh::skyrim::GeometryExtraction geometry;
        const auto assetCacheDirectory = navmesh::skyrim::ModelAssetCacheDirectory(
            resolved ? options.data : options.plugin.parent_path(), (mo2Input || vortexInput) ? &modelAssets : nullptr,
            options.assetCache);
        navmesh::skyrim::ModelGeometryCache modelCache(options.workingMemoryMiB * 1024ULL * 1024ULL / 2);
        for (std::size_t cellIndex{}; cellIndex < sceneCells.size(); ++cellIndex)
        {
            auto extracted =
                options.terrainOnly
                    ? navmesh::skyrim::GeometryExtraction{}
                    : navmesh::skyrim::ExtractGeometry(
                          resolved ? options.data : options.plugin.parent_path(), sceneCells[cellIndex],
                          assetCacheDirectory,
                          [&](std::size_t completed, std::size_t total)
                          {
                              const auto percent = total == 0 ? 65 : 45 + static_cast<int>((20 * completed) / total);
                              const auto &source = sceneCells[cellIndex];
                              const auto model = completed < source.references.size()
                                                     ? source.references[completed].modelPath
                                                     : std::string{};
                              const auto message = std::format(
                                  "Extracting geometry source CELL {:08X} ({}/{}): reference {} of {}; model {}",
                                  source.id, cellIndex + 1, sceneCells.size(), completed, total, model);
                              events({.kind = RunEventKind::Detail, .percent = lastPercent, .message = message});
                              // Keep exception context and overall cell-list progress tied to pipeline updates.
                              if (completed == total)
                              {
                                  update(percent, message);
                              }
                          },
                          wasCancelled, (mo2Input || vortexInput) ? &modelAssets : nullptr, &modelCache);
            if (!options.terrainOnly)
            {
                navmesh::skyrim::TrimModelAssetCache(assetCacheDirectory, options.cacheBudgetMiB * 1024ULL * 1024ULL);
            }
            const bool inNeighborhood = sceneCellIds.contains(sceneCells[cellIndex].id);
            if (!inNeighborhood && neighborhoodBounds)
            {
                detail::CullGeometryToBounds(extracted, *neighborhoodBounds);
            }
            if (resolved && inNeighborhood)
            {
                auto terrain = navmesh::skyrim::ExtractTerrain(*resolved, sceneCells[cellIndex]);
                navmesh::skyrim::GeometryExtraction terrainGeometry;
                terrainGeometry.scene = std::move(terrain.scene);

                terrainGeometry.terrainSupported = terrain.landRecordsDecoded != 0;
                terrainGeometry.terrainLandRecords = terrain.landRecordsFound;
                terrainGeometry.terrainLandDecoded = terrain.landRecordsDecoded;
                terrainGeometry.terrainLandMissing = terrain.landRecordsMissing;
                for (const auto &warning : terrain.warnings)
                {
                    std::cerr << warning << "\n";
                    events({.kind = RunEventKind::Warning, .percent = lastPercent, .message = warning});
                }
                detail::AppendGeometry(extracted, std::move(terrainGeometry));
            }
            if (options.sceneBounds)
            {
                const auto &bounds = *options.sceneBounds;
                if (bounds[0] > bounds[2] || bounds[1] > bounds[3])
                {
                    return fail("Invalid --scene-bounds: minimum X/Y exceeds maximum X/Y. Supply minX minY maxX maxY "
                                "in world coordinates with each minimum no greater than its maximum.",
                                1);
                }
                detail::CullGeometryToBounds(extracted,
                                             {.min = {bounds[0], bounds[1], std::numeric_limits<float>::lowest()},
                                              .max = {bounds[2], bounds[3], std::numeric_limits<float>::max()}});
            }
            detail::AppendGeometry(geometry, std::move(extracted));
            if (wasCancelled())
            {
                update(0, "Cancelled");
                return 3;
            }
        }
        geometry.collisionGeometrySupported = geometry.collisionModelsLoaded != 0;
        update(65, "Geometry extracted");
        if (wasCancelled())
        {
            return 3;
        }
        // Record extraction coverage before exporting geometry and evaluating existing NAVM support.
        navmesh::reproducibility::ExportMetadata metadata{
            .inputPlugin = options.plugin,
            .selectedCell = &*cell,
            .coverage = {.references = cell->references.size(),
                         .referencesWithModels = geometry.referencesWithModels,
                         .modelsLoaded = geometry.modelsLoaded,
                         .modelsMissing = geometry.modelsMissing,
                         .geometryVertices = geometry.scene.mesh.vertices.size(),
                         .geometryTriangles = geometry.scene.mesh.triangles.size(),
                         .terrainLandRecords = geometry.terrainLandRecords,
                         .terrainLandDecoded = geometry.terrainLandDecoded,
                         .terrainLandMissing = geometry.terrainLandMissing,
                         .terrainSupported = geometry.terrainSupported,
                         .collisionGeometrySupported = geometry.collisionGeometrySupported},
            .warnings = {geometry.terrainLandMissing
                             ? "Terrain coverage is missing for this exterior CELL; no flat substitute was emitted."
                             : "Terrain is not applicable to this interior CELL.",
                         geometry.collisionGeometrySupported
                             ? "Collision support includes reachable triangle meshes "
                               "and convex hulls, boxes and tessellated capsules; other shapes are "
                               "not approximated."
                             : "No supported Havok collision was found; any "
                               "render triangles are low-confidence fallbacks."}};
        const auto findings = navmesh::validation::Validate(*cell);
        const auto report = navmesh::cli::ToJson(*cell, findings, metadata);
        const auto reportPath = options.output / "report.json";
        std::ofstream reportStream(reportPath, std::ios::trunc | std::ios::binary);
        reportStream << report;
        const auto geometryPath =
            options.exportGeometry.empty() ? options.output / "geometry.obj" : options.exportGeometry;
        if (!navmesh::skyrim::WriteGeometryObj(geometryPath, geometry))
        {
            std::cerr << WriteFailure(geometryPath) << '\n';
        }
        if (!navmesh::reproducibility::WriteSidecar(geometryPath, metadata))
        {
            std::cerr << WriteFailure(navmesh::core::PathFromUtf8(PathUtf8(geometryPath) + ".metadata.json")) << '\n';
        }
        if (!navmesh::skyrim::WriteGeometryJson(options.output / "geometry.json", *cell, geometry, metadata))
        {
            std::cerr << WriteFailure(options.output / "geometry.json") << '\n';
        }
        update(78, "Geometry exports written");
        if (wasCancelled())
        {
            return 3;
        }

        // Analyze existing navigation independently of the optional replacement candidate.
        update(79, "Analyzing existing NAVM support");
        const auto geometrySummary = navmesh::analysis::AnalyzeGeometry(geometry.scene.mesh);
        const auto meshSummary = navmesh::analysis::AnalyzeNavMesh(cell->navMeshes.empty() ? navmesh::core::NavMesh{}
                                                                                           : cell->navMeshes.front());
        auto analysisConfig =
            navmesh::analysis::AnalysisConfiguration{.surfaceSearchRadius = options.surfaceSearchRadius,
                                                     .maxSupportDistance = options.maxSupportDistance,
                                                     .maxSlope = options.maxSlope};
        if (cell->exteriorCoordinates)
        {
            const auto [x, y] = *cell->exteriorCoordinates;
            analysisConfig.cellBounds =
                navmesh::core::AABB{.min = {x * 4096.0F, y * 4096.0F, std::numeric_limits<float>::lowest()},
                                    .max = {(x + 1) * 4096.0F, (y + 1) * 4096.0F, std::numeric_limits<float>::max()}};
        }
        const auto triangleSources = detail::BuildTriangleSources(geometry);
        auto analysisReport = cell->navMeshes.empty()
                                  ? navmesh::analysis::AnalysisReport{}
                                  : navmesh::analysis::AnalyzeNavMeshPolygons(
                                        cell->navMeshes.front(), geometry.scene.mesh, triangleSources, analysisConfig);
        update(82, "Existing NAVM analysis complete");
        if (wasCancelled())
        {
            return 3;
        }
        detail::AnnotateSupportSources(analysisReport, geometry);
        std::vector<navmesh::core::NavMesh> sceneNavmeshes;
        for (const auto &sceneCell : sceneCells)
        {
            sceneNavmeshes.insert(sceneNavmeshes.end(), sceneCell.navMeshes.begin(), sceneCell.navMeshes.end());
        }
        std::vector<navmesh::core::DiagnosticMarker> sceneMarkers;
        sceneMarkers.reserve(analysisReport.polygons.size());
        for (const auto &polygon : analysisReport.polygons)
        {
            sceneMarkers.push_back(
                {polygon.centroid, polygon.classification, polygon.index,
                 polygon.support.found ? std::optional<std::size_t>{polygon.support.triangleIndex} : std::nullopt,
                 cell->navMeshes.front().id});
        }
        // Generate a neutral candidate, stitch resolved exterior neighbors, and export its evidence.
        std::optional<navmesh::core::CandidateNavMesh> candidate;
        detail::AssetFailureCatalog failedAssets;
        std::string unavailableGeometry;
        if (options.generateCandidate && !geometry.failures.empty())
        {
            for (const auto &failure : geometry.failures)
            {
                failedAssets.Add(failure, "extraction", cell->id);
            }
            unavailableGeometry =
                std::format("CELL {:08X}: required geometry unavailable; authored NAVM retained. {} No replacement "
                            "plugin was written.",
                            cell->id, navmesh::skyrim::DescribeGeometryFailure(geometry.failures.front()));
            if (options.cellErrorPolicy == "stop")
            {
                return failedCell("skipped_extraction_failed", unavailableGeometry, &failedAssets);
            }
        }
        if (options.generateCandidate && unavailableGeometry.empty())
        {
            std::vector<navmesh::core::CandidateExit> exits;
            std::optional<navmesh::core::AABB> candidateBounds;
            for (const auto &sceneCell : sceneCells)
            {
                for (const auto &reference : sceneCell.references)
                {
                    if (reference.recordType == "REFR" && reference.baseRecordType == "DOOR" && !reference.deleted)
                    {
                        exits.push_back({.referenceId = reference.id,
                                         .position = reference.teleportArrival.value_or(reference.position)});
                    }
                }
            }
            candidateBounds = analysisConfig.cellBounds;
            if (resolved && candidateBounds)
            {
                const auto *selectedRecord = resolved->FindWinning(cell->id);
                std::unordered_map<std::uint32_t, std::optional<std::uint32_t>> worldspaceByCell;
                for (const auto &record : resolved->records)
                {
                    if (record.type == "CELL")
                    {
                        worldspaceByCell.emplace(record.formId, record.worldspaceFormId);
                    }
                }
                for (const auto &worldCell : resolved->cells)
                {
                    const auto owner = worldspaceByCell.find(worldCell.id);
                    if (!selectedRecord || owner == worldspaceByCell.end() || !selectedRecord->worldspaceFormId ||
                        owner->second != selectedRecord->worldspaceFormId)
                    {
                        continue;
                    }
                    for (const auto &reference : worldCell.references)
                    {
                        if (reference.recordType == "REFR" && reference.baseRecordType == "DOOR" &&
                            !reference.deleted && reference.position.x >= candidateBounds->min.x &&
                            reference.position.x <= candidateBounds->max.x &&
                            reference.position.y >= candidateBounds->min.y &&
                            reference.position.y <= candidateBounds->max.y)
                        {
                            exits.push_back({.referenceId = reference.id,
                                             .position = reference.teleportArrival.value_or(reference.position)});
                        }
                    }
                }
            }
            std::sort(exits.begin(), exits.end(),
                      [](const auto &a, const auto &b) { return a.referenceId < b.referenceId; });
            exits.erase(std::unique(exits.begin(), exits.end(),
                                    [](const auto &a, const auto &b) { return a.referenceId == b.referenceId; }),
                        exits.end());
            try
            {
                update(83, "Generating candidate NAVM");
                candidate = navmesh::core::RecastCandidateGenerator{}.Generate(
                    geometry.scene, options.navigationProfile, candidateBounds, std::move(exits),
                    options.partitioningAlgorithm, options.recastSettings,
                    navmesh::core::CandidateRetention::AllWalkable);
                update(85, "Candidate NAVM generated");
                if (wasCancelled())
                {
                    return 3;
                }
                if (analysisConfig.cellBounds)
                {
                    update(86, "Matching adjacent NAVM borders");
                    std::vector<navmesh::core::NavMesh> adjacent;
                    const auto &borderSnapshot = *resolved;
                    const auto *selectedRecord = borderSnapshot.FindWinning(cell->id);
                    for (const auto &other : borderSnapshot.cells)
                    {
                        if (!other.exteriorCoordinates || !cell->exteriorCoordinates || other.id == cell->id)
                        {
                            continue;
                        }
                        const auto dx = std::abs((*other.exteriorCoordinates)[0] - (*cell->exteriorCoordinates)[0]);
                        const auto dy = std::abs((*other.exteriorCoordinates)[1] - (*cell->exteriorCoordinates)[1]);
                        if (dx + dy != 1)
                        {
                            continue;
                        }
                        const auto *otherRecord = borderSnapshot.FindWinning(other.id);
                        if (selectedRecord && otherRecord &&
                            otherRecord->worldspaceFormId == selectedRecord->worldspaceFormId)
                        {
                            adjacent.insert(adjacent.end(), other.navMeshes.begin(), other.navMeshes.end());
                        }
                    }
                    (void)navmesh::core::StitchCandidateBorders(*candidate, *analysisConfig.cellBounds, adjacent,
                                                                cell->navMeshes, false, &geometry.scene, true,
                                                                cell->waterHeight);
                }
                // Match portals before filtering; exterior entrances identify playable networks,
                // while doorless selections and interiors retain a primary floor.
                if (candidate->topology.valid)
                {
                    const auto *selectedRecord = resolved ? resolved->FindWinning(cell->id) : nullptr;
                    std::array reachability{navmesh::core::CandidateReachabilityTarget{
                        .cellId = cell->id,
                        .worldspaceId = candidateBounds ? selectedRecord->worldspaceFormId : std::nullopt,
                        .bounds = candidateBounds,
                        .candidate = &*candidate}};
                    (void)navmesh::core::RemoveCandidateIslands(reachability);
                    (void)navmesh::core::SimplifyCandidateSurface(*candidate, candidateBounds);
                }
                navmesh::core::TagCandidateTriangles(*candidate, cell->navMeshes, cell->waterHeight,
                                                     options.tagTriangles);
                navmesh::core::CaptureCoverGeometry(*candidate, geometry.scene);
            }
            catch (const std::bad_alloc &)
            {
                throw;
            }
            catch (const std::length_error &)
            {
                throw;
            }
            catch (const std::exception &error)
            {
                return failedCell(
                    "skipped_generation_failed",
                    std::format("Candidate generation failed for CELL {:08X} ('{}'): {}. Review the generation "
                                "settings and geometry.json in '{}'. If the inputs are valid, include this "
                                "reason and the settings when reporting the failure.",
                                cell->id, cell->editorId, error.what(),
                                PathUtf8(std::filesystem::absolute(options.output))));
            }
            const auto jsonPath = options.output / "candidate-navm.json";
            const auto objPath = options.output / "candidate-navm.obj";
            update(87, "Writing candidate exports");
            if (!navmesh::core::WriteCandidateJson(jsonPath, *candidate, geometry.scene,
                                                   navmesh::reproducibility::ToJson(metadata, "    ")))
            {
                return fail(WriteFailure(jsonPath));
            }
            if (!navmesh::core::WriteCandidateObj(objPath, *candidate))
            {
                return fail(WriteFailure(objPath));
            }
            if (!navmesh::reproducibility::WriteSidecar(objPath, metadata))
            {
                auto sidecarPath = objPath;
                sidecarPath += ".metadata.json";
                return fail(WriteFailure(sidecarPath));
            }
        }
        // Publish counts and the inspection scene before the guarded plugin writer runs.
        std::size_t existingSelectedPolygons{};
        for (const auto &navmesh : cell->navMeshes)
        {
            existingSelectedPolygons += navmesh.polygons.size();
        }
        const auto generatedPolygons = candidate ? candidate->mesh.polygons.size() : 0U;
        const auto navmeshCounts = std::format("Number of original navmesh polygons (selected cell): {}\n"
                                               "Number of generated navmesh polygons: {}\n",
                                               existingSelectedPolygons, generatedPolygons);
        std::ofstream countLog(options.output / "navmesh-counts.txt", std::ios::trunc);
        countLog << navmeshCounts;
        countLog.close();
        if (!countLog)
        {
            return fail(WriteFailure(options.output / "navmesh-counts.txt"));
        }
        std::cout << navmeshCounts;
        if (options.makeScene && selectedCells.size() > 1)
        {
            combinedScene.Append(geometry.scene, sceneCells, sceneMarkers);
        }
        const auto sceneDoors = CollectSceneDoors(sceneCells, sceneNavmeshes, candidate ? &*candidate : nullptr);
        navmesh::core::SceneExportOptions sceneOptions{.layers = detail::ParseSceneLayers(options.geometryLayers),
                                                       .detailedProvenance = options.outputDetail != "summary",
                                                       .candidateEntrances = &sceneDoors};
        if (candidate)
        {
            sceneOptions.layers.push_back(navmesh::core::SceneLayer::CandidateNavmesh);
            sceneOptions.candidateNavmesh = &candidate->mesh;
            sceneOptions.candidateBorderLinks = &candidate->borderLinks;
        }
        if (options.sceneBounds)
        {
            const auto &bounds = *options.sceneBounds;
            if (bounds[0] > bounds[2] || bounds[1] > bounds[3])
            {
                return fail("Invalid --scene-bounds: minimum X/Y exceeds maximum X/Y. Supply minX minY maxX maxY in "
                            "world coordinates with each minimum no greater than its maximum.",
                            1);
            }
            sceneOptions.bounds = navmesh::core::SceneBounds{
                .world = {.min = {bounds[0], bounds[1], std::numeric_limits<float>::lowest()},
                          .max = {bounds[2], bounds[3], std::numeric_limits<float>::max()}}};
        }
        const auto scenePath = options.exportScene.empty() ? options.output / "scene.glb" : options.exportScene;
        update(88, "Writing combined scene");
        const auto sceneExport = navmesh::core::WriteCombinedGlb(scenePath, geometry.scene, sceneNavmeshes,
                                                                 sceneMarkers, metadata, sceneOptions);
        if (!sceneExport.written)
        {
            return fail(WriteFailure(scenePath) + " The scene or its .provenance.json sidecar could not be saved.");
        }
        if (!navmesh::reproducibility::WriteSidecar(scenePath, metadata))
        {
            return fail(WriteFailure(navmesh::core::PathFromUtf8(PathUtf8(scenePath) + ".metadata.json")));
        }
        std::cout << std::format(
            "Exported combined GLB scene to {} ({} objects, {} triangles, {} geometry triangles culled)\n",
            navmesh::core::PathToUtf8(scenePath), sceneExport.objects, sceneExport.triangles,
            sceneExport.culledTriangles);
        if (!unavailableGeometry.empty())
        {
            return failedCell("skipped_extraction_failed", unavailableGeometry, &failedAssets);
        }
        if (candidate && !candidate->topology.valid)
        {
            const auto failure = std::format(
                "Candidate topology validation failed: {}. Inspection scene saved to {}; see candidate-navm.json.",
                candidate->topology.findings.empty() ? "invalid candidate" : candidate->topology.findings.front(),
                navmesh::core::PathToUtf8(scenePath));
            return failedCell("skipped_generation_failed",
                              failure + " Plugin export stopped. Inspect the reported geometry; include the validation "
                                        "reason and candidate exports when reporting the issue.");
        }
        // Serialization requires a valid candidate and retains the resolved source master ordering.
        if (options.generatePlugin && candidate->mesh.polygons.empty())
        {
            std::ofstream generationReport(options.output / "generation-report.json", std::ios::trunc);
            generationReport << "{\n  \"metadata\": " << navmesh::reproducibility::ToJson(metadata, "    ") << ",\n";
            generationReport << std::format("  \"form_id\":\"{:08X}\",\"status\":\"skipped_empty_candidate\"\n}}\n",
                                            cell->id);
            if (!generationReport)
            {
                return fail(WriteFailure(options.output / "generation-report.json"));
            }
            std::cout << "No supported walkable floor survives generation; plugin writing skipped.\n";
        }
        else if (options.generatePlugin)
        {
            std::vector<std::filesystem::path> inputPaths;
            if (mo2Input)
            {
                inputPaths = mo2Input->pluginPaths;
            }
            else if (vortexInput)
            {
                inputPaths = vortexInput->pluginPaths;
            }
            else
            {
                for (auto path : navmesh::skyrim::ReadLoadOrderManifest(options.loadOrder))
                {
                    inputPaths.push_back(path.is_absolute() ? path : options.data / path);
                }
            }
            std::filesystem::path pluginPath;
            std::vector<std::filesystem::path> pluginPaths;
            std::string error;
            update(89, "Writing NAVM override plugin");
            if (bosInputs && !navmesh::skyrim::BaseObjectSwapSnapshotMatches(*bosInputs))
            {
                return fail("BOS configurations changed during generation; rerun with coherent inputs.\n", 2);
            }
            if (!navmesh::skyrim::WriteNavmeshOverride(options.output, inputPaths, *resolved, *cell, *candidate,
                                                       pluginPath, error, options.autoSplitPlugins, &pluginPaths))
            {
                return fail(std::format(
                    "Plugin generation failed: {}. Review candidate-navm.json and scene.glb in '{}' before retrying. "
                    "For unsupported record or linking failures, include this reason when reporting the issue.",
                    error, PathUtf8(std::filesystem::absolute(options.output))));
            }
            std::ifstream writtenPlugin(pluginPath, std::ios::binary);
            std::array<unsigned char, 12> pluginHeader{};
            writtenPlugin.read(reinterpret_cast<char *>(pluginHeader.data()),
                               static_cast<std::streamsize>(pluginHeader.size()));
            const bool eslFlagged = writtenPlugin && (pluginHeader[9] & 0x02U) != 0;
            std::cout << "Generated NAVM override plugin: " << navmesh::core::PathToUtf8(pluginPath)
                      << (eslFlagged ? " (ESL-flagged ESP)\n" : " (regular ESP)\n");
            for (std::size_t index = 1; index < pluginPaths.size(); ++index)
            {
                std::cout << "Generated NAVM override plugin: " << PathUtf8(pluginPaths[index]) << '\n';
            }
            std::cout << "Verified NAVM, NAVI, cover, reciprocal portals and door XNDP associations against the input "
                         "load order.\n";
        }
        update(90, "Navmesh support analyzed");
        if (wasCancelled())
        {
            return 3;
        }

        // Export support/topology diagnostics after optional generation; these remain analysis evidence.
        if (!options.exportAnalysis.empty() && !cell->navMeshes.empty())
        {
            navmesh::cli::WriteAnalysisObj(options.exportAnalysis, cell->navMeshes.front(), geometry.scene.mesh,
                                           analysisReport);
            if (!navmesh::reproducibility::WriteSidecar(options.exportAnalysis, metadata))
            {
                std::cerr << WriteFailure(
                                 navmesh::core::PathFromUtf8(PathUtf8(options.exportAnalysis) + ".metadata.json"))
                          << '\n';
            }
            std::cout << "Exported analysis OBJ to " << options.exportAnalysis << "\n";
        }

        for (std::size_t index = 0; index < cell->navMeshes.size(); ++index)
        {
            const auto &mesh = cell->navMeshes[index];
            const auto meshPath = options.output / std::format("navmesh_{}.obj", index);
            navmesh::cli::WriteObj(meshPath, mesh, std::format("NAVM {:08X}", mesh.id));
            if (!navmesh::reproducibility::WriteSidecar(meshPath, metadata))
            {
                std::cerr << WriteFailure(navmesh::core::PathFromUtf8(PathUtf8(meshPath) + ".metadata.json")) << '\n';
            }
        }

        std::cout << std::format("Geometry bounds:\n  X: {} -> {}\n  Y: {} -> {}\n  Z: {} -> {}\n",
                                 geometrySummary.bounds.min.x, geometrySummary.bounds.max.x,
                                 geometrySummary.bounds.min.y, geometrySummary.bounds.max.y,
                                 geometrySummary.bounds.min.z, geometrySummary.bounds.max.z);
        std::cout << std::format("NAVM bounds:\n  X: {} -> {}\n  Y: {} -> {}\n  Z: {} -> {}\n",
                                 meshSummary.bounds.min.x, meshSummary.bounds.max.x, meshSummary.bounds.min.y,
                                 meshSummary.bounds.max.y, meshSummary.bounds.min.z, meshSummary.bounds.max.z);
        std::cout << std::format("Geometry center: ({}, {}, {})\nNAVM center: ({}, {}, {})\nGeometry extent: ({}, {}, "
                                 "{})\nNAVM extent: ({}, {}, {})\n",
                                 geometrySummary.center.x, geometrySummary.center.y, geometrySummary.center.z,
                                 meshSummary.center.x, meshSummary.center.y, meshSummary.center.z,
                                 geometrySummary.extent.x, geometrySummary.extent.y, geometrySummary.extent.z,
                                 meshSummary.extent.x, meshSummary.extent.y, meshSummary.extent.z);
        std::cout << std::format("Analysis thresholds: surfaceSearchRadius={} maxSupportDistance={} maxSlope={}\n",
                                 options.surfaceSearchRadius, options.maxSupportDistance, options.maxSlope);

        navmesh::cli::WriteAnalysisJson(options.output / "analysis.json", analysisReport, geometrySummary, meshSummary,
                                        metadata);
        if (!cell->navMeshes.empty())
        {
            const auto sceneReportPath = options.output / "scene-report.html";
            navmesh::cli::WriteDiagnosticHtml(sceneReportPath, *cell, cell->navMeshes.front(), geometry.scene.mesh,
                                              geometry, analysisReport);
            if (!navmesh::reproducibility::WriteSidecar(sceneReportPath, metadata))
            {
                std::cerr << WriteFailure(navmesh::core::PathFromUtf8(PathUtf8(sceneReportPath) + ".metadata.json"))
                          << '\n';
            }
        }
        if (options.diagnostics && !cell->navMeshes.empty())
        {
            const auto diagnosticPath = options.output / "navmesh_diagnostics.html";
            navmesh::cli::WriteDiagnosticHtml(diagnosticPath, *cell, cell->navMeshes.front(), geometry.scene.mesh,
                                              geometry, analysisReport);
            if (!navmesh::reproducibility::WriteSidecar(diagnosticPath, metadata))
            {
                std::cerr << WriteFailure(navmesh::core::PathFromUtf8(PathUtf8(diagnosticPath) + ".metadata.json"))
                          << '\n';
            }
            std::cout << "Wrote diagnostic report to " << diagnosticPath << "\n";
        }
        navmesh::cli::PrintAnalysisSummary(analysisReport);

        std::cout << std::format("Resolved cell: {:08X} editor_id=\"{}\" name=\"{}\" type={}", cell->id, cell->editorId,
                                 cell->name, cell->isInterior ? "interior" : "exterior");
        if (cell->exteriorCoordinates)
        {
            std::cout << std::format(" coords=({}, {})", (*cell->exteriorCoordinates)[0],
                                     (*cell->exteriorCoordinates)[1]);
        }
        std::cout << "\n";

        if (!cell->navMeshes.empty())
        {
            const auto analysis = navmesh::analysis::Analyze(cell->navMeshes.front());
            std::cout << std::format(
                "Cell: {:08X}\nWorldspace: {}\nNAVM vertices: {}\nNAVM polygons: {}\nConnected regions: {}\nIsolated "
                "polygons: {}\nDegenerate polygons: {}\nAverage polygon area: {}\n",
                cell->id, options.worldspace.empty() ? "<unspecified>" : options.worldspace, analysis.vertexCount,
                analysis.polygonCount, analysis.connectedComponents, analysis.isolatedPolygonCount,
                analysis.degeneratePolygonCount, analysis.averagePolygonArea);
        }

        std::cout << "Wrote report to " << reportPath << "\n";
        std::cout << std::format("References: {}\nReferences with models: {}\nModels loaded: {}\nModels missing: "
                                 "{}\nGeometry vertices: {}\nGeometry triangles: {}\nExported geometry: {}\n",
                                 cell->references.size(), geometry.referencesWithModels, geometry.modelsLoaded,
                                 geometry.modelsMissing, geometry.scene.mesh.vertices.size(),
                                 geometry.scene.mesh.triangles.size(), navmesh::core::PathToUtf8(geometryPath));
        update(100, std::format("Original navmesh polygons: {}; generated navmesh polygons: {}",
                                existingSelectedPolygons, generatedPolygons));
        return 0;
    };
    if (selectedCells.size() <= 1)
    {
        return processCell();
    }
    const auto outputRoot = options.output;
    inspectionCellCount = selectedCells.size();
    processingCellList = true;
    for (const auto *selected : selectedCells)
    {
        if (wasCancelled())
        {
            return 3;
        }
        cell = *selected;
        options.output = outputRoot / "cells" / std::format("{:08X}", selected->id);
        std::filesystem::create_directories(options.output);
        if (const auto status = processCell(); status != 0)
        {
            return status;
        }
        ++activeCellIndex;
    }
    processingCellList = false;
    if (options.makeScene)
    {
        if (wasCancelled())
        {
            return 3;
        }
        update(95, "Writing scene for selected cells");
        if (!combinedScene.Write(outputRoot / "scene.glb", options, selectedCells))
        {
            return fail(WriteFailure(outputRoot / "scene.glb"));
        }
    }
    update(100, std::format("Inspected {} cells", selectedCells.size()));
    return 0;
}

int navmesh::app::Run(const Options &options, const ProgressCallback &progress, const CancellationCallback &cancelled,
                      const RunEventCallback &events)
{
    detail::RunProgress activity(options.output, progress, events);
    int lastPercent{};
    std::string stage = "Validating inputs";
    const auto update = [&](int percent, std::string_view status)
    {
        lastPercent = percent;
        stage = status;
        activity.Publish({.percent = percent, .message = std::string(status)});
    };
    const auto observe = [&](const RunEvent &event) { activity.Publish(event); };
    int code{};
    try
    {
        code = RunWithDiagnostics(options, update, cancelled, observe);
    }
    catch (const std::filesystem::filesystem_error &error)
    {
        code = ReportFailure(
            update, lastPercent,
            std::format("{} failed: {}. Check that the named path exists, is accessible, and has enough free space for "
                        "writes; choose another output or cache folder if needed.",
                        stage, error.what()),
            1);
    }
    catch (const std::exception &error)
    {
        code = ReportFailure(update, lastPercent,
                             std::format("{} failed: {}. Output folder: '{}'. If the inputs and settings are valid, "
                                         "include this reason and the run settings when reporting the issue.",
                                         stage, error.what(), PathUtf8(options.output)),
                             1);
    }
    activity.Finish(code);
    return code;
}
