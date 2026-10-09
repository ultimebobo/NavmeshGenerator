#include "analysis/navmesh_analysis.h"
#include "app/batch_runner.h"
#include "app/cell_scene.h"
#include "app/geometry_pipeline.h"
#include "skyrim/extraction/asset_cache.h"
#include "app/run.h"
#include "cli/inspection_report.h"
#include "cli/json_report.h"
#include "core/navmesh/candidate.h"
#include "core/navmesh/generator.h"
#include "core/navmesh/triangle_tagging.h"
#include "core/scene/scene_exporter.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/extraction/terrain_extractor.h"
#include "skyrim/mo2/mo2_importer.h"
#include "skyrim/parser/affected_cells.h"
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
                    doors.try_emplace(reference.id, navmesh::core::CandidateExit{.referenceId = reference.id,
                                                                                 .position = reference.position});
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

    /// Validate run-mode combinations and normalize generation prerequisites before performing I/O.
    int ValidateRunOptions(navmesh::app::Options &options)
    {
        using namespace navmesh::app;
        if (options.mo2.empty() && options.plugin.empty() && options.loadOrder.empty())
        {
            std::cerr
                << "Usage: NavmeshGenerator --mo2 <instance-or-portable-root> --profile <existing-profile> "
                   "[--mods-dir <moved-mods-root>] [--list-cells] [--cell-formid <hex> | --cells <identifiers> | "
                   "--rebuild-plugin <active "
                   "filename> | --rebuild-load-order] [--generate-plugin] [--copy-plugin] [--skip-existing-navmesh] "
                   "[--no-triangle-tagging] [--batch-output <auto|full|compact|plugin_only>] [--asset-cache <dir>] "
                   "[--cache-budget-mib <MiB>] [--working-memory-mib <MiB>] [--workers <count>] [--estimate-only] "
                   "--output "
                   "<dir>\nDeveloper/test override: "
                   "--data <Data> --load-order <plugins.txt>.\n";
            return 1;
        }
        if (!options.mo2.empty() && options.profile.empty())
        {
            std::cerr << "--mo2 requires --profile naming an existing MO2 profile.\n";
            return 1;
        }
        if (options.generatePlugin && (options.listCells || (options.mo2.empty() && options.loadOrder.empty())))
        {
            std::cerr << "Plugin generation requires a resolved MO2/load-order input and a rebuild selection.\n";
            return 1;
        }
        if (options.copyPlugin && (options.rebuildScope == RebuildScope::LoadOrder || !options.generatePlugin ||
                                   (options.rebuildScope == RebuildScope::Cell ? options.copySourcePlugin.empty()
                                                                               : options.affectedPlugin.empty())))
        {
            std::cerr << "--copy-plugin requires Cell or Plugin scope and an active copy source: "
                         "--copy-plugin-source for Cell scope or --rebuild-plugin for Plugin scope.\n";
            return 1;
        }
        if (!options.copySourcePlugin.empty() && (!options.copyPlugin || options.rebuildScope != RebuildScope::Cell))
        {
            std::cerr << "--copy-plugin-source requires --copy-plugin in Cell scope.\n";
            return 1;
        }
        if (options.copyPlugin && options.rebuildScope == RebuildScope::Cell && options.cellSelection.empty() &&
            !options.cellFormId && options.editorId.empty() && !(options.cellX && options.cellY))
        {
            std::cerr << "Cell copy exports require explicit cell identifiers or complete exterior coordinates.\n";
            return 1;
        }
        if (!options.cellSelection.empty() &&
            (options.rebuildScope != RebuildScope::Cell || ParseCellSelection(options.cellSelection).empty() ||
             options.listCells || !options.cell.empty() || options.cellFormId || !options.editorId.empty() ||
             options.cellX || options.cellY || !options.worldspace.empty()))
        {
            std::cerr << "--cells requires a nonempty Cell-mode list and cannot combine other selectors or listing.\n";
            return 1;
        }
        const bool batchGeneration = UsesBatchGeneration(options);
        if (options.makeScene &&
            (options.rebuildScope != RebuildScope::Cell || options.listCells || options.estimateOnly))
        {
            std::cerr << "--make-scene requires Cell scope and cannot combine listing or cost estimation.\n";
            return 1;
        }
        if (options.rebuildScope != RebuildScope::Cell)
        {
            if (options.mo2.empty() && options.loadOrder.empty())
            {
                std::cerr << "Batch rebuilding requires MO2 or --load-order input.\n";
                return 1;
            }
            if (options.listCells || !options.cell.empty() || options.cellFormId || !options.editorId.empty() ||
                options.cellX || options.cellY || !options.exportGeometry.empty() || !options.exportAnalysis.empty() ||
                !options.exportScene.empty() || options.sceneBounds)
            {
                std::cerr << "Batch rebuilding cannot combine cell selectors, listing, scene bounds, or custom export "
                             "paths.\n";
                return 1;
            }
            if (options.rebuildScope == RebuildScope::Plugin && options.affectedPlugin.empty())
            {
                std::cerr << "Select an active plugin filename.\n";
                return 1;
            }
            options.generateCandidate = true;
        }
        if (options.skipExistingNavmesh &&
            (options.listCells || (!options.generateCandidate && !options.generatePlugin) ||
             (options.mo2.empty() && options.loadOrder.empty())))
        {
            std::cerr << "--skip-existing-navmesh requires generation with resolved MO2/load-order input.\n";
            return 1;
        }
        if (options.estimateOnly && (!batchGeneration || options.listCells))
        {
            std::cerr << "--estimate-only requires multi-cell generation, Cell copy, Plugin or Load order scope.\n";
            return 1;
        }
        if (options.batchOutput != "auto" && options.batchOutput != "full" && options.batchOutput != "compact" &&
            options.batchOutput != "plugin_only")
        {
            std::cerr << "--batch-output must be auto, full, compact, or plugin_only.\n";
            return 1;
        }
        if (options.workers == 0 || options.workers > 64 || options.workingMemoryMiB < 64 ||
            options.workingMemoryMiB > 1048576 || options.cacheBudgetMiB > 1048576)
        {
            std::cerr << "Workers must be between 1 and 64; working memory must be between 64 and 1048576 MiB; cache "
                         "budget must not exceed 1048576 MiB.\n";
            return 1;
        }
        if (options.batchOutput == "plugin_only" && !options.listCells &&
            ((!options.generatePlugin && !options.estimateOnly) || !batchGeneration))
        {
            std::cerr << "plugin_only output requires batch plugin generation or cost estimation.\n";
            return 1;
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
            std::cerr << "Multiple-cell and copy exports cannot combine scene bounds or custom export paths.\n";
            return 1;
        }
        if (options.neighboringCellRadius < 0)
        {
            std::cerr << "Neighboring-cell radius cannot be negative.\n";
            return 1;
        }
        if (options.generatePlugin)
        {
            options.generateCandidate = true;
        }

        if (options.generateCandidate)
        {
            navmesh::core::ValidateRecastSettings(options.navigationProfile, options.recastSettings);
        }
        if (!std::isfinite(options.surfaceSearchRadius) || options.surfaceSearchRadius <= 0 ||
            !std::isfinite(options.maxSupportDistance) || options.maxSupportDistance < 0 ||
            !std::isfinite(options.maxSlope) || options.maxSlope < 0 || options.maxSlope >= 90)
        {
            std::cerr << "Analysis settings require a positive search radius, nonnegative support distance, "
                         "and a slope below 90 degrees; all values must be finite.\n";
            return 1;
        }

        return 0;
    }

} // namespace

int navmesh::app::Run(const Options &input, const ProgressCallback &progress, const CancellationCallback &cancelled)
{
    auto options = input;
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
                progress(overallPercent,
                         std::format("CELL {}/{}: {}", activeCellIndex + 1, inspectionCellCount, status));
            }
            else
            {
                progress(percent, status);
            }
        }
    };
    const auto wasCancelled = [&] { return cancelled && cancelled(); };
    update(0, "Validating inputs");
    if (const auto status = ValidateRunOptions(options); status != 0)
    {
        return status;
    }

    const auto inputStarted = std::chrono::steady_clock::now();
    // Resolve one coherent input snapshot shared by listing, single-cell, and batch processing.
    std::optional<navmesh::skyrim::ResolvedLoadOrder> resolved;
    std::optional<navmesh::skyrim::Mo2ProfileInput> mo2Input;
    navmesh::skyrim::ModelAssetSources modelAssets;
    const auto sharedCacheRoot = options.assetCache.empty()
                                     ? std::filesystem::temp_directory_path() / "NavmeshGenerator" / "assets"
                                     : options.assetCache;
    if (!options.mo2.empty())
    {
        std::filesystem::create_directories(options.output);
        update(5, "Reading MO2 profile");
        std::cerr << "Importing MO2 profile and virtual-file winners...\n";
        if (!options.modsDirectory.empty())
        {
            std::cerr << "Using --mods-dir override: " << options.modsDirectory.string() << "\n";
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
            mo2Input->diagnostics.push_back({navmesh::skyrim::DiagnosticKind::InvalidPlugin, options.mo2.string(),
                                             std::string("MO2 import failed: ") + error.what()});
        }
        if (!navmesh::skyrim::WriteInputReport(options.output / "input-report.json", *mo2Input,
                                               options.listCells || !UsesBatchGeneration(options) ||
                                                   options.batchOutput == "full"))
        {
            std::cerr << "Failed to write input-report.json.\n";
        }
        update(15, mo2Input->looseAssetCacheUsed ? "MO2 loose-asset cache loaded" : "MO2 loose-asset cache created");
        std::cerr << (mo2Input->looseAssetCacheUsed ? "Using" : "Created") << " MO2 loose-asset cache in "
                  << (sharedCacheRoot / ".mo2").string() << "\n";
        std::cerr << "Effective MO2 mods directory: " << mo2Input->modsDirectory.string() << "\n";
        for (const auto &diagnostic : mo2Input->diagnostics)
        {
            std::cerr << diagnostic.plugin << ": " << diagnostic.message << "\n";
        }
        if (mo2Input->pluginPaths.empty())
        {
            std::cerr << "MO2 import found no usable active plugin paths; see input-report.json.\n";
            return 2;
        }
        for (const auto &file : mo2Input->looseAssetWinners)
        {
            if (std::filesystem::path(file.logicalPath).extension() == ".nif")
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
                     update(percent, completed == total ? "Load order records resolved"
                                                        : std::format("Resolving plugin {} of {}: {}", completed + 1,
                                                                      total, plugin.filename().string()));
                 }});
        }
        catch (const std::exception &error)
        {
            std::cerr << "Load-order resolution failed: " << error.what()
                      << ". See input-report.json for the imported profile inputs.\n";
            return 2;
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
            std::cerr << "Developer load-order resolution failed: " << error.what() << "\n";
            return 2;
        }
    }
    else if ((options.generateCandidate || !options.cellSelection.empty()) && !options.plugin.empty())
    {
        // Standalone previews need the same resolved CELL ownership, terrain
        // halo and neighboring NAVMs as the shared load-order generation path.
        if (options.data.empty())
        {
            options.data = std::filesystem::absolute(options.plugin).parent_path();
        }
        try
        {
            resolved = navmesh::skyrim::ResolveLoadOrder(
                {.dataDirectory = options.data, .plugins = {std::filesystem::absolute(options.plugin)}});
        }
        catch (const std::exception &error)
        {
            std::cerr << "Preview plugin resolution failed: " << error.what() << "\n";
            return 2;
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
    if (resolved && std::any_of(resolved->diagnostics.begin(), resolved->diagnostics.end(),
                                [](const auto &d)
                                {
                                    return d.kind == navmesh::skyrim::DiagnosticKind::MissingMaster ||
                                           d.kind == navmesh::skyrim::DiagnosticKind::Cycle ||
                                           d.kind == navmesh::skyrim::DiagnosticKind::InvalidPlugin;
                                }))
    {
        return 2;
    }
    if (mo2Input && !navmesh::skyrim::ProfileSnapshotMatches(*mo2Input))
    {
        std::cerr << "MO2 profile inputs changed while resolving the load order; rerun so the snapshot is coherent.\n";
        return 2;
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
            navmesh::cli::WriteLoadOrderJson(options.output / "load-order.json", *resolved);
        }
        const auto cells = resolved ? resolved->cells : navmesh::skyrim::ListCells(options.plugin);
        if (!options.output.empty())
        {
            std::filesystem::create_directories(options.output);
            navmesh::cli::WriteCellsJson(options.output / "cells.json", cells);
        }
        update(100, cells.empty() ? "No cells found" : "Cell list complete");
        return cells.empty() ? 2 : 0;
    }

    std::filesystem::create_directories(options.output);
    if (resolved && (!UsesBatchGeneration(options) || options.batchOutput == "full"))
    {
        navmesh::cli::WriteLoadOrderJson(options.output / "load-order.json", *resolved);
    }
    const auto runBatch = [&]()
    {
        if (resolved->plugins.empty())
        {
            std::cerr << "The load order contains no active plugin inputs.\n";
            return 2;
        }
        std::vector<std::filesystem::path> paths;
        if (mo2Input)
        {
            paths = mo2Input->pluginPaths;
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
        return detail::RunBatch(options, *resolved, mo2Input ? &modelAssets : nullptr, paths, progress, cancelled,
                                std::chrono::duration<double>(std::chrono::steady_clock::now() - inputStarted).count());
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
            std::cerr << error.what() << '\n';
            return 2;
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
            const auto coordinateMatch = candidate.exteriorCoordinates && options.cellX && options.cellY &&
                                         (*candidate.exteriorCoordinates)[0] == *options.cellX &&
                                         (*candidate.exteriorCoordinates)[1] == *options.cellY;
            if ((!options.cellFormId && options.editorId.empty() && !options.cellX && !options.cellY) || formMatch ||
                editorMatch || coordinateMatch)
            {
                cell = candidate;
                break;
            }
        }
    }
    if (!cell)
    {
        if (resolved)
        {
            std::cerr << "No matching Skyrim CELL was found in the resolved MO2 load order";
            if (!options.editorId.empty())
            {
                std::cerr << " for editor ID '" << options.editorId << "'";
            }
            else if (options.cellFormId)
            {
                std::cerr << " for form ID " << std::format("{:08X}", *options.cellFormId);
            }
            else if (options.cellX && options.cellY)
            {
                std::cerr << " at exterior coordinates " << *options.cellX << "," << *options.cellY;
            }
            std::cerr << ". Use List cells to find an exact CELL editor ID or form ID.\n";
        }
        else
        {
            std::cerr << "No matching Skyrim cell was found in " << options.plugin << "\n";
        }
        return 2;
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
                std::cerr << "Cannot write generation-report.json.\n";
                return 2;
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
        const auto assetCacheDirectory =
            navmesh::skyrim::ModelAssetCacheDirectory(resolved ? options.data : options.plugin.parent_path(),
                                                      mo2Input ? &modelAssets : nullptr, options.assetCache);
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
                              update(percent, std::format("Extracting geometry source cell {}/{}: reference {} of {}",
                                                          cellIndex + 1, sceneCells.size(), completed, total));
                          },
                          wasCancelled, mo2Input ? &modelAssets : nullptr, &modelCache);
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
                }
                detail::AppendGeometry(extracted, std::move(terrainGeometry));
            }
            if (options.sceneBounds)
            {
                const auto &bounds = *options.sceneBounds;
                if (bounds[0] > bounds[2] || bounds[1] > bounds[3])
                {
                    std::cerr << "--scene-bounds requires minX minY maxX maxY.\n";
                    return 1;
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
                         geometry.collisionGeometrySupported ? "Collision support is limited to reachable "
                                                               "bhkPackedNiTriStripsData; unsupported Havok shapes are "
                                                               "not approximated."
                                                             : "No supported packed Havok collision was found; any "
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
            std::cerr << "Failed to write geometry OBJ to " << geometryPath << "\n";
        }
        if (!navmesh::reproducibility::WriteSidecar(geometryPath, metadata))
        {
            std::cerr << "Failed to write geometry metadata sidecar\n";
        }
        if (!navmesh::skyrim::WriteGeometryJson(options.output / "geometry.json", *cell, geometry, metadata))
        {
            std::cerr << "Failed to write geometry JSON\n";
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
        if (options.generateCandidate)
        {
            std::vector<navmesh::core::CandidateExit> exits;
            std::optional<navmesh::core::AABB> candidateBounds;
            for (const auto &sceneCell : sceneCells)
            {
                for (const auto &reference : sceneCell.references)
                {
                    if (reference.recordType == "REFR" && reference.baseRecordType == "DOOR" && !reference.deleted &&
                        !reference.initiallyDisabled)
                    {
                        exits.push_back({.referenceId = reference.id, .position = reference.position});
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
                            !reference.deleted && !reference.initiallyDisabled &&
                            reference.position.x >= candidateBounds->min.x &&
                            reference.position.x <= candidateBounds->max.x &&
                            reference.position.y >= candidateBounds->min.y &&
                            reference.position.y <= candidateBounds->max.y)
                        {
                            exits.push_back({.referenceId = reference.id, .position = reference.position});
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
                    options.partitioningAlgorithm, options.recastSettings);
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
                                                                cell->navMeshes);
                }
                navmesh::core::TagCandidateTriangles(*candidate, cell->navMeshes, cell->waterHeight,
                                                     options.tagTriangles);
            }
            catch (const std::exception &error)
            {
                std::cerr << "Candidate generation failed: " << error.what() << "\n";
                return 2;
            }
            const auto jsonPath = options.output / "candidate-navm.json";
            const auto objPath = options.output / "candidate-navm.obj";
            update(87, "Writing candidate exports");
            if (!navmesh::core::WriteCandidateJson(jsonPath, *candidate, geometry.scene,
                                                   navmesh::reproducibility::ToJson(metadata, "    ")) ||
                !navmesh::core::WriteCandidateObj(objPath, *candidate) ||
                !navmesh::reproducibility::WriteSidecar(objPath, metadata))
            {
                std::cerr << "Failed to write neutral candidate exports.\n";
                return 2;
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
            std::cerr << "Failed to write navmesh-counts.txt.\n";
            return 2;
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
                std::cerr << "--scene-bounds requires minX minY maxX maxY.\n";
                return 1;
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
            std::cerr << "Failed to write the inspection scene or its provenance sidecar.\n";
            return 2;
        }
        if (!navmesh::reproducibility::WriteSidecar(scenePath, metadata))
        {
            std::cerr << "Failed to write GLB metadata sidecar\n";
            return 2;
        }
        std::cout << std::format(
            "Exported combined GLB scene to {} ({} objects, {} triangles, {} geometry triangles culled)\n",
            scenePath.string(), sceneExport.objects, sceneExport.triangles, sceneExport.culledTriangles);
        if (candidate && !candidate->topology.valid)
        {
            std::cerr << "Candidate topology validation failed; see candidate-navm.json.\n";
            return 2;
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
                std::cerr << "Cannot write generation-report.json.\n";
                return 2;
            }
            std::cout << "No generated component reaches a matched door or neighboring NAVM; plugin writing skipped.\n";
        }
        else if (options.generatePlugin)
        {
            std::vector<std::filesystem::path> inputPaths;
            if (mo2Input)
            {
                inputPaths = mo2Input->pluginPaths;
            }
            else
            {
                for (auto path : navmesh::skyrim::ReadLoadOrderManifest(options.loadOrder))
                {
                    inputPaths.push_back(path.is_absolute() ? path : options.data / path);
                }
            }
            std::filesystem::path pluginPath;
            std::string error;
            update(89, "Writing NAVM override plugin");
            if (!navmesh::skyrim::WriteNavmeshOverride(options.output, inputPaths, *resolved, *cell, *candidate,
                                                       pluginPath, error))
            {
                std::cerr << "Plugin generation failed: " << error << "\n";
                update(89, std::format("Plugin generation failed: {}", error));
                return 2;
            }
            std::ifstream writtenPlugin(pluginPath, std::ios::binary);
            std::array<unsigned char, 12> pluginHeader{};
            writtenPlugin.read(reinterpret_cast<char *>(pluginHeader.data()),
                               static_cast<std::streamsize>(pluginHeader.size()));
            const bool eslFlagged = writtenPlugin && (pluginHeader[9] & 0x02U) != 0;
            std::cout << "Generated NAVM override plugin: " << pluginPath.string()
                      << (eslFlagged ? " (ESL-flagged ESP)\n" : " (regular ESP)\n");
            std::cerr
                << "Generated NAVM includes any matched border and door links; NAVI, teleport-door XNDP, cover, and "
                   "other authored links still need independent validation.\n";
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
                std::cerr << "Failed to write analysis metadata sidecar\n";
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
                std::cerr << "Failed to write navmesh metadata sidecar\n";
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
                std::cerr << "Failed to write scene report metadata sidecar\n";
            }
        }
        if (options.diagnostics && !cell->navMeshes.empty())
        {
            const auto diagnosticPath = options.output / "navmesh_diagnostics.html";
            navmesh::cli::WriteDiagnosticHtml(diagnosticPath, *cell, cell->navMeshes.front(), geometry.scene.mesh,
                                              geometry, analysisReport);
            if (!navmesh::reproducibility::WriteSidecar(diagnosticPath, metadata))
            {
                std::cerr << "Failed to write diagnostic metadata sidecar\n";
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
                                 geometry.scene.mesh.triangles.size(), geometryPath.string());
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
            std::cerr << "Failed to write the selected-cell scene.\n";
            return 2;
        }
    }
    update(100, std::format("Inspected {} cells", selectedCells.size()));
    return 0;
}
