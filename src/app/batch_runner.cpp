#include "app/batch_runner.h"

#include "app/geometry_pipeline.h"
#include "app/candidate_artifacts.h"
#include "app/candidate_cache.h"
#include "app/batch_generation.h"
#include <future>
#include "skyrim/extraction/asset_cache.h"
#include <chrono>
#include "core/navmesh/generator.h"
#include "skyrim/extraction/terrain_extractor.h"
#include "skyrim/parser/affected_cells.h"
#include "skyrim/parser/plugin_writer.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <deque>
#include <format>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>

namespace
{
    [[nodiscard]] std::string JsonEscape(const std::string &value)
    {
        std::string result;
        for (const char character : value)
        {
            if (character == '\\')
            {
                result += "\\\\";
            }
            else if (character == '"')
            {
                result += "\\\"";
            }
            else if (character == '\n')
            {
                result += "\\n";
            }
            else if (character == '\r')
            {
                result += "\\r";
            }
            else
            {
                result += character;
            }
        }
        return result;
    }

    using ArchiveCounters = std::map<std::string, std::uint64_t>;

    ArchiveCounters ReadArchiveCounters(const std::filesystem::path &snapshot)
    {
        std::ifstream file(snapshot / ".archive-statistics.json");
        std::string json((std::istreambuf_iterator<char>(file)), {});
        ArchiveCounters counters;
        for (const auto *key : {"metadata_bytes_read", "payload_bytes_read", "entries_extracted", "index_hits"})
        {
            auto position = json.find(std::string("\"") + key + "\"");
            if (position != std::string::npos)
            {
                position = json.find(':', position);
                if (position != std::string::npos)
                {
                    position = json.find_first_not_of(" \t\r\n", position + 1);
                    if (position != std::string::npos)
                    {
                        std::uint64_t value{};
                        if (std::from_chars(json.data() + position, json.data() + json.size(), value).ec == std::errc{})
                        {
                            counters[key] = value;
                        }
                    }
                }
            }
        }
        return counters;
    }

    [[nodiscard]] bool EqualsIgnoreCase(const std::string &left, const std::string &right)
    {
        return left.size() == right.size() &&
               std::equal(left.begin(), left.end(), right.begin(),
                          [](unsigned char a, unsigned char b) { return std::tolower(a) == std::tolower(b); });
    }

} // namespace

namespace navmesh::app::detail
{
    /// Extract in target order and admit independent generation tasks within an estimated byte budget.
    /// Full scene meshes are owned only by extraction caches and in-flight tasks.
    int RunBatch(const navmesh::app::Options &options, const navmesh::skyrim::offline::ResolvedLoadOrder &resolved,
                 const navmesh::skyrim::offline::ModelAssetSources *assets,
                 const std::vector<std::filesystem::path> &paths, const navmesh::app::ProgressCallback &progress,
                 const navmesh::app::CancellationCallback &cancelled, double inputPreparationSeconds)
    {
        using namespace navmesh;
        const auto stop = [&] { return cancelled && cancelled(); };
        const auto started = std::chrono::steady_clock::now();
        const auto assetCache = skyrim::offline::ModelAssetCacheDirectory(options.data, assets, options.assetCache);
        struct CacheRetention
        {
            std::filesystem::path snapshot;
            std::size_t budget;
            ~CacheRetention()
            {
                try
                {
                    skyrim::offline::TrimModelAssetCache(snapshot, budget);
                }
                catch (const std::exception &error)
                {
                    std::cerr << "Cannot finalize cache retention: " << error.what() << '\n';
                }
            }
        } retention{assetCache, options.cacheBudgetMiB * 1024ULL * 1024ULL};
        const auto initialArchiveCounters = ReadArchiveCounters(assetCache);
        skyrim::offline::ModelGeometryCache modelCache(options.workingMemoryMiB * 1024ULL * 1024ULL / 2);
        skyrim::offline::TrimModelAssetCache(assetCache, options.cacheBudgetMiB * 1024ULL * 1024ULL);
        const skyrim::offline::CellImpactIndex index(resolved);
        const auto existingNavmeshCells = skyrim::offline::CellsWithExistingNavmesh(resolved);
        // Associate asset winners with the rebuild scope before selecting conservative affected targets.
        std::set<std::string> changedModels;
        bool archiveModelsChanged{};
        std::set<std::filesystem::path> changedArchives;
        if (assets)
        {
            const auto inside = [](const std::filesystem::path &path, const std::filesystem::path &root)
            {
                const auto relative = path.lexically_relative(root);
                return !relative.empty() && *relative.begin() != "..";
            };
            if (options.rebuildScope == app::RebuildScope::LoadOrder)
            {
                for (const auto &[logical, physical] : assets->looseModels)
                {
                    if (!inside(physical, options.data))
                    {
                        changedModels.insert(logical);
                    }
                }
                archiveModelsChanged = std::any_of(assets->archives.begin(), assets->archives.end(),
                                                   [&](const auto &archive) { return !inside(archive, options.data); });
            }
            else
            {
                const auto selected =
                    std::find_if(paths.begin(), paths.end(), [&](const auto &path)
                                 { return EqualsIgnoreCase(path.filename().string(), options.affectedPlugin); });
                if (selected != paths.end() && !inside(*selected, options.data))
                {
                    for (const auto &[logical, physical] : assets->looseModels)
                    {
                        if (inside(physical, selected->parent_path()))
                        {
                            changedModels.insert(logical);
                        }
                    }
                }
                const auto pluginStem = std::filesystem::path(options.affectedPlugin).stem().string();
                archiveModelsChanged =
                    std::any_of(assets->archives.begin(), assets->archives.end(),
                                [&](const auto &archive)
                                {
                                    const auto stem = archive.stem().string();
                                    return EqualsIgnoreCase(stem, pluginStem) ||
                                           (stem.size() > pluginStem.size() && stem[pluginStem.size()] == ' ' &&
                                            EqualsIgnoreCase(stem.substr(0, pluginStem.size()), pluginStem));
                                });
            }
        }
        if (archiveModelsChanged && assets)
        {
            const auto pluginStem = std::filesystem::path(options.affectedPlugin).stem().string();
            for (const auto &archive : assets->archives)
            {
                const auto relative = archive.lexically_relative(options.data);
                const auto stem = archive.stem().string();
                if ((options.rebuildScope == app::RebuildScope::LoadOrder &&
                     (relative.empty() || *relative.begin() == "..")) ||
                    (options.rebuildScope == app::RebuildScope::Plugin &&
                     (EqualsIgnoreCase(stem, pluginStem) ||
                      (stem.size() > pluginStem.size() && stem[pluginStem.size()] == ' ' &&
                       EqualsIgnoreCase(stem.substr(0, pluginStem.size()), pluginStem)))))
                {
                    changedArchives.insert(archive);
                }
            }
            archiveModelsChanged = !skyrim::offline::ChangedArchiveModels(options.data, *assets, assetCache,
                                                                          changedArchives, changedModels);
        }
        skyrim::offline::ImpactSelectionStatistics selectionStatistics;
        auto targets = index.AffectedCells(
            options.rebuildScope == app::RebuildScope::Plugin ? options.affectedPlugin : "",
            options.neighboringCellRadius, changedModels, archiveModelsChanged, &selectionStatistics);
        const auto selectedCells = targets.size();
        const auto eligible = [&](const core::Cell *cell)
        {
            const auto *record = resolved.FindWinning(cell->id);
            return (options.skipExistingNavmesh ? !existingNavmeshCells.contains(cell->id)
                                                : !cell->navMeshes.empty()) &&
                   !(record && record->raw && (record->raw->flags & 0x20U));
        };
        const auto fullEligibleCells =
            static_cast<std::size_t>(std::count_if(targets.begin(), targets.end(), eligible));
        if (options.estimateOnly)
        {
            std::map<int, std::vector<const core::Cell *>> strata;
            for (const auto *cell : targets)
            {
                if (!eligible(cell))
                {
                    continue;
                }
                const auto neighbors = index.Neighbors(*cell, 1);
                const bool authoredNeighbor =
                    !cell->isInterior &&
                    std::any_of(neighbors.begin(), neighbors.end(), [&](const auto *neighbor)
                                { return neighbor->id != cell->id && !neighbor->navMeshes.empty(); });
                strata[cell->isInterior ? 0 : authoredNeighbor ? 1 : 2].push_back(cell);
            }
            std::set<std::uint32_t> sampled;
            for (auto &[stratum, cells] : strata)
            {
                const auto density = [](const core::Cell *cell)
                {
                    std::size_t value = cell->references.size();
                    for (const auto &mesh : cell->navMeshes)
                    {
                        value += mesh.polygons.size();
                    }
                    return value;
                };
                std::stable_sort(cells.begin(), cells.end(),
                                 [&](const auto *left, const auto *right) { return density(left) < density(right); });
                sampled.insert(cells.front()->id);
                sampled.insert(cells[cells.size() / 2]->id);
                sampled.insert(cells.back()->id);
            }
            std::erase_if(targets, [&](const auto *cell) { return !sampled.contains(cell->id); });
        }
        // Retain compact candidate evidence and status for every target, including skips and failures.
        using Result = BatchCellResult;
        std::vector<Result> results;
        results.reserve(targets.size());
        for (const auto *target : targets)
        {
            results.push_back({.cell = target, .status = "pending"});
        }
        std::map<std::uint32_t, skyrim::offline::GeometryExtraction> cache;
        std::deque<std::uint32_t> cacheOrder;
        std::size_t cacheTriangles{}, cacheHits{}, extractedCells{}, originalPolygons{}, generatedPolygons{},
            completedCells{}, eligibleCells{}, candidateHits{};
        double extractionSeconds{}, generationSeconds{};
        std::size_t peakWorkers{}, peakAdmittedBytes{};
        const auto workBudget = options.workingMemoryMiB * 1024ULL * 1024ULL / 2;
        eligibleCells = fullEligibleCells;
        const auto auditDirectory =
            options.output /
            (".candidate-staging-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        struct StagingCleanup
        {
            std::filesystem::path path, output;
            ~StagingCleanup()
            {
                // Verify containment at removal time, including directory links introduced during the run.
                std::error_code error;
                const auto root = std::filesystem::weakly_canonical(output, error);
                if (error)
                {
                    return;
                }
                const auto target = std::filesystem::weakly_canonical(path, error);
                const auto relative = target.lexically_relative(root);
                if (!error && !relative.empty() && *relative.begin() != ".." && target != root &&
                    !std::filesystem::is_symlink(path, error))
                {
                    std::filesystem::remove_all(target, error);
                }
            }
        } cleanup{auditDirectory, options.output};
        const auto batchMetadata = reproducibility::ToJson(
            reproducibility::ExportMetadata{
                .inputPlugin = options.affectedPlugin,
                .warnings = {"Load-order scope treats plugins after the first active baseline plugin as changes.",
                             "Skipped targets and writer limitations are recorded in the batch report and "
                             "docs/batch-rebuilding.md."}},
            "    ");
        const auto summaryPath = options.output / "batch-report.json";
        // Rewrite the batch status report at checkpoints so failures and cancellation retain useful evidence.
        const auto summary = [&](const std::string &state, const std::string &error = "")
        {
            std::ofstream out(summaryPath, std::ios::trunc);
            out << "{\n  \"metadata\": " << batchMetadata << ",\n";
            out << std::format("  \"input_preparation_seconds\":{},\"estimate_only\":{},\"sampled_cells\":{},\n  "
                               "\"input_plugins\":[\n",
                               inputPreparationSeconds, options.estimateOnly ? "true" : "false",
                               options.estimateOnly ? targets.size() : 0);
            for (std::size_t i{}; i < paths.size(); ++i)
            {
                std::error_code inputError;
                const auto size = std::filesystem::file_size(paths[i], inputError);
                const auto stamp = std::filesystem::last_write_time(paths[i], inputError).time_since_epoch().count();
                out << std::format("    {{\"path\":\"{}\",\"bytes\":{},\"timestamp\":{}}}{}\n",
                                   JsonEscape(paths[i].generic_string()), inputError ? 0 : size, stamp,
                                   i + 1 == paths.size() ? "" : ",");
            }
            out << "  ],\n  \"input_diagnostics\":[\n";
            for (std::size_t i{}; i < resolved.diagnostics.size(); ++i)
            {
                const auto &diagnostic = resolved.diagnostics[i];
                out << std::format("    {{\"plugin\":\"{}\",\"kind\":{},\"message\":\"{}\"}}{}\n",
                                   JsonEscape(diagnostic.plugin), static_cast<int>(diagnostic.kind),
                                   JsonEscape(diagnostic.message), i + 1 == resolved.diagnostics.size() ? "" : ",");
            }
            out << "  ],\n";
            out << "  \"selection_changed_records\": {";
            bool firstCount = true;
            for (const auto &[type, count] : selectionStatistics.changedRecords)
            {
                out << std::format("{}\"{}\":{}", firstCount ? "" : ",", JsonEscape(type), count);
                firstCount = false;
            }
            out << std::format("}},\n  "
                               "\"selection_equivalent_records\":{},\"selection_base_object_uses\":{},\"selection_"
                               "asset_uses\":{},\"archive_impact_fallback\":{},\n",
                               selectionStatistics.equivalentRecords, selectionStatistics.baseObjectUses,
                               selectionStatistics.assetUses, archiveModelsChanged ? "true" : "false");
            out << "  \"archive_io\": {";
            bool firstArchiveCount = true;
            for (const auto &[key, count] : ReadArchiveCounters(assetCache))
            {
                const auto initial = initialArchiveCounters.contains(key) ? initialArchiveCounters.at(key) : 0;
                out << std::format("{}\"{}\":{}", firstArchiveCount ? "" : ",", key,
                                   count >= initial ? count - initial : count);
                firstArchiveCount = false;
            }
            out << "},\n  \"artifact_bytes\": {";
            std::map<std::string, std::uintmax_t> artifactBytes;
            if (state != "running")
            {
                std::error_code sizeError;
                for (auto iterator = std::filesystem::recursive_directory_iterator(
                         options.output, std::filesystem::directory_options::skip_permission_denied, sizeError);
                     !sizeError && iterator != std::filesystem::recursive_directory_iterator();
                     iterator.increment(sizeError))
                {
                    if (iterator->is_symlink(sizeError))
                    {
                        iterator.disable_recursion_pending();
                        continue;
                    }
                    if (iterator->is_directory(sizeError) &&
                        iterator->path().filename().string().starts_with(".candidate-staging-"))
                    {
                        iterator.disable_recursion_pending();
                        continue;
                    }
                    if (iterator->is_regular_file(sizeError) && iterator->path() != summaryPath)
                    {
                        artifactBytes[iterator->path().extension().string()] += iterator->file_size(sizeError);
                    }
                }
            }
            bool firstArtifact = true;
            for (const auto &[type, count] : artifactBytes)
            {
                out << std::format("{}\"{}\":{}", firstArtifact ? "" : ",", JsonEscape(type), count);
                firstArtifact = false;
            }
            out << "},\n";
            out << "  \"skip_existing_navmesh\":" << (options.skipExistingNavmesh ? "true" : "false") << ",\n";
            out << "  \"copy_plugin\":" << (options.copyPlugin ? "true" : "false") << ",\n";
            out << std::format("  \"scope\":\"{}\",\"plugin\":\"{}\",\"status\":\"{}\",\"error\":\"{}\",\n",
                               options.rebuildScope == app::RebuildScope::Plugin ? "plugin" : "load_order",
                               JsonEscape(options.affectedPlugin), state, JsonEscape(error));
            out << std::format("  \"selected_cells\":{},\"geometry_cells_extracted\":{},\"geometry_cache_hits\":{},\n  "
                               "\"original_polygons\":{},\"generated_polygons\":{},\n  \"cells\":[\n",
                               selectedCells, extractedCells, cacheHits, originalPolygons, generatedPolygons);
            for (std::size_t i{}; i < results.size(); ++i)
            {
                out << std::format("    "
                                   "{{\"form_id\":\"{:08X}\",\"status\":\"{}\",\"polygons\":{},\"supplier_cells\":{},"
                                   "\"references\":{},\"unique_models\":{},\"extraction_seconds\":{},\"generation_"
                                   "seconds\":{},\"candidate_reused\":{}}}{}\n",
                                   results[i].cell->id, JsonEscape(results[i].status),
                                   results[i].candidate.mesh.polygons.size(), results[i].supplierCells,
                                   results[i].references, results[i].models, results[i].extractionSeconds,
                                   results[i].generationSeconds, results[i].reused ? "true" : "false",
                                   i + 1 == results.size() ? "" : ",");
            }
            const auto stats = modelCache.Statistics();
            const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            const auto remaining =
                completedCells ? elapsed * (eligibleCells - std::min(eligibleCells, completedCells)) / completedCells
                               : 0.0;
            out << std::format(
                "  ],\n  \"workers_peak\":{},\"admitted_work_bytes_peak\":{},\n  "
                "\"eligible_cells\":{},\"completed_cells\":{},\"candidate_cache_hits\":{},\n"
                "  "
                "\"batch_output\":\"{}\",\"elapsed_seconds\":{},\"extraction_seconds\":{},\"generation_seconds\":{},\n"
                "  "
                "\"models_decoded\":{},\"model_cache_hits\":{},\"placements_built\":{},\"placement_cache_hits\":{},"
                "\"model_cache_bytes\":{},\n"
                "  \"estimated_remaining_seconds\":{},\"estimate_range_seconds\":[{},{}],\"estimate_basis\":\"elapsed "
                "time per completed eligible cell; heuristic range, excludes final export and writer; heterogeneous "
                "cells may vary\"\n}}\n",
                peakWorkers, peakAdmittedBytes, eligibleCells, completedCells, candidateHits, options.batchOutput,
                elapsed, extractionSeconds, generationSeconds, stats.modelsDecoded, stats.modelHits,
                stats.placementsBuilt, stats.placementHits, stats.retainedBytes, remaining, remaining * 0.5,
                remaining * 2.0);
            return out.good();
        };
        // Export only targets with completed metadata; failure handling uses the same partial-export path.
        const auto exports = [&]
        {
            if (options.batchOutput == "plugin_only")
            {
                return true;
            }
            for (const auto &result : results)
            {
                if (!result.metadata.empty())
                {
                    const auto directory = options.output / "cells" / std::format("{:08X}", result.cell->id);
                    std::filesystem::create_directories(directory);
                    core::CandidateNavMesh audit;
                    core::Scene evidence;
                    if (!LoadCandidate(result.auditPath, audit, evidence))
                    {
                        return false;
                    }
                    audit.borderLinks = result.candidate.borderLinks;
                    if (options.batchOutput == "compact")
                    {
                        if (!WriteCompressedCandidateJson(directory / "candidate-navm.json.gz", audit, evidence,
                                                          result.metadata))
                        {
                            return false;
                        }
                        continue;
                    }
                    if (!core::WriteCandidateJson(directory / "candidate-navm.json", audit, evidence,
                                                  result.metadata) ||
                        !core::WriteCandidateObj(directory / "candidate-navm.obj", audit))
                    {
                        return false;
                    }
                    std::ofstream sidecar(directory / "candidate-navm.obj.metadata.json", std::ios::trunc);
                    sidecar << "{\n  \"metadata\": " << result.metadata << "\n}\n";
                    if (!sidecar)
                    {
                        return false;
                    }
                }
            }
            return true;
        };
        const auto fail = [&](const std::string &error)
        {
            exports();
            summary("failed", error);
            std::cerr << error << '\n';
            return 2;
        };
        if (!summary("running"))
        {
            return fail("Cannot write batch-report.json");
        }
        struct Pending
        {
            std::size_t target{}, estimatedBytes{};
            std::future<Result> result;
        };
        std::deque<Pending> pending;
        std::size_t pendingBytes{};
        const auto collect = [&]() -> int
        {
            auto job = std::move(pending.front());
            pending.pop_front();
            pendingBytes -= job.estimatedBytes;
            auto &result = results[job.target];
            result = job.result.get();
            if (!result.error.empty())
            {
                return fail(result.error);
            }
            ++completedCells;
            candidateHits += result.reused ? 1 : 0;
            generatedPolygons += result.candidate.mesh.polygons.size();
            extractionSeconds += result.extractionSeconds;
            generationSeconds += result.generationSeconds;
            skyrim::offline::TrimModelAssetCache(assetCache, options.cacheBudgetMiB * 1024ULL * 1024ULL, true);
            if (!summary("running"))
            {
                return fail("Cannot checkpoint batch progress");
            }
            if (stop())
            {
                summary("cancelled");
                return 3;
            }
            return 0;
        };
        // Process targets independently while reusing source-cell geometry across overlapping neighborhoods.
        for (std::size_t targetIndex{}; targetIndex < targets.size(); ++targetIndex)
        {
            if (stop())
            {
                summary("cancelled");
                return 3;
            }
            const auto &cell = *targets[targetIndex];
            auto &result = results[targetIndex];
            for (const auto &mesh : cell.navMeshes)
            {
                originalPolygons += mesh.polygons.size();
            }
            if (options.skipExistingNavmesh && existingNavmeshCells.contains(cell.id))
            {
                result.status = "skipped_existing_navm";
                continue;
            }
            if (!options.skipExistingNavmesh && cell.navMeshes.empty())
            {
                result.status = "skipped_no_existing_navm";
                continue;
            }
            if (const auto *record = resolved.FindWinning(cell.id);
                record && record->raw && (record->raw->flags & 0x20U))
            {
                result.status = "skipped_deleted_cell";
                continue;
            }
            const auto update = [&](std::string_view status)
            {
                if (progress)
                {
                    progress(30 + static_cast<int>(60 * targetIndex / std::max<std::size_t>(1, targets.size())),
                             std::format("CELL {}/{} {:08X}: {}", targetIndex + 1, targets.size(), cell.id, status));
                }
            };
            update("Extracting neighboring geometry");
            const auto extractionStarted = std::chrono::steady_clock::now();
            std::optional<core::AABB> bounds;
            if (cell.exteriorCoordinates)
            {
                const auto [x, y] = *cell.exteriorCoordinates;
                bounds = core::AABB{.min = {x * 4096.0F, y * 4096.0F, std::numeric_limits<float>::lowest()},
                                    .max = {(static_cast<float>(x) + 1) * 4096.0F,
                                            (static_cast<float>(y) + 1) * 4096.0F, std::numeric_limits<float>::max()}};
            }
            skyrim::offline::GeometryExtraction geometry;
            const auto localNeighbors = index.Neighbors(cell, std::max(1, options.neighboringCellRadius));
            std::set<std::uint32_t> terrainCells;
            for (const auto *neighbor : localNeighbors)
            {
                terrainCells.insert(neighbor->id);
            }
            // Winning bounds filter placements before NIF requests; unknown bounds remain conservative suppliers.
            const auto suppliers = index.GeometryNeighbors(cell, std::max(1, options.neighboringCellRadius));
            result.supplierCells = suppliers.size();
            std::set<std::string> uniqueModels;
            for (const auto *neighbor : suppliers)
            {
                auto geometryCell = index.GeometryCell(*neighbor, bounds);
                result.references += geometryCell.references.size();
                for (const auto &reference : geometryCell.references)
                {
                    if (!reference.modelPath.empty() && !reference.deleted && !reference.initiallyDisabled)
                    {
                        uniqueModels.insert(reference.modelPath);
                    }
                }
                auto extracted =
                    options.terrainOnly
                        ? skyrim::offline::GeometryExtraction{}
                        : skyrim::offline::ExtractGeometry(
                              options.data, geometryCell, assetCache, [&](std::size_t done, std::size_t total)
                              { update(std::format("Geometry {:08X}: reference {}/{}", neighbor->id, done, total)); },
                              stop, assets, &modelCache, true);
                if (stop())
                {
                    result.status = "cancelled";
                    summary("cancelled");
                    return 3;
                }
                // Distant placement suppliers do not supply their unrelated LAND surface.
                if (terrainCells.contains(neighbor->id))
                {
                    auto found = cache.find(neighbor->id);
                    if (found == cache.end())
                    {
                        auto terrain = skyrim::offline::ExtractTerrain(resolved, *neighbor);
                        skyrim::offline::GeometryExtraction terrainGeometry;
                        terrainGeometry.scene = std::move(terrain.scene);
                        terrainGeometry.terrainSupported = terrain.landRecordsDecoded != 0;
                        terrainGeometry.terrainLandRecords = terrain.landRecordsFound;
                        terrainGeometry.terrainLandDecoded = terrain.landRecordsDecoded;
                        terrainGeometry.terrainLandMissing = terrain.landRecordsMissing;
                        cacheTriangles += terrainGeometry.scene.mesh.triangles.size();
                        found = cache.emplace(neighbor->id, std::move(terrainGeometry)).first;
                        cacheOrder.push_back(neighbor->id);
                        ++extractedCells;
                    }
                    else
                    {
                        ++cacheHits;
                        std::erase(cacheOrder, neighbor->id);
                        cacheOrder.push_back(neighbor->id);
                    }
                    AppendGeometry(extracted, found->second);
                }
                AppendGeometry(geometry, extracted);
                while (cache.size() > 16 || cacheTriangles > 2000000)
                {
                    const auto id = cacheOrder.front();
                    cacheOrder.pop_front();
                    cacheTriangles -= cache.at(id).scene.mesh.triangles.size();
                    cache.erase(id);
                }
            }
            result.models = uniqueModels.size();
            result.extractionSeconds =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - extractionStarted).count();
            std::vector<core::CandidateExit> exits;
            // Physical bucketing includes DOORs from persistent worldspace parents.
            for (const auto &reference : index.GeometryCell(cell).references)
            {
                if (reference.baseRecordType == "DOOR" && !reference.deleted && !reference.initiallyDisabled)
                {
                    exits.push_back({.referenceId = reference.id, .position = reference.position});
                }
            }
            update("Generating NAVM");
            std::vector<core::NavMesh> adjacent;
            if (bounds)
            {
                for (const auto *neighbor : index.Neighbors(cell, 1))
                {
                    if (!neighbor->exteriorCoordinates || neighbor->id == cell.id)
                    {
                        continue;
                    }
                    const auto [x, y] = *neighbor->exteriorCoordinates;
                    if (std::abs(static_cast<std::int64_t>(x) - (*cell.exteriorCoordinates)[0]) +
                            std::abs(static_cast<std::int64_t>(y) - (*cell.exteriorCoordinates)[1]) ==
                        1)
                    {
                        adjacent.insert(adjacent.end(), neighbor->navMeshes.begin(), neighbor->navMeshes.end());
                    }
                }
            }
            const auto estimatedBytes = EstimateGenerationBytes(geometry.scene, bounds);
            while (!pending.empty() && (pending.size() >= options.workers ||
                                        estimatedBytes > workBudget - std::min(workBudget, pendingBytes)))
            {
                if (const auto status = collect(); status != 0)
                {
                    return status;
                }
            }
            // Extraction remains ordered. Workers receive isolated scenes; final results publish in target order.
            BatchGenerationInput input{.result = result,
                                       .geometry = std::move(geometry),
                                       .bounds = bounds,
                                       .exits = std::move(exits),
                                       .adjacent = std::move(adjacent),
                                       .cacheDirectory = assetCache,
                                       .stagingDirectory = auditDirectory};
            pendingBytes += estimatedBytes;
            peakAdmittedBytes = std::max(peakAdmittedBytes, pendingBytes);
            peakWorkers = std::max(peakWorkers, pending.size() + 1);
            pending.push_back({targetIndex, estimatedBytes,
                               std::async(std::launch::async, [input = std::move(input), &options]() mutable
                                          { return BuildBatchCandidate(std::move(input), options); })});
            // Leave capacity for the next extraction and avoid starting unnecessary threads in single-worker mode.
            if (pending.size() >= options.workers)
            {
                if (const auto status = collect(); status != 0)
                {
                    return status;
                }
            }
        }
        while (!pending.empty())
        {
            if (const auto status = collect(); status != 0)
            {
                return status;
            }
        }
        if (options.estimateOnly)
        {
            if (!exports())
            {
                return fail("Cannot write cost sample exports");
            }
            skyrim::offline::TrimModelAssetCache(assetCache, options.cacheBudgetMiB * 1024ULL * 1024ULL);
            if (!summary("estimated"))
            {
                return fail("Cannot finalize cost estimate");
            }
            if (progress)
            {
                progress(100, std::format("Cost sample complete: {} of {} eligible targets; see batch-report.json",
                                          completedCells, eligibleCells));
            }
            return 0;
        }
        // Replace authored neighbor triangle identities with reciprocal generated
        // edges. Endpoint equality is required; incompatible partitions fail closed.
        std::map<std::uint32_t, Result *> generatedByCell;
        for (auto &result : results)
        {
            if (result.status == "generated")
            {
                generatedByCell.emplace(result.cell->id, &result);
            }
        }
        const auto near = [](core::Vec3 a, core::Vec3 b)
        { return std::abs(a.x - b.x) <= 1 && std::abs(a.y - b.y) <= 1 && std::abs(a.z - b.z) <= 1; };
        for (auto &result : results)
        {
            for (auto &link : result.candidate.borderLinks)
            {
                const auto *record = resolved.FindWinning(link.neighborNavmeshId);
                if (!record || !record->cellFormId)
                {
                    return fail("Border target has no CELL ownership");
                }
                const auto other = generatedByCell.find(*record->cellFormId);
                if (other == generatedByCell.end())
                {
                    continue;
                }
                const auto &face = result.candidate.mesh.polygons.at(link.polygon);
                const auto a = result.candidate.mesh.vertices.at(face.vertices[link.edge]);
                const auto b = result.candidate.mesh.vertices.at(face.vertices[(link.edge + 1) % 3]);
                bool matched{};
                for (const auto &reverse : other->second->candidate.borderLinks)
                {
                    const auto &target = other->second->candidate.mesh.polygons.at(reverse.polygon);
                    const auto c = other->second->candidate.mesh.vertices.at(target.vertices[reverse.edge]);
                    const auto d = other->second->candidate.mesh.vertices.at(target.vertices[(reverse.edge + 1) % 3]);
                    if (!near(a, d) || !near(b, c))
                    {
                        continue;
                    }
                    const auto &meshes = other->second->cell->navMeshes;
                    link.neighborNavmeshId =
                        std::max_element(meshes.begin(), meshes.end(), [](const auto &x, const auto &y)
                                         { return x.polygons.size() < y.polygons.size(); })
                            ->id;
                    link.neighborPolygon = reverse.polygon;
                    link.neighborEdge = reverse.edge;
                    matched = true;
                    break;
                }
                if (!matched)
                {
                    return fail(std::format("Generated border partitions do not match between CELL {:08X} and {:08X}",
                                            result.cell->id, other->second->cell->id));
                }
            }
        }
        if (!exports())
        {
            return fail("Cannot write batch candidate exports");
        }
        if (stop())
        {
            summary("cancelled");
            return 3;
        }
        // Write one combined override only after all generated border targets and exports are ready.
        if (options.generatePlugin && !generatedByCell.empty())
        {
            std::vector<skyrim::offline::NavmeshReplacement> replacements;
            for (const auto &result : results)
            {
                if (result.status == "generated")
                {
                    replacements.push_back({result.cell, &result.candidate});
                }
            }
            std::filesystem::path written;
            std::string error;
            if (progress)
            {
                progress(92,
                         options.copyPlugin ? "Writing selected plugin copy" : "Writing batch NAVM override plugin");
            }
            if (!skyrim::offline::WriteNavmeshOverrides(options.output, paths, resolved, replacements, written, error,
                                                        options.copyPlugin ? options.affectedPlugin : ""))
            {
                return fail(error);
            }
            std::cout << (options.copyPlugin ? "Generated plugin copy: " : "Generated batch NAVM override plugin: ")
                      << written.string() << '\n';
        }
        skyrim::offline::TrimModelAssetCache(assetCache, options.cacheBudgetMiB * 1024ULL * 1024ULL);
        if (!summary("complete"))
        {
            return fail("Cannot finalize batch-report.json");
        }
        const auto status =
            std::format("Affected cells: {}; rebuilt: {}; skipped: {}; original polygons: {}; generated polygons: {}",
                        targets.size(), generatedByCell.size(), targets.size() - generatedByCell.size(),
                        originalPolygons, generatedPolygons);
        std::cout << status << '\n';
        if (progress)
        {
            progress(100, status);
        }
        return 0;
    }
} // namespace navmesh::app::detail
