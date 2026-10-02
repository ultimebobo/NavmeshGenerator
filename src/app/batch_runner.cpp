#include "app/batch_runner.h"

#include "app/geometry_pipeline.h"
#include "core/navmesh/generator.h"
#include "skyrim/extraction/terrain_extractor.h"
#include "skyrim/parser/affected_cells.h"
#include "skyrim/parser/plugin_writer.h"

#include <algorithm>
#include <cctype>
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

    [[nodiscard]] bool EqualsIgnoreCase(const std::string &left, const std::string &right)
    {
        return left.size() == right.size() &&
               std::equal(left.begin(), left.end(), right.begin(),
                          [](unsigned char a, unsigned char b) { return std::tolower(a) == std::tolower(b); });
    }

} // namespace

namespace navmesh::app::detail
{
    /// Generate one CELL at a time, retaining only candidate evidence and a bounded
    /// geometry cache. Full scene meshes never accumulate across the load order.
    int RunBatch(const navmesh::app::Options &options, const navmesh::skyrim::offline::ResolvedLoadOrder &resolved,
                 const navmesh::skyrim::offline::ModelAssetSources *assets,
                 const std::vector<std::filesystem::path> &paths, const navmesh::app::ProgressCallback &progress,
                 const navmesh::app::CancellationCallback &cancelled)
    {
        using namespace navmesh;
        const auto stop = [&] { return cancelled && cancelled(); };
        const skyrim::offline::CellImpactIndex index(resolved);
        const auto existingNavmeshCells = skyrim::offline::CellsWithExistingNavmesh(resolved);
        // Associate asset winners with the rebuild scope before selecting conservative affected targets.
        std::set<std::string> changedModels;
        bool archiveModelsChanged{};
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
        const auto targets =
            index.AffectedCells(options.rebuildScope == app::RebuildScope::Plugin ? options.affectedPlugin : "",
                                options.neighboringCellRadius, changedModels, archiveModelsChanged);
        // Retain compact candidate evidence and status for every target, including skips and failures.
        struct Result
        {
            const core::Cell *cell{};
            core::CandidateNavMesh candidate;
            core::Scene evidence;
            std::string metadata, status;
        };
        std::vector<Result> results;
        results.reserve(targets.size());
        for (const auto *target : targets)
        {
            results.push_back({.cell = target, .status = "pending"});
        }
        std::map<std::uint32_t, skyrim::offline::GeometryExtraction> cache;
        std::deque<std::uint32_t> cacheOrder;
        std::size_t cacheTriangles{}, cacheHits{}, extractedCells{}, originalPolygons{}, generatedPolygons{};
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
            out << "  \"skip_existing_navmesh\":" << (options.skipExistingNavmesh ? "true" : "false") << ",\n";
            out << "  \"copy_plugin\":" << (options.copyPlugin ? "true" : "false") << ",\n";
            out << std::format("  \"scope\":\"{}\",\"plugin\":\"{}\",\"status\":\"{}\",\"error\":\"{}\",\n",
                               options.rebuildScope == app::RebuildScope::Plugin ? "plugin" : "load_order",
                               JsonEscape(options.affectedPlugin), state, JsonEscape(error));
            out << std::format("  \"selected_cells\":{},\"geometry_cells_extracted\":{},\"geometry_cache_hits\":{},\n  "
                               "\"original_polygons\":{},\"generated_polygons\":{},\n  \"cells\":[\n",
                               targets.size(), extractedCells, cacheHits, originalPolygons, generatedPolygons);
            for (std::size_t i{}; i < results.size(); ++i)
            {
                out << std::format("    {{\"form_id\":\"{:08X}\",\"status\":\"{}\",\"polygons\":{}}}{}\n",
                                   results[i].cell->id, JsonEscape(results[i].status),
                                   results[i].candidate.mesh.polygons.size(), i + 1 == results.size() ? "" : ",");
            }
            out << "  ]\n}\n";
            return out.good();
        };
        // Export only targets with completed metadata; failure handling uses the same partial-export path.
        const auto exports = [&]
        {
            for (const auto &result : results)
            {
                if (!result.metadata.empty())
                {
                    const auto directory = options.output / "cells" / std::format("{:08X}", result.cell->id);
                    std::filesystem::create_directories(directory);
                    if (!core::WriteCandidateJson(directory / "candidate-navm.json", result.candidate, result.evidence,
                                                  result.metadata) ||
                        !core::WriteCandidateObj(directory / "candidate-navm.obj", result.candidate))
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
            skyrim::offline::GeometryExtraction geometry;
            for (const auto *neighbor : index.GeometryNeighbors(cell, std::max(1, options.neighboringCellRadius)))
            {
                auto found = cache.find(neighbor->id);
                if (found == cache.end())
                {
                    auto geometryCell = index.GeometryCell(*neighbor);
                    auto extracted = options.terrainOnly
                                         ? skyrim::offline::GeometryExtraction{}
                                         : skyrim::offline::ExtractGeometry(
                                               options.data, geometryCell, options.output / ".bsa-cache",
                                               [&](std::size_t done, std::size_t total)
                                               {
                                                   update(std::format("Geometry {:08X}: reference {}/{}", neighbor->id,
                                                                      done, total));
                                               },
                                               stop, assets);
                    if (stop())
                    {
                        result.status = "cancelled";
                        summary("cancelled");
                        return 3;
                    }
                    auto terrain = skyrim::offline::ExtractTerrain(resolved, *neighbor);
                    skyrim::offline::GeometryExtraction terrainGeometry;
                    terrainGeometry.scene = std::move(terrain.scene);
                    terrainGeometry.mesh = std::move(terrain.mesh);
                    terrainGeometry.terrainSupported = terrain.landRecordsDecoded != 0;
                    terrainGeometry.terrainLandRecords = terrain.landRecordsFound;
                    terrainGeometry.terrainLandDecoded = terrain.landRecordsDecoded;
                    terrainGeometry.terrainLandMissing = terrain.landRecordsMissing;
                    AppendGeometry(extracted, std::move(terrainGeometry));
                    cacheTriangles +=
                        extracted.mesh.triangles.size() + extracted.scene.renderFallbackMesh.triangles.size();
                    found = cache.emplace(neighbor->id, std::move(extracted)).first;
                    cacheOrder.push_back(neighbor->id);
                    ++extractedCells;
                }
                else
                {
                    ++cacheHits;
                    std::erase(cacheOrder, neighbor->id);
                    cacheOrder.push_back(neighbor->id);
                }
                AppendGeometry(geometry, skyrim::offline::GeometryExtraction(found->second));
                // LRU is bounded both by entry count and triangle volume. A single
                // oversized entry may be used for this target but is not retained.
                while (cache.size() > 16 || cacheTriangles > 2000000)
                {
                    const auto id = cacheOrder.front();
                    cacheOrder.pop_front();
                    const auto &entry = cache.at(id);
                    cacheTriangles -= entry.mesh.triangles.size() + entry.scene.renderFallbackMesh.triangles.size();
                    cache.erase(id);
                }
            }
            // Clip generated navigation to this target, while retaining neighboring geometry as input evidence.
            std::optional<core::AABB> bounds;
            if (cell.exteriorCoordinates)
            {
                const auto [x, y] = *cell.exteriorCoordinates;
                bounds = core::AABB{.min = {x * 4096.0F, y * 4096.0F, std::numeric_limits<float>::lowest()},
                                    .max = {(static_cast<float>(x) + 1) * 4096.0F,
                                            (static_cast<float>(y) + 1) * 4096.0F, std::numeric_limits<float>::max()}};
            }
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
            // Build the neutral candidate and resolve authored neighbor portals before retaining its evidence.
            try
            {
                result.candidate = core::RecastCandidateGenerator{}.Generate(
                    geometry.scene, core::NavigationProfile{}, bounds, std::move(exits), options.partitioningAlgorithm);
                if (bounds)
                {
                    std::vector<core::NavMesh> adjacent;
                    for (const auto *neighbor : index.Neighbors(cell, 1))
                    {
                        if (!neighbor->exteriorCoordinates || neighbor->id == cell.id)
                        {
                            continue;
                        }
                        const auto [x, y] = *neighbor->exteriorCoordinates;
                        if (std::abs(static_cast<std::int64_t>(x) - (*cell.exteriorCoordinates)[0]) +
                                std::abs(static_cast<std::int64_t>(y) - (*cell.exteriorCoordinates)[1]) !=
                            1)
                        {
                            continue;
                        }
                        adjacent.insert(adjacent.end(), neighbor->navMeshes.begin(), neighbor->navMeshes.end());
                    }
                    (void)core::StitchCandidateBorders(result.candidate, *bounds, adjacent,
                                                       !options.skipExistingNavmesh);
                }
            }
            catch (const std::exception &error)
            {
                result.status = "failed";
                return fail(std::format("CELL {:08X}: {}", cell.id, error.what()));
            }
            if (!result.candidate.topology.valid)
            {
                result.status = "invalid_topology";
                return fail("Candidate topology validation failed");
            }
            result.status = result.candidate.mesh.polygons.empty() ? "skipped_empty_candidate" : "generated";
            generatedPolygons += result.candidate.mesh.polygons.size();
            reproducibility::ExportMetadata metadata{
                .inputPlugin = options.affectedPlugin,
                .selectedCell = &cell,
                .coverage = {.references = cell.references.size(),
                             .geometryVertices = geometry.mesh.vertices.size(),
                             .geometryTriangles = geometry.mesh.triangles.size(),
                             .terrainSupported = geometry.terrainSupported,
                             .collisionGeometrySupported = geometry.collisionModelsLoaded != 0},
                .warnings = {"Batch candidates use neighboring geometry but remain clipped to their target CELL.",
                             "Generation eligibility and empty candidates are recorded in "
                             "batch-report.json."}};
            result.metadata = reproducibility::ToJson(metadata, "    ");
            // Compact source evidence after generation; retain no scene mesh and
            // only provenance entries actually used by candidate polygons.
            result.evidence.geometrySources = std::move(geometry.scene.geometrySources);
            std::map<std::size_t, std::size_t> remap;
            const auto source = [&](std::size_t old)
            {
                const auto [it, added] = remap.emplace(old, result.evidence.triangleProvenance.size());
                if (added)
                {
                    result.evidence.triangleProvenance.push_back(geometry.scene.triangleProvenance.at(old));
                }
                return it->second;
            };
            for (auto &value : result.candidate.polygonSourceTriangles)
            {
                value = source(value);
            }
            for (auto &values : result.candidate.polygonContributingTriangles)
            {
                for (auto &value : values)
                {
                    value = source(value);
                }
            }
            for (auto &region : result.candidate.regions)
            {
                for (auto &value : region.sourceTriangles)
                {
                    value = source(value);
                }
            }
            if (stop())
            {
                result.status = "cancelled";
                summary("cancelled");
                return 3;
            }
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
