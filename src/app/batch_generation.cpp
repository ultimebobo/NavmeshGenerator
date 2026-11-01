#include "app/batch_generation.h"

#include "app/candidate_cache.h"
#include "core/io/file_path.h"
#include "core/navmesh/batch_stitching.h"
#include "core/navmesh/candidate_reachability.h"
#include "core/navmesh/generator.h"
#include "core/navmesh/surface_simplification.h"
#include "core/navmesh/triangle_tagging.h"
#include "core/navmesh/cover.h"
#include "core/navmesh/ledge_links.h"
#include "core/reproducibility/export_metadata.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <format>
#include <map>
#include <limits>
#include <new>
#include <set>

namespace navmesh::app::detail
{
    namespace
    {
        /// Borrow winning meshes for existing authored portals; keys remain in resolved FormID space.
        std::map<std::uint32_t, const core::NavMesh *> IndexAuthoredPortalTargets(
            const std::vector<core::GeneratedCellCandidate> &exterior, const skyrim::ResolvedLoadOrder &resolved)
        {
            std::map<std::uint32_t, const core::NavMesh *> targets;
            for (const auto &target : exterior)
            {
                for (const auto &link : target.candidate->borderLinks)
                {
                    if (!link.generatedNeighborCell)
                    {
                        targets.emplace(link.neighborNavmeshId, nullptr);
                    }
                }
            }
            if (targets.empty())
            {
                return targets;
            }
            for (const auto &cell : resolved.cells)
            {
                for (const auto &mesh : cell.navMeshes)
                {
                    const auto found = targets.find(mesh.id);
                    if (found != targets.end())
                    {
                        found->second = &mesh;
                    }
                }
            }
            return targets;
        }

        /// Compact both joins together so polygon and region indices remain valid in the spooled scene.
        core::Scene CompactEvidence(core::CandidateNavMesh &candidate, const core::Scene &scene)
        {
            core::Scene evidence;
            std::map<std::size_t, std::size_t> sourceRemap, triangleRemap;
            const auto geometrySource = [&](std::size_t old)
            {
                const auto [it, added] = sourceRemap.emplace(old, evidence.geometrySources.size());
                if (added)
                {
                    evidence.geometrySources.push_back(scene.geometrySources.at(old));
                }
                return it->second;
            };
            const auto triangleSource = [&](std::size_t old)
            {
                const auto [it, added] = triangleRemap.emplace(old, evidence.triangleProvenance.size());
                if (added)
                {
                    auto triangle = scene.triangleProvenance.at(old);
                    triangle.geometrySource = geometrySource(triangle.geometrySource);
                    evidence.triangleProvenance.push_back(triangle);
                }
                return it->second;
            };
            for (auto &value : candidate.polygonSourceTriangles)
            {
                value = triangleSource(value);
            }
            for (auto &values : candidate.polygonContributingTriangles)
            {
                for (auto &value : values)
                {
                    value = triangleSource(value);
                }
            }
            for (auto &region : candidate.regions)
            {
                for (auto &value : region.sourceTriangles)
                {
                    value = triangleSource(value);
                }
                for (auto &value : region.geometrySources)
                {
                    value = geometrySource(value);
                }
            }
            return evidence;
        }

        std::filesystem::path PinAudit(const std::filesystem::path &source, const std::filesystem::path &directory,
                                       std::uint32_t cellId)
        {
            std::filesystem::create_directories(core::NativeFilePath(directory));
            const auto pinned = directory / std::format("{:08X}.gz", cellId);
            const auto nativeSource = core::NativeFilePath(source);
            const auto nativePinned = core::NativeFilePath(pinned);
            std::error_code error;
            std::filesystem::create_hard_link(nativeSource, nativePinned, error);
            if (error && !std::filesystem::copy_file(nativeSource, nativePinned,
                                                     std::filesystem::copy_options::overwrite_existing, error))
            {
                throw std::runtime_error("Cannot retain candidate evidence for export");
            }
            return pinned;
        }

    } // namespace

    std::size_t EstimateGenerationBytes(const core::Scene &scene, std::optional<core::AABB> bounds)
    {
        if (!bounds)
        {
            core::AABB measured;
            for (const auto &vertex : scene.mesh.vertices)
            {
                measured.Expand(vertex);
            }
            bounds = measured;
        }
        const auto width = std::max(0.0, static_cast<double>(bounds->max.x) - bounds->min.x);
        const auto depth = std::max(0.0, static_cast<double>(bounds->max.y) - bounds->min.y);
        const auto voxelSize = std::max({4.0, width / 2048.0, depth / 2048.0});
        const auto rasterCells = (std::ceil(width / voxelSize) + 2) * (std::ceil(depth / voxelSize) + 2);
        const auto rasterBytes = std::isfinite(rasterCells) ? static_cast<std::size_t>(rasterCells * 128) : 0;
        return rasterBytes + scene.mesh.vertices.capacity() * sizeof(core::Vec3) +
               scene.mesh.triangles.capacity() * (sizeof(core::Triangle) + sizeof(core::TriangleProvenance) + 128) +
               scene.geometrySources.capacity() * sizeof(core::GeometrySource) +
               scene.waterSurfaces.vertices.capacity() * sizeof(core::Vec3) +
               scene.waterSurfaces.triangles.capacity() * sizeof(core::Triangle);
    }

    BatchCellResult BuildBatchCandidate(BatchGenerationInput input, const Options &options)
    {
        auto result = std::move(input.result);
        try
        {
            if (!input.geometry.failures.empty())
            {
                result.status = options.cellErrorPolicy == "stop" ? "failed" : "skipped_extraction_failed";
                result.error = skyrim::DescribeGeometryFailure(input.geometry.failures.front());
                return result;
            }
            const auto algorithm =
                options.partitioningAlgorithm == core::RegionPartitioningAlgorithm::Monotone ? "monotone"
                : options.partitioningAlgorithm == core::RegionPartitioningAlgorithm::Layers ? "layers"
                                                                                             : "watershed";
            const auto key =
                CandidateFingerprint(input.geometry.scene, options.navigationProfile, input.bounds, input.exits,
                                     input.adjacent, algorithm, options.recastSettings, result.cell->navMeshes,
                                     result.cell->waterHeight, options.tagWater, options.tagPreferredPaths);
            result.auditPath = key.empty() ? input.stagingDirectory / std::format("{:08X}.gz", result.cell->id)
                                           : input.cacheDirectory / "candidates" / (key + ".gz");
            core::Scene evidence;
            result.reused = !key.empty() && LoadCandidate(result.auditPath, result.candidate, evidence);
            const auto started = std::chrono::steady_clock::now();
            // Only cell-local generation/validation failures are recoverable.
            // Evidence storage and allocation exceptions still stop the run.
            try
            {
                if (result.reused)
                {
                    // A cached mesh cannot bypass the current scene-height safety check.
                    // Keep compatible successful caches without rerunning rasterization.
                    core::ValidateRecastSceneHeightRange(input.geometry.scene, options.navigationProfile, input.bounds,
                                                         options.recastSettings);
                }
                else
                {
                    result.candidate = core::RecastCandidateGenerator{}.Generate(
                        input.geometry.scene, options.navigationProfile, input.bounds, std::move(input.exits),
                        options.partitioningAlgorithm, options.recastSettings, core::CandidateRetention::AllWalkable);
                    (void)core::AttachCandidateAuthoredDoors(result.candidate, result.cell->navMeshes);
                    core::RepairCandidateBookkeeping(result.candidate);
                    if (input.bounds && !input.adjacent.empty())
                    {
                        (void)core::StitchCandidateBorders(result.candidate, *input.bounds, input.adjacent,
                                                           result.cell->navMeshes, true, &input.geometry.scene, false,
                                                           result.cell->waterHeight);
                    }
                    // Detect drops before cleanup so departure and landing triangles stay protected.
                    core::CaptureLedgeGeometry(result.candidate, input.geometry.scene, result.cell->waterHeight);
                    (void)core::BuildCandidateLedgeLinks(result.candidate);
                    (void)core::SimplifyCandidateSurface(result.candidate, input.bounds);
                    core::TagCandidateTriangles(result.candidate, result.cell->navMeshes, result.cell->waterHeight,
                                                options.tagWater, options.tagPreferredPaths);
                    if (!result.candidate.topology.valid)
                    {
                        result.diagnostics = result.candidate.topology.findings;
                        throw std::runtime_error("Candidate topology validation failed: " + result.diagnostics.front());
                    }
                }
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
                result.generationSeconds =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
                result.status = options.cellErrorPolicy == "stop" ? "failed" : "skipped_generation_failed";
                result.error = std::format("CELL {:08X}: {}", result.cell->id, error.what());
                result.candidate = {};
                result.auditPath.clear();
                result.reused = false;
                return result;
            }
            result.generationSeconds =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
            if (!result.reused)
            {
                core::CaptureCoverGeometry(result.candidate, input.geometry.scene);
                core::CaptureLedgeGeometry(result.candidate, input.geometry.scene, result.cell->waterHeight);
                evidence = CompactEvidence(result.candidate, input.geometry.scene);
                if (!StoreCandidate(result.auditPath, result.candidate, evidence))
                {
                    std::error_code error;
                    std::filesystem::remove(core::NativeFilePath(result.auditPath), error);
                    if (!StoreCandidate(result.auditPath, result.candidate, evidence))
                    {
                        throw std::runtime_error("Cannot spool candidate evidence");
                    }
                }
            }
            if (options.batchOutput != "plugin_only")
            {
                result.auditPath = PinAudit(result.auditPath, input.stagingDirectory, result.cell->id);
            }
            reproducibility::ExportMetadata metadata{
                .inputPlugin = options.affectedPlugin,
                .selectedCell = result.cell,
                .coverage = {.references = result.cell->references.size(),
                             .geometryVertices = input.geometry.scene.mesh.vertices.size(),
                             .geometryTriangles = input.geometry.scene.mesh.triangles.size(),
                             .terrainSupported = input.geometry.terrainSupported,
                             .collisionGeometrySupported = input.geometry.collisionModelsLoaded != 0},
                .warnings = {"Batch candidates are linked after every target has been generated.",
                             "Empty candidates have no retained walkable floor; authored geometry is not reused."}};
            result.metadata = reproducibility::ToJson(metadata, "    ");
            result.status = "generated";
        }
        catch (const std::exception &error)
        {
            result.status = "failed";
            result.error = std::format("CELL {:08X}: {}", result.cell->id, error.what());
        }
        return result;
    }

    /// One reconciliation attempt; the caller owns rollback of every candidate in this graph.
    static std::string ReconcileBatchBordersOnce(std::vector<BatchCellResult> &results,
                                                 const skyrim::ResolvedLoadOrder &resolved)
    {
        std::vector<core::GeneratedCellCandidate> exterior;
        std::vector<core::CandidateReachabilityTarget> reachability;
        std::map<std::uint32_t, std::array<std::int32_t, 2>> generatedCoordinates;
        std::vector<const BatchCellResult *> skippedExteriors;
        for (auto &result : results)
        {
            if ((result.status == "skipped_generation_failed" || result.status == "skipped_extraction_failed") &&
                result.cell->exteriorCoordinates)
            {
                skippedExteriors.push_back(&result);
            }
            if (result.status != "generated")
            {
                continue;
            }
            core::RepairCandidateBookkeeping(result.candidate);
            if (!result.candidate.topology.valid)
            {
                throw core::CandidateCellError(
                    result.cell->id,
                    std::format("CELL {:08X}: {}", result.cell->id, result.candidate.topology.findings.front()));
            }
            if (!result.cell->exteriorCoordinates)
            {
                reachability.push_back({result.cell->id, std::nullopt, std::nullopt, &result.candidate});
                continue;
            }
            const auto *record = resolved.FindWinning(result.cell->id);
            if (!record || !record->worldspaceFormId)
            {
                return "Generated exterior CELL has no resolved worldspace";
            }
            const auto [x, y] = *result.cell->exteriorCoordinates;
            generatedCoordinates.emplace(result.cell->id, *result.cell->exteriorCoordinates);
            const core::AABB bounds{.min = {x * 4096.0F, y * 4096.0F, std::numeric_limits<float>::lowest()},
                                    .max = {(static_cast<float>(x) + 1) * 4096.0F,
                                            (static_cast<float>(y) + 1) * 4096.0F, std::numeric_limits<float>::max()}};
            exterior.push_back({result.cell->id, *record->worldspaceFormId, bounds, &result.candidate});
            reachability.push_back({result.cell->id, *record->worldspaceFormId, bounds, &result.candidate});
        }
        try
        {
            const auto portalTargets = skippedExteriors.empty() ? std::map<std::uint32_t, const core::NavMesh *>{}
                                                                : IndexAuthoredPortalTargets(exterior, resolved);
            // Failed targets remain authored cells. Reintroduce their NAVMs as
            // untouched border constraints before joining successful generated cells.
            for (auto &target : exterior)
            {
                const auto *owner = resolved.FindWinning(target.cellId);
                const auto [ownerX, ownerY] = generatedCoordinates.at(target.cellId);
                std::vector<core::NavMesh> adjacent;
                for (const auto *result : skippedExteriors)
                {
                    const auto *neighbor = resolved.FindWinning(result->cell->id);
                    if (!neighbor || neighbor->worldspaceFormId != owner->worldspaceFormId)
                    {
                        continue;
                    }
                    const auto [x, y] = *result->cell->exteriorCoordinates;
                    if (std::abs(static_cast<std::int64_t>(x) - ownerX) +
                            std::abs(static_cast<std::int64_t>(y) - ownerY) ==
                        1)
                    {
                        adjacent.insert(adjacent.end(), result->cell->navMeshes.begin(), result->cell->navMeshes.end());
                    }
                }
                if (!adjacent.empty())
                {
                    // Stitching validates every portal against this neighbor set.
                    // Include established destinations alongside the failed targets;
                    // a failed cell without authored NAVM contributes no constraint.
                    std::set<std::uint32_t> neighborIds;
                    for (const auto &mesh : adjacent)
                    {
                        neighborIds.insert(mesh.id);
                    }
                    for (const auto &link : target.candidate->borderLinks)
                    {
                        if (link.generatedNeighborCell || !neighborIds.insert(link.neighborNavmeshId).second)
                        {
                            continue;
                        }
                        const auto *mesh = portalTargets.at(link.neighborNavmeshId);
                        if (!mesh)
                        {
                            throw std::runtime_error(
                                std::format("CELL {:08X}: authored border target {:08X} is missing", target.cellId,
                                            link.neighborNavmeshId));
                        }
                        adjacent.push_back(*mesh);
                    }
                    const auto result = std::find_if(results.begin(), results.end(),
                                                     [&](const auto &item) { return item.cell->id == target.cellId; });
                    (void)core::StitchCandidateBorders(*target.candidate, target.bounds, adjacent,
                                                       result->cell->navMeshes, true, nullptr, false,
                                                       result->cell->waterHeight);
                    if (!target.candidate->topology.valid)
                    {
                        throw core::CandidateCellError(
                            target.cellId,
                            std::format("CELL {:08X}: {}", target.cellId, target.candidate->topology.findings.front()));
                    }
                }
            }
            (void)core::StitchGeneratedCandidates(exterior);
            for (const auto &target : reachability)
            {
                (void)core::BuildCandidateLedgeLinks(*target.candidate);
            }
            (void)core::RemoveCandidateIslands(reachability);
        }
        catch (const core::CandidateCellError &)
        {
            throw;
        }
        catch (const std::exception &error)
        {
            return error.what();
        }
        return {};
    }

    std::string ReconcileBatchBorders(std::vector<BatchCellResult> &results, const skyrim::ResolvedLoadOrder &resolved,
                                      bool continueOnError)
    {
        // Roll back the complete graph after a local failure: earlier cells can already have new portal
        // destinations or compacted indices. Workers have spooled the original candidates, avoiding another
        // full in-memory copy of the batch. In-memory callers retain their own rollback snapshots here.
        std::map<std::uint32_t, core::CandidateNavMesh> originals;
        std::set<std::uint32_t> cellIds;
        for (const auto &result : results)
        {
            if (!result.cell || !cellIds.insert(result.cell->id).second)
            {
                return "Batch reconciliation requires unique valid CELL ownership";
            }
            if (result.status == "generated" && result.auditPath.empty())
            {
                if (!originals.emplace(result.cell->id, result.candidate).second)
                {
                    return "Batch reconciliation found duplicate generated CELL ownership";
                }
            }
        }
        while (true)
        {
            try
            {
                return ReconcileBatchBordersOnce(results, resolved);
            }
            catch (const core::CandidateCellError &error)
            {
                const auto failed =
                    std::find_if(results.begin(), results.end(), [&](const auto &result)
                                 { return result.cell->id == error.cellId && result.status == "generated"; });
                for (auto &result : results)
                {
                    if (result.status == "generated")
                    {
                        if (result.auditPath.empty())
                        {
                            result.candidate = originals.at(result.cell->id);
                        }
                        else
                        {
                            core::Scene evidence;
                            if (!LoadCandidate(result.auditPath, result.candidate, evidence))
                            {
                                return "Cannot restore spooled candidate after reconciliation failure";
                            }
                        }
                    }
                }
                if (!continueOnError || failed == results.end())
                {
                    return error.what();
                }
                failed->status = "skipped_generation_failed";
                failed->error = error.what();
                failed->diagnostics = {error.what()};
                failed->candidate = {};
                failed->auditPath.clear();
                failed->reused = false;
                originals.erase(error.cellId);
            }
        }
    }
} // namespace navmesh::app::detail
