#include "app/batch_generation.h"

#include "app/candidate_cache.h"
#include "core/navmesh/generator.h"
#include "core/navmesh/triangle_tagging.h"
#include "core/reproducibility/export_metadata.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <map>

namespace navmesh::app::detail
{
    namespace
    {
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
            std::filesystem::create_directories(directory);
            const auto pinned = directory / std::format("{:08X}.gz", cellId);
            std::error_code error;
            std::filesystem::create_hard_link(source, pinned, error);
            if (error &&
                !std::filesystem::copy_file(source, pinned, std::filesystem::copy_options::overwrite_existing, error))
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
               scene.geometrySources.capacity() * sizeof(core::GeometrySource);
    }

    BatchCellResult BuildBatchCandidate(BatchGenerationInput input, const Options &options)
    {
        auto result = std::move(input.result);
        try
        {
            const auto algorithm =
                options.partitioningAlgorithm == core::RegionPartitioningAlgorithm::Monotone ? "monotone"
                : options.partitioningAlgorithm == core::RegionPartitioningAlgorithm::Layers ? "layers"
                                                                                             : "watershed";
            const auto key = CandidateFingerprint(
                input.geometry.scene, options.navigationProfile, input.bounds, input.exits, input.adjacent, algorithm,
                options.recastSettings, result.cell->navMeshes, result.cell->waterHeight, options.tagTriangles);
            result.auditPath = key.empty() ? input.stagingDirectory / std::format("{:08X}.gz", result.cell->id)
                                           : input.cacheDirectory / "candidates" / (key + ".gz");
            core::Scene evidence;
            result.reused = !key.empty() && LoadCandidate(result.auditPath, result.candidate, evidence);
            if (!result.reused)
            {
                const auto started = std::chrono::steady_clock::now();
                result.candidate = core::RecastCandidateGenerator{}.Generate(
                    input.geometry.scene, options.navigationProfile, input.bounds, std::move(input.exits),
                    options.partitioningAlgorithm, options.recastSettings);
                if (input.bounds)
                {
                    (void)core::StitchCandidateBorders(result.candidate, *input.bounds, input.adjacent,
                                                       result.cell->navMeshes);
                }
                result.generationSeconds =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
                core::TagCandidateTriangles(result.candidate, result.cell->navMeshes, result.cell->waterHeight,
                                            options.tagTriangles);
                if (!result.candidate.topology.valid)
                {
                    throw std::runtime_error("Candidate topology validation failed");
                }
                evidence = CompactEvidence(result.candidate, input.geometry.scene);
                if (!StoreCandidate(result.auditPath, result.candidate, evidence))
                {
                    std::error_code error;
                    std::filesystem::remove(result.auditPath, error);
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
                .warnings = {"Batch candidates use neighboring geometry but remain clipped to their target CELL.",
                             "Generation eligibility and empty candidates are recorded in batch-report.json."}};
            result.metadata = reproducibility::ToJson(metadata, "    ");
            result.status = result.candidate.mesh.polygons.empty() ? "skipped_empty_candidate" : "generated";
            ReleaseCandidateAudit(result.candidate);
        }
        catch (const std::exception &error)
        {
            result.status = "failed";
            result.error = std::format("CELL {:08X}: {}", result.cell->id, error.what());
        }
        return result;
    }
} // namespace navmesh::app::detail
