#include "app/cell_scene.h"

#include <algorithm>
#include <bit>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

namespace navmesh::app::detail
{
    std::vector<core::SceneLayer> ParseSceneLayers(const std::string &value)
    {
        std::vector<core::SceneLayer> result;
        std::stringstream input(value);
        std::string token;
        while (std::getline(input, token, ','))
        {
            std::transform(token.begin(), token.end(), token.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (token == "navmesh" || token == "navm")
            {
                result.push_back(core::SceneLayer::ExistingNavmesh);
            }
            else if (token == "terrain")
            {
                result.push_back(core::SceneLayer::Terrain);
            }
            else if (token == "collision")
            {
                result.push_back(core::SceneLayer::Collision);
            }
            else if (token == "render" || token == "render_fallback")
            {
                result.push_back(core::SceneLayer::RenderFallback);
            }
            else if (token == "diagnostics" || token == "markers")
            {
                result.push_back(core::SceneLayer::DiagnosticMarkers);
            }
            else if (token == "candidate")
            {
                result.push_back(core::SceneLayer::CandidateNavmesh);
            }
        }
        return result;
    }

    void CellScene::AppendMesh(const core::Scene &scene, bool render)
    {
        const auto &mesh = render ? scene.renderFallbackMesh : scene.mesh;
        const auto &provenance = render ? scene.renderFallbackTriangleProvenance : scene.triangleProvenance;
        auto &destination = render ? scene_.renderFallbackMesh : scene_.mesh;
        auto &joins = render ? scene_.renderFallbackTriangleProvenance : scene_.triangleProvenance;
        std::map<std::uint32_t, std::uint32_t> vertices;
        // Source identities distinguish separate placements. Exact positions also distinguish source
        // triangles reused at different transforms, without welding across coordinate spaces.
        for (std::size_t index{}; index < mesh.triangles.size() && index < provenance.size(); ++index)
        {
            const auto &triangle = mesh.triangles[index];
            auto join = provenance[index];
            if (join.geometrySource >= scene.geometrySources.size() ||
                std::any_of(triangle.vertices.begin(), triangle.vertices.end(),
                            [&](auto vertex)
                            {
                                return vertex >= mesh.vertices.size() || !std::isfinite(mesh.vertices[vertex].x) ||
                                       !std::isfinite(mesh.vertices[vertex].y) ||
                                       !std::isfinite(mesh.vertices[vertex].z);
                            }))
            {
                continue;
            }
            const auto &source = scene.geometrySources[join.geometrySource];
            SourceKey key{source.reference.plugin,  source.reference.formId, source.baseObject.plugin,
                          source.baseObject.formId, source.modelPath,        source.sourceType,
                          source.materialClass,     source.collisionType,    source.navigationObstacle};
            const auto [found, added] = sources_.try_emplace(std::move(key), scene_.geometrySources.size());
            if (added)
            {
                scene_.geometrySources.push_back(source);
            }
            join.geometrySource = found->second;
            std::array<std::uint32_t, 9> positions;
            for (std::size_t corner{}; corner < triangle.vertices.size(); ++corner)
            {
                const auto &point = mesh.vertices[triangle.vertices[corner]];
                positions[corner * 3] = std::bit_cast<std::uint32_t>(point.x);
                positions[corner * 3 + 1] = std::bit_cast<std::uint32_t>(point.y);
                positions[corner * 3 + 2] = std::bit_cast<std::uint32_t>(point.z);
            }
            if (!triangles_
                     .emplace(join.geometrySource, join.sourceTriangle, join.terrain ? join.terrain->landFormId : 0,
                              positions, render)
                     .second)
            {
                continue;
            }
            core::Triangle copied;
            for (std::size_t corner{}; corner < triangle.vertices.size(); ++corner)
            {
                const auto old = triangle.vertices[corner];
                const auto [vertex, inserted] =
                    vertices.try_emplace(old, static_cast<std::uint32_t>(destination.vertices.size()));
                if (inserted)
                {
                    destination.vertices.push_back(mesh.vertices[old]);
                }
                copied.vertices[corner] = vertex->second;
            }
            destination.triangles.push_back(copied);
            joins.push_back(join);
        }
    }

    void CellScene::Append(const core::Scene &scene, const std::vector<core::Cell> &cells,
                           const std::vector<core::DiagnosticMarker> &markers)
    {
        AppendMesh(scene, false);
        AppendMesh(scene, true);
        markers_.insert(markers_.end(), markers.begin(), markers.end());
        std::set<std::uint32_t> authoredDoors;
        for (const auto &cell : cells)
        {
            for (const auto &mesh : cell.navMeshes)
            {
                navmeshes_.try_emplace(mesh.id, mesh);
                for (const auto &link : mesh.doorLinks)
                {
                    authoredDoors.insert(link.referenceId);
                }
            }
        }
        for (const auto &cell : cells)
        {
            for (const auto &reference : cell.references)
            {
                if (!reference.deleted && !reference.initiallyDisabled &&
                    (reference.teleportExit || authoredDoors.contains(reference.id)))
                {
                    doors_.try_emplace(
                        reference.id, core::CandidateExit{.referenceId = reference.id, .position = reference.position});
                }
            }
        }
    }

    bool CellScene::Write(const std::filesystem::path &path, const Options &options,
                          const std::vector<const core::Cell *> &selected,
                          const std::vector<core::SceneCandidate> &candidates) const
    {
        std::vector<core::NavMesh> meshes;
        for (const auto &[id, mesh] : navmeshes_)
        {
            meshes.push_back(mesh);
        }
        auto exits = doors_;
        for (const auto &view : candidates)
        {
            for (auto exit : view.candidate->exits)
            {
                // Global markers have no candidate-local polygon index. Each candidate supplies its own face colors.
                exit.polygon.reset();
                if (exits.contains(exit.referenceId))
                {
                    exits[exit.referenceId] = exit;
                }
            }
        }
        std::vector<core::CandidateExit> doors;
        for (const auto &[id, exit] : exits)
        {
            doors.push_back(exit);
        }
        reproducibility::ExportMetadata metadata{
            .inputPlugin = options.copyPlugin ? options.copySourcePlugin : options.plugin,
            .coverage = {.geometryVertices = scene_.mesh.vertices.size(),
                         .geometryTriangles = scene_.mesh.triangles.size()},
            .warnings = {"Cells retain native Skyrim coordinates; separate interiors or worldspaces may overlap."},
            .selectedCells = selected};
        core::SceneExportOptions settings{.layers = ParseSceneLayers(options.geometryLayers),
                                          .detailedProvenance = options.outputDetail != "summary",
                                          .candidateEntrances = &doors,
                                          .candidates = candidates};
        if (!candidates.empty())
        {
            settings.layers.push_back(core::SceneLayer::CandidateNavmesh);
        }
        const auto result = core::WriteCombinedGlb(path, scene_, meshes, markers_, metadata, settings);
        return result.written && reproducibility::WriteSidecar(path, metadata);
    }
} // namespace navmesh::app::detail
