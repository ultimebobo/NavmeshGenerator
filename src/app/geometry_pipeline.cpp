#include "app/geometry_pipeline.h"

#include <iterator>
#include <utility>

namespace navmesh::app::detail
{
    void AnnotateSupportSources(navmesh::analysis::AnalysisReport &report,
                                const navmesh::skyrim::GeometryExtraction &geometry)
    {
        for (auto &polygon : report.polygons)
        {
            if (!polygon.support.found)
            {
                continue;
            }
            if (polygon.support.triangleIndex < geometry.scene.triangleProvenance.size())
            {
                const auto &provenance = geometry.scene.triangleProvenance[polygon.support.triangleIndex];
                if (provenance.geometrySource < geometry.scene.geometrySources.size())
                {
                    const auto &source = geometry.scene.geometrySources[provenance.geometrySource];
                    switch (source.sourceType)
                    {
                    case navmesh::core::GeometrySourceType::Terrain:
                        polygon.support.sourceType = "terrain";
                        break;
                    case navmesh::core::GeometrySourceType::Collision:
                        polygon.support.sourceType = "collision";
                        break;
                    case navmesh::core::GeometrySourceType::RenderFallback:
                        polygon.support.sourceType = "render_fallback";
                        break;
                    }
                    polygon.support.collisionType = source.collisionType;
                    polygon.support.sourceConfidence = source.confidence;
                    polygon.support.sourceTriangleIndex = provenance.sourceTriangle;
                }
            }
            for (const auto &reference : geometry.references)
            {
                if (polygon.support.triangleIndex < reference.meshTriangleOffset ||
                    polygon.support.triangleIndex >= reference.meshTriangleOffset + reference.triangles)
                {
                    continue;
                }
                polygon.support.sourceNifPath = reference.modelPath;
                if (!polygon.support.sourceTriangleIndex)
                {
                    polygon.support.sourceTriangleIndex = polygon.support.triangleIndex - reference.meshTriangleOffset;
                }
                break;
            }
        }
    }

    [[nodiscard]] std::vector<navmesh::analysis::TriangleSource> BuildTriangleSources(
        const navmesh::skyrim::GeometryExtraction &geometry)
    {
        std::vector<navmesh::analysis::TriangleSource> result(geometry.scene.mesh.triangles.size());
        for (std::size_t index{}; index < result.size() && index < geometry.scene.triangleProvenance.size(); ++index)
        {
            const auto &provenance = geometry.scene.triangleProvenance[index];
            if (provenance.geometrySource >= geometry.scene.geometrySources.size())
            {
                continue;
            }
            const auto &source = geometry.scene.geometrySources[provenance.geometrySource];
            using Type = navmesh::core::GeometrySourceType;
            result[index] = {
                .type = source.sourceType == Type::Collision ? navmesh::analysis::SupportSourceType::Collision
                        : source.sourceType == Type::Terrain ? navmesh::analysis::SupportSourceType::Terrain
                                                             : navmesh::analysis::SupportSourceType::RenderFallback,
                .confidence = source.confidence,
                .id = std::format("{}:{:08X}:{}", source.reference.plugin, source.reference.formId, source.modelPath)};
        }
        return result;
    }

    void AppendGeometry(navmesh::skyrim::GeometryExtraction &destination,
                        const navmesh::skyrim::GeometryExtraction &source)
    {
        const auto sourceOffset = destination.scene.geometrySources.size();
        const auto vertexOffset = static_cast<std::uint32_t>(destination.scene.mesh.vertices.size());
        const auto renderVertexOffset =
            static_cast<std::uint32_t>(destination.scene.renderFallbackMesh.vertices.size());
        const auto triangleOffset = destination.scene.mesh.triangles.size();
        const auto nodeStart = destination.scene.nodes.size();
        // Rebase source identifiers before copying provenance that points into the combined source table.
        destination.scene.geometrySources.insert(destination.scene.geometrySources.end(),
                                                 source.scene.geometrySources.begin(),
                                                 source.scene.geometrySources.end());
        destination.scene.nodes.insert(destination.scene.nodes.end(), source.scene.nodes.begin(),
                                       source.scene.nodes.end());
        for (std::size_t index = nodeStart; index < destination.scene.nodes.size(); ++index)
        {
            if (destination.scene.nodes[index].geometrySource)
            {
                *destination.scene.nodes[index].geometrySource += sourceOffset;
            }
        }
        // Append support and visual geometry with independent vertex offsets and matching provenance order.
        destination.scene.mesh.vertices.insert(destination.scene.mesh.vertices.end(),
                                               source.scene.mesh.vertices.begin(), source.scene.mesh.vertices.end());
        for (auto triangle : source.scene.mesh.triangles)
        {
            for (auto &vertex : triangle.vertices)
            {
                vertex += vertexOffset;
            }
            destination.scene.mesh.triangles.push_back(triangle);
        }
        for (auto provenance : source.scene.triangleProvenance)
        {
            provenance.geometrySource += sourceOffset;
            destination.scene.triangleProvenance.push_back(provenance);
        }
        destination.scene.renderFallbackMesh.vertices.insert(destination.scene.renderFallbackMesh.vertices.end(),
                                                             source.scene.renderFallbackMesh.vertices.begin(),
                                                             source.scene.renderFallbackMesh.vertices.end());
        for (auto triangle : source.scene.renderFallbackMesh.triangles)
        {
            for (auto &vertex : triangle.vertices)
            {
                vertex += renderVertexOffset;
            }
            destination.scene.renderFallbackMesh.triangles.push_back(triangle);
        }
        for (auto provenance : source.scene.renderFallbackTriangleProvenance)
        {
            provenance.geometrySource += sourceOffset;
            destination.scene.renderFallbackTriangleProvenance.push_back(provenance);
        }
        // Retain coverage and per-reference ranges alongside geometry so reports can trace every extracted model.
        destination.scene.coverage.insert(destination.scene.coverage.end(), source.scene.coverage.begin(),
                                          source.scene.coverage.end());
        for (auto reference : source.references)
        {
            reference.meshVertexOffset += vertexOffset;
            reference.meshTriangleOffset += triangleOffset;
            destination.references.push_back(std::move(reference));
        }
        // Aggregate extraction counters and publish the same support mesh through both extraction views.
        destination.referencesWithModels += source.referencesWithModels;
        destination.modelsLoaded += source.modelsLoaded;
        destination.modelsMissing += source.modelsMissing;
        destination.invalidVertices += source.invalidVertices;
        destination.invalidIndices += source.invalidIndices;
        destination.modelsExcluded += source.modelsExcluded;
        destination.modelsUnreadable += source.modelsUnreadable;
        destination.modelsUnsupported += source.modelsUnsupported;
        destination.collisionModelsLoaded += source.collisionModelsLoaded;
        destination.collisionTriangles += source.collisionTriangles;
        destination.renderFallbackModels += source.renderFallbackModels;
        destination.renderFallbackTriangles += source.renderFallbackTriangles;
        destination.terrainLandRecords += source.terrainLandRecords;
        destination.terrainLandDecoded += source.terrainLandDecoded;
        destination.terrainLandMissing += source.terrainLandMissing;
        destination.terrainSupported = destination.terrainSupported || source.terrainSupported;
        destination.collisionGeometrySupported =
            destination.collisionGeometrySupported || source.collisionGeometrySupported;
    }

    void CullGeometryToBounds(navmesh::skyrim::GeometryExtraction &geometry, const navmesh::core::AABB &bounds)
    {
        navmesh::core::Mesh selected;
        std::vector<navmesh::core::TriangleProvenance> provenance;
        // Save reference ownership ranges before rebuilding triangle and vertex offsets after culling.
        std::vector<std::pair<std::size_t, std::size_t>> oldRanges;
        oldRanges.reserve(geometry.references.size());
        for (const auto &reference : geometry.references)
        {
            oldRanges.push_back({reference.meshTriangleOffset, reference.triangles});
        }
        for (auto &reference : geometry.references)
        {
            reference.meshVertexOffset = 0;
            reference.meshTriangleOffset = 0;
            reference.vertices = 0;
            reference.triangles = 0;
        }
        std::size_t referenceIndex{};
        // Keep intersecting triangles in source order; duplicate their vertices and retain the corresponding audit join.
        for (std::size_t triangleIndex{}; triangleIndex < geometry.scene.mesh.triangles.size(); ++triangleIndex)
        {
            const auto &triangle = geometry.scene.mesh.triangles[triangleIndex];
            navmesh::core::AABB triangleBounds;
            bool valid = true;
            for (const auto vertex : triangle.vertices)
            {
                if (vertex >= geometry.scene.mesh.vertices.size())
                {
                    valid = false;
                    break;
                }
                triangleBounds.Expand(geometry.scene.mesh.vertices[vertex]);
            }
            if (!valid || !triangleBounds.Intersects(bounds))
            {
                continue;
            }
            while (referenceIndex < oldRanges.size() &&
                   triangleIndex >= oldRanges[referenceIndex].first + oldRanges[referenceIndex].second)
            {
                ++referenceIndex;
            }
            const auto base = static_cast<std::uint32_t>(selected.vertices.size());
            for (const auto vertex : triangle.vertices)
            {
                selected.vertices.push_back(geometry.scene.mesh.vertices[vertex]);
            }
            selected.triangles.push_back({{base, base + 1, base + 2}});
            if (triangleIndex < geometry.scene.triangleProvenance.size())
            {
                provenance.push_back(geometry.scene.triangleProvenance[triangleIndex]);
            }
            if (referenceIndex < geometry.references.size() && triangleIndex >= oldRanges[referenceIndex].first)
            {
                auto &reference = geometry.references[referenceIndex];
                if (reference.triangles == 0)
                {
                    reference.meshVertexOffset = base;
                    reference.meshTriangleOffset = selected.triangles.size() - 1;
                }
                reference.vertices += 3;
                ++reference.triangles;
            }
        }
        geometry.scene.mesh = std::move(selected);

        geometry.scene.triangleProvenance = std::move(provenance);

        // Display-only render meshes share the spatial selection with support
        // geometry while retaining their independent source-triangle audit join.
        navmesh::core::Mesh selectedRender;
        std::vector<navmesh::core::TriangleProvenance> renderProvenance;
        const auto &render = geometry.scene.renderFallbackMesh;
        for (std::size_t triangleIndex{}; triangleIndex < render.triangles.size(); ++triangleIndex)
        {
            const auto &triangle = render.triangles[triangleIndex];
            navmesh::core::AABB triangleBounds;
            bool valid = true;
            for (const auto vertex : triangle.vertices)
            {
                if (vertex >= render.vertices.size())
                {
                    valid = false;
                    break;
                }
                triangleBounds.Expand(render.vertices[vertex]);
            }
            if (!valid || !triangleBounds.Intersects(bounds))
            {
                continue;
            }
            const auto base = static_cast<std::uint32_t>(selectedRender.vertices.size());
            for (const auto vertex : triangle.vertices)
            {
                selectedRender.vertices.push_back(render.vertices[vertex]);
            }
            selectedRender.triangles.push_back({{base, base + 1, base + 2}});
            if (triangleIndex < geometry.scene.renderFallbackTriangleProvenance.size())
            {
                renderProvenance.push_back(geometry.scene.renderFallbackTriangleProvenance[triangleIndex]);
            }
        }
        geometry.scene.renderFallbackMesh = std::move(selectedRender);
        geometry.scene.renderFallbackTriangleProvenance = std::move(renderProvenance);
    }

} // namespace navmesh::app::detail
