#pragma once

#include "analysis/navmesh_analysis.h"
#include "skyrim/extraction/geometry_extractor.h"

namespace navmesh::app::detail
{
    /// Add model paths and source provenance to found support hits; all triangle indices refer to geometry.mesh.
    void AnnotateSupportSources(navmesh::analysis::AnalysisReport &report,
                                const navmesh::skyrim::offline::GeometryExtraction &geometry);

    /// Map scene provenance to analysis sources in mesh-triangle order; missing evidence keeps default source values.
    [[nodiscard]] std::vector<navmesh::analysis::TriangleSource> BuildTriangleSources(
        const navmesh::skyrim::offline::GeometryExtraction &geometry);

    /// Append world-space geometry and coverage, rebasing vertex and provenance indices; consumes source without transforming it.
    void AppendGeometry(navmesh::skyrim::offline::GeometryExtraction &destination,
                        navmesh::skyrim::offline::GeometryExtraction &&source);

    /// Retain valid support triangles whose world-space bounds intersect bounds; rebuild provenance and reference ranges in place.
    void CullGeometryToBounds(navmesh::skyrim::offline::GeometryExtraction &geometry,
                              const navmesh::core::AABB &bounds);
} // namespace navmesh::app::detail
