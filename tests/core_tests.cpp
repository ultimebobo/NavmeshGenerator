#include "analysis/navmesh_analysis.h"
#include "core/geometry/types.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "validation/validation.h"

#include <array>

#include <NifFile.hpp>

#include <cassert>
#include <filesystem>
#include <fstream>
#include <vector>

namespace
{
    void TestBstTriShapeExtraction()
    {
        const auto tempRoot = std::filesystem::temp_directory_path() / "navmesh-bstrishape-test";
        std::filesystem::remove_all(tempRoot);
        std::filesystem::create_directories(tempRoot / "meshes");
        const auto modelPath = tempRoot / "meshes" / "MarkerX.nif";

        nifly::NifFile nif;
        nif.Create({ nifly::V20_2_0_7, 12, 130 });
        const std::vector<nifly::Vector3> vertices = { { 0.0F, 0.0F, 0.0F }, { 1.0F, 0.0F, 0.0F }, { 0.0F, 1.0F, 0.0F } };
        const std::vector<nifly::Triangle> triangles = { { 0, 1, 2 } };
        assert(nif.CreateShapeFromData("MarkerX", &vertices, &triangles, nullptr) != nullptr);
        assert(nif.Save(modelPath) == 0);

        navmesh::core::Cell cell;
        cell.references.push_back({ .id = 1, .baseObjectId = 2, .recordType = "STAT", .modelPath = "MarkerX.nif", .position = { 10.0F, 20.0F, 30.0F }, .scale = 1.0F });

        const auto geometry = navmesh::skyrim::offline::ExtractGeometry(tempRoot, cell, tempRoot);
        assert(geometry.modelsLoaded == 1u);
        assert(geometry.mesh.vertices.size() == 3u);
        assert(geometry.mesh.triangles.size() == 1u);
        assert(geometry.mesh.vertices[0].x == 10.0F && geometry.mesh.vertices[0].y == 20.0F && geometry.mesh.vertices[0].z == 30.0F);
    }
}

int main()
{
    TestBstTriShapeExtraction();
    navmesh::core::Mesh mesh{ .vertices = { { -1.0F, 2.0F, 3.0F }, { 4.0F, -5.0F, 6.0F } } };
    const auto bounds = mesh.Bounds();
    assert(bounds.IsValid());
    assert(bounds.min.x == -1.0F && bounds.min.y == -5.0F && bounds.max.z == 6.0F);

    navmesh::core::Cell cell;
    cell.navMeshes.push_back({ .id = 1, .vertices = { { 0.0F, 0.0F, 0.0F }, { 1.0F, 0.0F, 0.0F }, { 0.0F, 1.0F, 0.0F }, { 2.0F, 2.0F, 0.0F } }, .polygons = { { .vertices = { 0, 1, 2 }, .neighbors = { 0, 0, 0 }, .flags = 1 }, { .vertices = { 1, 3, 2 }, .neighbors = { 0, 0, 0 }, .flags = 2 } } });
    const auto findings = navmesh::validation::Validate(cell);
    assert(findings.empty());

    const auto analysis = navmesh::analysis::Analyze(cell.navMeshes.front());
    assert(analysis.vertexCount == 4u);
    assert(analysis.polygonCount == 2u);
    assert(analysis.connectedComponents == 1u);
    assert(analysis.boundingBox.min.x == 0.0F);
    assert(analysis.boundingBox.max.x == 2.0F);
    assert(analysis.isolatedPolygonCount == 0u);
    assert(analysis.degeneratePolygonCount == 0u);
    assert(analysis.averagePolygonArea > 0.0F);

    const auto box = navmesh::core::AABB{ .min = { -1.0F, -2.0F, -3.0F }, .max = { 3.0F, 4.0F, 5.0F } };
    const auto center = box.Center();
    const auto extent = box.Extent();
    assert(center.x == 1.0F && center.y == 1.0F && center.z == 1.0F);
    assert(extent.x == 4.0F && extent.y == 6.0F && extent.z == 8.0F);

    navmesh::analysis::SpatialIndex spatial;
    spatial.Build({ navmesh::core::Triangle{ { 0u, 1u, 2u } } }, {
        { 0.0F, 0.0F, 0.0F },
        { 1.0F, 0.0F, 0.0F },
        { 0.0F, 1.0F, 0.0F }
    });
    const auto nearest = spatial.NearestSurface({ 0.5F, 0.5F, -1.0F }, 5.0F);
    assert(nearest.has_value());
    assert(fabs(nearest->point.z) < 1.0e-4F);
    assert(nearest->distance >= 0.0F);
    const auto overlap = spatial.QueryAABB({ .min = { -0.1F, -0.1F, -0.1F }, .max = { 0.5F, 0.5F, 0.5F } });
    assert(!overlap.empty());

    const auto horizontalNormal = navmesh::core::Vec3{ 0.0F, 0.0F, 1.0F };
    assert(navmesh::analysis::SurfaceSlopeDegrees(horizontalNormal) < 1.0e-3F);

    const auto slopedNormal = navmesh::core::Vec3{ 0.0F, 0.5F, 0.8660254F };
    const auto slopeDegrees = navmesh::analysis::SurfaceSlopeDegrees(slopedNormal);
    assert(slopeDegrees > 29.0F && slopeDegrees < 31.0F);

    assert(navmesh::analysis::ClassifySupport(-3.0F, 10.0F, 2.0F, 45.0F) == "buried");
    assert(navmesh::analysis::ClassifySupport(3.0F, 10.0F, 2.0F, 45.0F) == "floating");
    assert(navmesh::analysis::ClassifySupport(1.0F, 50.0F, 2.0F, 45.0F) == "too_steep");
    assert(navmesh::analysis::ClassifySupport(1.0F, 10.0F, 2.0F, 45.0F) == "supported");

    navmesh::core::NavMesh syntheticMesh{ .vertices = { { 0.0F, 0.0F, 3.0F }, { 1.0F, 0.0F, 3.0F }, { 0.0F, 1.0F, 3.0F } }, .polygons = { { .vertices = { 0, 1, 2 }, .neighbors = { 0, 0, 0 }, .flags = 1 } } };
    navmesh::core::Mesh syntheticGeometry{ .vertices = { { 0.0F, 0.0F, 1.0F }, { 1.0F, 0.0F, 1.0F }, { 0.0F, 1.0F, 1.0F } }, .triangles = { { { 0u, 1u, 2u } } } };
    const auto polygonReport = navmesh::analysis::AnalyzeNavMeshPolygons(syntheticMesh, syntheticGeometry, { .surfaceSearchRadius = 64.0F, .maxSupportDistance = 2.0F, .maxSlope = 45.0F });
    assert(polygonReport.summary.polygonsAnalyzed == 1u);
    assert(polygonReport.polygons.front().support.found);
    assert(polygonReport.polygons.front().classification == "supported");

    navmesh::core::NavMesh centroidMesh{ .vertices = { { 0.5F, 0.5F, 3.0F }, { 1.5F, 0.5F, 3.0F }, { 0.5F, 1.5F, 3.0F } }, .polygons = { { .vertices = { 0, 1, 2 }, .neighbors = { 0, 0, 0 }, .flags = 1 } } };
    navmesh::core::Mesh wallScene{ .vertices = {
        { 0.0F, 0.0F, 0.0F }, { 2.0F, 0.0F, 0.0F }, { 0.0F, 2.0F, 0.0F },
        { 0.45F, 0.0F, 0.0F }, { 0.55F, 0.0F, 1.5F }, { 0.50F, 1.0F, 0.0F }
    }, .triangles = { { { 0u, 1u, 2u } }, { { 3u, 4u, 5u } } } };
    const auto wallSelection = navmesh::analysis::AnalyzeNavMeshPolygons(centroidMesh, wallScene, { .surfaceSearchRadius = 64.0F, .maxSupportDistance = 2.0F, .maxSlope = 45.0F });
    assert(wallSelection.polygons.front().support.found);
    assert(wallSelection.polygons.front().support.triangleIndex == 0u);
    assert(wallSelection.polygons.front().classification == "floating");
    return 0;
}
