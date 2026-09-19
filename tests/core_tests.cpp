#include "analysis/navmesh_analysis.h"
#include "core/geometry/types.h"
#include "validation/validation.h"

#include <cassert>

int main()
{
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
    return 0;
}
