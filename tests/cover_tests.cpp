#ifdef NDEBUG
#undef NDEBUG
#endif

#include "core/navmesh/cover.h"

#include <cassert>

void TestNavmeshCover()
{
    using namespace navmesh::core;
    CandidateNavMesh candidate;
    candidate.mesh.vertices = {{0, 0, 0}, {128, 0, 0}, {0, 128, 0}};
    NavPolygon face;
    face.vertices = {0, 1, 2};
    face.neighbors.fill(std::numeric_limits<std::uint32_t>::max());
    candidate.mesh.polygons.push_back(face);
    Scene scene;
    scene.mesh.vertices = {{0, -12, 0}, {128, -12, 0}, {128, -12, 128}, {0, -12, 128}};
    scene.mesh.triangles = {{{0, 1, 2}}, {{0, 2, 3}}};
    scene.geometrySources.push_back({.sourceType = GeometrySourceType::Collision});
    scene.triangleProvenance = {{.geometrySource = 0, .sourceTriangle = 0}, {.geometrySource = 0, .sourceTriangle = 1}};
    CaptureCoverGeometry(candidate, scene);
    assert(candidate.coverGeometry.triangles.size() == 2 && candidate.coverGeometry.vertices.size() == 4);
    auto generated = candidate.mesh;
    BuildNavmeshCover(generated, candidate, {});
    assert(generated.polygons[0].coverFlags == (0x4000U | 8));

    // A linked edge must remain a navigation portal even with physical geometry beyond it.
    generated = candidate.mesh;
    generated.polygons[0].flags = 1;
    generated.polygons[0].neighbors[0] = 0;
    BuildNavmeshCover(generated, candidate, {});
    assert(generated.polygons[0].coverFlags == 0 && generated.polygons[0].flags == 1);
    assert(generated.polygons[0].neighbors[0] == 0);

    scene.geometrySources[0].sourceType = GeometrySourceType::NavigationExclusion;
    CaptureCoverGeometry(candidate, scene);
    assert(candidate.coverGeometry.triangles.empty());
    generated = candidate.mesh;
    BuildNavmeshCover(generated, candidate, {});
    assert(generated.polygons[0].coverFlags == 0);

    // Cover on the third edge rotates all edge-dependent fields as one operation.
    scene.geometrySources[0].sourceType = GeometrySourceType::Collision;
    scene.mesh.vertices = {{-12, 0, 0}, {-12, 128, 0}, {-12, 128, 128}, {-12, 0, 128}};
    CaptureCoverGeometry(candidate, scene);
    generated = candidate.mesh;
    generated.polygons[0].flags = 1;
    generated.polygons[0].neighbors[0] = 5;
    BuildNavmeshCover(generated, candidate, {});
    const auto &rotated = generated.polygons[0];
    assert((rotated.vertices == std::array<std::uint32_t, 3>({1, 2, 0})));
    assert(rotated.coverFlags == (0x4000U | (8U << 6)));
    assert((rotated.flags & 7U) == 4 && rotated.neighbors[2] == 5);

    NavMesh authored = candidate.mesh;
    authored.polygons[0].coverFlags = 0x4004;
    authored.polygons[0].flags = 1U << 4;
    auto retained = candidate;
    retained.coverGeometry = {};
    generated = candidate.mesh;
    std::array originals{authored};
    BuildNavmeshCover(generated, retained, originals);
    assert(generated.polygons[0].coverFlags == 0x4004);
    assert(generated.polygons[0].flags & (1U << 4));
}
