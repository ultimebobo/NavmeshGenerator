#ifdef NDEBUG
#undef NDEBUG
#endif

#include "core/navmesh/triangle_tagging.h"

#include <cassert>
#include <limits>
#include <stdexcept>

void TestTriangleTagging()
{
    using namespace navmesh::core;
    NavMesh authored{.vertices = {{0, 0, 0}, {100, 0, 0}, {0, 100, 0}},
                     .polygons = {{.vertices = {0, 1, 2}, .flags = WaterFlag | PreferredPathFlag | 1U}}};
    CandidateNavMesh candidate;
    candidate.mesh = authored;
    candidate.mesh.vertices.insert(candidate.mesh.vertices.end(), {{0, 0, 100}, {100, 0, 100}, {0, 100, 100}});
    candidate.mesh.polygons.push_back({.vertices = {3, 4, 5}});
    const std::vector<NavMesh> originals{authored};
    TagCandidateTriangles(candidate, originals, std::nullopt);
    assert(candidate.triangleTagging);
    assert(candidate.mesh.polygons[0].flags == (WaterFlag | PreferredPathFlag | 1U));
    assert(candidate.mesh.polygons[1].flags == 0);

    // Explicit water overrides authored water; preference and connection bits are independent.
    TagCandidateTriangles(candidate, originals, -1.0F);
    assert(candidate.mesh.polygons[0].flags == (PreferredPathFlag | 1U));
    TagCandidateTriangles(candidate, originals, 10.0F);
    assert(candidate.mesh.polygons[0].flags == (WaterFlag | PreferredPathFlag | 1U));
    assert(candidate.mesh.polygons[1].flags == 0);
    TagCandidateTriangles(candidate, originals, 10.0F, false);
    assert(!candidate.triangleTagging && candidate.mesh.polygons[0].flags == 1U);

    // Closest unmarked floors constrain transfer even within the allowed climb tolerance.
    auto stacked = authored;
    for (auto &point : stacked.vertices)
    {
        point.z = 20;
    }
    stacked.polygons[0].flags = 0;
    for (auto &point : candidate.mesh.vertices)
    {
        point.z = 20;
    }
    TagCandidateTriangles(candidate, std::vector<NavMesh>{authored, stacked}, std::nullopt);
    assert(candidate.mesh.polygons[0].flags == 1U);

    // Contradictory coplanar evidence, deleted faces and malformed geometry cannot invent tags.
    stacked.vertices = authored.vertices;
    candidate.mesh.vertices = authored.vertices;
    candidate.mesh.polygons.resize(1);
    TagCandidateTriangles(candidate, std::vector<NavMesh>{authored, stacked}, std::nullopt);
    assert(candidate.mesh.polygons[0].flags == 1U);
    authored.polygons[0].flags |= 1U << 3;
    stacked.polygons[0].vertices = {0, 0, 0};
    TagCandidateTriangles(candidate, std::vector<NavMesh>{authored, stacked}, std::nullopt);
    assert(candidate.mesh.polygons[0].flags == 1U);
    authored.polygons[0].vertices[0] = 99;
    TagCandidateTriangles(candidate, std::vector<NavMesh>{authored}, std::nullopt);
    assert(candidate.mesh.polygons[0].flags == 1U);
    bool rejected{};
    try
    {
        TagCandidateTriangles(candidate, {}, std::numeric_limits<float>::quiet_NaN());
    }
    catch (const std::invalid_argument &)
    {
        rejected = true;
    }
    assert(rejected);
}
