#ifdef NDEBUG
#undef NDEBUG
#endif

#include "app/batch_generation.h"
#include "app/candidate_cache.h"
#include "app/geometry_pipeline.h"
#include "core/navmesh/batch_stitching.h"
#include "core/navmesh/candidate_reachability.h"
#include "core/navmesh/detail_triangulation.h"
#include "core/navmesh/generator.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <Recast.h>

namespace
{
    navmesh::core::CandidateNavMesh Floor(float x, float y, float height, bool splitWest = false)
    {
        using namespace navmesh::core;
        CandidateNavMesh result;
        result.mesh.vertices = {
            {x, y, height}, {x + 4096, y, height}, {x + 4096, y + 4096, height}, {x, y + 4096, height}};
        result.mesh.polygons = {{.vertices = {0, 1, 2}}, {.vertices = {0, 2, 3}}};
        if (splitWest)
        {
            result.mesh.vertices.push_back({x, y + 2048, height});
            result.mesh.polygons[1].vertices = {0, 2, 4};
            result.mesh.polygons.push_back({.vertices = {4, 2, 3}});
        }
        CandidateRegion region{.id = 0};
        for (std::uint32_t index{}; index < result.mesh.polygons.size(); ++index)
        {
            result.mesh.polygons[index].flags = 0x40;
            region.polygons.push_back(index);
            result.polygonSourceTriangles.push_back(7);
            result.polygonContributingTriangles.push_back({7, 8});
        }
        result.regions.push_back(region);
        RefreshCandidateTopology(result);
        assert(result.topology.valid);
        return result;
    }

    void VerifyPortals(std::span<navmesh::core::GeneratedCellCandidate> cells)
    {
        for (const auto &cell : cells)
        {
            assert(cell.candidate->topology.valid);
            for (const auto &portal : cell.candidate->borderLinks)
            {
                if (!portal.generatedNeighborCell)
                {
                    continue;
                }
                const auto target = std::find_if(cells.begin(), cells.end(), [&](const auto &other)
                                                 { return other.cellId == *portal.generatedNeighborCell; });
                assert(target != cells.end());
                const auto &sourceFace = cell.candidate->mesh.polygons.at(portal.polygon);
                const auto &targetFace = target->candidate->mesh.polygons.at(portal.neighborPolygon);
                const auto a = cell.candidate->mesh.vertices.at(sourceFace.vertices[portal.edge]);
                const auto b = cell.candidate->mesh.vertices.at(sourceFace.vertices[(portal.edge + 1) % 3]);
                const auto c = target->candidate->mesh.vertices.at(targetFace.vertices[portal.neighborEdge]);
                const auto d = target->candidate->mesh.vertices.at(targetFace.vertices[(portal.neighborEdge + 1) % 3]);
                assert(a.x == d.x && a.y == d.y && a.z == d.z);
                assert(b.x == c.x && b.y == c.y && b.z == c.z);
                assert(std::count_if(target->candidate->borderLinks.begin(), target->candidate->borderLinks.end(),
                                     [&](const auto &reverse)
                                     {
                                         return reverse.generatedNeighborCell == cell.cellId &&
                                                reverse.polygon == portal.neighborPolygon &&
                                                reverse.edge == portal.neighborEdge &&
                                                reverse.neighborPolygon == portal.polygon &&
                                                reverse.neighborEdge == portal.edge;
                                     }) == 1);
            }
        }
    }

    /// Append a disconnected floor with distinct evidence to exercise stable index compaction.
    void AppendFloor(navmesh::core::CandidateNavMesh &candidate, navmesh::core::CandidateNavMesh floor)
    {
        const auto vertexOffset = static_cast<std::uint32_t>(candidate.mesh.vertices.size());
        const auto polygonOffset = static_cast<std::uint32_t>(candidate.mesh.polygons.size());
        candidate.mesh.vertices.insert(candidate.mesh.vertices.end(), floor.mesh.vertices.begin(),
                                       floor.mesh.vertices.end());
        for (auto polygon : floor.mesh.polygons)
        {
            for (auto &vertex : polygon.vertices)
            {
                vertex += vertexOffset;
            }
            candidate.mesh.polygons.push_back(polygon);
            candidate.polygonSourceTriangles.push_back(11);
            candidate.polygonContributingTriangles.push_back({11, 12});
        }
        for (auto region : floor.regions)
        {
            region.id = static_cast<std::uint32_t>(candidate.regions.size());
            for (auto &polygon : region.polygons)
            {
                polygon += polygonOffset;
            }
            candidate.regions.push_back(std::move(region));
        }
        navmesh::core::RefreshCandidateTopology(candidate);
        assert(candidate.topology.valid);
    }

    void TestIndependentStackedSeamPartitions()
    {
        using namespace navmesh::core;
        for (const bool vertical : {true, false})
        {
            std::array<CandidateNavMesh, 2> candidates{Floor(0, 0, 0), Floor(4096, 0, 0)};
            auto roof = Floor(4096, 0, 400, true);
            // The landing fits the test agent's diameter, while every roof
            // partition is narrower. Its tiny middle interval belongs only upstairs.
            for (auto &candidate : candidates)
            {
                for (auto &vertex : candidate.mesh.vertices)
                {
                    vertex.y *= 96.0F / 4096;
                }
                RefreshCandidateTopology(candidate);
            }
            for (auto &vertex : roof.mesh.vertices)
            {
                vertex.y *= 96.0F / 4096;
            }
            roof.mesh.vertices.push_back({4096, 48.25F, 400});
            roof.mesh.polygons[2].vertices = {4, 2, 5};
            roof.mesh.polygons.push_back({.vertices = {5, 2, 3}});
            roof.polygonSourceTriangles.push_back(7);
            roof.polygonContributingTriangles.push_back({7, 8});
            roof.regions[0].polygons.push_back(3);
            auto upperLeft = Floor(0, 0, 400);
            for (auto &vertex : upperLeft.mesh.vertices)
            {
                vertex.y *= 96.0F / 4096;
            }
            AppendFloor(candidates[0], std::move(upperLeft));
            AppendFloor(candidates[1], std::move(roof));
            if (!vertical)
            {
                for (auto &candidate : candidates)
                {
                    for (auto &vertex : candidate.mesh.vertices)
                    {
                        std::swap(vertex.x, vertex.y);
                    }
                    for (auto &face : candidate.mesh.polygons)
                    {
                        std::swap(face.vertices[1], face.vertices[2]);
                    }
                    RefreshCandidateTopology(candidate);
                }
            }
            const AABB next =
                vertical ? AABB{{4096, 0, -100}, {8192, 4096, 1000}} : AABB{{0, 4096, -100}, {4096, 8192, 1000}};
            std::array<GeneratedCellCandidate, 2> cells{
                {{1, 1, {{0, 0, -100}, {4096, 4096, 1000}}, &candidates[0]}, {2, 1, next, &candidates[1]}}};
            assert(StitchGeneratedCandidates(cells) == 4);
            VerifyPortals(cells);
            for (const auto &candidate : candidates)
            {
                std::size_t floorLinks{}, roofLinks{};
                for (const auto &link : candidate.borderLinks)
                {
                    const auto &face = candidate.mesh.polygons[link.polygon];
                    const auto a = candidate.mesh.vertices[face.vertices[link.edge]];
                    const auto b = candidate.mesh.vertices[face.vertices[(link.edge + 1) % 3]];
                    if (a.z == 0)
                    {
                        ++floorLinks;
                        assert(b.z == 0 && std::hypot(b.x - a.x, b.y - a.y) == 96);
                        assert(candidate.polygonSourceTriangles[link.polygon] == 7);
                    }
                    else
                    {
                        ++roofLinks;
                        assert(a.z == 400 && b.z == 400);
                        assert(std::hypot(b.x - a.x, b.y - a.y) < 64);
                        assert(candidate.polygonSourceTriangles[link.polygon] == 11);
                    }
                }
                assert(floorLinks == 1 && roofLinks == 3);
            }
            assert(StitchGeneratedCandidates(cells) == 0);
            VerifyPortals(cells);
        }
    }

    void TestCandidateIslandRemoval()
    {
        using namespace navmesh::core;
        // Immediate single-cell linking can finish seam validation before the
        // shared filter chooses a primary floor without authored navigation.
        auto single = Floor(128, 128, 0);
        for (auto &vertex : single.mesh.vertices)
        {
            vertex.x = 128 + (vertex.x - 128) / 2;
            vertex.y = 128 + (vertex.y - 128) / 2;
        }
        auto island = Floor(128, 128, 400);
        for (auto &vertex : island.mesh.vertices)
        {
            vertex.x = 128 + (vertex.x - 128) / 8;
            vertex.y = 128 + (vertex.y - 128) / 8;
        }
        AppendFloor(single, island);
        const AABB singleBounds{{0, 0, -100}, {4096, 4096, 1000}};
        assert(StitchCandidateBorders(single, singleBounds, {}, {}, false, nullptr, true) == 0);
        assert(single.topology.valid && single.mesh.polygons.size() == 4);
        std::array singleTarget{CandidateReachabilityTarget{1, 1, singleBounds, &single}};
        assert(RemoveCandidateIslands(singleTarget) == 2);
        assert(single.topology.valid && single.mesh.polygons.size() == 2);
        assert(single.borderLinks.empty() && single.exits.empty());
        assert(single.statistics.rejectedUnreachable == 2);
        for (const auto &vertex : single.mesh.vertices)
        {
            assert(vertex.z == 0);
        }

        // Entrance-reachable exterior floors take precedence over larger disconnected scenery.
        // Incidental boundary matches cannot turn that unused scenery into a playable component.
        for (const bool authoredPortal : {false, true})
        {
            auto enclosed = Floor(0, 0, 400);
            auto playable = Floor(0, 0, 0);
            for (auto &vertex : playable.mesh.vertices)
            {
                vertex.x /= 4;
                vertex.y /= 4;
            }
            AppendFloor(enclosed, playable);
            enclosed.exits = {{200, {100, 100, 0}, 1, 2}};
            if (authoredPortal)
            {
                enclosed.borderLinks = {{0, 0, 300, 0, 0}};
                enclosed.authoredBorderVertexEdits = {{300, 0, {0, 0, 400}, {0, 0, 401}}};
            }
            CandidateReachabilityTarget target{1, 1, singleBounds, &enclosed};
            assert(RemoveCandidateIslands(std::span(&target, 1)) == 2);
            assert(enclosed.mesh.polygons.size() == 2 && enclosed.borderLinks.empty());
            assert(enclosed.authoredBorderVertexEdits.empty());
            assert(enclosed.exits[0].polygon == 0 && enclosed.topology.valid);
            assert(enclosed.mesh.vertices.size() == playable.mesh.vertices.size());
            for (std::size_t index{}; index < playable.mesh.vertices.size(); ++index)
            {
                const auto actual = enclosed.mesh.vertices[index];
                const auto expected = playable.mesh.vertices[index];
                assert(actual.x == expected.x && actual.y == expected.y && actual.z == expected.z);
            }
        }

        // A separate floor can lead to an exterior route through an untouched cell,
        // even when a door anchors another network inside this selected area.
        auto crossing = Floor(0, 0, 400);
        AppendFloor(crossing, Floor(0, 0, 0));
        crossing.exits = {{200, {100, 100, 0}, 1, 2}};
        crossing.borderLinks = {{.polygon = 0, .edge = 0, .neighborNavmeshId = 300, .requiredAuthoredCrossing = true}};
        CandidateReachabilityTarget crossingTarget{1, 1, singleBounds, &crossing};
        assert(RemoveCandidateIslands(std::span(&crossingTarget, 1)) == 0);
        assert(crossing.mesh.polygons.size() == 4 && crossing.borderLinks.size() == 1);
        assert(crossing.borderLinks.front().requiredAuthoredCrossing && crossing.exits.front().polygon == 2);
        assert(HasCandidateFloorRoute(crossing, {100, 100, 400}, {200, 200, 400}));
        assert(!HasCandidateFloorRoute(crossing, {100, 100, 400}, {100, 100, 0}));
        assert(!HasCandidateFloorRoute(crossing, {100, 100, 400}, {100, 100, 800}));

        // This roof spans a CELL seam and acquires reciprocal portals, but has no
        // route to the larger ground network. Put it first so retained identities move.
        std::array<CandidateNavMesh, 2> candidates{Floor(0, 0, 400), Floor(4096, 0, 400, true)};
        for (std::size_t index{}; index < candidates.size(); ++index)
        {
            for (auto &vertex : candidates[index].mesh.vertices)
            {
                vertex.y = 512 + vertex.y / 4;
            }
            RefreshCandidateTopology(candidates[index]);
            AppendFloor(candidates[index], Floor(static_cast<float>(index) * 4096, 0, 0, index == 1));
        }
        candidates[0].exits = {{100, {100, 100, 0}, 1, 2}, {101, {100, 100, 800}, std::nullopt, std::nullopt}};
        std::array<GeneratedCellCandidate, 2> cells{{{1, 1, {{0, 0, -100}, {4096, 4096, 1000}}, &candidates[0]},
                                                     {2, 1, {{4096, 0, -100}, {8192, 4096, 1000}}, &candidates[1]}}};
        assert(StitchGeneratedCandidates(cells) >= 4);
        VerifyPortals(cells);
        std::array<CandidateReachabilityTarget, 2> targets{
            {{1, 1, cells[0].bounds, &candidates[0]}, {2, 1, cells[1].bounds, &candidates[1]}}};
        assert(RemoveCandidateIslands(targets) > 0);
        VerifyPortals(cells);
        for (const auto &candidate : candidates)
        {
            assert(candidate.regions.size() == 1 && candidate.regions[0].id == 0);
            assert(candidate.regions[0].reachesBorder);
            assert(candidate.statistics.rejectedUnreachable > 0);
            assert(candidate.statistics.outputPolygons == candidate.mesh.polygons.size());
            assert(candidate.regions[0].sourceTriangles == std::vector<std::size_t>({11, 12}));
            for (const auto vertex : candidate.mesh.vertices)
            {
                assert(vertex.z == 0);
            }
            for (std::size_t polygon{}; polygon < candidate.mesh.polygons.size(); ++polygon)
            {
                assert(candidate.mesh.polygons[polygon].flags == 0x40);
                assert(candidate.polygonSourceTriangles[polygon] == 11);
                assert(candidate.polygonContributingTriangles[polygon] == std::vector<std::size_t>({11, 12}));
            }
            for (const auto &contour : candidate.contours)
            {
                assert(contour.region == 0 && contour.closed);
                for (const auto vertex : contour.vertices)
                {
                    assert(vertex < candidate.mesh.vertices.size());
                }
            }
        }
        assert(candidates[0].exits[0].region == 0 && candidates[0].exits[0].polygon);
        assert(!candidates[0].exits[1].region && !candidates[0].exits[1].polygon);
        assert(RemoveCandidateIslands(targets) == 0);
        VerifyPortals(cells);

        // A target containing only the rejected cross-CELL roof becomes an empty
        // replacement, and both directions of its generated portals disappear.
        candidates = {Floor(0, 0, 400), Floor(4096, 0, 400)};
        for (auto &candidate : candidates)
        {
            for (auto &vertex : candidate.mesh.vertices)
            {
                vertex.y /= 4;
            }
            RefreshCandidateTopology(candidate);
        }
        AppendFloor(candidates[0], Floor(0, 0, 0));
        assert(StitchGeneratedCandidates(cells) > 0);
        assert(RemoveCandidateIslands(targets) > 0);
        VerifyPortals(cells);
        assert(candidates[0].mesh.polygons.size() == 2 && candidates[0].borderLinks.empty());
        assert(candidates[1].mesh.polygons.empty() && candidates[1].mesh.vertices.empty());
        assert(candidates[1].regions.empty() && candidates[1].contours.empty() && candidates[1].borderLinks.empty());

        // Doorless interiors retain their primary floor; smaller disconnected
        // surfaces survive only when a matched door or authored portal provides access.
        auto interior = Floor(0, 0, 400);
        for (auto &vertex : interior.mesh.vertices)
        {
            vertex.x /= 4;
            vertex.y /= 4;
        }
        RefreshCandidateTopology(interior);
        AppendFloor(interior, Floor(0, 0, 0));
        auto door = interior;
        door.exits = {{200, {100, 100, 400}, 0, 0}};
        auto authored = interior;
        authored.borderLinks = {{0, 0, 300, 0, 0}};
        std::array<CandidateReachabilityTarget, 3> interiors{
            {{10, {}, {}, &interior}, {11, {}, {}, &door}, {12, {}, {}, &authored}}};
        assert(RemoveCandidateIslands(interiors) == 2);
        assert(interior.mesh.polygons.size() == 2 && interior.regions.size() == 1);
        assert(door.mesh.polygons.size() == 4 && door.exits[0].polygon == 0 && door.exits[0].region == 0);
        assert(authored.mesh.polygons.size() == 4 && authored.borderLinks.size() == 1);

        // Spatially separate selections and overlapping grids in another worldspace
        // each retain their own primary floor, including a valid empty target.
        candidates = {Floor(0, 0, 0), Floor(8192, 0, 0)};
        auto otherWorld = Floor(0, 0, 400);
        CandidateNavMesh empty;
        std::array<CandidateReachabilityTarget, 4> separate{
            {{1, 1, cells[0].bounds, &candidates[0]},
             {2, 1, AABB{{8192, 0, -100}, {12288, 4096, 1000}}, &candidates[1]},
             {3, 2, cells[0].bounds, &otherWorld},
             {4, {}, {}, &empty}}};
        assert(RemoveCandidateIslands(separate) == 0);
        assert(candidates[0].mesh.polygons.size() == 2 && candidates[1].mesh.polygons.size() == 2);
        assert(otherWorld.mesh.polygons.size() == 2 && empty.topology.valid && empty.mesh.vertices.empty());

        // A walkable ramp keeps its elevated landing in the primary network.
        // Another floor touching that landing at one vertex remains an island.
        const auto platform = [](float x, float y, float height)
        {
            auto result = Floor(0, 0, height);
            for (auto &vertex : result.mesh.vertices)
            {
                vertex.x = x + vertex.x / 8;
                vertex.y = y + vertex.y / 8;
            }
            RefreshCandidateTopology(result);
            return result;
        };
        auto accessedRoof = platform(0, 0, 0);
        auto ramp = platform(512, 0, 0);
        for (auto &vertex : ramp.mesh.vertices)
        {
            vertex.z = (vertex.x - 512) * 400 / 512;
        }
        AppendFloor(accessedRoof, std::move(ramp));
        AppendFloor(accessedRoof, platform(1024, 0, 400));
        AppendFloor(accessedRoof, platform(1536, 512, 400));
        CandidateReachabilityTarget accessed{20, {}, {}, &accessedRoof};
        assert(RemoveCandidateIslands(std::span(&accessed, 1)) == 2);
        assert(accessedRoof.topology.valid && accessedRoof.mesh.polygons.size() == 6);
        assert(std::any_of(accessedRoof.mesh.vertices.begin(), accessedRoof.mesh.vertices.end(),
                           [](auto vertex) { return vertex.z == 400; }));
    }
} // namespace

/// A wall enclosure with a teleport gate must not acquire a route into the surrounding scenery floor.
void TestTeleportEnclosure()
{
    using namespace navmesh;
    skyrim::GeometryExtraction geometry;
    geometry.scene.geometrySources.push_back({.sourceType = core::GeometrySourceType::Collision});
    const auto quad = [&](std::array<core::Vec3, 4> corners)
    {
        const auto offset = static_cast<std::uint32_t>(geometry.scene.mesh.vertices.size());
        geometry.scene.mesh.vertices.insert(geometry.scene.mesh.vertices.end(), corners.begin(), corners.end());
        for (const auto triangle :
             {core::Triangle{{offset, offset + 1, offset + 2}}, core::Triangle{{offset, offset + 2, offset + 3}}})
        {
            geometry.scene.mesh.triangles.push_back(triangle);
            geometry.scene.triangleProvenance.push_back({0, geometry.scene.triangleProvenance.size()});
        }
    };
    quad({{{0, 0, 0}, {4096, 0, 0}, {4096, 4096, 0}, {0, 4096, 0}}});
    const auto wall = [&](core::Vec3 a, core::Vec3 b)
    { quad({a, b, b + core::Vec3{0, 0, 256}, a + core::Vec3{0, 0, 256}}); };
    wall({1024, 1024, 0}, {3072, 1024, 0});
    wall({3072, 1024, 0}, {3072, 3072, 0});
    wall({3072, 3072, 0}, {2176, 3072, 0});
    wall({1920, 3072, 0}, {1024, 3072, 0});
    wall({1024, 3072, 0}, {1024, 1024, 0});
    core::Cell cell;
    cell.references.push_back({.id = 1,
                               .recordType = "REFR",
                               .modelPath = "synthetic/threshold.nif",
                               .position = {2048, 3072, 0},
                               .rotation = {0, 0, 1.57079632679F},
                               .baseRecordType = "DOOR",
                               .teleportExit = true,
                               .teleportArrival = core::Vec3{2048, 2816, 0},
                               .localBounds = core::AABB{{-8, -128, -8}, {8, 128, 256}}});
    const auto barrier = skyrim::ExtractGeometry({}, cell, {}, {}, {}, nullptr, nullptr, true);
    assert(barrier.scene.mesh.triangles.size() == 12 && barrier.scene.HasCompleteTriangleProvenance());
    app::detail::AppendGeometry(geometry, barrier);
    const core::AABB bounds{{0, 0, -100}, {4096, 4096, 1000}};
    auto candidate = core::RecastCandidateGenerator{}.Generate(
        geometry.scene, {}, bounds, {{1, *cell.references.front().teleportArrival}},
        core::RegionPartitioningAlgorithm::Watershed, {}, core::CandidateRetention::AllWalkable);
    assert(candidate.topology.valid && candidate.exits.front().polygon);
    core::CandidateReachabilityTarget target{1, 1, bounds, &candidate};
    assert(core::RemoveCandidateIslands(std::span(&target, 1)) > 0);
    assert(candidate.topology.valid && !candidate.mesh.polygons.empty());
    for (const auto vertex : candidate.mesh.vertices)
    {
        assert(vertex.x > 1024 && vertex.x < 3072 && vertex.y > 1024 && vertex.y < 3072);
    }
}

void TestGeneratedLandingSeams()
{
    using namespace navmesh::core;

    // A small height bend on a broad neighbor must not become a steep barrier
    // when its partitions subdivide a thin strip on the other side of the CELL.
    {
        std::array<CandidateNavMesh, 2> candidates;
        candidates[0].mesh.vertices = {{4096, 0, 2}, {4096, 300, 2}, {4084, 150, 2}};
        candidates[0].mesh.polygons = {{.vertices = {0, 1, 2}}};
        candidates[1].mesh.vertices = {{4096, 0, 1}, {4096, 100, 1}, {4096, 103, 0}, {4096, 300, 0}, {4296, 150, 1}};
        candidates[1].mesh.polygons = {{.vertices = {1, 0, 4}}, {.vertices = {2, 1, 4}}, {.vertices = {3, 2, 4}}};
        for (auto &candidate : candidates)
        {
            CandidateRegion region{.id = 0};
            for (std::uint32_t index{}; index < candidate.mesh.polygons.size(); ++index)
            {
                candidate.polygonSourceTriangles.push_back(0);
                candidate.polygonContributingTriangles.push_back({0});
                region.polygons.push_back(index);
            }
            candidate.regions.push_back(std::move(region));
            RefreshCandidateTopology(candidate);
            assert(candidate.topology.valid);
        }
        std::array<GeneratedCellCandidate, 2> cells{{{1, 1, {{0, 0, -100}, {4096, 4096, 100}}, &candidates[0]},
                                                     {2, 1, {{4096, 0, -100}, {8192, 4096, 100}}, &candidates[1]}}};
        assert(StitchGeneratedCandidates(cells) == 3);
        VerifyPortals(cells);
        for (const auto &candidate : candidates)
        {
            for (const auto &face : candidate.mesh.polygons)
            {
                const auto a = candidate.mesh.vertices[face.vertices[0]];
                const auto u = candidate.mesh.vertices[face.vertices[1]] - a;
                const auto v = candidate.mesh.vertices[face.vertices[2]] - a;
                const auto normal = Vec3{u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x};
                assert(std::hypot(normal.x, normal.y) <= normal.z);
            }
        }
    }

    // Independently extracted cells share the same elevated landing even when
    // their lower collision floors have unrelated fractional world heights.
    for (const auto lowerHeight : {-500.25F, 20.25F})
    {
        std::array<CandidateNavMesh, 2> landings;
        std::array<GeneratedCellCandidate, 2> landingCells;
        for (std::size_t index{}; index < landings.size(); ++index)
        {
            Scene scene;
            scene.geometrySources.push_back({.sourceType = GeometrySourceType::Collision, .confidence = 1});
            for (const auto height : {lowerHeight + index * 0.75F, 300.25F})
            {
                const auto offset = static_cast<std::uint32_t>(scene.mesh.vertices.size());
                scene.mesh.vertices.insert(
                    scene.mesh.vertices.end(),
                    {{3900, 0, height}, {4300, 0, height}, {4300, 1024, height}, {3900, 1024, height}});
                scene.mesh.triangles.push_back({{offset, offset + 1, offset + 2}});
                scene.mesh.triangles.push_back({{offset, offset + 2, offset + 3}});
                scene.triangleProvenance.push_back({0, scene.triangleProvenance.size()});
                scene.triangleProvenance.push_back({0, scene.triangleProvenance.size()});
            }
            const auto x = index * 4096.0F;
            const AABB bounds{{x, 0, -1000}, {x + 4096, 4096, 1000}};
            landings[index] = RecastCandidateGenerator{}.Generate(
                scene, {}, bounds, {}, RegionPartitioningAlgorithm::Watershed, {}, CandidateRetention::AllWalkable);
            assert(landings[index].topology.valid && !landings[index].mesh.polygons.empty());
            landingCells[index] = {static_cast<std::uint32_t>(index + 1), 1, bounds, &landings[index]};
        }
        std::optional<float> landingHeight;
        for (const auto &candidate : landings)
        {
            bool foundLanding{};
            for (const auto vertex : candidate.mesh.vertices)
            {
                if (vertex.z > 200)
                {
                    if (landingHeight)
                    {
                        assert(vertex.z == *landingHeight);
                    }
                    landingHeight = vertex.z;
                    foundLanding = true;
                }
            }
            assert(foundLanding);
        }
        assert(StitchGeneratedCandidates(landingCells) > 0);
        VerifyPortals(landingCells);
        for (const auto &candidate : landings)
        {
            for (const auto vertex : candidate.mesh.vertices)
            {
                if (vertex.z > 200)
                {
                    assert(vertex.z == *landingHeight);
                }
            }
        }
    }
}

void TestGeneratedBatchTopology()
{
    using namespace navmesh::core;
    {
        auto candidate = Floor(0, 0, 0);
        const auto vertices = candidate.mesh.vertices;
        const auto polygons = candidate.mesh.polygons;
        candidate.mesh.vertices.push_back({100, 100, 100});
        candidate.regions.front().polygons = {0, 0, 99};
        candidate.contours = {{.closed = false, .vertices = {99}}};
        candidate.exits = {{.referenceId = 300, .region = 42, .polygon = 1}};
        assert(!ValidateCandidateTopology(candidate).valid);
        RepairCandidateBookkeeping(candidate);
        assert(candidate.topology.valid && candidate.mesh.vertices.size() == vertices.size());
        assert(candidate.mesh.polygons.size() == polygons.size() && candidate.regions.size() == 1);
        assert(candidate.exits.front().region == 0 && candidate.exits.front().polygon == 1);
        assert(candidate.regions.front().exitFormIds == std::vector<std::uint32_t>{300});
        assert(std::all_of(candidate.contours.begin(), candidate.contours.end(),
                           [](const auto &contour) { return contour.closed; }));
        for (std::size_t index{}; index < polygons.size(); ++index)
        {
            assert(candidate.mesh.polygons[index].vertices == polygons[index].vertices);
        }
        candidate.polygonContributingTriangles.front().clear();
        RepairCandidateBookkeeping(candidate);
        assert(!candidate.topology.valid);
    }
    TestCandidateIslandRemoval();
    TestTeleportEnclosure();
    TestGeneratedLandingSeams();
    TestIndependentStackedSeamPartitions();
    // Batch interiors retain a supported floor even without any authored NAVM,
    // exterior boundary or door anchor, under every partitioning strategy.
    Scene interior;
    interior.geometrySources.push_back({.sourceType = GeometrySourceType::Collision, .confidence = 1.0F});
    interior.mesh.vertices = {{0, 0, 0}, {512, 0, 0}, {512, 512, 0}, {0, 512, 0}};
    interior.mesh.triangles = {{{0, 1, 2}}, {{0, 2, 3}}};
    interior.triangleProvenance = {{0, 0, {}}, {0, 1, {}}};
    for (const auto algorithm : {RegionPartitioningAlgorithm::Watershed, RegionPartitioningAlgorithm::Monotone,
                                 RegionPartitioningAlgorithm::Layers})
    {
        const auto candidate = RecastCandidateGenerator{}.Generate(interior, {}, std::nullopt, {}, algorithm, {},
                                                                   CandidateRetention::AllWalkable);
        assert(candidate.topology.valid && !candidate.mesh.polygons.empty());
        assert(candidate.exits.empty() && candidate.statistics.rejectedUnreachable == 0);
        assert(candidate.regions.size() == 1 && !candidate.regions[0].reachesBorder);
    }

    // A small retained island forces contour refinement beside a concave floor.
    // All region interfaces must use one pass so the island has one floor covering it.
    Scene regional;
    regional.geometrySources = interior.geometrySources;
    const auto rectangle = [&](float x0, float y0, float x1, float y1)
    {
        const auto base = static_cast<std::uint32_t>(regional.mesh.vertices.size());
        regional.mesh.vertices.insert(regional.mesh.vertices.end(),
                                      {{x0, y0, 0}, {x1, y0, 0}, {x1, y1, 0}, {x0, y1, 0}});
        for (const auto triangle : {Triangle{{base, base + 1, base + 2}}, Triangle{{base, base + 2, base + 3}}})
        {
            regional.triangleProvenance.push_back({0, regional.mesh.triangles.size(), {}});
            regional.mesh.triangles.push_back(triangle);
        }
    };
    rectangle(0, 0, 512, 320);
    rectangle(0, 320, 320, 512);
    rectangle(364, 364, 396, 396);
    NavigationProfile regionalProfile;
    regionalProfile.agentRadius = 0;
    regionalProfile.minimumRegionArea = 0;
    RecastSettings coarse;
    coarse.maxSimplificationError = 64;
    const auto refined = RecastCandidateGenerator{}.Generate(regional, regionalProfile, std::nullopt, {},
                                                             RegionPartitioningAlgorithm::Watershed, coarse,
                                                             CandidateRetention::AllWalkable);
    assert(refined.topology.valid && refined.regions.size() == 2);
    const auto coverings = [&](Vec3 point)
    {
        return std::count_if(refined.mesh.polygons.begin(), refined.mesh.polygons.end(),
                             [&](const auto &face)
                             {
                                 for (std::size_t side{}; side < 3; ++side)
                                 {
                                     const auto a = refined.mesh.vertices[face.vertices[side]];
                                     const auto b = refined.mesh.vertices[face.vertices[(side + 1) % 3]];
                                     if ((b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x) < 0)
                                     {
                                         return false;
                                     }
                                 }
                                 return true;
                             });
    };
    assert(coverings({375, 379, 0}) == 1);
    assert(coverings({348, 348, 0}) == 0);

    // Different seam subdivisions and floor heights meet at a shared four-CELL corner.
    // No authored NAVM identities or door anchors are supplied to retain these floors.
    std::array<CandidateNavMesh, 4> candidates{Floor(0, 0, 0), Floor(4096, 0, 2, true), Floor(0, 4096, 4),
                                               Floor(4096, 4096, 6, true)};
    std::array<GeneratedCellCandidate, 4> cells;
    for (std::size_t index{}; index < cells.size(); ++index)
    {
        const auto x = static_cast<float>(index % 2) * 4096;
        const auto y = static_cast<float>(index / 2) * 4096;
        cells[index] = {
            static_cast<std::uint32_t>(index + 1), 1, {{x, y, -100}, {x + 4096, y + 4096, 100}}, &candidates[index]};
    }
    assert(StitchGeneratedCandidates(cells) == 6);
    VerifyPortals(cells);
    for (const auto &candidate : candidates)
    {
        assert(candidate.regions.size() == 1);
        assert(candidate.regions.front().area == 4096.0F * 4096.0F);
        for (std::size_t index{}; index < candidate.mesh.polygons.size(); ++index)
        {
            assert(candidate.mesh.polygons[index].flags == 0x40);
            assert(candidate.polygonSourceTriangles[index] == 7);
            assert(candidate.polygonContributingTriangles[index] == std::vector<std::size_t>({7, 8}));
        }
    }

    // A climb-compatible chain need not admit one common height at its corner.
    // Preserve separate seam levels and their internal step connections.
    candidates = {Floor(0, 0, 0), Floor(4096, 0, 30), Floor(0, 4096, 60), Floor(4096, 4096, 60)};
    assert(StitchGeneratedCandidates(cells) == 3);
    VerifyPortals(cells);

    // Corner welding can bring a previously unreachable neighboring interval
    // within climb range. Complete linking must revisit it and keep existing returns.
    candidates[0] = Floor(0, 0, 0);
    candidates[0].mesh.vertices[3].z = 25;
    candidates[1] = Floor(4096, 0, 30);
    candidates[2] = {};
    candidates[2].mesh.vertices = {{4000, 4096, 60}, {4096, 4096, 60}, {4000, 4200, 60}};
    candidates[2].mesh.polygons = {{.vertices = {0, 1, 2}}};
    candidates[2].polygonSourceTriangles = {7};
    candidates[2].polygonContributingTriangles = {{7}};
    candidates[2].regions = {{.id = 0, .polygons = {0}}};
    candidates[3] = Floor(4096, 4096, 60);
    for (auto &candidate : candidates)
    {
        RefreshCandidateTopology(candidate);
    }
    assert(StitchGeneratedCandidates(cells) == 3);
    VerifyPortals(cells);
    assert(StitchGeneratedCandidates(cells) == 0);

    // Untouched neighboring portals can pin different heights at one XY corner.
    // Preserve both authored edges and join the generated floors through a valid step.
    candidates[0] = Floor(0, 0, 0);
    candidates[1] = Floor(4096, 0, 2);
    candidates[0].borderLinks = {{0, 0, 100, 0, 0}};
    candidates[1].borderLinks = {{0, 0, 200, 0, 0}};
    assert(StitchGeneratedCandidates(std::span(cells).first(2)) == 1);
    VerifyPortals(std::span(cells).first(2));
    for (std::size_t index{}; index < 2; ++index)
    {
        const auto &candidate = candidates[index];
        const auto &link = candidate.borderLinks.front();
        const auto &face = candidate.mesh.polygons[link.polygon];
        const auto a = candidate.mesh.vertices[face.vertices[link.edge]];
        const auto b = candidate.mesh.vertices[face.vertices[(link.edge + 1) % 3]];
        assert(a.z == index * 2 && b.z == index * 2 && a.y == 0 && b.y == 0);
    }

    // Every positive shared interval is linked, even below the weld tolerance.
    candidates[0] = Floor(0, 0, 0);
    candidates[1] = Floor(4096, 0, 0, true);
    candidates[1].mesh.vertices[4].y = 0.025F;
    RefreshCandidateTopology(candidates[1]);
    assert(StitchGeneratedCandidates(std::span(cells).first(2)) == 2);
    VerifyPortals(std::span(cells).first(2));

    // A one-ULP strip at a distant CELL boundary has no float interior centroid.
    // Triangulate its subdivided boundary directly and retain every portal interval.
    candidates[0] = {};
    candidates[0].mesh.vertices = {{131071.984375F, 0, 0}, {131072, 0, 0}, {131072, 4096, 0}};
    candidates[0].mesh.polygons = {{.vertices = {0, 1, 2}}};
    candidates[0].polygonSourceTriangles = {7};
    candidates[0].polygonContributingTriangles = {{7}};
    candidates[0].regions = {{.id = 0, .polygons = {0}}};
    RefreshCandidateTopology(candidates[0]);
    candidates[1] = Floor(131072, 0, 0, true);
    auto strips =
        std::array<GeneratedCellCandidate, 2>{{{1, 1, {{126976, 0, -100}, {131072, 4096, 100}}, &candidates[0]},
                                               {2, 1, {{131072, 0, -100}, {135168, 4096, 100}}, &candidates[1]}}};
    assert(StitchGeneratedCandidates(strips) == 2);
    VerifyPortals(strips);
    assert(candidates[0].regions[0].area == 32);

    // Adjacent float endpoints still form a positive portal interval without a float midpoint.
    candidates[0] = Floor(0, 131072, 0);
    candidates[1] = Floor(4096, 131072, 0, true);
    candidates[1].mesh.vertices[4].y = 131072.015625F;
    RefreshCandidateTopology(candidates[1]);
    auto narrowIntervals =
        std::array<GeneratedCellCandidate, 2>{{{1, 1, {{0, 131072, -100}, {4096, 135168, 100}}, &candidates[0]},
                                               {2, 1, {{4096, 131072, -100}, {8192, 135168, 100}}, &candidates[1]}}};
    assert(StitchGeneratedCandidates(narrowIntervals) == 2);
    VerifyPortals(narrowIntervals);

    // A missing or unreachable neighboring floor leaves valid navigation intact.
    candidates[0] = Floor(0, 0, 0);
    candidates[1] = {};
    assert(StitchGeneratedCandidates(std::span(cells).first(2)) == 0);
    assert(candidates[0].mesh.polygons.size() == 2);
    candidates[1] = Floor(4096, 0, 1000);
    assert(StitchGeneratedCandidates(std::span(cells).first(2)) == 0);
    assert(candidates[0].mesh.polygons.size() == 2 && candidates[1].mesh.polygons.size() == 2);

    // Quantization must not confuse short internal edges with neighboring sides.
    auto thin = Floor(0, 0, 0);
    thin.mesh.vertices = {{0, 0, 0}, {100, 0, 0}, {100, 0.001F, 0}, {0, 100, 0}};
    RefreshCandidateTopology(thin);
    assert(thin.topology.valid);

    // Collinear hull samples and interior points on an initial diagonal retain
    // complete floor coverage without reusing a directed triangle edge.
    const std::array<Vec3, 7> samples{{{100000, -200000, 0},
                                       {100100, -200000, 1},
                                       {100200, -200000, 2},
                                       {100200, -199800, 4},
                                       {100000, -199800, 2},
                                       {100100, -199900, 10},
                                       {100050, -199950, 6}}};
    const std::array<std::uint32_t, 5> hull{0, 1, 2, 3, 4};
    const auto triangles = TriangulateDetailSamples(samples, hull);
    std::set<std::pair<std::uint32_t, std::uint32_t>> directed;
    std::set<std::uint32_t> used;
    double area{};
    for (const auto &triangle : triangles)
    {
        const auto a = samples[triangle[0]], b = samples[triangle[1]], c = samples[triangle[2]];
        const auto twiceArea = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
        assert(twiceArea > 0);
        area += twiceArea * 0.5;
        for (std::size_t edge{}; edge < 3; ++edge)
        {
            assert(directed.emplace(triangle[edge], triangle[(edge + 1) % 3]).second);
            used.insert(triangle[edge]);
        }
    }
    assert(area == 40000 && used.size() == samples.size());

    // A proposed authored corner subdivision must leave an already consumed side intact.
    const auto open = std::numeric_limits<std::uint32_t>::max();
    auto corner = Floor(0, 0, 0);
    corner.mesh.vertices = {{0, 0, 0}, {300, 0, 0}, {0, 300, 0}};
    corner.mesh.polygons = {{.vertices = {0, 1, 2}, .neighbors = {open, open, open}}};
    corner.polygonSourceTriangles = {0};
    corner.polygonContributingTriangles = {{0}};
    corner.regions = {{.id = 0, .polygons = {0}}};
    RefreshCandidateTopology(corner);
    corner.borderLinks = {{0, 2, 201, 0, 0}};
    const NavMesh west{.id = 201,
                       .vertices = {{0, 0, 0}, {0, 300, 0}, {-100, 100, 0}},
                       .polygons = {{.vertices = {0, 1, 2}, .neighbors = {open, open, open}}}};
    const NavMesh south{.id = 202,
                        .vertices = {{300, 0, 0}, {0, -1, 0}, {100, -100, 0}},
                        .polygons = {{.vertices = {0, 1, 2}, .neighbors = {open, open, open}}}};
    (void)StitchCandidateBorders(corner, {{0, 0, -100}, {300, 300, 100}}, {west, south});
    assert(corner.topology.valid);
    const auto retained = std::find_if(corner.borderLinks.begin(), corner.borderLinks.end(),
                                       [&](const auto &link) { return link.neighborNavmeshId == west.id; });
    assert(retained != corner.borderLinks.end());
    const auto &face = corner.mesh.polygons[retained->polygon];
    const auto first = corner.mesh.vertices[face.vertices[retained->edge]];
    const auto last = corner.mesh.vertices[face.vertices[(retained->edge + 1) % 3]];
    assert(first.x == 0 && first.y == 300 && first.z == 0);
    assert(last.x == 0 && last.y == 0 && last.z == 0);
}

void TestBatchGenerationFailurePolicy()
{
    using namespace navmesh;
    const auto directory =
        std::filesystem::temp_directory_path() /
        ("navmesh-generation-failure-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(directory);
    app::Options options;
    options.batchOutput = "plugin_only";
    options.tagTriangles = false;
    core::Cell badCell{.id = 10};
    core::Scene floor;
    floor.geometrySources.push_back({.sourceType = core::GeometrySourceType::Collision, .confidence = 1});
    floor.mesh.vertices = {{0, 0, 0}, {512, 0, 0}, {512, 512, 0}, {0, 512, 0}};
    floor.mesh.triangles = {{{0, 1, 2}}, {{0, 2, 3}}};
    floor.triangleProvenance = {{0, 0, {}}, {0, 1, {}}};
    // Height validation must reject incomplete cached-scene evidence before indexing it.
    auto incomplete = floor;
    incomplete.triangleProvenance.clear();
    bool rejectedIncomplete{};
    try
    {
        core::ValidateRecastSceneHeightRange(incomplete, options.navigationProfile, {}, options.recastSettings);
    }
    catch (const std::invalid_argument &)
    {
        rejectedIncomplete = true;
    }
    assert(rejectedIncomplete);
    auto oversized = floor;
    const auto excessiveHeight = (RC_SPAN_MAX_HEIGHT + 1) * options.recastSettings.cellHeight;
    oversized.mesh.vertices.insert(
        oversized.mesh.vertices.end(),
        {{0, 0, excessiveHeight}, {512, 0, excessiveHeight}, {512, 512, excessiveHeight}, {0, 512, excessiveHeight}});
    oversized.mesh.triangles.insert(oversized.mesh.triangles.end(), {{{4, 5, 6}}, {{4, 6, 7}}});
    oversized.triangleProvenance.insert(oversized.triangleProvenance.end(), {{0, 2, {}}, {0, 3, {}}});
    const auto input = [&](const core::Scene &scene, const core::Cell &cell)
    {
        app::detail::BatchGenerationInput value;
        value.result.cell = &cell;
        value.geometry.scene = scene;
        value.cacheDirectory = directory / "cache";
        value.stagingDirectory = directory / "staging";
        return value;
    };
    auto skipped = app::detail::BuildBatchCandidate(input(oversized, badCell), options);
    assert(skipped.status == "skipped_generation_failed" && skipped.error.contains("vertical span limit"));
    assert(skipped.candidate.mesh.polygons.empty() && skipped.auditPath.empty() && skipped.metadata.empty());
    core::Cell goodCell{.id = 11};
    const auto successful = app::detail::BuildBatchCandidate(input(floor, goodCell), options);
    assert(successful.status == "generated" && successful.error.empty() && !successful.candidate.mesh.polygons.empty());
    const auto reused = app::detail::BuildBatchCandidate(input(floor, goodCell), options);
    assert(reused.status == "generated" && reused.reused);

    // Cell and Repair reuse the same bounded authored-neighbor border adjustments under either error policy.
    const auto floorKey = app::detail::CandidateFingerprint(floor, options.navigationProfile, {}, {}, {}, "watershed",
                                                            options.recastSettings, {}, {}, options.tagTriangles);
    const auto floorCache = directory / "cache" / "candidates" / (floorKey + ".gz");
    auto movingNeighbor = successful.candidate;
    movingNeighbor.authoredBorderVertexEdits.push_back({201, 0, {0, 0, 0}, {0, 0, 1}});
    assert(app::detail::StoreCandidate(floorCache, movingNeighbor, floor));
    for (const auto scope : {app::RebuildScope::Cell, app::RebuildScope::Repair})
    {
        for (const auto policy : {"continue", "stop"})
        {
            auto rebuildOptions = options;
            rebuildOptions.rebuildScope = scope;
            rebuildOptions.cellErrorPolicy = policy;
            const auto rebuilt = app::detail::BuildBatchCandidate(input(floor, goodCell), rebuildOptions);
            assert(rebuilt.status == "generated" && rebuilt.error.empty() && rebuilt.reused);
            assert(rebuilt.candidate.mesh.polygons.size() == successful.candidate.mesh.polygons.size());
            const auto &edits = rebuilt.candidate.authoredBorderVertexEdits;
            assert(edits.size() == 1 && edits[0].navmeshId == 201 && edits[0].vertex == 0);
            assert(edits[0].original.z == 0 && edits[0].position.z == 1);
        }
    }
    assert(app::detail::StoreCandidate(floorCache, successful.candidate, floor));

    // A roof with only a downward exit has no entrance from the retained primary floor.
    auto roofScene = floor;
    roofScene.mesh.vertices.insert(roofScene.mesh.vertices.end(),
                                   {{160, 160, 400}, {352, 160, 400}, {352, 352, 400}, {160, 352, 400}});
    roofScene.mesh.triangles.insert(roofScene.mesh.triangles.end(), {{{4, 5, 6}}, {{4, 6, 7}}});
    roofScene.triangleProvenance.insert(roofScene.triangleProvenance.end(), {{0, 2, {}}, {0, 3, {}}});
    for (const auto algorithm :
         {core::RegionPartitioningAlgorithm::Watershed, core::RegionPartitioningAlgorithm::Monotone,
          core::RegionPartitioningAlgorithm::Layers})
    {
        auto roofOptions = options;
        roofOptions.partitioningAlgorithm = algorithm;
        for (const bool cached : {false, true})
        {
            auto roof = app::detail::BuildBatchCandidate(input(roofScene, goodCell), roofOptions);
            assert(roof.status == "generated" && roof.reused == cached && roof.candidate.regions.size() == 2);
            std::vector<app::detail::BatchCellResult> roofResults{std::move(roof)};
            assert(app::detail::ReconcileBatchBorders(roofResults, {}).empty());
            const auto &retained = roofResults[0].candidate;
            assert(retained.topology.valid && retained.regions.size() == 1);
            assert(retained.ledgeLinks.empty());
            assert(std::all_of(retained.mesh.vertices.begin(), retained.mesh.vertices.end(),
                               [](auto point) { return point.z < 400; }));
        }
    }

    // Cached candidates must not hide a scene whose solids exceed raster height storage.
    const auto key = app::detail::CandidateFingerprint(oversized, options.navigationProfile, {}, {}, {}, "watershed",
                                                       options.recastSettings, {}, {}, options.tagTriangles);
    assert(
        app::detail::StoreCandidate(directory / "cache" / "candidates" / (key + ".gz"), successful.candidate, floor));
    skipped = app::detail::BuildBatchCandidate(input(oversized, badCell), options);
    assert(skipped.status == "skipped_generation_failed" && skipped.error.contains("vertical span limit"));
    assert(!skipped.reused && skipped.candidate.mesh.polygons.empty());

    // Output failures are global infrastructure errors, not cell skips.
    const auto blockedPath = directory / "blocked";
    std::ofstream(blockedPath) << "regular file";
    auto blocked = input(floor, goodCell);
    blocked.cacheDirectory = blockedPath;
    const auto failed = app::detail::BuildBatchCandidate(std::move(blocked), options);
    assert(failed.status == "failed" && !failed.error.empty());

    // A failed neighbor's authored geometry remains an available reciprocal border target.
    goodCell.exteriorCoordinates = std::array<std::int32_t, 2>{0, 0};
    badCell.exteriorCoordinates = std::array<std::int32_t, 2>{1, 0};
    auto authored = Floor(4096, 0, 0).mesh;
    authored.id = 201;
    badCell.navMeshes = {authored};
    std::vector<app::detail::BatchCellResult> results{
        {.cell = &goodCell, .candidate = Floor(0, 0, 0), .status = "generated"},
        {.cell = &badCell, .status = "skipped_generation_failed", .error = "synthetic generation failure"}};
    skyrim::ResolvedLoadOrder resolved;
    resolved.records = {{.type = "CELL", .formId = goodCell.id, .worldspaceFormId = 1},
                        {.type = "CELL", .formId = badCell.id, .worldspaceFormId = 1}};
    assert(app::detail::ReconcileBatchBorders(results, resolved).empty());
    const auto &links = results.front().candidate.borderLinks;
    assert(!links.empty() && std::all_of(links.begin(), links.end(), [](const auto &link)
                                         { return link.neighborNavmeshId == 201 && !link.generatedNeighborCell; }));
    assert(results.back().candidate.mesh.polygons.empty() && badCell.navMeshes.front().polygons.size() == 2);
    std::filesystem::remove_all(directory);
}

void TestBatchRecoveryBorderNeighbors()
{
    using namespace navmesh;
    for (const bool continueOnError : {true, false})
    {
        core::Cell left{.id = 11, .exteriorCoordinates = std::array<std::int32_t, 2>{0, 0}};
        core::Cell right{.id = 10, .exteriorCoordinates = std::array<std::int32_t, 2>{1, 0}};
        right.navMeshes = {Floor(4096, 0, 0).mesh};
        right.navMeshes.front().id = 201;
        skyrim::ResolvedLoadOrder resolved;
        resolved.cells = {left, right};
        resolved.records = {{.type = "CELL", .formId = left.id, .worldspaceFormId = 1},
                            {.type = "CELL", .formId = right.id, .worldspaceFormId = 1}};
        auto invalid = Floor(4096, 0, 0);
        invalid.mesh.vertices.front().z = std::numeric_limits<float>::infinity();
        auto good = Floor(0, 0, 0);
        good.mesh.vertices.push_back({0, 0, 100});
        std::vector<app::detail::BatchCellResult> results{
            {.cell = &left, .candidate = good, .status = "generated"},
            {.cell = &right, .candidate = invalid, .status = "generated"}};
        const auto error = app::detail::ReconcileBatchBorders(results, resolved, continueOnError);
        if (continueOnError)
        {
            assert(error.empty() && results.back().status == "skipped_generation_failed");
            assert(results.back().candidate.mesh.polygons.empty() && results.back().auditPath.empty());
            assert(results.front().status == "generated" && results.front().candidate.topology.valid);
            assert(!results.front().candidate.borderLinks.empty());
            for (const auto &link : results.front().candidate.borderLinks)
            {
                assert(!link.generatedNeighborCell && link.neighborNavmeshId == 201);
            }
        }
        else
        {
            assert(!error.empty() && results.back().status == "generated");
            assert(results.front().candidate.mesh.vertices.size() == good.mesh.vertices.size());
            assert(results.front().candidate.borderLinks.empty());
        }

        // Worker-spooled snapshots restore the complete graph without duplicating the batch in memory.
        const auto directory =
            std::filesystem::temp_directory_path() /
            ("navmesh-reconciliation-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        core::Scene evidence;
        evidence.geometrySources.resize(1);
        evidence.triangleProvenance.resize(9);
        results = {{.cell = &left, .candidate = good, .status = "generated", .auditPath = directory / "left.gz"},
                   {.cell = &right, .candidate = invalid, .status = "generated", .auditPath = directory / "right.gz"}};
        assert(app::detail::StoreCandidate(results.front().auditPath, Floor(0, 0, 0), evidence));
        assert(app::detail::StoreCandidate(results.back().auditPath, Floor(4096, 0, 0), evidence));
        const auto spooledError = app::detail::ReconcileBatchBorders(results, resolved, continueOnError);
        assert(spooledError.empty() == continueOnError);
        assert(results.front().candidate.topology.valid && results.front().candidate.mesh.vertices.size() == 4);
        if (continueOnError)
        {
            assert(results.back().status == "skipped_generation_failed" && results.back().auditPath.empty());
            assert(!results.front().candidate.borderLinks.empty());
        }
        else
        {
            assert(results.back().status == "generated" && results.back().candidate.topology.valid);
            assert(std::isfinite(results.back().candidate.mesh.vertices.front().z));
            assert(results.front().candidate.borderLinks.empty());
        }
        // Evidence loss must prevent recovery and publication rather than accepting partially restored state.
        results.back().candidate = invalid;
        results.back().status = "generated";
        results.back().auditPath = directory / "right.gz";
        std::ofstream(results.front().auditPath, std::ios::trunc) << "corrupt";
        assert(!app::detail::ReconcileBatchBorders(results, resolved, true).empty());
        std::filesystem::remove_all(directory);
    }
    for (const bool westAuthored : {true, false})
    {
        for (const bool eastAuthored : {true, false})
        {
            core::Cell goodCell{.id = 11, .exteriorCoordinates = std::array<std::int32_t, 2>{0, 0}};
            core::Cell badCell{.id = 10, .exteriorCoordinates = std::array<std::int32_t, 2>{1, 0}};
            core::Cell westCell{.id = 12, .exteriorCoordinates = std::array<std::int32_t, 2>{-1, 0}};
            auto westMesh = Floor(-4096, 0, 0).mesh;
            westMesh.id = 202;
            if (westAuthored)
            {
                westCell.navMeshes.push_back(westMesh);
            }
            auto eastMesh = Floor(4096, 0, 0).mesh;
            eastMesh.id = 201;
            if (eastAuthored)
            {
                badCell.navMeshes.push_back(eastMesh);
            }
            auto candidate = Floor(0, 0, 0);
            const core::AABB bounds{.min = {0, 0, -100}, .max = {4096, 4096, 100}};
            if (westAuthored)
            {
                (void)core::StitchCandidateBorders(candidate, bounds, westCell.navMeshes, {}, true);
                assert(candidate.topology.valid && candidate.borderLinks.size() == 1);
            }
            const auto polygons = candidate.mesh.polygons.size();
            skyrim::ResolvedLoadOrder resolved;
            resolved.cells = {goodCell, badCell, westCell};
            resolved.records = {{.type = "CELL", .formId = goodCell.id, .worldspaceFormId = 1},
                                {.type = "CELL", .formId = badCell.id, .worldspaceFormId = 1},
                                {.type = "CELL", .formId = westCell.id, .worldspaceFormId = 1}};
            std::vector<app::detail::BatchCellResult> results{
                {.cell = &goodCell, .candidate = std::move(candidate), .status = "generated"},
                {.cell = &badCell, .status = "skipped_generation_failed", .error = "synthetic generation failure"}};
            assert(app::detail::ReconcileBatchBorders(results, resolved).empty());
            const auto &recovered = results.front().candidate;
            assert(recovered.topology.valid && recovered.mesh.polygons.size() == polygons);
            std::set<std::uint32_t> targets;
            for (const auto &link : recovered.borderLinks)
            {
                assert(!link.generatedNeighborCell);
                targets.insert(link.neighborNavmeshId);
            }
            std::set<std::uint32_t> expected;
            if (westAuthored)
            {
                expected.insert(westMesh.id);
            }
            if (eastAuthored)
            {
                expected.insert(eastMesh.id);
            }
            assert(targets == expected && recovered.borderLinks.size() == expected.size());
            assert(goodCell.navMeshes.empty() && results.back().candidate.mesh.polygons.empty());
        }
    }
}
