#ifdef NDEBUG
#undef NDEBUG
#endif

#include "core/navmesh/candidate.h"
#include "core/navmesh/generator.h"
#include "core/navmesh/surface_simplification.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>

namespace
{
    using namespace navmesh::core;

    void AddFace(NavMesh &mesh, std::array<std::uint32_t, 3> vertices, std::uint16_t flags = 0)
    {
        NavPolygon face;
        face.vertices = vertices;
        face.neighbors.fill(std::numeric_limits<std::uint32_t>::max());
        face.flags = flags;
        mesh.polygons.push_back(face);
    }

    NavMesh Grid(bool opening = false, float offset = 0)
    {
        NavMesh mesh;
        for (int y{}; y <= 10; ++y)
        {
            for (int x{}; x <= 10; ++x)
            {
                mesh.vertices.push_back({offset + x * 16.0F, offset + y * 16.0F, x * 2.0F + y * 4.0F});
            }
        }
        for (std::uint32_t y{}; y < 10; ++y)
        {
            for (std::uint32_t x{}; x < 10; ++x)
            {
                // A hole and two rooms joined by a narrow strip must retain their complete footprint.
                if (opening && ((x >= 2 && x < 4 && y >= 2 && y < 4) || (x == 6 && y != 5)))
                {
                    continue;
                }
                const auto a = x + y * 11;
                AddFace(mesh, {a, a + 1, a + 12});
                AddFace(mesh, {a, a + 12, a + 11});
            }
        }
        return mesh;
    }

    double Area(const NavMesh &mesh)
    {
        double area{};
        for (const auto &face : mesh.polygons)
        {
            const auto a = mesh.vertices[face.vertices[0]];
            const auto b = mesh.vertices[face.vertices[1]];
            const auto c = mesh.vertices[face.vertices[2]];
            const double cross = (static_cast<double>(b.x) - a.x) * (static_cast<double>(c.y) - a.y) -
                                 (static_cast<double>(b.y) - a.y) * (static_cast<double>(c.x) - a.x);
            assert(cross > 0);
            area += cross / 2;
        }
        return area;
    }

    std::optional<double> Height(const NavMesh &mesh, float x, float y)
    {
        for (const auto &face : mesh.polygons)
        {
            const auto a = mesh.vertices[face.vertices[0]];
            const auto b = mesh.vertices[face.vertices[1]];
            const auto c = mesh.vertices[face.vertices[2]];
            const double denominator = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            const double u = ((x - a.x) * (c.y - a.y) - (y - a.y) * (c.x - a.x)) / denominator;
            const double v = ((b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x)) / denominator;
            if (u >= 0 && v >= 0 && u + v <= 1)
            {
                return a.z + u * (b.z - a.z) + v * (c.z - a.z);
            }
        }
        return {};
    }

    void CheckTopology(const NavMesh &mesh, std::size_t expectedComponents = 1)
    {
        CandidateNavMesh candidate;
        candidate.mesh = mesh;
        candidate.polygonSourceTriangles.resize(mesh.polygons.size());
        candidate.polygonContributingTriangles.resize(mesh.polygons.size(), {0});
        CandidateRegion region;
        for (std::uint32_t index{}; index < mesh.polygons.size(); ++index)
        {
            region.polygons.push_back(index);
        }
        candidate.regions.push_back(region);
        RefreshCandidateTopology(candidate);
        assert(candidate.topology.valid);
        for (const auto &face : candidate.mesh.polygons)
        {
            for (const auto neighbor : face.neighbors)
            {
                assert(neighbor == std::numeric_limits<std::uint32_t>::max() || neighbor < mesh.polygons.size());
            }
        }
        std::vector<bool> seen(mesh.polygons.size());
        std::size_t components{};
        for (std::size_t index{}; index < mesh.polygons.size(); ++index)
        {
            if (seen[index])
            {
                continue;
            }
            ++components;
            std::queue<std::size_t> pending;
            pending.push(index);
            seen[index] = true;
            while (!pending.empty())
            {
                const auto current = pending.front();
                pending.pop();
                for (const auto neighbor : candidate.mesh.polygons[current].neighbors)
                {
                    if (neighbor < seen.size() && !seen[neighbor])
                    {
                        pending.push(neighbor);
                        seen[neighbor] = true;
                    }
                }
            }
        }
        assert(components == expectedComponents);
    }

    void CheckGrid(bool opening, float offset)
    {
        const auto original = Grid(opening, offset);
        auto simplified = original;
        const auto removed = SimplifyGeneratedSurface(simplified);
        assert(removed > original.polygons.size() / 2);
        assert(Area(simplified) == Area(original));
        CheckTopology(simplified);
        // Probe both retained rooms, the narrow connection and empty obstacle regions.
        for (float y = 3; y < 160; y += 7)
        {
            for (float x = 5; x < 160; x += 7)
            {
                const auto before = Height(original, offset + x, offset + y);
                const auto after = Height(simplified, offset + x, offset + y);
                assert(before.has_value() == after.has_value());
                assert(!before || std::abs(*before - *after) < 1e-9);
            }
        }
        std::cout << "Planar surface opening=" << opening << " offset=" << offset << ": " << original.polygons.size()
                  << " -> " << simplified.polygons.size() << " triangles\n";
    }

    NavMesh BoundarySliver()
    {
        NavMesh mesh;
        mesh.vertices = {{0, 0, 0}, {0.125F, 0, 0}, {16, 0, 0}, {16, 16, 0}, {0, 16, 0}};
        AddFace(mesh, {0, 1, 4});
        AddFace(mesh, {1, 3, 4});
        AddFace(mesh, {1, 2, 3});
        return mesh;
    }

    void CheckProtectedDetail()
    {
        auto sliver = BoundarySliver();
        assert(SimplifyGeneratedSurface(sliver) == 1);
        assert(sliver.polygons.size() == 2 && Area(sliver) == 256);
        CheckTopology(sliver);

        for (const bool bend : {false, true})
        {
            auto detailed = BoundarySliver();
            if (bend)
            {
                detailed.vertices[1].z = 0.01F;
            }
            else
            {
                detailed.vertices[1].y = -0.01F;
            }
            assert(SimplifyGeneratedSurface(detailed) == 0);
        }
        auto seam = BoundarySliver();
        const AABB bounds{.min = {0, 0, -10}, .max = {16, 16, 10}};
        assert(SimplifyGeneratedSurface(seam, bounds) == 0);
        auto limited = BoundarySliver();
        assert(SimplifyGeneratedSurface(limited, {}, {.maximumBoundaryEdgeLength = 8}) == 0);
        auto mixed = BoundarySliver();
        mixed.polygons.front().flags = WaterFlag;
        assert(SimplifyGeneratedSurface(mixed) == 0 && mixed.polygons.front().flags == WaterFlag);

        NavMesh peak;
        peak.vertices = {{8, 8, 0.01F}, {0, 0, 0}, {16, 0, 0}, {16, 16, 0}, {0, 16, 0}};
        for (std::uint32_t side = 1; side <= 4; ++side)
        {
            AddFace(peak, {0, side, side % 4 + 1});
        }
        assert(SimplifyGeneratedSurface(peak) == 0);

        // Vertex-only contact cannot be replaced by a fan that invents shared-edge connectivity.
        NavMesh touching;
        touching.vertices = {{0, 0, 0}, {16, 0, 0}, {0, 16, 0}, {-16, 0, 0}, {0, -16, 0}};
        AddFace(touching, {0, 1, 2});
        AddFace(touching, {0, 3, 4});
        assert(SimplifyGeneratedSurface(touching) == 0);

        // A nonmanifold edge protects all of its incident samples.
        auto ambiguous = BoundarySliver();
        AddFace(ambiguous, {1, 3, 4});
        assert(SimplifyGeneratedSurface(ambiguous) == 0);

        auto linked = BoundarySliver();
        linked.doorLinks.push_back({.polygon = 0, .referenceId = 1});
        bool rejected{};
        try
        {
            SimplifyGeneratedSurface(linked);
        }
        catch (const std::invalid_argument &)
        {
            rejected = true;
        }
        assert(rejected && linked.polygons.size() == 3);
    }

    void CheckBoundedApproximation()
    {
        auto original = Grid(true, 1000000);
        for (std::size_t vertex{}; vertex < original.vertices.size(); ++vertex)
        {
            original.vertices[vertex].z += vertex % 2 ? 0.4F : -0.4F;
        }
        auto simplified = original;
        constexpr float error = 1;
        const auto removed =
            SimplifyGeneratedSurface(simplified, {}, {.maximumHeightError = error, .maximumSlopeDegrees = 45});
        assert(removed > original.polygons.size() / 3);
        assert(Area(simplified) == Area(original));
        CheckTopology(simplified);
        // Compare against immutable input over many passes, including removed height samples.
        for (float y{}; y <= 160; y += 2)
        {
            for (float x{}; x <= 160; x += 2)
            {
                const auto before = Height(original, 1000000 + x, 1000000 + y);
                const auto after = Height(simplified, 1000000 + x, 1000000 + y);
                assert(before.has_value() == after.has_value());
                assert(!before || std::abs(*before - *after) <= error);
            }
        }
        std::cout << "Noisy surface: " << original.polygons.size() << " -> " << simplified.polygons.size()
                  << " triangles, bounded height error " << error << '\n';

        auto boundary = BoundarySliver();
        boundary.vertices[1].z = 0.01F;
        assert(SimplifyGeneratedSurface(boundary, {}, {.maximumHeightError = error, .maximumSlopeDegrees = 45}) == 0);
        NavMesh peak;
        peak.vertices = {{8, 8, 4}, {0, 0, 0}, {16, 0, 0}, {16, 16, 0}, {0, 16, 0}};
        for (std::uint32_t side = 1; side <= 4; ++side)
        {
            AddFace(peak, {0, side, side % 4 + 1});
        }
        assert(SimplifyGeneratedSurface(peak, {}, {.maximumHeightError = error, .maximumSlopeDegrees = 45}) == 0);

        auto stacked = Grid();
        const auto upper = Grid();
        const auto vertexOffset = static_cast<std::uint32_t>(stacked.vertices.size());
        for (auto point : upper.vertices)
        {
            point.z += 128;
            stacked.vertices.push_back(point);
        }
        for (auto face : upper.polygons)
        {
            for (auto &vertex : face.vertices)
            {
                vertex += vertexOffset;
            }
            stacked.polygons.push_back(face);
        }
        assert(SimplifyGeneratedSurface(stacked, {}, {.maximumHeightError = error, .maximumSlopeDegrees = 45}) > 0);
        CheckTopology(stacked, 2);
        for (const auto &face : stacked.polygons)
        {
            const auto below = stacked.vertices[face.vertices[0]].z < 100;
            for (const auto vertex : face.vertices)
            {
                assert((stacked.vertices[vertex].z < 100) == below);
            }
        }
    }

    void CheckBoundaryEars()
    {
        const SurfaceSimplificationSettings settings{.maximumBoundaryInset = 2};
        auto tip = BoundarySliver();
        tip.vertices[1].x = 8;
        tip.vertices[1].y = -0.5F;
        const auto original = tip;
        assert(SimplifyGeneratedSurface(tip, {}, settings) == 1);
        assert(tip.polygons.size() == 2 && Area(original) - Area(tip) == 4);
        CheckTopology(tip);
        assert(Height(tip, 8, 8) && !Height(tip, 8, -0.25F));

        auto concave = original;
        concave.vertices[1].y = 0.5F;
        assert(SimplifyGeneratedSurface(concave, {}, settings) == 0);
        auto deep = original;
        deep.vertices[1].y = -3;
        assert(SimplifyGeneratedSurface(deep, {}, settings) == 0);
        auto pinned = original;
        const std::array<std::uint32_t, 1> pins{1};
        assert(SimplifyGeneratedSurface(pinned, {}, settings, pins) == 0);
        auto bent = original;
        bent.vertices[1].z = 0.01F;
        assert(SimplifyGeneratedSurface(bent, {}, settings) == 0);

        NavMesh leaf;
        leaf.vertices = {{8, -0.5F, 0}, {16, 0, 0}, {0, 0, 0}, {0, 16, 0}};
        AddFace(leaf, {0, 1, 2});
        AddFace(leaf, {2, 1, 3});
        assert(SimplifyGeneratedSurface(leaf, {}, settings) == 1 && leaf.polygons.size() == 1);
        CheckTopology(leaf);
        NavMesh island;
        island.vertices = {{8, -0.5F, 0}, {16, 0, 0}, {0, 0, 0}};
        AddFace(island, {0, 1, 2});
        assert(SimplifyGeneratedSurface(island, {}, settings) == 0);

        // Many shallow tips on one edge cannot gradually consume the room or seal its narrow connection.
        auto noisyBoundary = Grid(true);
        for (std::size_t vertex = 1; vertex < 10; ++vertex)
        {
            noisyBoundary.vertices[vertex].y -= vertex % 2 ? 0.5F : 0.25F;
            noisyBoundary.vertices[vertex].z =
                noisyBoundary.vertices[vertex].x / 8 + noisyBoundary.vertices[vertex].y / 4;
        }
        const auto before = noisyBoundary;
        assert(SimplifyGeneratedSurface(noisyBoundary, {}, settings) > 0);
        CheckTopology(noisyBoundary);
        assert(Area(noisyBoundary) <= Area(before));
        for (float y = 1; y < 160; y += 5)
        {
            for (float x = 1; x < 160; x += 5)
            {
                assert(Height(noisyBoundary, x, y).has_value() == Height(before, x, y).has_value());
            }
        }
    }

    void CheckPinnedRim()
    {
        // All boundary samples are protected, but the redundant interior sample is removable.
        NavMesh mesh;
        mesh.vertices = {{0, 0, 0}, {64, 0, 8}, {64, 64, 16}, {0, 64, 8}, {32, 32, 8}};
        for (std::uint32_t side{}; side < 4; ++side)
        {
            AddFace(mesh, {side, (side + 1) % 4, 4});
        }
        const auto original = mesh;
        const std::array<std::uint32_t, 4> pins{0, 1, 2, 3};
        assert(SimplifyGeneratedSurface(mesh, {}, {}, pins) == 2);
        assert(mesh.vertices.size() == pins.size());
        assert(Area(mesh) == Area(original));
        for (const auto vertex : pins)
        {
            const auto point = original.vertices[vertex];
            assert(std::any_of(mesh.vertices.begin(), mesh.vertices.end(), [&](auto retained)
                               { return retained.x == point.x && retained.y == point.y && retained.z == point.z; }));
        }
        for (float y = 1; y < 64; y += 4)
        {
            for (float x = 1; x < 64; x += 4)
            {
                assert(Height(mesh, x, y) == Height(original, x, y));
            }
        }
        CheckTopology(mesh);
    }

    void CheckFoldedFloorReduction()
    {
        // A redundant sample on a ridge can be removed without flattening either side of the floor.
        NavMesh original;
        original.vertices = {{8, 8, 8}, {0, 0, 0}, {16, 0, 8}, {16, 16, 0}, {0, 16, 8}};
        for (std::uint32_t side = 1; side <= 4; ++side)
        {
            AddFace(original, {0, side, side % 4 + 1});
        }
        auto simplified = original;
        assert(SimplifyGeneratedSurface(simplified, {}, {.maximumHeightError = 1}) == 2);
        CheckTopology(simplified);
        assert(Area(simplified) == Area(original));
        for (float y{}; y <= 16; y += 0.5F)
        {
            for (float x{}; x <= 16; x += 0.5F)
            {
                assert(Height(original, x, y) == Height(simplified, x, y));
            }
        }
        auto peak = original;
        peak.vertices[0].z = 12;
        assert(SimplifyGeneratedSurface(peak, {}, {.maximumHeightError = 1}) == 0);

        // Matching every input vertex is insufficient when the new diagonal crosses an old ridge.
        auto crossing = original;
        crossing.vertices[0] = {4, 8, 6};
        const auto crossingInput = crossing;
        assert(SimplifyGeneratedSurface(crossing, {}, {.maximumHeightError = 1}) == 0);
        assert(SimplifyGeneratedSurface(crossing, {}, {.maximumHeightError = 4}) == 2);
        for (float y{}; y <= 16; y += 0.5F)
        {
            for (float x{}; x <= 16; x += 0.5F)
            {
                assert(std::abs(*Height(crossingInput, x, y) - *Height(crossing, x, y)) <= 4);
            }
        }
    }

    void CheckCandidateTriangleReduction(bool opening)
    {
        CandidateNavMesh candidate;
        candidate.mesh = Grid(opening);
        for (std::size_t vertex{}; vertex < candidate.mesh.vertices.size(); ++vertex)
        {
            candidate.mesh.vertices[vertex].z += vertex % 2 ? 0.4F : -0.4F;
        }
        // Omitted grid squares leave unused input samples; candidates require compact vertex storage.
        const auto inputVertices = std::move(candidate.mesh.vertices);
        std::map<std::uint32_t, std::uint32_t> remap;
        for (auto &face : candidate.mesh.polygons)
        {
            for (auto &vertex : face.vertices)
            {
                const auto [found, inserted] = remap.emplace(vertex, static_cast<std::uint32_t>(remap.size()));
                if (inserted)
                {
                    candidate.mesh.vertices.push_back(inputVertices[vertex]);
                }
                vertex = found->second;
            }
        }
        CandidateRegion region{.id = 0, .sourceTriangles = {0}};
        for (std::uint32_t index{}; index < candidate.mesh.polygons.size(); ++index)
        {
            region.polygons.push_back(index);
            candidate.polygonSourceTriangles.push_back(0);
            candidate.polygonContributingTriangles.push_back({0});
        }
        candidate.regions.push_back(region);
        RefreshCandidateTopology(candidate);
        const auto original = candidate.mesh;
        assert(candidate.topology.valid);
        assert(SimplifyCandidateSurface(candidate) > original.polygons.size() * 2 / 3);
        assert(candidate.topology.valid);
        CheckTopology(candidate.mesh);
        assert(Area(candidate.mesh) >= Area(original) * 0.9);
        if (!opening)
        {
            assert(Area(candidate.mesh) == Area(original));
        }
        const auto heightBudget = candidate.recastSettings.cellHeight;
        for (float y = 16; y < 144; y += 2)
        {
            for (float x = 16; x < 144; x += 2)
            {
                const auto before = Height(original, x, y);
                const auto after = Height(candidate.mesh, x, y);
                assert(!after || (before && std::abs(*before - *after) <= heightBudget));
            }
        }
        // Both rooms retain their shared narrow route at the production trimming budget.
        for (float x = 16; x < 144; x += 2)
        {
            assert(Height(candidate.mesh, x, 88));
        }
        if (opening)
        {
            assert(!Height(candidate.mesh, 104, 40) && !Height(candidate.mesh, 104, 120));
        }
        std::cout << "Candidate floor reduction: " << original.polygons.size() << " -> "
                  << candidate.mesh.polygons.size() << " triangles\n";
    }

    void CheckCandidateRemapping()
    {
        CandidateNavMesh candidate;
        candidate.mesh = Grid();
        CandidateRegion region{.id = 0, .sourceTriangles = {0, 1}, .geometrySources = {0}, .exitFormIds = {10}};
        for (std::uint32_t index{}; index < candidate.mesh.polygons.size(); ++index)
        {
            region.polygons.push_back(index);
            candidate.polygonSourceTriangles.push_back(index % 2);
            candidate.polygonContributingTriangles.push_back({index % 2});
        }
        candidate.regions.push_back(region);
        candidate.exits.push_back({.referenceId = 10, .position = {1, 1, 0}, .region = 0, .polygon = 0});
        candidate.borderLinks.push_back(
            {.polygon = 2, .edge = 0, .neighborNavmeshId = 100, .neighborPolygon = 7, .neighborEdge = 1});
        RefreshCandidateTopology(candidate);
        assert(candidate.topology.valid);
        const auto points = [&](std::uint32_t polygon)
        {
            const auto &face = candidate.mesh.polygons.at(polygon);
            return std::array{candidate.mesh.vertices.at(face.vertices[0]),
                              candidate.mesh.vertices.at(face.vertices[1]),
                              candidate.mesh.vertices.at(face.vertices[2])};
        };
        const auto door = points(0);
        const auto portal = points(2);
        assert(SimplifyCandidateSurface(candidate) > 0);
        assert(candidate.topology.valid && candidate.exits.front().polygon && candidate.borderLinks.size() == 1);
        const auto samePoints = [](const auto &before, const auto &after)
        {
            for (std::size_t index{}; index < before.size(); ++index)
            {
                assert(before[index].x == after[index].x && before[index].y == after[index].y &&
                       before[index].z == after[index].z);
            }
        };
        samePoints(door, points(*candidate.exits.front().polygon));
        samePoints(portal, points(candidate.borderLinks.front().polygon));
        assert(candidate.borderLinks.front().neighborPolygon == 7 && candidate.borderLinks.front().edge == 0);
        assert(candidate.polygonSourceTriangles.size() == candidate.mesh.polygons.size());
        assert(candidate.polygonContributingTriangles.size() == candidate.mesh.polygons.size());
        assert(candidate.regions.front().polygons.size() == candidate.mesh.polygons.size());
        assert(candidate.regions.front().sourceTriangles == std::vector<std::size_t>({0, 1}));
        // Reciprocal generated triangle identities are coordinated by the batch linker, not a cell-local pass.
        candidate.borderLinks.front().generatedNeighborCell = 20;
        const auto count = candidate.mesh.polygons.size();
        assert(SimplifyCandidateSurface(candidate) == 0 && candidate.mesh.polygons.size() == count);
    }

    void CheckBoundaryBand()
    {
        // A long, thin leaf can follow several original segments, even with uneven heights.
        NavMesh leaf;
        leaf.vertices = {{128, -0.5F, 128}, {256, 0, 0}, {0, 0, 0}, {0, 256, 0}};
        AddFace(leaf, {0, 1, 2});
        AddFace(leaf, {2, 1, 3});
        auto limited = leaf;
        assert(SimplifyGeneratedSurface(limited, {}, {.maximumBoundaryEdgeLength = 64, .maximumBoundaryInset = 2}) ==
               0);
        assert(SimplifyGeneratedSurface(leaf, {}, {.maximumBoundaryInset = 2}) == 1);
        assert(leaf.polygons.size() == 1 && Area(leaf) == 32768);
        CheckTopology(leaf);

        // All triangle corners can be near different boundaries while its interior is far from all of them.
        NavMesh room;
        room.vertices = {{0, 0, 0}, {100, 0, 0}, {100, 100, 0}, {0, 100, 0}, {50, 98, 10}};
        for (std::uint32_t side{}; side < 4; ++side)
        {
            AddFace(room, {side, (side + 1) % 4, 4});
        }
        const auto centerHeight = Height(room, 50, 50);
        assert(SimplifyGeneratedSurface(room, {}, {.maximumBoundaryInset = 2}) == 1);
        assert(Area(room) == 9900 && Height(room, 50, 50) == centerHeight);
        CheckTopology(room);
    }

    void CheckUnevenObstacleBoundary()
    {
        // Alternate rock-outline recesses on an uneven floor, with a pinned outer inspection boundary.
        constexpr std::uint32_t samples = 48;
        NavMesh original;
        std::vector<std::uint32_t> pinned;
        for (std::uint32_t ring{}; ring < 3; ++ring)
        {
            for (std::uint32_t sample{}; sample < samples; ++sample)
            {
                const auto angle = sample * 2 * std::acos(-1.0) / samples;
                const auto radius = ring == 0 ? (sample % 2 ? 24.0 : 32.0) : (ring == 1 ? 96.0 : 200.0);
                const auto x = static_cast<float>(radius * std::cos(angle));
                const auto y = static_cast<float>(radius * std::sin(angle));
                const auto noise = ring == 0 && sample % 2 ? 0.2F : 0.0F;
                original.vertices.push_back({x, y, x / 16 + y / 32 + noise});
                if (ring == 2)
                {
                    pinned.push_back(ring * samples + sample);
                }
            }
        }
        for (std::uint32_t ring{}; ring < 2; ++ring)
        {
            for (std::uint32_t sample{}; sample < samples; ++sample)
            {
                const auto next = (sample + 1) % samples;
                const auto inner = ring * samples;
                const auto outer = inner + samples;
                AddFace(original, {inner + sample, outer + sample, outer + next});
                AddFace(original, {inner + sample, outer + next, inner + next});
            }
        }
        auto simplified = original;
        const SurfaceSimplificationSettings settings{.maximumHeightError = 1,
                                                     .maximumSlopeDegrees = 45,
                                                     .maximumBoundaryInset = 16,
                                                     .maximumBoundaryAreaFraction = 0.02F};
        assert(SimplifyGeneratedSurface(simplified, {}, settings, pinned) >= samples / 2);
        CheckTopology(simplified);
        assert(Area(simplified) <= Area(original));
        assert(Area(simplified) >= Area(original) * (1 - settings.maximumBoundaryAreaFraction));
        for (float y = -198; y < 200; y += 3)
        {
            for (float x = -198; x < 200; x += 3)
            {
                const auto before = Height(original, x, y);
                const auto after = Height(simplified, x, y);
                assert(!after || before);
                assert(!after || std::abs(*before - *after) <= settings.maximumHeightError);
                if (std::hypot(x, y) > 32 + settings.maximumBoundaryInset)
                {
                    assert(before.has_value() == after.has_value());
                }
            }
        }
        std::cout << "Uneven obstacle boundary: " << original.polygons.size() << " -> " << simplified.polygons.size()
                  << " triangles\n";

        // A reserved drop between stacked floors must not freeze unrelated obstacle outlines.
        CandidateNavMesh linked;
        for (std::uint32_t floor{}; floor < 2; ++floor)
        {
            const auto offset = static_cast<std::uint32_t>(linked.mesh.vertices.size());
            for (const auto point : original.vertices)
            {
                linked.mesh.vertices.push_back({point.x, point.y, point.z - floor * 160.0F});
            }
            CandidateRegion region{.id = floor};
            for (const auto &face : original.polygons)
            {
                region.polygons.push_back(static_cast<std::uint32_t>(linked.mesh.polygons.size()));
                AddFace(linked.mesh, {face.vertices[0] + offset, face.vertices[1] + offset, face.vertices[2] + offset});
                linked.polygonSourceTriangles.push_back(floor);
                linked.polygonContributingTriangles.push_back({floor});
            }
            linked.regions.push_back(std::move(region));
        }
        linked.ledgeLinks.push_back({.upperPolygon = 1,
                                     .lowerPolygon = static_cast<std::uint32_t>(original.polygons.size() + 1),
                                     .upperEdge = 2,
                                     .lowerEdge = 2});
        RefreshCandidateTopology(linked);
        assert(linked.topology.valid);
        const auto portPoints = [&]()
        {
            std::vector<Vec3> points;
            const auto &link = linked.ledgeLinks.front();
            for (const auto polygon : {link.upperPolygon, link.lowerPolygon})
            {
                for (const auto vertex : linked.mesh.polygons.at(polygon).vertices)
                {
                    points.push_back(linked.mesh.vertices.at(vertex));
                }
            }
            return points;
        };
        const auto ports = portPoints();
        const auto linkedArea = Area(linked.mesh);
        assert(SimplifyCandidateSurface(linked) >= samples);
        assert(linked.topology.valid && linked.ledgeLinks.size() == 1);
        CheckTopology(linked.mesh, 2);
        const auto retainedPorts = portPoints();
        for (std::size_t index{}; index < ports.size(); ++index)
        {
            assert(ports[index].x == retainedPorts[index].x && ports[index].y == retainedPorts[index].y &&
                   ports[index].z == retainedPorts[index].z);
        }
        assert(Area(linked.mesh) <= linkedArea && Area(linked.mesh) >= linkedArea * 0.9);
    }
    void CheckNarrowPassage()
    {
        auto mesh = Grid(true);
        assert(SimplifyGeneratedSurface(
                   mesh, {},
                   {.maximumHeightError = 1, .maximumBoundaryInset = 32, .maximumBoundaryAreaFraction = 0.02F}) > 0);
        CheckTopology(mesh);
        // A passage narrower than the trimming band still connects both rooms.
        for (float x = 1; x < 160; x += 2)
        {
            assert(Height(mesh, x, 88));
        }
        assert(!Height(mesh, 104, 40) && !Height(mesh, 104, 120));
    }

} // namespace

void TestSurfaceSimplification()
{
    CheckGrid(false, 0);
    CheckGrid(true, 0);
    CheckGrid(true, 1000000);
    CheckProtectedDetail();
    CheckBoundedApproximation();
    CheckFoldedFloorReduction();
    CheckCandidateTriangleReduction(false);
    CheckCandidateTriangleReduction(true);
    CheckBoundaryEars();
    CheckPinnedRim();
    CheckCandidateRemapping();
    CheckBoundaryBand();
    CheckUnevenObstacleBoundary();
    CheckNarrowPassage();
}
