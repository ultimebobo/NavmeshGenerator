#ifdef NDEBUG
#undef NDEBUG
#endif

#include "core/navmesh/candidate.h"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <limits>
#include <numeric>

namespace
{
    using namespace navmesh::core;
    constexpr auto Open = std::numeric_limits<std::uint32_t>::max();

    CandidateNavMesh MakeFloor(std::vector<Vec3> vertices, std::vector<std::array<std::uint32_t, 3>> faces)
    {
        CandidateNavMesh candidate;
        candidate.mesh.vertices = std::move(vertices);
        for (const auto &face : faces)
        {
            const auto index = candidate.mesh.polygons.size();
            candidate.mesh.polygons.push_back({.vertices = face, .neighbors = {Open, Open, Open}, .flags = WaterFlag});
            candidate.polygonSourceTriangles.push_back(index);
            candidate.polygonContributingTriangles.push_back({index});
        }
        candidate.regions = {{.id = 0}};
        candidate.regions.front().polygons.resize(faces.size());
        std::iota(candidate.regions.front().polygons.begin(), candidate.regions.front().polygons.end(), 0U);
        RefreshCandidateTopology(candidate);
        assert(candidate.topology.valid);
        return candidate;
    }

    NavMesh MakePortal(std::uint32_t id, Vec3 first, Vec3 last, Vec3 opposite)
    {
        return {.id = id,
                .vertices = {first, last, opposite},
                .polygons = {{.vertices = {0, 1, 2}, .neighbors = {Open, Open, Open}}}};
    }

    void CheckPortal(const CandidateNavMesh &candidate, const NavMesh &neighbor)
    {
        assert(candidate.topology.valid);
        assert(ValidateCandidateTopology(candidate).valid);
        const auto portal = std::find_if(candidate.borderLinks.begin(), candidate.borderLinks.end(),
                                         [&](const auto &link) { return link.neighborNavmeshId == neighbor.id; });
        assert(portal != candidate.borderLinks.end());
        const auto &face = candidate.mesh.polygons[portal->polygon];
        const auto first = candidate.mesh.vertices[face.vertices[portal->edge]];
        const auto last = candidate.mesh.vertices[face.vertices[(portal->edge + 1) % 3]];
        const auto a = neighbor.vertices[1];
        const auto b = neighbor.vertices[0];
        assert(first.x == a.x && first.y == a.y && first.z == a.z);
        assert(last.x == b.x && last.y == b.y && last.z == b.z);
        assert(candidate.mesh.polygons[portal->polygon].flags == WaterFlag);
        assert(!candidate.polygonContributingTriangles[portal->polygon].empty());
    }

    void CheckTerminalCavities()
    {
        const AABB bounds{{0, 0, -100}, {300, 300, 100}};
        for (const bool turning : {false, true})
        {
            auto candidate =
                turning ? MakeFloor({{20, 0, 0}, {120, 0, 0}, {220, 100, 0}, {20, 150, 0}}, {{0, 1, 3}, {1, 2, 3}})
                        : MakeFloor({{20, 0, 0}, {250, 0, 0}, {150, 150, 0}}, {{0, 1, 2}});
            auto authored = MakePortal(0x200, {0, 0, 0}, {170, 0, 0}, {100, 100, 0});
            auto neighbor = MakePortal(0x201, authored.vertices[1], authored.vertices[0], {100, -100, 0});
            authored.externalLinks = {{0, 0, neighbor.id, 0}};
            neighbor.externalLinks = {{0, 0, authored.id, 0}};
            assert(StitchCandidateBorders(candidate, bounds, {neighbor}, {authored}) == 1);
            CheckPortal(candidate, neighbor);
            assert(StitchCandidateBorders(candidate, bounds, {neighbor}, {authored}) == 0);
            CheckPortal(candidate, neighbor);
        }
    }

    void CheckExcludedAuthoredFloor()
    {
        const AABB bounds{{0, 0, -100}, {300, 300, 200}};
        const auto original = MakeFloor({{0, 0, 0}, {300, 0, 0}, {150, 200, 0}}, {{0, 1, 2}});
        auto authored = MakePortal(0x200, {20, 0, 100}, {280, 0, 100}, {150, 150, 100});
        auto neighbor = MakePortal(0x201, authored.vertices[1], authored.vertices[0], {150, -100, 100});
        authored.externalLinks = {{0, 0, neighbor.id, 0}};
        neighbor.externalLinks = {{0, 0, authored.id, 0}};
        for (const auto kind : {GeometrySourceType::Collision, GeometrySourceType::RenderFallback})
        {
            for (const bool obstacle : {false, true})
            {
                Scene scene;
                scene.mesh.vertices = {{0, 80, 100}, {300, 80, 100}, {150, 250, 100}};
                // Invalid source indices cannot shift the positive obstacle evidence join.
                scene.mesh.triangles = {{{0, 1, 9999}}, {{0, 1, 2}}};
                scene.geometrySources = {{.sourceType = kind, .navigationObstacle = obstacle}};
                scene.triangleProvenance = {{.geometrySource = 0}, {.geometrySource = 0}};
                auto candidate = original;
                candidate.exits = {{.referenceId = 0x300, .position = {150, 100, 0}, .region = 0, .polygon = 0}};
                assert(StitchCandidateBorders(candidate, bounds, {neighbor}, {authored}, false, &scene) == 0);
                const bool excluded = kind == GeometrySourceType::Collision && obstacle;
                assert(candidate.topology.valid);
                assert(!candidate.mesh.polygons.empty());
                const auto warning =
                    std::any_of(candidate.warnings.begin(), candidate.warnings.end(), [](const auto &message)
                                { return message.find("excluded obstacle collision") != std::string::npos; });
                assert(warning == excluded);
                assert(candidate.borderLinks.empty());
            }
        }
    }

    void CheckDeferredRequiredCrossing()
    {
        auto candidate = MakeFloor({{50, 50, -500}, {150, 50, -500}, {100, 150, -500}}, {{0, 1, 2}});
        auto authored = MakePortal(0x200, {50, 0, 0}, {150, 0, 0}, {100, 100, 0});
        auto neighbor = MakePortal(0x201, authored.vertices[1], authored.vertices[0], {100, -100, 0});
        authored.externalLinks = {{0, 0, neighbor.id, 0}};
        neighbor.externalLinks = {{0, 0, authored.id, 0}};
        assert(StitchCandidateBorders(candidate, {{0, 0, -1000}, {300, 300, 1000}}, {neighbor}, {authored}, true) == 0);
        assert(candidate.topology.valid && candidate.borderLinks.empty());
        assert(std::any_of(candidate.warnings.begin(), candidate.warnings.end(), [](const auto &warning)
                           { return warning.find("Dropped authored border portal") != std::string::npos; }));
    }

    void CheckAuthoredFanEdits()
    {
        const auto open = Open;
        NavMesh neighbor{.id = 0x201,
                         .vertices = {{0, 0, 0}, {64, 0, 0}, {0, 64, 0}, {64, 64, 0}},
                         .polygons = {{.vertices = {0, 1, 2}, .neighbors = {open, open, open}},
                                      {.vertices = {1, 3, 2}, .neighbors = {open, open, open}}}};
        const NavigationProfile profile;
        std::vector<AuthoredBorderVertexEdit> edits{{neighbor.id, 0, neighbor.vertices[0], {0, 0, 50}}};
        const std::array pinned{0U};
        auto unrelaxed = neighbor;
        assert(!ApplyAuthoredBorderVertexEdits(unrelaxed, edits, profile));
        assert(unrelaxed.vertices[0].z == 0);
        assert(CompleteAuthoredBorderFanEdits(neighbor, edits, pinned, profile));
        assert(edits.size() > 1);
        auto revised = neighbor;
        assert(ApplyAuthoredBorderVertexEdits(revised, edits, profile));
        assert(revised.vertices[0].z == 50 && revised.polygons[0].vertices == neighbor.polygons[0].vertices);
        auto conflict = edits;
        conflict.push_back({neighbor.id, 0, neighbor.vertices[0], {0, 0, 1}});
        assert(!ApplyAuthoredBorderVertexEdits(revised, conflict, profile));
        auto invalid = edits;
        invalid[0].position.z = std::numeric_limits<float>::infinity();
        auto unchanged = neighbor;
        assert(!ApplyAuthoredBorderVertexEdits(unchanged, invalid, profile));
        assert(unchanged.vertices[0].z == 0);
        std::vector<AuthoredBorderVertexEdit> blocked{{neighbor.id, 0, neighbor.vertices[0], {0, 0, 50}}};
        const std::array locked{0U, 1U, 2U, 3U};
        assert(!CompleteAuthoredBorderFanEdits(neighbor, blocked, locked, profile));
        assert(blocked.size() == 1);
    }

    void CheckSupportedAuthoredTerminalRepair()
    {
        auto candidate = MakeFloor({{64, 0, 0}, {384, 0, -18.6F}, {436.33F, 84.18F, -4.39F}}, {{0, 1, 2}});
        auto authored = MakePortal(0x200, {0, 0, -57.486F}, {64, 0, 0}, {64, 64, 2.35F});
        auto neighbor = MakePortal(0x201, authored.vertices[1], authored.vertices[0], {64, -128, -17.94F});
        neighbor.vertices.insert(neighbor.vertices.end(),
                                 {{384, 0, -18.6F}, {384, -128, -18.6F}, {-64, -64, -84.3F}, {-64, 0, -113.93F}});
        neighbor.polygons.push_back({.vertices = {3, 0, 4}, .neighbors = {Open, Open, Open}});
        neighbor.polygons.push_back({.vertices = {5, 1, 6}, .neighbors = {Open, Open, Open}});
        authored.externalLinks = {{0, 0, neighbor.id, 0}};
        neighbor.externalLinks = {{0, 0, authored.id, 0}};
        candidate.borderLinks = {{0, 0, neighbor.id, 1, 0}};
        Scene scene;
        scene.mesh.vertices = candidate.mesh.vertices;
        scene.mesh.triangles = {{{0, 1, 2}}};
        scene.geometrySources = {{.sourceType = GeometrySourceType::Collision}};
        scene.triangleProvenance = {{.geometrySource = 0}};
        const AABB bounds{{-200, 0, -500}, {500, 500, 500}};
        auto unsupported = candidate;
        auto renderOnly = scene;
        renderOnly.geometrySources.front().sourceType = GeometrySourceType::RenderFallback;
        (void)StitchCandidateBorders(unsupported, bounds, {neighbor}, {authored}, true, &renderOnly);
        assert(unsupported.topology.valid && unsupported.authoredBorderVertexEdits.empty());
        assert(std::any_of(unsupported.warnings.begin(), unsupported.warnings.end(), [](const auto &warning)
                           { return warning.find("Dropped authored border portal") != std::string::npos; }));
        const auto originalCandidate = candidate;
        auto wrongDirection = scene;
        for (auto &point : wrongDirection.mesh.vertices)
        {
            point.z -= 45;
        }
        auto rejected = candidate;
        (void)StitchCandidateBorders(rejected, bounds, {neighbor}, {authored}, true, &wrongDirection);
        assert(rejected.topology.valid && rejected.authoredBorderVertexEdits.empty());
        auto offsetScene = scene;
        for (auto &point : offsetScene.mesh.vertices)
        {
            point.z += 45;
        }
        auto corrected = originalCandidate;
        assert(StitchCandidateBorders(corrected, bounds, {neighbor}, {authored}, true, &offsetScene) == 1);
        assert(corrected.topology.valid && !corrected.authoredBorderVertexEdits.empty());
        assert(StitchCandidateBorders(candidate, bounds, {neighbor}, {authored}, true, &scene) == 1);
        assert(candidate.topology.valid && !candidate.authoredBorderVertexEdits.empty());
        auto editedNeighbor = neighbor;
        assert(ApplyAuthoredBorderVertexEdits(editedNeighbor, candidate.authoredBorderVertexEdits, candidate.profile));
        const auto crossing = std::find_if(candidate.borderLinks.begin(), candidate.borderLinks.end(),
                                           [](const auto &link) { return link.neighborPolygon == 0; });
        assert(crossing != candidate.borderLinks.end());
        const auto &face = candidate.mesh.polygons[crossing->polygon];
        const auto first = candidate.mesh.vertices[face.vertices[crossing->edge]];
        const auto last = candidate.mesh.vertices[face.vertices[(crossing->edge + 1) % 3]];
        assert(first.x == editedNeighbor.vertices[1].x && first.z == editedNeighbor.vertices[1].z);
        assert(last.x == editedNeighbor.vertices[0].x && last.z == editedNeighbor.vertices[0].z);
        assert(editedNeighbor.vertices[0].x == neighbor.vertices[0].x && editedNeighbor.vertices[0].z == 0);
        assert(neighbor.vertices[1].x == 0 && neighbor.vertices[1].z == -57.486F);
    }

    void CheckCornerHeightDrift()
    {
        // Two authored borders meet at the same XY position with a small height
        // difference. The inset floor terminates before one portal's other end.
        for (int rotation{}; rotation < 4; ++rotation)
        {
            auto candidate = MakeFloor({{225.9375F, 0, 25.8953F},
                                        {300, 0, 10.5247F},
                                        {300, 119.1113F, 82.3254F},
                                        {210.5469F, 59.6362F, 82.3254F}},
                                       {{0, 1, 3}, {1, 2, 3}});
            auto eastSource =
                MakePortal(0x200, {300, 0, -0.1494F}, {300, 78.416F, 79.2903F}, {197.4375F, 51.4604F, 79.5315F});
            auto southSource =
                MakePortal(0x202, {207.6875F, 0, 16.9865F}, {300, 0, 0}, {197.4375F, 51.4604F, 79.5315F});
            auto east = MakePortal(0x201, eastSource.vertices[1], eastSource.vertices[0], {330, 60, 80});
            auto south = MakePortal(0x203, southSource.vertices[1], southSource.vertices[0], {210, -20, 16});
            eastSource.externalLinks = {{0, 0, east.id, 0}};
            southSource.externalLinks = {{0, 0, south.id, 0}};
            east.externalLinks = {{0, 0, eastSource.id, 0}};
            south.externalLinks = {{0, 0, southSource.id, 0}};
            for (auto *mesh : {&candidate.mesh, &eastSource, &southSource, &east, &south})
            {
                for (auto &point : mesh->vertices)
                {
                    for (int turn{}; turn < rotation; ++turn)
                    {
                        point = {300 - point.y, point.x, point.z};
                    }
                    point.x -= 180224;
                    point.y += 4096;
                    point.z -= 2413.0754F;
                }
            }
            RefreshCandidateTopology(candidate);
            const AABB bounds{{-180224, 4096, -3000}, {-179924, 4396, 0}};
            assert(StitchCandidateBorders(candidate, bounds, {east, south}, {eastSource, southSource}) == 2);
            CheckPortal(candidate, east);
            CheckPortal(candidate, south);
            assert(StitchCandidateBorders(candidate, bounds, {east, south}, {eastSource, southSource}) == 0);
            CheckPortal(candidate, east);
            CheckPortal(candidate, south);
        }
    }

    void CheckThinSeamRetraction(bool withPortal)
    {
        // A seam sliver can have no representable float point strictly inside it.
        // Its door, source evidence and floor must survive retraction on every CELL side.
        for (const auto offset : {0.0F, 131072.0F, -180224.0F})
        {
            for (int rotation{}; rotation < 4; ++rotation)
            {
                for (const bool deferReachability : {false, true})
                {
                    const auto transform = [&](Vec3 point)
                    {
                        for (int turn{}; turn < rotation; ++turn)
                        {
                            point = {-point.y, point.x, point.z};
                        }
                        return point + Vec3{offset, offset, 0};
                    };
                    const auto thinWidth = 0.015625F;
                    std::vector<Vec3> vertices{transform({-72, 0, 0}), transform({0, 0, 0}),
                                               transform({0, thinWidth, 0})};
                    std::vector<std::array<std::uint32_t, 3>> faces{{0, 1, 2}};
                    if (withPortal)
                    {
                        vertices.push_back(transform({0, 200, 0}));
                        faces.push_back({0, 2, 3});
                    }
                    auto candidate = MakeFloor(std::move(vertices), std::move(faces));
                    candidate.exits = {
                        {.referenceId = 0x300, .position = transform({-62, 0, 0}), .region = 0, .polygon = 0}};
                    std::vector<NavMesh> neighbors;
                    if (withPortal)
                    {
                        // A wider adjacent floor pins one thin-edge endpoint through
                        // its existing portal. The nearby unpaired endpoint must retract independently.
                        neighbors.push_back(MakePortal(0x201, transform({0, 200, 0}), transform({0, thinWidth, 0}),
                                                       transform({100, 100, 0})));
                        candidate.borderLinks = {{1, 1, neighbors.front().id, 0, 0}};
                    }
                    AABB bounds;
                    bounds.Expand(transform({-4096, -1024, -100}));
                    bounds.Expand(transform({0, 3072, 100}));
                    assert(StitchCandidateBorders(candidate, bounds, neighbors, {}, false, nullptr,
                                                  deferReachability) == 0);
                    if (!candidate.topology.valid)
                    {
                        std::fprintf(stderr, "Thin seam: offset=%g rotation=%d portal=%d deferred=%d\n", offset,
                                     rotation, withPortal, deferReachability);
                        for (const auto &finding : candidate.topology.findings)
                        {
                            std::fprintf(stderr, "%s\n", finding.c_str());
                        }
                    }
                    assert(candidate.topology.valid);
                    assert(!candidate.mesh.polygons.empty());
                    assert(candidate.exits.front().polygon && candidate.exits.front().region);
                    assert(candidate.regions.size() == 1);
                    assert(candidate.regions.front().area > 0);
                    assert(candidate.statistics.rejectedUnreachable == 0);
                    assert(candidate.borderLinks.size() == (withPortal ? 1U : 0U));
                    if (withPortal)
                    {
                        CheckPortal(candidate, neighbors.front());
                    }
                    for (std::size_t index{}; index < candidate.mesh.polygons.size(); ++index)
                    {
                        assert(candidate.mesh.polygons[index].flags == WaterFlag);
                        const auto source = candidate.polygonSourceTriangles[index];
                        assert(source <= (withPortal ? 1U : 0U));
                        assert(candidate.polygonContributingTriangles[index] == std::vector<std::size_t>{source});
                    }
                    for (std::size_t source{}; source < (withPortal ? 2U : 1U); ++source)
                    {
                        assert(std::find(candidate.polygonSourceTriangles.begin(),
                                         candidate.polygonSourceTriangles.end(),
                                         source) != candidate.polygonSourceTriangles.end());
                    }
                    assert(StitchCandidateBorders(candidate, bounds, neighbors, {}, false, nullptr,
                                                  deferReachability) == 0);
                    assert(candidate.topology.valid);
                }
            }
        }
    }

    void CheckAlternativeCavityCuts()
    {
        // An endpoint projection can leave a steep seam sliver or miss a
        // compatible terminal cut around a turning boundary. Repair must
        // consider nearby seam cuts while keeping the complete interior rim.
        for (const bool turning : {false, true})
        {
            for (const bool mirrored : {false, true})
            {
                for (int rotation{}; rotation < 4; ++rotation)
                {
                    const auto transform = [&](Vec3 point)
                    {
                        if (mirrored)
                        {
                            point.x = -point.x;
                        }
                        for (int turn{}; turn < rotation; ++turn)
                        {
                            point = {-point.y, point.x, point.z};
                        }
                        return point + Vec3{131072, -180224, -9000};
                    };
                    std::vector<Vec3> vertices =
                        turning ? std::vector<Vec3>{{-100, 60, -44}, {-100, 20, -32}, {-110, 0, -40},
                                                    {120, 2, -43},   {170, 0, -34},   {350, 170, -40}}
                                : std::vector<Vec3>{{0, 0, 0},  {5, 0, 0},    {300, 0, -30},   {550, 0, -60},
                                                    {0, 60, 0}, {50, 80, -6}, {400, 200, -45}, {430, 180, -50}};
                    std::vector<std::array<std::uint32_t, 3>> faces =
                        turning ? std::vector<std::array<std::uint32_t, 3>>{{3, 4, 5}, {3, 5, 0}, {3, 0, 1}, {3, 1, 2}}
                                : std::vector<std::array<std::uint32_t, 3>>{{4, 0, 1}, {4, 1, 5}, {1, 2, 7},
                                                                            {2, 3, 7}, {1, 7, 6}, {1, 6, 5}};
                    for (auto &point : vertices)
                    {
                        point = transform(point);
                    }
                    if (mirrored)
                    {
                        for (auto &face : faces)
                        {
                            std::swap(face[0], face[1]);
                        }
                    }
                    auto candidate = MakeFloor(vertices, faces);
                    auto first = transform(turning ? Vec3{0, 0, 0} : Vec3{10, 0, -8});
                    auto last = transform(turning ? Vec3{170, 0, -34} : Vec3{300, 0, -30});
                    if (mirrored)
                    {
                        std::swap(first, last);
                    }
                    const auto opposite = transform(turning ? Vec3{80, 200, -20} : Vec3{120, 70, -10});
                    auto authored = MakePortal(0x200, first, last, opposite);
                    auto neighbor = MakePortal(0x201, last, first, transform({150, -100, -20}));
                    authored.externalLinks = {{0, 0, neighbor.id, 0}};
                    neighbor.externalLinks = {{0, 0, authored.id, 0}};
                    // Pin the adjacent partition before repairing the sliver.
                    std::vector<NavMesh> neighbors{neighbor};
                    if (!turning)
                    {
                        auto adjacent = MakePortal(0x203, transform({550, 0, -60}), transform({300, 0, -30}),
                                                   transform({400, -100, -45}));
                        if (mirrored)
                        {
                            std::swap(adjacent.vertices[0], adjacent.vertices[1]);
                        }
                        candidate.borderLinks = {{3, 0, adjacent.id, 0, 0}};
                        neighbors.push_back(adjacent);
                    }
                    AABB bounds;
                    bounds.Expand(transform({-300, 0, -100}));
                    bounds.Expand(transform({600, 400, 100}));
                    const auto door = transform(turning ? Vec3{100, 80, -35} : Vec3{100, 80, -12});
                    candidate.exits = {{.referenceId = 0x300,
                                        .position = door,
                                        .region = 0,
                                        .polygon = static_cast<std::uint32_t>(turning ? 1 : 5)}};
                    if (turning)
                    {
                        auto unsupportedCuts = candidate;
                        // A different floor level supplies no compatible portal support.
                        for (auto &point : unsupportedCuts.mesh.vertices)
                        {
                            point.z -= 1000;
                        }
                        assert(StitchCandidateBorders(unsupportedCuts, bounds, neighbors, {authored}) == 0);
                        assert(unsupportedCuts.topology.valid);
                        assert(unsupportedCuts.borderLinks.empty());
                    }
                    const auto added = StitchCandidateBorders(candidate, bounds, neighbors, {authored});
                    if (added != 1 || !candidate.topology.valid)
                    {
                        std::fprintf(stderr, "Alternative cut: turning=%d mirrored=%d rotation=%d added=%zu\n", turning,
                                     mirrored, rotation, added);
                        for (const auto &finding : candidate.topology.findings)
                        {
                            std::fprintf(stderr, "%s\n", finding.c_str());
                        }
                    }
                    assert(added == 1);
                    CheckPortal(candidate, neighbor);
                    assert(candidate.exits.front().polygon && candidate.exits.front().region);
                    if (!turning)
                    {
                        CheckPortal(candidate, neighbors.back());
                    }
                    for (const auto &contributors : candidate.polygonContributingTriangles)
                    {
                        assert(!contributors.empty());
                    }
                    const std::vector<std::size_t> interiorRim =
                        turning ? std::vector<std::size_t>{0, 1, 5} : std::vector<std::size_t>{4, 5, 6, 7};
                    for (const auto index : interiorRim)
                    {
                        const auto expected = vertices[index];
                        const auto kept = std::any_of(
                            candidate.mesh.vertices.begin(), candidate.mesh.vertices.end(), [&](Vec3 point)
                            { return point.x == expected.x && point.y == expected.y && point.z == expected.z; });
                        assert(kept);
                    }
                    assert(StitchCandidateBorders(candidate, bounds, neighbors, {authored}) == 0);
                    CheckPortal(candidate, neighbor);
                }
            }
        }
    }
} // namespace

void TestBorderCavityRegressions()
{
    CheckTerminalCavities();
    CheckExcludedAuthoredFloor();
    CheckDeferredRequiredCrossing();
    CheckAuthoredFanEdits();
    CheckSupportedAuthoredTerminalRepair();
    CheckCornerHeightDrift();
    CheckThinSeamRetraction(false);
    CheckThinSeamRetraction(true);
    CheckAlternativeCavityCuts();
}
