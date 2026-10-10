#ifdef NDEBUG
#undef NDEBUG
#endif

#include "core/navmesh/candidate.h"

#include <algorithm>
#include <cassert>
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
                assert(candidate.topology.valid == excluded);
                assert(!candidate.mesh.polygons.empty());
                const auto warning =
                    std::any_of(candidate.warnings.begin(), candidate.warnings.end(), [](const auto &message)
                                { return message.find("excluded obstacle collision") != std::string::npos; });
                assert(warning == excluded);
                assert(candidate.borderLinks.empty());
            }
        }
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
} // namespace

void TestBorderCavityRegressions()
{
    CheckTerminalCavities();
    CheckExcludedAuthoredFloor();
    CheckCornerHeightDrift();
}
