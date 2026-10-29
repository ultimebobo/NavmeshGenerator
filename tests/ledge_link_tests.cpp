#include "core/navmesh/ledge_links.h"
#include "core/navmesh/generator.h"
#include "core/navmesh/candidate_reachability.h"
#include "core/navmesh/surface_simplification.h"
#include "core/scene/scene_exporter.h"
#include "app/candidate_cache.h"
#include "skyrim/parser/plugin_parser.h"
#include "skyrim/parser/plugin_writer.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>

namespace
{
    using namespace navmesh::core;

    /// Closed boxes are project-authored collision; top faces provide floors and side faces obstruct drops.
    void AddBox(Scene &scene, Vec3 minimum, Vec3 maximum, std::size_t geometrySource = 0)
    {
        const auto first = static_cast<std::uint32_t>(scene.mesh.vertices.size());
        for (const auto point : std::array<Vec3, 8>{{{minimum.x, minimum.y, minimum.z},
                                                     {maximum.x, minimum.y, minimum.z},
                                                     {maximum.x, maximum.y, minimum.z},
                                                     {minimum.x, maximum.y, minimum.z},
                                                     {minimum.x, minimum.y, maximum.z},
                                                     {maximum.x, minimum.y, maximum.z},
                                                     {maximum.x, maximum.y, maximum.z},
                                                     {minimum.x, maximum.y, maximum.z}}})
        {
            scene.mesh.vertices.push_back(point);
        }
        constexpr std::array<std::array<std::uint32_t, 3>, 12> faces{{{0, 2, 1},
                                                                      {0, 3, 2},
                                                                      {4, 5, 6},
                                                                      {4, 6, 7},
                                                                      {0, 1, 5},
                                                                      {0, 5, 4},
                                                                      {1, 2, 6},
                                                                      {1, 6, 5},
                                                                      {2, 3, 7},
                                                                      {2, 7, 6},
                                                                      {3, 0, 4},
                                                                      {3, 4, 7}}};
        for (const auto &face : faces)
        {
            scene.triangleProvenance.push_back({geometrySource, scene.mesh.triangles.size()});
            scene.mesh.triangles.push_back({{first + face[0], first + face[1], first + face[2]}});
        }
    }

    Scene BuildLedgeScene()
    {
        Scene scene;
        scene.geometrySources.push_back({.modelPath = "synthetic/drop-platforms",
                                         .materialClass = MaterialCollisionClass::HavokPackedTriangles,
                                         .sourceType = GeometrySourceType::Collision,
                                         .confidence = 1.0F});
        // Stations: safe descent, damaging descent, ceiling across the departure, and a wide horizontal gap.
        for (int station{}; station < 4; ++station)
        {
            const float y = station * 512.0F;
            const float height = station == 1 ? 512.0F : 160.0F;
            const float gap = station == 3 ? 128.0F : 0.0F;
            AddBox(scene, {-256, y - 128, -32}, {0, y + 128, height});
            AddBox(scene, {gap, y - 128, -32}, {gap + 256, y + 128, 0});
            if (station == 2)
            {
                scene.geometrySources.push_back({.modelPath = "synthetic/overhead-obstruction",
                                                 .sourceType = GeometrySourceType::Collision,
                                                 .confidence = 1,
                                                 .navigationObstacle = true});
                AddBox(scene, {4, y - 128, 200}, {80, y + 128, 220}, scene.geometrySources.size() - 1);
            }
        }
        return scene;
    }

    CandidateNavMesh Generate(const Scene &scene)
    {
        auto candidate = RecastCandidateGenerator{}.Generate(
            scene, {}, std::nullopt, {}, RegionPartitioningAlgorithm::Watershed, {}, CandidateRetention::AllWalkable);
        CaptureLedgeGeometry(candidate, scene);
        (void)BuildCandidateLedgeLinks(candidate);
        return candidate;
    }

    void CheckScene(const CandidateNavMesh &candidate)
    {
        assert(candidate.topology.valid);
        if (candidate.ledgeLinks.size() != 1)
        {
            std::cerr << "Expected one safe ledge; got " << candidate.ledgeLinks.size() << '\n';
            for (const auto &link : candidate.ledgeLinks)
            {
                for (const auto [polygon, edge] :
                     {std::pair{link.upperPolygon, link.upperEdge}, std::pair{link.lowerPolygon, link.lowerEdge}})
                {
                    const auto &face = candidate.mesh.polygons[polygon];
                    for (const auto index : {face.vertices[edge], face.vertices[(edge + 1) % 3]})
                    {
                        const auto point = candidate.mesh.vertices[index];
                        std::cerr << " (" << point.x << ',' << point.y << ',' << point.z << ')';
                    }
                }
                std::cerr << '\n';
            }
        }
        assert(candidate.ledgeLinks.size() == 1);
        const auto &link = candidate.ledgeLinks.front();
        const auto &upper = candidate.mesh.polygons.at(link.upperPolygon);
        const auto &lower = candidate.mesh.polygons.at(link.lowerPolygon);
        assert(std::abs(candidate.mesh.vertices.at(upper.vertices[0]).y) < 128);
        assert(candidate.mesh.vertices.at(upper.vertices[0]).z > candidate.mesh.vertices.at(lower.vertices[0]).z);
        assert(HasCandidateLedgeRoute(candidate, candidate.mesh, link.upperPolygon, candidate.mesh, link.lowerPolygon));
        assert(
            !HasCandidateLedgeRoute(candidate, candidate.mesh, link.lowerPolygon, candidate.mesh, link.upperPolygon));
    }

    void CheckPluginRoundTrip(const std::filesystem::path &root, const CandidateNavMesh &candidate)
    {
        using namespace navmesh::skyrim;
        std::filesystem::create_directories(root);
        // A minimal redistributable plugin supplies only CELL ownership; the writer must allocate its NAVM.
        std::vector<std::uint8_t> plugin;
        const auto put32 = [&](std::uint32_t value)
        {
            for (int shift{}; shift < 32; shift += 8)
            {
                plugin.push_back(static_cast<std::uint8_t>(value >> shift));
            }
        };
        const auto header = [&](const char *type, std::uint32_t size, std::uint32_t id)
        {
            plugin.insert(plugin.end(), type, type + 4);
            put32(size);
            put32(0);
            put32(id);
            put32(0);
            put32(44);
        };
        header("TES4", 0, 0);
        plugin.insert(plugin.end(), {'G', 'R', 'U', 'P'});
        put32(48);
        put32(0x4C4C4543);
        put32(0);
        put32(0);
        put32(0);
        header("CELL", 0, 0x100);
        const auto source = root / "Drops.esm";
        {
            std::ofstream file(source, std::ios::binary);
            file.write(reinterpret_cast<const char *>(plugin.data()), static_cast<std::streamsize>(plugin.size()));
        }
        auto resolved = ResolveLoadOrder({.dataDirectory = root, .plugins = {source}});
        assert(resolved.diagnostics.empty() && resolved.cells.size() == 1);
        std::filesystem::path target;
        std::string error;
        const auto checkWritten = [&](const std::filesystem::path &path)
        {
            auto written = ResolveLoadOrder({.dataDirectory = root, .plugins = {source, path}});
            assert(written.diagnostics.empty() && written.cells.size() == 1 &&
                   written.cells.front().navMeshes.size() == 1);
            const auto &mesh = written.cells.front().navMeshes.front();
            assert(mesh.externalLinks.size() == candidate.ledgeLinks.size() * 2);
            for (const auto &link : mesh.externalLinks)
            {
                assert(link.navmeshId == mesh.id);
                assert(link.type == 1 || link.type == 2);
                const auto &face = mesh.polygons[link.polygon];
                const auto &destination = mesh.polygons[link.targetPolygon];
                const auto sourceHeight = mesh.vertices[face.vertices[link.edge]].z;
                const auto targetHeight = mesh.vertices[destination.vertices[0]].z;
                assert(link.type == 2 ? sourceHeight > targetHeight : sourceHeight < targetHeight);
            }
            return written;
        };
        bool written =
            WriteNavmeshOverride(root / "new", {source}, resolved, resolved.cells.front(), candidate, target, error);
        if (!written)
        {
            std::cerr << error << '\n';
        }
        assert(written);
        auto generated = checkWritten(target);
        const auto generatedPath = target;
        for (const bool copy : {false, true})
        {
            const std::vector<NavmeshReplacement> replacements{{&generated.cells.front(), &candidate}};
            written = WriteNavmeshOverrides(root / (copy ? "copy" : "replace"), {source, generatedPath}, generated,
                                            replacements, target, error,
                                            copy ? generatedPath.filename().string() : std::string{});
            if (!written)
            {
                std::cerr << error << '\n';
            }
            assert(written);
            const auto effective =
                ResolveLoadOrder({.dataDirectory = root,
                                  .plugins = copy ? std::vector<std::filesystem::path>{source, target}
                                                  : std::vector<std::filesystem::path>{source, generatedPath, target}});
            assert(effective.diagnostics.empty());
            assert(effective.cells.front().navMeshes.front().externalLinks.size() == 2);
        }
    }

    void CheckSceneGroups(const std::filesystem::path &root, const CandidateNavMesh &candidate)
    {
        const auto check =
            [&](const SceneExportOptions &options, std::vector<NavMesh> authored, const char *expectedKind)
        {
            const auto path = root / "groups.glb";
            assert(WriteCombinedGlb(path, {}, authored, {}, {}, options).written);
            std::ifstream file(path, std::ios::binary);
            const std::string bytes((std::istreambuf_iterator<char>(file)), {});
            assert(bytes.contains("\"name\":\"Ledge links\""));
            assert(bytes.contains(expectedKind));
            assert(bytes.contains("Ledge links (pink)"));
        };
        check({.layers = {SceneLayer::CandidateNavmesh},
               .candidateNavmesh = &candidate.mesh,
               .candidateLedgeLinks = &candidate.ledgeLinks},
              {}, "candidate_ledge_link");
        check({.layers = {SceneLayer::CandidateNavmesh}, .candidates = {{1, &candidate}}}, {}, "candidate_ledge_link");
        auto authored = candidate.mesh;
        authored.id = 0x200;
        for (const auto &link : candidate.ledgeLinks)
        {
            authored.externalLinks.push_back({link.upperPolygon, link.upperEdge, authored.id, link.lowerPolygon, 2});
            authored.externalLinks.push_back({link.lowerPolygon, link.lowerEdge, authored.id, link.upperPolygon, 1});
        }
        check({.layers = {SceneLayer::ExistingNavmesh}}, {authored}, "authored_ledge_link");
    }

    void CheckSubdividedLanding(const CandidateNavMesh &original)
    {
        auto candidate = original;
        const auto link = candidate.ledgeLinks.front();
        const auto face = candidate.mesh.polygons[link.lowerPolygon];
        const auto a = face.vertices[link.lowerEdge];
        const auto b = face.vertices[(link.lowerEdge + 1) % 3];
        const auto c = face.vertices[(link.lowerEdge + 2) % 3];
        const auto midpoint = static_cast<std::uint32_t>(candidate.mesh.vertices.size());
        candidate.mesh.vertices.push_back((candidate.mesh.vertices[a] + candidate.mesh.vertices[b]) * 0.5F);
        candidate.mesh.polygons[link.lowerPolygon].vertices = {a, midpoint, c};
        auto child = face;
        child.vertices = {midpoint, b, c};
        const auto childId = static_cast<std::uint32_t>(candidate.mesh.polygons.size());
        candidate.mesh.polygons.push_back(child);
        candidate.polygonSourceTriangles.push_back(candidate.polygonSourceTriangles[link.lowerPolygon]);
        const auto contributors = candidate.polygonContributingTriangles[link.lowerPolygon];
        candidate.polygonContributingTriangles.push_back(contributors);
        for (auto &region : candidate.regions)
        {
            if (std::find(region.polygons.begin(), region.polygons.end(), link.lowerPolygon) != region.polygons.end())
            {
                region.polygons.push_back(childId);
            }
        }
        candidate.ledgeLinks.clear();
        RefreshCandidateTopology(candidate);
        assert(candidate.topology.valid);
        assert(BuildCandidateLedgeLinks(candidate) > 0);
        assert(HasCandidateLedgeRoute(candidate, original.mesh, link.upperPolygon, original.mesh, link.lowerPolygon));
        const auto polygons = candidate.mesh.polygons.size();
        const auto vertices = candidate.mesh.vertices.size();
        assert(BuildCandidateLedgeLinks(candidate) > 0);
        assert(candidate.mesh.polygons.size() == polygons && candidate.mesh.vertices.size() == vertices);
    }

    CandidateNavMesh BuildInteriorLandingFixture()
    {
        Scene scene;
        scene.geometrySources.push_back({.sourceType = GeometrySourceType::Collision, .confidence = 1});
        AddBox(scene, {-512, -512, -32}, {1024, 1024, 0});
        AddBox(scene, {-256, -128, 128}, {0, 128, 160});
        CandidateNavMesh candidate;
        candidate.mesh.vertices = {{-256, -128, 160}, {0, -128, 160},  {0, 128, 160},  {-256, 128, 160},
                                   {-512, -512, 0},   {1024, -512, 0}, {-512, 1024, 0}};
        candidate.mesh.polygons = {{.vertices = {0, 1, 2}}, {.vertices = {0, 2, 3}}, {.vertices = {4, 5, 6}}};
        candidate.polygonSourceTriangles = {0, 0, 0};
        candidate.polygonContributingTriangles = {{0}, {0}, {0}};
        candidate.regions = {{.id = 0, .polygons = {0, 1}}, {.id = 1, .polygons = {2}}};
        RefreshCandidateTopology(candidate);
        assert(candidate.topology.valid);
        const auto validationMesh = candidate.mesh;
        CaptureLedgeGeometry(candidate, scene);
        assert(BuildCandidateLedgeLinks(candidate) == 1);
        assert(candidate.topology.valid);
        assert(HasCandidateLedgeRoute(candidate, validationMesh, 0, validationMesh, 2));
        assert(!HasCandidateLedgeRoute(candidate, validationMesh, 2, validationMesh, 0));
        const auto &port = candidate.ledgeLinks.front();
        const auto &lower = candidate.mesh.polygons[port.lowerPolygon];
        for (const auto vertex : {lower.vertices[port.lowerEdge], lower.vertices[(port.lowerEdge + 1) % 3]})
        {
            const auto point = candidate.mesh.vertices[vertex];
            assert(point.x > -512 && point.y > -512 && point.x + point.y < 512);
        }
        // The receiving floor stays connected around the short reserved port edge.
        assert(HasCandidateFloorRoute(candidate, {32, -64, 0}, {32, 64, 0}));
        const NavMesh broadLanding{.vertices = {{-512, -512, 160}, {2048, -512, 160}, {-512, 2048, 160}},
                                   .polygons = {{.vertices = {0, 1, 2}}}};
        assert(HasCandidateLedgeRoute(candidate, broadLanding, 0, validationMesh, 2));
        const auto polygons = candidate.mesh.polygons.size();
        assert(BuildCandidateLedgeLinks(candidate) == 1);
        assert(candidate.mesh.polygons.size() == polygons);
        (void)SimplifyCandidateSurface(candidate);
        assert(candidate.topology.valid);
        assert(HasCandidateLedgeRoute(candidate, validationMesh, 0, validationMesh, 2));
        assert(HasCandidateFloorRoute(candidate, {32, -64, 0}, {32, 64, 0}));
        return candidate;
    }

    void CheckDeepWaterLandings()
    {
        Scene scene;
        scene.geometrySources.push_back({.sourceType = GeometrySourceType::Collision, .confidence = 1});
        AddBox(scene, {-256, -128, -32}, {0, 128, 1024});
        AddBox(scene, {0, -128, -32}, {256, 128, 0});
        assert(Generate(scene).ledgeLinks.empty());
        scene.waterSurfaces.vertices = {{0, -128, 256}, {256, -128, 256}, {256, 128, 256}, {0, 128, 256}};
        scene.waterSurfaces.triangles = {{{0, 1, 2}}, {{0, 2, 3}}};
        const auto deep = Generate(scene);
        assert(deep.ledgeLinks.size() == 1);
        assert(CandidateWaterHeight(deep, {64, 0, 0}) == 256);
        assert(!CandidateWaterHeight(deep, {512, 0, 0}));
        for (auto &point : scene.waterSurfaces.vertices)
        {
            point.z = 64;
        }
        assert(Generate(scene).ledgeLinks.empty());
    }

    void CheckUnevenDeparture(const CandidateNavMesh &original)
    {
        for (const bool wall : {false, true})
        {
            auto candidate = original;
            Scene obstruction;
            const auto rise = wall ? candidate.profile.agentHeight : candidate.profile.stepHeight * 0.5F;
            // A ridge under the departure footprint is a walking step; a standing-height wall blocks the drop.
            AddBox(obstruction, {-12, -128, 160}, {-8, 128, 160 + rise});
            const auto offset = static_cast<std::uint32_t>(candidate.ledgeGeometry.vertices.size());
            candidate.ledgeGeometry.vertices.insert(candidate.ledgeGeometry.vertices.end(),
                                                    obstruction.mesh.vertices.begin(), obstruction.mesh.vertices.end());
            for (auto face : obstruction.mesh.triangles)
            {
                for (auto &vertex : face.vertices)
                {
                    vertex += offset;
                }
                candidate.ledgeGeometry.triangles.push_back(face);
            }
            assert(BuildCandidateLedgeLinks(candidate) == (wall ? 0 : 1));
            assert(candidate.topology.valid);
        }
    }

    /// Find an unused boundary on the landing's connected floor for a synthetic external portal.
    CandidateBorderLink LandingCrossing(const CandidateNavMesh &candidate, const CandidateLedgeLink &landing)
    {
        std::vector<std::uint32_t> pending{landing.lowerPolygon};
        std::vector<bool> visited(candidate.mesh.polygons.size());
        while (!pending.empty())
        {
            const auto polygon = pending.back();
            pending.pop_back();
            if (visited[polygon])
            {
                continue;
            }
            visited[polygon] = true;
            for (std::uint8_t edge{}; edge < 3; ++edge)
            {
                const auto neighbor = candidate.mesh.polygons[polygon].neighbors[edge];
                if (neighbor != std::numeric_limits<std::uint32_t>::max())
                {
                    pending.push_back(neighbor);
                }
                else if (polygon != landing.lowerPolygon || edge != landing.lowerEdge)
                {
                    return {.polygon = polygon, .edge = edge, .neighborNavmeshId = 3};
                }
            }
        }
        assert(false);
        return {};
    }

    /// Drops into exterior dead ends cannot retain scenery; complete descending exit routes remain usable.
    void CheckExteriorEscapeRoutes()
    {
        Scene scene;
        scene.geometrySources.push_back({.sourceType = GeometrySourceType::Collision, .confidence = 1});
        AddBox(scene, {-256, -128, -32}, {0, 128, 320});
        AddBox(scene, {0, -128, -32}, {256, 128, 160});
        AddBox(scene, {256, -128, -32}, {512, 128, 0});
        const auto original = Generate(scene);
        assert(original.topology.valid && original.ledgeLinks.size() == 2);
        const auto first = std::find_if(
            original.ledgeLinks.begin(), original.ledgeLinks.end(), [&](const auto &link)
            { return original.mesh.vertices[original.mesh.polygons[link.upperPolygon].vertices[0]].z > 300; });
        const auto last = std::find_if(
            original.ledgeLinks.begin(), original.ledgeLinks.end(), [&](const auto &link)
            { return original.mesh.vertices[original.mesh.polygons[link.lowerPolygon].vertices[0]].z < 100; });
        assert(first != original.ledgeLinks.end() && last != original.ledgeLinks.end());
        enum class TerminalAccess
        {
            None,
            Door,
            RequiredCrossing,
            IncidentalCrossing
        };
        for (const bool exterior : {false, true})
        {
            // A terminal door and a required authored crossing both provide exits.
            // An incidental border match supplies no access in an entrance-anchored selection.
            for (const auto terminalAccess : {TerminalAccess::None, TerminalAccess::Door,
                                              TerminalAccess::RequiredCrossing, TerminalAccess::IncidentalCrossing})
            {
                auto candidate = original;
                candidate.exits.push_back({.referenceId = 1, .polygon = first->upperPolygon});
                if (terminalAccess == TerminalAccess::Door)
                {
                    candidate.exits.push_back({.referenceId = 2, .polygon = last->lowerPolygon});
                }
                if (terminalAccess == TerminalAccess::RequiredCrossing ||
                    terminalAccess == TerminalAccess::IncidentalCrossing)
                {
                    auto crossing = LandingCrossing(candidate, *last);
                    crossing.requiredAuthoredCrossing = terminalAccess == TerminalAccess::RequiredCrossing;
                    candidate.borderLinks.push_back(crossing);
                }
                CandidateReachabilityTarget target{.cellId = 1, .candidate = &candidate};
                if (exterior)
                {
                    target.worldspaceId = 1;
                    target.bounds = AABB{.min = {-512, -256, -32}, .max = {1024, 256, 1024}};
                }
                const auto keepsLandings = !exterior || terminalAccess == TerminalAccess::Door ||
                                           terminalAccess == TerminalAccess::RequiredCrossing;
                const auto removed = RemoveCandidateIslands(std::span(&target, 1));
                assert(candidate.topology.valid);
                assert(candidate.ledgeLinks.size() == (keepsLandings ? 2 : 0));
                assert(HasCandidateLedgeRoute(candidate, original.mesh, first->upperPolygon, original.mesh,
                                              last->lowerPolygon) == keepsLandings);
                if (!keepsLandings)
                {
                    assert(removed > 0);
                    assert(std::all_of(candidate.mesh.vertices.begin(), candidate.mesh.vertices.end(),
                                       [](Vec3 point) { return point.z > 300; }));
                    assert(candidate.borderLinks.empty());
                }
                assert(RemoveCandidateIslands(std::span(&target, 1)) == 0);
            }
        }
    }
} // namespace

void TestLedgeLinkFixture()
{
    using namespace navmesh::core;
    const auto scene = BuildLedgeScene();
    auto candidate = Generate(scene);
    CheckScene(candidate);
    const auto original = candidate;
    CheckSubdividedLanding(original);
    CheckUnevenDeparture(original);
    const auto interior = BuildInteriorLandingFixture();
    CheckDeepWaterLandings();
    CheckExteriorEscapeRoutes();
    auto translated = candidate;
    for (auto *mesh : {&translated.mesh.vertices, &translated.ledgeGeometry.vertices})
    {
        for (auto &point : *mesh)
        {
            point = point + Vec3{1000000, -1000000, 100000};
        }
    }
    assert(BuildCandidateLedgeLinks(translated) == original.ledgeLinks.size());
    assert(translated.ledgeLinks.front().upperPolygon == original.ledgeLinks.front().upperPolygon);
    (void)SimplifyCandidateSurface(candidate);
    CheckScene(candidate);
    for (const bool upperAnchor : {true, false})
    {
        candidate = original;
        const auto &link = candidate.ledgeLinks.front();
        const auto polygon = upperAnchor ? link.upperPolygon : link.lowerPolygon;
        candidate.exits.push_back({.referenceId = 1, .polygon = polygon});
        // Exterior door anchoring avoids the independent primary-floor fallback used by interiors.
        std::array targets{
            CandidateReachabilityTarget{.cellId = 1,
                                        .worldspaceId = 1,
                                        .bounds = AABB{.min = {-512, -256, -32}, .max = {512, 2048, 1024}},
                                        .candidate = &candidate}};
        (void)RemoveCandidateIslands(targets);
        assert(candidate.topology.valid);
        assert(candidate.ledgeLinks.empty());
        assert(candidate.regions.size() == 1);
        if (upperAnchor)
        {
            assert(std::all_of(candidate.mesh.vertices.begin(), candidate.mesh.vertices.end(),
                               [](auto point) { return point.z > 128; }));
        }
        else
        {
            assert(std::all_of(candidate.mesh.vertices.begin(), candidate.mesh.vertices.end(),
                               [](auto point) { return point.z < 128; }));
        }
    }
    candidate = original;
    candidate.profile.maximumDropHeight = 100;
    assert(BuildCandidateLedgeLinks(candidate) == 0);
    candidate = original;
    Scene blocked;
    AddBox(blocked, {18, -128, -32}, {36, 128, 160 + candidate.profile.agentHeight});
    const auto vertexOffset = static_cast<std::uint32_t>(candidate.ledgeGeometry.vertices.size());
    candidate.ledgeGeometry.vertices.insert(candidate.ledgeGeometry.vertices.end(), blocked.mesh.vertices.begin(),
                                            blocked.mesh.vertices.end());
    for (auto face : blocked.mesh.triangles)
    {
        for (auto &vertex : face.vertices)
        {
            vertex += vertexOffset;
        }
        candidate.ledgeGeometry.triangles.push_back(face);
    }
    assert(BuildCandidateLedgeLinks(candidate) == 0);
    candidate = original;
    candidate.borderLinks.push_back(
        {.polygon = candidate.ledgeLinks.front().upperPolygon, .edge = candidate.ledgeLinks.front().upperEdge});
    assert(BuildCandidateLedgeLinks(candidate) == 0);

    const auto root = std::filesystem::current_path() / "build" / "ledge-cache-test";
    // This directory is owned solely by the synthetic fixture; repeated runs must use empty plugin destinations.
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    CheckPluginRoundTrip(root, original);
    CheckPluginRoundTrip(root / "interior-landing", interior);
    CheckSceneGroups(root, original);
    assert(navmesh::app::detail::StoreCandidate(root / "candidate.gz", original, scene));
    CandidateNavMesh loaded;
    Scene evidence;
    assert(navmesh::app::detail::LoadCandidate(root / "candidate.gz", loaded, evidence));
    assert(BuildCandidateLedgeLinks(loaded) == original.ledgeLinks.size());
    CheckScene(loaded);
    auto waterEvidence = original;
    waterEvidence.ledgeWaterHeight = 128.0F;
    waterEvidence.ledgeWaterGeometry = {.vertices = {{0, 0, 64}, {64, 0, 64}, {0, 64, 64}}, .triangles = {{{0, 1, 2}}}};
    assert(navmesh::app::detail::StoreCandidate(root / "water-candidate.gz", waterEvidence, scene));
    assert(navmesh::app::detail::LoadCandidate(root / "water-candidate.gz", loaded, evidence));
    assert(loaded.ledgeWaterHeight == waterEvidence.ledgeWaterHeight);
    assert(loaded.ledgeWaterGeometry.vertices.size() == 3 && loaded.ledgeWaterGeometry.triangles.size() == 1);

    navmesh::skyrim::ResolvedLoadOrder resolved;
    assert(navmesh::skyrim::ResolveNpcSafeDropHeight(resolved) == NavigationProfile{}.maximumDropHeight);
    navmesh::skyrim::ResolvedRecord setting{.type = "GMST", .editorId = "fJumpFallHeightMinNPC"};
    setting.raw.emplace();
    const auto bits = std::bit_cast<std::uint32_t>(96.0F);
    const std::vector<std::uint8_t> bytes{static_cast<std::uint8_t>(bits), static_cast<std::uint8_t>(bits >> 8),
                                          static_cast<std::uint8_t>(bits >> 16), static_cast<std::uint8_t>(bits >> 24)};
    setting.raw->subrecords.push_back({.type = "DATA", .data = bytes});
    resolved.records.push_back(setting);
    assert(navmesh::skyrim::ResolveNpcSafeDropHeight(resolved) == 96.0F);
    resolved.records.front().raw->subrecords.clear();
    assert(navmesh::skyrim::ResolveNpcSafeDropHeight(resolved) == 0);
}

void ExportLedgeLinkFixture(const std::filesystem::path &directory)
{
    const auto scene = BuildLedgeScene();
    const auto candidate = Generate(scene);
    CheckScene(candidate);
    std::filesystem::create_directories(directory);
    assert(navmesh::core::WriteCandidateJson(directory / "candidate-navm.json", candidate, scene, "{}"));
    const navmesh::core::SceneExportOptions options{
        .layers = {navmesh::core::SceneLayer::Collision, navmesh::core::SceneLayer::CandidateNavmesh},
        .candidateNavmesh = &candidate.mesh,
        .candidateLedgeLinks = &candidate.ledgeLinks};
    assert(navmesh::core::WriteCombinedGlb(directory / "scene.glb", scene, {}, {}, {}, options).written);
    std::ofstream report(directory / "checks.json");
    report << "{\"stations\":[\"safe_drop\",\"damaging_drop\",\"blocked_drop\",\"wide_gap\"],"
              "\"expected_links\":[1,0,0,0],\"generated_links\":"
           << candidate.ledgeLinks.size() << "}\n";
    std::cout << "Exported ledge-link scene to " << directory / "scene.glb" << '\n';
}
