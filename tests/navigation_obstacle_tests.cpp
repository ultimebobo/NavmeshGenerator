#ifdef NDEBUG
#undef NDEBUG
#endif

#include "core/navmesh/generator.h"
#include "core/scene/scene_exporter.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
    using namespace navmesh::core;

    struct SurfaceProbe
    {
        std::string name;
        Vec3 position;
        std::size_t entrance;
        bool covered{true};
    };

    struct ObstacleFixture
    {
        Scene scene;
        std::vector<CandidateExit> entrances;
        std::vector<SurfaceProbe> probes;
        std::vector<std::pair<std::size_t, std::size_t>> separatedEntrances;
    };

    struct Placement
    {
        Vec3 origin;
        float angle{};

        Vec3 World(Vec3 point) const
        {
            const auto cosine = std::cos(angle);
            const auto sine = std::sin(angle);
            return origin + Vec3{point.x * cosine - point.y * sine, point.x * sine + point.y * cosine, point.z};
        }
    };

    std::size_t AddSource(ObstacleFixture &fixture, const std::string &name)
    {
        const auto index = fixture.scene.geometrySources.size();
        fixture.scene.geometrySources.push_back(
            {.modelPath = "synthetic/" + name,
             .sourceType = GeometrySourceType::Collision,
             .confidence = 1.0F,
             .reference = {"Synthetic.esm", static_cast<std::uint32_t>(index + 1), "REFR"}});
        return index;
    }

    void AddQuad(ObstacleFixture &fixture, std::size_t source, const Placement &placement, std::array<Vec3, 4> corners)
    {
        const auto base = static_cast<std::uint32_t>(fixture.scene.mesh.vertices.size());
        for (const auto corner : corners)
        {
            fixture.scene.mesh.vertices.push_back(placement.World(corner));
        }
        for (const auto face : {std::array{base, base + 1, base + 2}, std::array{base, base + 2, base + 3}})
        {
            fixture.scene.mesh.triangles.push_back({face});
            fixture.scene.triangleProvenance.push_back({source, fixture.scene.mesh.triangles.size() - 1, {}});
        }
    }

    // Closed solids supply both support and obstruction evidence. Vertical faces
    // must survive input preparation even though they can never be walkable.
    void AddBox(ObstacleFixture &fixture, std::size_t source, const Placement &placement, Vec3 low, Vec3 high)
    {
        AddQuad(fixture, source, placement,
                {{{low.x, low.y, high.z}, {high.x, low.y, high.z}, {high.x, high.y, high.z}, {low.x, high.y, high.z}}});
        AddQuad(fixture, source, placement,
                {{{low.x, high.y, low.z}, {high.x, high.y, low.z}, {high.x, low.y, low.z}, {low.x, low.y, low.z}}});
        AddQuad(fixture, source, placement,
                {{{low.x, low.y, low.z}, {high.x, low.y, low.z}, {high.x, low.y, high.z}, {low.x, low.y, high.z}}});
        AddQuad(fixture, source, placement,
                {{{high.x, high.y, low.z}, {low.x, high.y, low.z}, {low.x, high.y, high.z}, {high.x, high.y, high.z}}});
        AddQuad(fixture, source, placement,
                {{{low.x, high.y, low.z}, {low.x, low.y, low.z}, {low.x, low.y, high.z}, {low.x, high.y, high.z}}});
        AddQuad(fixture, source, placement,
                {{{high.x, low.y, low.z}, {high.x, high.y, low.z}, {high.x, high.y, high.z}, {high.x, low.y, high.z}}});
    }

    std::size_t AddEntrance(ObstacleFixture &fixture, Vec3 position)
    {
        const auto index = fixture.entrances.size();
        fixture.entrances.push_back({.referenceId = static_cast<std::uint32_t>(0x100 + index), .position = position});
        return index;
    }

    void AddStaircase(ObstacleFixture &fixture, const std::string &name, const Placement &placement, float tread,
                      float rise, bool traversable)
    {
        constexpr int count = 16;
        constexpr float width = 128;
        const auto source = AddSource(fixture, name);
        AddBox(fixture, source, placement, {-128, 0, -32}, {0, width, 0});
        const auto entrance = AddEntrance(fixture, placement.World({-64, width / 2, 0}));
        fixture.probes.push_back({name + " lower landing", placement.World({-64, width / 2, 0}), entrance});
        for (int step = 0; step < count; ++step)
        {
            const auto start = step * tread;
            const auto height = (step + 1) * rise;
            AddBox(fixture, source, placement, {start, 0, -32}, {start + tread, width, height});
            fixture.probes.push_back({name + " tread " + std::to_string(step),
                                      placement.World({start + tread / 2, width / 2, height}), entrance, traversable});
        }
        AddBox(fixture, source, placement, {count * tread, 0, -32}, {count * tread + 128, width, count * rise});
        fixture.probes.push_back({name + " upper landing",
                                  placement.World({count * tread + 64, width / 2, count * rise}), entrance,
                                  traversable});
    }

    void AddSwitchback(ObstacleFixture &fixture)
    {
        const Placement placement{{0, 900, 0}};
        const auto source = AddSource(fixture, "switchback stairs");
        AddBox(fixture, source, placement, {-128, 0, -32}, {0, 128, 0});
        const auto entrance = AddEntrance(fixture, placement.World({-64, 64, 0}));
        for (int step = 0; step < 8; ++step)
        {
            const float x = step * 24.0F;
            const float height = (step + 1) * 24.0F;
            AddBox(fixture, source, placement, {x, 0, -32}, {x + 24, 128, height});
            AddBox(fixture, source, placement, {168 - x, 128, -32}, {192 - x, 256, 192 + height});
            fixture.probes.push_back({"switchback outward tread", placement.World({x + 12, 64, height}), entrance});
            fixture.probes.push_back(
                {"switchback return tread", placement.World({180 - x, 192, 192 + height}), entrance});
        }
        AddBox(fixture, source, placement, {192, 0, -32}, {320, 256, 192});
        AddBox(fixture, source, placement, {-128, 128, -32}, {0, 256, 384});
        fixture.probes.push_back({"switchback turn", placement.World({256, 128, 192}), entrance});
        fixture.probes.push_back({"switchback upper landing", placement.World({-64, 192, 384}), entrance});
    }

    void AddOverpass(ObstacleFixture &fixture, const Placement &placement, float deckHeight, bool underpass)
    {
        const auto name = underpass ? "bridge over road" : "low overpass";
        const auto source = AddSource(fixture, name);
        AddBox(fixture, source, placement, {-128, -384, -32}, {128, 384, 0});
        AddBox(fixture, source, placement, {-384, -64, deckHeight - 24}, {384, 64, deckHeight});
        AddBox(fixture, source, placement, {-24, -24, 0}, {24, 24, deckHeight - 24});
        const auto road = AddEntrance(fixture, placement.World({80, -300, 0}));
        const auto farRoad = AddEntrance(fixture, placement.World({80, 300, 0}));
        const auto bridge = AddEntrance(fixture, placement.World({-300, 0, deckHeight}));
        fixture.separatedEntrances.emplace_back(road, bridge);
        if (!underpass)
        {
            fixture.separatedEntrances.emplace_back(road, farRoad);
        }
        fixture.probes.push_back(
            {std::string(name) + " road beneath deck", placement.World({80, 0, 0}), road, underpass});
        fixture.probes.push_back(
            {std::string(name) + " far road", placement.World({80, 300, 0}), underpass ? road : farRoad});
        fixture.probes.push_back({std::string(name) + " pier obstruction", placement.World({0, 0, 0}), road, false});
        fixture.probes.push_back({std::string(name) + " deck", placement.World({0, 0, deckHeight}), bridge});
        fixture.probes.push_back({std::string(name) + " far deck", placement.World({300, 0, deckHeight}), bridge});
    }

    void AddAccessibleBridge(ObstacleFixture &fixture)
    {
        const Placement placement{{1500, 1900, 0}};
        const auto stairs = fixture.entrances.size();
        AddStaircase(fixture, "bridge access stairs", placement, 32, 24, true);
        const auto source = AddSource(fixture, "bridge with stair access");
        AddBox(fixture, source, placement, {512, 0, 360}, {1664, 128, 384});
        AddBox(fixture, source, placement, {960, -320, -32}, {1216, 448, 0});
        const auto road = AddEntrance(fixture, placement.World({1088, -256, 0}));
        fixture.separatedEntrances.emplace_back(stairs, road);
        fixture.probes.push_back({"accessible bridge deck", placement.World({1088, 64, 384}), stairs});
        fixture.probes.push_back({"accessible bridge far deck", placement.World({1600, 64, 384}), stairs});
        fixture.probes.push_back({"accessible bridge road", placement.World({1088, 64, 0}), road});
    }

    // A descent starts on a border-reaching street and meets the lower road
    // beneath a crossing deck. Uneven risers exercise climb quantization without
    // inventing links to the elevated deck or bypassing headroom filtering.
    void AddCornerDescent(ObstacleFixture &fixture, const Placement &placement)
    {
        constexpr int count = 15;
        constexpr float tread = 48;
        constexpr float rise = 32;
        constexpr float width = 192;
        constexpr float streetHeight = count * rise;
        const auto source = AddSource(fixture, "corner descent beneath bridge");
        AddBox(fixture, source, placement, {-256, -80, -32}, {0, 416, streetHeight});
        const auto street = AddEntrance(fixture, placement.World({-64, width / 2, streetHeight}));
        fixture.probes.push_back({"corner upper street", placement.World({-64, width / 2, streetHeight}), street});
        for (int step = 0; step < count; ++step)
        {
            const float height = streetHeight - (step + 1) * rise - (step % 4 == 2 && step + 1 < count ? 3 : 0);
            const float x = step * tread;
            AddBox(fixture, source, placement, {x, 0, -32}, {x + tread, width, height});
            fixture.probes.push_back({"corner descending tread " + std::to_string(step),
                                      placement.World({x + tread / 2, width / 2, height}), street});
        }
        AddBox(fixture, source, placement, {count * tread, -128, -32}, {1408, 256, 0});
        AddBox(fixture, source, placement, {1024, -256, streetHeight - 16}, {1152, 416, streetHeight});
        fixture.probes.push_back({"corner lower landing", placement.World({800, width / 2, 0}), street});
        fixture.probes.push_back({"corner route under bridge", placement.World({1088, width / 2, 0}), street});
        fixture.probes.push_back({"corner far lower road", placement.World({1280, width / 2, 0}), street});
        const auto deck = AddEntrance(fixture, placement.World({1088, -192, streetHeight}));
        fixture.separatedEntrances.emplace_back(street, deck);
        fixture.probes.push_back({"corner crossing deck", placement.World({1088, width / 2, streetHeight}), deck});
    }

    ObstacleFixture BuildObstacleFixture()
    {
        ObstacleFixture fixture;
        AddStaircase(fixture, "narrow stairs", {{0, 0, 0}}, 12, 24, true);
        AddStaircase(fixture, "diagonal stairs", {{0, 450, 0}, 0.785398163F}, 16, 24, true);
        AddSwitchback(fixture);
        AddStaircase(fixture, "oversized steps", {{0, 1700, 0}}, 64, 48, false);
        AddOverpass(fixture, {{1500, 0, 0}}, 256, true);
        AddOverpass(fixture, {{1500, 1000, 0}}, 96, false);
        AddAccessibleBridge(fixture);
        AddCornerDescent(fixture, {{0, 2800, 0}});
        return fixture;
    }

    float Cross(Vec3 a, Vec3 b, Vec3 c)
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    // Probe the actual triangle footprint and height, rather than accepting a
    // nearby polygon on another floor or an entrance's generous matching reach.
    std::optional<std::uint32_t> SurfaceRegion(const CandidateNavMesh &candidate, Vec3 point)
    {
        for (const auto &region : candidate.regions)
        {
            for (const auto polygon : region.polygons)
            {
                const auto &face = candidate.mesh.polygons[polygon];
                const auto a = candidate.mesh.vertices[face.vertices[0]];
                const auto b = candidate.mesh.vertices[face.vertices[1]];
                const auto c = candidate.mesh.vertices[face.vertices[2]];
                const auto area = Cross(a, b, c);
                if (std::abs(area) < 0.0001F)
                {
                    continue;
                }
                const auto u = Cross(point, b, c) / area;
                const auto v = Cross(a, point, c) / area;
                const auto w = 1 - u - v;
                const auto height = u * a.z + v * b.z + w * c.z;
                if (u >= -0.001F && v >= -0.001F && w >= -0.001F &&
                    std::abs(height - point.z) <= candidate.profile.stepHeight)
                {
                    return region.id;
                }
            }
        }
        return std::nullopt;
    }

    void CheckObstacleFixture(const ObstacleFixture &fixture, const CandidateNavMesh &candidate)
    {
        assert(candidate.topology.valid);
        for (const auto &probe : fixture.probes)
        {
            const auto region = SurfaceRegion(candidate, probe.position);
            const bool correct = probe.covered ? region && region == candidate.exits[probe.entrance].region : !region;
            if (!correct)
            {
                std::cerr << "Navigation fixture failed (" << candidate.partitioningAlgorithm << "): " << probe.name
                          << '\n';
                std::exit(EXIT_FAILURE);
            }
        }
        // Stacked levels and opposite sides of a blocked underpass cannot join
        // through XY overlap or the entrance matching reach.
        for (const auto [first, second] : fixture.separatedEntrances)
        {
            assert(candidate.exits[first].region && candidate.exits[second].region);
            assert(candidate.exits[first].region != candidate.exits[second].region);
        }
    }

    void CheckCompactStraightFlight(RegionPartitioningAlgorithm partitioning)
    {
        ObstacleFixture flight;
        const auto source = AddSource(flight, "straight flight");
        constexpr int count = 16;
        constexpr float tread = 32;
        constexpr float rise = 24;
        for (int step{}; step < count; ++step)
        {
            const float x = step * tread;
            const float height = (step + 1) * rise;
            AddBox(flight, source, {}, {x, 0, -32}, {x + tread, 128, height});
            const auto probeX = step == 0 ? x + 24 : step + 1 == count ? x + 8 : x + tread / 2;
            flight.probes.push_back({"straight flight tread " + std::to_string(step), {probeX, 64, height}, 0});
        }
        AddEntrance(flight, {tread / 2, 64, rise});
        const auto candidate =
            RecastCandidateGenerator{}.Generate(flight.scene, {}, std::nullopt, flight.entrances, partitioning);
        CheckObstacleFixture(flight, candidate);
        assert(candidate.mesh.polygons.size() == 2);
    }

    void CheckFloorSeams(RegionPartitioningAlgorithm partitioning)
    {
        // Separated collision pieces model a landing and a first tread. Repairs
        // must retain shared-edge routes while obeying solid, climb and clearance limits.
        for (const auto [gap, rise, obstruction, connected] :
             {std::tuple{8.0F, 24.0F, 0, true}, std::tuple{48.0F, 24.0F, 0, false}, std::tuple{8.0F, 64.0F, 0, false},
              std::tuple{8.0F, 24.0F, 1, false}, std::tuple{8.0F, 24.0F, 2, false}, std::tuple{8.0F, 24.0F, 3, false}})
        {
            ObstacleFixture fixture;
            const auto floor = AddSource(fixture, "separated floor pieces");
            AddBox(fixture, floor, {}, {-256, -128, -32}, {0, 128, 0});
            AddBox(fixture, floor, {}, {gap, -128, -32}, {gap + 256, 128, rise});
            if (obstruction == 1)
            {
                AddBox(fixture, floor, {}, {0, -128, 0}, {gap, 128, 256});
            }
            else if (obstruction == 2)
            {
                AddBox(fixture, floor, {}, {-32, -128, 96}, {gap + 32, 128, 112});
            }
            else if (obstruction == 3)
            {
                const auto solid = AddSource(fixture, "excluded solid in seam");
                fixture.scene.geometrySources[solid].navigationObstacle = true;
                AddBox(fixture, solid, {}, {0, -128, -32}, {gap, 128, rise});
            }
            AddEntrance(fixture, {-128, 0, 0});
            AddEntrance(fixture, {gap + 128, 0, rise});
            const auto candidate =
                RecastCandidateGenerator{}.Generate(fixture.scene, {}, std::nullopt, fixture.entrances, partitioning);
            assert(candidate.topology.valid);
            const auto first = SurfaceRegion(candidate, {-128, 0, 0});
            const auto second = SurfaceRegion(candidate, {gap + 128, 0, rise});
            assert(first && second);
            assert((first == second) == connected);
            assert(SurfaceRegion(candidate, {gap / 2, 0, rise}).has_value() == connected);
        }
    }

    void CheckObstacleOnlyCollision(RegionPartitioningAlgorithm partitioning)
    {
        ObstacleFixture fixture;
        const auto floor = AddSource(fixture, "floor around solid");
        AddBox(fixture, floor, {}, {-512, -256, -32}, {512, 256, 0});
        const auto rock = AddSource(fixture, "low solid");
        // Its top is within climb reach and large enough to survive region filtering.
        // Obstacle tagging must prevent navigation even when the floor is connected.
        fixture.scene.geometrySources[rock].navigationObstacle = true;
        AddBox(fixture, rock, {}, {-96, -96, -32}, {96, 96, 24});
        AddEntrance(fixture, {-384, 0, 0});
        fixture.probes = {{"solid top excluded", {0, 0, 24}, 0, false},
                          {"solid footprint blocked", {0, 0, 0}, 0, false},
                          {"route beside solid", {0, 192, 0}, 0},
                          {"far floor connected", {384, 0, 0}, 0}};
        const RecastCandidateGenerator generator;
        const auto candidate = generator.Generate(fixture.scene, {}, std::nullopt, fixture.entrances, partitioning);
        CheckObstacleFixture(fixture, candidate);
        assert(candidate.statistics.rejectedObstruction > 0);
        for (const auto source : candidate.polygonSourceTriangles)
        {
            assert(fixture.scene.triangleProvenance[source].geometrySource == floor);
        }
        // Untagged low collision remains eligible; the policy cannot remove arbitrary steps.
        fixture.scene.geometrySources[rock].navigationObstacle = false;
        const auto walkable = generator.Generate(fixture.scene, {}, std::nullopt, fixture.entrances, partitioning);
        assert(SurfaceRegion(walkable, {0, 0, 24}) == walkable.exits[0].region);
        fixture.scene.geometrySources[floor].navigationObstacle = true;
        fixture.scene.geometrySources[rock].navigationObstacle = true;
        const auto allObstacles = generator.Generate(fixture.scene, {}, std::nullopt, fixture.entrances, partitioning);
        assert(allObstacles.mesh.polygons.empty() && !allObstacles.exits[0].polygon);
    }
} // namespace

void TestNavigationObstacleFixture()
{
    const auto fixture = BuildObstacleFixture();
    const RecastCandidateGenerator generator;
    for (const auto partitioning : {RegionPartitioningAlgorithm::Watershed, RegionPartitioningAlgorithm::Monotone,
                                    RegionPartitioningAlgorithm::Layers})
    {
        CheckCompactStraightFlight(partitioning);
        CheckFloorSeams(partitioning);
        CheckObstacleOnlyCollision(partitioning);
        const auto candidate = generator.Generate(fixture.scene, {}, std::nullopt, fixture.entrances, partitioning);
        CheckObstacleFixture(fixture, candidate);
    }
    NavigationProfile quantized;
    quantized.stepHeight = 35;
    RecastSettings coarse;
    coarse.cellSize = 8;
    coarse.cellHeight = 8;
    const auto candidate = generator.Generate(fixture.scene, quantized, std::nullopt, fixture.entrances,
                                              RegionPartitioningAlgorithm::Watershed, coarse);
    assert(std::any_of(candidate.warnings.begin(), candidate.warnings.end(),
                       [](const auto &warning) { return warning.contains("step height down to 32"); }));

    ObstacleFixture corner;
    AddCornerDescent(corner, {});
    const AABB cornerBounds{.min = {-128, -256, -64}, .max = {1536, 304, 1024}};
    for (const auto partitioning : {RegionPartitioningAlgorithm::Watershed, RegionPartitioningAlgorithm::Monotone,
                                    RegionPartitioningAlgorithm::Layers})
    {
        // The street's true boundary edges anchor this exterior path without a
        // synthetic door. Check its entire descent after halo rasterization and clipping.
        const auto generated = generator.Generate(corner.scene, {}, cornerBounds, {}, partitioning);
        assert(generated.topology.valid);
        const auto street = SurfaceRegion(generated, {-64, 96, 480});
        assert(street && generated.regions[*street].reachesBorder);
        for (const auto &probe : corner.probes)
        {
            if (probe.entrance == 0)
            {
                const auto region = SurfaceRegion(generated, probe.position);
                if (region != street)
                {
                    std::cerr << "Border-anchored fixture failed: " << probe.name << '\n';
                    std::exit(EXIT_FAILURE);
                }
            }
        }
    }
    NavigationProfile insufficientClimb;
    insufficientClimb.stepHeight = 28;
    const auto blockedDescent = generator.Generate(corner.scene, insufficientClimb, cornerBounds, {});
    assert(!SurfaceRegion(blockedDescent, {800, 96, 0}));
}

void ExportNavigationObstacleFixture(const std::filesystem::path &directory)
{
    const auto fixture = BuildObstacleFixture();
    const auto candidate = RecastCandidateGenerator{}.Generate(fixture.scene, {}, std::nullopt, fixture.entrances);
    CheckObstacleFixture(fixture, candidate);
    std::filesystem::create_directories(directory);
    if (!WriteCandidateJson(directory / "candidate-navm.json", candidate, fixture.scene, "{}"))
    {
        throw std::runtime_error("Cannot write navigation fixture candidate");
    }
    const SceneExportOptions options{
        .layers = {SceneLayer::Collision, SceneLayer::CandidateNavmesh, SceneLayer::DiagnosticMarkers},
        .candidateNavmesh = &candidate.mesh,
        .candidateEntrances = &candidate.exits};
    const auto exported = WriteCombinedGlb(directory / "scene.glb", fixture.scene, {}, {}, {}, options);
    assert(exported.triangles > candidate.mesh.polygons.size());
    std::cout << "Exported synthetic stairs and overpasses to " << directory / "scene.glb" << '\n';
}
