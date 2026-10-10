#ifdef NDEBUG
#undef NDEBUG
#endif

#include "core/navmesh/generator.h"
#include "core/scene/scene_exporter.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    using namespace navmesh::core;

    struct SpiralCase
    {
        std::string name;
        bool stairs{};
        float diameter{};
        float width{96};
        float rise{};
        int segmentsPerTurn{64};
        int routeSegments{144};
        int landingSegments{8};
        bool expectedTraversable{true};
        Vec3 origin;
        Scene scene;
        std::vector<Vec3> probes;
        std::vector<Vec3> voidProbes;
    };

    constexpr float TargetTreadLength = 36;
    constexpr float StairRiseToRun = 0.5F;
    constexpr float CorridorGap = 48;
    constexpr float SlabThickness = 8;

    Vec3 PolarPoint(const SpiralCase &specification, float segment, float offset, float height)
    {
        const auto angle = segment * 2 * std::numbers::pi_v<float> / specification.segmentsPerTurn;
        const auto pitch = specification.stairs ? 0 : specification.width + CorridorGap;
        const auto radius =
            specification.diameter / 2 + pitch * std::max(segment, 0.0F) / specification.segmentsPerTurn + offset;
        return specification.origin + Vec3{radius * std::cos(angle), radius * std::sin(angle), height};
    }

    void AddQuad(Scene &scene, const std::array<Vec3, 4> &corners, bool reverse = false)
    {
        const auto base = static_cast<std::uint32_t>(scene.mesh.vertices.size());
        scene.mesh.vertices.insert(scene.mesh.vertices.end(), corners.begin(), corners.end());
        for (auto face : {std::array{base, base + 1, base + 2}, std::array{base, base + 2, base + 3}})
        {
            if (reverse)
            {
                std::swap(face[1], face[2]);
            }
            const auto triangle = scene.mesh.triangles.size();
            scene.mesh.triangles.push_back({face});
            scene.triangleProvenance.push_back({0, triangle, {}});
        }
    }

    SpiralCase BuildSpiral(SpiralCase specification)
    {
        specification.scene.geometrySources.push_back({.modelPath = "synthetic/" + specification.name,
                                                       .sourceType = GeometrySourceType::Collision,
                                                       .confidence = 1.0F,
                                                       .reference = {"Synthetic.esm", 1, "REFR"}});
        // Each annular wedge is a closed thin slab, not a column to ground:
        // successive revolutions must leave actual air beneath the upper treads.
        // Skyrim Z is vertical; planar corridors grow radially to avoid overlap.
        for (int segment = -specification.landingSegments;
             segment < specification.routeSegments + specification.landingSegments; ++segment)
        {
            const auto step = std::clamp(segment + 1, 0, specification.routeSegments);
            const auto height = specification.stairs ? step * specification.rise : 0;
            const auto halfWidth = specification.width / 2;
            const auto startSegment = static_cast<float>(segment);
            const auto endSegment = startSegment + 1;
            const std::array top{PolarPoint(specification, startSegment, -halfWidth, height),
                                 PolarPoint(specification, startSegment, halfWidth, height),
                                 PolarPoint(specification, endSegment, halfWidth, height),
                                 PolarPoint(specification, endSegment, -halfWidth, height)};
            auto bottom = top;
            for (auto &point : bottom)
            {
                point.z -= SlabThickness;
            }
            AddQuad(specification.scene, top);
            AddQuad(specification.scene, bottom, true);
            for (std::size_t edge{}; edge < top.size(); ++edge)
            {
                const auto next = (edge + 1) % top.size();
                AddQuad(specification.scene, {bottom[edge], bottom[next], top[next], top[edge]});
            }
            // Centerline probes avoid radial erosion. End probes lie within
            // flat landing extensions so route failures cannot be end-cap erosion.
            if (segment >= 0 && segment < specification.routeSegments)
            {
                specification.probes.push_back(PolarPoint(specification, segment + 0.5F, 0, height));
            }
        }
        const auto lower = PolarPoint(specification, -specification.landingSegments / 2.0F, 0, 0);
        const auto upper = PolarPoint(specification, specification.routeSegments + specification.landingSegments / 2.0F,
                                      0, specification.stairs ? specification.routeSegments * specification.rise : 0);
        specification.probes.insert(specification.probes.begin(), lower);
        specification.probes.push_back(upper);
        // Sample the central shaft and gaps between planar windings. Navigation
        // bridging these unsupported spaces would be a false shortcut even if
        // every intended tread lies in the entrance's connected component.
        for (int segment{}; segment < specification.routeSegments; segment += specification.landingSegments)
        {
            const auto height = specification.stairs ? (segment + 1) * specification.rise : 0;
            specification.voidProbes.push_back(specification.origin + Vec3{0, 0, height});
            if (!specification.stairs && segment < specification.segmentsPerTurn)
            {
                specification.voidProbes.push_back(
                    PolarPoint(specification, segment + 0.5F, specification.width / 2 + CorridorGap / 2, 0));
            }
        }
        return specification;
    }

    SpiralCase BuildGentleStairs(float diameter, Vec3 origin)
    {
        const auto circumference = diameter * std::numbers::pi_v<float>;
        // Round the tread count to whole quarter-turns so every staircase has
        // the same angular route. Centerline tread depth stays near the wooden
        // stair reference, and rise/run fixes the incline across diameters.
        const auto quarterTurnSteps = std::max(1, static_cast<int>(std::round(circumference / TargetTreadLength / 4)));
        const auto stepsPerTurn = quarterTurnSteps * 4;
        return BuildSpiral({.name = "stairs-d" + std::to_string(static_cast<int>(diameter)),
                            .stairs = true,
                            .diameter = diameter,
                            .rise = circumference / stepsPerTurn * StairRiseToRun,
                            .segmentsPerTurn = stepsPerTurn,
                            .routeSegments = stepsPerTurn * 2 + quarterTurnSteps,
                            .landingSegments = std::max(2, static_cast<int>(std::ceil(stepsPerTurn / 8.0F))),
                            .origin = origin});
    }

    std::vector<SpiralCase> BuildSpiralCases()
    {
        std::vector<SpiralCase> cases;
        for (const bool stairs : {false, true})
        {
            std::size_t column{};
            for (const float diameter : {128.0F, 192.0F, 256.0F, 384.0F, 512.0F})
            {
                const Vec3 origin{static_cast<float>(column++) * 1600, stairs ? 2000.0F : 0, 0};
                if (stairs)
                {
                    cases.push_back(BuildGentleStairs(diameter, origin));
                }
                else
                {
                    cases.push_back(BuildSpiral({.name = "corridor-d" + std::to_string(static_cast<int>(diameter)),
                                                 .diameter = diameter,
                                                 .origin = origin}));
                }
            }
        }
        cases.push_back(BuildSpiral({.name = "corridor-insufficient-width",
                                     .diameter = 256,
                                     .width = 24,
                                     .expectedTraversable = false,
                                     .origin = {0, 4000, 0}}));
        cases.push_back(BuildSpiral({.name = "stairs-insufficient-headroom",
                                     .stairs = true,
                                     .diameter = 256,
                                     .rise = 1.5F,
                                     .expectedTraversable = false,
                                     .origin = {1600, 4000, 0}}));
        return cases;
    }

    void AppendScene(Scene &destination, const Scene &source)
    {
        const auto vertexBase = static_cast<std::uint32_t>(destination.mesh.vertices.size());
        const auto sourceBase = destination.geometrySources.size();
        destination.mesh.vertices.insert(destination.mesh.vertices.end(), source.mesh.vertices.begin(),
                                         source.mesh.vertices.end());
        destination.geometrySources.insert(destination.geometrySources.end(), source.geometrySources.begin(),
                                           source.geometrySources.end());
        for (std::size_t index{}; index < source.mesh.triangles.size(); ++index)
        {
            auto face = source.mesh.triangles[index];
            for (auto &vertex : face.vertices)
            {
                vertex += vertexBase;
            }
            auto provenance = source.triangleProvenance[index];
            provenance.geometrySource += sourceBase;
            destination.mesh.triangles.push_back(face);
            destination.triangleProvenance.push_back(provenance);
        }
    }

    float Cross(Vec3 a, Vec3 b, Vec3 c)
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    // Test the triangle's actual XY footprint and interpolated Z. A nearby
    // winding or overlapping upper floor cannot supply coverage for this probe.
    std::optional<std::uint32_t> ProbeRegion(const CandidateNavMesh &candidate, Vec3 point)
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

    std::string CsvText(const std::string &value)
    {
        std::string escaped{"\""};
        for (const auto character : value)
        {
            if (character == '"')
            {
                escaped += '"';
            }
            escaped += character;
        }
        return escaped + '"';
    }

    void ExportScene(const std::filesystem::path &directory, const Scene &scene, const CandidateNavMesh *candidate,
                     const std::vector<CandidateExit> &exits)
    {
        std::filesystem::create_directories(directory);
        const SceneExportOptions options{
            .layers = {SceneLayer::Collision, SceneLayer::CandidateNavmesh, SceneLayer::DiagnosticMarkers},
            .candidateNavmesh = candidate ? &candidate->mesh : nullptr,
            .candidateEntrances = candidate ? &candidate->exits : &exits};
        if (!WriteCombinedGlb(directory / "scene.glb", scene, {}, {}, {}, options).written)
        {
            throw std::runtime_error("Cannot write spiral fixture scene");
        }
        if (candidate && !WriteCandidateJson(directory / "candidate-navm.json", *candidate, scene, "{}"))
        {
            throw std::runtime_error("Cannot write spiral fixture candidate");
        }
    }

    void ReportCase(std::ostream &summary, std::ostream &probes, const SpiralCase &specification,
                    const CandidateNavMesh *candidate, const std::string &configuration, const std::string &scope,
                    double milliseconds, const std::string &error)
    {
        const auto lower = candidate ? ProbeRegion(*candidate, specification.probes.front()) : std::nullopt;
        const auto upper = candidate ? ProbeRegion(*candidate, specification.probes.back()) : std::nullopt;
        std::size_t covered{};
        std::size_t connected{};
        for (std::size_t index{}; index < specification.probes.size(); ++index)
        {
            const auto point = specification.probes[index];
            const auto region = candidate ? ProbeRegion(*candidate, point) : std::nullopt;
            const bool reachesLower = lower && region == lower;
            covered += region.has_value();
            connected += reachesLower;
            probes << configuration << ',' << scope << ',' << specification.name << ',' << index << ',' << point.x
                   << ',' << point.y << ',' << point.z << ",1," << region.has_value() << ',' << reachesLower << '\n';
        }
        std::size_t unsupportedCovered{};
        for (std::size_t index{}; index < specification.voidProbes.size(); ++index)
        {
            const auto point = specification.voidProbes[index];
            const auto region = candidate ? ProbeRegion(*candidate, point) : std::nullopt;
            unsupportedCovered += region.has_value();
            probes << configuration << ',' << scope << ',' << specification.name << ','
                   << specification.probes.size() + index << ',' << point.x << ',' << point.y << ',' << point.z << ",0,"
                   << region.has_value() << ',' << (lower && region == lower) << '\n';
        }
        std::string diagnostics = error;
        if (candidate)
        {
            for (const auto &warning : candidate->warnings)
            {
                diagnostics += warning + " | ";
            }
            for (const auto &finding : candidate->topology.findings)
            {
                diagnostics += finding + " | ";
            }
        }
        summary << configuration << ',' << scope << ',' << specification.name << ',' << specification.diameter << ','
                << specification.width << ',' << (specification.stairs ? specification.rise : 0) << ','
                << specification.expectedTraversable << ',' << (candidate != nullptr) << ','
                << (candidate && candidate->topology.valid) << ',' << (candidate ? candidate->mesh.polygons.size() : 0)
                << ',' << specification.probes.size() << ',' << covered << ',' << connected << ','
                << (lower && upper == lower) << ',' << unsupportedCovered << ',' << milliseconds << ','
                << CsvText(diagnostics) << '\n';
        std::cout << configuration << '/' << scope << '/' << specification.name << ": " << connected << '/'
                  << specification.probes.size() << " probes connected, end=" << (lower && upper == lower)
                  << ", time=" << milliseconds << " ms" << (error.empty() ? "" : ", " + error) << '\n';
    }
} // namespace

void TestSpiralNavigationFixture()
{
    const auto cases = BuildSpiralCases();
    for (const auto algorithm : {RegionPartitioningAlgorithm::Watershed, RegionPartitioningAlgorithm::Monotone,
                                 RegionPartitioningAlgorithm::Layers})
    {
        for (const auto &specification : cases)
        {
            if (specification.stairs && specification.expectedTraversable)
            {
                const auto tread = specification.diameter * std::numbers::pi_v<float> / specification.segmentsPerTurn;
                assert(specification.rise / tread < 0.6F);
                assert(specification.rise <= NavigationProfile{}.stepHeight);
                assert(specification.segmentsPerTurn * specification.rise - SlabThickness >
                       NavigationProfile{}.agentHeight);
            }
            if (specification.expectedTraversable && specification.diameter != 256)
            {
                continue;
            }
            const std::vector<CandidateExit> exits{{.referenceId = 1, .position = specification.probes.front()}};
            const auto candidate =
                RecastCandidateGenerator{}.Generate(specification.scene, {}, std::nullopt, exits, algorithm);
            assert(candidate.topology.valid);
            const auto lower = ProbeRegion(candidate, specification.probes.front());
            if (specification.expectedTraversable &&
                (!specification.stairs || algorithm != RegionPartitioningAlgorithm::Layers))
            {
                assert(lower);
                for (const auto point : specification.probes)
                {
                    assert(ProbeRegion(candidate, point) == lower);
                }
            }
            else if (!specification.expectedTraversable)
            {
                assert(!lower || ProbeRegion(candidate, specification.probes.back()) != lower);
            }
            for (const auto point : specification.voidProbes)
            {
                assert(!ProbeRegion(candidate, point));
            }
        }
    }
}

void ExportSpiralNavigationFixture(const std::filesystem::path &directory)
{
    const auto cases = BuildSpiralCases();
    Scene combined;
    std::vector<CandidateExit> exits;
    for (const auto &specification : cases)
    {
        AppendScene(combined, specification.scene);
        exits.push_back(
            {.referenceId = static_cast<std::uint32_t>(exits.size() + 1), .position = specification.probes.front()});
    }
    std::filesystem::create_directories(directory);
    std::ofstream summary(directory / "summary.csv");
    std::ofstream probes(directory / "probes.csv");
    std::ofstream parameters(directory / "parameters.csv");
    if (!summary || !probes || !parameters)
    {
        throw std::runtime_error("Cannot open spiral fixture reports");
    }
    summary << "configuration,scope,case,start_centerline_diameter,width,riser,expected_traversable,generated,"
               "topology_valid,run_polygons,probe_count,covered_probes,connected_probes,end_connected,"
               "unsupported_covered_probes,run_milliseconds,diagnostics\n";
    probes << "configuration,scope,case,probe,x,y,z,expected_covered,covered,connected_to_lower\n";
    parameters << "case,kind,start_centerline_diameter,width,riser,segments_per_turn,route_segments,landing_segments,"
                  "corridor_gap,slab_thickness,origin_x,origin_y,origin_z,expected_traversable,centerline_tread,"
                  "incline_degrees,revolution_headroom\n";
    for (const auto &specification : cases)
    {
        parameters << specification.name << ',' << (specification.stairs ? "stairs" : "corridor") << ','
                   << specification.diameter << ',' << specification.width << ','
                   << (specification.stairs ? specification.rise : 0) << ',' << specification.segmentsPerTurn << ','
                   << specification.routeSegments << ',' << specification.landingSegments << ','
                   << (specification.stairs ? 0 : CorridorGap) << ',' << SlabThickness << ',' << specification.origin.x
                   << ',' << specification.origin.y << ',' << specification.origin.z << ','
                   << specification.expectedTraversable;
        const auto tread = specification.diameter * std::numbers::pi_v<float> / specification.segmentsPerTurn;
        parameters << ',' << tread << ','
                   << (specification.stairs ? std::atan(specification.rise / tread) * 180 / std::numbers::pi_v<float>
                                            : 0)
                   << ','
                   << (specification.stairs ? specification.segmentsPerTurn * specification.rise - SlabThickness : 0)
                   << '\n';
    }
    parameters.flush();
    if (!parameters)
    {
        throw std::runtime_error("Cannot write spiral fixture parameters");
    }
    // Separate builds attribute failures to geometry; the combined build also
    // exercises scene-extent adaptation. One lower entrance per object prevents
    // an upper anchor from concealing a broken route through retention filtering.
    for (const bool fineContours : {false, true})
    {
        RecastSettings settings;
        if (fineContours)
        {
            settings.maxSimplificationError = 0.5F;
        }
        for (const auto algorithm : {RegionPartitioningAlgorithm::Watershed, RegionPartitioningAlgorithm::Monotone,
                                     RegionPartitioningAlgorithm::Layers})
        {
            const auto algorithmName = algorithm == RegionPartitioningAlgorithm::Watershed  ? "watershed"
                                       : algorithm == RegionPartitioningAlgorithm::Monotone ? "monotone"
                                                                                            : "layers";
            const auto configuration = std::string(algorithmName) + (fineContours ? "-fine" : "-default");
            for (std::size_t index{}; index <= cases.size(); ++index)
            {
                const bool allCases = index == cases.size();
                const auto &scene = allCases ? combined : cases[index].scene;
                const auto scope = allCases ? "combined" : "isolated";
                const auto output = directory / configuration / (allCases ? "combined" : cases[index].name);
                const auto inputExits = allCases ? exits : std::vector<CandidateExit>{exits[index]};
                std::optional<CandidateNavMesh> candidate;
                std::string error;
                const auto start = std::chrono::steady_clock::now();
                try
                {
                    candidate =
                        RecastCandidateGenerator{}.Generate(scene, {}, std::nullopt, inputExits, algorithm, settings);
                }
                catch (const std::exception &failure)
                {
                    error = failure.what();
                }
                const auto elapsed =
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start);
                ExportScene(output, scene, candidate ? &*candidate : nullptr, inputExits);
                for (std::size_t caseIndex = allCases ? 0 : index; caseIndex < (allCases ? cases.size() : index + 1);
                     ++caseIndex)
                {
                    ReportCase(summary, probes, cases[caseIndex], candidate ? &*candidate : nullptr, configuration,
                               scope, elapsed.count(), error);
                }
            }
        }
    }
    summary.flush();
    probes.flush();
    if (!summary || !probes)
    {
        throw std::runtime_error("Cannot finish spiral fixture reports");
    }
    std::cout << "Exported spiral scenes and reports to " << directory << '\n';
}
