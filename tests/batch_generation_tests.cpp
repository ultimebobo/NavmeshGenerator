#ifdef NDEBUG
#undef NDEBUG
#endif

#include "app/batch_generation.h"
#include "app/candidate_cache.h"
#include "core/navmesh/batch_stitching.h"
#include "core/navmesh/detail_triangulation.h"
#include "core/navmesh/generator.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
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
} // namespace

void TestGeneratedBatchTopology()
{
    using namespace navmesh::core;
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
    assert(corner.borderLinks.size() == 1 && corner.borderLinks[0].neighborNavmeshId == west.id);
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
