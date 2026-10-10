#ifdef NDEBUG
#undef NDEBUG
#endif

#include "core/navmesh/generator.h"
#include "core/navmesh/recast_contours.h"

#include <RecastAlloc.h>
#include <algorithm>
#include <array>
#include <cassert>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace
{
    using namespace navmesh::core;

    /// Flat floor with an enclosed region, a retained small island, and an already filtered island.
    auto MakeEnclosedRegionField(rcContext &context, bool stacked = false)
    {
        std::unique_ptr<rcHeightfield, decltype(&rcFreeHeightField)> heightfield(rcAllocHeightfield(),
                                                                                 rcFreeHeightField);
        const float low[]{0, 0, 0};
        const float high[]{18, 10, 8};
        assert(rcCreateHeightfield(&context, *heightfield, 18, 8, low, high, 1, 1));
        for (int y{}; y < 8; ++y)
        {
            for (int x{}; x < 18; ++x)
            {
                const bool floor = x < 8;
                const bool island = y >= 2 && y < 4 && ((x >= 10 && x < 12) || (x >= 14 && x < 16));
                if (floor || island)
                {
                    assert(rcAddSpan(&context, *heightfield, x, y, 0, 1, 1, 0));
                }
                if (floor && stacked)
                {
                    assert(rcAddSpan(&context, *heightfield, x, y, 8, 9, 1, 0));
                }
            }
        }
        std::unique_ptr<rcCompactHeightfield, decltype(&rcFreeCompactHeightfield)> compact(rcAllocCompactHeightfield(),
                                                                                           rcFreeCompactHeightfield);
        assert(rcBuildCompactHeightfield(&context, 3, 1, *heightfield, *compact));
        compact->maxRegions = stacked ? 4 : 3;
        for (int y{}; y < compact->height; ++y)
        {
            for (int x{}; x < compact->width; ++x)
            {
                const auto &cell = compact->cells[x + y * compact->width];
                for (unsigned int span = cell.index; span < cell.index + cell.count; ++span)
                {
                    const bool enclosed = x >= 2 && x < 6 && y >= 2 && y < 6;
                    auto &floorSpan = compact->spans[span];
                    if (floorSpan.y == 9)
                    {
                        floorSpan.reg = 4;
                    }
                    else if (x < 8)
                    {
                        floorSpan.reg = enclosed ? 2 : 1;
                    }
                    else if (x < 12)
                    {
                        floorSpan.reg = 3;
                    }
                    else
                    {
                        floorSpan.reg = 0;
                    }
                }
            }
        }
        return compact;
    }

    bool HasRegion(const rcContourSet &contours, unsigned short region)
    {
        for (int contour{}; contour < contours.nconts; ++contour)
        {
            if (contours.conts[contour].reg == region && contours.conts[contour].nverts >= 3)
            {
                return true;
            }
        }
        return false;
    }

    void CheckEnclosedRegionRecovery(float error, bool stacked = false)
    {
        rcContext context(false);
        auto compact = MakeEnclosedRegionField(context, stacked);
        // The inner interface has just one neighboring region. Even zero error
        // cannot refine its two-point simplified contour into a polygon.
        detail::RecastContourOwner raw(rcAllocContourSet(), rcFreeContourSet);
        assert(rcBuildContours(&context, *compact, 0, 0, *raw));
        assert(HasRegion(*raw, 1) && !HasRegion(*raw, 2));

        rcConfig config{};
        config.maxSimplificationError = error;
        config.minRegionArea = compact->spanCount + 1;
        config.maxVertsPerPoly = 6;
        std::vector<std::string> warnings;
        const auto contours = detail::BuildRetainedRegionContours(context, *compact, config,
                                                                  RegionPartitioningAlgorithm::Watershed, warnings);
        assert(warnings.size() == 1 && warnings.front().contains("layer region recovery"));
        std::size_t retained{};
        for (int span{}; span < compact->spanCount; ++span)
        {
            if (compact->areas[span] != RC_NULL_AREA)
            {
                ++retained;
                assert(HasRegion(*contours, compact->spans[span].reg));
                assert(compact->spans[span].y == 1 || (stacked && compact->spans[span].y == 9));
            }
            else
            {
                assert(compact->spans[span].reg == 0);
            }
        }
        // The small retained island survives repartitioning, while the region-zero
        // island cannot return despite still having a walkable area before recovery.
        assert(retained == 8 * 8 + 2 * 2 + (stacked ? 8 * 8 : 0));
        if (stacked)
        {
            // Layer recovery must keep overlapping floors in distinct regions;
            // contour tracing operates on horizontal boundaries within each layer.
            for (int column{}; column < compact->width * compact->height; ++column)
            {
                const auto &cell = compact->cells[column];
                if (cell.count == 2)
                {
                    assert(compact->spans[cell.index].reg != compact->spans[cell.index + 1].reg);
                }
            }
        }
        std::unique_ptr<rcPolyMesh, decltype(&rcFreePolyMesh)> mesh(rcAllocPolyMesh(), rcFreePolyMesh);
        assert(rcBuildPolyMesh(&context, *contours, config.maxVertsPerPoly, *mesh));
        int doubleArea{};
        for (int polygon{}; polygon < mesh->npolys; ++polygon)
        {
            const auto *vertices = &mesh->polys[polygon * mesh->nvp * 2];
            int count{};
            while (count < mesh->nvp && vertices[count] != RC_MESH_NULL_IDX)
            {
                ++count;
            }
            int area{};
            for (int edge{}; edge < count; ++edge)
            {
                const auto *a = &mesh->verts[vertices[edge] * 3];
                const auto *b = &mesh->verts[vertices[(edge + 1) % count] * 3];
                area += a[0] * b[2] - b[0] * a[2];
            }
            doubleArea += std::abs(area);
        }
        assert(doubleArea == 2 * static_cast<int>(retained));
    }

    void CheckOrdinaryContourRefinement()
    {
        rcContext context(false);
        auto compact = MakeEnclosedRegionField(context);
        for (int span{}; span < compact->spanCount; ++span)
        {
            if (compact->spans[span].reg == 2)
            {
                compact->spans[span].reg = 1;
            }
        }
        rcConfig config{};
        config.maxSimplificationError = 100;
        std::vector<std::string> warnings;
        const auto contours = detail::BuildRetainedRegionContours(context, *compact, config,
                                                                  RegionPartitioningAlgorithm::Watershed, warnings);
        assert(warnings.empty() && HasRegion(*contours, 1) && HasRegion(*contours, 3));
    }

    void CheckCoarsePolygonWinding()
    {
        detail::RecastPolyMeshOwner mesh(rcAllocPolyMesh(), rcFreePolyMesh);
        const std::array<unsigned short, 24> vertices{10, 0, 10, 30, 0, 10, 10, 0, 30, 30, 0, 30,
                                                      10, 8, 10, 30, 8, 10, 10, 8, 30, 30, 8, 30};
        mesh->nvp = 3;
        mesh->npolys = 2;
        mesh->nverts = 8;
        mesh->verts = static_cast<unsigned short *>(rcAlloc(sizeof(vertices), RC_ALLOC_PERM));
        mesh->polys = static_cast<unsigned short *>(rcAlloc(12 * sizeof(unsigned short), RC_ALLOC_PERM));
        assert(mesh->verts && mesh->polys);
        std::memcpy(mesh->verts, vertices.data(), sizeof(vertices));
        std::fill_n(mesh->polys, 12, RC_MESH_NULL_IDX);
        const auto setFace = [&](int polygon, std::array<unsigned short, 3> face)
        { std::copy(face.begin(), face.end(), mesh->polys + polygon * 6); };
        setFace(0, {0, 2, 1});
        setFace(1, {1, 2, 3});
        assert(detail::HasConsistentRegionMesh(*mesh));

        // An inverted coarse face must be rejected before conversion can flip
        // its winding and conceal a folded partition from detail-patch repair.
        setFace(1, {1, 3, 2});
        assert(!detail::HasConsistentRegionMesh(*mesh));
        // Even individually clockwise polygons cannot consume the same directed
        // edge at the same floor height. A separate stacked floor remains valid.
        setFace(1, {0, 2, 1});
        assert(!detail::HasConsistentRegionMesh(*mesh));
        setFace(1, {4, 6, 5});
        assert(detail::HasConsistentRegionMesh(*mesh));
    }

    void CheckCoarseMeshRefinement()
    {
        // Synthetic perforated floor whose simplified contour produces a
        // nonconvex coarse polygon. Keep the irregular boundary location independent.
        constexpr std::array<std::string_view, 24> rows{
            "........................", ".######..#.######.#..##.", ".##.##.######.#.#.#####.",
            "..####.#.#.###########..", ".############.#######.#.", ".##############.#######.",
            "..####.#####..####.###..", ".#####.####.##.########.", ".####.###.###.####.#..#.",
            ".##.##################..", ".###.###.########.#.###.", "..###.#.#.#######.####..",
            ".###########.###..#####.", ".##..#####.#.##..###.#..", ".###############.####.#.",
            ".#######.#.############.", ".#################..#.#.", ".############.##.##.###.",
            ".##.##.##.#########.###.", ".####.########.#####.##.", ".....#####.#####..#####.",
            "..#..###############.##.", ".##..#####..##.########.", "........................"};
        rcContext context(false);
        std::unique_ptr<rcHeightfield, decltype(&rcFreeHeightField)> heightfield(rcAllocHeightfield(),
                                                                                 rcFreeHeightField);
        const float low[]{0, 0, 0};
        const float high[]{24, 10, 24};
        assert(rcCreateHeightfield(&context, *heightfield, 24, 24, low, high, 1, 1));
        for (int y{}; y < 24; ++y)
        {
            for (int x{}; x < 24; ++x)
            {
                if (rows[y][x] == '#')
                {
                    assert(rcAddSpan(&context, *heightfield, x, y, 0, 1, 1, 0));
                }
            }
        }
        std::unique_ptr<rcCompactHeightfield, decltype(&rcFreeCompactHeightfield)> compact(rcAllocCompactHeightfield(),
                                                                                           rcFreeCompactHeightfield);
        assert(rcBuildCompactHeightfield(&context, 3, 1, *heightfield, *compact));
        assert(rcBuildDistanceField(&context, *compact));
        assert(rcBuildRegions(&context, *compact, 0, 0, 0));
        detail::RecastContourOwner raw(rcAllocContourSet(), rcFreeContourSet);
        detail::RecastPolyMeshOwner coarse(rcAllocPolyMesh(), rcFreePolyMesh);
        assert(rcBuildContours(&context, *compact, 2, 0, *raw));
        assert(rcBuildPolyMesh(&context, *raw, 6, *coarse));
        assert(!detail::HasConsistentRegionMesh(*coarse));

        const std::vector<unsigned char> areas(compact->areas, compact->areas + compact->spanCount);
        rcConfig config{};
        config.maxSimplificationError = 2;
        config.maxVertsPerPoly = 6;
        std::vector<std::string> warnings;
        const auto mesh = detail::BuildRetainedRegionMesh(context, *compact, config,
                                                          RegionPartitioningAlgorithm::Watershed, warnings);
        assert(detail::HasConsistentRegionMesh(*mesh));
        assert(warnings.size() == 1 && warnings.front().contains("refined the complete contour partition"));
        assert(std::equal(areas.begin(), areas.end(), compact->areas));
    }

    void CheckCoarseMeshLayerRecovery()
    {
        rcContext context(false);
        auto compact = MakeEnclosedRegionField(context, true);
        std::vector<unsigned short> heights;
        std::vector<bool> retained;
        for (int span{}; span < compact->spanCount; ++span)
        {
            heights.push_back(compact->spans[span].y);
            retained.push_back(compact->spans[span].reg != 0 && compact->areas[span] != RC_NULL_AREA);
        }
        rcConfig config{};
        config.maxSimplificationError = 2;
        config.maxVertsPerPoly = 6;
        config.minRegionArea = compact->spanCount + 1;
        std::vector<std::string> warnings;
        const auto mesh = detail::BuildRetainedRegionMesh(context, *compact, config,
                                                          RegionPartitioningAlgorithm::Watershed, warnings);
        assert(detail::HasConsistentRegionMesh(*mesh));
        assert(std::any_of(warnings.begin(), warnings.end(),
                           [](const auto &warning) { return warning.contains("layer region recovery"); }));
        for (int span{}; span < compact->spanCount; ++span)
        {
            assert(compact->spans[span].y == heights[span]);
            assert((compact->areas[span] != RC_NULL_AREA) == retained[span]);
        }
    }
} // namespace

void TestRecastContourRecovery()
{
    CheckCoarsePolygonWinding();
    CheckCoarseMeshRefinement();
    CheckCoarseMeshLayerRecovery();
    for (const auto error : {0.0F, 2.0F, 6.0F})
    {
        CheckEnclosedRegionRecovery(error);
    }
    CheckOrdinaryContourRefinement();
    CheckEnclosedRegionRecovery(2.0F, true);
    for (const auto algorithm : {RegionPartitioningAlgorithm::Monotone, RegionPartitioningAlgorithm::Layers})
    {
        rcContext context(false);
        auto compact = MakeEnclosedRegionField(context);
        rcConfig config{};
        std::vector<std::string> warnings;
        bool rejected{};
        try
        {
            (void)detail::BuildRetainedRegionContours(context, *compact, config, algorithm, warnings);
        }
        catch (const std::runtime_error &error)
        {
            rejected = std::string_view(error.what()).contains("lost a retained walkable region");
        }
        assert(rejected && warnings.empty());
    }
}
