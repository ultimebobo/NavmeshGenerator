#include "core/navmesh/generator.h"

#include "core/navmesh/detail_triangulation.h"

#include <Recast.h>
#include <RecastAlloc.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <map>
#include <set>
#include <memory>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{
    using namespace navmesh::core;
    constexpr auto NoNeighbor = std::numeric_limits<std::uint32_t>::max();

    template <class T, void (*Free)(T *)> using RecastOwner = std::unique_ptr<T, decltype(Free)>;

    [[nodiscard]] float Cross(Vec3 a, Vec3 b, Vec3 c)
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    /// Degeneracy is three-dimensional: upright collision faces still obstruct movement.
    [[nodiscard]] bool HasTriangleArea(Vec3 a, Vec3 b, Vec3 c)
    {
        const auto ab = b - a;
        const auto ac = c - a;
        const Vec3 normal{ab.y * ac.z - ab.z * ac.y, ab.z * ac.x - ab.x * ac.z, ab.x * ac.y - ab.y * ac.x};
        return normal.x * normal.x + normal.y * normal.y + normal.z * normal.z > 0.00000001F;
    }

    [[nodiscard]] bool HeightAt(Vec3 point, Vec3 a, Vec3 b, Vec3 c, float &height)
    {
        const float area = Cross(a, b, c);
        if (std::abs(area) < 0.0001F)
        {
            return false;
        }
        const float u = Cross(point, b, c) / area, v = Cross(a, point, c) / area;
        if (u < -0.01F || v < -0.01F || u + v > 1.01F)
        {
            return false;
        }
        height = u * a.z + v * b.z + (1 - u - v) * c.z;
        return true;
    }

    [[nodiscard]] Vec3 ClosestPointOnTriangle(Vec3 point, Vec3 a, Vec3 b, Vec3 c)
    {
        float height{};
        if (HeightAt(point, a, b, c, height))
        {
            return {point.x, point.y, height};
        }
        Vec3 closest{};
        float best = std::numeric_limits<float>::max();
        for (const auto [start, end] : {std::pair{a, b}, std::pair{b, c}, std::pair{c, a}})
        {
            const float dx = end.x - start.x, dy = end.y - start.y;
            const float length = dx * dx + dy * dy;
            const float t = length > 0
                                ? std::clamp(((point.x - start.x) * dx + (point.y - start.y) * dy) / length, 0.0F, 1.0F)
                                : 0.0F;
            const Vec3 projected = start + (end - start) * t;
            const float distance = std::hypot(projected.x - point.x, projected.y - point.y);
            if (distance < best)
            {
                best = distance;
                closest = projected;
            }
        }
        return closest;
    }

    [[nodiscard]] std::vector<Vec3> ClipToCell(std::array<Vec3, 3> triangle, const AABB &cell)
    {
        std::vector<Vec3> polygon(triangle.begin(), triangle.end());
        const auto clip = [&](bool xAxis, float limit, bool keepGreater)
        {
            std::vector<Vec3> output;
            if (polygon.empty())
            {
                return output;
            }
            const auto coordinate = [&](Vec3 point) { return xAxis ? point.x : point.y; };
            auto previous = polygon.back();
            bool previousInside = keepGreater ? coordinate(previous) >= limit : coordinate(previous) <= limit;
            for (const auto current : polygon)
            {
                const bool currentInside = keepGreater ? coordinate(current) >= limit : coordinate(current) <= limit;
                if (previousInside != currentInside)
                {
                    const auto low = coordinate(previous) < coordinate(current) ? previous : current;
                    const auto high = coordinate(previous) < coordinate(current) ? current : previous;
                    const float t = (limit - coordinate(low)) / (coordinate(high) - coordinate(low));
                    auto crossing = low + (high - low) * t;
                    if (xAxis)
                    {
                        crossing.x = limit;
                    }
                    else
                    {
                        crossing.y = limit;
                    }
                    output.push_back(crossing);
                }
                if (currentInside)
                {
                    output.push_back(current);
                }
                previous = current;
                previousInside = currentInside;
            }
            return output;
        };
        polygon = clip(true, cell.min.x, true);
        polygon = clip(true, cell.max.x, false);
        polygon = clip(false, cell.min.y, true);
        polygon = clip(false, cell.max.y, false);
        return polygon;
    }

    /// A border anchor requires a complete boundary edge; a nearby prop or corner is insufficient.
    [[nodiscard]] bool IsBorderEdge(Vec3 a, Vec3 b, const AABB &bounds, float tolerance)
    {
        const auto onLine = [&](float first, float second, float limit)
        { return std::abs(first - limit) <= tolerance && std::abs(second - limit) <= tolerance; };
        return onLine(a.x, b.x, bounds.min.x) || onLine(a.x, b.x, bounds.max.x) || onLine(a.y, b.y, bounds.min.y) ||
               onLine(a.y, b.y, bounds.max.y);
    }

    using SourceGrid = std::map<std::pair<int, int>, std::vector<std::size_t>>;
    constexpr float SourceCellSize = 128.0F;
    [[nodiscard]] int SourceCell(float value)
    {
        return static_cast<int>(std::floor(value / SourceCellSize));
    }

    [[nodiscard]] std::size_t FindSource(const Scene &scene, const SourceGrid &grid,
                                         const std::vector<std::size_t> &largeSources, std::size_t fallback, Vec3 point)
    {
        // Recast voxels have no source-triangle identifier. Join each output
        // triangle back to the closest source surface at its XY centroid.
        float best = std::numeric_limits<float>::max();
        std::size_t selected = fallback;
        const auto candidates = grid.find({SourceCell(point.x), SourceCell(point.y)});
        const auto inspect = [&](std::size_t index)
        {
            const auto &tri = scene.mesh.triangles[index];
            const auto a = scene.mesh.vertices[tri.vertices[0]], b = scene.mesh.vertices[tri.vertices[1]],
                       c = scene.mesh.vertices[tri.vertices[2]];
            float height{};
            if (!HeightAt(point, a, b, c, height))
            {
                return;
            }
            const float distance = std::abs(height - point.z);
            if (distance < best)
            {
                best = distance;
                selected = index;
            }
        };
        if (candidates != grid.end())
        {
            for (const auto index : candidates->second)
            {
                inspect(index);
            }
        }
        for (const auto index : largeSources)
        {
            inspect(index);
        }
        return selected;
    }
    /// Recast Y-up buffers plus their source joins and Skyrim Z-up world bounds.
    struct RecastInput
    {
        std::vector<float> vertices;
        std::vector<int> triangles;
        std::vector<std::size_t> sources;
        /// One obstacle-only tag per raster triangle, parallel to sources after clipping.
        std::vector<bool> obstacles;
        AABB bounds;
    };

    /// XY buckets limit provenance searches; very large triangles are checked separately.
    struct SourceIndex
    {
        SourceGrid grid;
        std::vector<std::size_t> largeSources;
    };

    /// Filter supported world-space triangles, clip to raster bounds, and reverse winding for Recast Y-up.
    [[nodiscard]] RecastInput PrepareRecastInput(const Scene &scene, const std::optional<AABB> &cellBounds,
                                                 CandidateStatistics &statistics)
    {
        RecastInput input;
        auto &vertices = input.vertices;
        auto &triangles = input.triangles;
        auto &sources = input.sources;
        auto &bounds = input.bounds;
        for (std::size_t i{}; i < scene.mesh.triangles.size(); ++i)
        {
            const auto &provenance = scene.triangleProvenance[i];
            if (provenance.geometrySource >= scene.geometrySources.size())
            {
                throw std::invalid_argument("Invalid triangle geometry source");
            }
            if (scene.geometrySources[provenance.geometrySource].sourceType == GeometrySourceType::RenderFallback)
            {
                ++statistics.rejectedSource;
                continue;
            }
            const auto &tri = scene.mesh.triangles[i];
            if (std::any_of(tri.vertices.begin(), tri.vertices.end(),
                            [&](auto v) { return v >= scene.mesh.vertices.size(); }))
            {
                throw std::invalid_argument("Invalid source triangle vertex");
            }
            std::array<Vec3, 3> points{scene.mesh.vertices[tri.vertices[0]], scene.mesh.vertices[tri.vertices[1]],
                                       scene.mesh.vertices[tri.vertices[2]]};
            const float area = Cross(points[0], points[1], points[2]);
            if (!HasTriangleArea(points[0], points[1], points[2]))
            {
                ++statistics.rejectedDegenerate;
                continue;
            }
            if (area < 0)
            {
                std::swap(points[1], points[2]);
            }
            const auto clipped =
                cellBounds ? ClipToCell(points, *cellBounds) : std::vector<Vec3>(points.begin(), points.end());
            for (std::size_t corner = 1; corner + 1 < clipped.size(); ++corner)
            {
                const std::array face{clipped[0], clipped[corner], clipped[corner + 1]};
                if (!HasTriangleArea(face[0], face[1], face[2]))
                {
                    continue;
                }
                const auto base = static_cast<int>(vertices.size() / 3);
                // Recast is Y-up. Swapping Skyrim Y and Z changes handedness,
                // so the input winding is reversed to keep walkable normals up.
                for (const auto point : face)
                {
                    vertices.insert(vertices.end(), {point.x, point.z, point.y});
                    bounds.Expand(point);
                }
                triangles.insert(triangles.end(), {base, base + 2, base + 1});
                sources.push_back(i);
                input.obstacles.push_back(scene.geometrySources[provenance.geometrySource].navigationObstacle);
            }
        }
        statistics.eligibleTriangles = sources.size();
        return input;
    }

    /// Index eligible source triangles by world XY bounds for the approximate audit join.
    [[nodiscard]] SourceIndex BuildSourceIndex(const Scene &scene, const std::vector<std::size_t> &sources)
    {
        SourceIndex result;
        auto &sourceGrid = result.grid;
        auto &largeSources = result.largeSources;
        for (const auto index : sources)
        {
            if (scene.geometrySources[scene.triangleProvenance[index].geometrySource].navigationObstacle)
            {
                continue;
            }
            const auto &triangle = scene.mesh.triangles[index];
            const auto a = scene.mesh.vertices[triangle.vertices[0]], b = scene.mesh.vertices[triangle.vertices[1]],
                       c = scene.mesh.vertices[triangle.vertices[2]];
            const int minX = SourceCell(std::min({a.x, b.x, c.x})), maxX = SourceCell(std::max({a.x, b.x, c.x}));
            const int minY = SourceCell(std::min({a.y, b.y, c.y})), maxY = SourceCell(std::max({a.y, b.y, c.y}));
            if (static_cast<std::int64_t>(maxX - minX + 1) * (maxY - minY + 1) > 1024)
            {
                largeSources.push_back(index);
                continue;
            }
            for (int x = minX; x <= maxX; ++x)
            {
                for (int y = minY; y <= maxY; ++y)
                {
                    sourceGrid[{x, y}].push_back(index);
                }
            }
        }

        return result;
    }

    /// Convert physical profile constraints to voxel counts and pad Recast Y-up raster bounds.
    [[nodiscard]] rcConfig MakeRecastConfig(const AABB &bounds, const NavigationProfile &profile,
                                            const RecastSettings &settings)
    {
        // Resolve narrow stair treads in one cell and the default neighboring
        // cell ring while limiting wider areas to roughly 2048 columns per axis.
        const float width = bounds.max.x - bounds.min.x, depth = bounds.max.y - bounds.min.y;
        const float cs = std::max({settings.cellSize, width / 2048.0F, depth / 2048.0F});
        const float ch = settings.cellHeight;
        rcConfig config{};
        config.cs = cs;
        config.ch = ch;
        config.walkableSlopeAngle = profile.maxSlopeDegrees;
        config.walkableHeight = std::max(
            3, static_cast<int>(std::ceil(std::max(profile.agentHeight, profile.clearance) / static_cast<double>(ch))));
        config.walkableClimb = static_cast<int>(std::floor(profile.stepHeight / static_cast<double>(ch)));
        config.walkableRadius = static_cast<int>(std::ceil(profile.agentRadius / static_cast<double>(cs)));
        config.maxEdgeLen = static_cast<int>(settings.maxEdgeLength / static_cast<double>(cs));
        config.maxSimplificationError = settings.maxSimplificationError;
        const double voxelArea = static_cast<double>(cs) * cs;
        config.minRegionArea = static_cast<int>(std::ceil(profile.minimumRegionArea / voxelArea));
        // Scale the merge threshold with the profile's physical minimum area.
        config.mergeRegionArea =
            static_cast<int>(std::ceil(settings.mergeRegionAreaMultiplier * (profile.minimumRegionArea / voxelArea)));
        // Merge convex contour triangles before detail sampling. Larger patches
        // avoid forcing sample vertices along artificial diagonals on a straight flight.
        // The neutral/exported mesh is still triangulated by the detail builder.
        config.maxVertsPerPoly = 6;
        config.detailSampleDist = cs * 6;
        // Surface approximation is bounded by movement resolution, rather than
        // tracing voxel-height noise and individual stair risers. Raster filters
        // still enforce actual climb and headroom before any height simplification.
        config.detailSampleMaxError = std::max(ch, profile.stepHeight);
        config.bmin[0] = bounds.min.x - cs * 2;
        config.bmin[1] = bounds.min.z - ch * 2;
        config.bmin[2] = bounds.min.y - cs * 2;
        config.bmax[0] = bounds.max.x + cs * 2;
        config.bmax[1] = bounds.max.z + profile.agentHeight + ch * 2;
        config.bmax[2] = bounds.max.y + cs * 2;
        rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);
        return config;
    }

    /// Require a contour for every surviving voxel region; coarse simplification can collapse small islands.
    [[nodiscard]] bool HasAllRegionContours(const rcCompactHeightfield &compact, const rcContourSet &contours)
    {
        std::vector<bool> represented(static_cast<std::size_t>(compact.maxRegions) + 1);
        for (int contour{}; contour < contours.nconts; ++contour)
        {
            const auto &boundary = contours.conts[contour];
            if (boundary.nverts >= 3 && boundary.reg < represented.size())
            {
                represented[boundary.reg] = true;
            }
        }
        for (int span{}; span < compact.spanCount; ++span)
        {
            const auto region = compact.spans[span].reg;
            if (compact.areas[span] != RC_NULL_AREA && region && !(region & RC_BORDER_REG) && !represented[region])
            {
                return false;
            }
        }
        return true;
    }

    /// Refine contour tolerance until every retained voxel region survives, never exceeding the requested error.
    [[nodiscard]] RecastOwner<rcContourSet, rcFreeContourSet> BuildRegionContours(rcContext &context,
                                                                                  rcCompactHeightfield &compact,
                                                                                  const rcConfig &config)
    {
        auto error = config.maxSimplificationError;
        while (true)
        {
            RecastOwner<rcContourSet, rcFreeContourSet> refined(rcAllocContourSet(), rcFreeContourSet);
            if (!refined || !rcBuildContours(&context, compact, error, config.maxEdgeLen, *refined))
            {
                throw std::runtime_error("Recast contour construction failed");
            }
            // Every neighboring region must use the same simplification pass:
            // their shared interfaces form one partition of the compact spans.
            if (HasAllRegionContours(compact, *refined))
            {
                return refined;
            }
            if (error == 0)
            {
                throw std::runtime_error("Recast contour construction lost a retained walkable region");
            }
            error = error > 1 ? error * 0.5F : 0;
        }
    }

    /** Recast detail patches may share a boundary, but cannot reuse a directed
     * edge after winding normalization. Check exact sampled coordinates before
     * releasing the compact heightfield so an invalid triangulation can be rebuilt.
     */
    bool HasValidDetailEdges(const rcPolyMeshDetail &detail)
    {
        using Point = std::array<float, 3>;
        std::set<std::pair<Point, Point>> edges;
        for (int patchIndex{}; patchIndex < detail.nmeshes; ++patchIndex)
        {
            const auto *patch = detail.meshes + patchIndex * 4;
            for (unsigned int triangle{}; triangle < patch[3]; ++triangle)
            {
                const auto *indices = detail.tris + (patch[2] + triangle) * 4;
                std::array<Point, 3> points;
                for (std::size_t corner{}; corner < 3; ++corner)
                {
                    const auto *point = detail.verts + (patch[0] + indices[corner]) * 3;
                    points[corner] = {point[0], point[2], point[1]};
                }
                const auto area = (points[1][0] - points[0][0]) * (points[2][1] - points[0][1]) -
                                  (points[1][1] - points[0][1]) * (points[2][0] - points[0][0]);
                if (area == 0)
                {
                    return false;
                }
                if (area < 0)
                {
                    std::swap(points[1], points[2]);
                }
                for (std::size_t edge{}; edge < 3; ++edge)
                {
                    if (!edges.emplace(points[edge], points[(edge + 1) % 3]).second)
                    {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    /** Rebuild only overlapping detail patches by inserting their existing samples
     * into a constrained boundary triangulation. Original contour order and boundary
     * sample projection recover the hull; shared endpoints and floor heights remain unchanged.
     */
    void RepairDetailTriangles(rcPolyMeshDetail &detail, const rcPolyMesh &polygons)
    {
        std::vector<unsigned char> triangles;
        for (int patchIndex{}; patchIndex < detail.nmeshes; ++patchIndex)
        {
            auto *patch = detail.meshes + patchIndex * 4;
            std::vector<Vec3> points;
            for (unsigned int vertex{}; vertex < patch[1]; ++vertex)
            {
                const auto *point = detail.verts + (patch[0] + vertex) * 3;
                points.push_back({point[0], point[2], point[1]});
            }
            std::set<std::pair<unsigned int, unsigned int>> edges;
            std::set<std::uint32_t> boundarySamples;
            bool valid = true;
            bool hasFloorArea = false;
            for (unsigned int triangle{}; triangle < patch[3]; ++triangle)
            {
                const auto *face = detail.tris + (patch[2] + triangle) * 4;
                const auto area = Cross(points[face[0]], points[face[1]], points[face[2]]);
                valid = valid && area != 0;
                hasFloorArea = hasFloorArea || area != 0;
                for (unsigned int edge{}; edge < 3; ++edge)
                {
                    auto first = face[edge];
                    auto last = face[(edge + 1) % 3];
                    if (area < 0)
                    {
                        std::swap(first, last);
                    }
                    valid = edges.emplace(first, last).second && valid;
                    if ((face[3] >> (edge * 2)) & 1U)
                    {
                        boundarySamples.insert(first);
                        boundarySamples.insert(last);
                    }
                }
            }
            const auto oldOffset = patch[2];
            const auto oldCount = patch[3];
            patch[2] = static_cast<unsigned int>(triangles.size() / 4);
            if (valid)
            {
                triangles.insert(triangles.end(), detail.tris + oldOffset * 4,
                                 detail.tris + (oldOffset + oldCount) * 4);
                continue;
            }
            if (!hasFloorArea)
            {
                // A collapsed XY patch has no floor footprint to serialize.
                patch[3] = 0;
                continue;
            }
            // Detail vertices begin with the original convex polygon vertices.
            // Order edge samples by projection on that polygon, rather than
            // trusting an overlapping triangulation's boundary edge table.
            const auto *polygon = polygons.polys + patchIndex * polygons.nvp * 2;
            std::vector<std::uint32_t> corners;
            for (int vertex{}; vertex < polygons.nvp && polygon[vertex] != RC_MESH_NULL_IDX; ++vertex)
            {
                corners.push_back(static_cast<std::uint32_t>(vertex));
            }
            double area{};
            for (std::size_t index = 2; index < corners.size(); ++index)
            {
                area += Cross(points[corners[0]], points[corners[index - 1]], points[corners[index]]);
            }
            if (area < 0)
            {
                std::reverse(corners.begin(), corners.end());
            }
            std::map<std::size_t, std::vector<std::pair<double, std::uint32_t>>> samplesByEdge;
            for (const auto sample : boundarySamples)
            {
                if (sample < corners.size())
                {
                    continue;
                }
                auto nearest = std::numeric_limits<double>::max();
                std::size_t selected{};
                double parameter{};
                for (std::size_t edge{}; edge < corners.size(); ++edge)
                {
                    const auto a = points[corners[edge]];
                    const auto b = points[corners[(edge + 1) % corners.size()]];
                    const double dx = static_cast<double>(b.x) - a.x;
                    const double dy = static_cast<double>(b.y) - a.y;
                    const double px = static_cast<double>(points[sample].x) - a.x;
                    const double py = static_cast<double>(points[sample].y) - a.y;
                    const auto lengthSquared = dx * dx + dy * dy;
                    if (lengthSquared == 0)
                    {
                        continue;
                    }
                    const auto fraction = std::clamp((px * dx + py * dy) / lengthSquared, 0.0, 1.0);
                    const auto distance =
                        (px - fraction * dx) * (px - fraction * dx) + (py - fraction * dy) * (py - fraction * dy);
                    if (distance < nearest)
                    {
                        nearest = distance;
                        selected = edge;
                        parameter = fraction;
                    }
                }
                samplesByEdge[selected].emplace_back(parameter, sample);
            }
            std::vector<std::uint32_t> boundary;
            for (std::size_t edge{}; edge < corners.size(); ++edge)
            {
                boundary.push_back(corners[edge]);
                auto &samples = samplesByEdge[edge];
                std::sort(samples.begin(), samples.end());
                for (const auto &[parameter, sample] : samples)
                {
                    boundary.push_back(sample);
                }
            }
            std::map<std::uint32_t, std::uint32_t> hull;
            for (std::size_t edge{}; edge < boundary.size(); ++edge)
            {
                hull.emplace(boundary[edge], boundary[(edge + 1) % boundary.size()]);
            }
            const auto repaired = TriangulateDetailSamples(points, boundary);
            patch[3] = static_cast<unsigned int>(repaired.size());
            for (const auto &face : repaired)
            {
                unsigned char flags{};
                for (unsigned int edge{}; edge < 3; ++edge)
                {
                    const auto found = hull.find(face[edge]);
                    if (found != hull.end() && found->second == face[(edge + 1) % 3])
                    {
                        flags |= 1U << (edge * 2);
                    }
                }
                triangles.insert(triangles.end(),
                                 {static_cast<unsigned char>(face[0]), static_cast<unsigned char>(face[1]),
                                  static_cast<unsigned char>(face[2]), flags});
            }
        }
        auto *replacement = static_cast<unsigned char *>(rcAlloc(triangles.size(), RC_ALLOC_PERM));
        if (!replacement && !triangles.empty())
        {
            throw std::runtime_error("Recast repaired height detail allocation failed");
        }
        std::copy(triangles.begin(), triangles.end(), replacement);
        rcFree(detail.tris);
        detail.tris = replacement;
        detail.ntris = static_cast<int>(triangles.size() / 4);
    }

    /// Own all Recast intermediates; retain height samples in a detail mesh or throw a stage-specific error.
    [[nodiscard]] RecastOwner<rcPolyMeshDetail, rcFreePolyMeshDetail> BuildRecastDetailMesh(
        const RecastInput &input, const rcConfig &config, RegionPartitioningAlgorithm partitioningAlgorithm,
        CandidateStatistics &statistics, std::vector<std::string> &warnings)
    {
        const auto &vertices = input.vertices;
        const auto &triangles = input.triangles;
        const auto &sources = input.sources;
        rcContext context(false);
        RecastOwner<rcHeightfield, rcFreeHeightField> heightfield(rcAllocHeightfield(), rcFreeHeightField);
        if (!heightfield || !rcCreateHeightfield(&context, *heightfield, config.width, config.height, config.bmin,
                                                 config.bmax, config.cs, config.ch))
        {
            throw std::runtime_error("Recast heightfield creation failed");
        }
        std::vector<unsigned char> areas(sources.size(), RC_NULL_AREA);
        rcMarkWalkableTriangles(&context, config.walkableSlopeAngle, vertices.data(),
                                static_cast<int>(vertices.size() / 3), triangles.data(),
                                static_cast<int>(sources.size()), areas.data());
        // Reserve the highest raster area for obstacle-only solids. Recast merges
        // coincident spans using the highest area, so a rock cannot become a floor
        // through overlap with terrain or low-hanging-obstacle promotion.
        constexpr unsigned char floorArea = 1;
        constexpr unsigned char obstacleArea = RC_WALKABLE_AREA;
        for (std::size_t triangle{}; triangle < areas.size(); ++triangle)
        {
            if (input.obstacles[triangle])
            {
                areas[triangle] = obstacleArea;
                ++statistics.rejectedObstruction;
            }
            else if (areas[triangle] == RC_NULL_AREA)
            {
                ++statistics.rejectedSlope;
            }
            else
            {
                areas[triangle] = floorArea;
            }
        }
        if (!rcRasterizeTriangles(&context, vertices.data(), static_cast<int>(vertices.size() / 3), triangles.data(),
                                  areas.data(), static_cast<int>(sources.size()), *heightfield, config.walkableClimb))
        {
            throw std::runtime_error("Recast rasterization failed");
        }
        // Remove spans that violate climb or vertical clearance before radius erosion.
        rcFilterLowHangingWalkableObstacles(&context, config.walkableClimb, *heightfield);
        // Clear tagged solids after promotion, retaining their raw spans so
        // ledge and clearance filtering still see the obstacle volume.
        for (int column{}; column < heightfield->width * heightfield->height; ++column)
        {
            for (auto *span = heightfield->spans[column]; span; span = span->next)
            {
                if (span->area == obstacleArea)
                {
                    span->area = RC_NULL_AREA;
                }
            }
        }
        rcFilterLedgeSpans(&context, config.walkableHeight, config.walkableClimb, *heightfield);
        rcFilterWalkableLowHeightSpans(&context, config.walkableHeight, *heightfield);
        RecastOwner<rcCompactHeightfield, rcFreeCompactHeightfield> compact(rcAllocCompactHeightfield(),
                                                                            rcFreeCompactHeightfield);
        if (!compact ||
            !rcBuildCompactHeightfield(&context, config.walkableHeight, config.walkableClimb, *heightfield, *compact))
        {
            throw std::runtime_error("Recast compact heightfield failed");
        }
        // Compact spans own the walkability data; raw pools have no further consumers.
        heightfield.reset();
        areas.clear();
        areas.shrink_to_fit();
        if (!rcErodeWalkableArea(&context, config.walkableRadius, *compact))
        {
            throw std::runtime_error("Recast radius erosion failed");
        }
        // Partition only the surviving walkable spans, then trace and triangulate contours.
        if (partitioningAlgorithm == RegionPartitioningAlgorithm::Watershed &&
            !rcBuildDistanceField(&context, *compact))
        {
            throw std::runtime_error("Recast distance field failed");
        }
        const bool regionsBuilt =
            partitioningAlgorithm == RegionPartitioningAlgorithm::Watershed
                ? rcBuildRegions(&context, *compact, 0, config.minRegionArea, config.mergeRegionArea)
            : partitioningAlgorithm == RegionPartitioningAlgorithm::Monotone
                ? rcBuildRegionsMonotone(&context, *compact, 0, config.minRegionArea, config.mergeRegionArea)
                : rcBuildLayerRegions(&context, *compact, 0, config.minRegionArea);
        if (!regionsBuilt)
        {
            throw std::runtime_error("Recast region partition failed");
        }
        const auto contours = BuildRegionContours(context, *compact, config);
        RecastOwner<rcPolyMesh, rcFreePolyMesh> polyMesh(rcAllocPolyMesh(), rcFreePolyMesh);
        if (!polyMesh || !rcBuildPolyMesh(&context, *contours, config.maxVertsPerPoly, *polyMesh))
        {
            throw std::runtime_error("Recast polygon construction failed");
        }

        // Contour vertices describe horizontal boundaries, not the height changes
        // inside a polygon. Sample the surviving compact spans before releasing
        // them so stairs, landings and terrain follow their supporting layer.
        RecastOwner<rcPolyMeshDetail, rcFreePolyMeshDetail> detail(rcAllocPolyMeshDetail(), rcFreePolyMeshDetail);
        if (!detail || !rcBuildPolyMeshDetail(&context, *polyMesh, *compact, config.detailSampleDist,
                                              config.detailSampleMaxError, *detail))
        {
            throw std::runtime_error("Recast height detail construction failed");
        }
        if (!HasValidDetailEdges(*detail))
        {
            RepairDetailTriangles(*detail, *polyMesh);
            warnings.push_back(
                "Recast retriangulated overlapping height detail while retaining its boundary and floor samples.");
        }
        // Detail vertices are already Recast world coordinates. Recast adds one
        // vertical voxel to every detail vertex; remove that offset to retain
        // the contour mesh's rasterized floor-height convention.
        for (int vertex = 0; vertex < detail->nverts; ++vertex)
        {
            detail->verts[vertex * 3 + 1] -= config.ch;
        }
        return detail;
    }

    /// Keep referenced vertices in first-use order and rebase polygons, preserving the no-orphan invariant.
    void CompactMeshVertices(NavMesh &mesh)
    {
        std::vector<std::uint32_t> remap(mesh.vertices.size(), NoNeighbor);
        std::vector<Vec3> used;
        for (auto &face : mesh.polygons)
        {
            for (auto &index : face.vertices)
            {
                if (remap[index] == NoNeighbor)
                {
                    remap[index] = static_cast<std::uint32_t>(used.size());
                    used.push_back(mesh.vertices[index]);
                }
                index = remap[index];
            }
        }
        mesh.vertices = std::move(used);
    }

    /// Restore Skyrim Z-up detail triangles; weld shared patch boundaries before assigning adjacency.
    [[nodiscard]] NavMesh ConvertRecastMesh(const rcPolyMeshDetail &detail)
    {
        NavMesh mesh;
        std::map<std::array<float, 3>, std::uint32_t> vertices;
        const auto vertexIndex = [&](unsigned int index)
        {
            const auto *vertex = detail.verts + index * 3;
            const Vec3 point{vertex[0], vertex[2], vertex[1]};
            const auto [entry, added] = vertices.try_emplace(std::array{point.x, point.y, point.z},
                                                             static_cast<std::uint32_t>(mesh.vertices.size()));
            if (added)
            {
                mesh.vertices.push_back(point);
            }
            return entry->second;
        };
        for (int patchIndex = 0; patchIndex < detail.nmeshes; ++patchIndex)
        {
            const auto *patch = detail.meshes + patchIndex * 4;
            for (unsigned int triangle = 0; triangle < patch[3]; ++triangle)
            {
                // Triangle indices are local to this patch's contiguous vertex range.
                const auto *indices = detail.tris + (patch[2] + triangle) * 4;
                NavPolygon face;
                face.vertices = {vertexIndex(patch[0] + indices[0]), vertexIndex(patch[0] + indices[1]),
                                 vertexIndex(patch[0] + indices[2])};
                if (Cross(mesh.vertices[face.vertices[0]], mesh.vertices[face.vertices[1]],
                          mesh.vertices[face.vertices[2]]) < 0)
                {
                    std::swap(face.vertices[1], face.vertices[2]);
                }
                face.neighbors.fill(NoNeighbor);
                mesh.polygons.push_back(face);
            }
        }
        CompactMeshVertices(mesh);
        return mesh;
    }

    /** Clip generated world-space triangles after rasterization of the exterior halo.
     * Shared crossing vertices are welded by exact coordinates so border cuts retain
     * reciprocal adjacency. Exterior bounds constrain output, not walkability evidence.
     */
    void ClipGeneratedMesh(NavMesh &mesh, const AABB &bounds)
    {
        NavMesh clipped;
        std::map<std::array<float, 3>, std::uint32_t> vertices;
        const auto vertexIndex = [&](Vec3 point)
        {
            const auto [it, added] = vertices.try_emplace(std::array{point.x, point.y, point.z},
                                                          static_cast<std::uint32_t>(clipped.vertices.size()));
            if (added)
            {
                clipped.vertices.push_back(point);
            }
            return it->second;
        };
        for (const auto &face : mesh.polygons)
        {
            const auto polygon = ClipToCell(
                {mesh.vertices[face.vertices[0]], mesh.vertices[face.vertices[1]], mesh.vertices[face.vertices[2]]},
                bounds);
            for (std::size_t corner = 1; corner + 1 < polygon.size(); ++corner)
            {
                if (Cross(polygon[0], polygon[corner], polygon[corner + 1]) <= 0.0001F)
                {
                    continue;
                }
                NavPolygon triangle;
                triangle.vertices = {vertexIndex(polygon[0]), vertexIndex(polygon[corner]),
                                     vertexIndex(polygon[corner + 1])};
                triangle.neighbors.fill(NoNeighbor);
                clipped.polygons.push_back(triangle);
            }
        }
        mesh = std::move(clipped);
    }

    /// Link reciprocal neighbors only where exactly two triangles share an undirected vertex-index edge.
    void BuildMeshAdjacency(NavMesh &mesh)
    {
        using Edge = std::pair<std::uint32_t, std::uint32_t>;
        std::map<Edge, std::vector<std::pair<std::size_t, std::size_t>>> edges;
        for (std::size_t i{}; i < mesh.polygons.size(); ++i)
        {
            for (std::size_t side{}; side < 3; ++side)
            {
                const auto &face = mesh.polygons[i];
                edges[std::minmax(face.vertices[side], face.vertices[(side + 1) % 3])].push_back({i, side});
            }
        }
        for (const auto &[edge, uses] : edges)
        {
            if (uses.size() == 2)
            {
                const auto [a, as] = uses[0];
                const auto [b, bs] = uses[1];
                mesh.polygons[a].neighbors[as] = static_cast<std::uint32_t>(b);
                mesh.polygons[b].neighbors[bs] = static_cast<std::uint32_t>(a);
            }
        }
    }

    /// Assign the nearest source surface at each world XY centroid; fallback must be an eligible source index.
    void AssignSourceProvenance(CandidateNavMesh &result, const Scene &scene, const SourceIndex &sourceIndex,
                                std::size_t fallback)
    {
        // Recast output retains mesh geometry but not original triangle IDs.
        // The nearest supporting triangle supplies the audit join.
        for (const auto &face : result.mesh.polygons)
        {
            const auto point = (result.mesh.vertices[face.vertices[0]] + result.mesh.vertices[face.vertices[1]] +
                                result.mesh.vertices[face.vertices[2]]) /
                               3.0F;
            const auto source = FindSource(scene, sourceIndex.grid, sourceIndex.largeSources, fallback, point);
            result.polygonSourceTriangles.push_back(source);
            result.polygonContributingTriangles.push_back({source});
        }
    }

    /// Flood reciprocal adjacency into components with source evidence and world-space exterior-border anchors.
    void BuildCandidateRegions(CandidateNavMesh &result, const Scene &scene, const std::optional<AABB> &cellBounds,
                               float borderTolerance)
    {
        std::vector<bool> seen(result.mesh.polygons.size());
        for (std::size_t start{}; start < seen.size(); ++start)
        {
            if (!seen[start])
            {
                CandidateRegion region;
                region.id = static_cast<std::uint32_t>(result.regions.size());
                std::queue<std::size_t> pending;
                pending.push(start);
                seen[start] = true;
                while (!pending.empty())
                {
                    const auto index = pending.front();
                    pending.pop();
                    region.polygons.push_back(static_cast<std::uint32_t>(index));
                    const auto &face = result.mesh.polygons[index];
                    const auto a = result.mesh.vertices[face.vertices[0]], b = result.mesh.vertices[face.vertices[1]],
                               c = result.mesh.vertices[face.vertices[2]];
                    region.area += std::abs(Cross(a, b, c)) * 0.5F;
                    region.sourceTriangles.push_back(result.polygonSourceTriangles[index]);
                    region.geometrySources.push_back(
                        scene.triangleProvenance[result.polygonSourceTriangles[index]].geometrySource);
                    for (auto neighbor : face.neighbors)
                    {
                        if (neighbor != NoNeighbor && !seen[neighbor])
                        {
                            seen[neighbor] = true;
                            pending.push(neighbor);
                        }
                    }
                    for (std::size_t edge{}; cellBounds && edge < face.vertices.size(); ++edge)
                    {
                        if (face.neighbors[edge] == NoNeighbor &&
                            IsBorderEdge(result.mesh.vertices[face.vertices[edge]],
                                         result.mesh.vertices[face.vertices[(edge + 1) % 3]], *cellBounds,
                                         borderTolerance))
                        {
                            region.reachesBorder = true;
                        }
                    }
                }
                std::sort(region.sourceTriangles.begin(), region.sourceTriangles.end());
                region.sourceTriangles.erase(std::unique(region.sourceTriangles.begin(), region.sourceTriangles.end()),
                                             region.sourceTriangles.end());
                std::sort(region.geometrySources.begin(), region.geometrySources.end());
                region.geometrySources.erase(std::unique(region.geometrySources.begin(), region.geometrySources.end()),
                                             region.geometrySources.end());
                result.regions.push_back(std::move(region));
            }
        }
    }

    /// Attach each world-space door to the nearest component within horizontal and vertical reach.
    void MatchExits(CandidateNavMesh &result, float horizontalTolerance, float verticalTolerance)
    {
        for (auto &door : result.exits)
        {
            float best = std::numeric_limits<float>::max();
            for (const auto &region : result.regions)
            {
                for (auto index : region.polygons)
                {
                    const auto &face = result.mesh.polygons[index];
                    const auto point = ClosestPointOnTriangle(door.position, result.mesh.vertices[face.vertices[0]],
                                                              result.mesh.vertices[face.vertices[1]],
                                                              result.mesh.vertices[face.vertices[2]]);
                    const float dx = point.x - door.position.x, dy = point.y - door.position.y,
                                dz = point.z - door.position.z;
                    const float distance = std::hypot(dx, dy);
                    if (distance < best && distance <= horizontalTolerance && std::abs(dz) <= verticalTolerance)
                    {
                        best = distance;
                        door.region = region.id;
                        door.polygon = index;
                    }
                }
            }
            if (door.region)
            {
                result.regions[*door.region].exitFormIds.push_back(door.referenceId);
            }
        }
    }

    /** Retain whole shared-edge components anchored to a border or matched door.
     * Stable polygon compaction preserves processing order and source provenance.
     * Every polygon, neighbor, region and door index is rebased before unused
     * vertices are removed. An unanchored interior produces an empty candidate.
     */
    void RetainReachableRegions(CandidateNavMesh &result)
    {
        std::vector<std::uint32_t> polygonRemap(result.mesh.polygons.size(), NoNeighbor);
        std::vector<std::uint32_t> regionRemap(result.regions.size(), NoNeighbor);
        std::uint32_t regionCount{};
        for (const auto &region : result.regions)
        {
            if (region.reachesBorder || !region.exitFormIds.empty())
            {
                regionRemap[region.id] = regionCount++;
                for (const auto polygon : region.polygons)
                {
                    polygonRemap[polygon] = 0;
                }
            }
        }
        std::uint32_t polygonCount{};
        for (auto &polygon : polygonRemap)
        {
            if (polygon != NoNeighbor)
            {
                polygon = polygonCount++;
            }
        }
        const auto removed = result.mesh.polygons.size() - polygonCount;
        if (!removed)
        {
            return;
        }

        // Components are removed together, so no retained adjacency crosses into
        // a discarded component. Source joins follow the same stable remapping.
        std::vector<NavPolygon> polygons;
        std::vector<std::size_t> sources;
        std::vector<std::vector<std::size_t>> contributions;
        for (std::size_t index{}; index < polygonRemap.size(); ++index)
        {
            if (polygonRemap[index] == NoNeighbor)
            {
                continue;
            }
            auto polygon = result.mesh.polygons[index];
            for (auto &neighbor : polygon.neighbors)
            {
                if (neighbor != NoNeighbor)
                {
                    neighbor = polygonRemap[neighbor];
                }
            }
            polygons.push_back(polygon);
            sources.push_back(result.polygonSourceTriangles[index]);
            contributions.push_back(std::move(result.polygonContributingTriangles[index]));
        }
        result.mesh.polygons = std::move(polygons);
        result.polygonSourceTriangles = std::move(sources);
        result.polygonContributingTriangles = std::move(contributions);
        std::erase_if(result.regions, [&](const auto &region) { return regionRemap[region.id] == NoNeighbor; });
        for (auto &region : result.regions)
        {
            region.id = regionRemap[region.id];
            for (auto &polygon : region.polygons)
            {
                polygon = polygonRemap[polygon];
            }
        }
        for (auto &door : result.exits)
        {
            if (door.polygon && door.region)
            {
                door.polygon = polygonRemap[*door.polygon];
                door.region = regionRemap[*door.region];
            }
        }
        CompactMeshVertices(result.mesh);
        result.statistics.rejectedUnreachable += removed;
    }

} // namespace

namespace navmesh::core
{
    void ValidateRecastSettings(const NavigationProfile &profile, const RecastSettings &settings)
    {
        const auto nonnegative = [](float value) { return std::isfinite(value) && value >= 0; };
        const auto positive = [&](float value) { return nonnegative(value) && value > 0; };
        if (!nonnegative(profile.agentRadius) || !positive(profile.agentHeight) || !positive(profile.clearance) ||
            !nonnegative(profile.stepHeight) || !nonnegative(profile.maxSlopeDegrees) ||
            profile.maxSlopeDegrees >= 90 || !nonnegative(profile.minimumRegionArea) ||
            !positive(profile.weldTolerance) || !positive(settings.cellSize) || !positive(settings.cellHeight) ||
            !nonnegative(settings.maxSimplificationError) || !nonnegative(settings.maxEdgeLength) ||
            !nonnegative(settings.mergeRegionAreaMultiplier))
        {
            throw std::invalid_argument("Invalid Recast settings: use finite positive voxel sizes, height, clearance "
                                        "and weld tolerance; nonnegative radius, climb, area and contour controls; "
                                        "and a slope below 90 degrees.");
        }
        // Check conversions at the finest requested resolution. Adaptive horizontal sizing
        // only reduces these counts; vertical spans must also fit Recast's packed span height.
        const double cellSize = settings.cellSize;
        const double area = profile.minimumRegionArea / (cellSize * cellSize);
        const double integerLimit = std::numeric_limits<int>::max() - 1.0;
        if (std::ceil(area) > integerLimit || std::ceil(area * settings.mergeRegionAreaMultiplier) > integerLimit ||
            std::ceil(profile.agentRadius / cellSize) > integerLimit ||
            settings.maxEdgeLength / cellSize > integerLimit ||
            std::ceil(std::max(profile.agentHeight, profile.clearance) / static_cast<double>(settings.cellHeight)) >
                RC_SPAN_MAX_HEIGHT ||
            std::floor(profile.stepHeight / static_cast<double>(settings.cellHeight)) > RC_SPAN_MAX_HEIGHT)
        {
            throw std::invalid_argument("Recast settings exceed voxel count or vertical span limits.");
        }
    }

    CandidateNavMesh RecastCandidateGenerator::Generate(const Scene &scene, const NavigationProfile &profile,
                                                        std::optional<AABB> cellBounds,
                                                        std::vector<CandidateExit> exits,
                                                        RegionPartitioningAlgorithm partitioningAlgorithm,
                                                        const RecastSettings &settings,
                                                        CandidateRetention retention) const
    {
        if (!scene.HasCompleteTriangleProvenance())
        {
            throw std::invalid_argument("Recast requires complete triangle provenance");
        }
        ValidateRecastSettings(profile, settings);
        CandidateNavMesh result;
        result.profile = profile;
        result.recastSettings = settings;
        result.exits = std::move(exits);
        for (auto &door : result.exits)
        {
            door.region.reset();
            door.polygon.reset();
        }
        switch (partitioningAlgorithm)
        {
        case RegionPartitioningAlgorithm::Watershed:
            result.partitioningAlgorithm = "watershed";
            break;
        case RegionPartitioningAlgorithm::Monotone:
            result.partitioningAlgorithm = "monotone";
            break;
        case RegionPartitioningAlgorithm::Layers:
            result.partitioningAlgorithm = "layers";
            break;
        default:
            throw std::invalid_argument("Invalid Recast region partitioning algorithm");
        }
        result.statistics.inputTriangles = scene.mesh.triangles.size();

        // Prepare input evidence before allocating Recast resources.
        // Rasterize a supported halo so radius erosion does not treat the CELL seam
        // as a cliff. Only the final mesh is clipped to the selected CELL.
        auto rasterBounds = cellBounds;
        if (rasterBounds)
        {
            const auto width = rasterBounds->max.x - rasterBounds->min.x;
            const auto depth = rasterBounds->max.y - rasterBounds->min.y;
            const auto voxelSize = std::max({settings.cellSize, width / 2048.0F, depth / 2048.0F});
            const auto halo = (std::ceil(profile.agentRadius / voxelSize) + 3.0F) * voxelSize;
            rasterBounds->min.x -= halo;
            rasterBounds->min.y -= halo;
            rasterBounds->max.x += halo;
            rasterBounds->max.y += halo;
        }
        const auto input = PrepareRecastInput(scene, rasterBounds, result.statistics);
        if (input.sources.empty())
        {
            result.warnings.push_back("No supported terrain or collision triangles were available for Recast.");
            return result;
        }
        const auto floorSource = std::find(input.obstacles.begin(), input.obstacles.end(), false);
        if (floorSource == input.obstacles.end())
        {
            result.statistics.rejectedObstruction = input.sources.size();
            result.warnings.push_back("Only obstacle collision was available; no walkable floor sources remain.");
            return result;
        }
        const auto fallbackSource = input.sources[static_cast<std::size_t>(floorSource - input.obstacles.begin())];
        const auto sourceIndex = BuildSourceIndex(scene, input.sources);
        const auto config = MakeRecastConfig(input.bounds, profile, settings);
        if (config.cs > settings.cellSize)
        {
            result.warnings.push_back(std::format(
                "Recast increased horizontal voxel size to {} Skyrim units (requested {}) for the raster extent; "
                "narrow stair treads may need a smaller generation area.",
                config.cs, settings.cellSize));
        }
        const auto effectiveClimb = config.walkableClimb * config.ch;
        if (effectiveClimb < profile.stepHeight)
        {
            result.warnings.push_back(
                std::format("Recast rounds step height down to {} Skyrim units (requested {}; vertical voxel size {}). "
                            "Use a finer vertical voxel size to represent small steps.",
                            effectiveClimb, profile.stepHeight, config.ch));
        }

        // Voxelize, partition, and restore a neutral mesh before attaching evidence.
        const auto detailMesh =
            BuildRecastDetailMesh(input, config, partitioningAlgorithm, result.statistics, result.warnings);
        result.mesh = ConvertRecastMesh(*detailMesh);
        if (cellBounds)
        {
            ClipGeneratedMesh(result.mesh, *cellBounds);
        }
        BuildMeshAdjacency(result.mesh);
        AssignSourceProvenance(result, scene, sourceIndex, fallbackSource);

        // Anchor connectivity after CELL clipping: paths outside the selected
        // bounds cannot keep a disconnected interior component alive.
        BuildCandidateRegions(result, scene, cellBounds, profile.weldTolerance);
        MatchExits(result, profile.agentRadius * 4 + config.cs * 2, profile.stepHeight + config.ch * 2);
        if (retention == CandidateRetention::Anchored)
        {
            RetainReachableRegions(result);
        }
        result.statistics.polygonsBeforeSimplification = result.mesh.polygons.size();
        result.statistics.outputPolygons = result.mesh.polygons.size();
        result.warnings.push_back("Recast source-triangle provenance is matched by nearest surface after voxelization; "
                                  "generated geometry remains inspection-only.");
        result.warnings.push_back("Profile weld tolerance, contour tolerance, and cell-border policy are legacy "
                                  "polygon-generator settings and do not control Recast voxelization.");
        if (result.mesh.polygons.empty())
        {
            result.warnings.push_back("No walkable polygons remain after Recast generation.");
        }
        result.topology = ValidateCandidateTopology(result);
        return result;
    }
} // namespace navmesh::core
