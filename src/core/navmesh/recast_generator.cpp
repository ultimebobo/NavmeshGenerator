#include "core/navmesh/generator.h"

#include <Recast.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
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

    [[nodiscard]] bool TouchesBorder(Vec3 a, Vec3 b, Vec3 c, const AABB &bounds, float tolerance)
    {
        // Intersect the triangle with a narrow strip at each outer cell edge.
        const auto intersectsStrip = [&](float left, float bottom, float right, float top)
        {
            const std::array<Vec3, 3> points{a, b, c};
            const float centerX = (left + right) * 0.5F, centerY = (bottom + top) * 0.5F;
            const float halfX = (right - left) * 0.5F, halfY = (top - bottom) * 0.5F;
            for (const auto [nx, ny] : {std::pair{1.0F, 0.0F}, std::pair{0.0F, 1.0F}})
            {
                const auto [low, high] = std::minmax({nx * a.x + ny * a.y, nx * b.x + ny * b.y, nx * c.x + ny * c.y});
                const float center = nx * centerX + ny * centerY;
                const float radius = std::abs(nx) * halfX + std::abs(ny) * halfY;
                if (high < center - radius || low > center + radius)
                {
                    return false;
                }
            }
            for (std::size_t side{}; side < 3; ++side)
            {
                const auto start = points[side], end = points[(side + 1) % 3];
                const float nx = start.y - end.y, ny = end.x - start.x;
                const auto [low, high] = std::minmax({nx * a.x + ny * a.y, nx * b.x + ny * b.y, nx * c.x + ny * c.y});
                const float center = nx * centerX + ny * centerY;
                const float radius = std::abs(nx) * halfX + std::abs(ny) * halfY;
                if (high < center - radius || low > center + radius)
                {
                    return false;
                }
            }
            return true;
        };
        return intersectsStrip(bounds.min.x - tolerance, bounds.min.y - tolerance, bounds.min.x + tolerance,
                               bounds.max.y + tolerance) ||
               intersectsStrip(bounds.max.x - tolerance, bounds.min.y - tolerance, bounds.max.x + tolerance,
                               bounds.max.y + tolerance) ||
               intersectsStrip(bounds.min.x - tolerance, bounds.min.y - tolerance, bounds.max.x + tolerance,
                               bounds.min.y + tolerance) ||
               intersectsStrip(bounds.min.x - tolerance, bounds.max.y - tolerance, bounds.max.x + tolerance,
                               bounds.max.y + tolerance);
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
            if (std::abs(area) < 0.0001F)
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
                if (Cross(face[0], face[1], face[2]) <= 0.0001F)
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
    [[nodiscard]] rcConfig MakeRecastConfig(const AABB &bounds, const NavigationProfile &profile)
    {
        // Resolve narrow stair treads in one cell and the default neighboring
        // cell ring while limiting wider areas to roughly 2048 columns per axis.
        const float width = bounds.max.x - bounds.min.x, depth = bounds.max.y - bounds.min.y;
        const float cs = std::max({4.0F, width / 2048.0F, depth / 2048.0F});
        const float ch = 2.0F;
        rcConfig config{};
        config.cs = cs;
        config.ch = ch;
        config.walkableSlopeAngle = profile.maxSlopeDegrees;
        config.walkableHeight =
            std::max(3, static_cast<int>(std::ceil(std::max(profile.agentHeight, profile.clearance) / ch)));
        config.walkableClimb = static_cast<int>(std::floor(profile.stepHeight / ch));
        config.walkableRadius = static_cast<int>(std::ceil(profile.agentRadius / cs));
        config.maxEdgeLen = 0;
        config.maxSimplificationError = 2.0F;
        config.minRegionArea = static_cast<int>(std::ceil(profile.minimumRegionArea / (cs * cs)));
        // Scale the merge threshold with the profile's physical minimum area.
        config.mergeRegionArea = static_cast<int>(std::ceil(4.0F * profile.minimumRegionArea / (cs * cs)));
        config.maxVertsPerPoly = 3;
        config.bmin[0] = bounds.min.x - cs * 2;
        config.bmin[1] = bounds.min.z - ch * 2;
        config.bmin[2] = bounds.min.y - cs * 2;
        config.bmax[0] = bounds.max.x + cs * 2;
        config.bmax[1] = bounds.max.z + profile.agentHeight + ch * 2;
        config.bmax[2] = bounds.max.y + cs * 2;
        rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);
        return config;
    }

    /// Own all Recast intermediates; return the polygon mesh or throw a stage-specific error.
    [[nodiscard]] RecastOwner<rcPolyMesh, rcFreePolyMesh> BuildRecastPolyMesh(
        const RecastInput &input, const rcConfig &config, RegionPartitioningAlgorithm partitioningAlgorithm,
        CandidateStatistics &statistics)
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
        for (const auto area : areas)
        {
            if (area == RC_NULL_AREA)
            {
                ++statistics.rejectedSlope;
            }
        }
        if (!rcRasterizeTriangles(&context, vertices.data(), static_cast<int>(vertices.size() / 3), triangles.data(),
                                  areas.data(), static_cast<int>(sources.size()), *heightfield, config.walkableClimb))
        {
            throw std::runtime_error("Recast rasterization failed");
        }
        // Remove spans that violate climb or vertical clearance before radius erosion.
        rcFilterLowHangingWalkableObstacles(&context, config.walkableClimb, *heightfield);
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
        RecastOwner<rcContourSet, rcFreeContourSet> contours(rcAllocContourSet(), rcFreeContourSet);
        if (!contours ||
            !rcBuildContours(&context, *compact, config.maxSimplificationError, config.maxEdgeLen, *contours))
        {
            throw std::runtime_error("Recast contour construction failed");
        }
        compact.reset();
        RecastOwner<rcPolyMesh, rcFreePolyMesh> polyMesh(rcAllocPolyMesh(), rcFreePolyMesh);
        if (!polyMesh || !rcBuildPolyMesh(&context, *contours, config.maxVertsPerPoly, *polyMesh))
        {
            throw std::runtime_error("Recast polygon construction failed");
        }

        return polyMesh;
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

    /// Restore Skyrim Z-up vertices and counterclockwise triangles with no adjacency assigned yet.
    [[nodiscard]] NavMesh ConvertRecastMesh(const rcPolyMesh &polyMesh)
    {
        NavMesh mesh;
        for (int i = 0; i < polyMesh.nverts; ++i)
        {
            const auto *v = polyMesh.verts + 3 * i;
            mesh.vertices.push_back({polyMesh.bmin[0] + v[0] * polyMesh.cs, polyMesh.bmin[2] + v[2] * polyMesh.cs,
                                     polyMesh.bmin[1] + v[1] * polyMesh.ch});
        }
        for (int i = 0; i < polyMesh.npolys; ++i)
        {
            const auto *poly = polyMesh.polys + i * polyMesh.nvp * 2;
            for (int j = 2; j < polyMesh.nvp && poly[j] != RC_MESH_NULL_IDX; ++j)
            {
                NavPolygon face;
                face.vertices = {poly[0], poly[j - 1], poly[j]};
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
                    if (cellBounds && TouchesBorder(a, b, c, *cellBounds, borderTolerance))
                    {
                        region.reachesBorder = true;
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

} // namespace

namespace navmesh::core
{
    CandidateNavMesh RecastCandidateGenerator::Generate(const Scene &scene, const NavigationProfile &profile,
                                                        std::optional<AABB> cellBounds,
                                                        std::vector<CandidateExit> exits,
                                                        RegionPartitioningAlgorithm partitioningAlgorithm) const
    {
        if (!scene.HasCompleteTriangleProvenance())
        {
            throw std::invalid_argument("Recast requires complete triangle provenance");
        }
        if (profile.agentRadius < 0 || profile.agentHeight <= 0 || profile.stepHeight < 0 ||
            profile.maxSlopeDegrees < 0 || profile.maxSlopeDegrees >= 90)
        {
            throw std::invalid_argument("Invalid navigation settings for Recast");
        }
        CandidateNavMesh result;
        result.profile = profile;
        result.exits = std::move(exits);
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
            const auto voxelSize = std::max({4.0F, width / 2048.0F, depth / 2048.0F});
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
        const auto sourceIndex = BuildSourceIndex(scene, input.sources);
        const auto config = MakeRecastConfig(input.bounds, profile);

        // Voxelize, partition, and restore a neutral mesh before attaching evidence.
        const auto polyMesh = BuildRecastPolyMesh(input, config, partitioningAlgorithm, result.statistics);
        result.mesh = ConvertRecastMesh(*polyMesh);
        if (cellBounds)
        {
            ClipGeneratedMesh(result.mesh, *cellBounds);
        }
        BuildMeshAdjacency(result.mesh);
        AssignSourceProvenance(result, scene, sourceIndex, input.sources.front());

        // Record connectivity and anchors without discarding walkable components.
        BuildCandidateRegions(result, scene, cellBounds, profile.agentRadius + config.cs * 2);
        MatchExits(result, profile.agentRadius * 4 + config.cs * 2, profile.stepHeight + config.ch * 2);
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
