#include "core/navmesh/candidate.h"

#include "analysis/navmesh_analysis.h"
#include "core/reproducibility/export_metadata.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <stdexcept>
#include <tuple>

namespace
{
    using namespace navmesh::core;
    constexpr auto NoNeighbor = std::numeric_limits<std::uint32_t>::max();
    /// Angular allowance in degrees for float roundoff when deforming a world-space fan.
    constexpr float BorderSlopeRoundoff = 0.001F;
    using Key = std::tuple<std::int64_t, std::int64_t, std::int64_t>;
    using Key2 = std::pair<std::int64_t, std::int64_t>;
    using Edge = std::pair<Key2, Key2>;

    [[nodiscard]] float Cross2(Vec3 a, Vec3 b, Vec3 c)
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }
    [[nodiscard]] float Area2(Vec3 a, Vec3 b, Vec3 c)
    {
        return std::abs(Cross2(a, b, c));
    }
    [[nodiscard]] Key Quantize(Vec3 p, float tolerance)
    {
        return {std::llround(static_cast<double>(p.x) / tolerance), std::llround(static_cast<double>(p.y) / tolerance),
                std::llround(static_cast<double>(p.z) / tolerance)};
    }
    [[nodiscard]] Key2 Quantize2(Vec3 p, float tolerance)
    {
        return {std::llround(static_cast<double>(p.x) / tolerance), std::llround(static_cast<double>(p.y) / tolerance)};
    }
    [[nodiscard]] Edge SortedEdge(Vec3 a, Vec3 b, float tolerance)
    {
        const auto first = Quantize2(a, tolerance), second = Quantize2(b, tolerance);
        return first < second ? Edge{first, second} : Edge{second, first};
    }
    [[nodiscard]] bool StepCompatible(Vec3 a, Vec3 b, Vec3 c, Vec3 d, float tolerance, float step)
    {
        // Traversable neighboring faces must use their shared XY edge in
        // opposite directions. Same-direction uses are overlapping surfaces,
        // not a portal between triangles.
        const auto first = Quantize2(a, tolerance), last = Quantize2(b, tolerance);
        return first != last && first == Quantize2(d, tolerance) && last == Quantize2(c, tolerance) &&
               std::abs(a.z - d.z) <= step + tolerance && std::abs(b.z - c.z) <= step + tolerance;
    }
    [[nodiscard]] Vec3 Interpolate(Vec3 a, Vec3 b, float t)
    {
        return a + (b - a) * t;
    }
    [[nodiscard]] bool HeightAt(Vec3 p, Vec3 a, Vec3 b, Vec3 c, float &height)
    {
        const auto denominator = Cross2(a, b, c);
        if (std::abs(denominator) < 1.0e-5F)
        {
            return false;
        }
        const auto u = Cross2(p, b, c) / denominator;
        const auto v = Cross2(a, p, c) / denominator;
        const auto w = 1.0F - u - v;
        if (u < -1.0e-4F || v < -1.0e-4F || w < -1.0e-4F)
        {
            return false;
        }
        height = u * a.z + v * b.z + w * c.z;
        return true;
    }
    [[nodiscard]] bool StrictlyInside(Vec3 p, const std::array<Vec3, 3> &triangle)
    {
        const auto d = Cross2(triangle[0], triangle[1], triangle[2]);
        if (d <= 1.0e-5F)
        {
            return false;
        }
        const auto u = Cross2(p, triangle[1], triangle[2]) / d;
        const auto v = Cross2(triangle[0], p, triangle[2]) / d;
        return u > 1.0e-3F && v > 1.0e-3F && u + v < 1.0F - 1.0e-3F;
    }
    [[nodiscard]] bool SegmentThroughInterior(Vec3 a, Vec3 b, const std::array<Vec3, 3> &triangle)
    {
        // A short post or railing inset from one edge is a local obstacle,
        // not a reason to discard an entire broad floor triangle. Reject
        // only a barrier that traverses the floor from side to side in the
        // agent's height band.
        std::vector<std::pair<float, std::size_t>> crossings;
        for (std::size_t side{}; side < 3; ++side)
        {
            const auto c = triangle[side], d = triangle[(side + 1) % 3];
            const auto denominator = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
            if (std::abs(denominator) < 1.0e-6F)
            {
                continue;
            }
            const auto t = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / denominator;
            const auto u = ((c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x)) / denominator;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
            {
                crossings.push_back({t, side});
            }
        }
        std::sort(crossings.begin(), crossings.end());
        for (std::size_t i{}; i < crossings.size(); ++i)
        {
            for (std::size_t j = i + 1; j < crossings.size(); ++j)
            {
                if (crossings[i].second == crossings[j].second || crossings[j].first - crossings[i].first <= 1.0e-4F)
                {
                    continue;
                }
                const auto point = Interpolate(a, b, (crossings[i].first + crossings[j].first) * 0.5F);
                if (StrictlyInside(point, triangle))
                {
                    return true;
                }
            }
        }
        return false;
    }
    [[nodiscard]] std::vector<Vec3> ClipInsideEdge(const std::vector<Vec3> &polygon, Vec3 a, Vec3 b, float inset)
    {
        std::vector<Vec3> result;
        if (polygon.empty())
        {
            return result;
        }
        const auto length = std::hypot(b.x - a.x, b.y - a.y);
        if (length < 1.0e-5F)
        {
            return result;
        }
        const auto signedDistance = [&](Vec3 p) { return Cross2(a, b, p) / length - inset; };
        for (std::size_t i{}; i < polygon.size(); ++i)
        {
            const auto previous = polygon[(i + polygon.size() - 1) % polygon.size()];
            const auto current = polygon[i];
            const auto before = signedDistance(previous), after = signedDistance(current);
            if ((before >= 0) != (after >= 0))
            {
                result.push_back(Interpolate(previous, current, before / (before - after)));
            }
            if (after >= 0)
            {
                result.push_back(current);
            }
        }
        return result;
    }
    [[nodiscard]] bool OnCellBorder(Vec3 a, Vec3 b, const AABB &bounds, float tolerance)
    {
        return (std::abs(a.x - bounds.min.x) <= tolerance && std::abs(b.x - bounds.min.x) <= tolerance) ||
               (std::abs(a.x - bounds.max.x) <= tolerance && std::abs(b.x - bounds.max.x) <= tolerance) ||
               (std::abs(a.y - bounds.min.y) <= tolerance && std::abs(b.y - bounds.min.y) <= tolerance) ||
               (std::abs(a.y - bounds.max.y) <= tolerance && std::abs(b.y - bounds.max.y) <= tolerance);
    }
    [[nodiscard]] bool TouchesCellBorder(Vec3 a, Vec3 b, const AABB &bounds, float tolerance)
    {
        // A source triangle can cross a cell edge without having either end
        // exactly on it. Count that crossing as a reachable border as well.
        const auto crossesX = [&](float x)
        {
            if (std::min(a.x, b.x) > x + tolerance || std::max(a.x, b.x) < x - tolerance)
            {
                return false;
            }
            const auto t = std::abs(b.x - a.x) <= tolerance ? 0.0F : std::clamp((x - a.x) / (b.x - a.x), 0.0F, 1.0F);
            const auto y = a.y + (b.y - a.y) * t;
            return y >= bounds.min.y - tolerance && y <= bounds.max.y + tolerance;
        };
        const auto crossesY = [&](float y)
        {
            if (std::min(a.y, b.y) > y + tolerance || std::max(a.y, b.y) < y - tolerance)
            {
                return false;
            }
            const auto t = std::abs(b.y - a.y) <= tolerance ? 0.0F : std::clamp((y - a.y) / (b.y - a.y), 0.0F, 1.0F);
            const auto x = a.x + (b.x - a.x) * t;
            return x >= bounds.min.x - tolerance && x <= bounds.max.x + tolerance;
        };
        for (float x = bounds.min.x; x <= bounds.max.x + 0.5F; x += 4096.0F)
        {
            if (crossesX(x))
            {
                return true;
            }
        }
        for (float y = bounds.min.y; y <= bounds.max.y + 0.5F; y += 4096.0F)
        {
            if (crossesY(y))
            {
                return true;
            }
        }
        return crossesX(bounds.max.x) || crossesY(bounds.max.y);
    }
    /** Find the closest floor triangle to a door marker. The XY projection may
     * lie just outside the polygon because door references sit in a threshold.
     * Height is interpolated on the closest point, keeping stacked floors apart.
     */
    [[nodiscard]] std::optional<std::uint32_t> ExitPolygon(const NavMesh &mesh, Vec3 exit, float stepHeight)
    {
        std::optional<std::uint32_t> best;
        float bestScore = std::numeric_limits<float>::max();
        for (std::uint32_t i{}; i < mesh.polygons.size(); ++i)
        {
            const auto &polygon = mesh.polygons[i];
            const std::array<Vec3, 3> vertices{mesh.vertices[polygon.vertices[0]], mesh.vertices[polygon.vertices[1]],
                                               mesh.vertices[polygon.vertices[2]]};
            float floor{}, horizontal{};
            if (!HeightAt(exit, vertices[0], vertices[1], vertices[2], floor))
            {
                horizontal = std::numeric_limits<float>::max();
                for (std::size_t side{}; side < 3; ++side)
                {
                    const auto a = vertices[side], b = vertices[(side + 1) % 3];
                    const auto dx = b.x - a.x, dy = b.y - a.y, length2 = dx * dx + dy * dy;
                    if (length2 <= 1.0e-5F)
                    {
                        continue;
                    }
                    const auto t = std::clamp(((exit.x - a.x) * dx + (exit.y - a.y) * dy) / length2, 0.0F, 1.0F);
                    const auto point = Interpolate(a, b, t);
                    const auto distance = std::hypot(exit.x - point.x, exit.y - point.y);
                    if (distance < horizontal)
                    {
                        horizontal = distance;
                        floor = point.z;
                    }
                }
            }
            const auto vertical = std::abs(exit.z - floor);
            if (horizontal > 96.0F || vertical > std::max(96.0F, stepHeight + 32.0F))
            {
                continue;
            }
            const auto score = horizontal * horizontal + vertical * vertical;
            if (score < bestScore)
            {
                bestScore = score;
                best = i;
            }
        }
        return best;
    }
    void SimplifyContour(CandidateContour &contour, const NavMesh &mesh, float tolerance)
    {
        if (!contour.closed || contour.vertices.size() <= 3)
        {
            return;
        }
        bool changed = true;
        while (changed && contour.vertices.size() > 3)
        {
            changed = false;
            for (std::size_t i{}; i < contour.vertices.size(); ++i)
            {
                const auto a =
                    mesh.vertices[contour.vertices[(i + contour.vertices.size() - 1) % contour.vertices.size()]];
                const auto b = mesh.vertices[contour.vertices[i]];
                const auto c = mesh.vertices[contour.vertices[(i + 1) % contour.vertices.size()]];
                const auto length = std::hypot(c.x - a.x, c.y - a.y);
                if (length <= tolerance)
                {
                    continue;
                }
                const auto lineDistance = std::abs(Cross2(a, c, b)) / length;
                const auto t = ((b.x - a.x) * (c.x - a.x) + (b.y - a.y) * (c.y - a.y)) / (length * length);
                if (t > 0 && t < 1 && lineDistance <= tolerance && std::abs(b.z - (a.z + (c.z - a.z) * t)) <= tolerance)
                {
                    contour.vertices.erase(contour.vertices.begin() + static_cast<std::ptrdiff_t>(i));
                    changed = true;
                    break;
                }
            }
        }
    }
    [[nodiscard]] bool HasClearance(const Scene &scene, const navmesh::analysis::SpatialIndex &index,
                                    std::size_t sourceTriangle, const std::array<Vec3, 3> &points, float height,
                                    float stepHeight)
    {
        const auto &mesh = scene.mesh;
        const std::array<Vec3, 7> samples{points[0],
                                          points[1],
                                          points[2],
                                          (points[0] + points[1]) * 0.5F,
                                          (points[1] + points[2]) * 0.5F,
                                          (points[2] + points[0]) * 0.5F,
                                          (points[0] + points[1] + points[2]) / 3.0F};
        for (const auto &sample : samples)
        {
            AABB query{.min = {sample.x - 0.01F, sample.y - 0.01F, sample.z + 0.1F},
                       .max = {sample.x + 0.01F, sample.y + 0.01F, sample.z + height}};
            for (const auto other : index.QueryAABB(query))
            {
                if (other == sourceTriangle || other >= mesh.triangles.size())
                {
                    continue;
                }
                const auto source = scene.triangleProvenance[other].geometrySource;
                if (source >= scene.geometrySources.size() ||
                    scene.geometrySources[source].sourceType == GeometrySourceType::RenderFallback)
                {
                    continue;
                }
                const auto &tri = mesh.triangles[other];
                if (tri.vertices[0] >= mesh.vertices.size() || tri.vertices[1] >= mesh.vertices.size() ||
                    tri.vertices[2] >= mesh.vertices.size())
                {
                    continue;
                }
                float surface{};
                if (HeightAt(sample, mesh.vertices[tri.vertices[0]], mesh.vertices[tri.vertices[1]],
                             mesh.vertices[tri.vertices[2]], surface) &&
                    surface > sample.z + std::max(0.1F, stepHeight) && surface < sample.z + height)
                {
                    return false;
                }
            }
        }
        return true;
    }
    [[nodiscard]] bool HasWallObstruction(const Scene &scene, const navmesh::analysis::SpatialIndex &index,
                                          std::size_t sourceTriangle, const std::array<Vec3, 3> &floor,
                                          const NavigationProfile &profile)
    {
        AABB query;
        for (const auto point : floor)
        {
            query.Expand(point);
        }
        const auto center = (floor[0] + floor[1] + floor[2]) / 3.0F;
        query.min.z = center.z + profile.stepHeight;
        query.max.z = center.z + profile.agentHeight;
        for (const auto other : index.QueryAABB(query))
        {
            if (other == sourceTriangle || other >= scene.mesh.triangles.size())
            {
                continue;
            }
            const auto source = scene.triangleProvenance[other].geometrySource;
            if (source >= scene.geometrySources.size() ||
                scene.geometrySources[source].sourceType != GeometrySourceType::Collision)
            {
                continue;
            }
            // Rails and posts in the same placed collision mesh can cross a
            // large deck triangle while leaving most of that deck walkable.
            // They need local trimming, not rejection of the whole floor.
            if (sourceTriangle < scene.triangleProvenance.size() &&
                source == scene.triangleProvenance[sourceTriangle].geometrySource &&
                scene.geometrySources[source].sourceType == GeometrySourceType::Collision)
            {
                continue;
            }
            const auto &tri = scene.mesh.triangles[other];
            if (tri.vertices[0] >= scene.mesh.vertices.size() || tri.vertices[1] >= scene.mesh.vertices.size() ||
                tri.vertices[2] >= scene.mesh.vertices.size())
            {
                continue;
            }
            const std::array<Vec3, 3> wall{scene.mesh.vertices[tri.vertices[0]], scene.mesh.vertices[tri.vertices[1]],
                                           scene.mesh.vertices[tri.vertices[2]]};
            const auto u = wall[1] - wall[0], v = wall[2] - wall[0];
            const auto horizontal = std::abs(u.x * v.y - u.y * v.x);
            const auto vertical = std::hypot(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z);
            if (std::atan2(vertical, horizontal) * 180.0F / 3.14159265358979323846F <= profile.maxSlopeDegrees)
            {
                continue;
            }
            for (std::size_t side{}; side < 3; ++side)
            {
                if (SegmentThroughInterior(wall[side], wall[(side + 1) % 3], floor))
                {
                    return true;
                }
            }
        }
        return false;
    }
    [[nodiscard]] std::vector<std::vector<std::uint32_t>> Components(const NavMesh &mesh)
    {
        std::vector<std::vector<std::uint32_t>> components;
        std::vector<bool> seen(mesh.polygons.size());
        for (std::uint32_t first{}; first < mesh.polygons.size(); ++first)
        {
            if (seen[first])
            {
                continue;
            }
            components.emplace_back();
            std::queue<std::uint32_t> pending;
            pending.push(first);
            seen[first] = true;
            while (!pending.empty())
            {
                const auto current = pending.front();
                pending.pop();
                components.back().push_back(current);
                for (const auto neighbor : mesh.polygons[current].neighbors)
                {
                    if (neighbor != NoNeighbor && neighbor < seen.size() && !seen[neighbor])
                    {
                        seen[neighbor] = true;
                        pending.push(neighbor);
                    }
                }
            }
        }
        return components;
    }
    void BuildAdjacency(NavMesh &mesh, float tolerance, float step)
    {
        std::map<Edge, std::vector<std::pair<std::uint32_t, std::uint32_t>>> edges;
        for (std::uint32_t i{}; i < mesh.polygons.size(); ++i)
        {
            auto &polygon = mesh.polygons[i];
            polygon.neighbors.fill(NoNeighbor);
            for (std::uint32_t side{}; side < 3; ++side)
            {
                edges[SortedEdge(mesh.vertices[polygon.vertices[side]], mesh.vertices[polygon.vertices[(side + 1) % 3]],
                                 tolerance)]
                    .push_back({i, side});
            }
        }
        for (const auto &[_, uses] : edges)
        {
            for (std::size_t i{}; i < uses.size(); ++i)
            {
                const auto &left = mesh.polygons[uses[i].first];
                std::optional<std::size_t> match;
                for (std::size_t j{}; j < uses.size(); ++j)
                {
                    if (j != i)
                    {
                        const auto &right = mesh.polygons[uses[j].first];
                        if (!StepCompatible(mesh.vertices[left.vertices[uses[i].second]],
                                            mesh.vertices[left.vertices[(uses[i].second + 1) % 3]],
                                            mesh.vertices[right.vertices[uses[j].second]],
                                            mesh.vertices[right.vertices[(uses[j].second + 1) % 3]], tolerance, step))
                        {
                            continue;
                        }
                        if (match)
                        {
                            match = std::nullopt;
                            break;
                        }
                        match = j;
                    }
                }
                if (match)
                {
                    mesh.polygons[uses[i].first].neighbors[uses[i].second] = uses[*match].first;
                }
            }
        }
    }
    [[nodiscard]] bool HasClearance(const Scene &scene, const navmesh::analysis::SpatialIndex &index,
                                    std::size_t sourceTriangle, const std::array<Vec3, 3> &points, float height,
                                    float stepHeight);
    [[nodiscard]] bool HasWallObstruction(const Scene &scene, const navmesh::analysis::SpatialIndex &index,
                                          std::size_t sourceTriangle, const std::array<Vec3, 3> &floor,
                                          const NavigationProfile &profile);
    /** Remove interior fan vertices only when their entire supported source
     * patch is nearly planar. A convex fan has the same XY footprint and
     * boundary after retriangulation; the source-plane test bounds cumulative
     * height error even when several passes simplify the same area.
     */
    void SimplifyInteriorSurface(CandidateNavMesh &candidate, const Scene &scene)
    {
        const auto maximumHeightError = std::min(16.0F, candidate.profile.stepHeight);
        constexpr std::size_t maximumPasses = 12;
        auto &mesh = candidate.mesh;
        auto &primary = candidate.polygonSourceTriangles;
        auto &contributors = candidate.polygonContributingTriangles;
        navmesh::analysis::SpatialIndex spatial;
        spatial.Build(scene.mesh.triangles, scene.mesh.vertices);
        for (std::size_t pass{}; pass < maximumPasses; ++pass)
        {
            std::vector<std::vector<std::uint32_t>> incident(mesh.vertices.size());
            for (std::uint32_t face{}; face < mesh.polygons.size(); ++face)
            {
                for (const auto vertex : mesh.polygons[face].vertices)
                {
                    incident[vertex].push_back(face);
                }
            }
            std::vector<bool> removed(mesh.polygons.size()), blocked(mesh.vertices.size());
            struct Replacement
            {
                std::array<std::uint32_t, 3> vertices;
                std::size_t source;
                std::vector<std::size_t> contributors;
                std::uint16_t flags;
            };
            std::vector<Replacement> replacements;
            std::size_t removedVertices{};
            for (std::uint32_t center{}; center < incident.size(); ++center)
            {
                const auto &faces = incident[center];
                if (blocked[center] || faces.size() < 3)
                {
                    continue;
                }
                const auto geometrySource = scene.triangleProvenance[primary[faces.front()]].geometrySource;
                const auto flags = mesh.polygons[faces.front()].flags;
                std::map<std::uint32_t, std::uint32_t> next;
                std::set<std::uint32_t> incoming;
                std::set<std::size_t> sources;
                bool eligible = true;
                for (const auto faceIndex : faces)
                {
                    const auto &face = mesh.polygons[faceIndex];
                    if (removed[faceIndex] || face.flags != flags ||
                        scene.triangleProvenance[primary[faceIndex]].geometrySource != geometrySource)
                    {
                        eligible = false;
                        break;
                    }
                    const auto at =
                        std::find(face.vertices.begin(), face.vertices.end(), center) - face.vertices.begin();
                    const auto from = face.vertices[(at + 1) % 3], to = face.vertices[(at + 2) % 3];
                    if (face.neighbors[at] == NoNeighbor || face.neighbors[(at + 2) % 3] == NoNeighbor ||
                        !next.emplace(from, to).second || !incoming.insert(to).second)
                    {
                        eligible = false;
                        break;
                    }
                    sources.insert(contributors[faceIndex].begin(), contributors[faceIndex].end());
                }
                if (!eligible || next.size() != faces.size() || incoming.size() != faces.size())
                {
                    continue;
                }
                // A traversable step can join equal XY edges whose outer
                // endpoints have distinct vertex IDs. Such a fan has no
                // closed vertex ring and must remain unchanged.
                std::vector<std::uint32_t> ring;
                auto current = next.begin()->first;
                do
                {
                    ring.push_back(current);
                    const auto edge = next.find(current);
                    if (edge == next.end())
                    {
                        eligible = false;
                        break;
                    }
                    current = edge->second;
                } while (current != ring.front() && ring.size() <= faces.size());
                if (!eligible || current != ring.front() || ring.size() != faces.size())
                {
                    continue;
                }
                if (std::any_of(ring.begin(), ring.end(), [&](auto vertex) { return blocked[vertex]; }))
                {
                    continue;
                }
                const auto &points = mesh.vertices;
                const auto minimumArea = candidate.profile.weldTolerance * candidate.profile.weldTolerance;
                for (std::size_t i{}; i < ring.size(); ++i)
                {
                    if (Cross2(points[ring[i]], points[ring[(i + 1) % ring.size()]],
                               points[ring[(i + 2) % ring.size()]]) <= minimumArea)
                    {
                        eligible = false;
                        break;
                    }
                }
                if (!eligible)
                {
                    continue;
                }
                // Fit the local surface rather than using an arbitrary three
                // points; the latter exaggerates error on a smooth hillside.
                // Every original source vertex must remain close to this plane.
                // Both the source surface and the replacement triangles are
                // linear, so each can differ from that plane by at most half
                // the total permitted height error throughout this patch.
                Vec3 mean = points[center];
                for (const auto vertex : ring)
                {
                    mean = mean + points[vertex];
                }
                mean = mean / static_cast<float>(ring.size() + 1);
                double xx{}, xy{}, yy{}, xz{}, yz{};
                const auto accumulate = [&](Vec3 point)
                {
                    const auto x = static_cast<double>(point.x - mean.x);
                    const auto y = static_cast<double>(point.y - mean.y);
                    const auto z = static_cast<double>(point.z - mean.z);
                    xx += x * x;
                    xy += x * y;
                    yy += y * y;
                    xz += x * z;
                    yz += y * z;
                };
                accumulate(points[center]);
                for (const auto vertex : ring)
                {
                    accumulate(points[vertex]);
                }
                const auto determinant = xx * yy - xy * xy;
                if (determinant <= 1.0e-6)
                {
                    continue;
                }
                const auto dzdx = (xz * yy - yz * xy) / determinant;
                const auto dzdy = (yz * xx - xz * xy) / determinant;
                const auto nearPlane = [&](Vec3 point)
                {
                    const auto expected =
                        static_cast<double>(mean.z) + dzdx * (point.x - mean.x) + dzdy * (point.y - mean.y);
                    return std::abs(static_cast<double>(point.z) - expected) <= maximumHeightError * 0.5;
                };
                if (!nearPlane(points[center]))
                {
                    continue;
                }
                for (const auto vertex : ring)
                {
                    if (!nearPlane(points[vertex]))
                    {
                        eligible = false;
                        break;
                    }
                }
                for (const auto source : sources)
                {
                    if (source >= scene.mesh.triangles.size() ||
                        scene.triangleProvenance[source].geometrySource != geometrySource)
                    {
                        eligible = false;
                        break;
                    }
                    const auto &original = scene.mesh.triangles[source];
                    for (const auto vertex : original.vertices)
                    {
                        if (vertex >= scene.mesh.vertices.size() || !nearPlane(scene.mesh.vertices[vertex]))
                        {
                            eligible = false;
                            break;
                        }
                    }
                    if (!eligible)
                    {
                        break;
                    }
                }
                if (!eligible)
                {
                    continue;
                }
                std::vector<Replacement> proposed;
                for (std::size_t corner = 1; corner + 1 < ring.size(); ++corner)
                {
                    const auto first = points[ring[0]], second = points[ring[corner]], third = points[ring[corner + 1]];
                    const auto area = Cross2(first, second, third);
                    const auto u = second - first, v = third - first;
                    const auto vertical = std::hypot(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z);
                    const auto slope = std::atan2(vertical, area) * 180.0F / 3.14159265358979323846F;
                    if (area <= minimumArea || slope > candidate.profile.maxSlopeDegrees)
                    {
                        eligible = false;
                        break;
                    }
                    const std::array<Vec3, 3> triangle{first, second, third};
                    if (!HasClearance(scene, spatial, *sources.begin(), triangle, candidate.profile.clearance,
                                      candidate.profile.stepHeight) ||
                        HasWallObstruction(scene, spatial, *sources.begin(), triangle, candidate.profile))
                    {
                        eligible = false;
                        break;
                    }
                    proposed.push_back({{ring[0], ring[corner], ring[corner + 1]},
                                        *sources.begin(),
                                        {sources.begin(), sources.end()},
                                        flags});
                }
                if (!eligible)
                {
                    continue;
                }
                for (const auto face : faces)
                {
                    removed[face] = true;
                }
                blocked[center] = true;
                for (const auto vertex : ring)
                {
                    blocked[vertex] = true;
                }
                replacements.insert(replacements.end(), std::make_move_iterator(proposed.begin()),
                                    std::make_move_iterator(proposed.end()));
                ++removedVertices;
            }
            if (!removedVertices)
            {
                break;
            }
            NavMesh nextMesh;
            std::vector<std::size_t> nextPrimary;
            std::vector<std::vector<std::size_t>> nextContributors;
            std::vector<std::uint32_t> remap(mesh.vertices.size(), NoNeighbor);
            const auto makeVertex = [&](std::uint32_t old)
            {
                if (remap[old] == NoNeighbor)
                {
                    remap[old] = static_cast<std::uint32_t>(nextMesh.vertices.size());
                    nextMesh.vertices.push_back(mesh.vertices[old]);
                }
                return remap[old];
            };
            for (std::size_t i{}; i < mesh.polygons.size(); ++i)
            {
                if (!removed[i])
                {
                    auto face = mesh.polygons[i];
                    for (auto &vertex : face.vertices)
                    {
                        vertex = makeVertex(vertex);
                    }
                    nextMesh.polygons.push_back(face);
                    nextPrimary.push_back(primary[i]);
                    nextContributors.push_back(std::move(contributors[i]));
                }
            }
            for (auto &replacement : replacements)
            {
                NavPolygon face;
                for (std::size_t i{}; i < 3; ++i)
                {
                    face.vertices[i] = makeVertex(replacement.vertices[i]);
                }
                face.flags = replacement.flags;
                nextMesh.polygons.push_back(face);
                nextPrimary.push_back(replacement.source);
                nextContributors.push_back(std::move(replacement.contributors));
            }
            mesh = std::move(nextMesh);
            primary = std::move(nextPrimary);
            contributors = std::move(nextContributors);
            BuildAdjacency(mesh, candidate.profile.weldTolerance, candidate.profile.stepHeight);
        }
    }
    /** Split inset triangle edges at nearby vertices on the same traversable
     * level. Adjacent source triangles can retain different portions of a
     * shared edge after their exposed sides are inset; without these cuts the
     * common portion has no matching edge and becomes a false island.
     */
    void SplitPartialOutputEdges(NavMesh &mesh, std::vector<std::size_t> &polygonSources,
                                 const NavigationProfile &profile)
    {
        if (mesh.polygons.empty())
        {
            return;
        }
        constexpr float binSize = 128.0F;
        std::map<Key2, std::vector<std::uint32_t>> bins;
        std::map<Key, std::uint32_t> keys;
        for (std::uint32_t i{}; i < mesh.vertices.size(); ++i)
        {
            bins[Quantize2(mesh.vertices[i], binSize)].push_back(i);
            keys.try_emplace(Quantize(mesh.vertices[i], profile.weldTolerance), i);
        }
        const auto makeVertex = [&](Vec3 point)
        {
            auto [where, created] = keys.try_emplace(Quantize(point, profile.weldTolerance),
                                                     static_cast<std::uint32_t>(mesh.vertices.size()));
            if (created)
            {
                mesh.vertices.push_back(point);
            }
            return where->second;
        };
        NavMesh split;
        std::vector<std::size_t> splitSources;
        for (std::size_t i{}; i < mesh.polygons.size(); ++i)
        {
            const auto face = mesh.polygons[i].vertices;
            std::vector<std::uint32_t> boundary;
            bool hasCuts{};
            for (std::size_t side{}; side < 3; ++side)
            {
                const auto a = mesh.vertices[face[side]], b = mesh.vertices[face[(side + 1) % 3]];
                boundary.push_back(face[side]);
                const auto dx = b.x - a.x, dy = b.y - a.y, length2 = dx * dx + dy * dy;
                if (length2 <= profile.weldTolerance * profile.weldTolerance)
                {
                    continue;
                }
                const auto length = std::sqrt(length2);
                const auto low = Quantize2({std::min(a.x, b.x), std::min(a.y, b.y), 0}, binSize);
                const auto high = Quantize2({std::max(a.x, b.x), std::max(a.y, b.y), 0}, binSize);
                std::vector<float> cuts;
                for (auto x = low.first - 1; x <= high.first + 1; ++x)
                {
                    for (auto y = low.second - 1; y <= high.second + 1; ++y)
                    {
                        if (const auto found = bins.find({x, y}); found != bins.end())
                        {
                            for (const auto vertex : found->second)
                            {
                                if (vertex == face[side] || vertex == face[(side + 1) % 3])
                                {
                                    continue;
                                }
                                const auto point = mesh.vertices[vertex];
                                const auto t = ((point.x - a.x) * dx + (point.y - a.y) * dy) / length2;
                                if (t <= profile.weldTolerance / length || t >= 1.0F - profile.weldTolerance / length ||
                                    std::abs(Cross2(a, b, point)) / length > profile.weldTolerance ||
                                    std::abs(point.z - (a.z + (b.z - a.z) * t)) >
                                        profile.stepHeight + profile.weldTolerance)
                                {
                                    continue;
                                }
                                cuts.push_back(t);
                            }
                        }
                    }
                }
                std::sort(cuts.begin(), cuts.end());
                float previous{};
                for (const auto t : cuts)
                {
                    if (t - previous <= profile.weldTolerance / length)
                    {
                        continue;
                    }
                    boundary.push_back(makeVertex(Interpolate(a, b, t)));
                    hasCuts = true;
                    previous = t;
                }
            }
            if (!hasCuts)
            {
                split.polygons.push_back(mesh.polygons[i]);
                splitSources.push_back(polygonSources[i]);
                continue;
            }
            const auto center =
                makeVertex((mesh.vertices[face[0]] + mesh.vertices[face[1]] + mesh.vertices[face[2]]) / 3.0F);
            for (std::size_t corner{}; corner < boundary.size(); ++corner)
            {
                NavPolygon polygon;
                polygon.vertices = {center, boundary[corner], boundary[(corner + 1) % boundary.size()]};
                polygon.neighbors.fill(NoNeighbor);
                if (Area2(mesh.vertices[polygon.vertices[0]], mesh.vertices[polygon.vertices[1]],
                          mesh.vertices[polygon.vertices[2]]) <= profile.weldTolerance * profile.weldTolerance)
                {
                    continue;
                }
                split.polygons.push_back(polygon);
                splitSources.push_back(polygonSources[i]);
            }
        }
        split.vertices = std::move(mesh.vertices);
        mesh = std::move(split);
        polygonSources = std::move(splitSources);
    }
    /** Resolve competing faces on a compatible edge by discarding the
     * smallest face. Split collision meshes and LAND may overlap at a seam;
     * keeping all three edge uses would make the candidate non-manifold.
     */
    [[nodiscard]] std::size_t CullNonManifoldFaces(NavMesh &mesh, std::vector<std::size_t> &polygonSources,
                                                   const NavigationProfile &profile)
    {
        std::size_t removed{};
        while (true)
        {
            std::map<Edge, std::vector<std::pair<std::size_t, std::size_t>>> edges;
            for (std::size_t i{}; i < mesh.polygons.size(); ++i)
            {
                for (std::size_t side{}; side < 3; ++side)
                {
                    const auto &face = mesh.polygons[i].vertices;
                    edges[SortedEdge(mesh.vertices[face[side]], mesh.vertices[face[(side + 1) % 3]],
                                     profile.weldTolerance)]
                        .push_back({i, side});
                }
            }
            std::vector<bool> discard(mesh.polygons.size());
            for (const auto &[_, uses] : edges)
            {
                if (uses.size() <= 2)
                {
                    continue;
                }
                for (const auto &[index, side] : uses)
                {
                    const auto &left = mesh.polygons[index].vertices;
                    const auto a = mesh.vertices[left[side]], b = mesh.vertices[left[(side + 1) % 3]];
                    std::vector<std::size_t> compatible{index};
                    for (const auto &[otherIndex, otherSide] : uses)
                    {
                        if (otherIndex != index)
                        {
                            const auto &right = mesh.polygons[otherIndex].vertices;
                            if (StepCompatible(a, b, mesh.vertices[right[otherSide]],
                                               mesh.vertices[right[(otherSide + 1) % 3]], profile.weldTolerance,
                                               profile.stepHeight))
                            {
                                compatible.push_back(otherIndex);
                            }
                        }
                    }
                    if (compatible.size() <= 2)
                    {
                        continue;
                    }
                    const auto area = [&](std::size_t polygon)
                    {
                        const auto &face = mesh.polygons[polygon].vertices;
                        return Area2(mesh.vertices[face[0]], mesh.vertices[face[1]], mesh.vertices[face[2]]);
                    };
                    const auto smallest =
                        *std::min_element(compatible.begin(), compatible.end(), [&](std::size_t lhs, std::size_t rhs)
                                          { return std::pair{area(lhs), lhs} < std::pair{area(rhs), rhs}; });
                    discard[smallest] = true;
                }
            }
            if (std::none_of(discard.begin(), discard.end(), [](bool value) { return value; }))
            {
                break;
            }
            NavMesh next;
            std::vector<std::size_t> sources;
            std::map<std::uint32_t, std::uint32_t> vertices;
            for (std::size_t i{}; i < mesh.polygons.size(); ++i)
            {
                if (discard[i])
                {
                    ++removed;
                    continue;
                }
                auto polygon = mesh.polygons[i];
                for (auto &vertex : polygon.vertices)
                {
                    auto [where, created] =
                        vertices.try_emplace(vertex, static_cast<std::uint32_t>(next.vertices.size()));
                    if (created)
                    {
                        next.vertices.push_back(mesh.vertices[vertex]);
                    }
                    vertex = where->second;
                }
                next.polygons.push_back(polygon);
                sources.push_back(polygonSources[i]);
            }
            mesh = std::move(next);
            polygonSources = std::move(sources);
        }
        BuildAdjacency(mesh, profile.weldTolerance, profile.stepHeight);
        return removed;
    }
    /** Bridge only short facing boundary edges whose intervening triangle is
     * supported by accepted source geometry. This repairs gaps introduced by
     * the radius inset where separately triangulated decks overlap in space;
     * a missing floor or a steep/blocked transition is never invented.
     */
    void ConnectSupportedSeams(NavMesh &mesh, std::vector<std::size_t> &polygonSources, const Scene &scene,
                               const navmesh::analysis::SpatialIndex &spatial, const std::vector<bool> &acceptedSources,
                               const NavigationProfile &profile)
    {
        if (profile.agentRadius <= 0 || mesh.polygons.empty())
        {
            return;
        }
        const auto components = Components(mesh);
        if (components.size() < 2)
        {
            return;
        }
        std::vector<std::size_t> componentOf(mesh.polygons.size());
        for (std::size_t i{}; i < components.size(); ++i)
        {
            for (const auto polygon : components[i])
            {
                componentOf[polygon] = i;
            }
        }
        struct BoundaryEdge
        {
            std::uint32_t polygon{}, side{};
            Vec3 a, b;
        };
        std::vector<BoundaryEdge> boundary;
        const auto maxGap = std::max(32.0F, profile.agentRadius * 6.0F);
        std::map<Key2, std::vector<std::size_t>> bins;
        for (std::uint32_t polygon{}; polygon < mesh.polygons.size(); ++polygon)
        {
            for (std::uint32_t side{}; side < 3; ++side)
            {
                if (mesh.polygons[polygon].neighbors[side] == NoNeighbor)
                {
                    const auto a = mesh.vertices[mesh.polygons[polygon].vertices[side]];
                    const auto b = mesh.vertices[mesh.polygons[polygon].vertices[(side + 1) % 3]];
                    const auto index = boundary.size();
                    boundary.push_back({polygon, side, a, b});
                    bins[Quantize2((a + b) * 0.5F, maxGap)].push_back(index);
                }
            }
        }
        std::vector<std::tuple<float, std::size_t, std::size_t>> pairs;
        for (std::size_t i{}; i < boundary.size(); ++i)
        {
            const auto &left = boundary[i];
            const auto bin = Quantize2((left.a + left.b) * 0.5F, maxGap);
            for (std::int64_t x = bin.first - 2; x <= bin.first + 2; ++x)
            {
                for (std::int64_t y = bin.second - 2; y <= bin.second + 2; ++y)
                {
                    if (const auto found = bins.find({x, y}); found != bins.end())
                    {
                        for (const auto j : found->second)
                        {
                            if (j <= i)
                            {
                                continue;
                            }
                            const auto &right = boundary[j];
                            if (componentOf[left.polygon] == componentOf[right.polygon])
                            {
                                continue;
                            }
                            // A split or inset of one source triangle must not
                            // be filled over its own surviving pieces.
                            if (polygonSources[left.polygon] == polygonSources[right.polygon])
                            {
                                continue;
                            }
                            const auto first = std::hypot(left.a.x - right.b.x, left.a.y - right.b.y);
                            const auto second = std::hypot(left.b.x - right.a.x, left.b.y - right.a.y);
                            if (first > maxGap || second > maxGap || first < 0.01F || second < 0.01F ||
                                std::abs(left.a.z - right.b.z) > profile.stepHeight + profile.weldTolerance ||
                                std::abs(left.b.z - right.a.z) > profile.stepHeight + profile.weldTolerance)
                            {
                                continue;
                            }
                            if (Cross2(left.a, left.b, right.a) >= -profile.weldTolerance ||
                                Cross2(left.a, left.b, right.b) >= -profile.weldTolerance ||
                                Cross2(right.a, right.b, left.a) >= -profile.weldTolerance ||
                                Cross2(right.a, right.b, left.b) >= -profile.weldTolerance)
                            {
                                continue;
                            }
                            pairs.emplace_back(std::max(first, second), i, j);
                        }
                    }
                }
            }
        }
        std::sort(pairs.begin(), pairs.end());
        std::vector<std::size_t> parent(components.size());
        std::iota(parent.begin(), parent.end(), 0);
        std::set<std::array<std::uint32_t, 3>> existingFaces;
        std::map<Edge, std::vector<std::pair<Vec3, Vec3>>> occupiedEdges;
        const auto recordFace = [&](const std::array<std::uint32_t, 3> &face)
        {
            auto sorted = face;
            std::sort(sorted.begin(), sorted.end());
            existingFaces.insert(sorted);
            for (std::size_t side{}; side < 3; ++side)
            {
                const auto a = mesh.vertices[face[side]], b = mesh.vertices[face[(side + 1) % 3]];
                occupiedEdges[SortedEdge(a, b, profile.weldTolerance)].push_back({a, b});
            }
        };
        for (const auto &polygon : mesh.polygons)
        {
            recordFace(polygon.vertices);
        }
        const auto root = [&](std::size_t i)
        {
            while (parent[i] != i)
            {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            return i;
        };
        const auto supported = [&](Vec3 point)
        {
            AABB query{.min = {point.x - 0.1F, point.y - 0.1F, point.z - profile.stepHeight - 1.0F},
                       .max = {point.x + 0.1F, point.y + 0.1F, point.z + profile.stepHeight + 1.0F}};
            for (const auto source : spatial.QueryAABB(query))
            {
                if (source >= acceptedSources.size() || !acceptedSources[source])
                {
                    continue;
                }
                const auto &tri = scene.mesh.triangles[source];
                float height{};
                if (HeightAt(point, scene.mesh.vertices[tri.vertices[0]], scene.mesh.vertices[tri.vertices[1]],
                             scene.mesh.vertices[tri.vertices[2]], height) &&
                    std::abs(height - point.z) <= profile.stepHeight + 1.0F)
                {
                    return true;
                }
            }
            return false;
        };
        for (const auto &[_, i, j] : pairs)
        {
            const auto &left = boundary[i], right = boundary[j];
            const auto leftStart = mesh.polygons[left.polygon].vertices[left.side];
            const auto leftEnd = mesh.polygons[left.polygon].vertices[(left.side + 1) % 3];
            const auto rightStart = mesh.polygons[right.polygon].vertices[right.side];
            const auto rightEnd = mesh.polygons[right.polygon].vertices[(right.side + 1) % 3];
            const auto from = root(componentOf[left.polygon]), to = root(componentOf[right.polygon]);
            if (from == to)
            {
                continue;
            }
            if (!supported((left.a + left.b + right.a + right.b) * 0.25F))
            {
                continue;
            }
            std::array<std::array<std::uint32_t, 3>, 2> faces{
                {{leftStart, leftEnd, rightStart}, {leftStart, rightStart, rightEnd}}};
            bool traversable = true;
            for (auto &face : faces)
            {
                auto points =
                    std::array<Vec3, 3>{mesh.vertices[face[0]], mesh.vertices[face[1]], mesh.vertices[face[2]]};
                const auto u = points[1] - points[0], v = points[2] - points[0];
                const auto horizontal = std::abs(u.x * v.y - u.y * v.x);
                if (horizontal <= profile.weldTolerance * profile.weldTolerance ||
                    std::atan2(std::hypot(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z), horizontal) * 180.0F /
                            3.14159265358979323846F >
                        profile.maxSlopeDegrees)
                {
                    traversable = false;
                    break;
                }
                if (Cross2(points[0], points[1], points[2]) < 0)
                {
                    std::swap(face[1], face[2]);
                    std::swap(points[1], points[2]);
                }
                const auto center = (points[0] + points[1] + points[2]) / 3.0F;
                if (!supported(center) ||
                    !HasClearance(scene, spatial, scene.mesh.triangles.size(), points, profile.clearance,
                                  profile.stepHeight) ||
                    HasWallObstruction(scene, spatial, scene.mesh.triangles.size(), points, profile))
                {
                    traversable = false;
                    break;
                }
            }
            if (!traversable)
            {
                continue;
            }
            // A connector may meet a third inset surface or another
            // connector. Never add a duplicate face or a third compatible
            // use of an edge; either creates invalid NAVM topology.
            std::map<Edge, std::vector<std::pair<Vec3, Vec3>>> prospectiveEdges;
            std::set<std::array<std::uint32_t, 3>> prospectiveFaces;
            for (const auto &face : faces)
            {
                auto sorted = face;
                std::sort(sorted.begin(), sorted.end());
                if (existingFaces.contains(sorted) || !prospectiveFaces.insert(sorted).second)
                {
                    traversable = false;
                    break;
                }
                for (std::size_t side{}; side < 3; ++side)
                {
                    const auto a = mesh.vertices[face[side]], b = mesh.vertices[face[(side + 1) % 3]];
                    prospectiveEdges[SortedEdge(a, b, profile.weldTolerance)].push_back({a, b});
                }
            }
            if (!traversable)
            {
                continue;
            }
            for (const auto &face : faces)
            {
                for (std::size_t side{}; side < 3; ++side)
                {
                    const auto a = mesh.vertices[face[side]], b = mesh.vertices[face[(side + 1) % 3]];
                    std::size_t compatible{};
                    const auto key = SortedEdge(a, b, profile.weldTolerance);
                    for (const auto &[otherA, otherB] : occupiedEdges[key])
                    {
                        if (StepCompatible(a, b, otherA, otherB, profile.weldTolerance, profile.stepHeight))
                        {
                            ++compatible;
                        }
                    }
                    for (const auto &[otherA, otherB] : prospectiveEdges[key])
                    {
                        if (StepCompatible(a, b, otherA, otherB, profile.weldTolerance, profile.stepHeight))
                        {
                            ++compatible;
                        }
                    }
                    if (compatible > 2)
                    {
                        traversable = false;
                    }
                }
            }
            if (!traversable)
            {
                continue;
            }
            for (std::size_t face{}; face < faces.size(); ++face)
            {
                NavPolygon connector;
                connector.vertices = faces[face];
                connector.neighbors.fill(NoNeighbor);
                mesh.polygons.push_back(connector);
                polygonSources.push_back(polygonSources[face == 0 ? left.polygon : right.polygon]);
                recordFace(faces[face]);
            }
            parent[to] = from;
        }
        BuildAdjacency(mesh, profile.weldTolerance, profile.stepHeight);
    }
} // namespace

namespace
{
    /// Compact unused vertices and remap polygon/contour indices without changing geometry.
    void CompactBorderVertices(CandidateNavMesh &candidate)
    {
        std::vector<std::uint32_t> remap(candidate.mesh.vertices.size(), NoNeighbor);
        std::vector<Vec3> vertices;
        for (auto &face : candidate.mesh.polygons)
        {
            for (auto &vertex : face.vertices)
            {
                if (remap[vertex] == NoNeighbor)
                {
                    remap[vertex] = static_cast<std::uint32_t>(vertices.size());
                    vertices.push_back(candidate.mesh.vertices[vertex]);
                }
                vertex = remap[vertex];
            }
        }
        std::erase_if(candidate.contours,
                      [&](const auto &contour)
                      {
                          return std::any_of(contour.vertices.begin(), contour.vertices.end(),
                                             [&](auto vertex) { return remap[vertex] == NoNeighbor; });
                      });
        for (auto &contour : candidate.contours)
        {
            for (auto &vertex : contour.vertices)
            {
                vertex = remap[vertex];
            }
        }
        candidate.mesh.vertices = std::move(vertices);
    }

    /** Remove a collinear boundary subdivision by retriangulating its incident fan.
     * The directed interior rim and XY footprint are preserved. Portal triangles,
     * mixed regions/flags, nonmanifold fans and nonwalkable replacements are left
     * alone. Polygon compaction remaps region, source, door and portal indices.
     */
    bool RemoveBorderSubdivision(CandidateNavMesh &candidate, std::uint32_t vertex)
    {
        const auto &mesh = candidate.mesh;
        std::vector<std::uint32_t> incident;
        std::map<std::uint32_t, std::uint32_t> rim;
        std::optional<std::uint32_t> incoming, outgoing;
        for (std::uint32_t polygon{}; polygon < mesh.polygons.size(); ++polygon)
        {
            const auto &face = mesh.polygons[polygon];
            const auto found = std::find(face.vertices.begin(), face.vertices.end(), vertex);
            if (found == face.vertices.end())
            {
                continue;
            }
            if ((!incident.empty() && face.flags != mesh.polygons[incident.front()].flags) ||
                std::any_of(candidate.borderLinks.begin(), candidate.borderLinks.end(),
                            [&](const auto &link) { return link.polygon == polygon; }))
            {
                return false;
            }
            incident.push_back(polygon);
            const auto side = static_cast<std::size_t>(found - face.vertices.begin());
            if (!rim.emplace(face.vertices[(side + 1) % 3], face.vertices[(side + 2) % 3]).second)
            {
                return false;
            }
            if (face.neighbors[side] == NoNeighbor)
            {
                if (outgoing)
                {
                    return false;
                }
                outgoing = face.vertices[(side + 1) % 3];
            }
            if (face.neighbors[(side + 2) % 3] == NoNeighbor)
            {
                if (incoming)
                {
                    return false;
                }
                incoming = face.vertices[(side + 2) % 3];
            }
        }
        if (incident.size() < 2 || !incoming || !outgoing)
        {
            return false;
        }
        const auto a = mesh.vertices[*incoming];
        const auto b = mesh.vertices[*outgoing];
        const auto point = mesh.vertices[vertex];
        const auto length = std::hypot(b.x - a.x, b.y - a.y);
        if (length <= candidate.profile.weldTolerance ||
            std::abs(Cross2(a, b, point)) > length * candidate.profile.weldTolerance)
        {
            return false;
        }
        const auto fraction = ((point.x - a.x) * (b.x - a.x) + (point.y - a.y) * (b.y - a.y)) / (length * length);
        if (fraction <= 0 || fraction >= 1 ||
            std::abs(Interpolate(a, b, fraction).z - point.z) > candidate.profile.stepHeight)
        {
            return false;
        }
        for (const auto &region : candidate.regions)
        {
            const auto count = std::count_if(incident.begin(), incident.end(),
                                             [&](auto polygon)
                                             {
                                                 return std::find(region.polygons.begin(), region.polygons.end(),
                                                                  polygon) != region.polygons.end();
                                             });
            if (count != 0 && static_cast<std::size_t>(count) != incident.size())
            {
                return false;
            }
        }

        // A boundary fan has one open directed rim. Ear clipping preserves its
        // interior edges while replacing the subdivided seam with a single edge.
        std::vector<std::uint32_t> boundary{*outgoing};
        while (boundary.back() != *incoming && boundary.size() <= rim.size())
        {
            const auto next = rim.find(boundary.back());
            if (next == rim.end())
            {
                return false;
            }
            boundary.push_back(next->second);
        }
        if (boundary.back() != *incoming || boundary.size() != incident.size() + 1)
        {
            return false;
        }
        std::vector<std::array<std::uint32_t, 3>> replacements;
        while (boundary.size() > 2)
        {
            bool removed{};
            for (std::size_t index{}; index < boundary.size(); ++index)
            {
                const std::array face{boundary[(index + boundary.size() - 1) % boundary.size()], boundary[index],
                                      boundary[(index + 1) % boundary.size()]};
                const auto first = mesh.vertices[face[0]];
                const auto second = mesh.vertices[face[1]];
                const auto third = mesh.vertices[face[2]];
                const auto u = second - first;
                const auto v = third - first;
                const auto area = Cross2(first, second, third);
                if (area <= 0.01F || std::atan2(std::hypot(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z), area) *
                                             180.0F / 3.14159265358979323846F >
                                         candidate.profile.maxSlopeDegrees)
                {
                    continue;
                }
                const auto containsOther = std::any_of(boundary.begin(), boundary.end(),
                                                       [&](auto other)
                                                       {
                                                           if (std::find(face.begin(), face.end(), other) != face.end())
                                                           {
                                                               return false;
                                                           }
                                                           const auto p = mesh.vertices[other];
                                                           return Cross2(first, second, p) >= -0.01F &&
                                                                  Cross2(second, third, p) >= -0.01F &&
                                                                  Cross2(third, first, p) >= -0.01F;
                                                       });
                if (containsOther)
                {
                    continue;
                }
                replacements.push_back(face);
                boundary.erase(boundary.begin() + index);
                removed = true;
                break;
            }
            if (!removed)
            {
                return false;
            }
        }

        // Work transactionally: invalid topology or an unplaceable door leaves
        // the input untouched. The removed polygon is the last incident index.
        auto revised = candidate;
        std::vector<std::size_t> contributors;
        for (const auto polygon : incident)
        {
            const auto &sources = candidate.polygonContributingTriangles[polygon];
            contributors.insert(contributors.end(), sources.begin(), sources.end());
        }
        std::sort(contributors.begin(), contributors.end());
        contributors.erase(std::unique(contributors.begin(), contributors.end()), contributors.end());
        for (std::size_t index{}; index < replacements.size(); ++index)
        {
            revised.mesh.polygons[incident[index]].vertices = replacements[index];
            revised.polygonContributingTriangles[incident[index]] = contributors;
        }
        for (auto &exit : revised.exits)
        {
            if (!exit.polygon || std::find(incident.begin(), incident.end(), *exit.polygon) == incident.end())
            {
                continue;
            }
            exit.polygon.reset();
            for (std::size_t index{}; index < replacements.size(); ++index)
            {
                const auto &face = replacements[index];
                float height{};
                if (HeightAt(exit.position, mesh.vertices[face[0]], mesh.vertices[face[1]], mesh.vertices[face[2]],
                             height))
                {
                    exit.polygon = incident[index];
                    break;
                }
            }
            if (!exit.polygon)
            {
                return false;
            }
        }
        const auto erased = incident.back();
        revised.mesh.polygons.erase(revised.mesh.polygons.begin() + erased);
        revised.polygonSourceTriangles.erase(revised.polygonSourceTriangles.begin() + erased);
        revised.polygonContributingTriangles.erase(revised.polygonContributingTriangles.begin() + erased);
        for (auto &region : revised.regions)
        {
            std::erase(region.polygons, erased);
            region.area = 0;
            for (auto &polygon : region.polygons)
            {
                polygon -= polygon > erased;
                const auto &face = revised.mesh.polygons[polygon];
                region.area += Area2(revised.mesh.vertices[face.vertices[0]], revised.mesh.vertices[face.vertices[1]],
                                     revised.mesh.vertices[face.vertices[2]]) *
                               0.5F;
            }
        }
        for (auto &exit : revised.exits)
        {
            if (exit.polygon)
            {
                *exit.polygon -= *exit.polygon > erased;
            }
        }
        for (auto &link : revised.borderLinks)
        {
            link.polygon -= link.polygon > erased;
        }
        for (auto &contour : revised.contours)
        {
            std::erase(contour.vertices, vertex);
        }
        BuildAdjacency(revised.mesh, revised.profile.weldTolerance, revised.profile.stepHeight);
        CompactBorderVertices(revised);
        if (!ValidateCandidateTopology(revised).valid)
        {
            return false;
        }
        candidate = std::move(revised);
        return true;
    }

    /** Coalesce generated subdivisions strictly inside a complete neighboring edge.
     * Only boundary fans on the same CELL side and at compatible heights qualify;
     * disconnected gaps and interior geometry cannot become border connections.
     */
    template <class Border>
    void CoalesceGeneratedBorders(CandidateNavMesh &candidate, const std::vector<NavMesh> &neighbors,
                                  const Border &border, float maximumGap)
    {
        for (const auto &neighbor : neighbors)
        {
            for (const auto &face : neighbor.polygons)
            {
                for (std::uint8_t edge{}; edge < 3; ++edge)
                {
                    if (face.vertices[edge] >= neighbor.vertices.size() ||
                        face.vertices[(edge + 1) % 3] >= neighbor.vertices.size() ||
                        (!(face.flags & (1U << edge)) && face.neighbors[edge] != NoNeighbor &&
                         face.neighbors[edge] != 0xffffU))
                    {
                        continue;
                    }
                    const auto a = neighbor.vertices[face.vertices[edge]];
                    const auto b = neighbor.vertices[face.vertices[(edge + 1) % 3]];
                    const auto side = border(a, b, AuthoredBorderTolerance);
                    const auto lengthSquared = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
                    if (side < 0 || lengthSquared <= 0.01F)
                    {
                        continue;
                    }
                    std::uint32_t vertex{};
                    while (vertex < candidate.mesh.vertices.size())
                    {
                        const auto p = candidate.mesh.vertices[vertex];
                        const auto fraction = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / lengthSquared;
                        if (fraction <= 0 || fraction >= 1 || border(p, p, maximumGap) != side ||
                            std::abs(p.z - Interpolate(a, b, fraction).z) > candidate.profile.stepHeight)
                        {
                            ++vertex;
                            continue;
                        }
                        if (RemoveBorderSubdivision(candidate, vertex))
                        {
                            vertex = 0;
                        }
                        else
                        {
                            ++vertex;
                        }
                    }
                }
            }
        }
    }

    /// Reject subdivisions that collapse after welding or exceed the walking slope.
    bool CanSubdivideBorder(Vec3 a, Vec3 b, Vec3 opposite, Vec3 start, Vec3 end, Vec3 targetStart, Vec3 targetEnd,
                            const NavigationProfile &profile)
    {
        const auto snap = [&](Vec3 point, Vec3 target)
        {
            return std::hypot(point.x - target.x, point.y - target.y) <=
                               std::max(profile.weldTolerance, AuthoredBorderTolerance) ||
                           Cross2(a, b, target) > 0
                       ? target
                       : point;
        };
        start = snap(start, targetStart);
        end = snap(end, targetEnd);
        const std::array<std::array<Vec3, 3>, 5> faces{
            std::array{start, end, opposite}, std::array{a, start, opposite}, std::array{end, b, opposite},
            std::array{start, targetStart, end}, std::array{end, targetStart, targetEnd}};
        for (const auto &face : faces)
        {
            const auto area = Cross2(face[0], face[1], face[2]);
            if (area < -0.01F)
            {
                return false;
            }
            if (area <= 0.01F)
            {
                continue;
            }
            const auto first = face[1] - face[0];
            const auto second = face[2] - face[0];
            const auto normalX = first.y * second.z - first.z * second.y;
            const auto normalY = first.z * second.x - first.x * second.z;
            const auto slope = std::atan2(std::hypot(normalX, normalY), area) * 180.0F / 3.14159265358979323846F;
            if (slope > profile.maxSlopeDegrees ||
                Quantize2(face[0], profile.weldTolerance) == Quantize2(face[1], profile.weldTolerance) ||
                Quantize2(face[1], profile.weldTolerance) == Quantize2(face[2], profile.weldTolerance) ||
                Quantize2(face[2], profile.weldTolerance) == Quantize2(face[0], profile.weldTolerance))
            {
                return false;
            }
        }
        return true;
    }

    /// Split a candidate boundary triangle to consume one complete authored edge.
    /// The remaining edge segments and every source/region/door join are retained.
    void AddPartitionedPortal(CandidateNavMesh &candidate, std::uint32_t polygon, std::uint8_t edge, Vec3 start,
                              Vec3 end, Vec3 targetStart, Vec3 targetEnd, std::uint32_t targetMesh,
                              std::uint32_t targetPolygon, std::uint8_t targetEdge)
    {
        auto &mesh = candidate.mesh;
        const auto original = mesh.polygons[polygon];
        const auto a = original.vertices[edge];
        const auto b = original.vertices[(edge + 1) % 3];
        const auto opposite = original.vertices[(edge + 2) % 3];
        const auto vertex = [&](Vec3 point)
        {
            const auto existing =
                std::find_if(mesh.vertices.begin(), mesh.vertices.end(), [&](Vec3 other)
                             { return point.x == other.x && point.y == other.y && point.z == other.z; });
            if (existing != mesh.vertices.end())
            {
                return static_cast<std::uint32_t>(std::distance(mesh.vertices.begin(), existing));
            }
            const auto index = static_cast<std::uint32_t>(mesh.vertices.size());
            mesh.vertices.push_back(point);
            return index;
        };
        const auto planarDistance = [](Vec3 left, Vec3 right)
        { return std::hypot(left.x - right.x, left.y - right.y); };
        // An authored endpoint inside the candidate trims the boundary fan. An
        // endpoint outside it needs a connector; both cases keep the exact seam.
        const auto aligned = [&](Vec3 point, Vec3 target)
        {
            return planarDistance(point, target) <=
                       std::max(candidate.profile.weldTolerance, AuthoredBorderTolerance) ||
                   Cross2(mesh.vertices[a], mesh.vertices[b], target) > 0;
        };
        const bool coincident = aligned(start, targetStart) && aligned(end, targetEnd);
        const auto first = vertex(aligned(start, targetStart) ? targetStart : start);
        const auto last = vertex(aligned(end, targetEnd) ? targetEnd : end);
        std::vector<std::uint32_t> faces{polygon};
        mesh.polygons[polygon].vertices = {first, last, opposite};
        const auto source = candidate.polygonSourceTriangles[polygon];
        const auto contributors = candidate.polygonContributingTriangles[polygon];
        const auto append = [&](std::array<std::uint32_t, 3> vertices)
        {
            if (Cross2(mesh.vertices[vertices[0]], mesh.vertices[vertices[1]], mesh.vertices[vertices[2]]) <= 0.01F)
            {
                return;
            }
            const auto index = static_cast<std::uint32_t>(mesh.polygons.size());
            mesh.polygons.push_back(
                {.vertices = vertices, .neighbors = {NoNeighbor, NoNeighbor, NoNeighbor}, .flags = original.flags});
            candidate.polygonSourceTriangles.push_back(source);
            candidate.polygonContributingTriangles.push_back(contributors);
            faces.push_back(index);
        };
        append({a, first, opposite});
        append({last, b, opposite});
        const auto targetFirst = vertex(targetStart);
        const auto targetLast = vertex(targetEnd);
        if (!coincident)
        {
            append({first, targetFirst, last});
            append({last, targetFirst, targetLast});
        }
        std::optional<std::pair<std::uint32_t, std::uint8_t>> portal;
        for (const auto face : faces)
        {
            for (std::uint8_t side{}; side < 3; ++side)
            {
                const auto &triangle = mesh.polygons[face];
                if (triangle.vertices[side] == targetFirst && triangle.vertices[(side + 1) % 3] == targetLast)
                {
                    portal = {face, side};
                }
            }
        }
        if (!portal)
        {
            throw std::logic_error("Border subdivision does not contain the complete authored edge");
        }
        // Splitting one side can rotate the triangle or move another portal to
        // a child. Preserve those portals by their directed endpoint identities.
        for (auto &link : candidate.borderLinks)
        {
            if (link.polygon != polygon)
            {
                continue;
            }
            const auto firstVertex = original.vertices[link.edge];
            const auto lastVertex = original.vertices[(link.edge + 1) % 3];
            bool found{};
            for (const auto child : faces)
            {
                for (std::uint8_t side{}; side < 3; ++side)
                {
                    const auto &triangle = mesh.polygons[child];
                    if (triangle.vertices[side] == firstVertex && triangle.vertices[(side + 1) % 3] == lastVertex)
                    {
                        link.polygon = child;
                        link.edge = side;
                        found = true;
                    }
                }
            }
            if (!found)
            {
                throw std::logic_error("Border subdivision would remove an existing portal");
            }
        }
        for (auto &region : candidate.regions)
        {
            if (std::find(region.polygons.begin(), region.polygons.end(), polygon) != region.polygons.end())
            {
                region.polygons.insert(region.polygons.end(), faces.begin() + 1, faces.end());
                region.reachesBorder = true;
                region.area = 0;
                for (const auto face : region.polygons)
                {
                    const auto &triangle = mesh.polygons[face];
                    region.area += Area2(mesh.vertices[triangle.vertices[0]], mesh.vertices[triangle.vertices[1]],
                                         mesh.vertices[triangle.vertices[2]]) *
                                   0.5F;
                }
                break;
            }
        }
        for (auto &exit : candidate.exits)
        {
            if (exit.polygon != polygon)
            {
                continue;
            }
            for (const auto face : faces)
            {
                const auto &triangle = mesh.polygons[face];
                float height{};
                if (HeightAt(exit.position, mesh.vertices[triangle.vertices[0]], mesh.vertices[triangle.vertices[1]],
                             mesh.vertices[triangle.vertices[2]], height))
                {
                    exit.polygon = face;
                    break;
                }
            }
        }
        candidate.borderLinks.push_back({portal->first, portal->second, targetMesh, targetPolygon, targetEdge});
        BuildAdjacency(mesh, candidate.profile.weldTolerance, candidate.profile.stepHeight);
    }

    /** Align seam endpoints through their complete incident fans.
     * Moving every coincident use preserves the interior rim's shared vertices.
     * Existing portals pin their endpoints. Inverted or nonwalkable incident
     * triangles reject the prepared copy before it can replace the candidate.
     */
    bool AlignBorderEndpoints(CandidateNavMesh &candidate, std::uint32_t polygon, std::uint8_t edge,
                              std::optional<Vec3> first, std::optional<Vec3> last)
    {
        const auto face = candidate.mesh.polygons[polygon];
        const std::array original{candidate.mesh.vertices[face.vertices[edge]],
                                  candidate.mesh.vertices[face.vertices[(edge + 1) % 3]]};
        const std::array replacements{first, last};
        const auto originalVertices = candidate.mesh.vertices;
        std::vector<bool> moved(candidate.mesh.vertices.size());
        std::set<std::uint32_t> pinned;
        for (const auto &link : candidate.borderLinks)
        {
            const auto &portal = candidate.mesh.polygons[link.polygon];
            pinned.insert(portal.vertices[link.edge]);
            pinned.insert(portal.vertices[(link.edge + 1) % 3]);
        }
        for (std::uint32_t vertex{}; vertex < candidate.mesh.vertices.size(); ++vertex)
        {
            const auto point = candidate.mesh.vertices[vertex];
            for (std::size_t end{}; end < replacements.size(); ++end)
            {
                if (!replacements[end] || Quantize(point, candidate.profile.weldTolerance) !=
                                              Quantize(original[end], candidate.profile.weldTolerance))
                {
                    continue;
                }
                const auto target = *replacements[end];
                if (pinned.contains(vertex) && (point.x != target.x || point.y != target.y || point.z != target.z))
                {
                    return false;
                }
                candidate.mesh.vertices[vertex] = target;
                moved[vertex] = true;
            }
        }
        // Recast floor triangles can include steep voxel transitions. Alignment
        // must not exceed the affected fan's existing slope envelope.
        auto maximumSlope = candidate.profile.maxSlopeDegrees;
        for (const auto &triangle : candidate.mesh.polygons)
        {
            if (std::any_of(triangle.vertices.begin(), triangle.vertices.end(),
                            [&](auto vertex) { return moved[vertex]; }))
            {
                const auto a = originalVertices[triangle.vertices[0]];
                const auto u = originalVertices[triangle.vertices[1]] - a;
                const auto v = originalVertices[triangle.vertices[2]] - a;
                maximumSlope =
                    std::max(maximumSlope, std::atan2(std::hypot(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z),
                                                      u.x * v.y - u.y * v.x) *
                                               180.0F / 3.14159265358979323846F);
            }
        }
        for (const auto &triangle : candidate.mesh.polygons)
        {
            if (std::none_of(triangle.vertices.begin(), triangle.vertices.end(),
                             [&](auto vertex) { return moved[vertex]; }))
            {
                continue;
            }
            const auto a = candidate.mesh.vertices[triangle.vertices[0]];
            const auto b = candidate.mesh.vertices[triangle.vertices[1]];
            const auto c = candidate.mesh.vertices[triangle.vertices[2]];
            const auto u = b - a;
            const auto v = c - a;
            const auto area = Cross2(a, b, c);
            if (area <= 0.01F || std::atan2(std::hypot(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z), area) * 180.0F /
                                         3.14159265358979323846F >
                                     maximumSlope + BorderSlopeRoundoff)
            {
                return false;
            }
        }
        BuildAdjacency(candidate.mesh, candidate.profile.weldTolerance, candidate.profile.stepHeight);
        return ValidateCandidateTopology(candidate).valid;
    }

    /** Match authored edge subdivisions to containing generated boundary edges.
     * Projection stays within the generated edge; distance and step limits bound
     * any extension. A matched authored edge always retains its complete endpoints.
     */
    template <class Border>
    void StitchPartitionedBorders(CandidateNavMesh &candidate, const std::vector<NavMesh> &neighbors,
                                  const Border &border, float maximumGap)
    {
        for (const auto &neighbor : neighbors)
        {
            for (std::uint32_t targetPolygon{}; targetPolygon < neighbor.polygons.size(); ++targetPolygon)
            {
                const auto &target = neighbor.polygons[targetPolygon];
                for (std::uint8_t targetEdge{}; targetEdge < 3; ++targetEdge)
                {
                    if (target.vertices[targetEdge] >= neighbor.vertices.size() ||
                        target.vertices[(targetEdge + 1) % 3] >= neighbor.vertices.size() ||
                        (!(target.flags & (1U << targetEdge)) && target.neighbors[targetEdge] != NoNeighbor &&
                         target.neighbors[targetEdge] != 0xffffU) ||
                        std::any_of(candidate.borderLinks.begin(), candidate.borderLinks.end(),
                                    [&](const auto &link)
                                    {
                                        return link.neighborNavmeshId == neighbor.id &&
                                               link.neighborPolygon == targetPolygon && link.neighborEdge == targetEdge;
                                    }))
                    {
                        continue;
                    }
                    const auto c = neighbor.vertices[target.vertices[(targetEdge + 1) % 3]];
                    const auto d = neighbor.vertices[target.vertices[targetEdge]];
                    const auto cellSide = border(c, d, AuthoredBorderTolerance);
                    if (cellSide < 0)
                    {
                        continue;
                    }
                    struct Match
                    {
                        std::uint32_t polygon{};
                        std::uint8_t edge{};
                        Vec3 start, end;
                        float score{};
                        std::optional<CandidateNavMesh> prepared;
                    };
                    std::optional<Match> best;
                    for (std::uint32_t polygon{}; polygon < candidate.mesh.polygons.size(); ++polygon)
                    {
                        const auto &face = candidate.mesh.polygons[polygon];
                        for (std::uint8_t edge{}; edge < 3; ++edge)
                        {
                            if (face.neighbors[edge] != NoNeighbor ||
                                std::any_of(candidate.borderLinks.begin(), candidate.borderLinks.end(),
                                            [&](const auto &link)
                                            { return link.polygon == polygon && link.edge == edge; }))
                            {
                                continue;
                            }
                            const auto a = candidate.mesh.vertices[face.vertices[edge]];
                            const auto b = candidate.mesh.vertices[face.vertices[(edge + 1) % 3]];
                            if (border(a, b, maximumGap) != cellSide)
                            {
                                continue;
                            }
                            const auto dx = b.x - a.x;
                            const auto dy = b.y - a.y;
                            const auto lengthSquared = dx * dx + dy * dy;
                            if (lengthSquared <= 0.01F)
                            {
                                continue;
                            }
                            const auto start = ((c.x - a.x) * dx + (c.y - a.y) * dy) / lengthSquared;
                            const auto end = ((d.x - a.x) * dx + (d.y - a.y) * dy) / lengthSquared;
                            if (end <= start || end <= 0 || start >= 1)
                            {
                                continue;
                            }
                            auto p = Interpolate(a, b, std::clamp(start, 0.0F, 1.0F));
                            auto q = Interpolate(a, b, std::clamp(end, 0.0F, 1.0F));
                            const auto gapA = std::hypot(p.x - c.x, p.y - c.y);
                            const auto gapB = std::hypot(q.x - d.x, q.y - d.y);
                            if (gapA > maximumGap || gapB > maximumGap ||
                                std::abs(p.z - c.z) > candidate.profile.stepHeight ||
                                std::abs(q.z - d.z) > candidate.profile.stepHeight)
                            {
                                continue;
                            }
                            const auto score = gapA + gapB + std::abs(p.z - c.z) + std::abs(q.z - d.z);
                            if (best && score >= best->score)
                            {
                                continue;
                            }
                            std::optional<CandidateNavMesh> prepared;
                            auto preparedA = a;
                            auto preparedB = b;
                            if (start < 0 || end > 1)
                            {
                                prepared = candidate;
                                if (!AlignBorderEndpoints(*prepared, polygon, edge,
                                                          start < 0 ? std::optional(c) : std::nullopt,
                                                          end > 1 ? std::optional(d) : std::nullopt))
                                {
                                    continue;
                                }
                                preparedA = prepared->mesh.vertices[face.vertices[edge]];
                                preparedB = prepared->mesh.vertices[face.vertices[(edge + 1) % 3]];
                                if (start < 0)
                                {
                                    p = c;
                                }
                                if (end > 1)
                                {
                                    q = d;
                                }
                            }
                            const auto &mesh = prepared ? prepared->mesh : candidate.mesh;
                            if (!CanSubdivideBorder(preparedA, preparedB, mesh.vertices[face.vertices[(edge + 2) % 3]],
                                                    p, q, c, d, candidate.profile))
                            {
                                continue;
                            }
                            best = {polygon, edge, p, q, score, std::move(prepared)};
                        }
                    }
                    if (best)
                    {
                        if (best->prepared)
                        {
                            candidate = std::move(*best->prepared);
                        }
                        AddPartitionedPortal(candidate, best->polygon, best->edge, best->start, best->end, c, d,
                                             neighbor.id, targetPolygon, targetEdge);
                    }
                }
            }
        }
    }

    /** Find a triangulation retaining every rim vertex and the constrained portal edge.
     * Dynamic programming rejects inverted, welded or excessive-slope triangles
     * and considers every diagonal instead of committing to a greedy ear.
     */
    std::vector<std::array<std::uint32_t, 3>> TriangulateBorderOutline(
        const std::vector<std::uint32_t> &outline, const NavMesh &mesh, const NavigationProfile &profile,
        std::optional<std::uint32_t> interior = std::nullopt)
    {
        std::vector<std::array<std::uint32_t, 3>> replacements;
        const auto validTriangle = [&](std::array<std::uint32_t, 3> face)
        {
            const auto a = mesh.vertices[face[0]];
            const auto b = mesh.vertices[face[1]];
            const auto opposite = mesh.vertices[face[2]];
            const auto area = Cross2(a, b, opposite);
            if (area <= 0.01F || !CanSubdivideBorder(a, b, opposite, a, b, a, b, profile) ||
                std::any_of(outline.begin(), outline.end(),
                            [&](auto other)
                            {
                                if (std::find(face.begin(), face.end(), other) != face.end())
                                {
                                    return false;
                                }
                                const auto p = mesh.vertices[other];
                                return Cross2(a, b, p) >= -0.01F && Cross2(b, opposite, p) >= -0.01F &&
                                       Cross2(opposite, a, p) >= -0.01F;
                            }))
            {
                return false;
            }
            if (interior && std::find(face.begin(), face.end(), *interior) == face.end())
            {
                const auto point = mesh.vertices[*interior];
                if (Cross2(a, b, point) >= -0.01F && Cross2(b, opposite, point) >= -0.01F &&
                    Cross2(opposite, a, point) >= -0.01F)
                {
                    return false;
                }
            }
            return true;
        };
        // Dynamic programming considers alternative diagonals: a large
        // early ear can leave a steep final triangle beside the portal.
        std::vector<std::vector<int>> split(outline.size(), std::vector<int>(outline.size(), -1));
        for (std::size_t i{}; i + 1 < outline.size(); ++i)
        {
            split[i][i + 1] = 0;
        }
        for (std::size_t span = 2; span < outline.size(); ++span)
        {
            for (std::size_t i{}; i + span < outline.size(); ++i)
            {
                const auto j = i + span;
                for (auto k = i + 1; k < j; ++k)
                {
                    if (split[i][k] >= 0 && split[k][j] >= 0 && validTriangle({outline[i], outline[k], outline[j]}))
                    {
                        split[i][j] = static_cast<int>(k);
                        break;
                    }
                }
            }
        }
        const auto emit = [&](auto &&self, std::size_t i, std::size_t j) -> void
        {
            if (j <= i + 1)
            {
                return;
            }
            const auto k = static_cast<std::size_t>(split[i][j]);
            replacements.push_back({outline[i], outline[k], outline[j]});
            self(self, i, k);
            self(self, k, j);
        };
        if (interior && outline.size() >= 3)
        {
            // Keep one interior height sample. A fan may skip a convex rim run
            // only when the ordinary triangulation fills that run without
            // covering the sample. All faces retain the original directed rim.
            std::vector<std::size_t> previous(outline.size() + 1, outline.size() + 1);
            previous[0] = 0;
            for (std::size_t end = 1; end <= outline.size(); ++end)
            {
                for (std::size_t begin{}; begin < end; ++begin)
                {
                    if (previous[begin] > outline.size() || (end < outline.size() && split[begin][end] < 0) ||
                        (end == outline.size() && begin + 1 != end) ||
                        !validTriangle({outline[begin], outline[end % outline.size()], *interior}))
                    {
                        continue;
                    }
                    previous[end] = begin;
                    break;
                }
            }
            if (previous.back() <= outline.size())
            {
                auto end = outline.size();
                while (end != 0)
                {
                    const auto begin = previous[end];
                    replacements.push_back({outline[begin], outline[end % outline.size()], *interior});
                    if (end < outline.size())
                    {
                        emit(emit, begin, end);
                    }
                    end = begin;
                }
            }
        }
        else if (outline.size() >= 3 && split[0].back() >= 0)
        {
            emit(emit, 0, outline.size() - 1);
        }
        return replacements;
    }

    /** Replace a cavity atomically, preserving portals, door anchors and source joins.
     * Removed faces contribute to every replacement; unmodified faces retain their
     * evidence. Invalid remaps or topology leave the caller's mesh untouched.
     */
    bool CommitBorderCavity(CandidateNavMesh &candidate, CandidateNavMesh revised,
                            const std::set<std::uint32_t> &selected,
                            const std::vector<std::array<std::uint32_t, 3>> &replacements, std::uint32_t targetFirst,
                            std::uint32_t targetLast, const NavMesh &neighbor, std::uint32_t targetPolygon,
                            std::uint8_t targetEdge)
    {
        const auto &mesh = candidate.mesh;
        const auto flags = mesh.polygons[*selected.begin()].flags;
        if (std::any_of(selected.begin(), selected.end(),
                        [&](auto polygon) { return mesh.polygons[polygon].flags != flags; }))
        {
            return false;
        }
        std::vector<std::size_t> contributors;
        std::vector<std::uint32_t> remap(mesh.polygons.size(), NoNeighbor);
        revised.mesh.polygons.clear();
        revised.polygonSourceTriangles.clear();
        revised.polygonContributingTriangles.clear();
        for (std::uint32_t polygon{}; polygon < mesh.polygons.size(); ++polygon)
        {
            if (selected.contains(polygon))
            {
                const auto &sources = candidate.polygonContributingTriangles[polygon];
                contributors.insert(contributors.end(), sources.begin(), sources.end());
                continue;
            }
            remap[polygon] = static_cast<std::uint32_t>(revised.mesh.polygons.size());
            revised.mesh.polygons.push_back(mesh.polygons[polygon]);
            revised.polygonSourceTriangles.push_back(candidate.polygonSourceTriangles[polygon]);
            revised.polygonContributingTriangles.push_back(candidate.polygonContributingTriangles[polygon]);
        }
        std::sort(contributors.begin(), contributors.end());
        contributors.erase(std::unique(contributors.begin(), contributors.end()), contributors.end());
        if (contributors.empty())
        {
            return false;
        }
        const auto begin = static_cast<std::uint32_t>(revised.mesh.polygons.size());
        for (const auto &face : replacements)
        {
            revised.mesh.polygons.push_back(
                {.vertices = face, .neighbors = {NoNeighbor, NoNeighbor, NoNeighbor}, .flags = flags});
            revised.polygonSourceTriangles.push_back(contributors.front());
            revised.polygonContributingTriangles.push_back(contributors);
        }
        bool preserved = true;
        for (auto &link : revised.borderLinks)
        {
            const auto source = mesh.polygons[link.polygon];
            const auto a = source.vertices[link.edge];
            const auto b = source.vertices[(link.edge + 1) % 3];
            if (!selected.contains(link.polygon))
            {
                link.polygon = remap[link.polygon];
                continue;
            }
            bool found{};
            for (std::uint32_t p = begin; p < revised.mesh.polygons.size(); ++p)
            {
                for (std::uint8_t e{}; e < 3; ++e)
                {
                    const auto &face = revised.mesh.polygons[p];
                    if (face.vertices[e] == a && face.vertices[(e + 1) % 3] == b)
                    {
                        link.polygon = p;
                        link.edge = e;
                        found = true;
                    }
                }
            }
            preserved &= found;
        }
        bool assigned{};
        for (auto &region : revised.regions)
        {
            const auto touched = std::any_of(region.polygons.begin(), region.polygons.end(),
                                             [&](auto p) { return selected.contains(p); });
            std::erase_if(region.polygons, [&](auto p) { return selected.contains(p); });
            for (auto &p : region.polygons)
            {
                p = remap[p];
            }
            if (touched)
            {
                preserved &= !assigned;
                assigned = true;
                for (std::uint32_t p = begin; p < revised.mesh.polygons.size(); ++p)
                {
                    region.polygons.push_back(p);
                }
            }
        }
        for (auto &exit : revised.exits)
        {
            if (!exit.polygon)
            {
                continue;
            }
            if (!selected.contains(*exit.polygon))
            {
                exit.polygon = remap[*exit.polygon];
                continue;
            }
            exit.polygon.reset();
            for (std::uint32_t p = begin; p < revised.mesh.polygons.size(); ++p)
            {
                const auto &face = revised.mesh.polygons[p];
                float height{};
                if (HeightAt(exit.position, revised.mesh.vertices[face.vertices[0]],
                             revised.mesh.vertices[face.vertices[1]], revised.mesh.vertices[face.vertices[2]], height))
                {
                    exit.polygon = p;
                    break;
                }
            }
            preserved &= exit.polygon.has_value();
        }
        bool portal{};
        for (std::uint32_t p = begin; p < revised.mesh.polygons.size(); ++p)
        {
            for (std::uint8_t e{}; e < 3; ++e)
            {
                const auto &face = revised.mesh.polygons[p];
                if (face.vertices[e] == targetFirst && face.vertices[(e + 1) % 3] == targetLast)
                {
                    revised.borderLinks.push_back({p, e, neighbor.id, targetPolygon, targetEdge});
                    portal = true;
                }
            }
        }
        revised.contours.clear();
        BuildAdjacency(revised.mesh, candidate.profile.weldTolerance, candidate.profile.stepHeight);
        CompactBorderVertices(revised);
        const auto validation = ValidateCandidateTopology(revised);
        if (preserved && portal && validation.valid)
        {
            candidate = std::move(revised);
            return true;
        }
        return false;
    }

    /// Required crossing repair bounds, in Skyrim world units, derived from its authored floor.
    struct BorderCavityLimits
    {
        float depth;
        float heightTolerance;
    };

    /** Resolve selected-cell exterior links through their reciprocal neighboring edges.
     * A geometric edge match supplies the target only when the return entry is absent.
     * The authored triangle bounds repair depth and accommodates endpoint height drift.
     */
    template <class Border>
    auto ResolveAuthoredBorderPortals(const std::vector<NavMesh> &authored, const std::vector<NavMesh> &neighbors,
                                      const Border &border, const NavigationProfile &profile, float maximumGap)
    {
        const auto distance = [](Vec3 a, Vec3 b) { return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z); };
        // Reciprocal identities disambiguate stacked floors and short corners.
        std::map<std::tuple<std::uint32_t, std::uint32_t, std::uint8_t>, BorderCavityLimits> required;
        for (const auto &source : authored)
        {
            for (const auto &link : source.externalLinks)
            {
                const auto neighbor = std::find_if(neighbors.begin(), neighbors.end(),
                                                   [&](const auto &mesh) { return mesh.id == link.navmeshId; });
                if (neighbor == neighbors.end() || link.targetPolygon >= neighbor->polygons.size() ||
                    link.polygon >= source.polygons.size() || link.edge >= 3)
                {
                    continue;
                }
                const auto &face = source.polygons[link.polygon];
                const auto a = source.vertices.at(face.vertices[link.edge]);
                const auto b = source.vertices.at(face.vertices[(link.edge + 1) % 3]);
                const auto side = border(a, b, maximumGap);
                std::optional<std::uint8_t> targetEdge;
                for (const auto &incoming : neighbor->externalLinks)
                {
                    if (incoming.polygon == link.targetPolygon && incoming.navmeshId == source.id &&
                        incoming.targetPolygon == link.polygon && incoming.edge < 3)
                    {
                        targetEdge = incoming.edge;
                        break;
                    }
                }
                const auto &target = neighbor->polygons[link.targetPolygon];
                float best = std::numeric_limits<float>::max();
                if (!targetEdge)
                {
                    for (std::uint8_t edge{}; edge < 3; ++edge)
                    {
                        const auto c = neighbor->vertices.at(target.vertices[(edge + 1) % 3]);
                        const auto d = neighbor->vertices.at(target.vertices[edge]);
                        const auto score = distance(a, c) + distance(b, d);
                        if (border(c, d, AuthoredBorderTolerance) == side && score < best)
                        {
                            targetEdge = edge;
                            best = score;
                        }
                    }
                }
                if (targetEdge && side >= 0 &&
                    border(neighbor->vertices.at(target.vertices[*targetEdge]),
                           neighbor->vertices.at(target.vertices[(*targetEdge + 1) % 3]),
                           AuthoredBorderTolerance) == side)
                {
                    const auto c = neighbor->vertices.at(target.vertices[(*targetEdge + 1) % 3]);
                    const auto d = neighbor->vertices.at(target.vertices[*targetEdge]);
                    const auto opposite = source.vertices.at(face.vertices[(link.edge + 2) % 3]);
                    const auto length = std::hypot(d.x - c.x, d.y - c.y);
                    const auto depth = length > 0 ? std::abs(Cross2(c, d, opposite)) / length : maximumGap;
                    const auto heights = profile.stepHeight + std::max(std::abs(a.z - c.z), std::abs(b.z - d.z));
                    required.emplace(std::tuple{neighbor->id, link.targetPolygon, *targetEdge},
                                     BorderCavityLimits{std::max(maximumGap, depth), heights});
                }
            }
        }
        return required;
    }

    /** Prepare near-coincident portal endpoints through their complete incident fans.
     * Distances and heights are in Skyrim world units. Only vertices on the
     * target CELL side qualify; existing portals and invalid deformations stay fixed.
     */
    template <class Border>
    void AlignCavityEndpoints(CandidateNavMesh &candidate, Vec3 first, Vec3 last, int cellSide, float heightTolerance,
                              const Border &border)
    {
        for (const auto endpoint : {first, last})
        {
            std::set<std::uint32_t> visited;
            for (std::uint32_t polygon{}; polygon < candidate.mesh.polygons.size(); ++polygon)
            {
                const auto face = candidate.mesh.polygons[polygon];
                for (std::uint8_t edge{}; edge < 3; ++edge)
                {
                    const auto index = face.vertices[edge];
                    const auto point = candidate.mesh.vertices[index];
                    if (!visited.insert(index).second ||
                        (point.x == endpoint.x && point.y == endpoint.y && point.z == endpoint.z) ||
                        border(point, point, AuthoredBorderTolerance) != cellSide ||
                        std::hypot(point.x - endpoint.x, point.y - endpoint.y) > AuthoredBorderTolerance ||
                        std::abs(point.z - endpoint.z) > heightTolerance)
                    {
                        continue;
                    }
                    auto aligned = candidate;
                    if (AlignBorderEndpoints(aligned, polygon, edge, endpoint, std::nullopt))
                    {
                        candidate = std::move(aligned);
                    }
                }
            }
        }
    }

    /** Preserve a removed seam height sample inside a repaired cavity.
     * Try bounded moves toward incident floor centroids in Skyrim world space.
     * Successful triangulation adds one vertex to revised; failure leaves it unchanged.
     * The complete portal and immutable rim stay intact under the cavity slope envelope.
     */
    template <class Border>
    std::vector<std::array<std::uint32_t, 3>> TriangulateCavityWithHeightSample(
        NavMesh &revised, const NavMesh &original, const std::vector<std::uint32_t> &boundary,
        const std::vector<std::uint32_t> &outline, const std::set<std::uint32_t> &selected,
        const NavigationProfile &profile, int cellSide, float maximumGap, const Border &border)
    {
        for (const auto sample : boundary)
        {
            const auto point = original.vertices[sample];
            if (std::find(outline.begin(), outline.end(), sample) != outline.end() ||
                border(point, point, maximumGap) != cellSide)
            {
                continue;
            }
            for (const auto polygon : selected)
            {
                const auto &face = original.polygons[polygon];
                if (std::find(face.vertices.begin(), face.vertices.end(), sample) == face.vertices.end())
                {
                    continue;
                }
                const auto center = (original.vertices[face.vertices[0]] + original.vertices[face.vertices[1]] +
                                     original.vertices[face.vertices[2]]) /
                                    3.0F;
                const auto distance = std::hypot(center.x - point.x, center.y - point.y);
                const auto inset = std::max(profile.agentRadius, profile.weldTolerance * 4.0F);
                const auto fraction = distance > inset ? std::min(0.5F, inset / distance) : 0.5F;
                const auto position = Interpolate(point, center, fraction);
                if (border(position, position, profile.weldTolerance) >= 0)
                {
                    continue;
                }
                const auto interior = static_cast<std::uint32_t>(revised.vertices.size());
                revised.vertices.push_back(position);
                auto replacements = TriangulateBorderOutline(outline, revised, profile, interior);
                if (!replacements.empty())
                {
                    return replacements;
                }
                revised.vertices.pop_back();
            }
        }
        return {};
    }

    /** Retriangulate a connected boundary cavity around a complete authored portal.
     * The exterior chain is replaced by the exact portal; the cavity's interior
     * rim and other portals remain fixed. Enlarging through adjacent triangles
     * gives height adjustments room without deforming the surrounding floor.
     * All changes are transactional, including source, region and door joins.
     */
    template <class Border>
    bool RetriangulateBorderPortal(CandidateNavMesh &candidate, const NavMesh &neighbor, std::uint32_t targetPolygon,
                                   std::uint8_t targetEdge, const Border &border, float maximumGap, float cavityDepth,
                                   float heightTolerance)
    {
        const auto &target = neighbor.polygons[targetPolygon];
        const auto c = neighbor.vertices[target.vertices[(targetEdge + 1) % 3]];
        const auto d = neighbor.vertices[target.vertices[targetEdge]];
        const auto cellSide = border(c, d, AuthoredBorderTolerance);
        const auto dx = d.x - c.x;
        const auto dy = d.y - c.y;
        const auto lengthSquared = dx * dx + dy * dy;
        if (cellSide < 0 || lengthSquared <= 0.01F)
        {
            return false;
        }
        const auto projection = [&](Vec3 p) { return ((p.x - c.x) * dx + (p.y - c.y) * dy) / lengthSquared; };
        auto prepared = candidate;
        AlignCavityEndpoints(prepared, c, d, cellSide, heightTolerance, border);
        const auto &mesh = prepared.mesh;
        std::set<std::uint32_t> selected;
        for (std::uint32_t polygon{}; polygon < mesh.polygons.size(); ++polygon)
        {
            const auto &face = mesh.polygons[polygon];
            for (std::uint8_t edge{}; edge < 3; ++edge)
            {
                if (face.neighbors[edge] != NoNeighbor)
                {
                    continue;
                }
                const auto a = mesh.vertices[face.vertices[edge]];
                const auto b = mesh.vertices[face.vertices[(edge + 1) % 3]];
                const auto start = projection(a);
                const auto end = projection(b);
                if (border(a, b, maximumGap) != cellSide || end <= start || end <= 0 || start >= 1)
                {
                    continue;
                }
                const auto p = Interpolate(a, b, std::clamp(-start / (end - start), 0.0F, 1.0F));
                const auto q = Interpolate(a, b, std::clamp((1 - start) / (end - start), 0.0F, 1.0F));
                if (std::abs(p.z - Interpolate(c, d, std::clamp(projection(p), 0.0F, 1.0F)).z) <= heightTolerance &&
                    std::abs(q.z - Interpolate(c, d, std::clamp(projection(q), 0.0F, 1.0F)).z) <= heightTolerance)
                {
                    selected.insert(polygon);
                }
            }
        }
        if (selected.empty())
        {
            return false;
        }
        // Boundary vertices removed from the chain must have their entire fan
        // inside the cavity. Otherwise the new edge would leave a T-junction.
        std::set<std::uint32_t> seamVertices;
        for (const auto polygon : selected)
        {
            const auto &face = mesh.polygons[polygon];
            for (std::uint8_t edge{}; edge < 3; ++edge)
            {
                if (face.neighbors[edge] != NoNeighbor ||
                    border(mesh.vertices[face.vertices[edge]], mesh.vertices[face.vertices[(edge + 1) % 3]],
                           maximumGap) != cellSide)
                {
                    continue;
                }
                for (const auto vertex : {face.vertices[edge], face.vertices[(edge + 1) % 3]})
                {
                    const auto point = mesh.vertices[vertex];
                    if (border(point, point, maximumGap) == cellSide && projection(point) >= 0 &&
                        projection(point) <= 1)
                    {
                        seamVertices.insert(vertex);
                    }
                }
            }
        }
        for (std::uint32_t polygon{}; polygon < mesh.polygons.size(); ++polygon)
        {
            if (std::any_of(mesh.polygons[polygon].vertices.begin(), mesh.polygons[polygon].vertices.end(),
                            [&](auto vertex) { return seamVertices.contains(vertex); }))
            {
                selected.insert(polygon);
            }
        }
        for (;;)
        {
            // Immutable rim edges can already contain steep voxel transitions;
            // replacements must remain within this cavity's slope envelope.
            auto cavityProfile = candidate.profile;
            for (const auto polygon : selected)
            {
                const auto &face = mesh.polygons[polygon];
                const auto a = mesh.vertices[face.vertices[0]];
                const auto u = mesh.vertices[face.vertices[1]] - a;
                const auto v = mesh.vertices[face.vertices[2]] - a;
                const auto slope =
                    std::atan2(std::hypot(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z), u.x * v.y - u.y * v.x) *
                    180.0F / 3.14159265358979323846F;
                cavityProfile.maxSlopeDegrees = std::max(cavityProfile.maxSlopeDegrees, slope);
            }
            std::map<std::uint32_t, std::uint32_t> rim;
            bool simple = true;
            for (const auto polygon : selected)
            {
                const auto &face = mesh.polygons[polygon];
                for (std::size_t edge{}; edge < 3; ++edge)
                {
                    if (!selected.contains(face.neighbors[edge]) &&
                        !rim.emplace(face.vertices[edge], face.vertices[(edge + 1) % 3]).second)
                    {
                        simple = false;
                    }
                }
            }
            std::vector<std::uint32_t> boundary;
            if (simple && !rim.empty())
            {
                auto vertex = rim.begin()->first;
                do
                {
                    boundary.push_back(vertex);
                    const auto next = rim.find(vertex);
                    if (next == rim.end())
                    {
                        break;
                    }
                    vertex = next->second;
                } while (vertex != boundary.front() && boundary.size() <= rim.size());
                simple = vertex == boundary.front() && boundary.size() == rim.size();
            }
            std::optional<std::size_t> first, last;
            if (simple)
            {
                for (std::size_t i{}; i < boundary.size(); ++i)
                {
                    const auto a = mesh.vertices[boundary[i]];
                    const auto b = mesh.vertices[boundary[(i + 1) % boundary.size()]];
                    if (border(a, b, maximumGap) == cellSide && projection(b) > projection(a))
                    {
                        if (projection(a) <= 0 && projection(b) > 0)
                        {
                            first = i;
                        }
                        if (projection(a) < 1 && projection(b) >= 1)
                        {
                            last = i;
                        }
                    }
                }
            }
            if (first && last)
            {
                auto revised = prepared;
                const auto vertex = [&](Vec3 point)
                {
                    for (std::uint32_t i{}; i < revised.mesh.vertices.size(); ++i)
                    {
                        const auto other = revised.mesh.vertices[i];
                        if (point.x == other.x && point.y == other.y && point.z == other.z)
                        {
                            return i;
                        }
                    }
                    revised.mesh.vertices.push_back(point);
                    return static_cast<std::uint32_t>(revised.mesh.vertices.size() - 1);
                };
                const auto targetFirst = vertex(c);
                const auto targetLast = vertex(d);
                std::vector<std::uint32_t> outline{boundary[*first], targetFirst, targetLast};
                auto cursor = (*last + 1) % boundary.size();
                while (cursor != *first && outline.size() <= boundary.size() + 3)
                {
                    outline.push_back(boundary[cursor]);
                    cursor = (cursor + 1) % boundary.size();
                }
                // Coincident endpoints are represented by the authored vertex;
                // every other rim vertex keeps its identity for shared edges.
                outline.erase(std::unique(outline.begin(), outline.end()), outline.end());
                auto replacements = TriangulateBorderOutline(outline, revised.mesh, cavityProfile);
                if (replacements.empty())
                {
                    replacements = TriangulateCavityWithHeightSample(revised.mesh, mesh, boundary, outline, selected,
                                                                     cavityProfile, cellSide, maximumGap, border);
                }
                if (!replacements.empty() &&
                    CommitBorderCavity(candidate, std::move(revised), selected, replacements, targetFirst, targetLast,
                                       neighbor, targetPolygon, targetEdge))
                {
                    return true;
                }
            }
            auto expanded = selected;
            // Long floor triangles can reach the repair band even when their
            // centroids lie outside it. Keep those incident rim choices available.
            for (const auto polygon : selected)
            {
                for (const auto next : mesh.polygons[polygon].neighbors)
                {
                    if (next == NoNeighbor)
                    {
                        continue;
                    }
                    const auto &face = mesh.polygons[next];
                    if (std::any_of(face.vertices.begin(), face.vertices.end(),
                                    [&](auto vertex)
                                    {
                                        const auto point = mesh.vertices[vertex];
                                        const auto projected =
                                            Interpolate(c, d, std::clamp(projection(point), 0.0F, 1.0F));
                                        return std::hypot(point.x - projected.x, point.y - projected.y) <= cavityDepth;
                                    }))
                    {
                        expanded.insert(next);
                    }
                }
            }
            if (expanded.size() == selected.size())
            {
                return false;
            }
            selected = std::move(expanded);
        }
    }

    /// Squared distance to a triangle in Skyrim world space, including its interpolated floor height.
    float DoorDistanceSquared(Vec3 position, const NavMesh &mesh, const NavPolygon &face)
    {
        const auto a = mesh.vertices[face.vertices[0]];
        const auto b = mesh.vertices[face.vertices[1]];
        const auto c = mesh.vertices[face.vertices[2]];
        float height{};
        if (HeightAt(position, a, b, c, height))
        {
            return (height - position.z) * (height - position.z);
        }
        auto best = std::numeric_limits<float>::max();
        for (std::size_t edge{}; edge < 3; ++edge)
        {
            const auto first = mesh.vertices[face.vertices[edge]];
            const auto direction = mesh.vertices[face.vertices[(edge + 1) % 3]] - first;
            const auto offset = position - first;
            const auto lengthSquared =
                direction.x * direction.x + direction.y * direction.y + direction.z * direction.z;
            const auto fraction = std::clamp(
                (offset.x * direction.x + offset.y * direction.y + offset.z * direction.z) / lengthSquared, 0.0F, 1.0F);
            const auto distance = offset - direction * fraction;
            best = std::min(best, distance.x * distance.x + distance.y * distance.y + distance.z * distance.z);
        }
        return best;
    }

    /// Place the trimming fan inside the original floor plane, near its unmatched seam or corner.
    Vec3 BorderFanCenter(const NavMesh &mesh, const NavPolygon &face, const std::array<bool, 3> &unmatched,
                         const NavigationProfile &profile)
    {
        const auto center =
            (mesh.vertices[face.vertices[0]] + mesh.vertices[face.vertices[1]] + mesh.vertices[face.vertices[2]]) /
            3.0F;
        auto seam = center;
        if (std::count(unmatched.begin(), unmatched.end(), true) == 1)
        {
            const auto edge =
                static_cast<std::size_t>(std::find(unmatched.begin(), unmatched.end(), true) - unmatched.begin());
            seam = (mesh.vertices[face.vertices[edge]] + mesh.vertices[face.vertices[(edge + 1) % 3]]) / 2.0F;
        }
        else
        {
            for (std::size_t vertex{}; vertex < 3; ++vertex)
            {
                if (unmatched[vertex] && unmatched[(vertex + 2) % 3])
                {
                    seam = mesh.vertices[face.vertices[vertex]];
                    break;
                }
            }
        }
        const auto distance = std::hypot(center.x - seam.x, center.y - seam.y);
        const auto inset = std::max(profile.agentRadius, profile.weldTolerance * 4.0F);
        return distance > inset ? Interpolate(seam, center, inset / distance) : center;
    }

    /** Retract unmatched CELL seams while preserving every shared interior edge.
     * An interior fan keeps the original floor plane and exact matched portal
     * endpoints. Only the wedges facing an unlinked seam are omitted; adjacent
     * triangles keep their shared edges, so no T-junctions are introduced.
     */
    template <class Border> void RetractUnlinkedBorders(CandidateNavMesh &candidate, const Border &border)
    {
        const auto original = candidate.mesh.polygons;
        std::vector<std::vector<std::uint32_t>> children(original.size());
        std::vector<std::array<std::uint32_t, 3>> edgeChildren(original.size());
        std::vector<NavPolygon> polygons;
        std::vector<std::size_t> sources;
        std::vector<std::vector<std::size_t>> contributors;
        std::size_t retracted{};
        std::set<std::pair<std::uint32_t, std::uint8_t>> linkedEdges;
        for (const auto &link : candidate.borderLinks)
        {
            linkedEdges.emplace(link.polygon, link.edge);
        }
        for (std::uint32_t index{}; index < original.size(); ++index)
        {
            const auto &face = original[index];
            std::array<bool, 3> unmatched{};
            for (std::uint8_t edge{}; edge < 3; ++edge)
            {
                unmatched[edge] = face.neighbors[edge] == NoNeighbor &&
                                  border(candidate.mesh.vertices[face.vertices[edge]],
                                         candidate.mesh.vertices[face.vertices[(edge + 1) % 3]],
                                         candidate.profile.weldTolerance) >= 0 &&
                                  !linkedEdges.contains({index, edge});
            }
            edgeChildren[index].fill(NoNeighbor);
            const bool split = std::any_of(unmatched.begin(), unmatched.end(), [](bool value) { return value; });
            auto center = NoNeighbor;
            if (split)
            {
                center = static_cast<std::uint32_t>(candidate.mesh.vertices.size());
                candidate.mesh.vertices.push_back(BorderFanCenter(candidate.mesh, face, unmatched, candidate.profile));
            }
            for (std::uint8_t edge{}; edge < (split ? 3 : 1); ++edge)
            {
                if (unmatched[edge])
                {
                    ++retracted;
                    continue;
                }
                const auto child = static_cast<std::uint32_t>(polygons.size());
                children[index].push_back(child);
                auto replacement = face;
                if (split)
                {
                    replacement.vertices = {face.vertices[edge], face.vertices[(edge + 1) % 3], center};
                    edgeChildren[index][edge] = child;
                }
                else
                {
                    edgeChildren[index].fill(child);
                }
                polygons.push_back(replacement);
                sources.push_back(candidate.polygonSourceTriangles[index]);
                contributors.push_back(candidate.polygonContributingTriangles[index]);
            }
        }
        if (!retracted)
        {
            return;
        }
        candidate.mesh.polygons = std::move(polygons);
        candidate.polygonSourceTriangles = std::move(sources);
        candidate.polygonContributingTriangles = std::move(contributors);
        for (auto &link : candidate.borderLinks)
        {
            const auto polygon = link.polygon;
            link.polygon = edgeChildren[polygon][link.edge];
            if (children[polygon].size() != 1 ||
                original[polygon].vertices != candidate.mesh.polygons[link.polygon].vertices)
            {
                link.edge = 0;
            }
        }
        for (auto &door : candidate.exits)
        {
            if (!door.polygon)
            {
                continue;
            }
            const auto &choices = children[*door.polygon];
            door.polygon.reset();
            auto best = std::numeric_limits<float>::max();
            for (const auto child : choices)
            {
                const auto distance =
                    DoorDistanceSquared(door.position, candidate.mesh, candidate.mesh.polygons[child]);
                if (distance < best)
                {
                    best = distance;
                    door.polygon = child;
                }
            }
            if (!door.polygon)
            {
                door.region.reset();
            }
        }
        for (auto &region : candidate.regions)
        {
            std::vector<std::uint32_t> revised;
            for (const auto polygon : region.polygons)
            {
                revised.insert(revised.end(), children[polygon].begin(), children[polygon].end());
            }
            region.polygons = std::move(revised);
        }
        candidate.warnings.push_back(
            std::format("Retracted {} unmatched exterior seam edges into the selected CELL.", retracted));
        BuildAdjacency(candidate.mesh, candidate.profile.weldTolerance, candidate.profile.stepHeight);
    }

    /** Retain shared-edge components with real portals or matched doors.
     * Stitching can join regions and seam retraction can split them. Flood the
     * final adjacency rather than trusting provisional region anchors, then
     * compact polygon/source indices and rebuild region/door membership.
     */
    void RetainConnectedBorderRegions(CandidateNavMesh &candidate)
    {
        const auto count = candidate.mesh.polygons.size();
        std::vector<std::uint32_t> originalRegion(count, NoNeighbor);
        for (std::uint32_t index{}; index < candidate.regions.size(); ++index)
        {
            const auto &region = candidate.regions[index];
            for (const auto polygon : region.polygons)
            {
                originalRegion[polygon] = index;
            }
        }
        std::vector<bool> anchored(count);
        for (const auto &link : candidate.borderLinks)
        {
            anchored[link.polygon] = true;
        }
        for (const auto &door : candidate.exits)
        {
            if (door.polygon)
            {
                anchored[*door.polygon] = true;
            }
        }
        std::vector<std::uint32_t> remap(count, NoNeighbor);
        std::vector<bool> seen(count);
        std::vector<CandidateRegion> regions;
        for (std::uint32_t start{}; start < count; ++start)
        {
            if (seen[start])
            {
                continue;
            }
            CandidateRegion region;
            std::set<std::uint32_t> sourceRegions;
            bool connected{};
            std::queue<std::uint32_t> pending;
            pending.push(start);
            seen[start] = true;
            while (!pending.empty())
            {
                const auto polygon = pending.front();
                pending.pop();
                region.polygons.push_back(polygon);
                connected = connected || anchored[polygon];
                const auto &face = candidate.mesh.polygons[polygon];
                region.area +=
                    Area2(candidate.mesh.vertices[face.vertices[0]], candidate.mesh.vertices[face.vertices[1]],
                          candidate.mesh.vertices[face.vertices[2]]) *
                    0.5F;
                const auto &sources = candidate.polygonContributingTriangles[polygon];
                region.sourceTriangles.insert(region.sourceTriangles.end(), sources.begin(), sources.end());
                if (originalRegion[polygon] != NoNeighbor)
                {
                    sourceRegions.insert(originalRegion[polygon]);
                }
                for (const auto neighbor : face.neighbors)
                {
                    if (neighbor != NoNeighbor && !seen[neighbor])
                    {
                        seen[neighbor] = true;
                        pending.push(neighbor);
                    }
                }
            }
            if (connected)
            {
                for (const auto source : sourceRegions)
                {
                    const auto &geometry = candidate.regions[source].geometrySources;
                    region.geometrySources.insert(region.geometrySources.end(), geometry.begin(), geometry.end());
                }
                region.id = static_cast<std::uint32_t>(regions.size());
                for (const auto polygon : region.polygons)
                {
                    remap[polygon] = 0;
                }
                for (auto *values : {&region.sourceTriangles, &region.geometrySources})
                {
                    std::sort(values->begin(), values->end());
                    values->erase(std::unique(values->begin(), values->end()), values->end());
                }
                regions.push_back(std::move(region));
            }
        }
        std::vector<NavPolygon> polygons;
        std::vector<std::size_t> sources;
        std::vector<std::vector<std::size_t>> contributors;
        for (std::uint32_t polygon{}; polygon < count; ++polygon)
        {
            if (remap[polygon] == NoNeighbor)
            {
                continue;
            }
            remap[polygon] = static_cast<std::uint32_t>(polygons.size());
            polygons.push_back(candidate.mesh.polygons[polygon]);
            sources.push_back(candidate.polygonSourceTriangles[polygon]);
            contributors.push_back(std::move(candidate.polygonContributingTriangles[polygon]));
        }
        for (auto &face : polygons)
        {
            for (auto &neighbor : face.neighbors)
            {
                if (neighbor != NoNeighbor)
                {
                    neighbor = remap[neighbor];
                }
            }
        }
        std::vector<std::uint32_t> polygonRegion(polygons.size(), NoNeighbor);
        for (auto &region : regions)
        {
            for (auto &polygon : region.polygons)
            {
                polygon = remap[polygon];
                polygonRegion[polygon] = region.id;
            }
        }
        for (auto &link : candidate.borderLinks)
        {
            link.polygon = remap[link.polygon];
            regions[polygonRegion[link.polygon]].reachesBorder = true;
        }
        for (auto &door : candidate.exits)
        {
            door.region.reset();
            if (door.polygon && remap[*door.polygon] != NoNeighbor)
            {
                door.polygon = remap[*door.polygon];
                door.region = polygonRegion[*door.polygon];
                regions[*door.region].exitFormIds.push_back(door.referenceId);
            }
            else
            {
                door.polygon.reset();
            }
        }
        candidate.statistics.rejectedUnreachable += count - polygons.size();
        candidate.mesh.polygons = std::move(polygons);
        candidate.polygonSourceTriangles = std::move(sources);
        candidate.polygonContributingTriangles = std::move(contributors);
        candidate.regions = std::move(regions);
    }

    /** Retract orphan seam vertices into their surviving incident fans.
     * Portal endpoints are pinned by exact world coordinates. All welded uses
     * move together so shared edges remain joined; progressively smaller moves
     * are attempted when a nonplanar fan would invert or exceed walking slope.
     * An unmovable vertex is retained for the final coverage check to reject.
     */
    template <class Border>
    void RetractUnlinkedBorderVertices(CandidateNavMesh &candidate, const Border &border, bool retract = true)
    {
        std::map<Key, Vec3> pinned;
        for (const auto &link : candidate.borderLinks)
        {
            const auto &face = candidate.mesh.polygons[link.polygon];
            for (const auto index : {face.vertices[link.edge], face.vertices[(link.edge + 1) % 3]})
            {
                const auto point = candidate.mesh.vertices[index];
                pinned.emplace(Quantize(point, candidate.profile.weldTolerance), point);
            }
        }
        for (std::uint32_t vertex{}; vertex < candidate.mesh.vertices.size(); ++vertex)
        {
            const auto point = candidate.mesh.vertices[vertex];
            const auto key = Quantize(point, candidate.profile.weldTolerance);
            auto portalPoint = pinned.find(key);
            if (portalPoint == pinned.end())
            {
                auto best = candidate.profile.stepHeight;
                for (auto pointIt = pinned.begin(); pointIt != pinned.end(); ++pointIt)
                {
                    const auto other = pointIt->second;
                    const auto height = std::abs(point.z - other.z);
                    if (std::hypot(point.x - other.x, point.y - other.y) <= candidate.profile.weldTolerance &&
                        height < best)
                    {
                        best = height;
                        portalPoint = pointIt;
                    }
                }
            }
            if (border(point, point, candidate.profile.weldTolerance) < 0 ||
                (portalPoint != pinned.end() && point.x == portalPoint->second.x && point.y == portalPoint->second.y &&
                 point.z == portalPoint->second.z))
            {
                continue;
            }
            Vec3 center;
            std::vector<Vec3> fanCenters;
            std::size_t count{};
            std::optional<std::pair<std::uint32_t, std::uint8_t>> incident;
            for (std::uint32_t polygon{}; polygon < candidate.mesh.polygons.size(); ++polygon)
            {
                const auto &face = candidate.mesh.polygons[polygon];
                const auto found = std::find(face.vertices.begin(), face.vertices.end(), vertex);
                if (found != face.vertices.end())
                {
                    incident = {polygon, static_cast<std::uint8_t>(found - face.vertices.begin())};
                }
                if (std::any_of(
                        face.vertices.begin(), face.vertices.end(), [&](auto index)
                        { return Quantize(candidate.mesh.vertices[index], candidate.profile.weldTolerance) == key; }))
                {
                    const auto faceCenter =
                        (candidate.mesh.vertices[face.vertices[0]] + candidate.mesh.vertices[face.vertices[1]] +
                         candidate.mesh.vertices[face.vertices[2]]) /
                        3.0F;
                    center = center + faceCenter;
                    fanCenters.push_back(faceCenter);
                    ++count;
                }
            }
            if (!count || !incident)
            {
                continue;
            }
            if (portalPoint != pinned.end())
            {
                // Welding recognizes near-coincident vertices, but serialized
                // seams require the exact authored coordinate in every use.
                auto revised = candidate;
                if (AlignBorderEndpoints(revised, incident->first, incident->second, portalPoint->second, std::nullopt))
                {
                    candidate = std::move(revised);
                }
                continue;
            }
            if (!retract)
            {
                continue;
            }
            center = center / static_cast<float>(count);
            // A nonplanar fan's average may increase its steepest slope. Individual
            // floor centroids offer directions that preserve that slope envelope.
            fanCenters.insert(fanCenters.begin(), center);
            bool moved{};
            for (const auto target : fanCenters)
            {
                const auto distance = std::hypot(target.x - point.x, target.y - point.y);
                const auto inset = std::max(candidate.profile.agentRadius, candidate.profile.weldTolerance * 4.0F);
                auto fraction = distance > inset ? std::min(0.5F, inset / distance) : 0.5F;
                for (std::size_t attempt{}; attempt < 10; ++attempt, fraction *= 0.5F)
                {
                    const auto position = Interpolate(point, target, fraction);
                    if (border(position, position, candidate.profile.weldTolerance) >= 0)
                    {
                        break;
                    }
                    auto revised = candidate;
                    if (AlignBorderEndpoints(revised, incident->first, incident->second, position, std::nullopt))
                    {
                        candidate = std::move(revised);
                        moved = true;
                        break;
                    }
                }
                if (moved)
                {
                    break;
                }
            }
        }
        for (auto &region : candidate.regions)
        {
            region.area = 0;
            for (const auto polygon : region.polygons)
            {
                const auto &face = candidate.mesh.polygons[polygon];
                region.area +=
                    Area2(candidate.mesh.vertices[face.vertices[0]], candidate.mesh.vertices[face.vertices[1]],
                          candidate.mesh.vertices[face.vertices[2]]) *
                    0.5F;
            }
        }
    }

    /// Rebuild directed boundary loops after seam reshaping and component remapping.
    void RebuildBorderContours(CandidateNavMesh &candidate)
    {
        candidate.contours.clear();
        for (const auto &region : candidate.regions)
        {
            std::multimap<Key2, std::pair<std::uint32_t, Key2>> boundary;
            for (const auto polygon : region.polygons)
            {
                const auto &face = candidate.mesh.polygons[polygon];
                for (std::size_t edge{}; edge < 3; ++edge)
                {
                    if (face.neighbors[edge] == NoNeighbor)
                    {
                        const auto first = face.vertices[edge];
                        const auto last = face.vertices[(edge + 1) % 3];
                        boundary.emplace(Quantize2(candidate.mesh.vertices[first], candidate.profile.weldTolerance),
                                         std::pair{first, Quantize2(candidate.mesh.vertices[last],
                                                                    candidate.profile.weldTolerance)});
                    }
                }
            }
            while (!boundary.empty())
            {
                CandidateContour contour{.region = region.id};
                const auto first = boundary.begin()->first;
                auto current = first;
                while (!boundary.empty())
                {
                    const auto next = boundary.find(current);
                    if (next == boundary.end())
                    {
                        break;
                    }
                    contour.vertices.push_back(next->second.first);
                    current = next->second.second;
                    boundary.erase(next);
                    if (current == first)
                    {
                        contour.closed = true;
                        break;
                    }
                }
                candidate.contours.push_back(std::move(contour));
            }
        }
    }

    /// Reject dangling, reused, or geometrically incompatible portals and any remaining open CELL seam.
    template <class Border>
    void ValidateStitchedBorders(CandidateNavMesh &candidate, const std::vector<NavMesh> &neighbors,
                                 const Border &border)
    {
        std::set<std::pair<std::uint32_t, std::uint8_t>> sources;
        std::set<std::tuple<std::uint32_t, std::uint32_t, std::uint8_t>> targets;
        std::set<std::tuple<float, float, float>> portalVertices;
        const auto coordinate = [](Vec3 point) { return std::tuple{point.x, point.y, point.z}; };
        const auto equal = [](Vec3 a, Vec3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; };
        for (const auto &link : candidate.borderLinks)
        {
            const auto target = std::find_if(neighbors.begin(), neighbors.end(),
                                             [&](const auto &mesh) { return mesh.id == link.neighborNavmeshId; });
            if (link.polygon >= candidate.mesh.polygons.size() || link.edge >= 3 || link.neighborEdge >= 3 ||
                target == neighbors.end() || link.neighborPolygon >= target->polygons.size())
            {
                candidate.topology.findings.push_back("border portal has an invalid destination");
                continue;
            }
            const auto &sourceFace = candidate.mesh.polygons[link.polygon];
            portalVertices.insert(coordinate(candidate.mesh.vertices[sourceFace.vertices[link.edge]]));
            portalVertices.insert(coordinate(candidate.mesh.vertices[sourceFace.vertices[(link.edge + 1) % 3]]));
            const auto &targetFace = target->polygons[link.neighborPolygon];
            const auto first = targetFace.vertices[(link.neighborEdge + 1) % 3];
            const auto last = targetFace.vertices[link.neighborEdge];
            if (first >= target->vertices.size() || last >= target->vertices.size() ||
                sourceFace.neighbors[link.edge] != NoNeighbor ||
                !equal(candidate.mesh.vertices[sourceFace.vertices[link.edge]], target->vertices[first]) ||
                !equal(candidate.mesh.vertices[sourceFace.vertices[(link.edge + 1) % 3]], target->vertices[last]))
            {
                candidate.topology.findings.push_back("border portal endpoints do not match the neighboring edge");
            }
            if (!sources.emplace(link.polygon, link.edge).second ||
                !targets.emplace(link.neighborNavmeshId, link.neighborPolygon, link.neighborEdge).second)
            {
                candidate.topology.findings.push_back("border portal reuses a triangle edge");
            }
        }
        for (std::uint32_t polygon{}; polygon < candidate.mesh.polygons.size(); ++polygon)
        {
            const auto &face = candidate.mesh.polygons[polygon];
            for (std::uint8_t edge{}; edge < 3; ++edge)
            {
                if (face.neighbors[edge] == NoNeighbor && !sources.contains({polygon, edge}) &&
                    border(candidate.mesh.vertices[face.vertices[edge]],
                           candidate.mesh.vertices[face.vertices[(edge + 1) % 3]],
                           candidate.profile.weldTolerance) >= 0)
                {
                    candidate.topology.findings.push_back(
                        std::format("polygon {} has an unlinked CELL border edge", polygon));
                }
            }
        }
        for (const auto &vertex : candidate.mesh.vertices)
        {
            if (border(vertex, vertex, candidate.profile.weldTolerance) >= 0 &&
                !portalVertices.contains(coordinate(vertex)))
            {
                candidate.topology.findings.push_back("CELL border vertex has no matched neighboring endpoint");
            }
        }
        candidate.topology.valid = candidate.topology.findings.empty();
    }
} // namespace

namespace navmesh::core
{
    std::size_t StitchCandidateBorders(CandidateNavMesh &candidate, const AABB &cellBounds,
                                       const std::vector<NavMesh> &neighbors, const std::vector<NavMesh> &authored)
    {
        if (candidate.polygonSourceTriangles.size() != candidate.mesh.polygons.size() ||
            candidate.polygonContributingTriangles.size() != candidate.mesh.polygons.size())
        {
            throw std::invalid_argument("Candidate border stitching requires complete polygon source evidence");
        }
        const auto initialLinks = candidate.borderLinks.size();
        const float maximumGap = candidate.profile.agentRadius * 2.0F + 16.0F;
        const auto distance = [](Vec3 a, Vec3 b) { return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z); };
        const auto border = [&](Vec3 a, Vec3 b, float tolerance)
        {
            // Near a CELL corner, a short edge can fall within the search halo
            // of two sides. Its smallest perpendicular deviation selects the seam.
            const std::array deviations{std::max(std::abs(a.x - cellBounds.min.x), std::abs(b.x - cellBounds.min.x)),
                                        std::max(std::abs(a.x - cellBounds.max.x), std::abs(b.x - cellBounds.max.x)),
                                        std::max(std::abs(a.y - cellBounds.min.y), std::abs(b.y - cellBounds.min.y)),
                                        std::max(std::abs(a.y - cellBounds.max.y), std::abs(b.y - cellBounds.max.y))};
            const auto closest = std::min_element(deviations.begin(), deviations.end());
            return *closest <= tolerance ? static_cast<int>(closest - deviations.begin()) : -1;
        };
        const auto required = ResolveAuthoredBorderPortals(authored, neighbors, border, candidate.profile, maximumGap);
        CoalesceGeneratedBorders(candidate, neighbors, border, maximumGap);
        std::set<std::tuple<std::uint32_t, std::uint32_t, std::uint8_t>> used;
        for (const auto &link : candidate.borderLinks)
        {
            used.emplace(link.neighborNavmeshId, link.neighborPolygon, link.neighborEdge);
        }
        const auto originalCount = candidate.mesh.polygons.size();
        for (std::uint32_t polygon{}; polygon < originalCount; ++polygon)
        {
            for (std::uint8_t side{}; side < 3; ++side)
            {
                const auto face = candidate.mesh.polygons[polygon];
                if (face.neighbors[side] != NoNeighbor ||
                    std::any_of(candidate.borderLinks.begin(), candidate.borderLinks.end(),
                                [&](const auto &link) { return link.polygon == polygon && link.edge == side; }))
                {
                    continue;
                }
                const auto a = candidate.mesh.vertices[face.vertices[side]];
                const auto b = candidate.mesh.vertices[face.vertices[(side + 1) % 3]];
                const auto cellSide = border(a, b, maximumGap);
                if (cellSide < 0 || distance(a, b) <= maximumGap)
                {
                    continue;
                }
                struct Match
                {
                    std::uint32_t navmesh{}, polygon{};
                    std::uint8_t side{};
                    Vec3 a{}, b{};
                    float score{};
                };
                std::optional<Match> best;
                for (const auto &neighbor : neighbors)
                {
                    for (std::uint32_t other{}; other < neighbor.polygons.size(); ++other)
                    {
                        for (std::uint8_t otherSide{}; otherSide < 3; ++otherSide)
                        {
                            const auto &triangle = neighbor.polygons[other];
                            if (triangle.vertices[otherSide] >= neighbor.vertices.size() ||
                                triangle.vertices[(otherSide + 1) % 3] >= neighbor.vertices.size() ||
                                (!(triangle.flags & (1U << otherSide)) && triangle.neighbors[otherSide] != NoNeighbor &&
                                 triangle.neighbors[otherSide] != 0xffffU) ||
                                used.contains({neighbor.id, other, otherSide}))
                            {
                                continue;
                            }
                            auto c = neighbor.vertices[triangle.vertices[otherSide]];
                            auto d = neighbor.vertices[triangle.vertices[(otherSide + 1) % 3]];
                            if (border(c, d, AuthoredBorderTolerance) != cellSide)
                            {
                                continue;
                            }
                            if (distance(a, d) + distance(b, c) < distance(a, c) + distance(b, d))
                            {
                                std::swap(c, d);
                            }
                            const auto gapA = distance(a, c), gapB = distance(b, d);
                            if (gapA > maximumGap || gapB > maximumGap ||
                                std::hypot(a.x - c.x, a.y - c.y) <= candidate.profile.weldTolerance ||
                                std::hypot(b.x - d.x, b.y - d.y) <= candidate.profile.weldTolerance ||
                                std::abs(a.z - c.z) > candidate.profile.stepHeight ||
                                std::abs(b.z - d.z) > candidate.profile.stepHeight || Cross2(a, c, b) <= 0.01F ||
                                Cross2(b, c, d) <= 0.01F ||
                                !CanSubdivideBorder(a, b, candidate.mesh.vertices[face.vertices[(side + 2) % 3]], a, b,
                                                    c, d, candidate.profile))
                            {
                                continue;
                            }
                            const auto score = gapA + gapB;
                            if (!best || score < best->score)
                            {
                                best = {neighbor.id, other, otherSide, c, d, score};
                            }
                        }
                    }
                }
                if (!best)
                {
                    continue;
                }
                const auto c = static_cast<std::uint32_t>(candidate.mesh.vertices.size());
                candidate.mesh.vertices.push_back(best->a);
                const auto d = static_cast<std::uint32_t>(candidate.mesh.vertices.size());
                candidate.mesh.vertices.push_back(best->b);
                const auto first = static_cast<std::uint32_t>(candidate.mesh.polygons.size());
                NavPolygon inner{.vertices = {face.vertices[side], c, face.vertices[(side + 1) % 3]},
                                 .neighbors = {NoNeighbor, first + 1, polygon}};
                NavPolygon outer{.vertices = {face.vertices[(side + 1) % 3], c, d},
                                 .neighbors = {first, NoNeighbor, NoNeighbor}};
                candidate.mesh.polygons.push_back(inner);
                candidate.mesh.polygons.push_back(outer);
                candidate.mesh.polygons[polygon].neighbors[side] = first;
                candidate.polygonSourceTriangles.push_back(candidate.polygonSourceTriangles[polygon]);
                candidate.polygonSourceTriangles.push_back(candidate.polygonSourceTriangles[polygon]);
                candidate.polygonContributingTriangles.push_back(candidate.polygonContributingTriangles[polygon]);
                candidate.polygonContributingTriangles.push_back(candidate.polygonContributingTriangles[polygon]);
                for (auto &region : candidate.regions)
                {
                    if (std::find(region.polygons.begin(), region.polygons.end(), polygon) != region.polygons.end())
                    {
                        region.polygons.push_back(first);
                        region.polygons.push_back(first + 1);
                        region.area += (Area2(a, best->a, b) + Area2(b, best->a, best->b)) * 0.5F;
                        region.reachesBorder = true;
                        break;
                    }
                }
                candidate.borderLinks.push_back({first + 1, 1, best->navmesh, best->polygon, best->side});
                used.emplace(best->navmesh, best->polygon, best->side);
            }
        }
        StitchPartitionedBorders(candidate, neighbors, border, maximumGap);
        CompactBorderVertices(candidate);
        RetractUnlinkedBorderVertices(candidate, border, false);
        for (const auto &[destination, constraints] : required)
        {
            const auto [id, polygon, edge] = destination;
            const auto neighbor =
                std::find_if(neighbors.begin(), neighbors.end(), [&](const auto &mesh) { return mesh.id == id; });
            if (std::none_of(candidate.borderLinks.begin(), candidate.borderLinks.end(),
                             [&](const auto &link)
                             {
                                 return link.neighborNavmeshId == id && link.neighborPolygon == polygon &&
                                        link.neighborEdge == edge;
                             }))
            {
                (void)RetriangulateBorderPortal(candidate, *neighbor, polygon, edge, border, maximumGap,
                                                constraints.depth, constraints.heightTolerance);
            }
        }
        BuildAdjacency(candidate.mesh, candidate.profile.weldTolerance, candidate.profile.stepHeight);
        RetractUnlinkedBorders(candidate, border);
        RetainConnectedBorderRegions(candidate);
        // Endpoint alignment and component filtering can leave unused source
        // vertices. Compact them before validating incident-fan deformations.
        CompactBorderVertices(candidate);
        RetractUnlinkedBorderVertices(candidate, border);
        RebuildBorderContours(candidate);
        // Neighboring bridges can share an edge even when they target separate NAVM triangles.
        BuildAdjacency(candidate.mesh, candidate.profile.weldTolerance, candidate.profile.stepHeight);
        candidate.statistics.outputPolygons = candidate.mesh.polygons.size();
        candidate.topology = ValidateCandidateTopology(candidate);
        ValidateStitchedBorders(candidate, neighbors, border);
        for (const auto &[destination, constraints] : required)
        {
            const auto [id, polygon, edge] = destination;
            if (std::none_of(candidate.borderLinks.begin(), candidate.borderLinks.end(),
                             [&](const auto &link)
                             {
                                 return link.neighborNavmeshId == id && link.neighborPolygon == polygon &&
                                        link.neighborEdge == edge;
                             }))
            {
                candidate.topology.valid = false;
                candidate.topology.findings.push_back(
                    std::format("Authored border portal {:08X}:{}:{} was not retained", id, polygon, edge));
            }
        }
        return candidate.borderLinks.size() - initialLinks;
    }

    CandidateNavMesh GenerateCandidate(const Scene &scene, const NavigationProfile &profile,
                                       std::optional<AABB> cellBounds, std::vector<CandidateExit> exits)
    {
        CandidateNavMesh result;
        result.profile = profile;
        result.exits = std::move(exits);
        const auto &input = scene.mesh;
        result.statistics.inputTriangles = input.triangles.size();
        if (profile.agentRadius < 0 || profile.agentHeight <= 0 || profile.clearance < profile.agentHeight ||
            profile.maxSlopeDegrees < 0 || profile.maxSlopeDegrees >= 90 || profile.stepHeight < 0 ||
            profile.weldTolerance <= 0 || profile.minimumRegionArea < 0 || profile.contourSimplificationTolerance < 0 ||
            (profile.cellBorderPolicy != "preserve_open_border" && profile.cellBorderPolicy != "inset_all_edges"))
        {
            throw std::invalid_argument("Invalid navigation settings");
        }
        if (scene.triangleProvenance.size() != input.triangles.size())
        {
            throw std::invalid_argument("Candidate generation requires complete triangle provenance");
        }
        navmesh::analysis::SpatialIndex spatial;
        spatial.Build(input.triangles, input.vertices);
        struct Accepted
        {
            std::array<Vec3, 3> points;
            std::size_t source{};
        };
        std::vector<Accepted> accepted;
        for (std::size_t i{}; i < input.triangles.size(); ++i)
        {
            const auto &provenance = scene.triangleProvenance[i];
            if (provenance.geometrySource >= scene.geometrySources.size() ||
                scene.geometrySources[provenance.geometrySource].sourceType == GeometrySourceType::RenderFallback)
            {
                ++result.statistics.rejectedSource;
                continue;
            }
            const auto &tri = input.triangles[i];
            if (tri.vertices[0] >= input.vertices.size() || tri.vertices[1] >= input.vertices.size() ||
                tri.vertices[2] >= input.vertices.size())
            {
                ++result.statistics.rejectedDegenerate;
                continue;
            }
            std::array<Vec3, 3> points{input.vertices[tri.vertices[0]], input.vertices[tri.vertices[1]],
                                       input.vertices[tri.vertices[2]]};
            if (!std::isfinite(points[0].x) || !std::isfinite(points[0].y) || !std::isfinite(points[0].z) ||
                !std::isfinite(points[1].x) || !std::isfinite(points[1].y) || !std::isfinite(points[1].z) ||
                !std::isfinite(points[2].x) || !std::isfinite(points[2].y) || !std::isfinite(points[2].z) ||
                Area2(points[0], points[1], points[2]) <= profile.weldTolerance * profile.weldTolerance)
            {
                ++result.statistics.rejectedDegenerate;
                continue;
            }
            if (Cross2(points[0], points[1], points[2]) < 0)
            {
                std::swap(points[1], points[2]);
            }
            const auto u = points[1] - points[0], v = points[2] - points[0];
            const auto horizontal = std::abs(u.x * v.y - u.y * v.x);
            const auto vertical = std::hypot(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z);
            const auto slope = std::atan2(vertical, horizontal) * 180.0F / 3.14159265358979323846F;
            if (slope > profile.maxSlopeDegrees)
            {
                ++result.statistics.rejectedSlope;
                continue;
            }
            if (cellBounds)
            {
                const auto [minX, maxX] = std::minmax({points[0].x, points[1].x, points[2].x});
                const auto [minY, maxY] = std::minmax({points[0].y, points[1].y, points[2].y});
                if (maxX < cellBounds->min.x || minX > cellBounds->max.x || maxY < cellBounds->min.y ||
                    minY > cellBounds->max.y)
                {
                    continue;
                }
            }
            if (!HasClearance(scene, spatial, i, points, profile.clearance, profile.stepHeight))
            {
                ++result.statistics.rejectedClearance;
                continue;
            }
            if (HasWallObstruction(scene, spatial, i, points, profile))
            {
                ++result.statistics.rejectedObstruction;
                continue;
            }
            ++result.statistics.eligibleTriangles;
            if (cellBounds)
            {
                const auto &bounds = *cellBounds;
                std::vector<Vec3> clipped{points.begin(), points.end()};
                clipped = ClipInsideEdge(clipped, {bounds.min.x, bounds.min.y, 0}, {bounds.max.x, bounds.min.y, 0}, 0);
                clipped = ClipInsideEdge(clipped, {bounds.max.x, bounds.min.y, 0}, {bounds.max.x, bounds.max.y, 0}, 0);
                clipped = ClipInsideEdge(clipped, {bounds.max.x, bounds.max.y, 0}, {bounds.min.x, bounds.max.y, 0}, 0);
                clipped = ClipInsideEdge(clipped, {bounds.min.x, bounds.max.y, 0}, {bounds.min.x, bounds.min.y, 0}, 0);
                for (std::size_t corner = 1; corner + 1 < clipped.size(); ++corner)
                {
                    if (Area2(clipped[0], clipped[corner], clipped[corner + 1]) >
                        profile.weldTolerance * profile.weldTolerance)
                    {
                        accepted.push_back({.points = {clipped[0], clipped[corner], clipped[corner + 1]}, .source = i});
                    }
                }
            }
            else
            {
                accepted.push_back({.points = points, .source = i});
            }
        }
        // Separate NIF placements of the same architectural walkway have
        // sub-unit transform/serialization drift (the Riverwood seams are
        // roughly 0.05 world units apart). Canonicalize only near-coincident
        // vertices, preserving Z on a step and all genuinely stacked levels.
        constexpr float placementSeamTolerance = 0.25F;
        std::map<Key2, std::vector<Vec3>> seamVertices;
        for (auto &triangle : accepted)
        {
            for (auto &point : triangle.points)
            {
                const auto bin = Quantize2(point, placementSeamTolerance);
                std::optional<Vec3> nearest;
                float best = placementSeamTolerance;
                for (std::int64_t x = bin.first - 1; x <= bin.first + 1; ++x)
                {
                    for (std::int64_t y = bin.second - 1; y <= bin.second + 1; ++y)
                    {
                        if (const auto found = seamVertices.find({x, y}); found != seamVertices.end())
                        {
                            for (const auto &other : found->second)
                            {
                                const auto distance = std::hypot(point.x - other.x, point.y - other.y);
                                if (distance < best &&
                                    std::abs(point.z - other.z) <= profile.stepHeight + placementSeamTolerance)
                                {
                                    best = distance;
                                    nearest = other;
                                }
                            }
                        }
                    }
                }
                if (nearest)
                {
                    point.x = nearest->x;
                    point.y = nearest->y;
                    if (std::abs(point.z - nearest->z) <= placementSeamTolerance)
                    {
                        point.z = nearest->z;
                    }
                }
                seamVertices[Quantize2(point, placementSeamTolerance)].push_back(point);
            }
        }
        std::vector<std::vector<std::size_t>> acceptedBySource(input.triangles.size());
        for (std::size_t i{}; i < accepted.size(); ++i)
        {
            acceptedBySource[accepted[i].source].push_back(i);
        }
        for (std::size_t i{}; i < accepted.size(); ++i)
        {
            for (auto &point : accepted[i].points)
            {
                const auto original = point;
                std::optional<Vec3> nearest;
                float best = placementSeamTolerance;
                AABB query{.min = {point.x - placementSeamTolerance, point.y - placementSeamTolerance,
                                   point.z - profile.stepHeight - placementSeamTolerance},
                           .max = {point.x + placementSeamTolerance, point.y + placementSeamTolerance,
                                   point.z + profile.stepHeight + placementSeamTolerance}};
                for (const auto source : spatial.QueryAABB(query))
                {
                    if (source >= acceptedBySource.size())
                    {
                        continue;
                    }
                    for (const auto otherIndex : acceptedBySource[source])
                    {
                        if (otherIndex == i)
                        {
                            continue;
                        }
                        const auto &other = accepted[otherIndex];
                        for (std::size_t side{}; side < 3; ++side)
                        {
                            const auto a = other.points[side], b = other.points[(side + 1) % 3];
                            const auto dx = b.x - a.x, dy = b.y - a.y, length2 = dx * dx + dy * dy;
                            if (length2 <= 1.0e-5F)
                            {
                                continue;
                            }
                            const auto t = ((original.x - a.x) * dx + (original.y - a.y) * dy) / length2;
                            const auto margin = placementSeamTolerance / std::sqrt(length2);
                            if (t <= margin || t >= 1.0F - margin)
                            {
                                continue;
                            }
                            const auto projection = Interpolate(a, b, t);
                            const auto distance = std::hypot(original.x - projection.x, original.y - projection.y);
                            if (distance < best &&
                                std::abs(original.z - projection.z) <= profile.stepHeight + placementSeamTolerance)
                            {
                                best = distance;
                                nearest = projection;
                            }
                        }
                    }
                }
                if (nearest)
                {
                    point.x = nearest->x;
                    point.y = nearest->y;
                    if (std::abs(point.z - nearest->z) <= placementSeamTolerance)
                    {
                        point.z = nearest->z;
                    }
                }
            }
        }
        // Havok pieces and LAND often meet at a T junction. Split the longer
        // edge at every compatible accepted vertex before deciding which edges
        // are exposed; otherwise each half is inset as an isolated island.
        std::vector<Accepted> split;
        split.reserve(accepted.size());
        for (std::size_t i{}; i < accepted.size(); ++i)
        {
            std::vector<Vec3> boundary;
            for (std::size_t side{}; side < 3; ++side)
            {
                const auto a = accepted[i].points[side], b = accepted[i].points[(side + 1) % 3];
                boundary.push_back(a);
                const auto dx = b.x - a.x, dy = b.y - a.y;
                const auto length2 = dx * dx + dy * dy;
                if (length2 <= profile.weldTolerance * profile.weldTolerance)
                {
                    continue;
                }
                const auto length = std::sqrt(length2);
                AABB query{
                    .min = {std::min(a.x, b.x) - placementSeamTolerance, std::min(a.y, b.y) - placementSeamTolerance,
                            std::min(a.z, b.z) - profile.stepHeight - placementSeamTolerance},
                    .max = {std::max(a.x, b.x) + placementSeamTolerance, std::max(a.y, b.y) + placementSeamTolerance,
                            std::max(a.z, b.z) + profile.stepHeight + placementSeamTolerance}};
                std::vector<float> cuts;
                for (const auto source : spatial.QueryAABB(query))
                {
                    if (source >= acceptedBySource.size())
                    {
                        continue;
                    }
                    for (const auto otherIndex : acceptedBySource[source])
                    {
                        if (otherIndex == i)
                        {
                            continue;
                        }
                        for (const auto point : accepted[otherIndex].points)
                        {
                            const auto t = ((point.x - a.x) * dx + (point.y - a.y) * dy) / length2;
                            if (t <= profile.weldTolerance / length || t >= 1.0F - profile.weldTolerance / length ||
                                std::abs(Cross2(a, b, point)) / length > profile.weldTolerance ||
                                std::abs(point.z - (a.z + (b.z - a.z) * t)) >
                                    profile.stepHeight + profile.weldTolerance)
                            {
                                continue;
                            }
                            cuts.push_back(t);
                        }
                    }
                }
                std::sort(cuts.begin(), cuts.end());
                for (const auto t : cuts)
                {
                    if (t <= 0 || t >= 1 ||
                        (!boundary.empty() && std::hypot(boundary.back().x - (a.x + dx * t),
                                                         boundary.back().y - (a.y + dy * t)) <= profile.weldTolerance))
                    {
                        continue;
                    }
                    boundary.push_back(Interpolate(a, b, t));
                }
            }
            for (std::size_t corner = 1; corner + 1 < boundary.size(); ++corner)
            {
                if (Area2(boundary[0], boundary[corner], boundary[corner + 1]) >
                    profile.weldTolerance * profile.weldTolerance)
                {
                    split.push_back({.points = {boundary[0], boundary[corner], boundary[corner + 1]},
                                     .source = accepted[i].source});
                }
            }
        }
        accepted = std::move(split);
        std::map<Edge, std::vector<std::pair<std::size_t, std::size_t>>> sourceEdges;
        for (std::size_t i{}; i < accepted.size(); ++i)
        {
            for (std::size_t side{}; side < 3; ++side)
            {
                sourceEdges[SortedEdge(accepted[i].points[side], accepted[i].points[(side + 1) % 3],
                                       profile.weldTolerance)]
                    .push_back({i, side});
            }
        }
        std::map<Key, std::uint32_t> outputKeys;
        for (std::size_t i{}; i < accepted.size(); ++i)
        {
            auto polygon = std::vector<Vec3>{accepted[i].points.begin(), accepted[i].points.end()};
            for (std::size_t side{}; side < 3 && !polygon.empty(); ++side)
            {
                const auto a = accepted[i].points[side], b = accepted[i].points[(side + 1) % 3];
                const auto &uses = sourceEdges[SortedEdge(a, b, profile.weldTolerance)];
                std::size_t compatible{};
                for (const auto &[otherIndex, otherSide] : uses)
                {
                    if (otherIndex != i)
                    {
                        const auto &other = accepted[otherIndex];
                        if (StepCompatible(a, b, other.points[otherSide], other.points[(otherSide + 1) % 3],
                                           profile.weldTolerance, profile.stepHeight))
                        {
                            ++compatible;
                        }
                    }
                }
                const bool connected = compatible == 1;
                const bool border = cellBounds && profile.cellBorderPolicy == "preserve_open_border" &&
                                    OnCellBorder(a, b, *cellBounds, profile.weldTolerance);
                if (!connected && !border)
                {
                    polygon = ClipInsideEdge(polygon, a, b, profile.agentRadius);
                }
            }
            if (polygon.size() < 3)
            {
                continue;
            }
            const auto makeVertex = [&](Vec3 point)
            {
                const auto key = Quantize(point, profile.weldTolerance);
                auto [where, created] =
                    outputKeys.try_emplace(key, static_cast<std::uint32_t>(result.mesh.vertices.size()));
                if (created)
                {
                    result.mesh.vertices.push_back(point);
                }
                return where->second;
            };
            const auto first = makeVertex(polygon.front());
            for (std::size_t corner = 1; corner + 1 < polygon.size(); ++corner)
            {
                if (Area2(polygon.front(), polygon[corner], polygon[corner + 1]) <=
                    profile.weldTolerance * profile.weldTolerance)
                {
                    continue;
                }
                NavPolygon nav;
                nav.vertices = {first, makeVertex(polygon[corner]), makeVertex(polygon[corner + 1])};
                nav.neighbors.fill(NoNeighbor);
                if (nav.vertices[0] == nav.vertices[1] || nav.vertices[1] == nav.vertices[2] ||
                    nav.vertices[2] == nav.vertices[0])
                {
                    continue;
                }
                result.mesh.polygons.push_back(nav);
                result.polygonSourceTriangles.push_back(accepted[i].source);
            }
        }
        SplitPartialOutputEdges(result.mesh, result.polygonSourceTriangles, profile);
        BuildAdjacency(result.mesh, profile.weldTolerance, profile.stepHeight);
        std::vector<bool> acceptedSources(input.triangles.size());
        for (const auto &triangle : accepted)
        {
            acceptedSources[triangle.source] = true;
        }
        ConnectSupportedSeams(result.mesh, result.polygonSourceTriangles, scene, spatial, acceptedSources, profile);
        // Quantization can collapse a thin clipped triangle after the initial
        // area test. Remove those faces, along with coincident duplicates,
        // before building regions or validating the final topology.
        {
            NavMesh clean;
            std::vector<std::size_t> sources;
            std::map<std::uint32_t, std::uint32_t> vertices;
            std::set<std::array<std::uint32_t, 3>> faces;
            std::size_t removed{};
            for (std::size_t i{}; i < result.mesh.polygons.size(); ++i)
            {
                const auto &polygon = result.mesh.polygons[i];
                const auto &points = result.mesh.vertices;
                auto face = polygon.vertices;
                std::sort(face.begin(), face.end());
                if (Cross2(points[polygon.vertices[0]], points[polygon.vertices[1]], points[polygon.vertices[2]]) <=
                        1.0e-5F ||
                    !faces.insert(face).second)
                {
                    ++removed;
                    continue;
                }
                auto copy = polygon;
                for (auto &vertex : copy.vertices)
                {
                    auto [where, created] =
                        vertices.try_emplace(vertex, static_cast<std::uint32_t>(clean.vertices.size()));
                    if (created)
                    {
                        clean.vertices.push_back(points[vertex]);
                    }
                    vertex = where->second;
                }
                clean.polygons.push_back(copy);
                sources.push_back(result.polygonSourceTriangles[i]);
            }
            result.mesh = std::move(clean);
            result.polygonSourceTriangles = std::move(sources);
            if (removed)
            {
                result.warnings.push_back(
                    std::format("Removed {} collapsed or duplicate candidate triangles.", removed));
            }
            BuildAdjacency(result.mesh, profile.weldTolerance, profile.stepHeight);
        }
        if (const auto removed = CullNonManifoldFaces(result.mesh, result.polygonSourceTriangles, profile))
        {
            result.warnings.push_back(std::format("Removed {} faces competing for non-manifold edges.", removed));
        }
        const auto components = Components(result.mesh);
        std::vector<std::size_t> componentOf(result.mesh.polygons.size());
        std::vector<bool> anchored(components.size());
        std::vector<float> componentAreas(components.size());
        std::vector<bool> collisionFloor(components.size());
        for (std::size_t component{}; component < components.size(); ++component)
        {
            for (const auto index : components[component])
            {
                componentOf[index] = component;
                const auto &polygon = result.mesh.polygons[index];
                componentAreas[component] +=
                    Area2(result.mesh.vertices[polygon.vertices[0]], result.mesh.vertices[polygon.vertices[1]],
                          result.mesh.vertices[polygon.vertices[2]]) *
                    0.5F;
                const auto source = scene.triangleProvenance[result.polygonSourceTriangles[index]].geometrySource;
                if (scene.geometrySources[source].sourceType == GeometrySourceType::Collision)
                {
                    collisionFloor[component] = true;
                }
                if (cellBounds)
                {
                    for (std::size_t side{}; side < 3; ++side)
                    {
                        if (TouchesCellBorder(result.mesh.vertices[polygon.vertices[side]],
                                              result.mesh.vertices[polygon.vertices[(side + 1) % 3]], *cellBounds,
                                              profile.weldTolerance))
                        {
                            anchored[component] = true;
                        }
                    }
                }
            }
        }
        for (const auto &door : result.exits)
        {
            if (const auto polygon = ExitPolygon(result.mesh, door.position, profile.stepHeight))
            {
                anchored[componentOf[*polygon]] = true;
            }
        }
        // A broad supported deck can be entirely inside the extracted area,
        // with its stairs or neighboring supports lost to collision coverage
        // or a narrow seam. Preserve it as a separate inspection region.
        // Small unanchored props and fragments still fail the region filter.
        const auto standaloneArea = std::max(16384.0F, 64.0F * profile.agentRadius * profile.agentRadius);
        std::size_t retainedCollisionRegions{};
        for (std::size_t component{}; component < components.size(); ++component)
        {
            if (!anchored[component] && collisionFloor[component] && componentAreas[component] >= standaloneArea)
            {
                anchored[component] = true;
                ++retainedCollisionRegions;
            }
        }
        if (retainedCollisionRegions)
        {
            result.warnings.push_back(std::format(
                "Retained {} substantial collision regions without a border or door anchor; inspect their connections.",
                retainedCollisionRegions));
        }
        if (!components.empty() && std::none_of(anchored.begin(), anchored.end(), [](bool value) { return value; }))
        {
            const auto largest = std::max_element(componentAreas.begin(), componentAreas.end());
            anchored[static_cast<std::size_t>(largest - componentAreas.begin())] = true;
            result.warnings.push_back("No region reaches a cell border or identified exit; retained the largest "
                                      "connected region for review.");
        }
        std::vector<bool> keep(result.mesh.polygons.size(), true);
        for (std::size_t component{}; component < components.size(); ++component)
        {
            if (anchored[component])
            {
                continue;
            }
            for (const auto index : components[component])
            {
                keep[index] = false;
            }
            if (componentAreas[component] < profile.minimumRegionArea)
            {
                result.statistics.rejectedSmallRegion += components[component].size();
            }
            else
            {
                result.statistics.rejectedUnreachable += components[component].size();
            }
        }
        if (result.statistics.rejectedSmallRegion || result.statistics.rejectedUnreachable)
        {
            NavMesh filtered;
            std::vector<std::size_t> sources;
            for (std::size_t i{}; i < keep.size(); ++i)
            {
                if (keep[i])
                {
                    filtered.polygons.push_back(result.mesh.polygons[i]);
                    sources.push_back(result.polygonSourceTriangles[i]);
                }
            }
            std::map<std::uint32_t, std::uint32_t> remap;
            for (auto &polygon : filtered.polygons)
            {
                for (auto &vertex : polygon.vertices)
                {
                    auto [where, created] =
                        remap.try_emplace(vertex, static_cast<std::uint32_t>(filtered.vertices.size()));
                    if (created)
                    {
                        filtered.vertices.push_back(result.mesh.vertices[vertex]);
                    }
                    vertex = where->second;
                }
            }
            result.mesh = std::move(filtered);
            result.polygonSourceTriangles = std::move(sources);
            BuildAdjacency(result.mesh, profile.weldTolerance, profile.stepHeight);
        }
        result.statistics.polygonsBeforeSimplification = result.mesh.polygons.size();
        result.polygonContributingTriangles.reserve(result.polygonSourceTriangles.size());
        for (const auto source : result.polygonSourceTriangles)
        {
            result.polygonContributingTriangles.push_back({source});
        }
        SimplifyInteriorSurface(result, scene);
        for (const auto &component : Components(result.mesh))
        {
            CandidateRegion region;
            region.id = static_cast<std::uint32_t>(result.regions.size());
            region.polygons = component;
            std::set<std::size_t> sourceTriangles, geometrySources;
            for (const auto index : component)
            {
                const auto &tri = result.mesh.polygons[index];
                region.area += Area2(result.mesh.vertices[tri.vertices[0]], result.mesh.vertices[tri.vertices[1]],
                                     result.mesh.vertices[tri.vertices[2]]) *
                               0.5F;
                for (const auto source : result.polygonContributingTriangles[index])
                {
                    sourceTriangles.insert(source);
                    geometrySources.insert(scene.triangleProvenance[source].geometrySource);
                }
            }
            region.sourceTriangles.assign(sourceTriangles.begin(), sourceTriangles.end());
            region.geometrySources.assign(geometrySources.begin(), geometrySources.end());
            result.regions.push_back(std::move(region));
        }
        std::vector<std::uint32_t> regionOf(result.mesh.polygons.size());
        for (const auto &region : result.regions)
        {
            for (const auto polygon : region.polygons)
            {
                regionOf[polygon] = region.id;
            }
        }
        for (auto &region : result.regions)
        {
            if (cellBounds)
            {
                for (const auto index : region.polygons)
                {
                    const auto &polygon = result.mesh.polygons[index];
                    for (std::size_t side{}; side < 3; ++side)
                    {
                        if (TouchesCellBorder(result.mesh.vertices[polygon.vertices[side]],
                                              result.mesh.vertices[polygon.vertices[(side + 1) % 3]], *cellBounds,
                                              profile.weldTolerance))
                        {
                            region.reachesBorder = true;
                        }
                    }
                }
            }
        }
        for (auto &door : result.exits)
        {
            if (const auto polygon = ExitPolygon(result.mesh, door.position, profile.stepHeight))
            {
                door.region = regionOf[*polygon];
                door.polygon = static_cast<std::uint32_t>(*polygon);
                result.regions[*door.region].exitFormIds.push_back(door.referenceId);
            }
        }
        for (const auto &region : result.regions)
        {
            std::multimap<Key2, std::pair<std::uint32_t, Key2>> boundary;
            for (const auto index : region.polygons)
            {
                const auto &tri = result.mesh.polygons[index];
                for (std::size_t side{}; side < 3; ++side)
                {
                    if (tri.neighbors[side] == NoNeighbor)
                    {
                        boundary.emplace(
                            Quantize2(result.mesh.vertices[tri.vertices[side]], profile.weldTolerance),
                            std::pair{tri.vertices[side], Quantize2(result.mesh.vertices[tri.vertices[(side + 1) % 3]],
                                                                    profile.weldTolerance)});
                    }
                }
            }
            while (!boundary.empty())
            {
                CandidateContour contour;
                contour.region = region.id;
                const auto first = boundary.begin()->first;
                auto current = first;
                for (std::size_t count{}; count <= result.mesh.polygons.size() * 3; ++count)
                {
                    const auto edge = boundary.find(current);
                    if (edge == boundary.end())
                    {
                        break;
                    }
                    contour.vertices.push_back(edge->second.first);
                    current = edge->second.second;
                    boundary.erase(edge);
                    if (current == first)
                    {
                        contour.closed = true;
                        break;
                    }
                }
                SimplifyContour(contour, result.mesh, profile.contourSimplificationTolerance);
                result.contours.push_back(std::move(contour));
            }
        }
        result.statistics.outputPolygons = result.mesh.polygons.size();
        if (result.mesh.polygons.empty())
        {
            result.warnings.push_back(
                "No candidate polygons survived source, slope, clearance, radius, and region filters.");
        }
        result.topology = ValidateCandidateTopology(result);
        return result;
    }

    CandidateTopology ValidateCandidateTopology(const CandidateNavMesh &candidate)
    {
        CandidateTopology result;
        const auto &mesh = candidate.mesh;
        if (candidate.polygonSourceTriangles.size() != mesh.polygons.size())
        {
            result.findings.push_back("polygon provenance count mismatch");
        }
        if (candidate.polygonContributingTriangles.size() != mesh.polygons.size())
        {
            result.findings.push_back("polygon contributor count mismatch");
        }
        std::map<Edge, std::vector<std::pair<std::size_t, std::size_t>>> edges;
        std::set<std::array<std::uint32_t, 3>> faces;
        std::vector<bool> used(mesh.vertices.size());
        for (std::size_t i{}; i < mesh.polygons.size(); ++i)
        {
            const auto &tri = mesh.polygons[i];
            if (i < candidate.polygonContributingTriangles.size() && candidate.polygonContributingTriangles[i].empty())
            {
                result.findings.push_back(std::format("polygon {} has no source contributors", i));
            }
            for (const auto vertex : tri.vertices)
            {
                if (vertex >= mesh.vertices.size())
                {
                    result.findings.push_back(std::format("polygon {} has invalid vertex", i));
                }
                else
                {
                    used[vertex] = true;
                }
            }
            if (std::any_of(tri.vertices.begin(), tri.vertices.end(),
                            [&](auto vertex) { return vertex >= mesh.vertices.size(); }))
            {
                continue;
            }
            if (Cross2(mesh.vertices[tri.vertices[0]], mesh.vertices[tri.vertices[1]],
                       mesh.vertices[tri.vertices[2]]) <= 1.0e-5F)
            {
                result.findings.push_back(std::format("polygon {} is degenerate or clockwise", i));
            }
            auto face = tri.vertices;
            std::sort(face.begin(), face.end());
            if (!faces.insert(face).second)
            {
                result.findings.push_back(std::format("duplicate polygon {}", i));
            }
            for (std::size_t side{}; side < 3; ++side)
            {
                edges[SortedEdge(mesh.vertices[tri.vertices[side]], mesh.vertices[tri.vertices[(side + 1) % 3]],
                                 candidate.profile.weldTolerance)]
                    .push_back({i, side});
            }
        }
        for (const auto &[edge, uses] : edges)
        {
            for (const auto &[index, side] : uses)
            {
                auto expected = NoNeighbor;
                std::size_t matches{};
                const auto &left = mesh.polygons[index];
                for (const auto &[otherIndex, otherSide] : uses)
                {
                    if (otherIndex != index)
                    {
                        const auto &right = mesh.polygons[otherIndex];
                        if (StepCompatible(mesh.vertices[left.vertices[side]],
                                           mesh.vertices[left.vertices[(side + 1) % 3]],
                                           mesh.vertices[right.vertices[otherSide]],
                                           mesh.vertices[right.vertices[(otherSide + 1) % 3]],
                                           candidate.profile.weldTolerance, candidate.profile.stepHeight))
                        {
                            ++matches;
                            expected = static_cast<std::uint32_t>(otherIndex);
                        }
                    }
                }
                if (matches > 1)
                {
                    result.findings.push_back("non-manifold compatible edge");
                    expected = NoNeighbor;
                }
                if (matches == 0)
                {
                    expected = NoNeighbor;
                }
                if (mesh.polygons[index].neighbors[side] != expected)
                {
                    result.findings.push_back(std::format("invalid adjacency at polygon {} edge {}", index, side));
                }
            }
        }
        for (std::size_t i{}; i < used.size(); ++i)
        {
            if (!used[i])
            {
                result.findings.push_back(std::format("orphan vertex {}", i));
            }
        }
        std::vector<std::uint32_t> membership(mesh.polygons.size());
        for (const auto &region : candidate.regions)
        {
            for (const auto polygon : region.polygons)
            {
                if (polygon >= membership.size())
                {
                    result.findings.push_back("region references invalid polygon");
                }
                else
                {
                    ++membership[polygon];
                }
            }
        }
        for (std::size_t i{}; i < membership.size(); ++i)
        {
            if (membership[i] != 1)
            {
                result.findings.push_back(std::format("polygon {} has {} region memberships", i, membership[i]));
            }
        }
        for (std::size_t i{}; i < candidate.contours.size(); ++i)
        {
            if (!candidate.contours[i].closed)
            {
                result.findings.push_back(std::format("open contour {}", i));
            }
        }
        result.valid = result.findings.empty();
        return result;
    }

    bool WriteCandidateJson(const std::filesystem::path &path, const CandidateNavMesh &candidate, const Scene &scene,
                            const std::string &metadataJson)
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out)
        {
            return false;
        }
        return WriteCandidateJson(out, candidate, scene, metadataJson);
    }

    bool WriteCandidateJson(std::ostream &out, const CandidateNavMesh &candidate, const Scene &scene,
                            const std::string &metadataJson)
    {
        const auto &p = candidate.profile;
        out << "{\n  \"schema\": \"navmesh-generator/candidate-navm\",\n  \"schema_version\": \"1.1.0\",\n  "
               "\"metadata\": "
            << metadataJson << ",\n";
        out << std::format("  \"partitioning_algorithm\": \"{}\",\n", candidate.partitioningAlgorithm);
        out << std::format("  \"triangle_tagging\": {},\n", candidate.triangleTagging);
        const auto &recast = candidate.recastSettings;
        out << std::format("  \"recast_settings\": {{\"cell_size\":{},\"cell_height\":{},"
                           "\"max_simplification_error\":{},\"max_edge_length\":{},"
                           "\"merge_region_area_multiplier\":{}}},\n",
                           recast.cellSize, recast.cellHeight, recast.maxSimplificationError, recast.maxEdgeLength,
                           recast.mergeRegionAreaMultiplier);
        out << std::format("  \"profile\": "
                           "{{\"name\":\"{}\",\"agent_radius\":{},\"agent_height\":{},\"max_slope_degrees\":{},\"step_"
                           "height\":{},\"clearance\":{},\"weld_tolerance\":{},\"minimum_region_area\":{},\"contour_"
                           "simplification_tolerance\":{},\"cell_border_policy\":\"{}\"}},\n",
                           p.name, p.agentRadius, p.agentHeight, p.maxSlopeDegrees, p.stepHeight, p.clearance,
                           p.weldTolerance, p.minimumRegionArea, p.contourSimplificationTolerance, p.cellBorderPolicy);
        const auto &s = candidate.statistics;
        out << std::format(
            "  \"statistics\": "
            "{{\"input_triangles\":{},\"eligible_triangles\":{},\"rejected_source\":{},\"rejected_slope\":{},"
            "\"rejected_clearance\":{},\"rejected_obstruction\":{},\"rejected_degenerate\":{},\"rejected_small_"
            "region\":{},\"rejected_unreachable\":{},\"polygons_before_simplification\":{},\"output_polygons\":{}}},\n",
            s.inputTriangles, s.eligibleTriangles, s.rejectedSource, s.rejectedSlope, s.rejectedClearance,
            s.rejectedObstruction, s.rejectedDegenerate, s.rejectedSmallRegion, s.rejectedUnreachable,
            s.polygonsBeforeSimplification, s.outputPolygons);
        out << "  \"vertices\": [";
        for (std::size_t i{}; i < candidate.mesh.vertices.size(); ++i)
        {
            const auto &v = candidate.mesh.vertices[i];
            out << (i ? "," : "") << std::format("[{},{},{}]", v.x, v.y, v.z);
        }
        out << "],\n  \"polygons\": [";
        for (std::size_t i{}; i < candidate.mesh.polygons.size(); ++i)
        {
            const auto &tri = candidate.mesh.polygons[i];
            const auto sourceIndex = candidate.polygonSourceTriangles[i];
            const auto sourceId = scene.triangleProvenance[sourceIndex].geometrySource;
            out << (i ? "," : "")
                << std::format("{{\"vertices\":[{},{},{}],\"neighbors\":[", tri.vertices[0], tri.vertices[1],
                               tri.vertices[2]);
            for (std::size_t side{}; side < 3; ++side)
            {
                out << (side ? "," : "")
                    << (tri.neighbors[side] == NoNeighbor ? "null" : std::to_string(tri.neighbors[side]));
            }
            out << std::format("],\"source_triangle\":{},\"source_triangles\":[", sourceIndex);
            for (std::size_t j{}; j < candidate.polygonContributingTriangles[i].size(); ++j)
            {
                out << (j ? "," : "") << candidate.polygonContributingTriangles[i][j];
            }
            out << std::format("],\"geometry_source\":{},\"flags\":{},\"water\":{},\"preferred_path\":{}}}", sourceId,
                               tri.flags, (tri.flags & WaterFlag) != 0, (tri.flags & PreferredPathFlag) != 0);
        }
        out << "],\n  \"regions\": [";
        for (std::size_t i{}; i < candidate.regions.size(); ++i)
        {
            const auto &region = candidate.regions[i];
            out << (i ? "," : "") << std::format("{{\"id\":{},\"area\":{},\"polygons\":[", region.id, region.area);
            for (std::size_t j{}; j < region.polygons.size(); ++j)
            {
                out << (j ? "," : "") << region.polygons[j];
            }
            out << "],\"source_triangles\":[";
            for (std::size_t j{}; j < region.sourceTriangles.size(); ++j)
            {
                out << (j ? "," : "") << region.sourceTriangles[j];
            }
            out << "],\"geometry_sources\":[";
            for (std::size_t j{}; j < region.geometrySources.size(); ++j)
            {
                out << (j ? "," : "") << region.geometrySources[j];
            }
            out << "],\"reaches_border\":" << (region.reachesBorder ? "true" : "false") << ",\"exit_form_ids\":[";
            for (std::size_t j{}; j < region.exitFormIds.size(); ++j)
            {
                out << (j ? "," : "") << std::format("\"{:08X}\"", region.exitFormIds[j]);
            }
            out << "]}";
        }
        out << "],\n  \"exits\": [";
        for (std::size_t i{}; i < candidate.exits.size(); ++i)
        {
            const auto &exit = candidate.exits[i];
            out << (i ? "," : "")
                << std::format("{{\"reference_id\":\"{:08X}\",\"position\":[{},{},{}],\"region\":", exit.referenceId,
                               exit.position.x, exit.position.y, exit.position.z);
            if (exit.region)
            {
                out << *exit.region;
            }
            else
            {
                out << "null";
            }
            out << ",\"polygon\":";
            if (exit.polygon)
            {
                out << *exit.polygon;
            }
            else
            {
                out << "null";
            }
            out << '}';
        }
        out << "],\n  \"border_links\": [";
        for (std::size_t i{}; i < candidate.borderLinks.size(); ++i)
        {
            const auto &link = candidate.borderLinks[i];
            out << (i ? "," : "")
                << std::format("{{\"polygon\":{},\"edge\":{},\"neighbor_navmesh_id\":\"{:08X}\",\"neighbor_polygon\":{}"
                               ",\"neighbor_edge\":{}}}",
                               link.polygon, link.edge, link.neighborNavmeshId, link.neighborPolygon,
                               link.neighborEdge);
        }
        out << "],\n  \"contours\": [";
        for (std::size_t i{}; i < candidate.contours.size(); ++i)
        {
            const auto &contour = candidate.contours[i];
            out << (i ? "," : "")
                << std::format("{{\"region\":{},\"closed\":{},\"vertices\":[", contour.region,
                               contour.closed ? "true" : "false");
            for (std::size_t j{}; j < contour.vertices.size(); ++j)
            {
                out << (j ? "," : "") << contour.vertices[j];
            }
            out << "]}";
        }
        out << "],\n  \"geometry_sources\": [";
        for (std::size_t i{}; i < scene.geometrySources.size(); ++i)
        {
            const auto &source = scene.geometrySources[i];
            out << (i ? "," : "")
                << std::format("{{\"plugin\":\"{}\",\"form_id\":\"{:08X}\",\"record_type\":\"{}\",\"model\":\"{}\","
                               "\"type\":\"{}\",\"confidence\":{},\"navigation_obstacle\":{}}}",
                               reproducibility::EscapeJson(source.reference.plugin), source.reference.formId,
                               reproducibility::EscapeJson(source.reference.recordType),
                               reproducibility::EscapeJson(source.modelPath),
                               source.sourceType == GeometrySourceType::Terrain     ? "terrain"
                               : source.sourceType == GeometrySourceType::Collision ? "collision"
                                                                                    : "render_fallback",
                               source.confidence, source.navigationObstacle ? "true" : "false");
        }
        out << "],\n  \"source_triangles\": [";
        std::set<std::size_t> usedSources;
        for (const auto &contributors : candidate.polygonContributingTriangles)
        {
            usedSources.insert(contributors.begin(), contributors.end());
        }
        std::size_t emitted{};
        for (const auto sourceIndex : usedSources)
        {
            const auto &provenance = scene.triangleProvenance[sourceIndex];
            out << (emitted++ ? "," : "")
                << std::format("{{\"input_index\":{},\"geometry_source\":{},\"source_triangle\":{}", sourceIndex,
                               provenance.geometrySource, provenance.sourceTriangle);
            if (provenance.terrain)
            {
                const auto &terrain = *provenance.terrain;
                out << std::format(",\"terrain\":{{\"cell\":[{},{}],\"land_form_id\":\"{:08X}\",\"sample\":[{},{}]}}",
                                   terrain.cellX, terrain.cellY, terrain.landFormId, terrain.sampleX, terrain.sampleY);
            }
            out << '}';
        }
        out << "],\n  \"topology\": {\"valid\":" << (candidate.topology.valid ? "true" : "false") << ",\"findings\":[";
        for (std::size_t i{}; i < candidate.topology.findings.size(); ++i)
        {
            out << (i ? "," : "") << "\"" << reproducibility::EscapeJson(candidate.topology.findings[i]) << "\"";
        }
        out << "]},\n  \"warnings\": [";
        for (std::size_t i{}; i < candidate.warnings.size(); ++i)
        {
            out << (i ? "," : "") << "\"" << reproducibility::EscapeJson(candidate.warnings[i]) << "\"";
        }
        out << "]\n}\n";
        return out.good();
    }

    bool WriteCandidateObj(const std::filesystem::path &path, const CandidateNavMesh &candidate)
    {
        std::ofstream out(path, std::ios::trunc);
        if (!out)
        {
            return false;
        }
        out << "# neutral candidate NAVM; profile " << candidate.profile.name << "\n";
        for (const auto &vertex : candidate.mesh.vertices)
        {
            out << std::format("v {} {} {}\n", vertex.x, vertex.y, vertex.z);
        }
        for (const auto &region : candidate.regions)
        {
            out << "g candidate_region_" << region.id << "\n";
            for (const auto index : region.polygons)
            {
                const auto &tri = candidate.mesh.polygons[index];
                out << std::format("f {} {} {}\n", tri.vertices[0] + 1, tri.vertices[1] + 1, tri.vertices[2] + 1);
            }
        }
        return out.good();
    }
} // namespace navmesh::core
