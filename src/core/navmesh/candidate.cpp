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

namespace navmesh::core
{
    std::size_t StitchCandidateBorders(CandidateNavMesh &candidate, const AABB &cellBounds,
                                       const std::vector<NavMesh> &neighbors)
    {
        if (candidate.mesh.polygons.empty())
        {
            return 0;
        }
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
            if (std::abs(a.x - cellBounds.min.x) <= tolerance && std::abs(b.x - cellBounds.min.x) <= tolerance)
            {
                return 0;
            }
            if (std::abs(a.x - cellBounds.max.x) <= tolerance && std::abs(b.x - cellBounds.max.x) <= tolerance)
            {
                return 1;
            }
            if (std::abs(a.y - cellBounds.min.y) <= tolerance && std::abs(b.y - cellBounds.min.y) <= tolerance)
            {
                return 2;
            }
            if (std::abs(a.y - cellBounds.max.y) <= tolerance && std::abs(b.y - cellBounds.max.y) <= tolerance)
            {
                return 3;
            }
            return -1;
        };
        std::set<std::tuple<std::uint32_t, std::uint32_t, std::uint8_t>> used;
        const auto originalCount = candidate.mesh.polygons.size();
        for (std::uint32_t polygon{}; polygon < originalCount; ++polygon)
        {
            for (std::uint8_t side{}; side < 3; ++side)
            {
                const auto face = candidate.mesh.polygons[polygon];
                if (face.neighbors[side] != NoNeighbor)
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
                                used.contains({neighbor.id, other, otherSide}))
                            {
                                continue;
                            }
                            auto c = neighbor.vertices[triangle.vertices[otherSide]];
                            auto d = neighbor.vertices[triangle.vertices[(otherSide + 1) % 3]];
                            if (border(c, d, 1.0F) != cellSide)
                            {
                                continue;
                            }
                            if (distance(a, d) + distance(b, c) < distance(a, c) + distance(b, d))
                            {
                                std::swap(c, d);
                            }
                            const auto gapA = distance(a, c), gapB = distance(b, d);
                            if (gapA > maximumGap || gapB > maximumGap ||
                                std::abs(a.z - c.z) > candidate.profile.stepHeight ||
                                std::abs(b.z - d.z) > candidate.profile.stepHeight || Cross2(a, c, b) <= 0.01F ||
                                Cross2(b, c, d) <= 0.01F)
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
        std::vector<bool> portalPolygons(candidate.mesh.polygons.size());
        for (const auto &link : candidate.borderLinks)
        {
            if (link.polygon < portalPolygons.size())
            {
                portalPolygons[link.polygon] = true;
            }
        }
        for (const auto &exit : candidate.exits)
        {
            if (exit.polygon && *exit.polygon < portalPolygons.size())
            {
                portalPolygons[*exit.polygon] = true;
            }
        }
        std::vector<std::uint32_t> polygonRemap(candidate.mesh.polygons.size(), NoNeighbor);
        std::map<std::uint32_t, std::uint32_t> regionRemap;
        std::vector<NavPolygon> keptPolygons;
        std::vector<std::size_t> keptSources;
        std::vector<std::vector<std::size_t>> keptContributors;
        std::vector<CandidateRegion> keptRegions;
        std::size_t removedPolygons{};
        for (auto region : candidate.regions)
        {
            const bool connected = std::any_of(region.polygons.begin(), region.polygons.end(), [&](auto polygon)
                                               { return polygon < portalPolygons.size() && portalPolygons[polygon]; });
            if (!connected)
            {
                candidate.statistics.rejectedUnreachable += region.polygons.size();
                removedPolygons += region.polygons.size();
                continue;
            }
            const auto originalId = region.id;
            region.id = static_cast<std::uint32_t>(keptRegions.size());
            regionRemap.emplace(originalId, region.id);
            for (auto &polygon : region.polygons)
            {
                const auto old = polygon;
                polygon = static_cast<std::uint32_t>(keptPolygons.size());
                polygonRemap[old] = polygon;
                keptPolygons.push_back(candidate.mesh.polygons[old]);
                keptSources.push_back(candidate.polygonSourceTriangles[old]);
                keptContributors.push_back(std::move(candidate.polygonContributingTriangles[old]));
            }
            keptRegions.push_back(std::move(region));
        }
        candidate.mesh.polygons = std::move(keptPolygons);
        candidate.polygonSourceTriangles = std::move(keptSources);
        candidate.polygonContributingTriangles = std::move(keptContributors);
        candidate.regions = std::move(keptRegions);
        std::vector<std::uint32_t> vertexRemap(candidate.mesh.vertices.size(), NoNeighbor);
        std::vector<Vec3> keptVertices;
        for (auto &polygon : candidate.mesh.polygons)
        {
            for (auto &vertex : polygon.vertices)
            {
                if (vertexRemap[vertex] == NoNeighbor)
                {
                    vertexRemap[vertex] = static_cast<std::uint32_t>(keptVertices.size());
                    keptVertices.push_back(candidate.mesh.vertices[vertex]);
                    vertex = vertexRemap[vertex];
                }
                else
                {
                    vertex = vertexRemap[vertex];
                }
            }
        }
        candidate.mesh.vertices = std::move(keptVertices);
        for (auto &link : candidate.borderLinks)
        {
            link.polygon = polygonRemap[link.polygon];
        }
        for (auto &exit : candidate.exits)
        {
            if (!exit.polygon || *exit.polygon >= polygonRemap.size() || polygonRemap[*exit.polygon] == NoNeighbor)
            {
                exit.polygon.reset();
                exit.region.reset();
                continue;
            }
            exit.polygon = polygonRemap[*exit.polygon];
            const auto found = exit.region ? regionRemap.find(*exit.region) : regionRemap.end();
            exit.region = found == regionRemap.end() ? std::nullopt : std::optional<std::uint32_t>{found->second};
        }
        std::vector<CandidateContour> keptContours;
        for (auto contour : candidate.contours)
        {
            const auto region = regionRemap.find(contour.region);
            if (region == regionRemap.end() ||
                std::any_of(contour.vertices.begin(), contour.vertices.end(), [&](auto vertex)
                            { return vertex >= vertexRemap.size() || vertexRemap[vertex] == NoNeighbor; }))
            {
                continue;
            }
            contour.region = region->second;
            for (auto &vertex : contour.vertices)
            {
                vertex = vertexRemap[vertex];
            }
            keptContours.push_back(std::move(contour));
        }
        candidate.contours = std::move(keptContours);
        if (removedPolygons)
        {
            candidate.warnings.push_back("Removed candidate polygons without a matched door or adjacent NAVM portal.");
        }
        // Neighboring bridges can share an edge even when they target separate NAVM triangles.
        BuildAdjacency(candidate.mesh, candidate.profile.weldTolerance, candidate.profile.stepHeight);
        candidate.statistics.outputPolygons = candidate.mesh.polygons.size();
        candidate.topology = ValidateCandidateTopology(candidate);
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
        const auto &p = candidate.profile;
        out << "{\n  \"schema\": \"navmesh-generator/candidate-navm\",\n  \"schema_version\": \"1.1.0\",\n  "
               "\"metadata\": "
            << metadataJson << ",\n";
        out << std::format("  \"partitioning_algorithm\": \"{}\",\n", candidate.partitioningAlgorithm);
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
            out << std::format("],\"geometry_source\":{}}}", sourceId);
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
                               "\"type\":\"{}\",\"confidence\":{}}}",
                               reproducibility::EscapeJson(source.reference.plugin), source.reference.formId,
                               reproducibility::EscapeJson(source.reference.recordType),
                               reproducibility::EscapeJson(source.modelPath),
                               source.sourceType == GeometrySourceType::Terrain     ? "terrain"
                               : source.sourceType == GeometrySourceType::Collision ? "collision"
                                                                                    : "render_fallback",
                               source.confidence);
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
