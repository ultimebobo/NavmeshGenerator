#include "core/navmesh/ledge_links.h"
#include "analysis/navmesh_analysis.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <set>
#include <stdexcept>
#include <tuple>

namespace
{
    using namespace navmesh::core;
    constexpr auto NoNeighbor = std::numeric_limits<std::uint32_t>::max();

    float Dot(Vec3 a, Vec3 b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    Vec3 Cross(Vec3 a, Vec3 b)
    {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    }

    /// Bound vertical queries through the collision BVH; lateral floor search can issue many such rays.
    std::optional<navmesh::analysis::SurfaceHit> FloorBelow(const navmesh::analysis::SpatialIndex &index, Vec3 origin,
                                                            float distance)
    {
        AABB bounds{.min = {origin.x, origin.y, origin.z - distance}, .max = origin};
        std::optional<navmesh::analysis::SurfaceHit> found;
        for (const auto id : index.QueryAABB(bounds))
        {
            const auto &entry = index.entries[id];
            const auto a = index.vertices[entry.triangle.vertices[0]];
            const auto b = index.vertices[entry.triangle.vertices[1]];
            const auto c = index.vertices[entry.triangle.vertices[2]];
            const auto normal = Cross(b - a, c - a);
            if (std::abs(normal.z) < 1e-8F)
            {
                continue;
            }
            const auto u = ((static_cast<double>(b.y) - c.y) * (origin.x - c.x) +
                            (static_cast<double>(c.x) - b.x) * (origin.y - c.y)) /
                           normal.z;
            const auto v = ((static_cast<double>(c.y) - a.y) * (origin.x - c.x) +
                            (static_cast<double>(a.x) - c.x) * (origin.y - c.y)) /
                           normal.z;
            if (u < -1e-6 || v < -1e-6 || u + v > 1 + 1e-6)
            {
                continue;
            }
            const auto height = static_cast<float>(u * a.z + v * b.z + (1 - u - v) * c.z);
            const auto drop = origin.z - height;
            if (drop >= 0 && drop <= distance && (!found || drop < found->distance))
            {
                found = navmesh::analysis::SurfaceHit{{origin.x, origin.y, height}, normal, drop, entry.triangleIndex};
            }
        }
        return found;
    }

    /// Separating-axis triangle/box test also catches thin walls and ceilings between sampled endpoints.
    bool IntersectsBox(std::array<Vec3, 3> points, Vec3 minimum, Vec3 maximum)
    {
        const auto center = (minimum + maximum) * 0.5F;
        const auto half = (maximum - minimum) * 0.5F;
        for (auto &point : points)
        {
            point = point - center;
        }
        const auto separated = [&](Vec3 axis)
        {
            const auto radius = std::abs(axis.x) * half.x + std::abs(axis.y) * half.y + std::abs(axis.z) * half.z;
            const std::array projections{Dot(points[0], axis), Dot(points[1], axis), Dot(points[2], axis)};
            return *std::min_element(projections.begin(), projections.end()) > radius ||
                   *std::max_element(projections.begin(), projections.end()) < -radius;
        };
        constexpr std::array<Vec3, 3> axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        for (const auto axis : axes)
        {
            if (separated(axis))
            {
                return false;
            }
        }
        for (std::size_t edge{}; edge < 3; ++edge)
        {
            const auto direction = points[(edge + 1) % 3] - points[edge];
            for (const auto axis : axes)
            {
                if (separated(Cross(direction, axis)))
                {
                    return false;
                }
            }
        }
        return !separated(Cross(points[1] - points[0], points[2] - points[0]));
    }

    struct BoundaryEdge
    {
        std::uint32_t polygon{};
        /// Triangle edge index, or three for an interior receiving port.
        std::uint8_t edge{};
        Vec3 a, b, outward;
        /// External portal triangles cannot be repartitioned independently.
        bool canSplit{true};
    };

    /// Test the entire edge width: move outward above the lip, then descend beside its collision face.
    /// An upright square enclosing the actor's circular footprint makes the sweep conservative.
    bool ClearDrop(const CandidateNavMesh &candidate, const navmesh::analysis::SpatialIndex &collision,
                   const BoundaryEdge &boundary, Vec3 lowerA, Vec3 lowerB)
    {
        // Detail triangles approximate floor height within the movement profile's climb error.
        // Supporting collision supplies the swept foot planes within that approximation allowance.
        const auto support = [&](Vec3 point) -> std::optional<Vec3>
        {
            const auto allowance = candidate.profile.stepHeight + candidate.recastSettings.cellHeight * 2;
            auto origin = point;
            origin.z += allowance;
            const auto hit = FloorBelow(collision, origin, allowance * 2);
            if (!hit || std::abs(hit->point.z - point.z) > allowance ||
                hit->normal.z < std::sqrt(Dot(hit->normal, hit->normal)) *
                                    std::cos(candidate.profile.maxSlopeDegrees * std::numbers::pi_v<float> / 180))
            {
                return std::nullopt;
            }
            return hit->point;
        };
        const auto departureA = support(boundary.a);
        const auto departureB = support(boundary.b);
        const auto arrivalA = support(lowerA);
        const auto arrivalB = support(lowerB);
        if (!departureA || !departureB || !arrivalA || !arrivalB)
        {
            return false;
        }
        auto upper = boundary;
        upper.a = *departureA;
        upper.b = *departureB;
        lowerA = *arrivalA;
        lowerB = *arrivalB;
        const auto along = upper.b - upper.a;
        const auto length = std::hypot(along.x, along.y);
        const Vec3 tangent{along.x / length, along.y / length, 0};
        const auto local = [&](Vec3 point)
        {
            point = point - upper.a;
            return Vec3{Dot(point, tangent), Dot(point, upper.outward), point.z};
        };
        const auto first = local(lowerA);
        const auto last = local(lowerB);
        const auto radius = candidate.profile.agentRadius;
        const auto height = std::max(candidate.profile.agentHeight, candidate.profile.clearance);
        const auto epsilon = candidate.recastSettings.cellHeight * 2;
        const auto near = std::min(first.y, last.y);
        const auto far = std::max(first.y, last.y);
        const auto upperSlope = along.z / length;
        const auto lowerSlope = (last.z - first.z) / (last.x - first.x);
        const auto upperAboveLanding = [&](float x) { return x * upperSlope - (first.z + (x - first.x) * lowerSlope); };
        const auto descentHeight = std::max(upperAboveLanding(first.x), upperAboveLanding(last.x)) + height;
        if (descentHeight <= height + epsilon)
        {
            return false;
        }
        // Shearing follows the supporting slopes. Uneven floor within the walking climb is
        // traversable; reserve the same extra headroom so stepping cannot admit a low ceiling.
        const auto climb = candidate.profile.stepHeight;
        const Vec3 crossingMin{-radius, -radius, climb + epsilon};
        const Vec3 crossingMax{length + radius, far + radius, height + climb};
        const Vec3 descentMin{std::min(first.x, last.x) - radius, near - radius, climb + epsilon};
        const Vec3 descentMax{std::max(first.x, last.x) + radius, far + radius, descentHeight + climb};
        AABB query;
        for (const auto point : {upper.a, upper.b, lowerA, lowerB})
        {
            query.Expand(point - Vec3{radius * 2, radius * 2, 0});
            query.Expand(point + Vec3{radius * 2, radius * 2, height});
        }
        for (const auto index : collision.QueryAABB(query))
        {
            const auto &face = candidate.ledgeGeometry.triangles[index];
            std::array crossing{local(candidate.ledgeGeometry.vertices.at(face.vertices[0])),
                                local(candidate.ledgeGeometry.vertices.at(face.vertices[1])),
                                local(candidate.ledgeGeometry.vertices.at(face.vertices[2]))};
            auto descent = crossing;
            for (std::size_t vertex{}; vertex < crossing.size(); ++vertex)
            {
                crossing[vertex].z -= crossing[vertex].x * upperSlope;
                descent[vertex].z -= first.z + (descent[vertex].x - first.x) * lowerSlope;
            }
            if (IntersectsBox(crossing, crossingMin, crossingMax) || IntersectsBox(descent, descentMin, descentMax))
            {
                return false;
            }
        }
        return true;
    }

    /// A matched interval on two generated boundaries, before triangle subdivision assigns final edge indices.
    struct DropSpan
    {
        BoundaryEdge upper, lower;
        Vec3 upperA, upperB, lowerA, lowerB;
        float drop{};
    };

    /// Solid landings obey the NPC fall ceiling. Deep, covered water absorbs the fall before the bottom floor.
    bool SafeLanding(const CandidateNavMesh &candidate, Vec3 upperA, Vec3 upperB, Vec3 lowerA, Vec3 lowerB)
    {
        const auto margin = candidate.recastSettings.cellHeight * 2;
        if (std::max(upperA.z - lowerA.z, upperB.z - lowerB.z) + margin < candidate.profile.maximumDropHeight)
        {
            return true;
        }
        for (const auto fraction : {0.0F, 0.25F, 0.5F, 0.75F, 1.0F})
        {
            const auto point = lowerA + (lowerB - lowerA) * fraction;
            const auto water = CandidateWaterHeight(candidate, point);
            if (!water ||
                *water - point.z < std::max(candidate.profile.agentHeight, candidate.profile.clearance) + margin)
            {
                return false;
            }
        }
        return true;
    }

    /// Clip facing edges by their shared tangent projection; unrelated tessellation need not share endpoints.
    std::optional<DropSpan> OverlappingSpan(const BoundaryEdge &upper, const BoundaryEdge &lower,
                                            const CandidateNavMesh &candidate)
    {
        if (upper.polygon == lower.polygon || Dot(upper.outward, lower.outward) > -0.5F)
        {
            return std::nullopt;
        }
        const auto length = std::hypot(upper.b.x - upper.a.x, upper.b.y - upper.a.y);
        const Vec3 tangent{(upper.b.x - upper.a.x) / length, (upper.b.y - upper.a.y) / length, 0};
        const auto first = Dot(lower.b - upper.a, tangent);
        const auto last = Dot(lower.a - upper.a, tangent);
        if (last <= first)
        {
            return std::nullopt;
        }
        const auto start = std::max(0.0F, first);
        const auto end = std::min(length, last);
        if (end - start < candidate.recastSettings.cellSize * 2)
        {
            return std::nullopt;
        }
        DropSpan span{upper,
                      lower,
                      upper.a + (upper.b - upper.a) * (start / length),
                      upper.a + (upper.b - upper.a) * (end / length),
                      lower.b + (lower.a - lower.b) * ((start - first) / (last - first)),
                      lower.b + (lower.a - lower.b) * ((end - first) / (last - first))};
        const auto reach = std::max(candidate.profile.agentHeight, candidate.profile.agentRadius * 4) +
                           candidate.recastSettings.cellSize * 4;
        const auto firstGap = Dot(span.lowerA - span.upperA, upper.outward);
        const auto lastGap = Dot(span.lowerB - span.upperB, upper.outward);
        const auto firstDrop = span.upperA.z - span.lowerA.z;
        const auto lastDrop = span.upperB.z - span.lowerB.z;
        const auto margin = candidate.recastSettings.cellHeight * 2;
        if (std::min(firstGap, lastGap) <= 0 || std::max(firstGap, lastGap) > reach ||
            std::min(firstDrop, lastDrop) <= candidate.profile.stepHeight + margin ||
            !SafeLanding(candidate, span.upperA, span.upperB, span.lowerA, span.lowerB))
        {
            return std::nullopt;
        }
        const auto complete = [&](const BoundaryEdge &edge, Vec3 a, Vec3 b)
        {
            return Dot(a - edge.a, a - edge.a) <= candidate.profile.weldTolerance * candidate.profile.weldTolerance &&
                   Dot(b - edge.b, b - edge.b) <= candidate.profile.weldTolerance * candidate.profile.weldTolerance;
        };
        if ((!upper.canSplit && !complete(upper, span.upperA, span.upperB)) ||
            (!lower.canSplit && !complete(lower, span.lowerB, span.lowerA)))
        {
            return std::nullopt;
        }
        span.drop = std::max(firstDrop, lastDrop);
        return span;
    }

    /// Search clear subspans so a blocked endpoint cannot discard an otherwise usable aperture.
    void CollectClearSpans(const CandidateNavMesh &candidate, const navmesh::analysis::SpatialIndex &collision,
                           const DropSpan &span, std::vector<DropSpan> &output)
    {
        const BoundaryEdge upper{span.upper.polygon, span.upper.edge, span.upperA, span.upperB, span.upper.outward};
        if (ClearDrop(candidate, collision, upper, span.lowerA, span.lowerB))
        {
            // Every horizontal sample must have collision below it: a drop does not bridge a void.
            const auto distance = std::max(std::hypot(span.lowerA.x - span.upperA.x, span.lowerA.y - span.upperA.y),
                                           std::hypot(span.lowerB.x - span.upperB.x, span.lowerB.y - span.upperB.y));
            const auto samples = std::max(1, static_cast<int>(std::ceil(distance / candidate.recastSettings.cellSize)));
            bool supported = true;
            for (const auto fraction : {0.0F, 0.5F, 1.0F})
            {
                const auto start = span.upperA + (span.upperB - span.upperA) * fraction;
                const auto end = span.lowerA + (span.lowerB - span.lowerA) * fraction;
                for (int sample{}; sample <= samples; ++sample)
                {
                    auto point = start + (end - start) * (static_cast<float>(sample) / samples);
                    point.z = start.z + candidate.profile.stepHeight + candidate.recastSettings.cellHeight * 2;
                    const auto floor = FloorBelow(
                        collision, point,
                        std::max(candidate.profile.maximumDropHeight, span.drop + candidate.profile.stepHeight * 2 +
                                                                          candidate.recastSettings.cellHeight * 4));
                    if (!floor)
                    {
                        supported = false;
                        break;
                    }
                }
            }
            if (supported)
            {
                output.push_back(span);
                return;
            }
        }
        const auto width = std::hypot(span.upperB.x - span.upperA.x, span.upperB.y - span.upperA.y);
        if (width < candidate.recastSettings.cellSize * 4 || !span.upper.canSplit || !span.lower.canSplit)
        {
            return;
        }
        const auto upperMiddle = (span.upperA + span.upperB) * 0.5F;
        const auto lowerMiddle = (span.lowerA + span.lowerB) * 0.5F;
        auto half = span;
        half.upperB = upperMiddle;
        half.lowerB = lowerMiddle;
        CollectClearSpans(candidate, collision, half, output);
        half = span;
        half.upperA = upperMiddle;
        half.lowerA = lowerMiddle;
        CollectClearSpans(candidate, collision, half, output);
    }

    struct EdgeInterval
    {
        BoundaryEdge edge;
        Vec3 a, b;
    };

    float HorizontalArea(Vec3 a, Vec3 b, Vec3 c)
    {
        return static_cast<float>((static_cast<double>(b.x) - a.x) * (static_cast<double>(c.y) - a.y) -
                                  (static_cast<double>(b.y) - a.y) * (static_cast<double>(c.x) - a.x));
    }

    /// Interpolate a landing strictly inside one retained floor triangle, keeping its entire span on that plane.
    std::optional<Vec3> InteriorFloorPoint(Vec3 point, const NavMesh &mesh, std::uint32_t polygon, float inset)
    {
        const auto &face = mesh.polygons[polygon];
        const auto a = mesh.vertices[face.vertices[0]];
        const auto b = mesh.vertices[face.vertices[1]];
        const auto c = mesh.vertices[face.vertices[2]];
        const auto area = HorizontalArea(a, b, c);
        if (area <= 0)
        {
            return std::nullopt;
        }
        for (std::size_t edge{}; edge < 3; ++edge)
        {
            const auto first = mesh.vertices[face.vertices[edge]];
            const auto last = mesh.vertices[face.vertices[(edge + 1) % 3]];
            if (HorizontalArea(first, last, point) <= inset * std::hypot(last.x - first.x, last.y - first.y))
            {
                return std::nullopt;
            }
        }
        point.z = (HorizontalArea(point, b, c) * a.z + HorizontalArea(a, point, c) * b.z +
                   HorizontalArea(a, b, point) * c.z) /
                  area;
        return point;
    }

    /** Discover touchdown spans inside lower floors, independently of their boundary tessellation.
     * The receiving edge is inserted as an internal port; its adjacent floor remains connected around the port ends.
     */
    void CollectInteriorLandings(const CandidateNavMesh &candidate, const navmesh::analysis::SpatialIndex &collision,
                                 const std::vector<BoundaryEdge> &edges, const std::vector<std::uint32_t> &components,
                                 const std::set<std::uint32_t> &portalPolygons, std::vector<DropSpan> &proposals)
    {
        std::vector<Triangle> triangles;
        for (const auto &face : candidate.mesh.polygons)
        {
            triangles.push_back({face.vertices});
        }
        navmesh::analysis::SpatialIndex floors;
        floors.Build(triangles, candidate.mesh.vertices);
        const auto minimumFloor = std::min_element(candidate.mesh.vertices.begin(), candidate.mesh.vertices.end(),
                                                   [](Vec3 a, Vec3 b) { return a.z < b.z; })
                                      ->z;
        const auto stride = std::max(candidate.profile.agentRadius, candidate.recastSettings.cellSize * 2);
        const auto halfWidth = candidate.recastSettings.cellSize;
        const auto reach = std::max(candidate.profile.agentHeight, candidate.profile.agentRadius * 4) +
                           candidate.recastSettings.cellSize * 4;
        const auto margin = candidate.recastSettings.cellHeight * 2;
        for (const auto &upper : edges)
        {
            if (!upper.canSplit)
            {
                continue;
            }
            const auto length = std::hypot(upper.b.x - upper.a.x, upper.b.y - upper.a.y);
            const auto segments = std::max(1, static_cast<int>(std::ceil(length / stride)));
            const Vec3 tangent{(upper.b.x - upper.a.x) / length, (upper.b.y - upper.a.y) / length,
                               (upper.b.z - upper.a.z) / length};
            for (int sample{}; sample < segments; ++sample)
            {
                const auto position = length * (sample + 0.5F) / segments;
                if (position < halfWidth || position + halfWidth > length)
                {
                    continue;
                }
                const auto start = upper.a + tangent * position;
                const auto searchReach = CandidateWaterHeight(candidate, start)
                                             ? std::max(reach, std::sqrt(std::max(0.0F, start.z - minimumFloor) *
                                                                         candidate.profile.agentHeight))
                                             : reach;
                for (auto distance = stride; distance <= searchReach; distance += stride)
                {
                    auto query = start + upper.outward * distance;
                    query.z -= candidate.profile.stepHeight + margin;
                    const auto depth =
                        candidate.ledgeWaterGeometry.triangles.empty() && !candidate.ledgeWaterHeight
                            ? candidate.profile.maximumDropHeight
                            : std::max(candidate.profile.maximumDropHeight, query.z - minimumFloor + margin);
                    const auto hit = FloorBelow(floors, query, depth);
                    if (!hit || components[upper.polygon] == components[hit->triangleIndex] ||
                        portalPolygons.contains(static_cast<std::uint32_t>(hit->triangleIndex)))
                    {
                        continue;
                    }
                    const auto lower = static_cast<std::uint32_t>(hit->triangleIndex);
                    const auto first = InteriorFloorPoint(query - tangent * halfWidth, candidate.mesh, lower,
                                                          candidate.profile.weldTolerance * 2);
                    const auto last = InteriorFloorPoint(query + tangent * halfWidth, candidate.mesh, lower,
                                                         candidate.profile.weldTolerance * 2);
                    if (!first || !last)
                    {
                        continue;
                    }
                    const auto firstDrop = (start - tangent * halfWidth).z - first->z;
                    const auto lastDrop = (start + tangent * halfWidth).z - last->z;
                    if (std::min(firstDrop, lastDrop) <= candidate.profile.stepHeight + margin ||
                        !SafeLanding(candidate, start - tangent * halfWidth, start + tangent * halfWidth, *first,
                                     *last))
                    {
                        continue;
                    }
                    const BoundaryEdge landing{lower, 3, *last, *first, upper.outward * -1};
                    const DropSpan span{upper,  landing, start - tangent * halfWidth,  start + tangent * halfWidth,
                                        *first, *last,   std::max(firstDrop, lastDrop)};
                    CollectClearSpans(candidate, collision, span, proposals);
                }
            }
        }
    }

    /// Partition only exposed edges. Original floor planes, local shared edges and polygon evidence are retained.
    std::vector<std::pair<std::uint32_t, std::uint8_t>> PartitionDropEdges(CandidateNavMesh &candidate,
                                                                           const std::vector<EdgeInterval> &intervals)
    {
        const auto original = candidate.mesh.polygons;
        std::map<std::pair<std::uint32_t, std::uint8_t>, std::vector<Vec3>> cuts;
        std::map<std::uint32_t, EdgeInterval> ports;
        for (const auto &interval : intervals)
        {
            if (interval.edge.edge == 3)
            {
                ports.emplace(interval.edge.polygon, interval);
                continue;
            }
            auto &points = cuts[{interval.edge.polygon, interval.edge.edge}];
            points.push_back(interval.a);
            points.push_back(interval.b);
        }
        std::vector<std::vector<std::uint32_t>> replacements(original.size());
        const auto toleranceSquared = candidate.profile.weldTolerance * candidate.profile.weldTolerance;
        const auto near = [&](Vec3 a, Vec3 b) { return Dot(a - b, a - b) <= toleranceSquared; };
        for (std::uint32_t polygon{}; polygon < original.size(); ++polygon)
        {
            const auto face = original[polygon];
            if (const auto port = ports.find(polygon); port != ports.end())
            {
                // Two interior vertices create a landing edge without removing any physical floor area.
                const auto first = static_cast<std::uint32_t>(candidate.mesh.vertices.size());
                candidate.mesh.vertices.push_back(port->second.a);
                const auto last = static_cast<std::uint32_t>(candidate.mesh.vertices.size());
                candidate.mesh.vertices.push_back(port->second.b);
                std::vector<std::uint32_t> boundary;
                for (std::uint8_t edge{}; edge < 3; ++edge)
                {
                    const auto a = candidate.mesh.vertices[face.vertices[edge]];
                    const auto b = candidate.mesh.vertices[face.vertices[(edge + 1) % 3]];
                    auto points = cuts[{polygon, edge}];
                    std::sort(points.begin(), points.end(), [&](Vec3 firstPoint, Vec3 lastPoint)
                              { return Dot(firstPoint - a, b - a) < Dot(lastPoint - a, b - a); });
                    boundary.push_back(face.vertices[edge]);
                    for (const auto point : points)
                    {
                        if (!near(point, a) && !near(point, b) &&
                            !near(point, candidate.mesh.vertices[boundary.back()]))
                        {
                            boundary.push_back(static_cast<std::uint32_t>(candidate.mesh.vertices.size()));
                            candidate.mesh.vertices.push_back(point);
                        }
                    }
                }
                std::vector<std::array<std::uint32_t, 3>> triangles;
                for (std::size_t edge{}; edge < boundary.size(); ++edge)
                {
                    triangles.push_back({boundary[edge], boundary[(edge + 1) % boundary.size()], first});
                }
                std::vector<std::array<std::uint32_t, 3>> partition;
                for (const auto &triangle : triangles)
                {
                    bool inside = true;
                    std::optional<std::size_t> onEdge;
                    for (std::size_t edge{}; edge < 3; ++edge)
                    {
                        const auto a = candidate.mesh.vertices[triangle[edge]];
                        const auto b = candidate.mesh.vertices[triangle[(edge + 1) % 3]];
                        const auto area = HorizontalArea(a, b, port->second.b);
                        const auto tolerance = candidate.profile.weldTolerance * std::hypot(b.x - a.x, b.y - a.y);
                        inside = inside && area >= -tolerance;
                        if (std::abs(area) <= tolerance)
                        {
                            onEdge = edge;
                        }
                    }
                    if (!inside)
                    {
                        partition.push_back(triangle);
                    }
                    else if (onEdge)
                    {
                        const auto edge = *onEdge;
                        partition.push_back({triangle[edge], last, triangle[(edge + 2) % 3]});
                        partition.push_back({last, triangle[(edge + 1) % 3], triangle[(edge + 2) % 3]});
                    }
                    else
                    {
                        for (std::size_t edge{}; edge < 3; ++edge)
                        {
                            partition.push_back({triangle[edge], triangle[(edge + 1) % 3], last});
                        }
                    }
                }
                const auto source = candidate.polygonSourceTriangles[polygon];
                const auto contributors = candidate.polygonContributingTriangles[polygon];
                for (std::size_t index{}; index < partition.size(); ++index)
                {
                    auto child = face;
                    child.vertices = partition[index];
                    const auto id = index == 0 ? polygon : static_cast<std::uint32_t>(candidate.mesh.polygons.size());
                    if (index == 0)
                    {
                        candidate.mesh.polygons[polygon] = child;
                    }
                    else
                    {
                        candidate.mesh.polygons.push_back(child);
                        candidate.polygonSourceTriangles.push_back(source);
                        candidate.polygonContributingTriangles.push_back(contributors);
                    }
                    replacements[polygon].push_back(id);
                }
                continue;
            }
            std::vector<std::pair<std::uint32_t, std::uint8_t>> boundary;
            bool split = false;
            for (std::uint8_t edge{}; edge < 3; ++edge)
            {
                const auto a = candidate.mesh.vertices[face.vertices[edge]];
                const auto b = candidate.mesh.vertices[face.vertices[(edge + 1) % 3]];
                auto points = cuts[{polygon, edge}];
                std::sort(points.begin(), points.end(),
                          [&](Vec3 first, Vec3 last) { return Dot(first - a, b - a) < Dot(last - a, b - a); });
                boundary.emplace_back(face.vertices[edge], edge);
                for (const auto point : points)
                {
                    if (near(point, a) || near(point, b) || near(point, candidate.mesh.vertices[boundary.back().first]))
                    {
                        continue;
                    }
                    boundary.emplace_back(static_cast<std::uint32_t>(candidate.mesh.vertices.size()), edge);
                    candidate.mesh.vertices.push_back(point);
                    split = true;
                }
            }
            if (!split)
            {
                replacements[polygon].push_back(polygon);
                continue;
            }
            const auto center = static_cast<std::uint32_t>(candidate.mesh.vertices.size());
            const auto a = candidate.mesh.vertices[face.vertices[0]];
            const auto b = candidate.mesh.vertices[face.vertices[1]];
            const auto c = candidate.mesh.vertices[face.vertices[2]];
            candidate.mesh.vertices.push_back({static_cast<float>((static_cast<double>(a.x) + b.x + c.x) / 3),
                                               static_cast<float>((static_cast<double>(a.y) + b.y + c.y) / 3),
                                               static_cast<float>((static_cast<double>(a.z) + b.z + c.z) / 3)});
            const auto source = candidate.polygonSourceTriangles[polygon];
            const auto contributors = candidate.polygonContributingTriangles[polygon];
            for (std::size_t index{}; index < boundary.size(); ++index)
            {
                auto child = face;
                child.vertices = {boundary[index].first, boundary[(index + 1) % boundary.size()].first, center};
                const auto id = index == 0 ? polygon : static_cast<std::uint32_t>(candidate.mesh.polygons.size());
                if (index == 0)
                {
                    candidate.mesh.polygons[polygon] = child;
                }
                else
                {
                    candidate.mesh.polygons.push_back(child);
                    candidate.polygonSourceTriangles.push_back(source);
                    candidate.polygonContributingTriangles.push_back(contributors);
                }
                replacements[polygon].push_back(id);
            }
        }
        for (auto &region : candidate.regions)
        {
            std::vector<std::uint32_t> polygons;
            for (const auto polygon : region.polygons)
            {
                polygons.insert(polygons.end(), replacements[polygon].begin(), replacements[polygon].end());
            }
            region.polygons = std::move(polygons);
        }
        for (auto &exit : candidate.exits)
        {
            if (exit.polygon)
            {
                // Every child remains on the same connected floor; keep the closest child centroid as the door anchor.
                auto best = std::numeric_limits<float>::max();
                auto selected = *exit.polygon;
                for (const auto child : replacements[*exit.polygon])
                {
                    const auto &face = candidate.mesh.polygons[child];
                    const auto point =
                        (candidate.mesh.vertices[face.vertices[0]] + candidate.mesh.vertices[face.vertices[1]] +
                         candidate.mesh.vertices[face.vertices[2]]) /
                        3.0F;
                    const auto distance = Dot(point - exit.position, point - exit.position);
                    if (distance < best)
                    {
                        best = distance;
                        selected = child;
                    }
                }
                exit.polygon = selected;
            }
        }
        std::vector<std::pair<std::uint32_t, std::uint8_t>> result;
        for (const auto &interval : intervals)
        {
            bool found = false;
            for (const auto polygon : replacements[interval.edge.polygon])
            {
                const auto &face = candidate.mesh.polygons[polygon];
                for (std::uint8_t edge{}; edge < 3; ++edge)
                {
                    if (near(candidate.mesh.vertices[face.vertices[edge]], interval.a) &&
                        near(candidate.mesh.vertices[face.vertices[(edge + 1) % 3]], interval.b))
                    {
                        result.emplace_back(polygon, edge);
                        found = true;
                    }
                }
            }
            if (!found)
            {
                throw std::runtime_error("A generated drop span could not be represented by a boundary edge");
            }
        }
        return result;
    }
} // namespace

namespace navmesh::core
{
    void CaptureLedgeGeometry(CandidateNavMesh &candidate, const Scene &scene, std::optional<float> waterHeight)
    {
        if (waterHeight && !std::isfinite(*waterHeight))
        {
            throw std::invalid_argument("Drop geometry requires a finite water height");
        }
        candidate.ledgeWaterGeometry = scene.waterSurfaces;
        candidate.ledgeWaterHeight = waterHeight;
        candidate.ledgeGeometry = {};
        if (candidate.mesh.vertices.empty() || !scene.HasCompleteTriangleProvenance())
        {
            return;
        }
        AABB bounds;
        for (const auto point : candidate.mesh.vertices)
        {
            bounds.Expand(point);
        }
        const auto margin = candidate.profile.agentRadius * 3 + candidate.recastSettings.cellSize * 2;
        bounds.min = bounds.min - Vec3{margin, margin, candidate.recastSettings.cellHeight * 2};
        bounds.max =
            bounds.max + Vec3{margin, margin, std::max(candidate.profile.agentHeight, candidate.profile.clearance)};
        std::map<std::uint32_t, std::uint32_t> vertices;
        for (std::size_t index{}; index < scene.mesh.triangles.size(); ++index)
        {
            const auto &source = scene.geometrySources.at(scene.triangleProvenance[index].geometrySource);
            if (source.sourceType == GeometrySourceType::RenderFallback)
            {
                continue;
            }
            const auto &face = scene.mesh.triangles[index];
            AABB triangleBounds;
            for (const auto vertex : face.vertices)
            {
                triangleBounds.Expand(scene.mesh.vertices.at(vertex));
            }
            if (!bounds.Intersects(triangleBounds))
            {
                continue;
            }
            Triangle copied;
            for (std::size_t corner{}; corner < 3; ++corner)
            {
                const auto old = face.vertices[corner];
                const auto [entry, added] = vertices.emplace(old, static_cast<std::uint32_t>(vertices.size()));
                if (added)
                {
                    candidate.ledgeGeometry.vertices.push_back(scene.mesh.vertices[old]);
                }
                copied.vertices[corner] = entry->second;
            }
            candidate.ledgeGeometry.triangles.push_back(copied);
        }
    }

    std::optional<float> CandidateWaterHeight(const CandidateNavMesh &candidate, Vec3 point)
    {
        auto height = candidate.ledgeWaterHeight;
        const auto &mesh = candidate.ledgeWaterGeometry;
        for (const auto &triangle : mesh.triangles)
        {
            const auto a = mesh.vertices[triangle.vertices[0]];
            const auto b = mesh.vertices[triangle.vertices[1]];
            const auto c = mesh.vertices[triangle.vertices[2]];
            const auto area = HorizontalArea(a, b, c);
            if (area == 0)
            {
                continue;
            }
            const auto u = HorizontalArea(point, b, c) / area;
            const auto v = HorizontalArea(a, point, c) / area;
            if (u >= 0 && v >= 0 && u + v <= 1)
            {
                const auto z = u * a.z + v * b.z + (1 - u - v) * c.z;
                if (!height || z > *height)
                {
                    height = z;
                }
            }
        }
        return height;
    }

    /// Plan on a private candidate so optional partition failures never expose partially edited floors.
    static std::size_t BuildCandidateLedgeLinksImpl(CandidateNavMesh &candidate)
    {
        const auto existingLinks = candidate.ledgeLinks;
        candidate.ledgeLinks.clear();
        if (!existingLinks.empty())
        {
            RefreshCandidateTopology(candidate);
        }
        if (candidate.ledgeGeometry.triangles.empty() ||
            candidate.profile.maximumDropHeight <= candidate.profile.stepHeight)
        {
            return 0;
        }
        std::set<std::pair<std::uint32_t, std::uint8_t>> occupied;
        for (const auto &link : candidate.borderLinks)
        {
            occupied.emplace(link.polygon, link.edge);
        }
        std::set<std::uint32_t> portalPolygons;
        for (const auto &link : candidate.borderLinks)
        {
            portalPolygons.insert(link.polygon);
        }
        // Ordinary floor paths already supply reversible traversal; only separate floors need drop edges.
        std::vector<std::uint32_t> components(candidate.mesh.polygons.size(), NoNeighbor);
        for (std::uint32_t polygon{}; polygon < candidate.mesh.polygons.size(); ++polygon)
        {
            if (components[polygon] != NoNeighbor)
            {
                continue;
            }
            std::vector<std::uint32_t> pending{polygon};
            components[polygon] = polygon;
            while (!pending.empty())
            {
                const auto current = pending.back();
                pending.pop_back();
                for (const auto neighbor : candidate.mesh.polygons[current].neighbors)
                {
                    if (neighbor != NoNeighbor && components[neighbor] == NoNeighbor)
                    {
                        components[neighbor] = polygon;
                        pending.push_back(neighbor);
                    }
                }
            }
        }
        std::vector<BoundaryEdge> edges;
        for (std::uint32_t polygon{}; polygon < candidate.mesh.polygons.size(); ++polygon)
        {
            const auto &face = candidate.mesh.polygons[polygon];
            for (std::uint8_t edge{}; edge < 3; ++edge)
            {
                if (face.neighbors[edge] != NoNeighbor || occupied.contains({polygon, edge}))
                {
                    continue;
                }
                const auto a = candidate.mesh.vertices.at(face.vertices[edge]);
                const auto b = candidate.mesh.vertices.at(face.vertices[(edge + 1) % 3]);
                const auto length = std::hypot(b.x - a.x, b.y - a.y);
                if (length < candidate.recastSettings.cellSize * 2)
                {
                    continue;
                }
                Vec3 outward{(b.y - a.y) / length, (a.x - b.x) / length, 0};
                const auto interior = candidate.mesh.vertices.at(face.vertices[(edge + 2) % 3]);
                if (Dot(interior - a, outward) > 0)
                {
                    outward = outward * -1.0F;
                }
                edges.push_back({polygon, edge, a, b, outward, !portalPolygons.contains(polygon)});
            }
        }
        std::vector<DropSpan> proposals;
        std::vector<DropSpan> reusable;
        const auto reach = std::max(candidate.profile.agentHeight, candidate.profile.agentRadius * 4) +
                           candidate.recastSettings.cellSize * 4;
        navmesh::analysis::SpatialIndex collision;
        collision.Build(candidate.ledgeGeometry.triangles, candidate.ledgeGeometry.vertices);
        for (const auto &link : existingLinks)
        {
            if (link.upperPolygon >= candidate.mesh.polygons.size() ||
                link.lowerPolygon >= candidate.mesh.polygons.size() || link.upperEdge >= 3 || link.lowerEdge >= 3 ||
                components[link.upperPolygon] == components[link.lowerPolygon])
            {
                continue;
            }
            const auto &upperFace = candidate.mesh.polygons[link.upperPolygon];
            const auto &lowerFace = candidate.mesh.polygons[link.lowerPolygon];
            const auto a = candidate.mesh.vertices[upperFace.vertices[link.upperEdge]];
            const auto b = candidate.mesh.vertices[upperFace.vertices[(link.upperEdge + 1) % 3]];
            const auto c = candidate.mesh.vertices[lowerFace.vertices[(link.lowerEdge + 1) % 3]];
            const auto d = candidate.mesh.vertices[lowerFace.vertices[link.lowerEdge]];
            const auto length = std::hypot(b.x - a.x, b.y - a.y);
            const Vec3 outward{(b.y - a.y) / length, (a.x - b.x) / length, 0};
            if (SafeLanding(candidate, a, b, c, d))
            {
                CollectClearSpans(candidate, collision,
                                  {{link.upperPolygon, link.upperEdge, a, b, outward, false},
                                   {link.lowerPolygon, link.lowerEdge, d, c, outward * -1, false},
                                   a,
                                   b,
                                   c,
                                   d,
                                   std::max(a.z - c.z, b.z - d.z)},
                                  reusable);
            }
        }
        // Index whole edge bounds so partial overlaps remain discoverable on differently subdivided floors.
        const double bucketSize = reach;
        const auto bucket = [&](const BoundaryEdge &edge)
        {
            return std::array{std::floor(std::min(edge.a.x, edge.b.x) / bucketSize),
                              std::floor(std::min(edge.a.y, edge.b.y) / bucketSize),
                              std::floor(std::max(edge.a.x, edge.b.x) / bucketSize),
                              std::floor(std::max(edge.a.y, edge.b.y) / bucketSize)};
        };
        std::map<std::pair<double, double>, std::vector<std::size_t>> nearby;
        for (std::size_t index{}; index < edges.size(); ++index)
        {
            const auto bounds = bucket(edges[index]);
            for (auto x = bounds[0]; x <= bounds[2]; ++x)
            {
                for (auto y = bounds[1]; y <= bounds[3]; ++y)
                {
                    nearby[{x, y}].push_back(index);
                }
            }
        }
        for (const auto &upper : edges)
        {
            const auto bounds = bucket(upper);
            std::set<std::size_t> lowerEdges;
            for (auto x = bounds[0] - 1; x <= bounds[2] + 1; ++x)
            {
                for (auto y = bounds[1] - 1; y <= bounds[3] + 1; ++y)
                {
                    const auto found = nearby.find({x, y});
                    if (found != nearby.end())
                    {
                        lowerEdges.insert(found->second.begin(), found->second.end());
                    }
                }
            }
            for (const auto index : lowerEdges)
            {
                const auto &lower = edges[index];
                if (components[upper.polygon] == components[lower.polygon])
                {
                    continue;
                }
                if (const auto span = OverlappingSpan(upper, lower, candidate))
                {
                    CollectClearSpans(candidate, collision, *span, proposals);
                }
            }
        }
        CollectInteriorLandings(candidate, collision, edges, components, portalPolygons, proposals);
        // Allocate scarce receiving edges to floors with few departure apertures before broad floors consume them.
        std::map<std::uint32_t, std::set<std::pair<std::uint32_t, std::uint8_t>>> apertures;
        for (const auto &proposal : proposals)
        {
            apertures[components[proposal.upper.polygon]].emplace(proposal.upper.polygon, proposal.upper.edge);
        }
        std::stable_sort(proposals.begin(), proposals.end(),
                         [&](const auto &a, const auto &b)
                         {
                             return std::pair{apertures.at(components[a.upper.polygon]).size(), a.drop} <
                                    std::pair{apertures.at(components[b.upper.polygon]).size(), b.drop};
                         });
        proposals.insert(proposals.begin(), reusable.begin(), reusable.end());
        std::vector<DropSpan> selected;
        std::vector<EdgeInterval> intervals;
        std::set<std::uint32_t> interiorPolygons;
        std::map<std::uint32_t, std::vector<std::uint32_t>> descents;
        const auto alreadyReachable = [&](std::uint32_t start, std::uint32_t destination)
        {
            std::set<std::uint32_t> seen;
            std::vector<std::uint32_t> pending{start};
            while (!pending.empty())
            {
                const auto current = pending.back();
                pending.pop_back();
                if (current == destination)
                {
                    return true;
                }
                if (!seen.insert(current).second)
                {
                    continue;
                }
                if (const auto links = descents.find(current); links != descents.end())
                {
                    pending.insert(pending.end(), links->second.begin(), links->second.end());
                }
            }
            return false;
        };
        for (const auto &proposal : proposals)
        {
            const auto upper = std::pair{proposal.upper.polygon, proposal.upper.edge};
            const auto lower = std::pair{proposal.lower.polygon, proposal.lower.edge};
            if (alreadyReachable(components[proposal.upper.polygon], components[proposal.lower.polygon]))
            {
                continue;
            }
            if (!occupied.contains(upper) && !occupied.contains(lower) &&
                (proposal.lower.edge != 3 || !interiorPolygons.contains(proposal.lower.polygon)))
            {
                selected.push_back(proposal);
                intervals.push_back({proposal.upper, proposal.upperA, proposal.upperB});
                intervals.push_back({proposal.lower, proposal.lowerB, proposal.lowerA});
                occupied.insert(upper);
                occupied.insert(lower);
                descents[components[proposal.upper.polygon]].push_back(components[proposal.lower.polygon]);
                if (proposal.lower.edge == 3)
                {
                    interiorPolygons.insert(proposal.lower.polygon);
                }
            }
        }
        if (selected.empty())
        {
            return 0;
        }
        const auto partition = [&](const std::vector<EdgeInterval> &spans)
        {
            auto trial = candidate;
            const auto linkedEdges = PartitionDropEdges(trial, spans);
            for (std::size_t index{}; index < spans.size() / 2; ++index)
            {
                const auto [upperPolygon, upperEdge] = linkedEdges[index * 2];
                const auto [lowerPolygon, lowerEdge] = linkedEdges[index * 2 + 1];
                trial.ledgeLinks.push_back({upperPolygon, lowerPolygon, upperEdge, lowerEdge});
            }
            RepairCandidateBookkeeping(trial);
            if (!trial.topology.valid)
            {
                throw std::runtime_error(trial.topology.findings.front());
            }
            return trial;
        };
        try
        {
            candidate = partition(intervals);
        }
        catch (const std::runtime_error &)
        {
            // Retry cumulative accepted proposals in original polygon space. Each trial starts from the same
            // floor, so failed cuts cannot corrupt later proposals or shift their source indices.
            std::vector<EdgeInterval> accepted;
            auto retained = candidate;
            std::vector<std::string> warnings;
            for (std::size_t index{}; index < selected.size(); ++index)
            {
                auto proposed = accepted;
                proposed.push_back(intervals[index * 2]);
                proposed.push_back(intervals[index * 2 + 1]);
                try
                {
                    retained = partition(proposed);
                    accepted = std::move(proposed);
                }
                catch (const std::runtime_error &error)
                {
                    warnings.push_back("Dropped generated ledge from polygon " +
                                       std::to_string(selected[index].upper.polygon) + " to polygon " +
                                       std::to_string(selected[index].lower.polygon) + ": " + error.what());
                }
            }
            retained.warnings.insert(retained.warnings.end(), warnings.begin(), warnings.end());
            candidate = std::move(retained);
        }
        return candidate.ledgeLinks.size();
    }

    std::size_t BuildCandidateLedgeLinks(CandidateNavMesh &candidate)
    {
        auto revised = candidate;
        auto existing = std::move(revised.ledgeLinks);
        revised.ledgeLinks.clear();
        RepairCandidateBookkeeping(revised);
        if (!revised.topology.valid)
        {
            throw std::invalid_argument("Drop construction requires valid floor topology: " +
                                        revised.topology.findings.front());
        }
        revised.ledgeLinks = std::move(existing);
        const auto count = BuildCandidateLedgeLinksImpl(revised);
        candidate = std::move(revised);
        return count;
    }
} // namespace navmesh::core
