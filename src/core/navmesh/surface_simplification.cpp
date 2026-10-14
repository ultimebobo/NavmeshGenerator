#include "core/navmesh/surface_simplification.h"
#include "core/navmesh/detail_triangulation.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <stdexcept>

namespace
{
    using namespace navmesh::core;
    using Edge = std::pair<std::uint32_t, std::uint32_t>;
    constexpr auto NoNeighbor = std::numeric_limits<std::uint32_t>::max();
    double Orientation(Vec3 a, Vec3 b, Vec3 c);

    struct BoundarySegment
    {
        Vec3 first;
        Vec3 last;
        std::uint32_t component{};
    };

    struct BoundaryReference
    {
        std::vector<BoundarySegment> segments;
        std::vector<std::uint32_t> components;
        std::vector<double> areas;
    };

    using EdgeUses = std::map<Edge, std::vector<std::uint32_t>>;

    EdgeUses MeshEdgeUses(const NavMesh &mesh)
    {
        EdgeUses edges;
        for (std::uint32_t index{}; index < mesh.polygons.size(); ++index)
        {
            const auto &face = mesh.polygons[index];
            for (std::size_t side{}; side < 3; ++side)
            {
                edges[std::minmax(face.vertices[side], face.vertices[(side + 1) % 3])].push_back(index);
            }
        }
        return edges;
    }

    /// Freeze exposed edges and shared-edge floor ownership before any inward trimming.
    BoundaryReference OriginalBoundary(const NavMesh &mesh)
    {
        BoundaryReference reference;
        reference.components.resize(mesh.polygons.size());
        std::iota(reference.components.begin(), reference.components.end(), 0U);
        const auto root = [&](std::uint32_t index)
        {
            while (reference.components[index] != index)
            {
                reference.components[index] = reference.components[reference.components[index]];
                index = reference.components[index];
            }
            return index;
        };
        const auto edges = MeshEdgeUses(mesh);
        for (const auto &[edge, uses] : edges)
        {
            if (uses.size() == 2)
            {
                reference.components[root(uses[1])] = root(uses[0]);
            }
        }
        for (std::uint32_t index{}; index < reference.components.size(); ++index)
        {
            reference.components[index] = root(index);
        }
        reference.areas.resize(mesh.polygons.size());
        for (std::size_t index{}; index < mesh.polygons.size(); ++index)
        {
            const auto &face = mesh.polygons[index];
            reference.areas[reference.components[index]] +=
                std::max(0.0, Orientation(mesh.vertices[face.vertices[0]], mesh.vertices[face.vertices[1]],
                                          mesh.vertices[face.vertices[2]]) *
                                  0.5);
        }
        for (const auto &[edge, uses] : edges)
        {
            if (uses.size() == 1)
            {
                reference.segments.push_back(
                    {mesh.vertices[edge.first], mesh.vertices[edge.second], reference.components[uses.front()]});
            }
        }
        return reference;
    }

    /// Double XY subdivision coordinates keep midpoint coverage independent of world-coordinate float spacing.
    struct FootprintPoint
    {
        double x;
        double y;
    };

    double SegmentDistanceSquared(FootprintPoint point, Vec3 a, Vec3 b)
    {
        const std::array<double, 2> direction{static_cast<double>(b.x) - a.x, static_cast<double>(b.y) - a.y};
        const std::array<double, 2> offset{static_cast<double>(point.x) - a.x, static_cast<double>(point.y) - a.y};
        double length{}, projection{};
        for (std::size_t axis{}; axis < 2; ++axis)
        {
            length += direction[axis] * direction[axis];
            projection += offset[axis] * direction[axis];
        }
        const auto fraction = length > 0 ? std::clamp(projection / length, 0.0, 1.0) : 0.0;
        double distance{};
        for (std::size_t axis{}; axis < 2; ++axis)
        {
            const auto delta = offset[axis] - fraction * direction[axis];
            distance += delta * delta;
        }
        return distance;
    }

    /** Cover the removed footprint with convex capsules around immutable boundary segments.
     * A subtriangle whose corners share a capsule lies wholly inside it. Subdivision lets
     * several segments jointly cover a thin tip without accepting an untested interior.
     * Ambiguous coverage at the subdivision limit is rejected, never estimated from corner samples.
     */
    bool CoveredByBoundaryBand(const std::array<FootprintPoint, 3> &triangle,
                               const std::vector<const BoundarySegment *> &segments, double squaredInset,
                               unsigned remainingSubdivisions)
    {
        for (const auto *segment : segments)
        {
            if (std::all_of(triangle.begin(), triangle.end(), [&](auto point)
                            { return SegmentDistanceSquared(point, segment->first, segment->last) <= squaredInset; }))
            {
                return true;
            }
        }
        if (remainingSubdivisions == 0 ||
            std::any_of(
                triangle.begin(), triangle.end(),
                [&](auto point)
                {
                    return std::none_of(
                        segments.begin(), segments.end(), [&](const auto *segment)
                        { return SegmentDistanceSquared(point, segment->first, segment->last) <= squaredInset; });
                }))
        {
            return false;
        }
        std::size_t longest{};
        double length{};
        for (std::size_t side{}; side < 3; ++side)
        {
            const auto a = triangle[side];
            const auto b = triangle[(side + 1) % 3];
            const auto dx = b.x - a.x;
            const auto dy = b.y - a.y;
            const auto squared = dx * dx + dy * dy;
            if (squared > length)
            {
                longest = side;
                length = squared;
            }
        }
        const auto a = triangle[longest];
        const auto b = triangle[(longest + 1) % 3];
        const auto c = triangle[(longest + 2) % 3];
        const FootprintPoint midpoint{std::midpoint(a.x, b.x), std::midpoint(a.y, b.y)};
        return CoveredByBoundaryBand({a, midpoint, c}, segments, squaredInset, remainingSubdivisions - 1) &&
               CoveredByBoundaryBand({midpoint, b, c}, segments, squaredInset, remainingSubdivisions - 1);
    }

    /// Component ownership prevents other floor layers from extending the original horizontal trim band.
    bool WithinOriginalBoundaryBand(const BoundaryReference &reference, const std::array<Vec3, 3> &points,
                                    std::uint32_t originalPolygon, float maximumInset)
    {
        const double squaredInset = static_cast<double>(maximumInset) * maximumInset;
        std::vector<const BoundarySegment *> segments;
        const double minX = std::min({points[0].x, points[1].x, points[2].x}) - static_cast<double>(maximumInset);
        const double maxX = std::max({points[0].x, points[1].x, points[2].x}) + static_cast<double>(maximumInset);
        const double minY = std::min({points[0].y, points[1].y, points[2].y}) - static_cast<double>(maximumInset);
        const double maxY = std::max({points[0].y, points[1].y, points[2].y}) + static_cast<double>(maximumInset);
        for (const auto &segment : reference.segments)
        {
            if (segment.component == reference.components[originalPolygon] &&
                std::max(segment.first.x, segment.last.x) >= minX &&
                std::min(segment.first.x, segment.last.x) <= maxX &&
                std::max(segment.first.y, segment.last.y) >= minY && std::min(segment.first.y, segment.last.y) <= maxY)
            {
                segments.push_back(&segment);
            }
        }
        std::array<FootprintPoint, 3> footprint;
        for (std::size_t index{}; index < points.size(); ++index)
        {
            footprint[index] = {points[index].x, points[index].y};
        }
        constexpr unsigned maximumBandSubdivisions = 10;
        return CoveredByBoundaryBand(footprint, segments, squaredInset, maximumBandSubdivisions);
    }

    double Orientation(Vec3 a, Vec3 b, Vec3 c)
    {
        return (static_cast<double>(b.x) - a.x) * (static_cast<double>(c.y) - a.y) -
               (static_cast<double>(b.y) - a.y) * (static_cast<double>(c.x) - a.x);
    }

    /// Compare only double arithmetic roundoff, never a movement- or voxel-sized height error.
    bool RoundoffZero(double value, double scale)
    {
        return std::abs(value) <= std::numeric_limits<double>::epsilon() * 32 * scale;
    }

    bool Coplanar(Vec3 a, Vec3 b, Vec3 c, Vec3 point)
    {
        const double bx = static_cast<double>(b.x) - a.x;
        const double by = static_cast<double>(b.y) - a.y;
        const double bz = static_cast<double>(b.z) - a.z;
        const double cx = static_cast<double>(c.x) - a.x;
        const double cy = static_cast<double>(c.y) - a.y;
        const double cz = static_cast<double>(c.z) - a.z;
        const double px = static_cast<double>(point.x) - a.x;
        const double py = static_cast<double>(point.y) - a.y;
        const double pz = static_cast<double>(point.z) - a.z;
        const double terms[]{by * cz * px, bz * cy * px, bz * cx * py, bx * cz * py, bx * cy * pz, by * cx * pz};
        double scale{};
        for (const auto term : terms)
        {
            scale += std::abs(term);
        }
        return RoundoffZero(terms[0] - terms[1] + terms[2] - terms[3] + terms[4] - terms[5], scale);
    }

    bool StraightBoundary(Vec3 a, Vec3 center, Vec3 b, float maximumLength)
    {
        // Exact horizontal collinearity prevents cutting corners or enlarging obstacle openings.
        if (Orientation(a, center, b) != 0)
        {
            return false;
        }
        const double dx = static_cast<double>(b.x) - a.x;
        const double dy = static_cast<double>(b.y) - a.y;
        const double dz = static_cast<double>(b.z) - a.z;
        const double px = static_cast<double>(center.x) - a.x;
        const double py = static_cast<double>(center.y) - a.y;
        const double pz = static_cast<double>(center.z) - a.z;
        const double squaredLength = dx * dx + dy * dy;
        const double projection = px * dx + py * dy;
        if (projection <= 0 || projection >= squaredLength ||
            (maximumLength > 0 && squaredLength > static_cast<double>(maximumLength) * maximumLength))
        {
            return false;
        }
        return RoundoffZero(pz * squaredLength - dz * projection,
                            std::abs(pz * squaredLength) + std::abs(dz * projection));
    }

    bool IsCellSeam(Vec3 point, const std::optional<AABB> &bounds)
    {
        return bounds && (point.x == bounds->min.x || point.x == bounds->max.x || point.y == bounds->min.y ||
                          point.y == bounds->max.y);
    }

    /** Prune small triangles with exposed edges, smallest first. A two-exposed-edge leaf
     * cannot disconnect the remaining floor. For a one-exposed-edge triangle, its opposite
     * vertex must be interior: the remaining incident fan then supplies a local alternate
     * shared-edge route. Boundary vertices on both sides of a narrow passage forbid removal.
     * Immutable boundary distance prevents repeated passes from peeling progressively deeper.
     */
    std::vector<std::uint32_t> PruneBoundaryTriangles(NavMesh &mesh, const SurfaceSimplificationSettings &settings,
                                                      const std::optional<AABB> &bounds,
                                                      const std::vector<bool> &pinned,
                                                      const BoundaryReference &reference,
                                                      std::vector<double> &removedAreas)
    {
        auto edges = MeshEdgeUses(mesh);
        std::vector<int> boundaryDegree(mesh.vertices.size());
        std::vector<bool> ambiguous(mesh.vertices.size());
        for (const auto &[edge, uses] : edges)
        {
            if (uses.size() == 1)
            {
                ++boundaryDegree[edge.first];
                ++boundaryDegree[edge.second];
            }
            else if (uses.size() > 2)
            {
                ambiguous[edge.first] = true;
                ambiguous[edge.second] = true;
            }
        }
        using Priority = std::pair<double, std::uint32_t>;
        std::priority_queue<Priority, std::vector<Priority>, std::greater<Priority>> pending;
        std::vector<double> areas(mesh.polygons.size());
        for (std::uint32_t index{}; index < mesh.polygons.size(); ++index)
        {
            const auto &face = mesh.polygons[index];
            areas[index] = Orientation(mesh.vertices[face.vertices[0]], mesh.vertices[face.vertices[1]],
                                       mesh.vertices[face.vertices[2]]) *
                           0.5;
            if (areas[index] > 0)
            {
                pending.emplace(areas[index], index);
            }
        }
        std::vector<bool> removed(mesh.polygons.size());
        while (!pending.empty())
        {
            const auto index = pending.top().second;
            pending.pop();
            const auto &face = mesh.polygons[index];
            if (removed[index] ||
                std::any_of(
                    face.vertices.begin(), face.vertices.end(), [&](auto vertex)
                    { return pinned[vertex] || ambiguous[vertex] || IsCellSeam(mesh.vertices[vertex], bounds); }))
            {
                continue;
            }
            std::size_t exposed{};
            std::size_t exposedSide{};
            bool overlongReplacement{};
            for (std::size_t side{}; side < 3; ++side)
            {
                const auto first = face.vertices[side];
                const auto last = face.vertices[(side + 1) % 3];
                const auto uses = edges.at(std::minmax(first, last)).size();
                if (uses == 1)
                {
                    ++exposed;
                    exposedSide = side;
                }
                else if (uses == 2 && settings.maximumBoundaryEdgeLength > 0)
                {
                    const double dx = static_cast<double>(mesh.vertices[last].x) - mesh.vertices[first].x;
                    const double dy = static_cast<double>(mesh.vertices[last].y) - mesh.vertices[first].y;
                    overlongReplacement = overlongReplacement ||
                                          dx * dx + dy * dy > static_cast<double>(settings.maximumBoundaryEdgeLength) *
                                                                  settings.maximumBoundaryEdgeLength;
                }
            }
            if (overlongReplacement || exposed == 0 || exposed == 3 ||
                (exposed == 1 && boundaryDegree[face.vertices[(exposedSide + 2) % 3]] != 0))
            {
                continue;
            }
            const std::array points{mesh.vertices[face.vertices[0]], mesh.vertices[face.vertices[1]],
                                    mesh.vertices[face.vertices[2]]};
            const auto component = reference.components[index];
            if (removedAreas[component] + areas[index] >
                    reference.areas[component] * settings.maximumBoundaryAreaFraction ||
                !WithinOriginalBoundaryBand(reference, points, index, settings.maximumBoundaryInset))
            {
                continue;
            }
            removed[index] = true;
            removedAreas[component] += areas[index];
            for (std::size_t side{}; side < 3; ++side)
            {
                const auto edge = std::minmax(face.vertices[side], face.vertices[(side + 1) % 3]);
                auto &uses = edges.at(edge);
                const auto wasExposed = uses.size() == 1;
                std::erase(uses, index);
                const auto isExposed = uses.size() == 1;
                const int degreeChange = static_cast<int>(isExposed) - static_cast<int>(wasExposed);
                boundaryDegree[edge.first] += degreeChange;
                boundaryDegree[edge.second] += degreeChange;
                for (const auto neighbor : uses)
                {
                    if (areas[neighbor] > 0)
                    {
                        pending.emplace(areas[neighbor], neighbor);
                    }
                }
            }
        }
        std::vector<std::uint32_t> retainedIndices;
        std::vector<NavPolygon> retained;
        for (std::uint32_t index{}; index < mesh.polygons.size(); ++index)
        {
            if (!removed[index])
            {
                retainedIndices.push_back(index);
                retained.push_back(mesh.polygons[index]);
            }
        }
        mesh.polygons = std::move(retained);
        return retainedIndices;
    }

    struct Fan
    {
        std::vector<std::uint32_t> rim;
        bool boundary{};
    };

    bool SimpleRim(const NavMesh &mesh, const std::vector<std::uint32_t> &rim)
    {
        const auto onSegment = [](Vec3 a, Vec3 b, Vec3 point)
        {
            return Orientation(a, b, point) == 0 && point.x >= std::min(a.x, b.x) && point.x <= std::max(a.x, b.x) &&
                   point.y >= std::min(a.y, b.y) && point.y <= std::max(a.y, b.y);
        };
        for (std::size_t first{}; first < rim.size(); ++first)
        {
            const auto a = mesh.vertices[rim[first]];
            const auto b = mesh.vertices[rim[(first + 1) % rim.size()]];
            for (std::size_t second = first + 2; second < rim.size(); ++second)
            {
                if ((second + 1) % rim.size() == first)
                {
                    continue;
                }
                const auto c = mesh.vertices[rim[second]];
                const auto d = mesh.vertices[rim[(second + 1) % rim.size()]];
                const auto ac = Orientation(a, b, c);
                const auto ad = Orientation(a, b, d);
                const auto ca = Orientation(c, d, a);
                const auto cb = Orientation(c, d, b);
                if ((ac * ad < 0 && ca * cb < 0) || onSegment(a, b, c) || onSegment(a, b, d) || onSegment(c, d, a) ||
                    onSegment(c, d, b))
                {
                    return false;
                }
            }
        }
        return true;
    }

    /** Order opposite edges into one disk fan. Multiple chains, reused directed edges,
     * nonpositive triangles and mixed flags cannot define a replaceable floor patch.
     */
    std::optional<Fan> FindFan(const NavMesh &mesh, std::uint32_t center, const std::vector<std::uint32_t> &faces)
    {
        if (faces.empty())
        {
            return {};
        }
        std::map<std::uint32_t, std::uint32_t> next;
        std::set<std::uint32_t> incoming;
        const auto flags = mesh.polygons[faces.front()].flags;
        for (const auto index : faces)
        {
            const auto &face = mesh.polygons[index];
            const auto at = std::find(face.vertices.begin(), face.vertices.end(), center) - face.vertices.begin();
            if (face.flags != flags ||
                Orientation(mesh.vertices[face.vertices[0]], mesh.vertices[face.vertices[1]],
                            mesh.vertices[face.vertices[2]]) <= 0 ||
                !next.emplace(face.vertices[(at + 1) % 3], face.vertices[(at + 2) % 3]).second ||
                !incoming.insert(face.vertices[(at + 2) % 3]).second)
            {
                return {};
            }
        }
        Fan fan;
        auto first = next.begin()->first;
        for (const auto &[from, to] : next)
        {
            if (!incoming.contains(from))
            {
                if (fan.boundary)
                {
                    return {};
                }
                first = from;
                fan.boundary = true;
            }
        }
        auto current = first;
        do
        {
            fan.rim.push_back(current);
            const auto found = next.find(current);
            if (found == next.end())
            {
                break;
            }
            current = found->second;
        } while (current != first && fan.rim.size() <= faces.size());
        if (fan.rim.size() != faces.size() + static_cast<std::size_t>(fan.boundary) ||
            (!fan.boundary && current != first) || (fan.boundary && next.contains(fan.rim.back())))
        {
            return {};
        }
        if (!SimpleRim(mesh, fan.rim))
        {
            return {};
        }
        return fan;
    }

    /// Exact planar cleanup needs no approximation budget, including beside tiny edge slivers.
    bool CoplanarFan(const NavMesh &mesh, const Fan &fan, const std::vector<std::uint32_t> &faces)
    {
        const auto plane = *std::max_element(faces.begin(), faces.end(),
                                             [&](auto left, auto right)
                                             {
                                                 const auto area = [&](auto index)
                                                 {
                                                     const auto &face = mesh.polygons[index];
                                                     return Orientation(mesh.vertices[face.vertices[0]],
                                                                        mesh.vertices[face.vertices[1]],
                                                                        mesh.vertices[face.vertices[2]]);
                                                 };
                                                 return area(left) < area(right);
                                             });
        const auto &vertices = mesh.polygons[plane].vertices;
        for (const auto vertex : fan.rim)
        {
            if (!Coplanar(mesh.vertices[vertices[0]], mesh.vertices[vertices[1]], mesh.vertices[vertices[2]],
                          mesh.vertices[vertex]))
            {
                return false;
            }
        }
        return true;
    }

    /** Trim only an outward ear, never a concave obstacle corner or passage neck.
     * The entire rim must remain inside the new edge's floor half-plane. Original samples
     * from both consumed boundary segments constrain total deviation across repeated edits.
     */
    std::optional<std::vector<std::uint32_t>> BoundaryReplacementSamples(
        const NavMesh &mesh, std::uint32_t center, const Fan &fan, const std::vector<std::uint32_t> &faces,
        const SurfaceSimplificationSettings &settings,
        const std::map<Edge, std::vector<std::uint32_t>> &boundarySamples)
    {
        const auto first = fan.rim.front();
        const auto last = fan.rim.back();
        const auto a = mesh.vertices[last];
        const auto b = mesh.vertices[first];
        const auto point = mesh.vertices[center];
        const double dx = static_cast<double>(b.x) - a.x;
        const double dy = static_cast<double>(b.y) - a.y;
        const double squaredLength = dx * dx + dy * dy;
        if (squaredLength == 0 || (settings.maximumBoundaryEdgeLength > 0 &&
                                   squaredLength > static_cast<double>(settings.maximumBoundaryEdgeLength) *
                                                       settings.maximumBoundaryEdgeLength))
        {
            return {};
        }
        if (!StraightBoundary(a, point, b, settings.maximumBoundaryEdgeLength))
        {
            const auto earArea = -Orientation(a, b, point);
            if (settings.maximumBoundaryInset == 0 || earArea < 0 ||
                std::any_of(
                    fan.rim.begin(), fan.rim.end(),
                    [&](auto vertex) { return Orientation(a, b, mesh.vertices[vertex]) < 0; }))
            {
                return {};
            }
        }
        std::set<std::uint32_t> samples{center, first, last};
        for (const auto endpoint : {first, last})
        {
            const auto found = boundarySamples.find(std::minmax(center, endpoint));
            if (found != boundarySamples.end())
            {
                samples.insert(found->second.begin(), found->second.end());
            }
        }
        const auto &plane = mesh.polygons[faces.front()].vertices;
        for (const auto sample : samples)
        {
            const auto originalPoint = mesh.vertices[sample];
            if (SegmentDistanceSquared({originalPoint.x, originalPoint.y}, a, b) >
                    static_cast<double>(settings.maximumBoundaryInset) * settings.maximumBoundaryInset ||
                (settings.maximumHeightError == 0 &&
                 !Coplanar(mesh.vertices[plane[0]], mesh.vertices[plane[1]], mesh.vertices[plane[2]], originalPoint)))
            {
                return {};
            }
        }
        return std::vector<std::uint32_t>(samples.begin(), samples.end());
    }

    /** Fit a local plane to immutable input vertices. Checking every original vertex against
     * half the error budget bounds both old and replacement linear surfaces against that
     * plane over the whole footprint, hence their difference against the full budget.
     * Conservative contributor unions retain samples removed in every preceding pass.
     */
    bool WithinOriginalHeightError(const NavMesh &mesh, const std::vector<NavPolygon> &original,
                                   const std::vector<std::uint32_t> &sources, float maximumError)
    {
        if (maximumError == 0)
        {
            return false;
        }
        std::set<std::uint32_t> vertices;
        for (const auto source : sources)
        {
            vertices.insert(original[source].vertices.begin(), original[source].vertices.end());
        }
        double meanX{}, meanY{}, meanZ{};
        // Subtract a local origin before summing, avoiding cancellation at distant CELL coordinates.
        const auto origin = mesh.vertices[*vertices.begin()];
        for (const auto vertex : vertices)
        {
            const auto point = mesh.vertices[vertex];
            meanX += static_cast<double>(point.x) - origin.x;
            meanY += static_cast<double>(point.y) - origin.y;
            meanZ += static_cast<double>(point.z) - origin.z;
        }
        meanX /= vertices.size();
        meanY /= vertices.size();
        meanZ /= vertices.size();
        double xx{}, xy{}, yy{}, xz{}, yz{};
        for (const auto vertex : vertices)
        {
            const auto point = mesh.vertices[vertex];
            const double x = static_cast<double>(point.x) - origin.x - meanX;
            const double y = static_cast<double>(point.y) - origin.y - meanY;
            const double z = static_cast<double>(point.z) - origin.z - meanZ;
            xx += x * x;
            xy += x * y;
            yy += y * y;
            xz += x * z;
            yz += y * z;
        }
        const auto determinant = xx * yy - xy * xy;
        if (determinant <= 0 || RoundoffZero(determinant, xx * yy + xy * xy))
        {
            return false;
        }
        const auto dzdx = (xz * yy - yz * xy) / determinant;
        const auto dzdy = (yz * xx - xz * xy) / determinant;
        for (const auto vertex : vertices)
        {
            const auto point = mesh.vertices[vertex];
            const double expected = meanZ + dzdx * (static_cast<double>(point.x) - origin.x - meanX) +
                                    dzdy * (static_cast<double>(point.y) - origin.y - meanY);
            if (std::abs(static_cast<double>(point.z) - origin.z - expected) > maximumError * 0.5)
            {
                return false;
            }
        }
        return true;
    }

    double SlopeDegrees(const NavMesh &mesh, const NavPolygon &face)
    {
        const auto a = mesh.vertices[face.vertices[0]];
        const auto b = mesh.vertices[face.vertices[1]];
        const auto c = mesh.vertices[face.vertices[2]];
        const double bx = static_cast<double>(b.x) - a.x;
        const double by = static_cast<double>(b.y) - a.y;
        const double bz = static_cast<double>(b.z) - a.z;
        const double cx = static_cast<double>(c.x) - a.x;
        const double cy = static_cast<double>(c.y) - a.y;
        const double cz = static_cast<double>(c.z) - a.z;
        return std::atan2(std::hypot(by * cz - bz * cy, bz * cx - bx * cz), Orientation(a, b, c)) * 180.0 /
               std::acos(-1.0);
    }

    bool WithinSlopeEnvelope(const NavMesh &mesh, const std::vector<NavPolygon> &original,
                             const std::vector<std::uint32_t> &sources, const std::vector<NavPolygon> &replacement,
                             float maximumSlope)
    {
        double envelope = maximumSlope;
        for (const auto source : sources)
        {
            envelope = std::max(envelope, SlopeDegrees(mesh, original[source]));
        }
        return std::all_of(replacement.begin(), replacement.end(),
                           [&](const auto &face)
                           {
                               const auto slope = SlopeDegrees(mesh, face);
                               return slope <= envelope || RoundoffZero(slope - envelope, slope + envelope);
                           });
    }

    bool WithinFanHeightError(const NavMesh &mesh, const Fan &fan, const std::vector<std::uint32_t> &faces,
                              const std::vector<NavPolygon> &original, const std::vector<std::uint32_t> &sources,
                              float maximumError)
    {
        if (WithinOriginalHeightError(mesh, original, sources, maximumError))
        {
            return true;
        }
        // Exact cleanup also checks immutable contributors, retaining evidence removed by approximate edits.
        if (!CoplanarFan(mesh, fan, faces))
        {
            return false;
        }
        const auto &plane = mesh.polygons[faces.front()].vertices;
        for (const auto source : sources)
        {
            for (const auto vertex : original[source].vertices)
            {
                if (!Coplanar(mesh.vertices[plane[0]], mesh.vertices[plane[1]], mesh.vertices[plane[2]],
                              mesh.vertices[vertex]))
                {
                    return false;
                }
            }
        }
        return true;
    }

    /// New diagonals cannot consume an existing edge outside the cavity or reuse a nonmanifold rim.
    bool ReplacementEdgesAreManifold(const std::map<Edge, std::vector<std::uint32_t>> &edgeUses,
                                     const std::vector<std::uint32_t> &faces,
                                     const std::vector<NavPolygon> &replacement)
    {
        for (const auto &face : replacement)
        {
            for (std::size_t side{}; side < 3; ++side)
            {
                const auto edge = std::minmax(face.vertices[side], face.vertices[(side + 1) % 3]);
                const auto found = edgeUses.find(edge);
                if (found == edgeUses.end())
                {
                    continue;
                }
                // Rim edges have one cavity use; an existing diagonal has none.
                if (found->second.size() > 2 ||
                    std::none_of(found->second.begin(), found->second.end(), [&](auto index)
                                 { return std::find(faces.begin(), faces.end(), index) != faces.end(); }))
                {
                    return false;
                }
            }
        }
        return true;
    }

    /// Reuse the constrained triangulator with only rim samples, preserving every untouched rim edge.
    std::vector<NavPolygon> ReplaceFan(const NavMesh &mesh, const Fan &fan, std::uint16_t flags)
    {
        if (fan.rim.size() == 2)
        {
            return {};
        }
        std::vector<Vec3> points;
        std::vector<std::uint32_t> boundary;
        for (const auto vertex : fan.rim)
        {
            boundary.push_back(static_cast<std::uint32_t>(points.size()));
            points.push_back(mesh.vertices[vertex]);
        }
        std::vector<NavPolygon> replacement;
        for (const auto &indices : TriangulateDetailSamples(points, boundary))
        {
            NavPolygon face;
            face.vertices = {fan.rim[indices[0]], fan.rim[indices[1]], fan.rim[indices[2]]};
            face.neighbors.fill(NoNeighbor);
            face.flags = flags;
            replacement.push_back(face);
        }
        return replacement;
    }

    /// Publish a compact unlinked mesh; evidence and portal identities are assigned by the caller afterwards.
    void CompactVertices(NavMesh &mesh)
    {
        std::vector<std::uint32_t> remap(mesh.vertices.size(), NoNeighbor);
        std::vector<Vec3> vertices;
        for (auto &face : mesh.polygons)
        {
            face.neighbors.fill(NoNeighbor);
            for (auto &index : face.vertices)
            {
                if (remap[index] == NoNeighbor)
                {
                    remap[index] = static_cast<std::uint32_t>(vertices.size());
                    vertices.push_back(mesh.vertices[index]);
                }
                index = remap[index];
            }
        }
        mesh.vertices = std::move(vertices);
    }
} // namespace

namespace navmesh::core
{
    std::size_t SimplifyGeneratedSurface(NavMesh &mesh, std::optional<AABB> cellBounds,
                                         const SurfaceSimplificationSettings &settings,
                                         std::span<const std::uint32_t> pinnedVertices,
                                         std::vector<std::vector<std::uint32_t>> *originalPolygons)
    {
        if (!mesh.externalLinks.empty() || !mesh.doorLinks.empty() ||
            !std::isfinite(settings.maximumBoundaryEdgeLength) || settings.maximumBoundaryEdgeLength < 0 ||
            !std::isfinite(settings.maximumHeightError) || settings.maximumHeightError < 0 ||
            !std::isfinite(settings.maximumSlopeDegrees) || settings.maximumSlopeDegrees < 0 ||
            settings.maximumSlopeDegrees > 90 || !std::isfinite(settings.maximumBoundaryInset) ||
            settings.maximumBoundaryInset < 0 || !std::isfinite(settings.maximumBoundaryAreaFraction) ||
            settings.maximumBoundaryAreaFraction < 0 || settings.maximumBoundaryAreaFraction > 1)
        {
            throw std::invalid_argument(
                "Surface simplification requires an unlinked mesh and valid approximation limits");
        }
        for (const auto point : mesh.vertices)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                throw std::invalid_argument("Surface simplification requires finite world coordinates");
            }
        }
        for (const auto &face : mesh.polygons)
        {
            for (const auto vertex : face.vertices)
            {
                if (vertex >= mesh.vertices.size())
                {
                    throw std::invalid_argument("Surface simplification has an invalid vertex index");
                }
            }
        }
        const auto originalCount = mesh.polygons.size();
        const auto original = mesh.polygons;
        std::vector<bool> pinned(mesh.vertices.size());
        for (const auto vertex : pinnedVertices)
        {
            if (vertex >= pinned.size())
            {
                throw std::invalid_argument("Surface simplification has an invalid pinned vertex index");
            }
            pinned[vertex] = true;
        }
        std::map<Edge, std::vector<std::uint32_t>> boundarySamples;
        std::vector<std::vector<std::uint32_t>> contributors;
        for (std::uint32_t index{}; index < originalCount; ++index)
        {
            contributors.push_back({index});
        }
        const auto boundaryReference = settings.maximumBoundaryInset > 0 ? OriginalBoundary(mesh) : BoundaryReference{};
        std::vector<double> removedBoundaryAreas(originalCount);
        if (settings.maximumBoundaryInset > 0)
        {
            const auto retained =
                PruneBoundaryTriangles(mesh, settings, cellBounds, pinned, boundaryReference, removedBoundaryAreas);
            std::vector<std::vector<std::uint32_t>> retainedContributors;
            for (const auto index : retained)
            {
                retainedContributors.push_back(std::move(contributors[index]));
            }
            contributors = std::move(retainedContributors);
        }
        constexpr std::size_t maximumPasses = 12;
        for (std::size_t pass{}; pass < maximumPasses; ++pass)
        {
            std::vector<std::vector<std::uint32_t>> incident(mesh.vertices.size());
            std::map<Edge, std::vector<std::uint32_t>> edgeUses;
            for (std::uint32_t index{}; index < mesh.polygons.size(); ++index)
            {
                const auto &face = mesh.polygons[index];
                for (std::size_t side{}; side < 3; ++side)
                {
                    incident[face.vertices[side]].push_back(index);
                    edgeUses[std::minmax(face.vertices[side], face.vertices[(side + 1) % 3])].push_back(index);
                }
            }
            // Disjoint edits read unchanged topology within a pass. Blocking the complete rim
            // keeps simultaneous cavities from creating intersecting diagonals or shared new edges.
            std::vector<bool> blocked(mesh.vertices.size()), removed(mesh.polygons.size());
            std::vector<NavPolygon> replacements;
            std::vector<std::vector<std::uint32_t>> replacementContributors;
            bool changed{};
            for (std::uint32_t center{}; center < incident.size(); ++center)
            {
                if (blocked[center] || pinned[center] || IsCellSeam(mesh.vertices[center], cellBounds))
                {
                    continue;
                }
                const auto &faces = incident[center];
                const auto fan = FindFan(mesh, center, faces);
                if (!fan || std::any_of(fan->rim.begin(), fan->rim.end(),
                                        [&](auto vertex) { return blocked[vertex] || pinned[vertex]; }))
                {
                    continue;
                }
                std::optional<std::vector<std::uint32_t>> boundaryReplacement;
                double removedEarArea{};
                std::uint32_t earComponent{};
                if (fan->boundary)
                {
                    boundaryReplacement =
                        BoundaryReplacementSamples(mesh, center, *fan, faces, settings, boundarySamples);
                    if (!boundaryReplacement)
                    {
                        continue;
                    }
                    removedEarArea =
                        std::max(0.0, -Orientation(mesh.vertices[fan->rim.back()], mesh.vertices[fan->rim.front()],
                                                   mesh.vertices[center]) *
                                          0.5);
                    if (removedEarArea > 0)
                    {
                        const auto originalPolygon = contributors[faces.front()].front();
                        earComponent = boundaryReference.components[originalPolygon];
                        if (removedBoundaryAreas[earComponent] + removedEarArea >
                                boundaryReference.areas[earComponent] * settings.maximumBoundaryAreaFraction ||
                            !WithinOriginalBoundaryBand(boundaryReference,
                                                        {mesh.vertices[fan->rim.back()],
                                                         mesh.vertices[fan->rim.front()], mesh.vertices[center]},
                                                        originalPolygon, settings.maximumBoundaryInset))
                        {
                            continue;
                        }
                    }
                }
                bool manifold = true;
                if (fan->rim.size() == 2)
                {
                    // A tiny leaf may be removed only beside another floor triangle, never as an entire island.
                    manifold = edgeUses.at(std::minmax(fan->rim.front(), fan->rim.back())).size() == 2;
                }
                for (const auto vertex : fan->rim)
                {
                    manifold = manifold && edgeUses.at(std::minmax(center, vertex)).size() <= 2;
                }
                if (!manifold)
                {
                    continue;
                }
                std::set<std::uint32_t> sourceSet;
                for (const auto face : faces)
                {
                    sourceSet.insert(contributors[face].begin(), contributors[face].end());
                }
                // Keep approximation neighborhoods local and contributor storage bounded.
                constexpr std::size_t maximumOriginalTriangles = 256;
                if (sourceSet.size() > maximumOriginalTriangles)
                {
                    continue;
                }
                const std::vector<std::uint32_t> sources(sourceSet.begin(), sourceSet.end());
                if (fan->rim.size() > 2 &&
                    !WithinFanHeightError(mesh, *fan, faces, original, sources, settings.maximumHeightError))
                {
                    continue;
                }
                std::vector<NavPolygon> replacement;
                try
                {
                    replacement = ReplaceFan(mesh, *fan, mesh.polygons[faces.front()].flags);
                }
                catch (const std::runtime_error &)
                {
                    continue;
                }
                if (!ReplacementEdgesAreManifold(edgeUses, faces, replacement) || replacement.size() >= faces.size() ||
                    !WithinSlopeEnvelope(mesh, original, sources, replacement, settings.maximumSlopeDegrees))
                {
                    continue;
                }
                for (const auto index : faces)
                {
                    removed[index] = true;
                }
                changed = true;
                if (removedEarArea > 0)
                {
                    removedBoundaryAreas[earComponent] += removedEarArea;
                }
                blocked[center] = true;
                for (const auto vertex : fan->rim)
                {
                    blocked[vertex] = true;
                }
                replacements.insert(replacements.end(), replacement.begin(), replacement.end());
                replacementContributors.insert(replacementContributors.end(), replacement.size(), sources);
                if (boundaryReplacement)
                {
                    boundarySamples[std::minmax(fan->rim.front(), fan->rim.back())] = std::move(*boundaryReplacement);
                    boundarySamples.erase(std::minmax(center, fan->rim.front()));
                    boundarySamples.erase(std::minmax(center, fan->rim.back()));
                }
            }
            if (!changed)
            {
                break;
            }
            std::vector<NavPolygon> retained;
            std::vector<std::vector<std::uint32_t>> retainedContributors;
            for (std::size_t index{}; index < mesh.polygons.size(); ++index)
            {
                if (!removed[index])
                {
                    retained.push_back(mesh.polygons[index]);
                    retainedContributors.push_back(std::move(contributors[index]));
                }
            }
            retained.insert(retained.end(), replacements.begin(), replacements.end());
            mesh.polygons = std::move(retained);
            retainedContributors.insert(retainedContributors.end(),
                                        std::make_move_iterator(replacementContributors.begin()),
                                        std::make_move_iterator(replacementContributors.end()));
            contributors = std::move(retainedContributors);
        }
        CompactVertices(mesh);
        if (originalPolygons)
        {
            *originalPolygons = std::move(contributors);
        }
        return originalCount - mesh.polygons.size();
    }

    std::size_t SimplifyCandidateSurface(CandidateNavMesh &candidate, std::optional<AABB> cellBounds)
    {
        if (!candidate.topology.valid || candidate.mesh.polygons.empty() ||
            std::any_of(candidate.borderLinks.begin(), candidate.borderLinks.end(),
                        [](const auto &link) { return link.generatedNeighborCell.has_value(); }))
        {
            return 0;
        }
        std::set<std::uint32_t> pins;
        const auto pinFace = [&](std::uint32_t index)
        {
            const auto &face = candidate.mesh.polygons.at(index);
            pins.insert(face.vertices.begin(), face.vertices.end());
        };
        for (const auto &link : candidate.borderLinks)
        {
            pinFace(link.polygon);
        }
        for (const auto &exit : candidate.exits)
        {
            if (exit.polygon)
            {
                pinFace(*exit.polygon);
            }
        }
        // Vertex-only contacts between separate floor regions cannot become a merged evidence patch.
        std::vector<std::uint32_t> polygonRegions(candidate.mesh.polygons.size(), NoNeighbor);
        std::vector<std::uint32_t> vertexRegions(candidate.mesh.vertices.size(), NoNeighbor);
        for (const auto &region : candidate.regions)
        {
            for (const auto index : region.polygons)
            {
                polygonRegions.at(index) = region.id;
                for (const auto vertex : candidate.mesh.polygons.at(index).vertices)
                {
                    if (vertexRegions[vertex] != NoNeighbor && vertexRegions[vertex] != region.id)
                    {
                        pins.insert(vertex);
                    }
                    vertexRegions[vertex] = region.id;
                }
            }
        }
        const SurfaceSimplificationSettings settings{.maximumBoundaryEdgeLength =
                                                         candidate.recastSettings.maxEdgeLength,
                                                     .maximumHeightError = candidate.recastSettings.cellHeight,
                                                     .maximumSlopeDegrees = candidate.profile.maxSlopeDegrees,
                                                     .maximumBoundaryInset = candidate.profile.agentRadius * 2,
                                                     .maximumBoundaryAreaFraction = 0.02F};
        auto simplified = candidate;
        std::vector<std::vector<std::uint32_t>> originalPolygons;
        const std::vector<std::uint32_t> pinnedVertices(pins.begin(), pins.end());
        const auto removed =
            SimplifyGeneratedSurface(simplified.mesh, cellBounds, settings, pinnedVertices, &originalPolygons);
        if (removed == 0)
        {
            return 0;
        }
        std::vector<std::uint32_t> remap(candidate.mesh.polygons.size(), NoNeighbor);
        simplified.polygonSourceTriangles.clear();
        simplified.polygonContributingTriangles.clear();
        for (auto &region : simplified.regions)
        {
            region.polygons.clear();
            region.sourceTriangles.clear();
        }
        for (std::uint32_t index{}; index < originalPolygons.size(); ++index)
        {
            const auto &originals = originalPolygons[index];
            const auto first = originals.front();
            if (originals.size() == 1)
            {
                remap[first] = index;
            }
            std::set<std::size_t> sources;
            for (const auto original : originals)
            {
                sources.insert(candidate.polygonContributingTriangles.at(original).begin(),
                               candidate.polygonContributingTriangles.at(original).end());
            }
            simplified.polygonSourceTriangles.push_back(candidate.polygonSourceTriangles.at(first));
            simplified.polygonContributingTriangles.emplace_back(sources.begin(), sources.end());
            auto &region = simplified.regions.at(polygonRegions.at(first));
            region.polygons.push_back(index);
            region.sourceTriangles.insert(region.sourceTriangles.end(), sources.begin(), sources.end());
        }
        for (auto &region : simplified.regions)
        {
            std::sort(region.sourceTriangles.begin(), region.sourceTriangles.end());
            region.sourceTriangles.erase(std::unique(region.sourceTriangles.begin(), region.sourceTriangles.end()),
                                         region.sourceTriangles.end());
        }
        for (auto &link : simplified.borderLinks)
        {
            link.polygon = remap.at(link.polygon);
        }
        for (auto &exit : simplified.exits)
        {
            if (exit.polygon)
            {
                exit.polygon = remap.at(*exit.polygon);
            }
        }
        RefreshCandidateTopology(simplified);
        if (!simplified.topology.valid)
        {
            return 0;
        }
        simplified.warnings.push_back(std::format(
            "Removed {} detail and boundary triangles after authored border repair with vertical error bounded by {} "
            "Skyrim "
            "units "
            "and inward boundary deviation bounded by {} Skyrim units; portal and door triangles remain pinned.",
            removed, settings.maximumHeightError, settings.maximumBoundaryInset));
        candidate = std::move(simplified);
        return removed;
    }
} // namespace navmesh::core
