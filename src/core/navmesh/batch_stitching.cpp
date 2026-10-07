#include "core/navmesh/batch_stitching.h"

#include "core/navmesh/detail_triangulation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

namespace
{
    using namespace navmesh::core;
    constexpr auto Open = std::numeric_limits<std::uint32_t>::max();
    using EdgeIndex = std::pair<std::uint32_t, std::uint8_t>;
    using Coordinates = std::array<float, 3>;
    using DirectedEdge = std::pair<Coordinates, Coordinates>;

    Coordinates Coordinate(Vec3 point)
    {
        return {point.x, point.y, point.z};
    }

    double SignedArea(Vec3 a, Vec3 b, Vec3 c)
    {
        return (static_cast<double>(b.x) - a.x) * (static_cast<double>(c.y) - a.y) -
               (static_cast<double>(b.y) - a.y) * (static_cast<double>(c.x) - a.x);
    }

    /// Directed seam segment in the original candidate; the parameter is world X or Y in Skyrim units.
    struct SeamEdge
    {
        EdgeIndex index;
        Vec3 first, last;
        float start{}, end{};
    };

    float Parameter(Vec3 point, bool vertical)
    {
        return vertical ? point.y : point.x;
    }

    Vec3 PointAt(const SeamEdge &edge, float parameter)
    {
        const auto fraction = (parameter - edge.start) / (edge.end - edge.start);
        return edge.first + (edge.last - edge.first) * fraction;
    }

    std::vector<SeamEdge> Boundary(const CandidateNavMesh &candidate, bool vertical, float boundary)
    {
        std::set<EdgeIndex> consumed;
        for (const auto &portal : candidate.borderLinks)
        {
            consumed.emplace(portal.polygon, portal.edge);
        }
        std::vector<SeamEdge> edges;
        for (std::uint32_t polygon{}; polygon < candidate.mesh.polygons.size(); ++polygon)
        {
            const auto &face = candidate.mesh.polygons[polygon];
            for (std::uint8_t side{}; side < 3; ++side)
            {
                if (face.neighbors[side] != Open || consumed.contains({polygon, side}))
                {
                    continue;
                }
                const auto a = candidate.mesh.vertices.at(face.vertices[side]);
                const auto b = candidate.mesh.vertices.at(face.vertices[(side + 1) % 3]);
                const auto first = vertical ? a.x : a.y;
                const auto last = vertical ? b.x : b.y;
                if (first == boundary && last == boundary && Parameter(a, vertical) != Parameter(b, vertical))
                {
                    edges.push_back({{polygon, side}, a, b, Parameter(a, vertical), Parameter(b, vertical)});
                }
            }
        }
        return edges;
    }

    /// One common endpoint proposal, with the intersection of both floors' permitted height ranges.
    struct Endpoint
    {
        Vec3 point;
        std::uint32_t worldspace{};
        float minimumHeight{}, maximumHeight{}, climb{};
    };

    struct PortalPlan
    {
        std::size_t left{}, right{};
        EdgeIndex leftEdge, rightEdge;
        std::size_t first{}, last{};
        bool leftIncreasing{};
    };

    struct StitchPlan
    {
        std::vector<Endpoint> endpoints;
        std::vector<PortalPlan> portals;
    };

    /// Intersect both seam partitions before choosing nearest compatible stacked-floor pairs.
    void PlanSeam(StitchPlan &plan, std::span<GeneratedCellCandidate> cells, std::size_t left, std::size_t right,
                  bool vertical, float boundary)
    {
        const auto &a = *cells[left].candidate;
        const auto &b = *cells[right].candidate;
        const auto leftEdges = Boundary(a, vertical, boundary);
        const auto rightEdges = Boundary(b, vertical, boundary);
        std::vector<float> partitions;
        for (const auto *edges : {&leftEdges, &rightEdges})
        {
            for (const auto &edge : *edges)
            {
                partitions.push_back(edge.start);
                partitions.push_back(edge.end);
            }
        }
        std::sort(partitions.begin(), partitions.end());
        partitions.erase(std::unique(partitions.begin(), partitions.end()), partitions.end());
        const auto climb = std::min(a.profile.stepHeight, b.profile.stepHeight);
        const auto tolerance = std::max(a.profile.weldTolerance, b.profile.weldTolerance);
        // Endpoint containment remains exact even when an interval has no representable float midpoint.
        for (std::size_t partition = 1; partition < partitions.size(); ++partition)
        {
            const auto first = partitions[partition - 1];
            const auto last = partitions[partition];
            if (last <= first)
            {
                continue;
            }
            std::vector<std::tuple<float, std::size_t, std::size_t>> matches;
            for (std::size_t l{}; l < leftEdges.size(); ++l)
            {
                const auto &le = leftEdges[l];
                if (first < std::min(le.start, le.end) || last > std::max(le.start, le.end))
                {
                    continue;
                }
                for (std::size_t r{}; r < rightEdges.size(); ++r)
                {
                    const auto &re = rightEdges[r];
                    if ((le.end - le.start) * (re.end - re.start) >= 0 || first < std::min(re.start, re.end) ||
                        last > std::max(re.start, re.end))
                    {
                        continue;
                    }
                    const auto startGap = std::abs(PointAt(le, first).z - PointAt(re, first).z);
                    const auto endGap = std::abs(PointAt(le, last).z - PointAt(re, last).z);
                    if (startGap <= climb + tolerance && endGap <= climb + tolerance)
                    {
                        matches.emplace_back(startGap + endGap, l, r);
                    }
                }
            }
            std::sort(matches.begin(), matches.end());
            std::set<std::size_t> usedLeft, usedRight;
            for (const auto &[score, l, r] : matches)
            {
                if (usedLeft.contains(l) || usedRight.contains(r))
                {
                    continue;
                }
                usedLeft.insert(l);
                usedRight.insert(r);
                const auto endpoint = [&](float parameter)
                {
                    auto point = PointAt(leftEdges[l], parameter);
                    const auto other = PointAt(rightEdges[r], parameter);
                    if (vertical)
                    {
                        point.x = boundary;
                        point.y = parameter;
                    }
                    else
                    {
                        point.x = parameter;
                        point.y = boundary;
                    }
                    point.z = std::min(point.z, other.z);
                    const auto index = plan.endpoints.size();
                    plan.endpoints.push_back({point, cells[left].worldspaceId,
                                              std::max(PointAt(leftEdges[l], parameter).z, other.z) - climb - tolerance,
                                              std::min(PointAt(leftEdges[l], parameter).z, other.z) + climb + tolerance,
                                              climb + tolerance});
                    return index;
                };
                const auto start = endpoint(first);
                const auto end = endpoint(last);
                plan.portals.push_back({left, right, leftEdges[l].index, rightEdges[r].index, start, end,
                                        leftEdges[l].end > leftEdges[l].start});
            }
        }
    }

    /// Resolve all corner uses together so a later CELL side cannot move an already planned portal endpoint.
    void WeldEndpoints(StitchPlan &plan, std::span<GeneratedCellCandidate> cells)
    {
        using Position = std::tuple<std::uint32_t, float, float>;
        std::map<Position, std::vector<std::size_t>> groups;
        std::map<Position, std::vector<float>> pins;
        for (std::size_t index{}; index < plan.endpoints.size(); ++index)
        {
            const auto &point = plan.endpoints[index];
            groups[{point.worldspace, point.point.x, point.point.y}].push_back(index);
        }
        for (const auto &cell : cells)
        {
            for (const auto &link : cell.candidate->borderLinks)
            {
                const auto &face = cell.candidate->mesh.polygons.at(link.polygon);
                for (const auto index : {face.vertices[link.edge], face.vertices[(link.edge + 1) % 3]})
                {
                    const auto point = cell.candidate->mesh.vertices.at(index);
                    pins[{cell.worldspaceId, point.x, point.y}].push_back(point.z);
                }
            }
        }
        for (auto &[position, indices] : groups)
        {
            std::sort(indices.begin(), indices.end(),
                      [&](auto a, auto b) { return plan.endpoints[a].point.z < plan.endpoints[b].point.z; });
            for (std::size_t begin{}; begin < indices.size();)
            {
                auto end = begin + 1;
                auto low = plan.endpoints[indices[begin]].minimumHeight;
                auto high = plan.endpoints[indices[begin]].maximumHeight;
                auto climb = plan.endpoints[indices[begin]].climb;
                const auto initialHeight = plan.endpoints[indices[begin]].point.z;
                while (end < indices.size())
                {
                    const auto &next = plan.endpoints[indices[end]];
                    if (next.point.z - initialHeight > std::min(climb, next.climb) ||
                        std::max(low, next.minimumHeight) > std::min(high, next.maximumHeight))
                    {
                        break;
                    }
                    low = std::max(low, next.minimumHeight);
                    high = std::min(high, next.maximumHeight);
                    climb = std::min(climb, next.climb);
                    ++end;
                }
                auto height = std::clamp(initialHeight, low, high);
                std::optional<float> pinned;
                for (const auto pin : pins[position])
                {
                    if (std::abs(pin - initialHeight) > climb || pin < low || pin > high)
                    {
                        continue;
                    }
                    if (!pinned || std::abs(pin - initialHeight) < std::abs(*pinned - initialHeight))
                    {
                        pinned = pin;
                    }
                }
                if (pinned)
                {
                    height = *pinned;
                }
                for (auto index = begin; index < end; ++index)
                {
                    plan.endpoints[indices[index]].point.z = height;
                }
                begin = end;
            }
        }
    }

    using RefinedEdges = std::map<std::pair<std::uint32_t, DirectedEdge>, EdgeIndex>;

    /// Keep original face ownership in edge lookup: coincident independent floors
    /// must not redirect a planned portal or an existing authored portal to another fan.
    RefinedEdges EdgeLookup(const CandidateNavMesh &candidate, const std::vector<std::vector<std::uint32_t>> &children)
    {
        RefinedEdges result;
        for (std::uint32_t original{}; original < children.size(); ++original)
        {
            for (const auto polygon : children[original])
            {
                const auto &face = candidate.mesh.polygons[polygon];
                for (std::uint8_t edge{}; edge < 3; ++edge)
                {
                    const DirectedEdge key{Coordinate(candidate.mesh.vertices.at(face.vertices[edge])),
                                           Coordinate(candidate.mesh.vertices.at(face.vertices[(edge + 1) % 3]))};
                    if (!result.emplace(std::pair{original, key}, EdgeIndex{polygon, edge}).second)
                    {
                        throw std::runtime_error("Generated seam fan has duplicate directed edges");
                    }
                }
            }
        }
        return result;
    }

    /** Split only seam-facing triangles. Their unchanged interior edges and identities
     * remain shared with untouched triangles. Complete audit joins and region membership
     * are copied to every child; door anchors follow the child containing their XY point.
     */
    RefinedEdges ApplyPartitions(CandidateNavMesh &candidate, const std::map<EdgeIndex, std::vector<Vec3>> &cuts)
    {
        const auto original = candidate.mesh;
        std::vector<std::vector<std::uint32_t>> descendants(original.polygons.size());
        // Authored return edges retain their exact vertices. A generated seam
        // can use a separate height at the same XY corner and connect through
        // climb-compatible internal edges, without moving an authored endpoint.
        std::set<std::uint32_t> pinnedVertices;
        for (const auto &link : candidate.borderLinks)
        {
            const auto &face = original.polygons.at(link.polygon);
            pinnedVertices.insert(face.vertices[link.edge]);
            pinnedVertices.insert(face.vertices[(link.edge + 1) % 3]);
        }
        std::map<std::uint32_t, Vec3> moved;
        for (const auto &[index, points] : cuts)
        {
            const auto &face = original.polygons.at(index.first);
            for (const auto vertex : {face.vertices[index.second], face.vertices[(index.second + 1) % 3]})
            {
                const auto start = original.vertices.at(vertex);
                for (const auto point : points)
                {
                    if (point.x != start.x || point.y != start.y)
                    {
                        continue;
                    }
                    const auto [it, inserted] = moved.emplace(vertex, point);
                    if (!inserted && Coordinate(it->second) != Coordinate(point))
                    {
                        // Independent compatible seams can require different corner
                        // levels. Keep the incident floor vertex and give each seam
                        // its own endpoint, joined through climb-compatible fan edges.
                        pinnedVertices.insert(vertex);
                    }
                }
            }
        }
        for (const auto &[vertex, point] : moved)
        {
            if (!pinnedVertices.contains(vertex))
            {
                candidate.mesh.vertices.at(vertex) = point;
            }
        }
        std::map<Coordinates, std::uint32_t> vertices;
        for (std::uint32_t index{}; index < candidate.mesh.vertices.size(); ++index)
        {
            vertices.try_emplace(Coordinate(candidate.mesh.vertices[index]), index);
        }
        const auto vertexIndex = [&](Vec3 point)
        {
            const auto [it, inserted] =
                vertices.try_emplace(Coordinate(point), static_cast<std::uint32_t>(candidate.mesh.vertices.size()));
            if (inserted)
            {
                candidate.mesh.vertices.push_back(point);
            }
            return it->second;
        };
        std::vector<std::uint32_t> regions(original.polygons.size(), Open);
        for (const auto &region : candidate.regions)
        {
            for (const auto polygon : region.polygons)
            {
                regions.at(polygon) = region.id;
            }
        }
        for (std::uint32_t polygon{}; polygon < original.polygons.size(); ++polygon)
        {
            const auto &face = original.polygons[polygon];
            descendants[polygon] = {polygon};
            std::vector<std::uint32_t> outline;
            for (std::uint8_t edge{}; edge < 3; ++edge)
            {
                auto start = candidate.mesh.vertices[face.vertices[edge]];
                auto finish = candidate.mesh.vertices[face.vertices[(edge + 1) % 3]];
                const auto found = cuts.find({polygon, edge});
                std::vector<Vec3> points;
                if (found != cuts.end())
                {
                    points = found->second;
                    for (const auto point : points)
                    {
                        if (point.x == start.x && point.y == start.y)
                        {
                            start = point;
                        }
                        if (point.x == finish.x && point.y == finish.y)
                        {
                            finish = point;
                        }
                    }
                }
                points.push_back(start);
                points.push_back(finish);
                const auto direction = finish - start;
                const auto distance = [&](Vec3 point)
                { return (point.x - start.x) * direction.x + (point.y - start.y) * direction.y; };
                std::sort(points.begin(), points.end(), [&](Vec3 a, Vec3 b) { return distance(a) < distance(b); });
                for (const auto point : points)
                {
                    const auto index = vertexIndex(point);
                    if (outline.empty() || outline.back() != index)
                    {
                        outline.push_back(index);
                    }
                }
            }
            if (outline.size() > 1 && outline.front() == outline.back())
            {
                outline.pop_back();
            }
            if (outline.size() == 3)
            {
                continue;
            }
            const auto centerPoint = (original.vertices[face.vertices[0]] + original.vertices[face.vertices[1]] +
                                      original.vertices[face.vertices[2]]) /
                                     3.0F;
            std::vector<std::array<std::uint32_t, 3>> refined;
            bool interiorCenter = true;
            for (std::size_t edge{}; edge < outline.size(); ++edge)
            {
                const auto a = candidate.mesh.vertices[outline[edge]];
                const auto b = candidate.mesh.vertices[outline[(edge + 1) % outline.size()]];
                if ((a.x != b.x || a.y != b.y) && SignedArea(a, b, centerPoint) <= 0)
                {
                    interiorCenter = false;
                }
            }
            if (interiorCenter)
            {
                const auto center = vertexIndex(centerPoint);
                for (std::size_t edge{}; edge < outline.size(); ++edge)
                {
                    const auto a = candidate.mesh.vertices[outline[edge]];
                    const auto b = candidate.mesh.vertices[outline[(edge + 1) % outline.size()]];
                    if (a.x != b.x || a.y != b.y)
                    {
                        refined.push_back({outline[edge], outline[(edge + 1) % outline.size()], center});
                    }
                }
            }
            else
            {
                // A thin clipped triangle may contain no representable float
                // centroid. Boundary triangulation needs no new interior point.
                std::vector<Vec3> points;
                std::vector<std::uint32_t> boundary;
                for (const auto vertex : outline)
                {
                    boundary.push_back(static_cast<std::uint32_t>(points.size()));
                    points.push_back(candidate.mesh.vertices[vertex]);
                }
                for (const auto &triangle : TriangulateDetailSamples(points, boundary))
                {
                    refined.push_back({outline[triangle[0]], outline[triangle[1]], outline[triangle[2]]});
                }
            }
            std::vector<std::uint32_t> children;
            const auto source = candidate.polygonSourceTriangles.at(polygon);
            const auto contributors = candidate.polygonContributingTriangles.at(polygon);
            for (const auto &triangleVertices : refined)
            {
                NavPolygon child{.vertices = triangleVertices, .neighbors = {Open, Open, Open}, .flags = face.flags};
                if (SignedArea(candidate.mesh.vertices[child.vertices[0]], candidate.mesh.vertices[child.vertices[1]],
                               candidate.mesh.vertices[child.vertices[2]]) <= 0)
                {
                    throw std::runtime_error(
                        std::format("Generated seam subdivision creates a degenerate triangle in polygon {}", polygon));
                }
                const auto index =
                    children.empty() ? polygon : static_cast<std::uint32_t>(candidate.mesh.polygons.size());
                if (children.empty())
                {
                    candidate.mesh.polygons[polygon] = child;
                }
                else
                {
                    candidate.mesh.polygons.push_back(child);
                    candidate.polygonSourceTriangles.push_back(source);
                    candidate.polygonContributingTriangles.push_back(contributors);
                    candidate.regions.at(regions.at(polygon)).polygons.push_back(index);
                }
                children.push_back(index);
            }
            descendants[polygon] = children;
            for (auto &door : candidate.exits)
            {
                if (door.polygon != polygon)
                {
                    continue;
                }
                for (const auto child : children)
                {
                    const auto &triangle = candidate.mesh.polygons[child];
                    const auto a = candidate.mesh.vertices[triangle.vertices[0]];
                    const auto b = candidate.mesh.vertices[triangle.vertices[1]];
                    const auto c = candidate.mesh.vertices[triangle.vertices[2]];
                    if (SignedArea(a, b, door.position) >= 0 && SignedArea(b, c, door.position) >= 0 &&
                        SignedArea(c, a, door.position) >= 0)
                    {
                        door.polygon = child;
                        break;
                    }
                }
            }
        }
        const auto lookup = EdgeLookup(candidate, descendants);
        for (auto &link : candidate.borderLinks)
        {
            const auto &face = original.polygons.at(link.polygon);
            const DirectedEdge endpoints{Coordinate(original.vertices.at(face.vertices[link.edge])),
                                         Coordinate(original.vertices.at(face.vertices[(link.edge + 1) % 3]))};
            const auto found = lookup.find({link.polygon, endpoints});
            if (found == lookup.end())
            {
                throw std::runtime_error("Generated seam refinement cannot preserve an authored portal");
            }
            link.polygon = found->second.first;
            link.edge = found->second.second;
        }
        // Endpoint duplication can leave an unused original vertex. Keep the
        // no-orphan invariant while preserving polygon, portal and door indices.
        std::vector<std::uint32_t> remap(candidate.mesh.vertices.size(), Open);
        std::vector<Vec3> compact;
        for (auto &face : candidate.mesh.polygons)
        {
            for (auto &vertex : face.vertices)
            {
                if (remap[vertex] == Open)
                {
                    remap[vertex] = static_cast<std::uint32_t>(compact.size());
                    compact.push_back(candidate.mesh.vertices[vertex]);
                }
                vertex = remap[vertex];
            }
        }
        candidate.mesh.vertices = std::move(compact);
        RefreshCandidateTopology(candidate);
        return lookup;
    }
    /// Refining another side can move an existing portal onto a child triangle.
    /// Resolve both directions by their immutable endpoints after all local remaps.
    void RefreshGeneratedDestinations(std::span<GeneratedCellCandidate> cells)
    {
        using Target = std::tuple<std::uint32_t, std::uint32_t, DirectedEdge>;
        std::map<Target, EdgeIndex> targets;
        for (const auto &cell : cells)
        {
            for (const auto &link : cell.candidate->borderLinks)
            {
                if (!link.generatedNeighborCell)
                {
                    continue;
                }
                const auto &face = cell.candidate->mesh.polygons.at(link.polygon);
                const DirectedEdge edge{
                    Coordinate(cell.candidate->mesh.vertices.at(face.vertices[link.edge])),
                    Coordinate(cell.candidate->mesh.vertices.at(face.vertices[(link.edge + 1) % 3]))};
                if (!targets
                         .emplace(Target{cell.cellId, *link.generatedNeighborCell, edge},
                                  EdgeIndex{link.polygon, link.edge})
                         .second)
                {
                    throw std::runtime_error("Generated seam has duplicate reciprocal endpoints");
                }
            }
        }
        for (const auto &cell : cells)
        {
            for (auto &link : cell.candidate->borderLinks)
            {
                if (!link.generatedNeighborCell)
                {
                    continue;
                }
                const auto &face = cell.candidate->mesh.polygons.at(link.polygon);
                const DirectedEdge reversed{
                    Coordinate(cell.candidate->mesh.vertices.at(face.vertices[(link.edge + 1) % 3])),
                    Coordinate(cell.candidate->mesh.vertices.at(face.vertices[link.edge]))};
                const auto target = targets.at({*link.generatedNeighborCell, cell.cellId, reversed});
                link.neighborPolygon = target.first;
                link.neighborEdge = target.second;
            }
        }
    }
} // namespace

namespace navmesh::core
{
    std::size_t StitchGeneratedCandidates(std::span<GeneratedCellCandidate> cells)
    {
        std::map<std::tuple<std::uint32_t, float, float>, std::size_t> byPosition;
        for (std::size_t index{}; index < cells.size(); ++index)
        {
            const auto &cell = cells[index];
            if (!cell.candidate ||
                !byPosition.emplace(std::tuple{cell.worldspaceId, cell.bounds.min.x, cell.bounds.min.y}, index).second)
            {
                throw std::runtime_error("Batch contains duplicate or invalid exterior candidates");
            }
        }
        // Welding shared corners can make another previously unreachable interval
        // compatible. Consumed portals remain pinned on subsequent passes, so every
        // pass adds links without reopening existing crossings or removing floors.
        std::size_t added{};
        while (true)
        {
            StitchPlan plan;
            for (std::size_t index{}; index < cells.size(); ++index)
            {
                const auto &cell = cells[index];
                for (const bool vertical : {true, false})
                {
                    const auto boundary = vertical ? cell.bounds.max.x : cell.bounds.max.y;
                    const auto next = byPosition.find({cell.worldspaceId, vertical ? boundary : cell.bounds.min.x,
                                                       vertical ? cell.bounds.min.y : boundary});
                    if (next != byPosition.end())
                    {
                        PlanSeam(plan, cells, index, next->second, vertical, boundary);
                    }
                }
            }
            if (plan.portals.empty())
            {
                return added;
            }
            WeldEndpoints(plan, cells);
            std::vector<std::map<EdgeIndex, std::vector<Vec3>>> cuts(cells.size());
            for (const auto &portal : plan.portals)
            {
                for (const auto endpoint : {portal.first, portal.last})
                {
                    cuts[portal.left][portal.leftEdge].push_back(plan.endpoints[endpoint].point);
                    cuts[portal.right][portal.rightEdge].push_back(plan.endpoints[endpoint].point);
                }
            }
            std::vector<RefinedEdges> lookups;
            for (std::size_t index{}; index < cells.size(); ++index)
            {
                try
                {
                    lookups.push_back(ApplyPartitions(*cells[index].candidate, cuts[index]));
                }
                catch (const std::exception &error)
                {
                    throw std::runtime_error(std::format("CELL {:08X}: {}", cells[index].cellId, error.what()));
                }
                if (!cells[index].candidate->topology.valid)
                {
                    throw std::runtime_error(std::format("CELL {:08X}: {}", cells[index].cellId,
                                                         cells[index].candidate->topology.findings.front()));
                }
            }
            RefreshGeneratedDestinations(cells);
            for (const auto &portal : plan.portals)
            {
                const auto first = Coordinate(plan.endpoints[portal.first].point);
                const auto last = Coordinate(plan.endpoints[portal.last].point);
                const DirectedEdge leftKey =
                    portal.leftIncreasing ? DirectedEdge{first, last} : DirectedEdge{last, first};
                const auto &left = lookups[portal.left].at({portal.leftEdge.first, leftKey});
                const auto &right = lookups[portal.right].at({portal.rightEdge.first, {leftKey.second, leftKey.first}});
                cells[portal.left].candidate->borderLinks.push_back(
                    {left.first, left.second, 0, right.first, right.second, cells[portal.right].cellId});
                cells[portal.right].candidate->borderLinks.push_back(
                    {right.first, right.second, 0, left.first, left.second, cells[portal.left].cellId});
            }
            added += plan.portals.size();
        }
    }
} // namespace navmesh::core
