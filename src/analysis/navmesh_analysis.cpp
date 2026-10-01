#include "analysis/navmesh_analysis.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <numeric>
#include <numbers>
#include <map>
#include <set>
#include <sstream>
#include <vector>

namespace
{
    [[nodiscard]] navmesh::core::Vec3 Add(const navmesh::core::Vec3 &a, const navmesh::core::Vec3 &b)
    {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }

    [[nodiscard]] navmesh::core::Vec3 Subtract(const navmesh::core::Vec3 &a, const navmesh::core::Vec3 &b)
    {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }

    [[nodiscard]] navmesh::core::Vec3 Cross(const navmesh::core::Vec3 &a, const navmesh::core::Vec3 &b)
    {
        return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    }

    [[nodiscard]] double Dot(const navmesh::core::Vec3 &a, const navmesh::core::Vec3 &b)
    {
        return static_cast<double>(a.x * b.x + a.y * b.y + a.z * b.z);
    }

    [[nodiscard]] navmesh::core::Vec3 Normalize(const navmesh::core::Vec3 &value)
    {
        const auto length = std::sqrt(static_cast<double>(value.x * value.x + value.y * value.y + value.z * value.z));
        if (length <= 0.0)
        {
            return {0.0F, 0.0F, 0.0F};
        }
        const auto scale = 1.0 / length;
        return {static_cast<float>(value.x * scale), static_cast<float>(value.y * scale),
                static_cast<float>(value.z * scale)};
    }

    [[nodiscard]] double TriangleArea(const navmesh::core::Vec3 &a, const navmesh::core::Vec3 &b,
                                      const navmesh::core::Vec3 &c)
    {
        const auto ab = navmesh::core::Vec3{b.x - a.x, b.y - a.y, b.z - a.z};
        const auto ac = navmesh::core::Vec3{c.x - a.x, c.y - a.y, c.z - a.z};
        const auto cross = Cross(ab, ac);
        return 0.5 * std::sqrt(static_cast<double>(cross.x * cross.x + cross.y * cross.y + cross.z * cross.z));
    }

    [[nodiscard]] int FindRoot(std::vector<int> &parents, int index)
    {
        while (parents[index] != index)
        {
            parents[index] = parents[parents[index]];
            index = parents[index];
        }
        return index;
    }

    void Union(std::vector<int> &parents, int a, int b)
    {
        const auto rootA = FindRoot(parents, a);
        const auto rootB = FindRoot(parents, b);
        if (rootA == rootB)
        {
            return;
        }
        parents[rootB] = rootA;
    }

    [[nodiscard]] navmesh::core::AABB MakeBounds(const std::vector<navmesh::core::Vec3> &vertices)
    {
        navmesh::core::AABB bounds{};
        for (const auto &vertex : vertices)
        {
            bounds.Expand(vertex);
        }
        return bounds;
    }

    [[nodiscard]] navmesh::core::Vec3 TriangleNormal(const navmesh::core::Vec3 &a, const navmesh::core::Vec3 &b,
                                                     const navmesh::core::Vec3 &c)
    {
        return Normalize(Cross(Subtract(b, a), Subtract(c, a)));
    }

    [[nodiscard]] std::optional<navmesh::analysis::SurfaceHit> IntersectTriangle(
        const navmesh::core::Vec3 &origin, const navmesh::core::Vec3 &direction, const navmesh::core::Vec3 &a,
        const navmesh::core::Vec3 &b, const navmesh::core::Vec3 &c, std::size_t triangleIndex, float maxDistance)
    {
        const auto rawNormal = TriangleNormal(a, b, c);

        const double normalLength =
            std::sqrt(static_cast<double>(rawNormal.x * rawNormal.x) + static_cast<double>(rawNormal.y * rawNormal.y) +
                      static_cast<double>(rawNormal.z * rawNormal.z));

        if (normalLength < 1.0e-6)
        {
            return std::nullopt;
        }

        const auto normal = Normalize(rawNormal);

        const double denominator = Dot(normal, direction);

        if (std::abs(denominator) < 1.0e-6)
        {
            return std::nullopt;
        }

        const float t = static_cast<float>(Dot(Subtract(a, origin), normal) / denominator);

        if (t < 0.0F || t > maxDistance)
        {
            return std::nullopt;
        }

        const auto hit = Add(origin, {direction.x * t, direction.y * t, direction.z * t});

        const auto edge0 = Subtract(b, a);
        const auto edge1 = Subtract(c, b);
        const auto edge2 = Subtract(a, c);

        const auto c0 = Subtract(hit, a);
        const auto c1 = Subtract(hit, b);
        const auto c2 = Subtract(hit, c);

        constexpr float epsilon = 1.0e-4F;

        const auto inside0 = Dot(Cross(edge0, c0), normal) >= -epsilon;

        const auto inside1 = Dot(Cross(edge1, c1), normal) >= -epsilon;

        const auto inside2 = Dot(Cross(edge2, c2), normal) >= -epsilon;

        if (!inside0 || !inside1 || !inside2)
        {
            return std::nullopt;
        }

        return navmesh::analysis::SurfaceHit{
            .point = hit, .normal = normal, .distance = t, .triangleIndex = triangleIndex};
    }

    [[nodiscard]] std::vector<double> SortedValues(const std::vector<double> &values)
    {
        auto copy = values;
        std::sort(copy.begin(), copy.end());
        return copy;
    }

    [[nodiscard]] double Percentile(const std::vector<double> &sortedValues, double fraction)
    {
        if (sortedValues.empty())
        {
            return 0.0;
        }
        if (sortedValues.size() == 1)
        {
            return sortedValues.front();
        }
        const auto index =
            std::clamp(static_cast<std::size_t>(std::ceil(fraction * static_cast<double>(sortedValues.size())) - 1.0),
                       std::size_t{0}, sortedValues.size() - 1U);
        return sortedValues[index];
    }
    /// Report adjacency, shared-edge components, and exterior border gaps without altering support classifications.
    void AppendEdgeTopology(const navmesh::core::NavMesh &mesh, navmesh::analysis::AnalysisReport &report,
                            const navmesh::analysis::AnalysisConfiguration &configuration)
    {
        namespace core = navmesh::core;
        using namespace navmesh::analysis;
        auto topology =
            [&](std::string kind, std::vector<std::uint32_t> polygons, float confidence, std::string evidence)
        { report.topology.push_back({std::move(kind), std::move(polygons), confidence, std::move(evidence)}); };
        // Topology is reported separately from surface support.  These checks
        // are deterministic and intentionally produce review candidates only.
        std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<std::uint32_t>> edges;
        for (std::uint32_t polygonIndex{}; polygonIndex < mesh.polygons.size(); ++polygonIndex)
        {
            const auto &polygon = mesh.polygons[polygonIndex];
            for (std::size_t side{}; side < 3; ++side)
            {
                auto first = polygon.vertices[side], second = polygon.vertices[(side + 1) % 3];
                if (first > second)
                {
                    std::swap(first, second);
                }
                edges[{first, second}].push_back(polygonIndex);
                const auto neighbor = polygon.neighbors[side];
                if (neighbor != 0 && (neighbor >= mesh.polygons.size() ||
                                      !std::ranges::contains(mesh.polygons[neighbor].neighbors, polygonIndex)))
                {
                    topology("invalid_adjacency", {polygonIndex, neighbor}, 0.95F,
                             "non-zero neighbor does not reciprocate or is outside the NAVM polygon array");
                }
            }
        }
        std::vector<int> parents(mesh.polygons.size());
        std::iota(parents.begin(), parents.end(), 0);
        for (const auto &[edge, polygons] : edges)
        {
            if (polygons.size() == 2)
            {
                Union(parents, static_cast<int>(polygons[0]), static_cast<int>(polygons[1]));
            }
            if (polygons.size() > 2)
            {
                topology("non_manifold_edge", polygons, 0.95F, "more than two polygons share one NAVM edge");
            }
            if (polygons.size() == 1 && configuration.cellBounds && edge.first < mesh.vertices.size() &&
                edge.second < mesh.vertices.size())
            {
                const auto &first = mesh.vertices[edge.first];
                const auto &second = mesh.vertices[edge.second];
                const auto &cell = *configuration.cellBounds;
                constexpr float epsilon = 0.05F;
                const bool border =
                    (std::abs(first.x - cell.min.x) < epsilon && std::abs(second.x - cell.min.x) < epsilon) ||
                    (std::abs(first.x - cell.max.x) < epsilon && std::abs(second.x - cell.max.x) < epsilon) ||
                    (std::abs(first.y - cell.min.y) < epsilon && std::abs(second.y - cell.min.y) < epsilon) ||
                    (std::abs(first.y - cell.max.y) < epsilon && std::abs(second.y - cell.max.y) < epsilon);
                if (border)
                {
                    topology("cross_cell_border_gap", polygons, 0.45F,
                             "open NAVM edge lies on the exterior cell border; the adjoining cell must be reviewed");
                }
            }
        }
        std::map<int, std::vector<std::uint32_t>> components;
        for (std::uint32_t polygon{}; polygon < mesh.polygons.size(); ++polygon)
        {
            components[FindRoot(parents, static_cast<int>(polygon))].push_back(polygon);
        }
        if (components.size() > 1)
        {
            for (const auto &[_, polygons] : components)
            {
                topology("disconnected_component", polygons, 0.45F,
                         "component has no shared NAVM edge with the other components");
                if (polygons.size() == 1)
                {
                    topology("off_mesh_island", polygons, 0.70F, "single polygon has no shared NAVM edge");
                }
            }
        }
    }

    /// Search neighboring world-coordinate bins for coincident vertices with distinct indices.
    void AppendCoincidentVertexFindings(const navmesh::core::NavMesh &mesh, navmesh::analysis::AnalysisReport &report)
    {
        namespace core = navmesh::core;
        using namespace navmesh::analysis;
        auto topology =
            [&](std::string kind, std::vector<std::uint32_t> polygons, float confidence, std::string evidence)
        { report.topology.push_back({std::move(kind), std::move(polygons), confidence, std::move(evidence)}); };
        // Only vertices in the same or neighboring tolerance bins can be coincident.
        using VertexBin = std::array<std::int64_t, 3>;
        std::map<VertexBin, std::vector<std::uint32_t>> vertexBins;
        constexpr double duplicateTolerance = 0.01;
        const auto vertexBin = [](const core::Vec3 &vertex)
        {
            return VertexBin{static_cast<std::int64_t>(std::floor(vertex.x / duplicateTolerance)),
                             static_cast<std::int64_t>(std::floor(vertex.y / duplicateTolerance)),
                             static_cast<std::int64_t>(std::floor(vertex.z / duplicateTolerance))};
        };
        for (std::uint32_t second{}; second < mesh.vertices.size(); ++second)
        {
            const auto bin = vertexBin(mesh.vertices[second]);
            for (std::int64_t x = bin[0] - 1; x <= bin[0] + 1; ++x)
            {
                for (std::int64_t y = bin[1] - 1; y <= bin[1] + 1; ++y)
                {
                    for (std::int64_t z = bin[2] - 1; z <= bin[2] + 1; ++z)
                    {
                        const auto found = vertexBins.find({x, y, z});
                        if (found == vertexBins.end())
                        {
                            continue;
                        }
                        for (const auto first : found->second)
                        {
                            const auto delta = Subtract(mesh.vertices[first], mesh.vertices[second]);
                            if (Dot(delta, delta) < 0.0001)
                            {
                                topology("duplicate_vertex_or_gap", {}, 0.85F,
                                         std::format("vertices {} and {} are coincident but use distinct indices",
                                                     first, second));
                            }
                        }
                    }
                }
            }
            vertexBins[bin].push_back(second);
        }
    }

    /// Report strictly overlapping XY projections among nearby triangles that share no vertex index.
    void AppendOverlapFindings(const navmesh::core::NavMesh &mesh, navmesh::analysis::AnalysisReport &report)
    {
        namespace core = navmesh::core;
        using namespace navmesh::analysis;
        auto topology =
            [&](std::string kind, std::vector<std::uint32_t> polygons, float confidence, std::string evidence)
        { report.topology.push_back({std::move(kind), std::move(polygons), confidence, std::move(evidence)}); };
        const auto strictlyInsideXY =
            [](const core::Vec3 &point, const core::Vec3 &a, const core::Vec3 &b, const core::Vec3 &c)
        {
            const auto cross2 = [](const core::Vec3 &left, const core::Vec3 &right, const core::Vec3 &value)
            { return (right.x - left.x) * (value.y - left.y) - (right.y - left.y) * (value.x - left.x); };
            const auto one = cross2(a, b, point), two = cross2(b, c, point), three = cross2(c, a, point);
            return (one > 0 && two > 0 && three > 0) || (one < 0 && two < 0 && three < 0);
        };
        std::vector<core::Triangle> navmeshTriangles;
        navmeshTriangles.reserve(mesh.polygons.size());
        for (const auto &polygon : mesh.polygons)
        {
            navmeshTriangles.push_back({polygon.vertices});
        }
        SpatialIndex polygonIndex;
        polygonIndex.Build(navmeshTriangles, mesh.vertices);
        for (std::uint32_t first{}; first < mesh.polygons.size(); ++first)
        {
            const auto &left = mesh.polygons[first];
            if (std::any_of(left.vertices.begin(), left.vertices.end(),
                            [&](auto vertex) { return vertex >= mesh.vertices.size(); }))
            {
                continue;
            }
            const auto a = mesh.vertices[left.vertices[0]], b = mesh.vertices[left.vertices[1]],
                       c = mesh.vertices[left.vertices[2]];
            const core::AABB query{
                .min = {std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}), std::numeric_limits<float>::lowest()},
                .max = {std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y}), std::numeric_limits<float>::max()}};
            auto nearby = polygonIndex.QueryAABB(query);
            std::sort(nearby.begin(), nearby.end());
            for (const auto second : nearby)
            {
                if (second <= first)
                {
                    continue;
                }
                const auto &right = mesh.polygons[second];
                bool shared{};
                for (const auto leftVertex : left.vertices)
                {
                    for (const auto rightVertex : right.vertices)
                    {
                        shared |= leftVertex == rightVertex;
                    }
                }
                if (!shared && (strictlyInsideXY(a, mesh.vertices[right.vertices[0]], mesh.vertices[right.vertices[1]],
                                                 mesh.vertices[right.vertices[2]]) ||
                                strictlyInsideXY(mesh.vertices[right.vertices[0]], a, b, c)))
                {
                    topology("overlap", {first, static_cast<std::uint32_t>(second)}, 0.80F,
                             "a NAVM triangle vertex lies strictly inside another triangle in XY projection");
                }
            }
        }
    }

    /// Turn confident surface and topology evidence into deterministic manual-review entries, preserving finding order.
    void BuildRepairCandidates(const navmesh::core::NavMesh &mesh, navmesh::analysis::AnalysisReport &report)
    {
        namespace core = navmesh::core;
        using namespace navmesh::analysis;
        for (const auto &polygon : report.polygons)
        {
            if ((polygon.classification == "floating" || polygon.classification == "buried" ||
                 polygon.classification == "too_steep" || polygon.classification == "blocked") &&
                polygon.support.confidence >= 0.55F)
            {
                report.repairCandidates.push_back(
                    {std::format("navm-{:08X}-polygon-{}-{}", mesh.id, polygon.index, polygon.classification),
                     polygon.classification,
                     {polygon.index},
                     polygon.support.confidence,
                     "manual_review",
                     std::format("{} of {} samples agree; source={}", polygon.support.samplesCovered,
                                 polygon.support.samplesTotal, polygon.support.sourceType)});
            }
        }
        for (const auto &finding : report.topology)
        {
            if (finding.confidence >= 0.55F)
            {
                report.repairCandidates.push_back(
                    {std::format("navm-{:08X}-topology-{}-{}", mesh.id, finding.kind, report.repairCandidates.size()),
                     finding.kind, finding.polygons, finding.confidence, "manual_review", finding.evidence});
            }
        }
    }

} // namespace

namespace navmesh::analysis
{
    struct SupportCandidate
    {
        SurfaceHit hit{};
        float heightDelta{}; // navmesh sample Z - surface Z
        float slopeDegrees{};
        float upDot{};
        std::size_t sampleIndex{};
    };

    void SpatialIndex::Build(const std::vector<core::Triangle> &triangles,
                             const std::vector<core::Vec3> &triangleVertices)
    {
        this->vertices = triangleVertices;
        entries.clear();
        entries.reserve(triangles.size());
        for (std::size_t index = 0; index < triangles.size(); ++index)
        {
            const auto &triangle = triangles[index];
            if (triangle.vertices[0] >= triangleVertices.size() || triangle.vertices[1] >= triangleVertices.size() ||
                triangle.vertices[2] >= triangleVertices.size())
            {
                continue;
            }
            const auto a = triangleVertices[triangle.vertices[0]];
            const auto b = triangleVertices[triangle.vertices[1]];
            const auto c = triangleVertices[triangle.vertices[2]];
            core::AABB bounds{};
            bounds.Expand(a);
            bounds.Expand(b);
            bounds.Expand(c);
            entries.push_back({.bounds = bounds, .triangle = triangle, .triangleIndex = index});
        }
        // A compact BVH keeps repeated per-sample coverage queries from
        // scanning a whole exterior scene. Entries retain their original
        // triangle index so provenance joins remain stable.
        orderedEntries.resize(entries.size());
        std::iota(orderedEntries.begin(), orderedEntries.end(), 0);
        nodes.clear();
        const auto buildNode = [&](auto &&self, std::size_t first, std::size_t count) -> std::size_t
        {
            BvhNode node{.first = first, .count = count};
            for (std::size_t i = first; i < first + count; ++i)
            {
                node.bounds.Expand(entries[orderedEntries[i]].bounds.min),
                    node.bounds.Expand(entries[orderedEntries[i]].bounds.max);
            }
            const auto nodeIndex = nodes.size();
            nodes.push_back(node);
            if (count <= 8)
            {
                nodes[nodeIndex].leaf = true;
                return nodeIndex;
            }
            const auto extent = node.bounds.Extent();
            const int axis = extent.x >= extent.y && extent.x >= extent.z ? 0 : extent.y >= extent.z ? 1 : 2;
            const auto middle = first + count / 2;
            std::nth_element(orderedEntries.begin() + static_cast<std::ptrdiff_t>(first),
                             orderedEntries.begin() + static_cast<std::ptrdiff_t>(middle),
                             orderedEntries.begin() + static_cast<std::ptrdiff_t>(first + count),
                             [&](std::size_t left, std::size_t right)
                             {
                                 const auto &a = entries[left].bounds.Center();
                                 const auto &b = entries[right].bounds.Center();
                                 return axis == 0 ? a.x < b.x : axis == 1 ? a.y < b.y : a.z < b.z;
                             });
            nodes[nodeIndex].left = self(self, first, middle - first);
            nodes[nodeIndex].right = self(self, middle, first + count - middle);
            return nodeIndex;
        };
        if (!entries.empty())
        {
            buildNode(buildNode, 0, entries.size());
        }
    }

    std::vector<std::size_t> SpatialIndex::QueryAABB(const core::AABB &bounds) const
    {
        std::vector<std::size_t> hits;
        if (nodes.empty())
        {
            return hits;
        }
        const auto queryNode = [&](auto &&self, std::size_t nodeIndex) -> void
        {
            const auto &node = nodes[nodeIndex];
            if (!node.bounds.Intersects(bounds))
            {
                return;
            }
            if (node.leaf)
            {
                for (std::size_t i = node.first; i < node.first + node.count; ++i)
                {
                    const auto &entry = entries[orderedEntries[i]];
                    if (entry.bounds.Intersects(bounds))
                    {
                        hits.push_back(entry.triangleIndex);
                    }
                }
                return;
            }
            self(self, node.left);
            self(self, node.right);
        };
        queryNode(queryNode, 0);
        return hits;
    }

    std::optional<SurfaceHit> SpatialIndex::NearestSurface(const core::Vec3 &point, float maxDistance) const
    {
        const auto rayLength = std::max(1000.0F, maxDistance * 4.0F + 1024.0F);
        const auto origin = core::Vec3{point.x, point.y, point.z + rayLength};
        return Raycast(origin, {0.0F, 0.0F, -1.0F}, rayLength);
    }

    std::optional<SurfaceHit> SpatialIndex::Raycast(const core::Vec3 &origin, const core::Vec3 &direction,
                                                    float maxDistance) const
    {
        std::optional<SurfaceHit> best;
        float closestDistance = maxDistance;
        for (const auto &entry : entries)
        {
            if (entry.triangle.vertices[0] >= vertices.size() || entry.triangle.vertices[1] >= vertices.size() ||
                entry.triangle.vertices[2] >= vertices.size())
            {
                continue;
            }
            const auto a = vertices[entry.triangle.vertices[0]];
            const auto b = vertices[entry.triangle.vertices[1]];
            const auto c = vertices[entry.triangle.vertices[2]];
            const auto hit = IntersectTriangle(origin, direction, a, b, c, entry.triangleIndex, maxDistance);
            if (!hit)
            {
                continue;
            }
            if (hit->distance < closestDistance)
            {
                closestDistance = hit->distance;
                best = *hit;
            }
        }
        return best;
    }

    std::size_t SpatialIndex::Size() const noexcept
    {
        return entries.size();
    }

    float SurfaceSlopeDegrees(const core::Vec3 &normal)
    {
        const auto unit = Normalize(normal);
        const auto vertical = core::Vec3{0.0F, 0.0F, 1.0F};
        const auto cosine = std::clamp(std::abs(Dot(unit, vertical)), 0.0, 1.0);
        return static_cast<float>(std::acos(static_cast<double>(cosine)) * 180.0 / std::numbers::pi);
    }

    [[nodiscard]] float HorizontalDistance(const core::Vec3 &a, const core::Vec3 &b)
    {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    static std::vector<core::Vec3> MakeSupportSamples(const core::Vec3 &a, const core::Vec3 &b, const core::Vec3 &c)
    {
        return {
            // Centroid
            a * (1.0F / 3.0F) + b * (1.0F / 3.0F) + c * (1.0F / 3.0F),

            // Near each vertex, but safely inside
            a * 0.80F + b * 0.10F + c * 0.10F,
            a * 0.10F + b * 0.80F + c * 0.10F,
            a * 0.10F + b * 0.10F + c * 0.80F,

            // Three additional interior samples
            a * 0.50F + b * 0.40F + c * 0.10F,
            a * 0.10F + b * 0.50F + c * 0.40F,
            a * 0.40F + b * 0.10F + c * 0.50F,
        };
    }

    [[nodiscard]] std::optional<SupportCandidate> MakeSupportCandidate(const core::Vec3 &sample, const SurfaceHit &hit,
                                                                       std::size_t sampleIndex,
                                                                       float /*maxSupportDistance*/, float /*maxSlope*/)
    {
        const float heightDelta = sample.z - hit.point.z;

        const auto normal = Normalize(hit.normal);

        const float upDot = std::clamp(static_cast<float>(Dot(normal, core::Vec3{0.0F, 0.0F, 1.0F})), -1.0F, 1.0F);

        /*
        * Unlike the old implementation, don't use abs() here.
        *
        * A downward-facing surface is not a floor.
        */
        if (upDot <= 0.15F)
        {
            return std::nullopt;
        }

        const float slopeDegrees = SurfaceSlopeDegrees(normal);

        return SupportCandidate{.hit = hit,
                                .heightDelta = heightDelta,
                                .slopeDegrees = slopeDegrees,
                                .upDot = upDot,
                                .sampleIndex = sampleIndex};
    }

    std::string ClassifySupport(float heightDelta, float slopeDegrees, float maxSupportDistance, float maxSlope)
    {
        if (slopeDegrees > maxSlope)
        {
            return "too_steep";
        }
        if (heightDelta > maxSupportDistance)
        {
            return "floating";
        }
        if (heightDelta < -maxSupportDistance)
        {
            return "buried";
        }
        return "supported";
    }

    GeometrySummary AnalyzeGeometry(const core::Mesh &mesh)
    {
        GeometrySummary summary{};
        summary.vertexCount = mesh.vertices.size();
        summary.triangleCount = mesh.triangles.size();
        summary.bounds = MakeBounds(mesh.vertices);
        summary.center = summary.bounds.Center();
        summary.extent = summary.bounds.Extent();
        return summary;
    }

    NavMeshSummary AnalyzeNavMesh(const core::NavMesh &mesh)
    {
        NavMeshSummary summary{};
        summary.vertexCount = mesh.vertices.size();
        summary.polygonCount = mesh.polygons.size();
        summary.bounds = MakeBounds(mesh.vertices);
        summary.center = summary.bounds.Center();
        summary.extent = summary.bounds.Extent();
        return summary;
    }

    const char *SupportSourceName(SupportSourceType type)
    {
        switch (type)
        {
        case SupportSourceType::Terrain:
            return "terrain";
        case SupportSourceType::Collision:
            return "collision";
        case SupportSourceType::RenderFallback:
            return "render_fallback";
        default:
            return "unknown";
        }
    }

    AnalysisReport AnalyzeNavMeshPolygons(const core::NavMesh &mesh, const core::Mesh &geometry,
                                          const AnalysisConfiguration &configuration)
    {
        return AnalyzeNavMeshPolygons(mesh, geometry, {}, configuration);
    }

    AnalysisReport AnalyzeNavMeshPolygons(const core::NavMesh &mesh, const core::Mesh &geometry,
                                          const std::vector<TriangleSource> &sources,
                                          const AnalysisConfiguration &configuration)
    {
        struct Candidate
        {
            SupportCandidate hit;
            TriangleSource source;
        };
        struct Cluster
        {
            std::vector<const Candidate *> candidates;
            TriangleSource source;
            float delta{};
            float slope{};
        };
        const auto priority = [](SupportSourceType type)
        {
            return type == SupportSourceType::Collision        ? 3
                   : type == SupportSourceType::Terrain        ? 2
                   : type == SupportSourceType::RenderFallback ? 1
                                                               : 0;
        };
        AnalysisReport report{};
        report.configuration = configuration;
        report.polygons.reserve(mesh.polygons.size());
        SpatialIndex spatial;
        spatial.Build(geometry.triangles, geometry.vertices);
        const auto geometryBounds = geometry.Bounds();
        const float minZ = geometryBounds.IsValid() ? geometryBounds.min.z : 0.0F;
        const float maxZ = geometryBounds.IsValid() ? geometryBounds.max.z : 0.0F;
        const float rayMargin = std::max(4096.0F, configuration.maxSupportDistance * 8.0F);
        std::vector<double> heights, slopes;
        for (std::uint32_t index{}; index < mesh.polygons.size(); ++index)
        {
            const auto &polygon = mesh.polygons[index];
            if (polygon.vertices[0] >= mesh.vertices.size() || polygon.vertices[1] >= mesh.vertices.size() ||
                polygon.vertices[2] >= mesh.vertices.size())
            {
                continue;
            }
            const auto a = mesh.vertices[polygon.vertices[0]], b = mesh.vertices[polygon.vertices[1]],
                       c = mesh.vertices[polygon.vertices[2]];
            PolygonAnalysisResult result{.index = index,
                                         .centroid = (a + b + c) / 3.0F,
                                         .normal = TriangleNormal(a, b, c),
                                         .bounds = MakeBounds({a, b, c}),
                                         .vertexCount = 3};
            const auto samples = MakeSupportSamples(a, b, c);
            std::vector<Candidate> candidates;
            bool rawHit{};
            bool obstruction{};
            // Sample vertically across the polygon so a single centroid cannot decide coverage.
            for (std::size_t sampleIndex{}; sampleIndex < samples.size(); ++sampleIndex)
            {
                const auto &sample = samples[sampleIndex];
                const core::Vec3 origin{sample.x, sample.y, std::max(sample.z, maxZ) + rayMargin};
                const auto query = core::AABB{.min = {sample.x - configuration.surfaceSearchRadius,
                                                      sample.y - configuration.surfaceSearchRadius, minZ - rayMargin},
                                              .max = {sample.x + configuration.surfaceSearchRadius,
                                                      sample.y + configuration.surfaceSearchRadius, origin.z}};
                for (const auto triangleIndex : spatial.QueryAABB(query))
                {
                    if (triangleIndex >= geometry.triangles.size())
                    {
                        continue;
                    }
                    const auto &triangle = geometry.triangles[triangleIndex];
                    if (triangle.vertices[0] >= geometry.vertices.size() ||
                        triangle.vertices[1] >= geometry.vertices.size() ||
                        triangle.vertices[2] >= geometry.vertices.size())
                    {
                        continue;
                    }
                    const auto hit =
                        IntersectTriangle(origin, {0, 0, -1}, geometry.vertices[triangle.vertices[0]],
                                          geometry.vertices[triangle.vertices[1]],
                                          geometry.vertices[triangle.vertices[2]], triangleIndex, rayMargin * 2.0F);
                    if (!hit)
                    {
                        continue;
                    }
                    rawHit = true;
                    const auto candidate = MakeSupportCandidate(
                        sample, *hit, sampleIndex, configuration.maxSupportDistance, configuration.maxSlope);
                    if (!candidate)
                    {
                        continue;
                    }
                    const auto source =
                        triangleIndex < sources.size() ? sources[triangleIndex] : TriangleSource{.confidence = 1.0F};
                    if (source.type == SupportSourceType::Collision && candidate->heightDelta < 0.0F &&
                        candidate->heightDelta >= -configuration.obstructionClearance)
                    {
                        obstruction = true;
                    }
                    candidates.push_back({*candidate, source});
                }
            }
            // Group only hits with the same source identity and compatible height.
            // Candidate storage is complete before clusters retain pointers into it.
            std::vector<Cluster> clusters;
            for (const auto &candidate : candidates)
            {
                Cluster *cluster{};
                for (auto &current : clusters)
                {
                    if (current.source.type == candidate.source.type && current.source.id == candidate.source.id &&
                        std::abs(current.delta - candidate.hit.heightDelta) <= 2.0F)
                    {
                        cluster = &current;
                        break;
                    }
                }
                if (!cluster)
                {
                    clusters.push_back({.source = candidate.source});
                    cluster = &clusters.back();
                }
                cluster->candidates.push_back(&candidate);
                const auto count = static_cast<float>(cluster->candidates.size());
                cluster->delta += (candidate.hit.heightDelta - cluster->delta) / count;
                cluster->slope += (candidate.hit.slopeDegrees - cluster->slope) / count;
            }
            const auto coverage = [&](const Cluster &cluster)
            {
                std::set<std::size_t> samplesCovered;
                for (const auto *candidate : cluster.candidates)
                {
                    samplesCovered.insert(candidate->hit.sampleIndex);
                }
                return static_cast<float>(samplesCovered.size()) / static_cast<float>(samples.size());
            };
            Cluster *selected{};
            // Rank sufficient coverage by source priority, sample agreement, then height proximity.
            // Equal ranks preserve the first cluster so source joins remain deterministic.
            const auto isPreferred = [&](const Cluster &candidate, const Cluster &current)
            {
                const auto candidatePriority = priority(candidate.source.type);
                const auto currentPriority = priority(current.source.type);
                if (candidatePriority != currentPriority)
                {
                    return candidatePriority > currentPriority;
                }
                const auto candidateCoverage = coverage(candidate);
                const auto currentCoverage = coverage(current);
                if (candidateCoverage > currentCoverage)
                {
                    return true;
                }
                return candidateCoverage == currentCoverage && std::abs(candidate.delta) < std::abs(current.delta);
            };
            for (auto &cluster : clusters)
            {
                if (coverage(cluster) >= configuration.minimumCoverage &&
                    (!selected || isPreferred(cluster, *selected)))
                {
                    selected = &cluster;
                }
            }
            if (!selected)
            {
                result.classification = rawHit ? "ambiguous" : "out_of_coverage";
            }
            else
            {
                const auto conflicting = std::any_of(
                    clusters.begin(), clusters.end(),
                    [&](const Cluster &cluster)
                    {
                        return &cluster != selected &&
                               priority(cluster.source.type) == priority(selected->source.type) &&
                               coverage(cluster) >= configuration.minimumCoverage &&
                               std::abs(cluster.delta - selected->delta) > configuration.ambiguityHeightDelta;
                    });
                const auto *best = *std::min_element(
                    selected->candidates.begin(), selected->candidates.end(), [](const auto *left, const auto *right)
                    { return std::abs(left->hit.heightDelta) < std::abs(right->hit.heightDelta); });
                const auto agreement = coverage(*selected);
                const auto covered = static_cast<std::size_t>(std::round(agreement * samples.size()));
                result.support = {
                    .found = true,
                    .point = best->hit.hit.point,
                    .distance = best->hit.hit.distance,
                    .heightDelta = selected->delta,
                    .normal = best->hit.hit.normal,
                    .slopeDegrees = selected->slope,
                    .triangleIndex = best->hit.hit.triangleIndex,
                    .sourceType = SupportSourceName(selected->source.type),
                    .sourceConfidence = selected->source.confidence,
                    .confidence = std::clamp(
                        selected->source.confidence * agreement *
                            (1.0F - std::min(1.0F, std::abs(selected->delta) /
                                                       std::max(1.0F, configuration.maxSupportDistance * 4.0F))),
                        0.0F, 1.0F),
                    .samplesTotal = samples.size(),
                    .samplesCovered = covered,
                    .sampleAgreement = agreement};
                if (conflicting)
                {
                    result.classification = "ambiguous";
                }
                else if (obstruction && selected->delta >= 0.0F)
                {
                    result.classification = "blocked";
                }
                else
                {
                    result.classification = ClassifySupport(selected->delta, selected->slope,
                                                            configuration.maxSupportDistance, configuration.maxSlope);
                }
                heights.push_back(selected->delta);
                slopes.push_back(selected->slope);
            }
            ++report.summary.polygonsAnalyzed;
            if (result.support.found)
            {
                ++report.summary.supportFound;
            }
            if (result.classification == "supported")
            {
                ++report.summary.supported;
            }
            else if (result.classification == "floating")
            {
                ++report.summary.floating;
            }
            else if (result.classification == "buried")
            {
                ++report.summary.buried;
            }
            else if (result.classification == "too_steep")
            {
                ++report.summary.tooSteep;
            }
            else if (result.classification == "blocked")
            {
                ++report.summary.blocked;
            }
            else if (result.classification == "out_of_coverage")
            {
                ++report.summary.outOfCoverage;
            }
            else if (result.classification == "ambiguous")
            {
                ++report.summary.ambiguous;
            }
            report.polygons.push_back(std::move(result));
        }
        const auto calculate = [](const std::vector<double> &values, SummaryStats &output)
        {
            if (values.empty())
            {
                return;
            }
            const auto sorted = SortedValues(values);
            output = {.min = sorted.front(),
                      .max = sorted.back(),
                      .mean = std::accumulate(sorted.begin(), sorted.end(), 0.0) / static_cast<double>(sorted.size()),
                      .median = sorted[sorted.size() / 2],
                      .p95 = Percentile(sorted, 0.95)};
        };
        calculate(heights, report.heightDeltaStats);
        calculate(slopes, report.slopeStats);
        // Surface support and topology have independent evidence and confidence.
        AppendEdgeTopology(mesh, report, configuration);
        AppendCoincidentVertexFindings(mesh, report);
        AppendOverlapFindings(mesh, report);
        BuildRepairCandidates(mesh, report);
        report.summary.repairCandidates = report.repairCandidates.size();
        report.summary.topologyFindings = report.topology.size();
        return report;
    }

    NavMeshAnalysis Analyze(const core::NavMesh &mesh)
    {
        NavMeshAnalysis analysis{};
        analysis.vertexCount = static_cast<std::uint32_t>(mesh.vertices.size());
        analysis.polygonCount = static_cast<std::uint32_t>(mesh.polygons.size());

        if (!mesh.vertices.empty())
        {
            analysis.boundingBox.min = mesh.vertices.front();
            analysis.boundingBox.max = mesh.vertices.front();
            for (const auto &vertex : mesh.vertices)
            {
                analysis.boundingBox.min.x = std::min(analysis.boundingBox.min.x, vertex.x);
                analysis.boundingBox.min.y = std::min(analysis.boundingBox.min.y, vertex.y);
                analysis.boundingBox.min.z = std::min(analysis.boundingBox.min.z, vertex.z);
                analysis.boundingBox.max.x = std::max(analysis.boundingBox.max.x, vertex.x);
                analysis.boundingBox.max.y = std::max(analysis.boundingBox.max.y, vertex.y);
                analysis.boundingBox.max.z = std::max(analysis.boundingBox.max.z, vertex.z);
            }
        }

        std::vector<int> parents(analysis.polygonCount > 0 ? static_cast<std::size_t>(analysis.polygonCount) : 0);
        std::iota(parents.begin(), parents.end(), 0);

        // Sharing any vertex joins components; one prior polygon per vertex is sufficient.
        std::map<std::uint32_t, int> firstPolygonByVertex;
        for (std::uint32_t polygonIndex = 0; polygonIndex < analysis.polygonCount; ++polygonIndex)
        {
            for (const auto vertex : mesh.polygons[polygonIndex].vertices)
            {
                const auto [found, inserted] = firstPolygonByVertex.try_emplace(vertex, static_cast<int>(polygonIndex));
                if (!inserted)
                {
                    Union(parents, found->second, static_cast<int>(polygonIndex));
                }
            }
        }

        std::vector<int> componentSizes(analysis.polygonCount > 0 ? static_cast<std::size_t>(analysis.polygonCount) : 0,
                                        0);
        for (std::uint32_t polygonIndex = 0; polygonIndex < analysis.polygonCount; ++polygonIndex)
        {
            const auto root = FindRoot(parents, static_cast<int>(polygonIndex));
            ++componentSizes[root];
        }

        analysis.connectedComponents = static_cast<std::uint32_t>(
            std::count_if(componentSizes.begin(), componentSizes.end(), [](int size) { return size > 0; }));
        analysis.isolatedPolygonCount = static_cast<std::uint32_t>(
            std::count_if(componentSizes.begin(), componentSizes.end(), [](int size) { return size == 1; }));

        double totalArea = 0.0;
        double minArea = std::numeric_limits<double>::max();
        double maxArea = 0.0;
        for (const auto &polygon : mesh.polygons)
        {
            if (polygon.vertices[0] >= mesh.vertices.size() || polygon.vertices[1] >= mesh.vertices.size() ||
                polygon.vertices[2] >= mesh.vertices.size())
            {
                ++analysis.degeneratePolygonCount;
                continue;
            }
            const auto a = mesh.vertices[polygon.vertices[0]];
            const auto b = mesh.vertices[polygon.vertices[1]];
            const auto c = mesh.vertices[polygon.vertices[2]];
            const auto area = TriangleArea(a, b, c);
            const bool degenerate = area <= 1.0e-6 || (a.x == b.x && a.y == b.y && a.z == b.z) ||
                                    (a.x == c.x && a.y == c.y && a.z == c.z) ||
                                    (b.x == c.x && b.y == c.y && b.z == c.z);
            if (degenerate)
            {
                ++analysis.degeneratePolygonCount;
            }
            minArea = std::min(minArea, area);
            maxArea = std::max(maxArea, area);
            totalArea += area;
        }

        if (analysis.polygonCount > 0)
        {
            analysis.minPolygonArea = minArea == std::numeric_limits<double>::max() ? 0.0 : minArea;
            analysis.maxPolygonArea = maxArea;
            analysis.averagePolygonArea = totalArea / static_cast<double>(analysis.polygonCount);
        }

        return analysis;
    }
} // namespace navmesh::analysis
