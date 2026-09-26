#include "analysis/navmesh_analysis.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <numbers>
#include <vector>

namespace
{
    [[nodiscard]] navmesh::core::Vec3 Add(const navmesh::core::Vec3& a, const navmesh::core::Vec3& b)
    {
        return { a.x + b.x, a.y + b.y, a.z + b.z };
    }

    [[nodiscard]] navmesh::core::Vec3 Subtract(const navmesh::core::Vec3& a, const navmesh::core::Vec3& b)
    {
        return { a.x - b.x, a.y - b.y, a.z - b.z };
    }

    [[nodiscard]] navmesh::core::Vec3 Cross(const navmesh::core::Vec3& a, const navmesh::core::Vec3& b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }

    [[nodiscard]] double Dot(const navmesh::core::Vec3& a, const navmesh::core::Vec3& b)
    {
        return static_cast<double>(a.x * b.x + a.y * b.y + a.z * b.z);
    }

    [[nodiscard]] navmesh::core::Vec3 Normalize(const navmesh::core::Vec3& value)
    {
        const auto length = std::sqrt(static_cast<double>(value.x * value.x + value.y * value.y + value.z * value.z));
        if (length <= 0.0) {
            return { 0.0F, 0.0F, 0.0F };
        }
        const auto scale = 1.0 / length;
        return { static_cast<float>(value.x * scale), static_cast<float>(value.y * scale), static_cast<float>(value.z * scale) };
    }

    [[nodiscard]] double TriangleArea(const navmesh::core::Vec3& a, const navmesh::core::Vec3& b, const navmesh::core::Vec3& c)
    {
        const auto ab = navmesh::core::Vec3{ b.x - a.x, b.y - a.y, b.z - a.z };
        const auto ac = navmesh::core::Vec3{ c.x - a.x, c.y - a.y, c.z - a.z };
        const auto cross = Cross(ab, ac);
        return 0.5 * std::sqrt(static_cast<double>(cross.x * cross.x + cross.y * cross.y + cross.z * cross.z));
    }

    [[nodiscard]] int FindRoot(std::vector<int>& parents, int index)
    {
        while (parents[index] != index) {
            parents[index] = parents[parents[index]];
            index = parents[index];
        }
        return index;
    }

    void Union(std::vector<int>& parents, int a, int b)
    {
        const auto rootA = FindRoot(parents, a);
        const auto rootB = FindRoot(parents, b);
        if (rootA == rootB) {
            return;
        }
        parents[rootB] = rootA;
    }

    [[nodiscard]] navmesh::core::AABB MakeBounds(const std::vector<navmesh::core::Vec3>& vertices)
    {
        navmesh::core::AABB bounds{};
        for (const auto& vertex : vertices) {
            bounds.Expand(vertex);
        }
        return bounds;
    }

    [[nodiscard]] navmesh::core::Vec3 TriangleNormal(const navmesh::core::Vec3& a, const navmesh::core::Vec3& b, const navmesh::core::Vec3& c)
    {
        return Normalize(Cross(Subtract(b, a), Subtract(c, a)));
    }

    [[nodiscard]] std::optional<navmesh::analysis::SurfaceHit> IntersectTriangle(
        const navmesh::core::Vec3& origin,
        const navmesh::core::Vec3& direction,
        const navmesh::core::Vec3& a,
        const navmesh::core::Vec3& b,
        const navmesh::core::Vec3& c,
        std::size_t triangleIndex,
        float maxDistance)
    {
        const auto rawNormal = TriangleNormal(a, b, c);

        const double normalLength =
            std::sqrt(
                static_cast<double>(rawNormal.x * rawNormal.x) +
                static_cast<double>(rawNormal.y * rawNormal.y) +
                static_cast<double>(rawNormal.z * rawNormal.z));

        if (normalLength < 1.0e-6) {
            return std::nullopt;
        }

        const auto normal = Normalize(rawNormal);

        const double denominator = Dot(normal, direction);

        if (std::abs(denominator) < 1.0e-6) {
            return std::nullopt;
        }

        const float t =
            static_cast<float>(
                Dot(Subtract(a, origin), normal) / denominator);

        if (t < 0.0F || t > maxDistance) {
            return std::nullopt;
        }

        const auto hit = Add(
            origin,
            {
                direction.x * t,
                direction.y * t,
                direction.z * t
            });

        const auto edge0 = Subtract(b, a);
        const auto edge1 = Subtract(c, b);
        const auto edge2 = Subtract(a, c);

        const auto c0 = Subtract(hit, a);
        const auto c1 = Subtract(hit, b);
        const auto c2 = Subtract(hit, c);

        constexpr float epsilon = 1.0e-4F;

        const auto inside0 =
            Dot(Cross(edge0, c0), normal) >= -epsilon;

        const auto inside1 =
            Dot(Cross(edge1, c1), normal) >= -epsilon;

        const auto inside2 =
            Dot(Cross(edge2, c2), normal) >= -epsilon;

        if (!inside0 || !inside1 || !inside2) {
            return std::nullopt;
        }

        return navmesh::analysis::SurfaceHit{
            .point = hit,
            .normal = normal,
            .distance = t,
            .triangleIndex = triangleIndex
        };
    }

    [[nodiscard]] std::vector<double> SortedValues(const std::vector<double>& values)
    {
        auto copy = values;
        std::sort(copy.begin(), copy.end());
        return copy;
    }

    [[nodiscard]] double Percentile(const std::vector<double>& sortedValues, double fraction)
    {
        if (sortedValues.empty()) {
            return 0.0;
        }
        if (sortedValues.size() == 1) {
            return sortedValues.front();
        }
        const auto index = std::clamp(static_cast<std::size_t>(std::ceil(fraction * static_cast<double>(sortedValues.size())) - 1.0), std::size_t{ 0 }, sortedValues.size() - 1U);
        return sortedValues[index];
    }
}

namespace navmesh::analysis
{
    struct SupportCandidate
    {
        SurfaceHit hit{};
        float heightDelta{};   // navmesh sample Z - surface Z
        float slopeDegrees{};
        float upDot{};
        std::size_t sampleIndex{};
    };

    struct SupportCluster
    {
        std::vector<const SupportCandidate*> candidates;

        float meanHeightDelta{};
        float meanSlope{};
        float meanUpDot{};

        std::size_t sampleCount{};
    };

    void SpatialIndex::Build(const std::vector<core::Triangle>& triangles, const std::vector<core::Vec3>& triangleVertices)
    {
        this->vertices = triangleVertices;
        entries.clear();
        entries.reserve(triangles.size());
        for (std::size_t index = 0; index < triangles.size(); ++index) {
            const auto& triangle = triangles[index];
            if (triangle.vertices[0] >= triangleVertices.size() || triangle.vertices[1] >= triangleVertices.size() || triangle.vertices[2] >= triangleVertices.size()) {
                continue;
            }
            const auto a = triangleVertices[triangle.vertices[0]];
            const auto b = triangleVertices[triangle.vertices[1]];
            const auto c = triangleVertices[triangle.vertices[2]];
            core::AABB bounds{};
            bounds.Expand(a); bounds.Expand(b); bounds.Expand(c);
            entries.push_back({ .bounds = bounds, .triangle = triangle, .triangleIndex = index });
        }
    }

    std::vector<std::size_t> SpatialIndex::QueryAABB(const core::AABB& bounds) const
    {
        std::vector<std::size_t> hits;
        for (const auto& entry : entries) {
            if (entry.bounds.Intersects(bounds)) {
                hits.push_back(entry.triangleIndex);
            }
        }
        return hits;
    }

    std::optional<SurfaceHit> SpatialIndex::NearestSurface(const core::Vec3& point, float maxDistance) const
    {
        const auto rayLength = std::max(1000.0F, maxDistance * 4.0F + 1024.0F);
        const auto origin = core::Vec3{ point.x, point.y, point.z + rayLength };
        return Raycast(origin, { 0.0F, 0.0F, -1.0F }, rayLength);
    }

    std::optional<SurfaceHit> SpatialIndex::Raycast(const core::Vec3& origin, const core::Vec3& direction, float maxDistance) const
    {
        std::optional<SurfaceHit> best;
        float closestDistance = maxDistance;
        for (const auto& entry : entries) {
            if (entry.triangle.vertices[0] >= vertices.size() || entry.triangle.vertices[1] >= vertices.size() || entry.triangle.vertices[2] >= vertices.size()) {
                continue;
            }
            const auto a = vertices[entry.triangle.vertices[0]];
            const auto b = vertices[entry.triangle.vertices[1]];
            const auto c = vertices[entry.triangle.vertices[2]];
            const auto hit = IntersectTriangle(origin, direction, a, b, c, entry.triangleIndex, maxDistance);
            if (!hit) {
                continue;
            }
            if (hit->distance < closestDistance) {
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

    float SurfaceSlopeDegrees(const core::Vec3& normal)
    {
        const auto unit = Normalize(normal);
        const auto vertical = core::Vec3{ 0.0F, 0.0F, 1.0F };
        const auto cosine = std::clamp(std::abs(Dot(unit, vertical)), 0.0, 1.0);
        return static_cast<float>(std::acos(static_cast<double>(cosine)) * 180.0 / std::numbers::pi);
    }

    [[nodiscard]] float HorizontalDistance(const core::Vec3& a, const core::Vec3& b)
    {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    static std::vector<core::Vec3> MakeSupportSamples(
        const core::Vec3& a,
        const core::Vec3& b,
        const core::Vec3& c)
    {
        return {
            // Centroid
            a * (1.0F / 3.0F) +
            b * (1.0F / 3.0F) +
            c * (1.0F / 3.0F),

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

    [[nodiscard]] std::optional<SupportCandidate> MakeSupportCandidate(
        const core::Vec3& sample,
        const SurfaceHit& hit,
        std::size_t sampleIndex,
        float maxSupportDistance,
        float maxSlope)
    {
        const float heightDelta = sample.z - hit.point.z;

        /*
        * We are looking for the surface supporting the navmesh.
        *
        * Therefore the surface should normally be below the navmesh
        * sample. A very small tolerance is allowed for numerical/
        * navmesh-generation differences.
        */
        constexpr float aboveTolerance = 2.0F;

        if (heightDelta < -aboveTolerance) {
            return std::nullopt;
        }

        if (heightDelta > maxSupportDistance) {
            return std::nullopt;
        }

        const auto normal = Normalize(hit.normal);

        const float upDot = std::clamp(
            static_cast<float>(Dot(normal, core::Vec3{ 0.0F, 0.0F, 1.0F })),
            -1.0F,
            1.0F);

        /*
        * Unlike the old implementation, don't use abs() here.
        *
        * A downward-facing surface is not a floor.
        */
        if (upDot <= 0.15F) {
            return std::nullopt;
        }

        const float slopeDegrees = SurfaceSlopeDegrees(normal);

        if (slopeDegrees > maxSlope) {
            return std::nullopt;
        }

        return SupportCandidate{
            .hit = hit,
            .heightDelta = heightDelta,
            .slopeDegrees = slopeDegrees,
            .upDot = upDot,
            .sampleIndex = sampleIndex
        };
    }

    static std::vector<SupportCluster> ClusterSupportCandidates(std::vector<SupportCandidate>& candidates)
    {
        constexpr float heightTolerance = 1.5F;

        std::sort(
            candidates.begin(),
            candidates.end(),
            [](const SupportCandidate& lhs,
            const SupportCandidate& rhs)
            {
                return lhs.heightDelta < rhs.heightDelta;
            });

        std::vector<SupportCluster> clusters;

        for (auto& candidate : candidates) {
            SupportCluster* bestCluster = nullptr;

            for (auto& cluster : clusters) {
                if (std::abs(
                        cluster.meanHeightDelta -
                        candidate.heightDelta) <= heightTolerance)
                {
                    bestCluster = &cluster;
                    break;
                }
            }

            if (!bestCluster) {
                clusters.push_back({});
                bestCluster = &clusters.back();
            }

            bestCluster->candidates.push_back(&candidate);

            const float count =
                static_cast<float>(bestCluster->candidates.size());

            bestCluster->meanHeightDelta +=
                (candidate.heightDelta -
                bestCluster->meanHeightDelta) / count;

            bestCluster->meanSlope +=
                (candidate.slopeDegrees -
                bestCluster->meanSlope) / count;

            bestCluster->meanUpDot +=
                (candidate.upDot -
                bestCluster->meanUpDot) / count;

            bestCluster->sampleCount =
                bestCluster->candidates.size();
        }

        return clusters;
    }

    [[nodiscard]] std::optional<SupportCandidate> SelectSupportCluster(std::vector<SupportCandidate>& candidates, std::size_t sampleCount)
    {
        if (candidates.empty() || sampleCount == 0) {
            return std::nullopt;
        }

        auto clusters = ClusterSupportCandidates(candidates);

        constexpr float minimumCoverage = 0.50F;

        SupportCluster* bestCluster = nullptr;
        float bestScore = -std::numeric_limits<float>::infinity();

        for (auto& cluster : clusters) {
            const float coverage =
                static_cast<float>(cluster.sampleCount) /
                static_cast<float>(sampleCount);

            if (coverage < minimumCoverage) {
                continue;
            }

            /*
            * Lower heightDelta means closer to the navmesh.
            *
            * This is deliberately much more important than tiny
            * differences in slope.
            */
            const float heightScore =
                1.0F /
                (1.0F + std::max(0.0F, cluster.meanHeightDelta));

            const float slopeScore =
                std::clamp(
                    1.0F - cluster.meanSlope / 90.0F,
                    0.0F,
                    1.0F);

            const float coverageScore = coverage;

            const float score =
                1000.0F * coverageScore +
                300.0F * heightScore +
                100.0F * cluster.meanUpDot +
                100.0F * slopeScore;

            if (score > bestScore) {
                bestScore = score;
                bestCluster = &cluster;
            }
        }

        if (!bestCluster) {
            return std::nullopt;
        }

        /*
        * Pick the best individual geometry triangle belonging
        * to the selected surface.
        */
        const auto bestCandidateIt =
            std::min_element(
                bestCluster->candidates.begin(),
                bestCluster->candidates.end(),
                [](const SupportCandidate* lhs,
                const SupportCandidate* rhs)
                {
                    if (lhs->heightDelta != rhs->heightDelta) {
                        return lhs->heightDelta <
                            rhs->heightDelta;
                    }

                    return lhs->slopeDegrees <
                        rhs->slopeDegrees;
                });

        if (bestCandidateIt == bestCluster->candidates.end()) {
            return std::nullopt;
        }

        return **bestCandidateIt;
    }

    std::string ClassifySupport(float heightDelta, float slopeDegrees, float maxSupportDistance, float maxSlope)
    {
        if (slopeDegrees > maxSlope) {
            return "too_steep";
        }
        if (heightDelta > maxSupportDistance) {
            return "floating";
        }
        if (heightDelta < -maxSupportDistance) {
            return "buried";
        }
        return "supported";
    }

    GeometrySummary AnalyzeGeometry(const core::Mesh& mesh)
    {
        GeometrySummary summary{};
        summary.vertexCount = mesh.vertices.size();
        summary.triangleCount = mesh.triangles.size();
        summary.bounds = MakeBounds(mesh.vertices);
        summary.center = summary.bounds.Center();
        summary.extent = summary.bounds.Extent();
        return summary;
    }

    NavMeshSummary AnalyzeNavMesh(const core::NavMesh& mesh)
    {
        NavMeshSummary summary{};
        summary.vertexCount = mesh.vertices.size();
        summary.polygonCount = mesh.polygons.size();
        summary.bounds = MakeBounds(mesh.vertices);
        summary.center = summary.bounds.Center();
        summary.extent = summary.bounds.Extent();
        return summary;
    }

    AnalysisReport AnalyzeNavMeshPolygons(const core::NavMesh& mesh, const core::Mesh& geometry, const AnalysisConfiguration& configuration)
    {
        AnalysisReport report{};
        report.configuration = configuration;
        SpatialIndex spatial;
        spatial.Build(geometry.triangles, geometry.vertices);
        std::vector<double> heightValues;
        std::vector<double> slopeValues;
        report.polygons.reserve(mesh.polygons.size());
        /*
         * Cache geometry bounds once. The original implementation was
         * recalculating these for every navmesh polygon.
         */
        const auto geometryBounds = geometry.Bounds();
        const float geometryMinZ = geometry.vertices.empty() ? 0.0F : geometryBounds.min.z;
        const float geometryMaxZ = geometry.vertices.empty() ? 0.0F : geometryBounds.max.z;
        /*
         * No geometry means that no polygon can have physical support.
         */
        if (geometry.vertices.empty() || geometry.triangles.empty()){
        for (std::uint32_t index = 0; index < mesh.polygons.size(); ++index)
        {
        const auto& polygon = mesh.polygons[index];
        if (polygon.vertices[0] >= mesh.vertices.size() || polygon.vertices[1] >= mesh.vertices.size() || polygon.vertices[2] >= mesh.vertices.size()) { continue; }
        
        const auto a = mesh.vertices[polygon.vertices[0]];
        const auto b = mesh.vertices[polygon.vertices[1]];
        const auto c = mesh.vertices[polygon.vertices[2]];
        const auto centroid = core::Vec3{ (a.x + b.x + c.x) / 3.0F, (a.y + b.y + c.y) / 3.0F, (a.z + b.z + c.z) / 3.0F };
        const auto normal = TriangleNormal(a, b, c);
        const auto bounds = core::AABB{
            .min = { std::min({ a.x, b.x, c.x }),std::min({ a.y, b.y, c.y }), std::min({ a.z, b.z, c.z }) },
            .max = { std::max({ a.x, b.x, c.x }), std::max({ a.y, b.y, c.y }), std::max({ a.z, b.z, c.z }) }
        };
        PolygonAnalysisResult result{ .index = index, .centroid = centroid, .normal = normal, .bounds = bounds, .vertexCount = 3u };
        result.classification = "unsupported";
        ++report.summary.polygonsAnalyzed; ++report.summary.unsupported; report.polygons.push_back(result); } return report;
        }
        /*
         * Vertical ray starts sufficiently above all geometry.
         *
         * The extra margin is intentional. We want the ray to be above
         * roofs/ceilings/etc. as well as the navmesh itself.
         */
        const float rayMargin = std::max({ 4096.0F, configuration.surfaceSearchRadius * 8.0F, configuration.maxSupportDistance * 8.0F });
        const float maxRayDistance = std::max( 16384.0F, geometryMaxZ - geometryMinZ + rayMargin + 2048.0F);
        for (std::uint32_t index = 0; index < mesh.polygons.size(); ++index) {
            const auto& polygon = mesh.polygons[index];
            /*
            * Skyrim navmesh polygons handled by this routine are
            * triangles.
            */
            if (polygon.vertices[0] >= mesh.vertices.size() ||
                polygon.vertices[1] >= mesh.vertices.size() ||
                polygon.vertices[2] >= mesh.vertices.size())
                { continue; }
            const auto a = mesh.vertices[polygon.vertices[0]];
            const auto b = mesh.vertices[polygon.vertices[1]];
            const auto c = mesh.vertices[polygon.vertices[2]];
            const auto centroid = core::Vec3{ (a.x + b.x + c.x) / 3.0F, (a.y + b.y + c.y) / 3.0F, (a.z + b.z + c.z) / 3.0F };
            const auto normal = TriangleNormal(a, b, c);
            const auto bounds = core::AABB{
                .min = { std::min({ a.x, b.x, c.x }), std::min({ a.y, b.y, c.y }), std::min({ a.z, b.z, c.z }) },
                .max = { std::max({ a.x, b.x, c.x }), std::max({ a.y, b.y, c.y }), std::max({ a.z, b.z, c.z }) }
            };
            PolygonAnalysisResult result{ .index = index, .centroid = centroid, .normal = normal, .bounds = bounds, .vertexCount = 3u };
            /*
             * Generate multiple interior points on the navmesh polygon.
             *
             * Using interior points rather than exact vertices avoids
             * problems where a sample lands exactly on the boundary
             * between two geometry triangles.
             */
            const auto samples = MakeSupportSamples(a, b, c);
            /*
             * All physically plausible geometry intersections for all
             * samples.
             */
            std::vector<SupportCandidate> supportCandidates;
            /*
             * A typical polygon only needs a relatively small number
             * of candidates. Reserve enough space to avoid excessive
             * reallocations while still allowing arbitrary geometry.
             */
            supportCandidates.reserve( samples.size() * 16u);
            for (std::size_t sampleIndex = 0; sampleIndex < samples.size(); ++sampleIndex) {
                const auto& sample = samples[sampleIndex];
                /*
                 * Each sample gets its own vertical ray.
                 */
                const float rayOriginHeight = std::max( sample.z, geometryMaxZ) + rayMargin; const auto rayOrigin = core::Vec3{ sample.x, sample.y, rayOriginHeight };
                /*
                 * Query only geometry around this particular sample.
                 *
                 * This is preferable to using one large query around
                 * the entire navmesh polygon because it reduces the
                 * number of irrelevant triangles tested.
                 */
                const auto minQuery = core::Vec3{ sample.x - configuration.surfaceSearchRadius, sample.y - configuration.surfaceSearchRadius, geometryMinZ - 1024.0F };
                const auto maxQuery = core::Vec3{ sample.x + configuration.surfaceSearchRadius, sample.y + configuration.surfaceSearchRadius, rayOriginHeight + 1024.0F };
                auto candidateTriangles = spatial.QueryAABB({ .min = minQuery, .max = maxQuery });
                /*
                 * The spatial index should normally return candidates.
                 * Keep the fallback from the original implementation
                 * so malformed/empty index results don't silently turn
                 * into an unsupported polygon.
                 */
                if (candidateTriangles.empty()) {
                    candidateTriangles.reserve( geometry.triangles.size());
                    for (std::size_t candidateIndex = 0; candidateIndex < geometry.triangles.size(); ++candidateIndex) {
                        candidateTriangles.push_back( candidateIndex);
                    }
                }
                /*
                 * Multiple geometry triangles can overlap the same
                 * vertical ray. We intentionally test ALL of them.
                 *
                 * The old implementation effectively selected one
                 * triangle globally. That is what caused surfaces such
                 * as ceilings, balconies and lower floors to compete
                 * incorrectly with the actual supporting floor.
                 */
                for (const auto candidateIndex : candidateTriangles) {
                    if (candidateIndex >= geometry.triangles.size()){ continue; }
                    const auto& triangle = geometry.triangles[candidateIndex];
                    if (triangle.vertices[0] >= geometry.vertices.size() ||
                        triangle.vertices[1] >= geometry.vertices.size() ||
                        triangle.vertices[2] >= geometry.vertices.size())
                    { continue; }

                    const auto va = geometry.vertices[ triangle.vertices[0]];
                    const auto vb = geometry.vertices[ triangle.vertices[1]];
                    const auto vc = geometry.vertices[ triangle.vertices[2]];
                    const auto hit = IntersectTriangle( rayOrigin, { 0.0F, 0.0F, -1.0F }, va, vb, vc, candidateIndex, maxRayDistance);
                    if (!hit) { continue; }
                    /*
                     * Convert the raw intersection into a physically
                     * plausible support candidate.
                     *
                     * This performs hard filtering for:
                     *
                     * - geometry above the navmesh
                     * - geometry too far below it
                     * - downward-facing surfaces
                     * - excessive slope
                     */ const auto candidate = MakeSupportCandidate( sample, *hit, sampleIndex, configuration.maxSupportDistance, configuration.maxSlope);
                    if (!candidate) { continue; }
                    supportCandidates.push_back(*candidate);
                }
            }
            /*
             * Select a physical surface rather than an individual
             * geometry triangle.
             *
             * SelectSupportCluster() groups hits with similar Z values,
             * measures how much of the navmesh polygon is supported by
             * each surface, and then chooses the strongest surface.
             */ const auto best = SelectSupportCluster( supportCandidates, samples.size());
            if (best) {
                result.support.found = true;
                result.support.point = best->hit.point;
                result.support.distance = best->hit.distance;
                result.support.heightDelta = best->heightDelta;
                result.support.normal = best->hit.normal;
                result.support.slopeDegrees = best->slopeDegrees;
                result.support.triangleIndex = best->hit.triangleIndex;
                result.classification = ClassifySupport(
                    result.support.heightDelta,
                    result.support.slopeDegrees,
                    configuration.maxSupportDistance,
                    configuration.maxSlope
                );
                heightValues.push_back( static_cast<double>( result.support.heightDelta));
                slopeValues.push_back( static_cast<double>( result.support.slopeDegrees));
            } else {
                result.classification = "unsupported";
            }
            ++report.summary.polygonsAnalyzed;
            if (result.support.found) { ++report.summary.supportFound;
            }
            if (result.classification == "unsupported") {
                ++report.summary.unsupported;
            } else if (result.classification == "supported") {
                ++report.summary.supported;
            } else if (result.classification == "floating") {
                ++report.summary.floating;
            } else if (result.classification == "buried") {
                ++report.summary.buried;
            } else if (result.classification == "too_steep") {
                ++report.summary.tooSteep;
            }
            report.polygons.push_back(result);
        }
        /*
         * Aggregate height statistics.
         */
        if (!heightValues.empty()) {
            const auto sortedHeights = SortedValues(heightValues);
            report.heightDeltaStats.min = sortedHeights.front();
            report.heightDeltaStats.max = sortedHeights.back();
            report.heightDeltaStats.mean = std::accumulate( sortedHeights.begin(), sortedHeights.end(), 0.0) / static_cast<double>( sortedHeights.size());
            report.heightDeltaStats.median = sortedHeights[ sortedHeights.size() / 2];
            report.heightDeltaStats.p95 = Percentile( sortedHeights, 0.95);
        }
        /*
         * Aggregate slope statistics.
         */
        if (!slopeValues.empty()) {
            const auto sortedSlopes = SortedValues(slopeValues);
            report.slopeStats.min = sortedSlopes.front();
            report.slopeStats.max = sortedSlopes.back();
            report.slopeStats.mean = std::accumulate( sortedSlopes.begin(), sortedSlopes.end(), 0.0) / static_cast<double>( sortedSlopes.size());
            report.slopeStats.median = sortedSlopes[ sortedSlopes.size() / 2];
            report.slopeStats.p95 = Percentile( sortedSlopes, 0.95);
        }
        return report;
    }

    NavMeshAnalysis Analyze(const core::NavMesh& mesh)
    {
        NavMeshAnalysis analysis{};
        analysis.vertexCount = static_cast<std::uint32_t>(mesh.vertices.size());
        analysis.polygonCount = static_cast<std::uint32_t>(mesh.polygons.size());

        if (!mesh.vertices.empty()) {
            analysis.boundingBox.min = mesh.vertices.front();
            analysis.boundingBox.max = mesh.vertices.front();
            for (const auto& vertex : mesh.vertices) {
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

        for (std::uint32_t polygonIndex = 0; polygonIndex < analysis.polygonCount; ++polygonIndex) {
            const auto& polygon = mesh.polygons[polygonIndex];
            for (std::uint32_t compareIndex = polygonIndex + 1; compareIndex < analysis.polygonCount; ++compareIndex) {
                const auto& other = mesh.polygons[compareIndex];
                const bool sharesVertex = std::any_of(polygon.vertices.begin(), polygon.vertices.end(), [&](std::uint32_t value) {
                    return std::any_of(other.vertices.begin(), other.vertices.end(), [value](std::uint32_t otherValue) {
                        return value == otherValue;
                    });
                });
                if (sharesVertex) {
                    Union(parents, static_cast<int>(polygonIndex), static_cast<int>(compareIndex));
                }
            }
        }

        std::vector<int> componentSizes(analysis.polygonCount > 0 ? static_cast<std::size_t>(analysis.polygonCount) : 0, 0);
        for (std::uint32_t polygonIndex = 0; polygonIndex < analysis.polygonCount; ++polygonIndex) {
            const auto root = FindRoot(parents, static_cast<int>(polygonIndex));
            ++componentSizes[root];
        }

        analysis.connectedComponents = static_cast<std::uint32_t>(std::count_if(componentSizes.begin(), componentSizes.end(), [](int size) { return size > 0; }));
        analysis.isolatedPolygonCount = static_cast<std::uint32_t>(std::count_if(componentSizes.begin(), componentSizes.end(), [](int size) { return size == 1; }));

        double totalArea = 0.0;
        double minArea = std::numeric_limits<double>::max();
        double maxArea = 0.0;
        for (const auto& polygon : mesh.polygons) {
            if (polygon.vertices[0] >= mesh.vertices.size() || polygon.vertices[1] >= mesh.vertices.size() || polygon.vertices[2] >= mesh.vertices.size()) {
                ++analysis.degeneratePolygonCount;
                continue;
            }
            const auto a = mesh.vertices[polygon.vertices[0]];
            const auto b = mesh.vertices[polygon.vertices[1]];
            const auto c = mesh.vertices[polygon.vertices[2]];
            const auto area = TriangleArea(a, b, c);
            const bool degenerate = area <= 1.0e-6 || (a.x == b.x && a.y == b.y && a.z == b.z) || (a.x == c.x && a.y == c.y && a.z == c.z) || (b.x == c.x && b.y == c.y && b.z == c.z);
            if (degenerate) {
                ++analysis.degeneratePolygonCount;
            }
            minArea = std::min(minArea, area);
            maxArea = std::max(maxArea, area);
            totalArea += area;
        }

        if (analysis.polygonCount > 0) {
            analysis.minPolygonArea = minArea == std::numeric_limits<double>::max() ? 0.0 : minArea;
            analysis.maxPolygonArea = maxArea;
            analysis.averagePolygonArea = totalArea / static_cast<double>(analysis.polygonCount);
        }

        return analysis;
    }
}
