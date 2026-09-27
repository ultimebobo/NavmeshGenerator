#include "core/navmesh/candidate.h"

#include "analysis/navmesh_analysis.h"
#include "core/reproducibility/export_metadata.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <fstream>
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
    using Edge = std::pair<Key2,Key2>;

    [[nodiscard]] float Cross2(Vec3 a, Vec3 b, Vec3 c) { return (b.x-a.x)*(c.y-a.y) - (b.y-a.y)*(c.x-a.x); }
    [[nodiscard]] float Area2(Vec3 a, Vec3 b, Vec3 c) { return std::abs(Cross2(a,b,c)); }
    [[nodiscard]] Key Quantize(Vec3 p, float tolerance)
    {
        return { std::llround(static_cast<double>(p.x)/tolerance), std::llround(static_cast<double>(p.y)/tolerance), std::llround(static_cast<double>(p.z)/tolerance) };
    }
    [[nodiscard]] Key2 Quantize2(Vec3 p, float tolerance)
    {
        return { std::llround(static_cast<double>(p.x)/tolerance),std::llround(static_cast<double>(p.y)/tolerance) };
    }
    [[nodiscard]] Edge SortedEdge(Vec3 a, Vec3 b, float tolerance)
    {
        const auto first = Quantize2(a,tolerance), second = Quantize2(b,tolerance);
        return first < second ? Edge{first,second} : Edge{second,first};
    }
    [[nodiscard]] bool StepCompatible(Vec3 a, Vec3 b, Vec3 c, Vec3 d, float tolerance, float step)
    {
        if (Quantize2(a,tolerance) == Quantize2(c,tolerance)) return std::abs(a.z-c.z) <= step+tolerance && std::abs(b.z-d.z) <= step+tolerance;
        return std::abs(a.z-d.z) <= step+tolerance && std::abs(b.z-c.z) <= step+tolerance;
    }
    [[nodiscard]] Vec3 Interpolate(Vec3 a, Vec3 b, float t) { return a + (b-a)*t; }
    [[nodiscard]] bool HeightAt(Vec3 p, Vec3 a, Vec3 b, Vec3 c, float& height)
    {
        const auto denominator = Cross2(a,b,c);
        if (std::abs(denominator) < 1.0e-5F) return false;
        const auto u = Cross2(p,b,c)/denominator;
        const auto v = Cross2(a,p,c)/denominator;
        const auto w = 1.0F-u-v;
        if (u < -1.0e-4F || v < -1.0e-4F || w < -1.0e-4F) return false;
        height = u*a.z+v*b.z+w*c.z;
        return true;
    }
    [[nodiscard]] bool StrictlyInside(Vec3 p, const std::array<Vec3,3>& triangle)
    {
        const auto d = Cross2(triangle[0],triangle[1],triangle[2]);
        if (d <= 1.0e-5F) return false;
        const auto u = Cross2(p,triangle[1],triangle[2])/d;
        const auto v = Cross2(triangle[0],p,triangle[2])/d;
        return u > 1.0e-3F && v > 1.0e-3F && u+v < 1.0F-1.0e-3F;
    }
    [[nodiscard]] bool SegmentThroughInterior(Vec3 a, Vec3 b, const std::array<Vec3,3>& triangle)
    {
        std::vector<float> positions{0.0F,1.0F};
        for (std::size_t side{}; side < 3; ++side) {
            const auto c = triangle[side], d = triangle[(side+1)%3];
            const auto denominator = (b.x-a.x)*(d.y-c.y)-(b.y-a.y)*(d.x-c.x);
            if (std::abs(denominator) < 1.0e-6F) continue;
            const auto t = ((c.x-a.x)*(d.y-c.y)-(c.y-a.y)*(d.x-c.x))/denominator;
            const auto u = ((c.x-a.x)*(b.y-a.y)-(c.y-a.y)*(b.x-a.x))/denominator;
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) positions.push_back(t);
        }
        std::sort(positions.begin(),positions.end());
        for (std::size_t i = 1; i < positions.size(); ++i) if (StrictlyInside(Interpolate(a,b,(positions[i-1]+positions[i])*0.5F),triangle)) return true;
        return false;
    }
    [[nodiscard]] std::vector<Vec3> ClipInsideEdge(const std::vector<Vec3>& polygon, Vec3 a, Vec3 b, float inset)
    {
        std::vector<Vec3> result;
        if (polygon.empty()) return result;
        const auto length = std::hypot(b.x-a.x, b.y-a.y);
        if (length < 1.0e-5F) return result;
        const auto signedDistance = [&](Vec3 p) { return Cross2(a,b,p)/length-inset; };
        for (std::size_t i{}; i < polygon.size(); ++i) {
            const auto previous = polygon[(i+polygon.size()-1)%polygon.size()];
            const auto current = polygon[i];
            const auto before = signedDistance(previous), after = signedDistance(current);
            if ((before >= 0) != (after >= 0)) result.push_back(Interpolate(previous,current,before/(before-after)));
            if (after >= 0) result.push_back(current);
        }
        return result;
    }
    [[nodiscard]] bool OnCellBorder(Vec3 a, Vec3 b, const AABB& bounds, float tolerance)
    {
        return (std::abs(a.x-bounds.min.x) <= tolerance && std::abs(b.x-bounds.min.x) <= tolerance)
            || (std::abs(a.x-bounds.max.x) <= tolerance && std::abs(b.x-bounds.max.x) <= tolerance)
            || (std::abs(a.y-bounds.min.y) <= tolerance && std::abs(b.y-bounds.min.y) <= tolerance)
            || (std::abs(a.y-bounds.max.y) <= tolerance && std::abs(b.y-bounds.max.y) <= tolerance);
    }
    void SimplifyContour(CandidateContour& contour, const NavMesh& mesh, float tolerance)
    {
        if (!contour.closed || contour.vertices.size() <= 3) return;
        bool changed = true;
        while (changed && contour.vertices.size() > 3) {
            changed = false;
            for (std::size_t i{}; i < contour.vertices.size(); ++i) {
                const auto a = mesh.vertices[contour.vertices[(i+contour.vertices.size()-1)%contour.vertices.size()]];
                const auto b = mesh.vertices[contour.vertices[i]];
                const auto c = mesh.vertices[contour.vertices[(i+1)%contour.vertices.size()]];
                const auto length = std::hypot(c.x-a.x,c.y-a.y);
                if (length <= tolerance) continue;
                const auto lineDistance = std::abs(Cross2(a,c,b))/length;
                const auto t = ((b.x-a.x)*(c.x-a.x)+(b.y-a.y)*(c.y-a.y))/(length*length);
                if (t > 0 && t < 1 && lineDistance <= tolerance && std::abs(b.z-(a.z+(c.z-a.z)*t)) <= tolerance) {
                    contour.vertices.erase(contour.vertices.begin()+static_cast<std::ptrdiff_t>(i)); changed = true; break;
                }
            }
        }
    }
    [[nodiscard]] bool HasClearance(const Scene& scene, const navmesh::analysis::SpatialIndex& index,
        std::size_t sourceTriangle, const std::array<Vec3,3>& points, float height, float stepHeight)
    {
        const auto& mesh = scene.mesh;
        const std::array<Vec3,7> samples{ points[0], points[1], points[2],
            (points[0]+points[1])*0.5F, (points[1]+points[2])*0.5F, (points[2]+points[0])*0.5F,
            (points[0]+points[1]+points[2])/3.0F };
        for (const auto& sample : samples) {
            AABB query{ .min = { sample.x-0.01F,sample.y-0.01F,sample.z+0.1F },
                .max = { sample.x+0.01F,sample.y+0.01F,sample.z+height } };
            for (const auto other : index.QueryAABB(query)) {
                if (other == sourceTriangle || other >= mesh.triangles.size()) continue;
                const auto source = scene.triangleProvenance[other].geometrySource;
                if (source >= scene.geometrySources.size() || scene.geometrySources[source].sourceType == GeometrySourceType::RenderFallback) continue;
                const auto& tri = mesh.triangles[other];
                if (tri.vertices[0] >= mesh.vertices.size() || tri.vertices[1] >= mesh.vertices.size() || tri.vertices[2] >= mesh.vertices.size()) continue;
                float surface{};
                if (HeightAt(sample, mesh.vertices[tri.vertices[0]], mesh.vertices[tri.vertices[1]], mesh.vertices[tri.vertices[2]], surface)
                    && surface > sample.z+std::max(0.1F,stepHeight) && surface < sample.z+height) return false;
            }
        }
        return true;
    }
    [[nodiscard]] bool HasWallObstruction(const Scene& scene, const navmesh::analysis::SpatialIndex& index,
        std::size_t sourceTriangle, const std::array<Vec3,3>& floor, const NavigationProfile& profile)
    {
        AABB query; for (const auto point : floor) query.Expand(point);
        const auto center = (floor[0]+floor[1]+floor[2])/3.0F;
        query.min.z = center.z+profile.stepHeight;
        query.max.z = center.z+profile.agentHeight;
        for (const auto other : index.QueryAABB(query)) {
            if (other == sourceTriangle || other >= scene.mesh.triangles.size()) continue;
            const auto source = scene.triangleProvenance[other].geometrySource;
            if (source >= scene.geometrySources.size() || scene.geometrySources[source].sourceType != GeometrySourceType::Collision) continue;
            const auto& tri = scene.mesh.triangles[other];
            if (tri.vertices[0] >= scene.mesh.vertices.size() || tri.vertices[1] >= scene.mesh.vertices.size() || tri.vertices[2] >= scene.mesh.vertices.size()) continue;
            const std::array<Vec3,3> wall{scene.mesh.vertices[tri.vertices[0]],scene.mesh.vertices[tri.vertices[1]],scene.mesh.vertices[tri.vertices[2]]};
            const auto u = wall[1]-wall[0], v = wall[2]-wall[0];
            const auto horizontal = std::abs(u.x*v.y-u.y*v.x);
            const auto vertical = std::hypot(u.y*v.z-u.z*v.y,u.z*v.x-u.x*v.z);
            if (std::atan2(vertical,horizontal)*180.0F/3.14159265358979323846F <= profile.maxSlopeDegrees) continue;
            for (std::size_t side{}; side < 3; ++side)
                if (SegmentThroughInterior(wall[side],wall[(side+1)%3],floor)) return true;
        }
        return false;
    }
    [[nodiscard]] std::vector<std::vector<std::uint32_t>> Components(const NavMesh& mesh)
    {
        std::vector<std::vector<std::uint32_t>> components;
        std::vector<bool> seen(mesh.polygons.size());
        for (std::uint32_t first{}; first < mesh.polygons.size(); ++first) {
            if (seen[first]) continue;
            components.emplace_back(); std::queue<std::uint32_t> pending; pending.push(first); seen[first] = true;
            while (!pending.empty()) {
                const auto current = pending.front(); pending.pop(); components.back().push_back(current);
                for (const auto neighbor : mesh.polygons[current].neighbors) if (neighbor != NoNeighbor && neighbor < seen.size() && !seen[neighbor]) {
                    seen[neighbor] = true; pending.push(neighbor);
                }
            }
        }
        return components;
    }
    void BuildAdjacency(NavMesh& mesh, float tolerance, float step)
    {
        std::map<Edge,std::vector<std::pair<std::uint32_t,std::uint32_t>>> edges;
        for (std::uint32_t i{}; i < mesh.polygons.size(); ++i) {
            auto& polygon = mesh.polygons[i]; polygon.neighbors.fill(NoNeighbor);
            for (std::uint32_t side{}; side < 3; ++side)
                edges[SortedEdge(mesh.vertices[polygon.vertices[side]],mesh.vertices[polygon.vertices[(side+1)%3]],tolerance)].push_back({i,side});
        }
        for (const auto& [_, uses] : edges) for (std::size_t i{}; i < uses.size(); ++i) {
            const auto& left = mesh.polygons[uses[i].first];
            std::optional<std::size_t> match;
            for (std::size_t j{}; j < uses.size(); ++j) if (j != i) {
                const auto& right = mesh.polygons[uses[j].first];
                if (!StepCompatible(mesh.vertices[left.vertices[uses[i].second]],mesh.vertices[left.vertices[(uses[i].second+1)%3]],
                    mesh.vertices[right.vertices[uses[j].second]],mesh.vertices[right.vertices[(uses[j].second+1)%3]],tolerance,step)) continue;
                if (match) { match = std::nullopt; break; }
                match = j;
            }
            if (match) mesh.polygons[uses[i].first].neighbors[uses[i].second] = uses[*match].first;
        }
    }
}

namespace navmesh::core
{
    std::optional<NavigationProfile> FindNavigationProfile(const std::string& key)
    {
        if (key == "human@1.0.0") return NavigationProfile{};
        if (key == "small@1.0.0") return NavigationProfile{ .name = "small", .version = "1.0.0", .agentRadius = 8.0F,
            .agentHeight = 64.0F, .maxSlopeDegrees = 50.0F, .stepHeight = 12.0F, .clearance = 64.0F,
            .weldTolerance = 0.05F, .minimumRegionArea = 32.0F, .contourSimplificationTolerance = 0.05F };
        return std::nullopt;
    }

    CandidateNavMesh GenerateCandidate(const Scene& scene, const NavigationProfile& profile, std::optional<AABB> cellBounds)
    {
        CandidateNavMesh result; result.profile = profile;
        const auto& input = scene.mesh;
        result.statistics.inputTriangles = input.triangles.size();
        if (profile.agentRadius < 0 || profile.agentHeight <= 0 || profile.clearance < profile.agentHeight
            || profile.maxSlopeDegrees < 0 || profile.maxSlopeDegrees >= 90 || profile.stepHeight < 0
            || profile.weldTolerance <= 0 || profile.minimumRegionArea < 0 || profile.contourSimplificationTolerance < 0
            || (profile.cellBorderPolicy != "preserve_open_border" && profile.cellBorderPolicy != "inset_all_edges"))
            throw std::invalid_argument("Invalid navigation profile parameters");
        if (scene.triangleProvenance.size() != input.triangles.size())
            throw std::invalid_argument("Candidate generation requires complete triangle provenance");
        navmesh::analysis::SpatialIndex spatial; spatial.Build(input.triangles,input.vertices);
        struct Accepted { std::array<Vec3,3> points; std::size_t source{}; };
        std::vector<Accepted> accepted;
        for (std::size_t i{}; i < input.triangles.size(); ++i) {
            const auto& provenance = scene.triangleProvenance[i];
            if (provenance.geometrySource >= scene.geometrySources.size()
                || scene.geometrySources[provenance.geometrySource].sourceType == GeometrySourceType::RenderFallback) {
                ++result.statistics.rejectedSource; continue;
            }
            const auto& tri = input.triangles[i];
            if (tri.vertices[0] >= input.vertices.size() || tri.vertices[1] >= input.vertices.size() || tri.vertices[2] >= input.vertices.size()) {
                ++result.statistics.rejectedDegenerate; continue;
            }
            std::array<Vec3,3> points{ input.vertices[tri.vertices[0]],input.vertices[tri.vertices[1]],input.vertices[tri.vertices[2]] };
            if (!std::isfinite(points[0].x) || !std::isfinite(points[0].y) || !std::isfinite(points[0].z)
                || !std::isfinite(points[1].x) || !std::isfinite(points[1].y) || !std::isfinite(points[1].z)
                || !std::isfinite(points[2].x) || !std::isfinite(points[2].y) || !std::isfinite(points[2].z)
                || Area2(points[0],points[1],points[2]) <= profile.weldTolerance*profile.weldTolerance) {
                ++result.statistics.rejectedDegenerate; continue;
            }
            if (Cross2(points[0],points[1],points[2]) < 0) std::swap(points[1],points[2]);
            const auto u = points[1]-points[0], v = points[2]-points[0];
            const auto horizontal = std::abs(u.x*v.y-u.y*v.x);
            const auto vertical = std::hypot(u.y*v.z-u.z*v.y,u.z*v.x-u.x*v.z);
            const auto slope = std::atan2(vertical,horizontal)*180.0F/3.14159265358979323846F;
            if (slope > profile.maxSlopeDegrees) { ++result.statistics.rejectedSlope; continue; }
            if (cellBounds) {
                const auto center = (points[0]+points[1]+points[2])/3.0F;
                if (center.x < cellBounds->min.x || center.x >= cellBounds->max.x
                    || center.y < cellBounds->min.y || center.y >= cellBounds->max.y) continue;
            }
            if (!HasClearance(scene,spatial,i,points,profile.clearance,profile.stepHeight)) { ++result.statistics.rejectedClearance; continue; }
            if (HasWallObstruction(scene,spatial,i,points,profile)) { ++result.statistics.rejectedObstruction; continue; }
            accepted.push_back({ .points = points, .source = i });
        }
        result.statistics.eligibleTriangles = accepted.size();
        std::map<Edge,std::vector<std::pair<std::size_t,std::size_t>>> sourceEdges;
        for (std::size_t i{}; i < accepted.size(); ++i) for (std::size_t side{}; side < 3; ++side)
            sourceEdges[SortedEdge(accepted[i].points[side],accepted[i].points[(side+1)%3],profile.weldTolerance)].push_back({i,side});
        std::map<Key,std::uint32_t> outputKeys;
        for (std::size_t i{}; i < accepted.size(); ++i) {
            auto polygon = std::vector<Vec3>{ accepted[i].points.begin(),accepted[i].points.end() };
            for (std::size_t side{}; side < 3 && !polygon.empty(); ++side) {
                const auto a = accepted[i].points[side], b = accepted[i].points[(side+1)%3];
                const auto& uses = sourceEdges[SortedEdge(a,b,profile.weldTolerance)];
                std::size_t compatible{};
                for (const auto& [otherIndex,otherSide] : uses) if (otherIndex != i) {
                    const auto& other = accepted[otherIndex];
                    if (StepCompatible(a,b,other.points[otherSide],other.points[(otherSide+1)%3],profile.weldTolerance,profile.stepHeight)) ++compatible;
                }
                const bool connected = compatible == 1;
                const bool border = cellBounds && profile.cellBorderPolicy == "preserve_open_border"
                    && OnCellBorder(a,b,*cellBounds,profile.weldTolerance);
                if (!connected && !border) polygon = ClipInsideEdge(polygon,a,b,profile.agentRadius);
            }
            if (polygon.size() < 3) continue;
            const auto makeVertex = [&](Vec3 point) {
                const auto key = Quantize(point,profile.weldTolerance);
                auto [where,created] = outputKeys.try_emplace(key,static_cast<std::uint32_t>(result.mesh.vertices.size()));
                if (created) result.mesh.vertices.push_back(point);
                return where->second;
            };
            const auto first = makeVertex(polygon.front());
            for (std::size_t corner = 1; corner+1 < polygon.size(); ++corner) {
                if (Area2(polygon.front(),polygon[corner],polygon[corner+1]) <= profile.weldTolerance*profile.weldTolerance) continue;
                NavPolygon nav; nav.vertices = { first,makeVertex(polygon[corner]),makeVertex(polygon[corner+1]) }; nav.neighbors.fill(NoNeighbor);
                if (nav.vertices[0] == nav.vertices[1] || nav.vertices[1] == nav.vertices[2] || nav.vertices[2] == nav.vertices[0]) continue;
                result.mesh.polygons.push_back(nav); result.polygonSourceTriangles.push_back(accepted[i].source);
            }
        }
        BuildAdjacency(result.mesh,profile.weldTolerance,profile.stepHeight);
        const auto components = Components(result.mesh);
        std::vector<bool> keep(result.mesh.polygons.size(),true);
        for (const auto& component : components) {
            float area{};
            for (const auto index : component) { const auto& tri = result.mesh.polygons[index]; area += Area2(result.mesh.vertices[tri.vertices[0]],result.mesh.vertices[tri.vertices[1]],result.mesh.vertices[tri.vertices[2]])*0.5F; }
            if (area < profile.minimumRegionArea) { for (const auto index : component) keep[index] = false; result.statistics.rejectedSmallRegion += component.size(); }
        }
        if (result.statistics.rejectedSmallRegion) {
            NavMesh filtered; std::vector<std::size_t> sources;
            for (std::size_t i{}; i < keep.size(); ++i) if (keep[i]) { filtered.polygons.push_back(result.mesh.polygons[i]); sources.push_back(result.polygonSourceTriangles[i]); }
            std::map<std::uint32_t,std::uint32_t> remap;
            for (auto& polygon : filtered.polygons) for (auto& vertex : polygon.vertices) {
                auto [where,created] = remap.try_emplace(vertex,static_cast<std::uint32_t>(filtered.vertices.size()));
                if (created) filtered.vertices.push_back(result.mesh.vertices[vertex]); vertex = where->second;
            }
            result.mesh = std::move(filtered); result.polygonSourceTriangles = std::move(sources);
            BuildAdjacency(result.mesh,profile.weldTolerance,profile.stepHeight);
        }
        for (const auto& component : Components(result.mesh)) {
            CandidateRegion region; region.id = static_cast<std::uint32_t>(result.regions.size()); region.polygons = component;
            std::set<std::size_t> sourceTriangles, geometrySources;
            for (const auto index : component) {
                const auto& tri = result.mesh.polygons[index];
                region.area += Area2(result.mesh.vertices[tri.vertices[0]],result.mesh.vertices[tri.vertices[1]],result.mesh.vertices[tri.vertices[2]])*0.5F;
                const auto source = result.polygonSourceTriangles[index]; sourceTriangles.insert(source);
                geometrySources.insert(scene.triangleProvenance[source].geometrySource);
            }
            region.sourceTriangles.assign(sourceTriangles.begin(),sourceTriangles.end());
            region.geometrySources.assign(geometrySources.begin(),geometrySources.end());
            result.regions.push_back(std::move(region));
        }
        for (const auto& region : result.regions) {
            std::multimap<Key2,std::pair<std::uint32_t,Key2>> boundary;
            for (const auto index : region.polygons) {
                const auto& tri = result.mesh.polygons[index];
                for (std::size_t side{}; side < 3; ++side) if (tri.neighbors[side] == NoNeighbor)
                    boundary.emplace(Quantize2(result.mesh.vertices[tri.vertices[side]],profile.weldTolerance),
                        std::pair{tri.vertices[side],Quantize2(result.mesh.vertices[tri.vertices[(side+1)%3]],profile.weldTolerance)});
            }
            while (!boundary.empty()) {
                CandidateContour contour; contour.region = region.id;
                const auto first = boundary.begin()->first; auto current = first;
                for (std::size_t count{}; count <= result.mesh.polygons.size()*3; ++count) {
                    const auto edge = boundary.find(current); if (edge == boundary.end()) break;
                    contour.vertices.push_back(edge->second.first); current = edge->second.second; boundary.erase(edge);
                    if (current == first) { contour.closed = true; break; }
                }
                SimplifyContour(contour,result.mesh,profile.contourSimplificationTolerance);
                result.contours.push_back(std::move(contour));
            }
        }
        result.statistics.outputPolygons = result.mesh.polygons.size();
        if (result.mesh.polygons.empty()) result.warnings.push_back("No candidate polygons survived source, slope, clearance, radius, and region filters.");
        result.topology = ValidateCandidateTopology(result);
        return result;
    }

    CandidateTopology ValidateCandidateTopology(const CandidateNavMesh& candidate)
    {
        CandidateTopology result;
        const auto& mesh = candidate.mesh;
        if (candidate.polygonSourceTriangles.size() != mesh.polygons.size()) result.findings.push_back("polygon provenance count mismatch");
        std::map<Edge,std::vector<std::pair<std::size_t,std::size_t>>> edges;
        std::set<std::array<std::uint32_t,3>> faces;
        std::vector<bool> used(mesh.vertices.size());
        for (std::size_t i{}; i < mesh.polygons.size(); ++i) {
            const auto& tri = mesh.polygons[i];
            for (const auto vertex : tri.vertices) if (vertex >= mesh.vertices.size()) result.findings.push_back(std::format("polygon {} has invalid vertex",i)); else used[vertex] = true;
            if (std::any_of(tri.vertices.begin(),tri.vertices.end(),[&](auto vertex){return vertex >= mesh.vertices.size();})) continue;
            if (Cross2(mesh.vertices[tri.vertices[0]],mesh.vertices[tri.vertices[1]],mesh.vertices[tri.vertices[2]]) <= 1.0e-5F)
                result.findings.push_back(std::format("polygon {} is degenerate or clockwise",i));
            auto face = tri.vertices; std::sort(face.begin(),face.end());
            if (!faces.insert(face).second) result.findings.push_back(std::format("duplicate polygon {}",i));
            for (std::size_t side{}; side < 3; ++side) edges[SortedEdge(mesh.vertices[tri.vertices[side]],mesh.vertices[tri.vertices[(side+1)%3]],candidate.profile.weldTolerance)].push_back({i,side});
        }
        for (const auto& [edge,uses] : edges) {
            for (const auto& [index,side] : uses) {
                auto expected = NoNeighbor;
                std::size_t matches{};
                const auto& left = mesh.polygons[index];
                for (const auto& [otherIndex,otherSide] : uses) if (otherIndex != index) {
                    const auto& right = mesh.polygons[otherIndex];
                    if (StepCompatible(mesh.vertices[left.vertices[side]],mesh.vertices[left.vertices[(side+1)%3]],
                        mesh.vertices[right.vertices[otherSide]],mesh.vertices[right.vertices[(otherSide+1)%3]],
                        candidate.profile.weldTolerance,candidate.profile.stepHeight)) {
                        ++matches; expected = static_cast<std::uint32_t>(otherIndex);
                    }
                }
                if (matches > 1) { result.findings.push_back("non-manifold compatible edge"); expected = NoNeighbor; }
                if (matches == 0) expected = NoNeighbor;
                if (mesh.polygons[index].neighbors[side] != expected) result.findings.push_back(std::format("invalid adjacency at polygon {} edge {}",index,side));
            }
        }
        for (std::size_t i{}; i < used.size(); ++i) if (!used[i]) result.findings.push_back(std::format("orphan vertex {}",i));
        std::vector<std::uint32_t> membership(mesh.polygons.size());
        for (const auto& region : candidate.regions) for (const auto polygon : region.polygons) {
            if (polygon >= membership.size()) result.findings.push_back("region references invalid polygon");
            else ++membership[polygon];
        }
        for (std::size_t i{}; i < membership.size(); ++i) if (membership[i] != 1)
            result.findings.push_back(std::format("polygon {} has {} region memberships",i,membership[i]));
        for (std::size_t i{}; i < candidate.contours.size(); ++i) if (!candidate.contours[i].closed)
            result.findings.push_back(std::format("open contour {}",i));
        result.valid = result.findings.empty(); return result;
    }

    bool WriteCandidateJson(const std::filesystem::path& path, const CandidateNavMesh& candidate, const Scene& scene, const std::string& metadataJson)
    {
        std::ofstream out(path,std::ios::binary|std::ios::trunc); if (!out) return false;
        const auto& p = candidate.profile;
        out << "{\n  \"schema\": \"navmesh-generator/candidate-navm\",\n  \"schema_version\": \"1.0.0\",\n  \"metadata\": " << metadataJson << ",\n";
        out << std::format("  \"profile\": {{\"name\":\"{}\",\"version\":\"{}\",\"agent_radius\":{},\"agent_height\":{},\"max_slope_degrees\":{},\"step_height\":{},\"clearance\":{},\"weld_tolerance\":{},\"minimum_region_area\":{},\"contour_simplification_tolerance\":{},\"cell_border_policy\":\"{}\"}},\n",
            p.name,p.version,p.agentRadius,p.agentHeight,p.maxSlopeDegrees,p.stepHeight,p.clearance,p.weldTolerance,p.minimumRegionArea,p.contourSimplificationTolerance,p.cellBorderPolicy);
        const auto& s = candidate.statistics;
        out << std::format("  \"statistics\": {{\"input_triangles\":{},\"eligible_triangles\":{},\"rejected_source\":{},\"rejected_slope\":{},\"rejected_clearance\":{},\"rejected_obstruction\":{},\"rejected_degenerate\":{},\"rejected_small_region\":{},\"output_polygons\":{}}},\n",
            s.inputTriangles,s.eligibleTriangles,s.rejectedSource,s.rejectedSlope,s.rejectedClearance,s.rejectedObstruction,s.rejectedDegenerate,s.rejectedSmallRegion,s.outputPolygons);
        out << "  \"vertices\": [";
        for (std::size_t i{}; i < candidate.mesh.vertices.size(); ++i) { const auto& v = candidate.mesh.vertices[i]; out << (i ? "," : "") << std::format("[{},{},{}]",v.x,v.y,v.z); }
        out << "],\n  \"polygons\": [";
        for (std::size_t i{}; i < candidate.mesh.polygons.size(); ++i) {
            const auto& tri = candidate.mesh.polygons[i]; const auto sourceIndex = candidate.polygonSourceTriangles[i];
            const auto sourceId = scene.triangleProvenance[sourceIndex].geometrySource;
            out << (i ? "," : "") << std::format("{{\"vertices\":[{},{},{}],\"neighbors\":[",tri.vertices[0],tri.vertices[1],tri.vertices[2]);
            for (std::size_t side{}; side < 3; ++side) out << (side ? "," : "") << (tri.neighbors[side] == NoNeighbor ? "null" : std::to_string(tri.neighbors[side]));
            out << std::format("],\"source_triangle\":{},\"geometry_source\":{}}}",sourceIndex,sourceId);
        }
        out << "],\n  \"regions\": [";
        for (std::size_t i{}; i < candidate.regions.size(); ++i) {
            const auto& region = candidate.regions[i]; out << (i ? "," : "") << std::format("{{\"id\":{},\"area\":{},\"polygons\":[",region.id,region.area);
            for (std::size_t j{}; j < region.polygons.size(); ++j) out << (j ? "," : "") << region.polygons[j];
            out << "],\"source_triangles\":[";
            for (std::size_t j{}; j < region.sourceTriangles.size(); ++j) out << (j ? "," : "") << region.sourceTriangles[j];
            out << "],\"geometry_sources\":[";
            for (std::size_t j{}; j < region.geometrySources.size(); ++j) out << (j ? "," : "") << region.geometrySources[j];
            out << "]}";
        }
        out << "],\n  \"contours\": [";
        for (std::size_t i{}; i < candidate.contours.size(); ++i) {
            const auto& contour = candidate.contours[i]; out << (i ? "," : "") << std::format("{{\"region\":{},\"closed\":{},\"vertices\":[",contour.region,contour.closed ? "true" : "false");
            for (std::size_t j{}; j < contour.vertices.size(); ++j) out << (j ? "," : "") << contour.vertices[j]; out << "]}";
        }
        out << "],\n  \"geometry_sources\": [";
        for (std::size_t i{}; i < scene.geometrySources.size(); ++i) {
            const auto& source = scene.geometrySources[i]; out << (i ? "," : "")
                << std::format("{{\"plugin\":\"{}\",\"form_id\":\"{:08X}\",\"record_type\":\"{}\",\"model\":\"{}\",\"type\":\"{}\",\"confidence\":{}}}",
                    reproducibility::EscapeJson(source.reference.plugin),source.reference.formId,reproducibility::EscapeJson(source.reference.recordType),
                    reproducibility::EscapeJson(source.modelPath),source.sourceType == GeometrySourceType::Terrain ? "terrain" : source.sourceType == GeometrySourceType::Collision ? "collision" : "render_fallback",source.confidence);
        }
        out << "],\n  \"source_triangles\": [";
        std::set<std::size_t> usedSources(candidate.polygonSourceTriangles.begin(),candidate.polygonSourceTriangles.end());
        std::size_t emitted{};
        for (const auto sourceIndex : usedSources) {
            const auto& provenance = scene.triangleProvenance[sourceIndex];
            out << (emitted++ ? "," : "") << std::format("{{\"input_index\":{},\"geometry_source\":{},\"source_triangle\":{}",sourceIndex,provenance.geometrySource,provenance.sourceTriangle);
            if (provenance.terrain) {
                const auto& terrain = *provenance.terrain;
                out << std::format(",\"terrain\":{{\"cell\":[{},{}],\"land_form_id\":\"{:08X}\",\"sample\":[{},{}]}}",
                    terrain.cellX,terrain.cellY,terrain.landFormId,terrain.sampleX,terrain.sampleY);
            }
            out << '}';
        }
        out << "],\n  \"topology\": {\"valid\":" << (candidate.topology.valid ? "true" : "false") << ",\"findings\":[";
        for (std::size_t i{}; i < candidate.topology.findings.size(); ++i) out << (i ? "," : "") << "\"" << reproducibility::EscapeJson(candidate.topology.findings[i]) << "\"";
        out << "]},\n  \"warnings\": [";
        for (std::size_t i{}; i < candidate.warnings.size(); ++i) out << (i ? "," : "") << "\"" << reproducibility::EscapeJson(candidate.warnings[i]) << "\"";
        out << "]\n}\n"; return out.good();
    }

    bool WriteCandidateObj(const std::filesystem::path& path, const CandidateNavMesh& candidate)
    {
        std::ofstream out(path,std::ios::trunc); if (!out) return false;
        out << "# neutral candidate NAVM; profile " << candidate.profile.name << '@' << candidate.profile.version << "\n";
        for (const auto& vertex : candidate.mesh.vertices) out << std::format("v {} {} {}\n",vertex.x,vertex.y,vertex.z);
        for (const auto& region : candidate.regions) {
            out << "g candidate_region_" << region.id << "\n";
            for (const auto index : region.polygons) { const auto& tri = candidate.mesh.polygons[index]; out << std::format("f {} {} {}\n",tri.vertices[0]+1,tri.vertices[1]+1,tri.vertices[2]+1); }
        }
        return out.good();
    }
}
