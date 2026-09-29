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
#include <vector>

namespace
{
    using namespace navmesh::core;
    constexpr auto NoNeighbor = std::numeric_limits<std::uint32_t>::max();

    template<class T, void (*Free)(T*)>
    using RecastOwner = std::unique_ptr<T, decltype(Free)>;

    [[nodiscard]] float Cross(Vec3 a, Vec3 b, Vec3 c)
    {
        return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
    }

    [[nodiscard]] bool HeightAt(Vec3 point, Vec3 a, Vec3 b, Vec3 c, float& height)
    {
        const float area = Cross(a,b,c);
        if (std::abs(area) < 0.0001F) return false;
        const float u = Cross(point,b,c)/area, v = Cross(a,point,c)/area;
        if (u < -0.01F || v < -0.01F || u+v > 1.01F) return false;
        height = u*a.z+v*b.z+(1-u-v)*c.z;
        return true;
    }

    using SourceGrid = std::map<std::pair<int,int>,std::vector<std::size_t>>;
    constexpr float SourceCellSize = 128.0F;
    [[nodiscard]] int SourceCell(float value) { return static_cast<int>(std::floor(value/SourceCellSize)); }

    [[nodiscard]] std::size_t FindSource(const Scene& scene, const SourceGrid& grid,
        const std::vector<std::size_t>& largeSources, std::size_t fallback, Vec3 point)
    {
        // Recast voxels have no source-triangle identifier. Join each output
        // triangle back to the closest source surface at its XY centroid.
        float best = std::numeric_limits<float>::max();
        std::size_t selected = fallback;
        const auto candidates = grid.find({SourceCell(point.x),SourceCell(point.y)});
        const auto inspect = [&](std::size_t index) {
            const auto& tri = scene.mesh.triangles[index];
            const auto a = scene.mesh.vertices[tri.vertices[0]], b = scene.mesh.vertices[tri.vertices[1]], c = scene.mesh.vertices[tri.vertices[2]];
            float height{};
            if (!HeightAt(point,a,b,c,height)) return;
            const float distance = std::abs(height-point.z);
            if (distance < best) { best = distance; selected = index; }
        };
        if (candidates != grid.end()) for (const auto index : candidates->second) inspect(index);
        for (const auto index : largeSources) inspect(index);
        return selected;
    }
}

namespace navmesh::core
{
    CandidateNavMesh RecastCandidateGenerator::Generate(const Scene& scene, const NavigationProfile& profile,
        std::optional<AABB> cellBounds, std::vector<CandidateExit> exits) const
    {
        if (!scene.HasCompleteTriangleProvenance()) throw std::invalid_argument("Recast requires complete triangle provenance");
        if (profile.agentRadius < 0 || profile.agentHeight <= 0 || profile.stepHeight < 0
            || profile.maxSlopeDegrees < 0 || profile.maxSlopeDegrees >= 90)
            throw std::invalid_argument("Invalid navigation profile for Recast");
        CandidateNavMesh result;
        result.profile = profile;
        result.exits = std::move(exits);
        result.statistics.inputTriangles = scene.mesh.triangles.size();

        std::vector<float> vertices;
        std::vector<int> triangles;
        std::vector<std::size_t> sources;
        AABB bounds;
        for (std::size_t i{}; i < scene.mesh.triangles.size(); ++i) {
            const auto& provenance = scene.triangleProvenance[i];
            if (provenance.geometrySource >= scene.geometrySources.size()) throw std::invalid_argument("Invalid triangle geometry source");
            if (scene.geometrySources[provenance.geometrySource].sourceType == GeometrySourceType::RenderFallback) {
                ++result.statistics.rejectedSource; continue;
            }
            const auto& tri = scene.mesh.triangles[i];
            if (std::any_of(tri.vertices.begin(),tri.vertices.end(),[&](auto v){ return v >= scene.mesh.vertices.size(); }))
                throw std::invalid_argument("Invalid source triangle vertex");
            std::array<Vec3,3> points{scene.mesh.vertices[tri.vertices[0]],scene.mesh.vertices[tri.vertices[1]],scene.mesh.vertices[tri.vertices[2]]};
            const float area = Cross(points[0],points[1],points[2]);
            if (std::abs(area) < 0.0001F) { ++result.statistics.rejectedDegenerate; continue; }
            if (area < 0) std::swap(points[1],points[2]);
            const auto base = static_cast<int>(vertices.size()/3);
            // Recast is Y-up. Swapping Skyrim Y and Z changes handedness,
            // so the input winding is reversed to keep walkable normals up.
            for (const auto point : points) {
                vertices.insert(vertices.end(), {point.x,point.z,point.y});
                bounds.Expand(point);
            }
            triangles.insert(triangles.end(), {base,base+2,base+1});
            sources.push_back(i);
        }
        result.statistics.eligibleTriangles = sources.size();
        if (sources.empty()) {
            result.warnings.push_back("No supported terrain or collision triangles were available for Recast.");
            return result;
        }
        SourceGrid sourceGrid;
        std::vector<std::size_t> largeSources;
        for (const auto index : sources) {
            const auto& triangle = scene.mesh.triangles[index];
            const auto a=scene.mesh.vertices[triangle.vertices[0]], b=scene.mesh.vertices[triangle.vertices[1]], c=scene.mesh.vertices[triangle.vertices[2]];
            const int minX=SourceCell(std::min({a.x,b.x,c.x})), maxX=SourceCell(std::max({a.x,b.x,c.x}));
            const int minY=SourceCell(std::min({a.y,b.y,c.y})), maxY=SourceCell(std::max({a.y,b.y,c.y}));
            if (static_cast<std::int64_t>(maxX-minX+1)*(maxY-minY+1)>1024) { largeSources.push_back(index); continue; }
            for (int x=minX; x<=maxX; ++x) for (int y=minY; y<=maxY; ++y) sourceGrid[{x,y}].push_back(index);
        }

        // Resolve narrow stair treads in one cell and the default neighboring
        // cell ring while limiting wider areas to roughly 2048 columns per axis.
        const float width = bounds.max.x-bounds.min.x, depth = bounds.max.y-bounds.min.y;
        const float cs = std::max({4.0F,width/2048.0F,depth/2048.0F});
        const float ch = 2.0F;
        rcConfig config{};
        config.cs = cs; config.ch = ch;
        config.walkableSlopeAngle = profile.maxSlopeDegrees;
        config.walkableHeight = std::max(3,static_cast<int>(std::ceil(std::max(profile.agentHeight,profile.clearance)/ch)));
        config.walkableClimb = static_cast<int>(std::floor(profile.stepHeight/ch));
        config.walkableRadius = static_cast<int>(std::ceil(profile.agentRadius/cs));
        config.maxEdgeLen = 12;
        config.maxSimplificationError = 1.3F;
        config.minRegionArea = static_cast<int>(std::ceil(profile.minimumRegionArea/(cs*cs)));
        config.mergeRegionArea = 0;
        config.maxVertsPerPoly = 6;
        config.bmin[0] = bounds.min.x-cs*2; config.bmin[1] = bounds.min.z-ch*2; config.bmin[2] = bounds.min.y-cs*2;
        config.bmax[0] = bounds.max.x+cs*2; config.bmax[1] = bounds.max.z+profile.agentHeight+ch*2; config.bmax[2] = bounds.max.y+cs*2;
        rcCalcGridSize(config.bmin,config.bmax,config.cs,&config.width,&config.height);
        rcContext context(false);
        RecastOwner<rcHeightfield,rcFreeHeightField> heightfield(rcAllocHeightfield(),rcFreeHeightField);
        if (!heightfield || !rcCreateHeightfield(&context,*heightfield,config.width,config.height,
                config.bmin,config.bmax,config.cs,config.ch)) throw std::runtime_error("Recast heightfield creation failed");
        std::vector<unsigned char> areas(sources.size(),RC_NULL_AREA);
        rcMarkWalkableTriangles(&context,config.walkableSlopeAngle,vertices.data(),static_cast<int>(vertices.size()/3),
            triangles.data(),static_cast<int>(sources.size()),areas.data());
        for (const auto area : areas) if (area == RC_NULL_AREA) ++result.statistics.rejectedSlope;
        if (!rcRasterizeTriangles(&context,vertices.data(),static_cast<int>(vertices.size()/3),triangles.data(),areas.data(),
                static_cast<int>(sources.size()),*heightfield,config.walkableClimb)) throw std::runtime_error("Recast rasterization failed");
        rcFilterLowHangingWalkableObstacles(&context,config.walkableClimb,*heightfield);
        rcFilterLedgeSpans(&context,config.walkableHeight,config.walkableClimb,*heightfield);
        rcFilterWalkableLowHeightSpans(&context,config.walkableHeight,*heightfield);
        RecastOwner<rcCompactHeightfield,rcFreeCompactHeightfield> compact(rcAllocCompactHeightfield(),rcFreeCompactHeightfield);
        if (!compact || !rcBuildCompactHeightfield(&context,config.walkableHeight,config.walkableClimb,*heightfield,*compact))
            throw std::runtime_error("Recast compact heightfield failed");
        if (!rcErodeWalkableArea(&context,config.walkableRadius,*compact)) throw std::runtime_error("Recast radius erosion failed");
        if (!rcBuildRegionsMonotone(&context,*compact,0,config.minRegionArea,config.mergeRegionArea))
            throw std::runtime_error("Recast region partition failed");
        RecastOwner<rcContourSet,rcFreeContourSet> contours(rcAllocContourSet(),rcFreeContourSet);
        if (!contours || !rcBuildContours(&context,*compact,config.maxSimplificationError,config.maxEdgeLen,*contours))
            throw std::runtime_error("Recast contour construction failed");
        RecastOwner<rcPolyMesh,rcFreePolyMesh> polyMesh(rcAllocPolyMesh(),rcFreePolyMesh);
        if (!polyMesh || !rcBuildPolyMesh(&context,*contours,config.maxVertsPerPoly,*polyMesh))
            throw std::runtime_error("Recast polygon construction failed");

        for (int i=0; i<polyMesh->nverts; ++i) {
            const auto* v = polyMesh->verts+3*i;
            result.mesh.vertices.push_back({polyMesh->bmin[0]+v[0]*polyMesh->cs,
                polyMesh->bmin[2]+v[2]*polyMesh->cs,polyMesh->bmin[1]+v[1]*polyMesh->ch});
        }
        for (int i=0; i<polyMesh->npolys; ++i) {
            const auto* poly = polyMesh->polys+i*polyMesh->nvp*2;
            for (int j=2; j<polyMesh->nvp && poly[j] != RC_MESH_NULL_IDX; ++j) {
                NavPolygon face;
                face.vertices = {poly[0],poly[j-1],poly[j]};
                if (Cross(result.mesh.vertices[face.vertices[0]],result.mesh.vertices[face.vertices[1]],
                        result.mesh.vertices[face.vertices[2]]) < 0) std::swap(face.vertices[1],face.vertices[2]);
                face.neighbors.fill(NoNeighbor);
                result.mesh.polygons.push_back(face);
            }
        }
        // Drop vertices that fan triangulation did not use, preserving the
        // neutral model's no-orphan invariant.
        std::vector<std::uint32_t> remap(result.mesh.vertices.size(),NoNeighbor);
        std::vector<Vec3> used;
        for (auto& face : result.mesh.polygons) for (auto& index : face.vertices) {
            if (remap[index] == NoNeighbor) { remap[index] = static_cast<std::uint32_t>(used.size()); used.push_back(result.mesh.vertices[index]); }
            index = remap[index];
        }
        result.mesh.vertices = std::move(used);

        using Edge = std::pair<std::uint32_t,std::uint32_t>;
        std::map<Edge,std::vector<std::pair<std::size_t,std::size_t>>> edges;
        for (std::size_t i{}; i<result.mesh.polygons.size(); ++i)
            for (std::size_t side{}; side<3; ++side) {
                const auto& face = result.mesh.polygons[i];
                edges[std::minmax(face.vertices[side],face.vertices[(side+1)%3])].push_back({i,side});
            }
        for (const auto& [edge,uses] : edges) if (uses.size() == 2) {
            const auto [a,as] = uses[0];
            const auto [b,bs] = uses[1];
            result.mesh.polygons[a].neighbors[as] = static_cast<std::uint32_t>(b);
            result.mesh.polygons[b].neighbors[bs] = static_cast<std::uint32_t>(a);
        }

        // Recast output retains mesh geometry but not original triangle IDs.
        // The nearest supporting triangle supplies the audit join.
        for (const auto& face : result.mesh.polygons) {
            const auto point = (result.mesh.vertices[face.vertices[0]]+result.mesh.vertices[face.vertices[1]]
                +result.mesh.vertices[face.vertices[2]])/3.0F;
            const auto source = FindSource(scene,sourceGrid,largeSources,sources.front(),point);
            result.polygonSourceTriangles.push_back(source);
            result.polygonContributingTriangles.push_back({source});
        }
        std::vector<bool> seen(result.mesh.polygons.size());
        for (std::size_t start{}; start<seen.size(); ++start) if (!seen[start]) {
            CandidateRegion region;
            region.id = static_cast<std::uint32_t>(result.regions.size());
            std::queue<std::size_t> pending;
            pending.push(start); seen[start] = true;
            while (!pending.empty()) {
                const auto index = pending.front(); pending.pop();
                region.polygons.push_back(static_cast<std::uint32_t>(index));
                const auto& face = result.mesh.polygons[index];
                const auto a=result.mesh.vertices[face.vertices[0]], b=result.mesh.vertices[face.vertices[1]], c=result.mesh.vertices[face.vertices[2]];
                region.area += std::abs(Cross(a,b,c))*0.5F;
                region.sourceTriangles.push_back(result.polygonSourceTriangles[index]);
                region.geometrySources.push_back(scene.triangleProvenance[result.polygonSourceTriangles[index]].geometrySource);
                for (auto neighbor : face.neighbors) if (neighbor != NoNeighbor && !seen[neighbor]) {
                    seen[neighbor]=true; pending.push(neighbor);
                }
                if (cellBounds) for (auto vertex : face.vertices) {
                    const auto p=result.mesh.vertices[vertex];
                    if (std::abs(p.x-cellBounds->min.x)<=cs*2 || std::abs(p.x-cellBounds->max.x)<=cs*2
                        || std::abs(p.y-cellBounds->min.y)<=cs*2 || std::abs(p.y-cellBounds->max.y)<=cs*2)
                        region.reachesBorder=true;
                }
            }
            std::sort(region.sourceTriangles.begin(),region.sourceTriangles.end());
            region.sourceTriangles.erase(std::unique(region.sourceTriangles.begin(),region.sourceTriangles.end()),region.sourceTriangles.end());
            std::sort(region.geometrySources.begin(),region.geometrySources.end());
            region.geometrySources.erase(std::unique(region.geometrySources.begin(),region.geometrySources.end()),region.geometrySources.end());
            result.regions.push_back(std::move(region));
        }
        for (auto& door : result.exits) {
            float best = std::numeric_limits<float>::max();
            for (const auto& region : result.regions) for (auto index : region.polygons) {
                const auto& face = result.mesh.polygons[index];
                const auto point = (result.mesh.vertices[face.vertices[0]]+result.mesh.vertices[face.vertices[1]]
                    +result.mesh.vertices[face.vertices[2]])/3.0F;
                const float dx=point.x-door.position.x, dy=point.y-door.position.y, dz=point.z-door.position.z;
                const float distance=std::hypot(dx,dy);
                if (distance<best && distance<=profile.agentRadius*4+cs*2 && std::abs(dz)<=profile.stepHeight+ch*2) {
                    best=distance; door.region=region.id;
                }
            }
            if (door.region) result.regions[*door.region].exitFormIds.push_back(door.referenceId);
        }
        result.statistics.polygonsBeforeSimplification = result.mesh.polygons.size();
        result.statistics.outputPolygons = result.mesh.polygons.size();
        result.warnings.push_back("Recast source-triangle provenance is matched by nearest surface after voxelization; generated geometry remains inspection-only.");
        result.warnings.push_back("Profile weld tolerance, contour tolerance, and cell-border policy are legacy polygon-generator settings and do not control Recast voxelization.");
        if (result.mesh.polygons.empty()) result.warnings.push_back("Recast produced no walkable polygons for the supplied geometry and profile.");
        result.topology = ValidateCandidateTopology(result);
        return result;
    }
}
