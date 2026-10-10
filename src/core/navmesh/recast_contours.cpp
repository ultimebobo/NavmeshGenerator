#include "core/navmesh/recast_contours.h"

#include "core/navmesh/generator.h"

#include <array>
#include <cstdint>
#include <set>
#include <stdexcept>

namespace navmesh::core::detail
{
    namespace
    {
        /// Require a contour for each non-border region owning surviving walkable spans.
        bool HasAllRegionContours(const rcCompactHeightfield &compact, const rcContourSet &contours)
        {
            std::vector<bool> represented(static_cast<std::size_t>(compact.maxRegions) + 1);
            for (int contour{}; contour < contours.nconts; ++contour)
            {
                const auto &boundary = contours.conts[contour];
                if (boundary.nverts >= 3 && boundary.reg < represented.size())
                {
                    represented[boundary.reg] = true;
                }
            }
            for (int span{}; span < compact.spanCount; ++span)
            {
                const auto region = compact.spans[span].reg;
                if (compact.areas[span] != RC_NULL_AREA && region && !(region & RC_BORDER_REG) &&
                    (region >= represented.size() || !represented[region]))
                {
                    return false;
                }
            }
            return true;
        }

        /// Polygon construction must not drop a retained region represented by a contour.
        bool HasAllRegionPolygons(const rcContourSet &contours, const rcPolyMesh &mesh)
        {
            std::set<unsigned short> represented;
            for (int polygon{}; polygon < mesh.npolys; ++polygon)
            {
                represented.insert(mesh.regs[polygon]);
            }
            for (int contour{}; contour < contours.nconts; ++contour)
            {
                const auto &boundary = contours.conts[contour];
                if (boundary.nverts >= 3 && !(boundary.reg & RC_BORDER_REG) && !represented.contains(boundary.reg))
                {
                    return false;
                }
            }
            return true;
        }

        /** Refine the complete contour set together. When a mesh is requested,
         * accept only clockwise convex coarse polygons with consistent shared edges.
         * Null means the retained partition is unrepresentable even at zero error.
         */
        RecastContourOwner RefineRegionContours(rcContext &context, const rcCompactHeightfield &compact,
                                                const rcConfig &config, RecastPolyMeshOwner *mesh = nullptr,
                                                std::vector<std::string> *warnings = nullptr)
        {
            auto error = config.maxSimplificationError;
            while (true)
            {
                RecastContourOwner contours(rcAllocContourSet(), rcFreeContourSet);
                if (!contours || !rcBuildContours(&context, compact, error, config.maxEdgeLen, *contours))
                {
                    throw std::runtime_error("Recast contour construction failed");
                }
                // Shared interfaces must use one simplification pass, so adjacent
                // regions cannot overlap through a mixture of coarse and fine contours.
                if (HasAllRegionContours(compact, *contours))
                {
                    if (!mesh)
                    {
                        return contours;
                    }
                    RecastPolyMeshOwner trial(rcAllocPolyMesh(), rcFreePolyMesh);
                    if (!trial)
                    {
                        throw std::runtime_error("Recast polygon allocation failed");
                    }
                    if (rcBuildPolyMesh(&context, *contours, config.maxVertsPerPoly, *trial) &&
                        HasConsistentRegionMesh(*trial) && HasAllRegionPolygons(*contours, *trial))
                    {
                        *mesh = std::move(trial);
                        if (warnings && error != config.maxSimplificationError)
                        {
                            warnings->push_back("Recast refined the complete contour partition to produce "
                                                "consistent coarse floor polygons.");
                        }
                        return contours;
                    }
                }
                if (error == 0)
                {
                    return RecastContourOwner(nullptr, rcFreeContourSet);
                }
                error = error > 1 ? error * 0.5F : 0;
            }
        }

        /// Recover a failed watershed partition over the same retained spans without repeating island filtering.
        RecastContourOwner BuildRegionPartition(rcContext &context, rcCompactHeightfield &compact,
                                                const rcConfig &config, RegionPartitioningAlgorithm algorithm,
                                                std::vector<std::string> &warnings, RecastPolyMeshOwner *mesh = nullptr)
        {
            auto contours = RefineRegionContours(context, compact, config, mesh, mesh ? &warnings : nullptr);
            if (!contours && algorithm == RegionPartitioningAlgorithm::Watershed)
            {
                // Layer partitioning supplies new region interfaces when refinement
                // cannot represent the retained floor or produces folded polygons.
                // Mask initially filtered islands and disable further island filtering:
                // repartitioning must neither restore rejected spans nor remove retained ones.
                for (int span{}; span < compact.spanCount; ++span)
                {
                    if (compact.spans[span].reg == 0)
                    {
                        compact.areas[span] = RC_NULL_AREA;
                    }
                }
                if (!rcBuildLayerRegions(&context, compact, 0, 0))
                {
                    throw std::runtime_error("Recast layer region recovery failed");
                }
                contours = RefineRegionContours(context, compact, config, mesh, mesh ? &warnings : nullptr);
                if (contours)
                {
                    warnings.push_back(mesh ? "Recast used layer region recovery because watershed contour refinement "
                                              "could not produce a consistent mesh for every retained walkable region."
                                            : "Recast used layer region recovery because watershed contour refinement "
                                              "could not represent every retained walkable region.");
                }
            }
            if (!contours)
            {
                throw std::runtime_error(mesh ? "Recast could not construct a consistent retained walkable region mesh"
                                              : "Recast contour construction lost a retained walkable region");
            }
            return contours;
        }
    } // namespace

    bool HasConsistentRegionMesh(const rcPolyMesh &mesh)
    {
        // Integer voxel coordinates allow exact winding and convexity checks.
        // Full XYZ edge identities keep separate stacked floors independent.
        using Point = std::array<unsigned short, 3>;
        std::set<std::pair<Point, Point>> directed;
        const auto point = [&](unsigned short vertex)
        {
            const auto *coordinates = mesh.verts + vertex * 3;
            return Point{coordinates[0], coordinates[1], coordinates[2]};
        };
        const auto turn = [](const Point &a, const Point &b, const Point &c)
        {
            return (std::int64_t(b[0]) - a[0]) * (std::int64_t(c[2]) - a[2]) -
                   (std::int64_t(b[2]) - a[2]) * (std::int64_t(c[0]) - a[0]);
        };
        for (int polygon{}; polygon < mesh.npolys; ++polygon)
        {
            const auto *vertices = mesh.polys + polygon * mesh.nvp * 2;
            int count{};
            while (count < mesh.nvp && vertices[count] != RC_MESH_NULL_IDX)
            {
                ++count;
            }
            if (count < 3)
            {
                return false;
            }
            std::int64_t area{};
            for (int edge{}; edge < count; ++edge)
            {
                const auto a = point(vertices[edge]);
                const auto b = point(vertices[(edge + 1) % count]);
                const auto c = point(vertices[(edge + 2) % count]);
                area += turn(point(vertices[0]), a, b);
                if (turn(a, b, c) > 0 || !directed.emplace(a, b).second)
                {
                    return false;
                }
            }
            if (area >= 0)
            {
                return false;
            }
        }
        return true;
    }

    RecastContourOwner BuildRetainedRegionContours(rcContext &context, rcCompactHeightfield &compact,
                                                   const rcConfig &config, RegionPartitioningAlgorithm algorithm,
                                                   std::vector<std::string> &warnings)
    {
        return BuildRegionPartition(context, compact, config, algorithm, warnings);
    }

    RecastPolyMeshOwner BuildRetainedRegionMesh(rcContext &context, rcCompactHeightfield &compact,
                                                const rcConfig &config, RegionPartitioningAlgorithm algorithm,
                                                std::vector<std::string> &warnings)
    {
        RecastPolyMeshOwner mesh(nullptr, rcFreePolyMesh);
        (void)BuildRegionPartition(context, compact, config, algorithm, warnings, &mesh);
        return mesh;
    }
} // namespace navmesh::core::detail
