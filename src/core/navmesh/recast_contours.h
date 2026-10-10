#pragma once

#include <Recast.h>
#include <memory>
#include <string>
#include <vector>

namespace navmesh::core
{
    enum class RegionPartitioningAlgorithm;

    namespace detail
    {
        /// Own a complete Recast contour set and release its buffers with the Recast allocator.
        using RecastContourOwner = std::unique_ptr<rcContourSet, decltype(&rcFreeContourSet)>;

        /// Own a Recast polygon mesh and release its buffers with the Recast allocator.
        using RecastPolyMeshOwner = std::unique_ptr<rcPolyMesh, decltype(&rcFreePolyMesh)>;

        /** Check that coarse floor polygons have clockwise, convex Recast XZ footprints.
         * @param mesh Polygon mesh using valid vertex indices and Recast voxel coordinates.
         * @return False for collapsed or inverted polygons, nonconvex turns, or repeated directed
         * edges on the same floor. Does not change vertices, polygons, or adjacency.
         */
        [[nodiscard]] bool HasConsistentRegionMesh(const rcPolyMesh &mesh);

        /** Build a coarse floor mesh suitable for height-detail sampling.
         * @param context Recast logging and allocation context.
         * @param compact Partitioned Recast Y-up heightfield; watershed recovery can change region IDs
         * and clear previously rejected areas, preserving retained floor heights and connections.
         * @param config Contour error and edge length in voxels, with a supported polygon vertex limit.
         * @param algorithm Initial region strategy; watershed permits layer recovery.
         * @param warnings Receives diagnostics for refinement or layer recovery.
         * @return Owned mesh covering every retained region with consistent coarse polygon winding.
         * Refines the complete contour partition together before trying layer recovery over retained spans.
         * @throws std::runtime_error For allocation, construction, or an unrepresentable retained partition.
         */
        [[nodiscard]] RecastPolyMeshOwner BuildRetainedRegionMesh(rcContext &context, rcCompactHeightfield &compact,
                                                                  const rcConfig &config,
                                                                  RegionPartitioningAlgorithm algorithm,
                                                                  std::vector<std::string> &warnings);

        /** Build one consistent contour partition covering every retained walkable region.
         * @param context Recast logging and allocation context.
         * @param compact Partitioned Recast Y-up heightfield. Watershed recovery can change region IDs
         * and clear walkable areas already rejected by the initial region filter; floor heights and connections stay intact.
         * @param config Nonnegative simplification error in horizontal voxels and edge length in voxel cells.
         * @param algorithm Initial region strategy; only watershed permits recovery with layer regions.
         * @param warnings Receives a diagnostic when recovery succeeds.
         * @return Owned contours built at or below the requested error; throws on allocation, construction,
         * or unrepresentable regions. Recovery preserves all initially retained spans without rerunning island filtering.
         */
        [[nodiscard]] RecastContourOwner BuildRetainedRegionContours(rcContext &context, rcCompactHeightfield &compact,
                                                                     const rcConfig &config,
                                                                     RegionPartitioningAlgorithm algorithm,
                                                                     std::vector<std::string> &warnings);
    } // namespace detail
} // namespace navmesh::core
