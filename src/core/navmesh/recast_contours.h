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
