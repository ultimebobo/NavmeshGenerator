#include "core/navmesh/recast_contours.h"

#include "core/navmesh/generator.h"

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

        /// Refine the complete contour set together; null means a region is absent even at zero error.
        RecastContourOwner RefineRegionContours(rcContext &context, const rcCompactHeightfield &compact,
                                                const rcConfig &config)
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
                    return contours;
                }
                if (error == 0)
                {
                    return RecastContourOwner(nullptr, rcFreeContourSet);
                }
                error = error > 1 ? error * 0.5F : 0;
            }
        }
    } // namespace

    RecastContourOwner BuildRetainedRegionContours(rcContext &context, rcCompactHeightfield &compact,
                                                   const rcConfig &config, RegionPartitioningAlgorithm algorithm,
                                                   std::vector<std::string> &warnings)
    {
        auto contours = RefineRegionContours(context, compact, config);
        if (!contours && algorithm == RegionPartitioningAlgorithm::Watershed)
        {
            // A watershed region entirely enclosed by one other region has no
            // interface transitions. Recast simplifies that interface to two points
            // regardless of error tolerance. Layer partitioning creates representable
            // interfaces over the same retained floor instead of discarding the region.
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
            contours = RefineRegionContours(context, compact, config);
            if (contours)
            {
                warnings.push_back("Recast used layer region recovery because watershed contour refinement could "
                                   "not represent every retained walkable region.");
            }
        }
        if (!contours)
        {
            throw std::runtime_error("Recast contour construction lost a retained walkable region");
        }
        return contours;
    }
} // namespace navmesh::core::detail
