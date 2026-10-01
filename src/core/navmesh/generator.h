#pragma once

#include "core/navmesh/candidate.h"

namespace navmesh::core
{
    /// Region partition strategy used by Recast after walkable-area erosion.
    enum class RegionPartitioningAlgorithm
    {
        /// Grow regions outward from distance-field maxima; tends to produce natural boundaries.
        Watershed,
        /// Sweep regions along one axis; avoids watershed's small-region artifacts.
        Monotone,
        /// Partition independently between heightfield layers.
        Layers
    };

    /// Replaceable algorithm boundary for neutral, inspection-only navmesh generation.
    class CandidateGenerator
    {
      public:
        virtual ~CandidateGenerator() = default;

        /** Generate world-space navigation geometry from authoritative scene triangles.
         * @param scene Skyrim-world geometry with complete triangle provenance.
         * @param profile Fixed human agent dimensions and movement limits in Skyrim units.
         * @param cellBounds Optional exterior bounds in Skyrim world coordinates.
         * @param exits Enabled door positions in Skyrim world coordinates.
         * @param partitioningAlgorithm Recast region strategy; defaults to watershed.
         * @return Candidate mesh and evidence; throws on invalid input or a build failure.
         */
        [[nodiscard]] virtual CandidateNavMesh Generate(
            const Scene &scene, const NavigationProfile &profile, std::optional<AABB> cellBounds,
            std::vector<CandidateExit> exits,
            RegionPartitioningAlgorithm partitioningAlgorithm = RegionPartitioningAlgorithm::Watershed) const = 0;
    };

    /// Recast Navigation implementation of the neutral candidate generator.
    class RecastCandidateGenerator final : public CandidateGenerator
    {
      public:
        /** Rasterize authoritative Skyrim-world terrain and collision, then export only
         * connected regions that reach a nearby door or the exterior cell bounds.
         * An unanchored result has an empty mesh and a warning.
         * @param scene Geometry with complete triangle provenance in Skyrim world coordinates.
         * @param profile Agent dimensions and movement constraints in Skyrim world units.
         * @param cellBounds Optional exterior area in Skyrim world coordinates; absent for interiors.
         * @param exits Enabled placed DOOR references in Skyrim world coordinates.
         * @param partitioningAlgorithm Recast region strategy; defaults to watershed.
         * @return Candidate mesh and evidence; throws on invalid input or a Recast build failure.
         */
        [[nodiscard]] CandidateNavMesh Generate(
            const Scene &scene, const NavigationProfile &profile, std::optional<AABB> cellBounds,
            std::vector<CandidateExit> exits,
            RegionPartitioningAlgorithm partitioningAlgorithm = RegionPartitioningAlgorithm::Watershed) const override;
    };
} // namespace navmesh::core
