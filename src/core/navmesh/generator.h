#pragma once

#include "core/navmesh/candidate.h"

namespace navmesh::core
{
    /** Validate finite movement, voxel and contour controls before generation or cache lookup.
     * Distances use Skyrim world units; region area uses square units and slope uses degrees.
     * @param profile Nonnegative radius, step and area; positive height, clearance and weld tolerance.
     * @param settings Positive voxel sizes and nonnegative contour/merge controls.
     * @throws std::invalid_argument On invalid values or voxel conversions outside Recast's integer range.
     */
    void ValidateRecastSettings(const NavigationProfile &profile, const RecastSettings &settings);

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
         * @param profile Agent dimensions and movement limits in Skyrim units.
         * @param cellBounds Optional exterior bounds in Skyrim world coordinates.
         * @param exits Enabled door positions in Skyrim world coordinates.
         * @param partitioningAlgorithm Recast region strategy; defaults to watershed.
         * @param settings Voxel and contour controls; horizontal resolution adapts to scene extent.
         * @return Candidate mesh and evidence; every polygon has a shared-edge path to a
         * matched door or exterior boundary edge. Empty when no component is anchored.
         * Throws on invalid input or a build failure.
         */
        [[nodiscard]] virtual CandidateNavMesh Generate(
            const Scene &scene, const NavigationProfile &profile, std::optional<AABB> cellBounds,
            std::vector<CandidateExit> exits,
            RegionPartitioningAlgorithm partitioningAlgorithm = RegionPartitioningAlgorithm::Watershed,
            const RecastSettings &settings = {}) const = 0;
    };

    /// Recast Navigation implementation of the neutral candidate generator.
    class RecastCandidateGenerator final : public CandidateGenerator
    {
      public:
        /** Rasterize authoritative Skyrim-world terrain and collision and retain only
         * walkable components with a shared-edge path to a matched door or exterior
         * boundary edge. Border anchors do not require an authored neighboring portal.
         * @param scene Geometry with complete triangle provenance in Skyrim world coordinates.
         * @param profile Agent dimensions and movement constraints in Skyrim world units.
         * @param cellBounds Optional exterior area in Skyrim world coordinates; absent for interiors.
         * @param exits Enabled placed DOOR references in Skyrim world coordinates.
         * @param partitioningAlgorithm Recast region strategy; defaults to watershed.
         * @param settings Voxel and contour controls; invalid settings throw before voxel allocation.
         * @return Candidate mesh and evidence, empty when no component is anchored;
         * throws on invalid input or a Recast build failure.
         */
        [[nodiscard]] CandidateNavMesh Generate(
            const Scene &scene, const NavigationProfile &profile, std::optional<AABB> cellBounds,
            std::vector<CandidateExit> exits,
            RegionPartitioningAlgorithm partitioningAlgorithm = RegionPartitioningAlgorithm::Watershed,
            const RecastSettings &settings = {}) const override;
    };
} // namespace navmesh::core
