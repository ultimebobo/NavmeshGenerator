#pragma once

#include "core/navmesh/candidate.h"

namespace navmesh::core
{
    /// Reachability policy for standalone inspection or a batch whose neighbors are generated later.
    enum class CandidateRetention
    {
        /// Keep components with a door or exterior boundary anchor.
        Anchored,
        /// Keep every surviving Recast floor component for subsequent batch linking.
        AllWalkable
    };
    /** Validate finite movement, voxel and contour controls before generation or cache lookup.
     * Distances use Skyrim world units; region area uses square units and slope uses degrees.
     * @param profile Nonnegative radius, step and area; positive height, clearance and weld tolerance.
     * @param settings Positive voxel sizes and nonnegative contour/merge controls.
     * @throws std::invalid_argument On invalid values or voxel conversions outside Recast's integer range.
     */
    void ValidateRecastSettings(const NavigationProfile &profile, const RecastSettings &settings);

    /** Reject supported geometry whose floor heights would saturate Recast raster spans.
     * @param scene Authoritative Skyrim Z-up geometry with complete triangle provenance.
     * @param profile Valid movement constraints in Skyrim world units.
     * @param cellBounds Optional exterior bounds; validation uses the generation halo and clipping.
     * @param settings Requested voxel controls; vertical cell height stays fixed.
     * @throws std::runtime_error When retained collision or terrain exceeds the vertical raster range.
     * Also validates settings and input geometry. Does not rasterize or change the scene.
     */
    void ValidateRecastSceneHeightRange(const Scene &scene, const NavigationProfile &profile,
                                        std::optional<AABB> cellBounds, const RecastSettings &settings);

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

    /// Replaceable algorithm boundary for neutral navmesh generation.
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
         * @param retention Retain all walkable components when neighboring candidates are not yet available.
         * @return Candidate mesh and evidence. Anchored retains components reaching a door
         * or exterior boundary; AllWalkable retains every surviving floor component.
         * Empty when no floor survives the selected retention policy.
         * Throws on invalid input or a build failure.
         */
        [[nodiscard]] virtual CandidateNavMesh Generate(
            const Scene &scene, const NavigationProfile &profile, std::optional<AABB> cellBounds,
            std::vector<CandidateExit> exits,
            RegionPartitioningAlgorithm partitioningAlgorithm = RegionPartitioningAlgorithm::Watershed,
            const RecastSettings &settings = {}, CandidateRetention retention = CandidateRetention::Anchored) const = 0;
    };

    /// Recast Navigation implementation of the neutral candidate generator.
    class RecastCandidateGenerator final : public CandidateGenerator
    {
      public:
        /** Rasterize authoritative Skyrim-world terrain and collision, including vertical
         * and obstacle-only solids, and sample floor heights into detail triangles.
         * Convex patches share height detail with approximation error bounded by the larger
         * of climb and vertical voxel size. Contour error is refined if a retained voxel
         * region would collapse. Unrepresentable watershed contours trigger layer recovery
         * over the already-retained spans, reported in warnings. Obstacle-only sources cannot supply
         * walkable floors. Retain only components with a shared-edge path to a matched door or exterior
         * boundary edge as provisional anchors under Anchored retention. AllWalkable
         * keeps unanchored floors for whole-batch stitching.
         * @param scene Geometry with complete triangle provenance in Skyrim world coordinates.
         * @param profile Agent dimensions and movement constraints in Skyrim world units.
         * @param cellBounds Optional exterior area in Skyrim world coordinates; absent for interiors.
         * @param exits Enabled placed DOOR references in Skyrim world coordinates.
         * @param partitioningAlgorithm Recast region strategy; defaults to watershed.
         * @param settings Voxel and contour controls; invalid settings throw before voxel allocation.
         * @param retention AllWalkable keeps unanchored floors for subsequent batch linking.
         * @return Candidate mesh and evidence, with warnings for adaptive horizontal
         * resolution and downward climb quantization; empty when no floor survives retention;
         * throws on invalid input, unrepresentable vertical scene extent, or a Recast build failure.
         */
        [[nodiscard]] CandidateNavMesh Generate(
            const Scene &scene, const NavigationProfile &profile, std::optional<AABB> cellBounds,
            std::vector<CandidateExit> exits,
            RegionPartitioningAlgorithm partitioningAlgorithm = RegionPartitioningAlgorithm::Watershed,
            const RecastSettings &settings = {},
            CandidateRetention retention = CandidateRetention::Anchored) const override;
    };
} // namespace navmesh::core
