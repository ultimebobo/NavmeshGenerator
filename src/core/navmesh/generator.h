#pragma once

#include "core/navmesh/candidate.h"

namespace navmesh::core
{
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
         * @return Candidate mesh and evidence; throws on invalid input or a build failure.
         */
        [[nodiscard]] virtual CandidateNavMesh Generate(const Scene& scene, const NavigationProfile& profile,
            std::optional<AABB> cellBounds, std::vector<CandidateExit> exits) const = 0;
    };

    /// Recast Navigation implementation of the neutral candidate generator.
    class RecastCandidateGenerator final : public CandidateGenerator
    {
    public:
        /// Rasterize terrain and collision, then contour and triangulate Recast regions.
        [[nodiscard]] CandidateNavMesh Generate(const Scene& scene, const NavigationProfile& profile,
            std::optional<AABB> cellBounds, std::vector<CandidateExit> exits) const override;
    };
}
