#pragma once

#include "core/navmesh/candidate.h"

#include <span>

namespace navmesh::core
{
    /** Classify final generated triangles without changing geometry or connection indices.
     * Water uses effective CELL/WRLD and captured placed water surfaces independently of authored markings. Preferred
     * routes use each triangle's XY centroid and interpolated authored height to select the closest
     * overlapping floor within the profile climb or vertical voxel tolerance. Deleted, malformed
     * and degenerate authored faces are ignored.
     * @param candidate Final Skyrim-world candidate, after border reshaping; updated in place.
     * @param authored Winning selected-cell NAVMs, including unmarked floors that constrain matches.
     * @param waterHeight Effective finite water-surface Z in Skyrim world units. Submerged
     * centroids are water; absent height still permits captured placed water coverage.
     * @param tagWater Enable physical water classification; false clears water flags.
     * @param tagPreferredPaths Enable authored route transfer; false clears preferred-path flags.
     * Both settings preserve geometry, connection indices and every other flag.
     * @throws std::invalid_argument If a supplied water height is nonfinite.
     */
    void TagCandidateTriangles(CandidateNavMesh &candidate, std::span<const NavMesh> authored,
                               std::optional<float> waterHeight, bool tagWater = true, bool tagPreferredPaths = true);

    /** Retain authored large-creature exclusions on corresponding generated floors.
     * @param mesh Generated Skyrim-world geometry; only its creature-restriction flag may be added.
     * @param authored Original meshes; overlapping unmarked floors constrain stacked-floor matches.
     * @param heightTolerance Finite nonnegative world-space separation allowed between matching floors.
     * @throws std::invalid_argument If the height tolerance is invalid.
     */
    void PreserveAuthoredRestrictions(NavMesh &mesh, std::span<const NavMesh> authored, float heightTolerance);
} // namespace navmesh::core
