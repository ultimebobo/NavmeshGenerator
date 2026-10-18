#pragma once

#include "core/navmesh/candidate.h"

#include <span>

namespace navmesh::core
{
    /** Classify final generated triangles without changing geometry or connection indices.
     * Water uses the effective CELL/WRLD surface independently of authored markings. Preferred
     * routes use each triangle's XY centroid and interpolated authored height to select the closest
     * overlapping floor within the profile climb or vertical voxel tolerance. Deleted, malformed
     * and degenerate authored faces are ignored.
     * @param candidate Final Skyrim-world candidate, after border reshaping; updated in place.
     * @param authored Winning selected-cell NAVMs, including unmarked floors that constrain matches.
     * @param waterHeight Effective finite water-surface Z in Skyrim world units. Submerged
     * centroids are water; absent height supplies no water classification.
     * @param enabled False clears only the two classification bits, preserving every other flag.
     * @throws std::invalid_argument If a supplied water height is nonfinite.
     */
    void TagCandidateTriangles(CandidateNavMesh &candidate, std::span<const NavMesh> authored,
                               std::optional<float> waterHeight, bool enabled = true);

    /** Retain authored large-creature exclusions on corresponding generated floors.
     * @param mesh Generated Skyrim-world geometry; only its creature-restriction flag may be added.
     * @param authored Original meshes; overlapping unmarked floors constrain stacked-floor matches.
     * @param heightTolerance Finite nonnegative world-space separation allowed between matching floors.
     * @throws std::invalid_argument If the height tolerance is invalid.
     */
    void PreserveAuthoredRestrictions(NavMesh &mesh, std::span<const NavMesh> authored, float heightTolerance);
} // namespace navmesh::core
