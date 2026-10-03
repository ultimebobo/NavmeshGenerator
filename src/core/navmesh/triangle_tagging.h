#pragma once

#include "core/navmesh/candidate.h"

#include <span>

namespace navmesh::core
{
    /** Classify final generated triangles without changing geometry or connection indices.
     * Uses each triangle's XY centroid and interpolated authored height to select the closest
     * overlapping floor within the profile climb or vertical voxel tolerance. Only water and
     * preferred-route bits transfer; deleted, malformed and degenerate authored faces are ignored.
     * @param candidate Final Skyrim-world candidate, after border reshaping; updated in place.
     * @param authored Winning selected-cell NAVMs, including unmarked floors that constrain matches.
     * @param waterHeight Effective finite exterior water-surface Z in Skyrim units. Submerged
     * centroids are water; absent height falls back to matching authored water markings.
     * @param enabled False clears only the two classification bits, preserving every other flag.
     * @throws std::invalid_argument If a supplied water height is nonfinite.
     */
    void TagCandidateTriangles(CandidateNavMesh &candidate, std::span<const NavMesh> authored,
                               std::optional<float> waterHeight, bool enabled = true);
} // namespace navmesh::core
