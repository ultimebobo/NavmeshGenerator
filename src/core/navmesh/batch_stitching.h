#pragma once

#include "core/navmesh/candidate.h"

#include <span>

namespace navmesh::core
{
    /// One mutable exterior candidate; CELL and worldspace identities are opaque neutral keys.
    struct GeneratedCellCandidate
    {
        /// Unique destination key used to resolve reciprocal generated portals.
        std::uint32_t cellId{};
        /// Separates overlapping exterior coordinate grids in different worldspaces.
        std::uint32_t worldspaceId{};
        /// Exact exterior clipping bounds in Skyrim world coordinates.
        AABB bounds;
        /// Borrowed mesh with complete source evidence; existing authored portals are pinned.
        CandidateNavMesh *candidate{};
    };

    /** Subdivide adjacent generated seam edges into a shared partition and add reciprocal portals.
     * @param cells Complete generated exterior set with unique worldspace/bounds ownership.
     * Every candidate remains present even when a neighbor has no surviving floor.
     * Each pass plans all interval intersections before editing either mesh.
     * Linking repeats until corner welding exposes no new compatible intervals;
     * established portal endpoints remain pinned through subsequent refinement. Shared heights
     * are welded within both movement profiles' climb limits; stacked floors are matched
     * by nearest compatible height. Authored portal endpoints are immutable constraints.
     * Different authored corner heights retain separate endpoints joined by
     * climb-compatible internal edges. Triangulation preserves interior edges,
     * sources, flags, regions and door anchors.
     * @return Number of reciprocal generated portal pairs added.
     * @throws std::runtime_error For invalid input, ambiguous partitions,
     * or invalid final topology; no authored fallback or candidate deletion is performed.
     * @warning Failure can leave candidates partially refined; they must not be exported.
     */
    [[nodiscard]] std::size_t StitchGeneratedCandidates(std::span<GeneratedCellCandidate> cells);
} // namespace navmesh::core
