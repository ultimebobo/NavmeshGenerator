#pragma once

#include "core/navmesh/candidate.h"

#include <span>

namespace navmesh::core
{
    /// Borrowed candidate and opaque ownership keys for complete-set reachability filtering.
    struct CandidateReachabilityTarget
    {
        /// Unique CELL key used by generated reciprocal portals.
        std::uint32_t cellId{};
        /// Exterior worldspace key; absent for independently filtered interiors.
        std::optional<std::uint32_t> worldspaceId;
        /// Exterior CELL bounds in Skyrim world units, with finite positive XY extents;
        /// present exactly when worldspaceId is present. Vertical extents do not affect connectivity.
        std::optional<AABB> bounds;
        /// Mutable mesh with complete polygon provenance and valid adjacency, doors and portals.
        CandidateNavMesh *candidate{};
    };

    /** Remove disconnected floor islands after all generated portals have been reconciled.
     * @param targets Complete successful generated set, with unique CELL keys and mesh pointers. Shared edges
     * and reciprocal generated portals define connectivity; touching vertices do not.
     * Retains components reaching matched doors or untouched authored NAVMs, plus the largest
     * component by horizontal area in each contiguous exterior selection and each interior.
     * Exterior selections are grouped by shared CELL sides within the same worldspace,
     * independently of floor heights. Open CELL borders and generated portals alone are not anchors.
     * Compacts vertices, polygons, regions, source joins, doors and both ends of generated portals;
     * rebuilds contours and topology, and increments rejectedUnreachable for removed triangles.
     * @return Total removed triangle count. Empty candidates remain valid replacement targets.
     * @throws std::runtime_error On invalid ownership, topology or reciprocal portal destinations.
     * @warning Failure can leave candidates partially compacted; they must not be exported.
     */
    [[nodiscard]] std::size_t RemoveCandidateIslands(std::span<CandidateReachabilityTarget> targets);
} // namespace navmesh::core
