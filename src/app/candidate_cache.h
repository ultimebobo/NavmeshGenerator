#pragma once

#include "core/navmesh/candidate.h"

#include <span>

namespace navmesh::app::detail
{
    /** Fingerprint actual ordered generation inputs and authored neighboring portals.
     * Includes geometry, provenance, profile, voxel controls, partition strategy, exits, bounds, and tool/schema identity.
     * Neighbor changes invalidate each dependent target independently; no plugin filename shortcut is used.
     * @param scene Ordered Skyrim-world geometry and complete provenance.
     * @param profile Valid movement constraints in Skyrim world units.
     * @param bounds Optional exterior bounds in Skyrim world coordinates.
     * @param exits Ordered placed door positions and resolved identities.
     * @param neighbors Ordered authored neighboring meshes and portal evidence.
     * @param partitioning Recast region strategy name.
     * @param settings Valid requested voxel/contour controls; changes invalidate cached generation.
     * @return Stable dependency hash, or empty if tool identity or hashing cannot be obtained.
     */
    [[nodiscard]] std::string CandidateFingerprint(const core::Scene &scene, const core::NavigationProfile &profile,
                                                   std::optional<core::AABB> bounds,
                                                   const std::vector<core::CandidateExit> &exits,
                                                   const std::vector<core::NavMesh> &neighbors,
                                                   std::string_view partitioning,
                                                   const core::RecastSettings &settings = {});

    /// Store compact candidate evidence as private gzip data; atomic rename, false on failure.
    [[nodiscard]] bool StoreCandidate(const std::filesystem::path &path, const core::CandidateNavMesh &candidate,
                                      const core::Scene &evidence);

    /// Read bounded private cache data; false for missing, corrupt, incompatible, or invalid topology data.
    [[nodiscard]] bool LoadCandidate(const std::filesystem::path &path, core::CandidateNavMesh &candidate,
                                     core::Scene &evidence);

    /// Release audit arrays after spooling; keeps geometry, portals, exits, profile and topology for final writing.
    void ReleaseCandidateAudit(core::CandidateNavMesh &candidate);
} // namespace navmesh::app::detail
