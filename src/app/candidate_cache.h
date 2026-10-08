#pragma once

#include "core/navmesh/candidate.h"

#include <span>

namespace navmesh::app::detail
{
    /** Fingerprint actual ordered generation inputs and authored neighboring portals.
     * Includes geometry, provenance, profile, voxel controls, partition strategy, tagging evidence,
     * exits, bounds, and cache/pipeline compatibility identity. Compatible executable
     * rebuilds share keys; successful-generation changes require a pipeline revision.
     * Neighbor changes invalidate each dependent target independently; no plugin filename shortcut is used.
     * @param scene Ordered Skyrim-world geometry and complete provenance.
     * @param profile Valid movement constraints in Skyrim world units.
     * @param bounds Optional exterior bounds in Skyrim world coordinates.
     * @param exits Ordered placed door positions and resolved identities.
     * @param neighbors Ordered authored neighboring meshes and portal evidence.
     * @param partitioning Recast region strategy name.
     * @param settings Valid requested voxel/contour controls; changes invalidate cached generation.
     * @param authored Winning selected-cell meshes used for triangle classification in batch generation;
     * changes invalidate tagging. Batch seam geometry depends on generated neighbors after cache reuse.
     * @param waterHeight Effective water-surface Z in Skyrim units, or no supported plane.
     * @param tagTriangles Whether generated triangle classification is enabled.
     * @return Stable dependency hash, or empty if input serialization exceeds the cache limit.
     */
    [[nodiscard]] std::string CandidateFingerprint(
        const core::Scene &scene, const core::NavigationProfile &profile, std::optional<core::AABB> bounds,
        const std::vector<core::CandidateExit> &exits, const std::vector<core::NavMesh> &neighbors,
        std::string_view partitioning, const core::RecastSettings &settings = {},
        const std::vector<core::NavMesh> &authored = {}, std::optional<float> waterHeight = std::nullopt,
        bool tagTriangles = true);

    /// Store compact candidate evidence as private gzip data; atomic rename, false on failure.
    [[nodiscard]] bool StoreCandidate(const std::filesystem::path &path, const core::CandidateNavMesh &candidate,
                                      const core::Scene &evidence);

    /// Read bounded private cache data; false for missing, corrupt, incompatible, or invalid topology data.
    [[nodiscard]] bool LoadCandidate(const std::filesystem::path &path, core::CandidateNavMesh &candidate,
                                     core::Scene &evidence);

    /// Release audit arrays after spooling; keeps geometry, portals, exits, profile and topology for final writing.
    void ReleaseCandidateAudit(core::CandidateNavMesh &candidate);
} // namespace navmesh::app::detail
