#pragma once

#include "core/navmesh/types.h"

#include <span>
#include <vector>

namespace navmesh::core
{
    struct NavigationProfile;
    /// A bounded vertex adjustment to an adjacent authored NAVM in Skyrim world coordinates.
    struct AuthoredBorderVertexEdit
    {
        /// Resolved load-order NAVM identity and unchanged zero-based authored vertex index.
        std::uint32_t navmeshId{};
        std::uint32_t vertex{};
        /// Exact input position protects against stale input snapshots and conflicting edits.
        Vec3 original;
        Vec3 position;
    };

    /** Apply border edits transactionally without changing triangle or connection identities.
     * @param mesh Authored neighbor; only edits naming its resolved FormID apply.
     * @param edits Original-position checked edits in Skyrim world units.
     * @param profile Bounds horizontal displacement by the border search halo and vertical
     * displacement by climb plus slope over that halo. Incident faces must keep their
     * winding and cannot exceed their original or profile slope envelope.
     * @return False for invalid, conflicting, out-of-bounds or inverted edits, leaving mesh unchanged.
     */
    [[nodiscard]] bool ApplyAuthoredBorderVertexEdits(NavMesh &mesh, std::span<const AuthoredBorderVertexEdit> edits,
                                                      const NavigationProfile &profile);

    /** Extend requested endpoint edits with bounded height adjustments through their authored fan.
     * @param mesh Immutable input neighbor in Skyrim world coordinates.
     * @param edits Endpoint edits; receives a complete valid plan only on success.
     * @param pinned Requested endpoints and vertices consumed by unrelated or already matched portals,
     * which stay fixed at their requested positions. Other local fan heights may relax.
     * @param profile Displacement and per-face slope constraints used by application and export.
     * @return False when no bounded fan relaxation is found; edits remains unchanged.
     */
    [[nodiscard]] bool CompleteAuthoredBorderFanEdits(const NavMesh &mesh, std::vector<AuthoredBorderVertexEdit> &edits,
                                                      std::span<const std::uint32_t> pinned,
                                                      const NavigationProfile &profile);
} // namespace navmesh::core
