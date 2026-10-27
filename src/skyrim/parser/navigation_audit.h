#pragma once

#include "skyrim/parser/plugin_parser.h"

namespace navmesh::skyrim
{
    /// First occurrence of one structural fault kind in a winning NAVM; identities are resolved FormIDs.
    struct NavigationAuditFinding
    {
        std::uint32_t cellId{}, navmeshId{};
        /// Zero-based triangle, absent for mesh-wide faults.
        std::optional<std::uint32_t> polygon;
        /// Stable report key describing the failed invariant.
        std::string kind;
    };

    /// Geometry-free selection evidence; findings select owning cells, never neighboring regeneration targets.
    struct NavigationAudit
    {
        /// Decoded live meshes inspected and live record layouts that could not be inspected, respectively.
        std::size_t scannedNavmeshes{}, unsupportedNavmeshes{};
        std::vector<NavigationAuditFinding> findings;
    };

    /** Inspect live winning navigation without loading terrain, models or archives.
     * @param resolved Immutable resolved records and decoded world-space NAVMs, retained for this call.
     * @param stepHeight Allowed authored internal-edge height offset, in Skyrim units; finite and nonnegative.
     * @param cancelled Optional query between meshes and triangle batches; cancellation returns partial evidence.
     * @return Structural faults, deduplicated by NAVM and kind in CELL/triangle order. Deleted records,
     * disabled triangle geometry, valid empty meshes and cells without NAVM are not repair targets. Unsupported layouts
     * are counted separately, never guessed. Does not detect collision/support or heuristic island/gap issues.
     * @throws std::invalid_argument If stepHeight is invalid. Performs no I/O or mutations.
     */
    [[nodiscard]] NavigationAudit AuditNavigation(const ResolvedLoadOrder &resolved, float stepHeight,
                                                  const std::function<bool()> &cancelled = {});
} // namespace navmesh::skyrim
