#pragma once

#include "core/navmesh/candidate.h"

namespace navmesh::core
{
    /** Retain authoritative world-space collision for later drop-clearance queries.
     * @param candidate Generated floor footprint; replaces its owned ledgeGeometry.
     * @param scene Complete source provenance in Skyrim Z-up coordinates. Render fallback is excluded.
     * @param waterHeight Effective CELL/WRLD water plane Z in Skyrim units; absent for dry or unresolved input.
     */
    void CaptureLedgeGeometry(CandidateNavMesh &candidate, const Scene &scene,
                              std::optional<float> waterHeight = std::nullopt);

    /** Query physical water coverage retained with a candidate, independently of authored NAVM flags.
     * @param candidate Generated geometry with captured placed water surfaces and optional CELL/WRLD plane.
     * @param point Skyrim world position; only horizontal coordinates select coverage.
     * @return Highest covered water surface Z in Skyrim units, or no value outside known water coverage.
     */
    [[nodiscard]] std::optional<float> CandidateWaterHeight(const CandidateNavMesh &candidate, Vec3 point);

    /** Rebuild downward routes from physical collision and generated floors within one CELL.
     * @param candidate Valid world-space floor and retained ledgeGeometry. Uses profile dimensions,
     * climb and maximumDropHeight with vertical voxel uncertainty; captured deep water permits submerged landings.
     * Splits partial boundary spans and inserts interior receiving ports on unchanged floor planes, remapping
     * provenance, regions and door anchors. External portal triangles stay pinned. Replaces ledgeLinks,
     * reusing valid existing ports. Authored navigation is neither read nor used as a generation constraint.
     * @return Number of paired edges. Occupied portals, unsupported or obstructed landings and unsafe falls
     * supply no link; ordinary floor components need no additional drop connection.
     * @throws std::runtime_error When a planned partition cannot retain valid topology.
     * @warning A failed partition can leave the candidate partially edited; discard it instead of exporting.
     */
    std::size_t BuildCandidateLedgeLinks(CandidateNavMesh &candidate);
} // namespace navmesh::core
