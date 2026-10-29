#pragma once

#include "core/navmesh/candidate.h"

#include <optional>
#include <span>

namespace navmesh::core
{
    /// Local simplification limits in Skyrim world units, applied against immutable input geometry.
    struct SurfaceSimplificationSettings
    {
        /// Nonnegative contour edge limit; zero permits any straight boundary edge length.
        float maximumBoundaryEdgeLength{};
        /// Nonnegative vertical approximation budget; zero requires coplanarity.
        float maximumHeightError{};
        /// Walking slope limit in degrees between zero and a right angle; original steeper slopes remain the envelope.
        float maximumSlopeDegrees{90};
        /// Nonnegative horizontal inward boundary deviation; zero preserves the complete silhouette.
        /// Every removed footprint stays in the immutable input boundary band without expanding walkable floor.
        float maximumBoundaryInset{};
        /// Maximum removed horizontal area as a fraction of each original connected floor, in [0, 1].
        float maximumBoundaryAreaFraction{1};
    };

    /** Remove redundant samples from an unlinked generated floor with bounded vertical approximation.
     * Interior samples require a nearly planar incident fan; exposed samples also require a straight
     * boundary in world XYZ or an outward ear within the inward deviation budget. Exposed triangles
     * may be pruned only with a retained shared-edge route and complete coverage by the original boundary band.
     * Height changes exceed neither the error budget nor the original slope envelope; mixed flags
     * and ambiguous fans remain intact. Trimming never expands the floor or removes an entire island.
     * @param mesh Finite, counterclockwise Z-up floor triangles with valid indices and no door/external links.
     * Polygon neighbors are ignored and reset; polygon and vertex indices can change.
     * @param cellBounds Optional exterior bounds whose seam samples remain pinned for subsequent stitching.
     * @param settings Finite local error, edge and slope limits. Original floor and boundary samples
     * constrain every pass, so errors cannot accumulate. Retained vertex positions remain exact.
     * @param pinnedVertices Mesh vertex indices protected from removal, for example beside doors.
     * They may bound an interior retriangulation whose rim vertices and edges remain exact.
     * @param originalPolygons Optional output mapping each final polygon to its contributing input polygon indices.
     * @return Number of removed triangles. Unrepresentable local retriangulations leave their fans intact.
     * @throws std::invalid_argument For invalid indices, coordinates, simplification limits or existing connections.
     */
    std::size_t SimplifyGeneratedSurface(NavMesh &mesh, std::optional<AABB> cellBounds = {},
                                         const SurfaceSimplificationSettings &settings = {},
                                         std::span<const std::uint32_t> pinnedVertices = {},
                                         std::vector<std::vector<std::uint32_t>> *originalPolygons = nullptr);

    /** Clean up a provisionally linked candidate after authored border repairs, before generated batch linking.
     * Pins every portal and door triangle, exact CELL seams and mixed-region interfaces. Remaps polygon
     * evidence, region membership, doors and authored links transactionally, then rebuilds topology.
     * Pins generated drop triangles and preserves their shared-edge walking components.
     * @param candidate Valid Z-up candidate with complete polygon evidence and region membership.
     * @param cellBounds Exterior bounds for seam protection; absent for interiors.
     * @return Removed triangle count; zero when no safe reduction exists, validation rejects the result,
     * or generated-to-generated links already require coordinated identity remapping.
     * Height error is bounded by the vertical voxel size against this candidate's immutable input.
     * Trimming stays within the agent footprint diameter of the original boundary and obeys a
     * per-connected-floor horizontal area budget. Invoke once after exact generator cleanup;
     * approximation budgets do not carry across separate calls.
     */
    std::size_t SimplifyCandidateSurface(CandidateNavMesh &candidate, std::optional<AABB> cellBounds = {});
} // namespace navmesh::core
