#pragma once

#include "core/navmesh/types.h"
#include "core/scene/scene.h"

#include <filesystem>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

namespace navmesh::core
{
    /// Maximum perpendicular deviation of an authored portal edge from its exterior CELL border,
    /// in Skyrim world units. Only matched portal endpoints may extend beyond candidate cell bounds.
    inline constexpr float AuthoredBorderTolerance{4.0F};

    /// Agent movement and surface constraints used to build a candidate navmesh.
    /// Distances and areas use Skyrim world units; angles use degrees.
    struct NavigationProfile
    {
        std::string name{"human"};
        /// Finite, nonnegative horizontal footprint radius in Skyrim world units.
        float agentRadius{16.0F};
        /// Finite, positive standing height in Skyrim world units.
        float agentHeight{128.0F};
        /// Finite walking slope limit in degrees, nonnegative and below a right angle.
        float maxSlopeDegrees{45.0F};
        /// Finite, nonnegative maximum traversable step height in Skyrim world units.
        float stepHeight{28.0F};
        /// Finite, positive headroom in Skyrim world units; Recast uses the larger of this and agentHeight.
        float clearance{128.0F};
        /// Finite, positive distance tolerance for geometry and authored-border matching in Skyrim units.
        float weldTolerance{0.05F};
        /// Finite, nonnegative minimum disconnected-island area, in square Skyrim world units.
        float minimumRegionArea{8172.0F};
        float contourSimplificationTolerance{0.05F};
        std::string cellBorderPolicy{"preserve_open_border"};
    };

    /// Recast voxel and contour controls; input geometry remains in Skyrim world space.
    struct RecastSettings
    {
        /// Finite, positive requested horizontal voxel size in Skyrim units; adaptive sizing may increase it.
        float cellSize{4.0F};
        /// Vertical voxel size in Skyrim units; must be finite and positive.
        float cellHeight{2.0F};
        /// Maximum contour deviation in horizontal voxels; finite and nonnegative.
        float maxSimplificationError{2.0F};
        /// Maximum contour edge length in Skyrim units; zero disables subdivision.
        float maxEdgeLength{};
        /// Merge area relative to the profile's minimum region area; ignored by layer partitioning.
        float mergeRegionAreaMultiplier{4.0F};
    };

    struct CandidateRegion
    {
        std::uint32_t id{};
        float area{};
        std::vector<std::uint32_t> polygons;
        std::vector<std::size_t> sourceTriangles;
        std::vector<std::size_t> geometrySources;
        /// True when this region has a boundary edge on the selected exterior bounds.
        bool reachesBorder{};
        /// Placed DOOR reference IDs whose world positions reach this region.
        std::vector<std::uint32_t> exitFormIds;
    };
    /// A placed, enabled DOOR reference that can anchor navigation at its world position.
    struct CandidateExit
    {
        std::uint32_t referenceId{};
        Vec3 position;
        /// Region reached by the exit, or no value when no walkable surface reaches it.
        std::optional<std::uint32_t> region;
        /// Candidate triangle beside this door, or no value when its threshold has no reachable floor.
        std::optional<std::uint32_t> polygon;
    };
    /// Reciprocal exterior edge portal between triangles in adjacent Skyrim world-space cells.
    struct CandidateBorderLink
    {
        /// Candidate triangle and edge indices, starting at zero in the generated mesh.
        std::uint32_t polygon{};
        std::uint8_t edge{};
        /// Resolved load-order FormID and triangle edge in the neighboring NAVM.
        std::uint32_t neighborNavmeshId{};
        std::uint32_t neighborPolygon{};
        std::uint8_t neighborEdge{};
    };
    struct CandidateContour
    {
        std::uint32_t region{};
        bool closed{};
        std::vector<std::uint32_t> vertices;
    };
    struct CandidateTopology
    {
        bool valid{true};
        std::vector<std::string> findings;
    };
    /// Input filter counts and output triangle counts for a candidate run.
    struct CandidateStatistics
    {
        std::size_t inputTriangles{}, eligibleTriangles{}, rejectedSlope{}, rejectedClearance{}, rejectedObstruction{},
            rejectedSource{}, rejectedDegenerate{}, rejectedSmallRegion{}, rejectedUnreachable{};
        /// Polygon count after filtering and before conservative interior simplification.
        std::size_t polygonsBeforeSimplification{};
        /// Final candidate triangle count after interior simplification.
        std::size_t outputPolygons{};
    };
    /// Neutral candidate geometry and evidence; a separate guarded writer can encode eligible overrides.
    struct CandidateNavMesh
    {
        NavigationProfile profile;
        /// Requested voxel/contour controls retained as reproducibility evidence.
        RecastSettings recastSettings;
        /// Recast region partition strategy used to build this candidate.
        std::string partitioningAlgorithm{"watershed"};
        NavMesh mesh;
        /// Primary input triangle for each output polygon, retained for simple source joins.
        std::vector<std::size_t> polygonSourceTriangles;
        /// All input triangles covered by each polygon after interior simplification.
        /// Indices refer to Scene::triangleProvenance in the input scene.
        std::vector<std::vector<std::size_t>> polygonContributingTriangles;
        std::vector<CandidateRegion> regions;
        std::vector<CandidateExit> exits;
        /// Candidate boundary edges with reciprocal targets in adjacent NAVMs.
        std::vector<CandidateBorderLink> borderLinks;
        std::vector<CandidateContour> contours;
        CandidateTopology topology;
        CandidateStatistics statistics;
        std::vector<std::string> warnings;
    };

    /// Build a candidate from supported terrain and collision triangles in a scene.
    /// @param scene Geometry and source evidence to inspect.
    /// @param profile Agent movement and surface constraints in Skyrim world units.
    /// @param cellBounds Optional generated exterior area's world-space bounds for border handling.
    /// @param exits Enabled placed DOOR references in Skyrim world coordinates. A door
    /// anchors only a nearby polygon on the same vertical level.
    /// @return Candidate geometry, source evidence, statistics, and topology findings.
    [[nodiscard]] CandidateNavMesh GenerateCandidate(const Scene &scene, const NavigationProfile &profile,
                                                     std::optional<AABB> cellBounds = std::nullopt,
                                                     std::vector<CandidateExit> exits = {});
    /** Align candidate boundary edges with neighboring NAVM edges on an exterior cell border.
     * @param candidate Generated mesh and complete source evidence to reshape in place.
     * @param cellBounds Selected exterior CELL in Skyrim world coordinates.
     * @param neighbors Existing NAVMs in adjacent cells of the same worldspace.
     * @return Number of reciprocal border portals added.
     * @warning Complete authored edges at compatible heights are joined; containing generated
     * edges may be subdivided and compatible collinear generated subdivisions coalesced.
     * Inward offsets trim boundary fans; outward extensions obey distance, step, slope, and welding limits. Authored
     * border drift within AuthoredBorderTolerance is preserved at matched portal endpoints.
     * Valid unlinked regions are retained. Boundary preparation can retriangulate compatible
     * fans even without a final match. Geometry, region membership, source joins, contour,
     * door and portal indices, and topology are updated consistently; neighbors are unchanged.
     * @throws std::invalid_argument when polygon source evidence is incomplete.
     */
    [[nodiscard]] std::size_t StitchCandidateBorders(CandidateNavMesh &candidate, const AABB &cellBounds,
                                                     const std::vector<NavMesh> &neighbors);
    /// Check candidate polygon topology without modifying its geometry.
    [[nodiscard]] CandidateTopology ValidateCandidateTopology(const CandidateNavMesh &candidate);
    /// Write the candidate and its source evidence as JSON; returns false on output failure.
    [[nodiscard]] bool WriteCandidateJson(const std::filesystem::path &path, const CandidateNavMesh &candidate,
                                          const Scene &scene, const std::string &metadataJson);
    /// Stream the same JSON schema to an existing output stream; returns false on write failure.
    [[nodiscard]] bool WriteCandidateJson(std::ostream &output, const CandidateNavMesh &candidate, const Scene &scene,
                                          const std::string &metadataJson);
    /// Write candidate triangles as an OBJ inspection mesh; returns false on output failure.
    [[nodiscard]] bool WriteCandidateObj(const std::filesystem::path &path, const CandidateNavMesh &candidate);
} // namespace navmesh::core
