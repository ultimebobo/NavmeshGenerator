#pragma once

#include "core/navmesh/types.h"
#include "core/scene/scene.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace navmesh::core
{
    /// Fixed human movement and surface constraints used to build a candidate navmesh.
    /// Distances and areas use Skyrim world units; angles use degrees.
    struct NavigationProfile
    {
        std::string name{ "human" };
        float agentRadius{ 16.0F };
        float agentHeight{ 128.0F };
        float maxSlopeDegrees{ 45.0F };
        /// Maximum traversable step height in Skyrim world units.
        float stepHeight{ 28.0F };
        float clearance{ 128.0F };
        float weldTolerance{ 0.05F };
        /// Minimum area for a disconnected walkable island, in square Skyrim world units.
        float minimumRegionArea{ 8172.0F };
        float contourSimplificationTolerance{ 0.05F };
        std::string cellBorderPolicy{ "preserve_open_border" };
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
    };
    struct CandidateContour { std::uint32_t region{}; bool closed{}; std::vector<std::uint32_t> vertices; };
    struct CandidateTopology { bool valid{ true }; std::vector<std::string> findings; };
    /// Input filter counts and output triangle counts for a candidate run.
    struct CandidateStatistics
    {
        std::size_t inputTriangles{}, eligibleTriangles{}, rejectedSlope{}, rejectedClearance{}, rejectedObstruction{}, rejectedSource{},
            rejectedDegenerate{}, rejectedSmallRegion{}, rejectedUnreachable{};
        /// Polygon count after filtering and before conservative interior simplification.
        std::size_t polygonsBeforeSimplification{};
        /// Final candidate triangle count after interior simplification.
        std::size_t outputPolygons{};
    };
    /// Neutral candidate geometry and evidence; a separate guarded writer can encode eligible overrides.
    struct CandidateNavMesh
    {
        NavigationProfile profile;
        /// Recast region partition strategy used to build this candidate.
        std::string partitioningAlgorithm{ "watershed" };
        NavMesh mesh;
        /// Primary input triangle for each output polygon, retained for simple source joins.
        std::vector<std::size_t> polygonSourceTriangles;
        /// All input triangles covered by each polygon after interior simplification.
        /// Indices refer to Scene::triangleProvenance in the input scene.
        std::vector<std::vector<std::size_t>> polygonContributingTriangles;
        std::vector<CandidateRegion> regions;
        std::vector<CandidateExit> exits;
        std::vector<CandidateContour> contours;
        CandidateTopology topology;
        CandidateStatistics statistics;
        std::vector<std::string> warnings;
    };

    /// Build a candidate from supported terrain and collision triangles in a scene.
    /// @param scene Geometry and source evidence to inspect.
    /// @param profile Fixed human movement and surface constraints in Skyrim world units.
    /// @param cellBounds Optional generated exterior area's world-space bounds for border handling.
    /// @param exits Enabled placed DOOR references in Skyrim world coordinates. A door
    /// anchors only a nearby polygon on the same vertical level.
    /// @return Candidate geometry, source evidence, statistics, and topology findings.
    [[nodiscard]] CandidateNavMesh GenerateCandidate(const Scene& scene, const NavigationProfile& profile,
        std::optional<AABB> cellBounds = std::nullopt, std::vector<CandidateExit> exits = {});
    /// Check candidate polygon topology without modifying its geometry.
    [[nodiscard]] CandidateTopology ValidateCandidateTopology(const CandidateNavMesh& candidate);
    /// Write the candidate and its source evidence as JSON; returns false on output failure.
    [[nodiscard]] bool WriteCandidateJson(const std::filesystem::path& path, const CandidateNavMesh& candidate,
        const Scene& scene, const std::string& metadataJson);
    /// Write candidate triangles as an OBJ inspection mesh; returns false on output failure.
    [[nodiscard]] bool WriteCandidateObj(const std::filesystem::path& path, const CandidateNavMesh& candidate);
}
