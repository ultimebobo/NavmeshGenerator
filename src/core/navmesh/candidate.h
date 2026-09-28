#pragma once

#include "core/navmesh/types.h"
#include "core/scene/scene.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace navmesh::core
{
    /// Versioned movement and surface constraints used to build a candidate navmesh.
    /// Distances and areas use Skyrim world units; angles use degrees.
    struct NavigationProfile
    {
        std::string name{ "human" };
        std::string version{ "1.0.0" };
        float agentRadius{ 16.0F };
        float agentHeight{ 128.0F };
        float maxSlopeDegrees{ 45.0F };
        float stepHeight{ 18.0F };
        float clearance{ 128.0F };
        float weldTolerance{ 0.05F };
        float minimumRegionArea{ 64.0F };
        float contourSimplificationTolerance{ 0.05F };
        std::string cellBorderPolicy{ "preserve_open_border" };
    };

    /// Find a built-in profile by its `name@version` key.
    /// @return The profile, or no value for an unknown name or version.
    [[nodiscard]] std::optional<NavigationProfile> FindNavigationProfile(const std::string& key);

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
    struct CandidateStatistics
    {
        std::size_t inputTriangles{}, eligibleTriangles{}, rejectedSlope{}, rejectedClearance{}, rejectedObstruction{}, rejectedSource{},
            rejectedDegenerate{}, rejectedSmallRegion{}, rejectedUnreachable{}, outputPolygons{};
    };
    /// Inspection-only result; this does not encode or replace a plugin NAVM record.
    struct CandidateNavMesh
    {
        NavigationProfile profile;
        NavMesh mesh;
        std::vector<std::size_t> polygonSourceTriangles;
        std::vector<CandidateRegion> regions;
        std::vector<CandidateExit> exits;
        std::vector<CandidateContour> contours;
        CandidateTopology topology;
        CandidateStatistics statistics;
        std::vector<std::string> warnings;
    };

    /// Build a candidate from supported terrain and collision triangles in a scene.
    /// @param scene Geometry and source evidence to inspect.
    /// @param profile Versioned movement and surface constraints.
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
