#pragma once

#include "core/world/types.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace navmesh::analysis
{
    struct NavMeshAnalysis
    {
        std::uint32_t vertexCount{};
        std::uint32_t polygonCount{};
        std::uint32_t connectedComponents{};
        std::uint32_t isolatedPolygonCount{};
        std::uint32_t degeneratePolygonCount{};
        core::AABB boundingBox{};
        double minPolygonArea{};
        double maxPolygonArea{};
        double averagePolygonArea{};
    };

    struct SurfaceHit
    {
        core::Vec3 point{};
        core::Vec3 normal{};
        float distance{};
        std::size_t triangleIndex{};
    };

    struct SpatialIndex
    {
        struct Entry
        {
            core::AABB bounds{};
            core::Triangle triangle{};
            std::size_t triangleIndex{};
        };

        std::vector<Entry> entries;
        std::vector<core::Vec3> vertices;

        struct BvhNode { core::AABB bounds{}; std::size_t first{}; std::size_t count{}; std::size_t left{}; std::size_t right{}; bool leaf{}; };
        std::vector<BvhNode> nodes;
        std::vector<std::size_t> orderedEntries;

        void Build(const std::vector<core::Triangle>& triangles, const std::vector<core::Vec3>& vertices);
        [[nodiscard]] std::vector<std::size_t> QueryAABB(const core::AABB& bounds) const;
        [[nodiscard]] std::optional<SurfaceHit> NearestSurface(const core::Vec3& point, float maxDistance = 1000.0F) const;
        [[nodiscard]] std::optional<SurfaceHit> Raycast(const core::Vec3& origin, const core::Vec3& direction, float maxDistance = 1000.0F) const;
        [[nodiscard]] std::size_t Size() const noexcept;
    };

    struct GeometrySummary
    {
        std::size_t vertexCount{};
        std::size_t triangleCount{};
        core::AABB bounds{};
        core::Vec3 center{};
        core::Vec3 extent{};
    };

    struct NavMeshSummary
    {
        std::size_t vertexCount{};
        std::size_t polygonCount{};
        core::AABB bounds{};
        core::Vec3 center{};
        core::Vec3 extent{};
    };

    struct AnalysisConfiguration
    {
        float surfaceSearchRadius{};
        float maxSupportDistance{};
        float maxSlope{};
        float minimumCoverage{ 0.60F };
        float obstructionClearance{ 64.0F };
        float ambiguityHeightDelta{ 8.0F };
        // Set for exterior cells.  It allows topology inspection to identify
        // unstitched edges on the selected cell border without assuming a
        // border policy for interiors.
        std::optional<core::AABB> cellBounds;
    };

    enum class SupportSourceType { Unknown, RenderFallback, Terrain, Collision };

    struct TriangleSource
    {
        SupportSourceType type{ SupportSourceType::Unknown };
        float confidence{ 0.0F };
        std::string id;
    };

    struct AnalysisSummary
    {
        std::size_t polygonsAnalyzed{};
        std::size_t supportFound{};
        std::size_t unsupported{};
        std::size_t supported{};
        std::size_t floating{};
        std::size_t buried{};
        std::size_t tooSteep{};
        std::size_t blocked{};
        std::size_t outOfCoverage{};
        std::size_t ambiguous{};
        std::size_t repairCandidates{};
        std::size_t topologyFindings{};
    };

    struct SummaryStats
    {
        double min{};
        double max{};
        double mean{};
        double median{};
        double p95{};
    };

    struct PolygonSupport
    {
        bool found{};
        core::Vec3 point{};
        float distance{};
        float heightDelta{};
        core::Vec3 normal{};
        float slopeDegrees{};
        std::size_t triangleIndex{};
        std::string sourceNifPath;
        std::optional<std::size_t> sourceTriangleIndex;
        std::string sourceType{ "unknown" };
        std::string collisionType;
        float sourceConfidence{};
        float confidence{};
        std::size_t samplesTotal{};
        std::size_t samplesCovered{};
        float sampleAgreement{};
    };

    struct PolygonAnalysisResult
    {
        std::uint32_t index{};
        core::Vec3 centroid{};
        core::Vec3 normal{};
        core::AABB bounds{};
        std::uint32_t vertexCount{};
        PolygonSupport support{};
        std::string classification;
    };

    struct TopologyFinding
    {
        std::string kind;
        std::vector<std::uint32_t> polygons;
        float confidence{};
        std::string evidence;
    };

    // Candidates are intentionally advisory.  No structure in this module
    // represents a changed NAVM record or a replacement polygon.
    struct RepairCandidate
    {
        std::string id;
        std::string kind;
        std::vector<std::uint32_t> polygons;
        float confidence{};
        std::string disposition{ "manual_review" };
        std::string evidence;
    };

    struct AnalysisReport
    {
        AnalysisConfiguration configuration{};
        AnalysisSummary summary{};
        SummaryStats heightDeltaStats{};
        SummaryStats slopeStats{};
        std::vector<PolygonAnalysisResult> polygons;
        std::vector<TopologyFinding> topology;
        std::vector<RepairCandidate> repairCandidates;
    };

    [[nodiscard]] float SurfaceSlopeDegrees(const core::Vec3& normal);
    [[nodiscard]] std::string ClassifySupport(float heightDelta, float slopeDegrees, float maxSupportDistance, float maxSlope);
    [[nodiscard]] AnalysisReport AnalyzeNavMeshPolygons(const core::NavMesh& mesh, const core::Mesh& geometry, const AnalysisConfiguration& configuration);
    [[nodiscard]] AnalysisReport AnalyzeNavMeshPolygons(const core::NavMesh& mesh, const core::Mesh& geometry, const std::vector<TriangleSource>& sources, const AnalysisConfiguration& configuration);
    [[nodiscard]] const char* SupportSourceName(SupportSourceType type);

    [[nodiscard]] GeometrySummary AnalyzeGeometry(const core::Mesh& mesh);
    [[nodiscard]] NavMeshSummary AnalyzeNavMesh(const core::NavMesh& mesh);

    [[nodiscard]] NavMeshAnalysis Analyze(const core::NavMesh& mesh);
}
