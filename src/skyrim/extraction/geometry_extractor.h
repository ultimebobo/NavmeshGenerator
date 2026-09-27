#pragma once

#include "core/geometry/types.h"
#include "core/scene/scene.h"
#include "core/world/types.h"
#include "core/reproducibility/export_metadata.h"

#include <filesystem>
#include <functional>
#include <unordered_map>
#include <string>
#include <vector>

namespace navmesh::skyrim::offline
{
    using GeometryProgressCallback = std::function<void(std::size_t completedReferences, std::size_t totalReferences)>;
    using GeometryCancellationCallback = std::function<bool()>;
    struct GeometryReferenceReport
    {
        std::uint32_t formId{};
        std::uint32_t baseFormId{};
        std::string recordType;
        std::string modelPath;
        core::Vec3 position;
        core::Vec3 rotation;
        float scale{ 1.0F };
        std::size_t vertices{};
        std::size_t triangles{};
        std::size_t invalidIndices{};
        std::size_t degenerateTriangles{};
        std::size_t meshVertexOffset{};
        std::size_t meshTriangleOffset{};
        std::vector<std::string> shapes;
        std::string nifVersion;
        std::string sourceType;
        std::string collisionType;
        bool usedRenderFallback{};
        std::string failure;
    };

    struct GeometryExtraction
    {
        core::Scene scene;
        core::Mesh mesh;
        std::vector<GeometryReferenceReport> references;
        std::size_t referencesWithModels{};
        std::size_t modelsLoaded{};
        std::size_t modelsMissing{};
        std::size_t invalidVertices{};
        std::size_t invalidIndices{};
        std::size_t modelsExcluded{};
        std::size_t modelsUnreadable{};
        std::size_t modelsUnsupported{};
        std::size_t collisionModelsLoaded{};
        std::size_t collisionTriangles{};
        std::size_t renderFallbackModels{};
        std::size_t renderFallbackTriangles{};
        bool terrainSupported{};
        std::size_t terrainLandRecords{};
        std::size_t terrainLandDecoded{};
        std::size_t terrainLandMissing{};
        bool collisionGeometrySupported{};
    };

    /// Physical MO2 asset winners and archives in increasing mod priority.
    /// Logical paths use lowercase forward slashes and begin with meshes/.
    struct ModelAssetSources
    {
        std::unordered_map<std::string, std::filesystem::path> looseModels;
        std::vector<std::filesystem::path> archives;
    };

    /// Extract placed model geometry using MO2 assets when provided.
    /// @param dataDirectory Game Data directory for direct input and vanilla BSAs.
    /// @param cell Cell whose references are decoded in world coordinates.
    /// @param cacheDirectory Writable cache for requested archived NIFs.
    /// @param progress Optional per-reference progress callback.
    /// @param cancelled Optional cancellation callback; returns partial geometry if true.
    /// @param assets Optional MO2 loose winners and archive paths.
    /// @return Extracted support and render geometry with per-reference failures.
    [[nodiscard]] GeometryExtraction ExtractGeometry(const std::filesystem::path& dataDirectory, const core::Cell& cell, const std::filesystem::path& cacheDirectory = {}, const GeometryProgressCallback& progress = {}, const GeometryCancellationCallback& cancelled = {}, const ModelAssetSources* assets = nullptr);
    [[nodiscard]] bool WriteGeometryObj(const std::filesystem::path& outputPath, const GeometryExtraction& geometry);
    [[nodiscard]] bool WriteGeometryJson(const std::filesystem::path& outputPath, const core::Cell& cell, const GeometryExtraction& geometry, const reproducibility::ExportMetadata& metadata);
}
