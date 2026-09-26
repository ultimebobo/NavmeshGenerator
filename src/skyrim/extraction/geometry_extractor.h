#pragma once

#include "core/geometry/types.h"
#include "core/scene/scene.h"
#include "core/world/types.h"
#include "core/reproducibility/export_metadata.h"

#include <filesystem>
#include <string>
#include <vector>

namespace navmesh::skyrim::offline
{
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
        bool terrainSupported{};
        std::size_t terrainLandRecords{};
        std::size_t terrainLandDecoded{};
        std::size_t terrainLandMissing{};
        bool collisionGeometrySupported{};
    };

    [[nodiscard]] GeometryExtraction ExtractGeometry(const std::filesystem::path& dataDirectory, const core::Cell& cell, const std::filesystem::path& cacheDirectory = {});
    [[nodiscard]] bool WriteGeometryObj(const std::filesystem::path& outputPath, const GeometryExtraction& geometry);
    [[nodiscard]] bool WriteGeometryJson(const std::filesystem::path& outputPath, const core::Cell& cell, const GeometryExtraction& geometry, const reproducibility::ExportMetadata& metadata);
}
