#pragma once

#include "core/world/types.h"

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace navmesh::reproducibility
{
    inline constexpr std::string_view kExportMetadataSchemaVersion = "1.0.0";
    inline constexpr std::string_view kToolVersion = "0.1.0";
    struct SourceCoverage { std::size_t references{}; std::size_t referencesWithModels{}; std::size_t modelsLoaded{}; std::size_t modelsMissing{}; std::size_t geometryVertices{}; std::size_t geometryTriangles{}; std::size_t terrainLandRecords{}; std::size_t terrainLandDecoded{}; std::size_t terrainLandMissing{}; bool terrainSupported{}; bool collisionGeometrySupported{}; };
    struct ExportMetadata { std::filesystem::path inputPlugin; const core::Cell* selectedCell{}; SourceCoverage coverage; std::vector<std::string> warnings; };
    [[nodiscard]] std::string EscapeJson(const std::string& value);
    [[nodiscard]] std::string ToJson(const ExportMetadata& metadata, std::string_view indent = "  ");
    [[nodiscard]] bool WriteSidecar(const std::filesystem::path& exportedPath, const ExportMetadata& metadata);
}
