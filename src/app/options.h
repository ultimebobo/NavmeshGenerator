#pragma once

#include <cstdint>
#include <array>
#include <filesystem>
#include <optional>
#include <string>

namespace navmesh::app
{
    struct Options
    {
        std::filesystem::path plugin, data, loadOrder, mo2, modsDirectory, output{"."}, exportGeometry, exportAnalysis, exportScene;
        std::string profile, cell, editorId, worldspace, geometryLayers{"navmesh,terrain,collision,render,diagnostics"}, outputDetail{"full"}, navigationProfile{"human@1.0.0"};
        std::optional<std::int32_t> cellX, cellY;
        std::optional<std::uint32_t> cellFormId;
        bool listCells{}, diagnostics{}, terrainOnly{}, generateCandidate{};
        int neighboringCellRadius{};
        std::optional<std::array<float, 4>> sceneBounds;
        float surfaceSearchRadius{64.0F}, maxSupportDistance{32.0F}, maxSlope{45.0F};
    };

    [[nodiscard]] Options ParseCommandLine(int argc, char** argv);
}
