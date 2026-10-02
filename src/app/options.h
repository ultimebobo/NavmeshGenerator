#pragma once

#include "core/navmesh/generator.h"

#include <cstdint>
#include <array>
#include <filesystem>
#include <optional>
#include <string>

namespace navmesh::app
{
    /// Selection of navigation targets within a resolved load order.
    enum class RebuildScope
    {
        Cell,
        Plugin,
        LoadOrder
    };

    /// Shared CLI/desktop inputs for cell analysis and affected-cell rebuild runs.
    /// Shared CLI/desktop inputs for cell analysis and affected-cell rebuild runs.
    struct Options
    {
        std::filesystem::path plugin, data, loadOrder, mo2, modsDirectory, output{"."}, exportGeometry, exportAnalysis,
            exportScene;
        std::string profile, cell, editorId, worldspace, geometryLayers{"navmesh,terrain,collision,render,diagnostics"},
            outputDetail{"full"};
        std::optional<std::int32_t> cellX, cellY;
        std::optional<std::uint32_t> cellFormId;
        bool listCells{}, diagnostics{}, terrainOnly{}, generateCandidate{};
        /// Cells to rebuild; batch scopes require resolved MO2 or manifest input.
        RebuildScope rebuildScope{RebuildScope::Cell};
        /// Active plugin filename whose edits select affected cells in Plugin scope.
        std::string affectedPlugin;
        /// Write generated navigation as NAVM overrides or new records in uncovered cells.
        bool generatePlugin{};
        /// Skip generation for cells with any winning NAVM record, including empty or unsupported records.
        /// Requires resolved MO2/load-order input; uncovered cells may receive new NAVM records.
        bool skipExistingNavmesh{};
        /// Recast region strategy used for candidate generation; watershed is the default.
        core::RegionPartitioningAlgorithm partitioningAlgorithm{core::RegionPartitioningAlgorithm::Watershed};
        /// Exterior geometry/impact halo in CELL units; generation always includes
        /// adjacent geometry and remains clipped to each selected CELL.
        /// Exterior geometry/impact halo in CELL units; generation always includes
        /// adjacent geometry and remains clipped to each selected CELL.
        int neighboringCellRadius{};
        std::optional<std::array<float, 4>> sceneBounds;
        float surfaceSearchRadius{64.0F}, maxSupportDistance{32.0F}, maxSlope{45.0F};
    };

    /// Parse CLI options; `--generate-plugin` selects the format automatically and rejects a format argument.
    [[nodiscard]] Options ParseCommandLine(int argc, char **argv);
} // namespace navmesh::app
