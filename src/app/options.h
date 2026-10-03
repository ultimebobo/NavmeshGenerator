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
        /// Detect generated water and preferred-route triangles; disable to export unclassified geometry.
        bool tagTriangles{true};
        /// Copy the affected plugin with generated NAVMs instead of writing a NAVM-only patch.
        /// Requires Plugin scope and generatePlugin; keeps the source filename and master indices.
        bool copyPlugin{};
        /// Skip generation for cells with any winning NAVM record, including empty or unsupported records.
        /// Requires resolved MO2/load-order input; uncovered cells may receive new NAVM records.
        bool skipExistingNavmesh{};
        /// Recast region strategy used for candidate generation; watershed is the default.
        core::RegionPartitioningAlgorithm partitioningAlgorithm{core::RegionPartitioningAlgorithm::Watershed};
        /// Agent dimensions and movement limits in Skyrim units; shared by all generation scopes.
        core::NavigationProfile navigationProfile;
        /// Recast voxel and contour controls; validated before input resolution and cache reuse.
        core::RecastSettings recastSettings;
        /// Exterior geometry/impact halo in CELL units; generation always includes
        /// adjacent geometry and remains clipped to each selected CELL.
        int neighboringCellRadius{};
        /// Batch artifacts: auto uses plugin_only for plugin writing/estimates and full for inspection.
        /// full emits JSON/OBJ; compact emits gzip JSON; plugin_only emits the plugin and run reports.
        std::string batchOutput{"auto"};
        /// Shared generated asset/candidate cache root; empty uses system temporary storage.
        std::filesystem::path assetCache;
        /// Disk retention budget for generated cache files, in MiB; excludes requested exports.
        std::size_t cacheBudgetMiB{2048};
        /// Combined model/placement cache and admitted generation work budget, in MiB.
        std::size_t workingMemoryMiB{512};
        /// Maximum independent Recast tasks; extraction and final serialization remain ordered.
        std::size_t workers{1};
        /// Sample eligible batch targets for a cost estimate, checkpoint/cache results, and finish before plugin writing.
        bool estimateOnly{};
        std::optional<std::array<float, 4>> sceneBounds;
        float surfaceSearchRadius{64.0F}, maxSupportDistance{32.0F}, maxSlope{45.0F};
    };

    /// Parse CLI options; `--generate-plugin` selects the format automatically and rejects a format argument.
    [[nodiscard]] Options ParseCommandLine(int argc, char **argv);
} // namespace navmesh::app
