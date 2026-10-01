#pragma once

#include "core/scene/scene.h"
#include "core/world/types.h"
#include "skyrim/parser/plugin_parser.h"

namespace navmesh::skyrim::offline
{
    inline constexpr std::size_t kLandHeightSamplesPerSide = 33;
    inline constexpr float kLandSampleSpacing = 128.0F;
    inline constexpr float kLandCellSize = 4096.0F;
    struct TerrainExtraction
    {
        core::Scene scene;
        core::Mesh mesh;
        std::size_t landRecordsFound{};
        std::size_t landRecordsDecoded{};
        std::size_t landRecordsMissing{};
        std::vector<std::string> warnings;
    };
    /// Decode exterior LAND VHGT into world-space terrain triangles.
    /// A valid VHGT has a float base height and 1088 signed delta samples;
    /// missing or malformed data produces warnings rather than substitute terrain.
    [[nodiscard]] TerrainExtraction ExtractTerrain(const ResolvedLoadOrder &loadOrder, const core::Cell &cell);
} // namespace navmesh::skyrim::offline
