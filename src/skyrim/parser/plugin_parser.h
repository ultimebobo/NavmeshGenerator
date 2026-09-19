#pragma once

#include "core/world/types.h"

#include <filesystem>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace navmesh::skyrim::offline
{
    [[nodiscard]] std::vector<core::Cell> ListCells(
        const std::filesystem::path& pluginPath);

    [[nodiscard]] std::optional<core::Cell> LoadCell(
        const std::filesystem::path& pluginPath,
        const std::string& targetCell = {},
        const std::string& targetWorldspace = {},
        const std::optional<std::int32_t>& cellX = std::nullopt,
        const std::optional<std::int32_t>& cellY = std::nullopt,
        const std::optional<std::uint32_t>& cellFormId = std::nullopt,
        const std::string& targetEditorId = {});
}
