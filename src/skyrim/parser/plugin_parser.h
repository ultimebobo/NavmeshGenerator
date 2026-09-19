#pragma once

#include "core/world/types.h"

#include <filesystem>
#include <optional>
#include <string>

namespace navmesh::skyrim::offline
{
    [[nodiscard]] std::optional<core::Cell> LoadCell(
        const std::filesystem::path& pluginPath,
        const std::string& targetCell = {},
        const std::string& targetWorldspace = {},
        const std::optional<std::int32_t>& cellX = std::nullopt,
        const std::optional<std::int32_t>& cellY = std::nullopt);
}
