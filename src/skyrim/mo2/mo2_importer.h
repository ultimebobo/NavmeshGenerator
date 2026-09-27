#pragma once

#include "skyrim/parser/plugin_parser.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace navmesh::skyrim::offline
{
    struct VirtualFile { std::string logicalPath; std::filesystem::path physicalPath; std::string source; };
    struct EnabledMod { std::string name; std::filesystem::path path; std::size_t priority{}; };
    struct Mo2ProfileInput {
        std::filesystem::path instanceRoot; std::string profile;
        std::filesystem::path gameData; std::filesystem::path profileDirectory; std::filesystem::path modsDirectory;
        std::vector<EnabledMod> enabledMods; std::vector<std::filesystem::path> pluginPaths;
        std::vector<VirtualFile> looseAssetWinners;
        /// Physical winning BSAs from game Data and enabled mods, in increasing MO2 priority.
        std::vector<std::filesystem::path> archivePaths;
        std::vector<Diagnostic> diagnostics;
        std::string snapshotHash; bool looseAssetCacheUsed{};
    };

    // A mods-directory override is a read-only recovery option for an MO2 profile
    // whose configured storage was moved. It never changes MO2 configuration.
    [[nodiscard]] Mo2ProfileInput ImportMo2Profile(const std::filesystem::path& instanceOrPortableRoot, const std::string& profile,
        const std::optional<std::filesystem::path>& modsDirectoryOverride = std::nullopt,
        const std::filesystem::path& cacheDirectory = {});
    [[nodiscard]] bool ProfileSnapshotMatches(const Mo2ProfileInput& input);
    [[nodiscard]] bool WriteInputReport(const std::filesystem::path& outputPath, const Mo2ProfileInput& input);
    [[nodiscard]] std::string ToJson(const Mo2ProfileInput& input);
}
