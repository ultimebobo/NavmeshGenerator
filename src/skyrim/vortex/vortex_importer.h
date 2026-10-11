#pragma once

#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/parser/plugin_parser.h"

namespace navmesh::skyrim
{
    /// Read-only snapshot of Vortex's active deployed game files, without staging-folder provenance.
    struct VortexInput
    {
        /// Absolute Skyrim installation and deployed Data directories.
        std::filesystem::path gameRoot, gameData;
        /// Active game application directory containing plugins.txt, independent of Vortex's saved profiles.
        std::filesystem::path pluginsDirectory;
        /// Deployed plugins in game load order, including installed native masters and Skyrim.ccc entries.
        std::vector<std::filesystem::path> pluginPaths;
        /// Deployed loose models and active BSAs, with archives in increasing plugin priority.
        ModelAssetSources assets;
        /// Hash of plugin-list contents and deployed file membership, size and modification times.
        std::string snapshotHash;
        /// Unusable input diagnostics; pluginPaths is empty when import fails.
        std::vector<Diagnostic> diagnostics;
    };

    /** Resolve the currently deployed Vortex profile without changing game or manager files.
     * @param gameRoot Skyrim installation folder containing Data, not Vortex's staging folder.
     * @param pluginsDirectory Folder containing the active game's plugins.txt; empty uses standard Skyrim SE local app data.
     * plugins.txt activation markers and order are authoritative; installed native masters and Skyrim.ccc load first.
     * Only base-game and active-plugin BSAs are used. Missing active files, duplicate entries and invalid names fail import.
     * @return Snapshot and diagnostics; no plugin paths on invalid input. Assets expose deployed winners only.
     * @warning Activate the intended Vortex profile and deploy before importing; inactive saved profiles are not selectable.
     */
    [[nodiscard]] VortexInput ImportVortexDeployment(const std::filesystem::path &gameRoot,
                                                     const std::filesystem::path &pluginsDirectory = {});

    /// Recheck plugin-list contents and deployed file stamps; false on changes or unreadable inputs.
    [[nodiscard]] bool VortexSnapshotMatches(const VortexInput &input);

    /** Write the Vortex input snapshot and limitations as JSON, truncating only the supplied report path.
     * @param outputPath Generated report destination, outside the game input directories.
     * @param input Imported deployment, including failures when no usable plugins were found.
     * @param includeAssetWinners Include the loose-model table for inspection; counts and archives are always written.
     * @return False if the destination cannot be written completely.
     */
    [[nodiscard]] bool WriteInputReport(const std::filesystem::path &outputPath, const VortexInput &input,
                                        bool includeAssetWinners = true);
} // namespace navmesh::skyrim
