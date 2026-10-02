#pragma once

#include "skyrim/extraction/geometry_extractor.h"

#include <filesystem>
#include <set>
#include <string>

namespace navmesh::skyrim::offline
{
    /** Resolve a shared extracted-asset snapshot without modifying game inputs.
     * @param dataDirectory Game Data root; its archive set is used without MO2 assets.
     * @param assets Optional ordered MO2 archive providers.
     * @param cacheRoot Optional owned-cache root; empty uses the system temporary directory.
     * @return Snapshot directory keyed by provider paths, sizes, timestamps, priority, and schema.
     */
    [[nodiscard]] std::filesystem::path ModelAssetCacheDirectory(const std::filesystem::path &dataDirectory,
                                                                 const ModelAssetSources *assets,
                                                                 const std::filesystem::path &cacheRoot = {});

    /** Index winning NIF names supplied by changed archives, excluding loose winners.
     * @param dataDirectory Game Data root passed to the read-only archive helper.
     * @param assets Ordered archive providers and winning loose model paths.
     * @param snapshot Owned shared asset-cache snapshot directory.
     * @param changedArchives Archive paths belonging to the requested rebuild scope.
     * @param models Receives normalized logical model paths only on complete index success.
     * @return False on helper/index failure; callers must retain conservative archive impact.
     */
    [[nodiscard]] bool ChangedArchiveModels(const std::filesystem::path &dataDirectory, const ModelAssetSources &assets,
                                            const std::filesystem::path &snapshot,
                                            const std::set<std::filesystem::path> &changedArchives,
                                            std::set<std::string> &models);

    /// Evict only generated cache files to the byte budget after extraction; input/export files are excluded.
    /// protectCandidates pins in-flight audit data until global reconciliation/export has completed.
    /// Active pins and small ownership markers may temporarily exceed the retention budget.
    void TrimModelAssetCache(const std::filesystem::path &snapshot, std::size_t byteBudget,
                             bool protectCandidates = false);

    /// Invoke the project-owned indexed BSA helper; arguments must already be shell-quoted.
    [[nodiscard]] bool RunAssetHelper(const std::string &arguments);

    /// Quote a filesystem path for the platform shell used by the asset helper.
    [[nodiscard]] std::string QuoteAssetPath(const std::filesystem::path &path);
} // namespace navmesh::skyrim::offline
