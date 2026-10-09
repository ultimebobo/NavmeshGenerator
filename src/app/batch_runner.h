#pragma once

#include "app/run.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/parser/plugin_parser.h"

namespace navmesh::app::detail
{
    /** Resolve an explicit Cell-mode list against a coherent winning load order.
     * @param resolved Snapshot owning the returned CELL pointers; keep it alive throughout processing.
     * @param selection Form IDs or editor IDs separated by the shared cell-list delimiters.
     * @return Unique cells in resolved CELL order; aliases deduplicate by Form ID.
     * @throws std::runtime_error When any identifier is missing or ambiguous. Performs no generation or export.
     */
    [[nodiscard]] std::vector<const core::Cell *> ResolveCellSelection(const skyrim::ResolvedLoadOrder &resolved,
                                                                       const std::string &selection);

    /** Rebuild affected or explicitly selected cells from an already resolved input snapshot.
     * @param options Validated batch options; output names the export directory.
     * @param resolved Winning records and ownership, retained throughout the run.
     * @param assets Optional MO2 model winners; null uses the configured Data directory.
     * @param paths Source plugin paths in resolved load-order order.
     * @param progress Optional synchronous progress sink.
     * @param cancelled Optional cancellation query between processing stages.
     * @param inputPreparationSeconds Elapsed input/report preparation time, in seconds, for checkpoint telemetry.
     * Explicit identifiers must all resolve uniquely; aliases deduplicate by CELL Form ID.
     * @return Process-style status: success (including logged cell-generation skips),
     * input/export/global-linking failure, or cancellation. Skips preserve authored geometry.
     * @warning Writes batch reports and candidate exports, and a verified ESP when requested.
     */
    int RunBatch(const Options &options, const skyrim::ResolvedLoadOrder &resolved,
                 const skyrim::ModelAssetSources *assets, const std::vector<std::filesystem::path> &paths,
                 const ProgressCallback &progress, const CancellationCallback &cancelled,
                 double inputPreparationSeconds = 0);
} // namespace navmesh::app::detail
