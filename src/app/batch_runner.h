#pragma once

#include "app/run.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/parser/plugin_parser.h"

namespace navmesh::app::detail
{
    /** Rebuild affected cells from an already resolved input snapshot.
     * @param options Validated batch options; output names the export directory.
     * @param resolved Winning records and ownership, retained throughout the run.
     * @param assets Optional MO2 model winners; null uses the configured Data directory.
     * @param paths Source plugin paths in resolved load-order order.
     * @param progress Optional synchronous progress sink.
     * @param cancelled Optional cancellation query between processing stages.
     * @param inputPreparationSeconds Elapsed input/report preparation time, in seconds, for checkpoint telemetry.
     * @return Process-style status: success (including logged cell-generation skips),
     * input/export/global-linking failure, or cancellation. Skips preserve authored geometry.
     * @warning Writes batch reports and candidate exports, and a verified ESP when requested.
     */
    int RunBatch(const Options &options, const skyrim::ResolvedLoadOrder &resolved,
                 const skyrim::ModelAssetSources *assets, const std::vector<std::filesystem::path> &paths,
                 const ProgressCallback &progress, const CancellationCallback &cancelled,
                 double inputPreparationSeconds = 0);
} // namespace navmesh::app::detail
