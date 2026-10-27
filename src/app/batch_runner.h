#pragma once

#include "app/run.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/parser/plugin_parser.h"
#include "skyrim/parser/base_object_swapper.h"

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

    /** Rebuild affected, explicitly selected, or structurally faulty cells from an already resolved input snapshot.
     * @param options Validated batch options; output names the export directory.
     * @param resolved Winning records and ownership, retained throughout the run.
     * @param assets Optional mod-manager model winners; null uses the configured Data directory.
     * @param paths Source plugin paths in resolved load-order order.
     * @param progressSink Optional synchronous progress sink; fatal failures publish the cause and diagnostic output path.
     * @param cancelled Optional cancellation query between processing stages.
     * @param inputPreparationSeconds Elapsed input/report preparation time, in seconds, for checkpoint telemetry.
     * @param bosInputs Optional runtime snapshot; revalidated before publishing a generated plugin.
     * @param events Optional activity sink for completion counts and diagnostics, on the orchestration thread.
     * Explicit identifiers must all resolve uniquely; aliases deduplicate by CELL Form ID.
     * Repair scope audits winning navigation without collision-impact selection and reports its selection evidence.
     * @return Process-style status: success (including logged cell-generation skips),
     * input/export/global-linking failure, or cancellation. Skips preserve authored geometry.
     * @warning Writes batch reports and candidate exports, and a verified ESP when requested.
     */
    int RunBatch(const Options &options, const skyrim::ResolvedLoadOrder &resolved,
                 const skyrim::ModelAssetSources *assets, const std::vector<std::filesystem::path> &paths,
                 const ProgressCallback &progressSink, const CancellationCallback &cancelled,
                 double inputPreparationSeconds = 0, const skyrim::BaseObjectSwapInput *bosInputs = nullptr,
                 const RunEventCallback &events = {});
} // namespace navmesh::app::detail
