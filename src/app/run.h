#pragma once

#include "app/options.h"

#include <functional>
#include <string_view>

namespace navmesh::app
{
    /// Synchronous progress/status sink shared by the CLI and Windows UI; failed runs publish their terminal reason.
    using ProgressCallback = std::function<void(int percent, std::string_view status)>;
    /// Optional synchronous cancellation query between processing stages.
    using CancellationCallback = std::function<bool()>;
    /// Execute a cell analysis or an affected-cell plugin/load-order rebuild.
    /// Cell mode accepts one or multiple explicit cells; inputs resolve once per run.
    /// Coordinated generation exports a report and per-cell candidates;
    /// generatePlugin writes one verified plugin after all candidates are ready;
    /// copyPlugin preserves its named source's other records in Cell and Plugin scopes.
    /// makeScene writes one Cell or size-limited Plugin-selection GLB with finalized candidates independently
    /// of the batch output policy; Plugin selections must fit MaximumPluginSceneCells before skips.
    /// Failures publish their cause and recovery guidance through progress as well as stderr.
    /// Processing exceptions are reported with their stage; cancellation remains a separate status.
    /// @return Process-style status code; nonzero indicates failure or cancellation.
    int Run(const Options &options, const ProgressCallback &progress = {}, const CancellationCallback &cancelled = {});
} // namespace navmesh::app
