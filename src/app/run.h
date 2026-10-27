#pragma once

#include "app/options.h"
#include "app/run_progress.h"

#include <functional>
#include <string_view>

namespace navmesh::app
{
    /// Optional synchronous cancellation query between processing stages.
    using CancellationCallback = std::function<bool()>;
    /// Execute a cell analysis, affected-cell rebuild, or structural navigation repair of the winning load order.
    /// Cell mode accepts one or multiple explicit cells; inputs resolve once per run.
    /// Coordinated generation exports a report and per-cell candidates;
    /// generatePlugin writes one verified plugin after all candidates are ready;
    /// copyPlugin preserves its named source's other records in Cell and Plugin scopes.
    /// makeScene writes one Cell or size-limited Plugin-selection GLB with finalized candidates independently
    /// of the batch output policy; Plugin selections must fit MaximumPluginSceneCells before skips.
    /// Failures publish their cause and recovery guidance through progress as well as stderr.
    /// Processing exceptions are reported with their stage; cancellation remains a separate status.
    /// Appends flushed activity to output/run.log; log failures warn without changing the processing result.
    /// Failed runs also save bounded, pasteable diagnostics and settings to output/bug-report.txt when writable.
    /// Status and optional structured events include idle heartbeats, serialized across processing and timer threads.
    /// Callbacks must not throw or reenter Run; no callbacks remain active after return.
    /// @return Process-style status code; nonzero indicates failure or cancellation.
    int Run(const Options &options, const ProgressCallback &progress = {}, const CancellationCallback &cancelled = {},
            const RunEventCallback &events = {});
} // namespace navmesh::app
