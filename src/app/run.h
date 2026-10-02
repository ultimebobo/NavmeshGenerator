#pragma once

#include "app/options.h"

#include <functional>
#include <string_view>

namespace navmesh::app
{
    /// Progress and cancellation callbacks shared by the CLI and Windows UI.
    using ProgressCallback = std::function<void(int percent, std::string_view status)>;
    using CancellationCallback = std::function<bool()>;
    /// Execute a cell analysis or an affected-cell plugin/load-order rebuild.
    /// Batch scopes resolve inputs once and export a report and per-cell candidates;
    /// generatePlugin writes one verified plugin after all candidates are ready;
    /// copyPlugin preserves the affected plugin's other records in Plugin scope.
    /// @return Process-style status code; nonzero indicates failure or cancellation.
    int Run(const Options &options, const ProgressCallback &progress = {}, const CancellationCallback &cancelled = {});
} // namespace navmesh::app
