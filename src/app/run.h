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
    /// Cell mode accepts one or multiple explicit cells; inputs resolve once per run.
    /// Coordinated generation exports a report and per-cell candidates;
    /// generatePlugin writes one verified plugin after all candidates are ready;
    /// copyPlugin preserves its named source's other records in Cell and Plugin scopes.
    /// makeScene writes one Cell-selection GLB with finalized candidates independently of the batch output policy.
    /// @return Process-style status code; nonzero indicates failure or cancellation.
    int Run(const Options &options, const ProgressCallback &progress = {}, const CancellationCallback &cancelled = {});
} // namespace navmesh::app
