#pragma once

#include "app/options.h"
#include <functional>
#include <string_view>

namespace navmesh::app
{
    /// Progress and cancellation callbacks shared by the CLI and Windows UI.
    using ProgressCallback = std::function<void(int percent, std::string_view status)>;
    using CancellationCallback = std::function<bool()>;
    /// Execute an offline analysis/export run with the selected options.
    /// @return Process-style status code; nonzero indicates failure or cancellation.
    int Run(const Options& options, const ProgressCallback& progress = {}, const CancellationCallback& cancelled = {});
}
