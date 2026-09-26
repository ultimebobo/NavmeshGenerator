#pragma once

#include "app/options.h"
#include <functional>
#include <string_view>

namespace navmesh::app
{
    // The shared application service used by both the command-line and Windows UI adapters.
    using ProgressCallback = std::function<void(int percent, std::string_view status)>;
    using CancellationCallback = std::function<bool()>;
    int Run(const Options& options, const ProgressCallback& progress = {}, const CancellationCallback& cancelled = {});
}
