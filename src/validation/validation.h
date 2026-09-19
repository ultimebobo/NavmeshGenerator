#pragma once

#include "core/world/types.h"

namespace navmesh::validation
{
    enum class Severity { warning, error };
    struct Finding { Severity severity; std::string message; };
    [[nodiscard]] std::vector<Finding> Validate(const core::Cell& cell);
}
