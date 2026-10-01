#pragma once

#include "core/world/types.h"
#include "core/reproducibility/export_metadata.h"
#include "validation/validation.h"

namespace navmesh::cli
{
    [[nodiscard]] std::string ToJson(const core::Cell &cell, const std::vector<validation::Finding> &findings,
                                     const reproducibility::ExportMetadata &metadata);
    [[nodiscard]] std::string ToJson(const core::Cell &cell, const std::vector<validation::Finding> &findings);
} // namespace navmesh::cli
