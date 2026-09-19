#pragma once

#include "core/world/types.h"
#include "validation/validation.h"

namespace navmesh::cli { [[nodiscard]] std::string ToJson(const core::Cell& cell, const std::vector<validation::Finding>& findings); }
