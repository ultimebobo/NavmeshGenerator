#pragma once

#include "core/world/types.h"

namespace RE { class TESObjectCELL; }
namespace navmesh::skyrim { [[nodiscard]] core::Cell ExtractCell(const RE::TESObjectCELL& cell); }
