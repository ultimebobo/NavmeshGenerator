#pragma once

#include "core/geometry/types.h"
#include "core/navmesh/types.h"

#include <optional>
#include <string>

namespace navmesh::core
{
    struct Reference {
        std::uint32_t id{}; std::uint32_t baseObjectId{}; std::string name; std::string recordType; std::string editorId; std::string modelPath; Vec3 position; Vec3 rotation; float scale{ 1.0F };
        std::optional<AABB> localBounds;
    };
    struct Cell {
        std::uint32_t id{}; std::string editorId; std::string name; bool isInterior{};
        std::optional<std::array<std::int32_t, 2>> exteriorCoordinates;
        std::vector<Reference> references; std::vector<NavMesh> navMeshes;
    };
}
