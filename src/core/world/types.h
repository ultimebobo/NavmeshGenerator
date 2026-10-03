#pragma once

#include "core/geometry/types.h"
#include "core/navmesh/types.h"

#include <optional>
#include <string>

namespace navmesh::core
{
    struct Reference
    {
        std::uint32_t id{};
        std::uint32_t baseObjectId{};
        std::string name;
        std::string recordType;
        std::string editorId;
        std::string modelPath;
        Vec3 position;
        Vec3 rotation;
        float scale{1.0F};
        /// Winning record origins from resolved load order assembly.
        std::string sourcePlugin;
        std::string basePlugin;
        std::string baseRecordType;
        /// True when the winning placed-record header marks this reference initially disabled.
        bool initiallyDisabled{};
        /// True when the winning placed-record header marks this reference deleted.
        bool deleted{};
        /// Winning placement has a teleport destination (XTEL), including non-physical cave exits.
        bool teleportExit{};
        std::optional<AABB> localBounds;
    };
    struct Cell
    {
        std::uint32_t id{};
        std::string editorId;
        std::string name;
        bool isInterior{};
        std::optional<std::array<std::int32_t, 2>> exteriorCoordinates;
        std::vector<Reference> references;
        std::vector<NavMesh> navMeshes;
        /// Effective exterior water surface Z in Skyrim world units, when supported and finite.
        /// Absent for dry cells, interiors, or unresolved/default water data.
        std::optional<float> waterHeight;
    };
} // namespace navmesh::core
