#pragma once

#include "core/geometry/types.h"

namespace navmesh::core
{
    /// A triangular navigation polygon and its source neighbor/flag data.
    struct NavPolygon
    {
        std::array<std::uint32_t, 3> vertices{};
        std::array<std::uint32_t, 3> neighbors{};
        std::uint16_t flags{};
    };
    /// Authored portal consumed by an edge of the owning mesh; indices start at zero.
    struct NavmeshExternalLink
    {
        std::uint32_t polygon{};
        std::uint8_t edge{};
        /// Destination NAVM identity in the same FormID space as the owning mesh.
        std::uint32_t navmeshId{};
        std::uint32_t targetPolygon{};
    };
    /// Authored triangle associated with a placed exit reference, including cave entrances.
    struct NavmeshDoorLink
    {
        std::uint32_t polygon{};
        /// Placed reference identity in the same FormID space as the owning mesh.
        std::uint32_t referenceId{};
    };
    /// Neutral navmesh model with vertices in Skyrim world coordinates.
    struct NavMesh
    {
        std::uint32_t id{};
        std::vector<Vec3> vertices;
        std::vector<NavPolygon> polygons;
        /// Decoded authored connections for inspection; generated connections live in CandidateNavMesh.
        std::vector<NavmeshExternalLink> externalLinks;
        std::vector<NavmeshDoorLink> doorLinks;
    };
} // namespace navmesh::core
