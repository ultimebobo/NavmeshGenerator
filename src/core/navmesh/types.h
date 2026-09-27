#pragma once

#include "core/geometry/types.h"

namespace navmesh::core
{
    /// A triangular navigation polygon and its source neighbor/flag data.
    struct NavPolygon { std::array<std::uint32_t, 3> vertices{}; std::array<std::uint32_t, 3> neighbors{}; std::uint16_t flags{}; };
    /// Neutral navmesh model with vertices in Skyrim world coordinates.
    struct NavMesh { std::uint32_t id{}; std::vector<Vec3> vertices; std::vector<NavPolygon> polygons; };
}
