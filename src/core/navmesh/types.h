#pragma once

#include "core/geometry/types.h"

namespace navmesh::core
{
    struct NavPolygon { std::array<std::uint32_t, 3> vertices{}; std::array<std::uint32_t, 3> neighbors{}; std::uint16_t flags{}; };
    struct NavMesh { std::uint32_t id{}; std::vector<Vec3> vertices; std::vector<NavPolygon> polygons; };
}
