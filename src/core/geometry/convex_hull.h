#pragma once

#include "core/geometry/types.h"

#include <optional>
#include <span>

namespace navmesh::core
{
    /** Triangulate the closed convex hull of finite points in a common coordinate space.
     * @param vertices Points in consistent units; duplicates and interior points are permitted.
     * @return Outward-facing triangles indexing the input, or no value for nonfinite,
     * degenerate or numerically unrepresentable hulls. Exact orientation signs preserve
     * closed, consistently wound boundaries at nearly coplanar samples. Input positions are never moved.
     */
    [[nodiscard]] std::optional<std::vector<Triangle>> TriangulateConvexHull(std::span<const Vec3> vertices);
} // namespace navmesh::core
