#include "core/geometry/types.h"

#include <algorithm>

namespace navmesh::core
{
    bool AABB::IsValid() const noexcept { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    void AABB::Expand(const Vec3& point) noexcept {
        min.x = std::min(min.x, point.x); min.y = std::min(min.y, point.y); min.z = std::min(min.z, point.z);
        max.x = std::max(max.x, point.x); max.y = std::max(max.y, point.y); max.z = std::max(max.z, point.z);
    }
    AABB Mesh::Bounds() const noexcept { AABB bounds; for (const auto& vertex : vertices) bounds.Expand(vertex); return bounds; }
}
