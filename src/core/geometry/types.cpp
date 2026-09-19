#include "core/geometry/types.h"

#include <algorithm>

namespace navmesh::core
{
    bool AABB::IsValid() const noexcept { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    void AABB::Expand(const Vec3& point) noexcept {
        min.x = std::min(min.x, point.x); min.y = std::min(min.y, point.y); min.z = std::min(min.z, point.z);
        max.x = std::max(max.x, point.x); max.y = std::max(max.y, point.y); max.z = std::max(max.z, point.z);
    }
    Vec3 AABB::Center() const noexcept {
        return { (min.x + max.x) * 0.5F, (min.y + max.y) * 0.5F, (min.z + max.z) * 0.5F };
    }
    Vec3 AABB::Extent() const noexcept {
        return { max.x - min.x, max.y - min.y, max.z - min.z };
    }
    bool AABB::Contains(const Vec3& point) const noexcept {
        return point.x >= min.x && point.x <= max.x && point.y >= min.y && point.y <= max.y && point.z >= min.z && point.z <= max.z;
    }
    bool AABB::Intersects(const AABB& other) const noexcept {
        return max.x >= other.min.x && min.x <= other.max.x && max.y >= other.min.y && min.y <= other.max.y && max.z >= other.min.z && min.z <= other.max.z;
    }
    AABB Mesh::Bounds() const noexcept { AABB bounds; for (const auto& vertex : vertices) bounds.Expand(vertex); return bounds; }
}
