#include "core/scene/scene.h"

#include <cmath>

namespace navmesh::core
{
    Transform Transform::Identity() noexcept { return { { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } }; }
    Transform Transform::FromEulerXYZ(const Vec3 translation, const Vec3 radians, const float scale) noexcept
    {
        const auto cx = std::cos(radians.x), sx = std::sin(radians.x), cy = std::cos(radians.y), sy = std::sin(radians.y), cz = std::cos(radians.z), sz = std::sin(radians.z);
        // Column-major storage, matching ApplyPoint and affine composition.
        return { { scale * cz * cy, scale * sz * cy, -scale * sy, 0,
            scale * (cz * sy * sx - sz * cx), scale * (sz * sy * sx + cz * cx), scale * cy * sx, 0,
            scale * (cz * sy * cx + sz * sx), scale * (sz * sy * cx - cz * sx), scale * cy * cx, 0,
            translation.x, translation.y, translation.z, 1 } };
    }
    Transform Transform::FromSkyrimReference(const Vec3 translation, const Vec3 radians, const float scale) noexcept
    {
        const auto cx = std::cos(radians.x), sx = std::sin(radians.x), cy = std::cos(radians.y), sy = std::sin(radians.y), cz = std::cos(radians.z), sz = std::sin(radians.z);
        // Match NiMatrix3::SetEulerAnglesXYZ, then store its rows as columns
        // for Transform's column-major ApplyPoint representation.
        return { { scale * cy * cz, scale * (sx * sy * cz - cx * sz), scale * (cx * sy * cz + sx * sz), 0,
            scale * cy * sz, scale * (sx * sy * sz + cx * cz), scale * (cx * sy * sz - sx * cz), 0,
            -scale * sy, scale * sx * cy, scale * cx * cy, 0,
            translation.x, translation.y, translation.z, 1 } };
    }
    Transform Transform::Then(const Transform& child) const noexcept
    {
        Transform result{};
        for (std::size_t column = 0; column < 4; ++column) for (std::size_t row = 0; row < 4; ++row) for (std::size_t k = 0; k < 4; ++k) result.matrix[column * 4 + row] += matrix[k * 4 + row] * child.matrix[column * 4 + k];
        return result;
    }
    Vec3 Transform::ApplyPoint(const Vec3 point) const noexcept { return { matrix[0] * point.x + matrix[4] * point.y + matrix[8] * point.z + matrix[12], matrix[1] * point.x + matrix[5] * point.y + matrix[9] * point.z + matrix[13], matrix[2] * point.x + matrix[6] * point.y + matrix[10] * point.z + matrix[14] }; }
    std::optional<Transform> Scene::WorldTransform(const std::size_t nodeIndex) const noexcept
    {
        if (nodeIndex >= nodes.size()) return std::nullopt;
        Transform world = nodes[nodeIndex].localTransform; auto parent = nodes[nodeIndex].parent; std::size_t steps{};
        while (parent) { if (*parent >= nodes.size() || ++steps > nodes.size()) return std::nullopt; world = nodes[*parent].localTransform.Then(world); parent = nodes[*parent].parent; }
        return world;
    }
    bool Scene::HasCompleteTriangleProvenance() const noexcept { return triangleProvenance.size() == mesh.triangles.size(); }
}
