#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace navmesh::core
{
    struct Vec3 { float x{}; float y{}; float z{}; };
    struct AABB {
        Vec3 min{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
        Vec3 max{ std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest() };
        [[nodiscard]] bool IsValid() const noexcept;
        void Expand(const Vec3& point) noexcept;
    };
    struct Triangle { std::array<std::uint32_t, 3> vertices{}; };
    struct Mesh { std::vector<Vec3> vertices; std::vector<Triangle> triangles; [[nodiscard]] AABB Bounds() const noexcept; };
}
