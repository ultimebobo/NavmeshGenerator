#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace navmesh::core
{
    /// Three-dimensional point or vector in Skyrim world units.
    struct Vec3
    {
        float x{};
        float y{};
        float z{};

        [[nodiscard]] friend Vec3 operator+(const Vec3 &lhs, const Vec3 &rhs) noexcept
        {
            return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
        }

        [[nodiscard]] friend Vec3 operator-(const Vec3 &lhs, const Vec3 &rhs) noexcept
        {
            return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
        }

        [[nodiscard]] friend Vec3 operator*(const Vec3 &value, float scalar) noexcept
        {
            return {value.x * scalar, value.y * scalar, value.z * scalar};
        }

        [[nodiscard]] friend Vec3 operator*(float scalar, const Vec3 &value) noexcept
        {
            return value * scalar;
        }

        [[nodiscard]] friend Vec3 operator/(const Vec3 &value, float scalar) noexcept
        {
            return {value.x / scalar, value.y / scalar, value.z / scalar};
        }
    };

    /// Axis-aligned bounds. A default-constructed instance is invalid until expanded.
    struct AABB
    {
        Vec3 min{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                 std::numeric_limits<float>::max()};
        Vec3 max{std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                 std::numeric_limits<float>::lowest()};
        [[nodiscard]] bool IsValid() const noexcept;
        void Expand(const Vec3 &point) noexcept;
        [[nodiscard]] Vec3 Center() const noexcept;
        [[nodiscard]] Vec3 Extent() const noexcept;
        [[nodiscard]] bool Contains(const Vec3 &point) const noexcept;
        [[nodiscard]] bool Intersects(const AABB &other) const noexcept;
    };
    /// Indices into the owning mesh's vertex array.
    struct Triangle
    {
        std::array<std::uint32_t, 3> vertices{};
    };
    /// Triangle geometry with vertices in Skyrim world units.
    struct Mesh
    {
        std::vector<Vec3> vertices;
        std::vector<Triangle> triangles;
        [[nodiscard]] AABB Bounds() const noexcept;
    };
} // namespace navmesh::core
