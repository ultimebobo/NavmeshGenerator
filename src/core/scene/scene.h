#pragma once

#include "core/geometry/types.h"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace navmesh::core
{
    // Skyrim/NIF angles are radians. EulerXYZ means apply X, then Y, then Z to a
    // column vector (the resulting matrix is Rz * Ry * Rx).
    struct Transform
    {
        std::array<float, 16> matrix{};
        [[nodiscard]] static Transform Identity() noexcept;
        [[nodiscard]] static Transform FromEulerXYZ(Vec3 translation, Vec3 radians, float uniformScale = 1.0F) noexcept;
        [[nodiscard]] Transform Then(const Transform& child) const noexcept;
        [[nodiscard]] Vec3 ApplyPoint(Vec3 point) const noexcept;
    };

    enum class MaterialCollisionClass { Unknown, RenderVisual, Terrain, HavokPackedTriangles };
    // This is deliberately separate from material: it is the evidence class a
    // navigation query must report, even when the triangle was produced from a
    // model that also has a render mesh.
    enum class GeometrySourceType { Terrain, Collision, RenderFallback };
    enum class GeometryCoverage { Found, Excluded, Missing, Unreadable, Unsupported };
    struct RecordProvenance { std::string plugin; std::uint32_t formId{}; std::string recordType; };
    struct GeometrySource { std::string modelPath; MaterialCollisionClass materialClass{ MaterialCollisionClass::RenderVisual }; GeometrySourceType sourceType{ GeometrySourceType::RenderFallback }; std::string collisionType; float confidence{ 0.5F }; RecordProvenance reference; RecordProvenance baseObject; };
    // LAND samples identify the lower-left grid sample of the terrain quad that
    // emitted this triangle. Diagnostics can therefore identify the exact data.
    struct TerrainTriangleProvenance { std::int32_t cellX{}; std::int32_t cellY{}; std::uint32_t landFormId{}; std::uint8_t sampleX{}; std::uint8_t sampleY{}; };
    struct TriangleProvenance { std::size_t geometrySource{}; std::size_t sourceTriangle{}; std::optional<TerrainTriangleProvenance> terrain; };
    struct SceneNode { std::string name; std::optional<std::size_t> parent; Transform localTransform{ Transform::Identity() }; std::optional<std::size_t> geometrySource; };
    struct CoverageEntry { GeometryCoverage status{}; GeometrySource source; std::string detail; };
    struct Scene
    {
        std::vector<SceneNode> nodes;
        std::vector<GeometrySource> geometrySources;
        // Visual meshes retained for scene inspection when authoritative
        // collision also exists.  They are deliberately separate from mesh:
        // render geometry must not become navigation-support evidence.
        Mesh renderFallbackMesh;
        std::vector<TriangleProvenance> renderFallbackTriangleProvenance;
        Mesh mesh;
        std::vector<TriangleProvenance> triangleProvenance;
        std::vector<CoverageEntry> coverage;
        [[nodiscard]] std::optional<Transform> WorldTransform(std::size_t nodeIndex) const noexcept;
        [[nodiscard]] bool HasCompleteTriangleProvenance() const noexcept;
    };
}
