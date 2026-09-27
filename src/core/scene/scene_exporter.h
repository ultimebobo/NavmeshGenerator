#pragma once

#include "core/reproducibility/export_metadata.h"
#include "core/scene/scene.h"
#include "core/navmesh/types.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace navmesh::core
{
    // These names are deliberately stable: they are the visible layer names in
    // glTF viewers and the keys used by scene.provenance.json.
    enum class SceneLayer { ExistingNavmesh, Terrain, Collision, RenderFallback, DiagnosticMarkers };
    struct SceneBounds { AABB world; };
    struct DiagnosticMarker { Vec3 position; std::string classification; std::size_t navmeshPolygon{}; std::optional<std::size_t> supportTriangle; std::optional<std::uint32_t> navmeshFormId; };
    struct SceneExportOptions
    {
        std::vector<SceneLayer> layers{ SceneLayer::ExistingNavmesh, SceneLayer::Terrain, SceneLayer::Collision, SceneLayer::RenderFallback, SceneLayer::DiagnosticMarkers };
        std::optional<SceneBounds> bounds;
        bool detailedProvenance{ true };
    };
    struct SceneExportResult { std::size_t objects{}; std::size_t triangles{}; std::size_t culledTriangles{}; };

    [[nodiscard]] SceneExportResult WriteCombinedGlb(const std::filesystem::path& outputPath, const Scene& scene, const std::vector<NavMesh>& navmeshes, const std::vector<DiagnosticMarker>& markers, const reproducibility::ExportMetadata& metadata, const SceneExportOptions& options = {});
}
