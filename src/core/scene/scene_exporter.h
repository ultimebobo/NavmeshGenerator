#pragma once

#include "core/reproducibility/export_metadata.h"
#include "core/scene/scene.h"
#include "core/navmesh/types.h"
#include "core/navmesh/candidate.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace navmesh::core
{
    /// Visible inspection groups; exported ordering is independent of layer selection order.
    enum class SceneLayer
    {
        ExistingNavmesh,
        Terrain,
        Collision,
        RenderFallback,
        DiagnosticMarkers,
        CandidateNavmesh,
        /// Derived ownership group included by ExistingNavmesh selection.
        OriginalNavmesh,
        /// Derived connection group included by ExistingNavmesh selection.
        NavmeshLinks
    };
    struct SceneBounds
    {
        AABB world;
    };
    /// Polygon analysis evidence for coloring NAVM faces; no centroid marker is emitted.
    struct DiagnosticMarker
    {
        Vec3 position;
        std::string classification;
        std::size_t navmeshPolygon{};
        std::optional<std::size_t> supportTriangle;
        std::optional<std::uint32_t> navmeshFormId;
    };
    /// A finalized generated mesh owned by a selected CELL, in Skyrim world coordinates.
    struct SceneCandidate
    {
        /// Resolved CELL identity used to resolve generated-neighbor links before plugin allocation.
        std::uint32_t cellFormId{};
        /// Borrowed complete candidate; must remain valid through scene export.
        const CandidateNavMesh *candidate{};
    };
    /// GLB selection and borrowed candidate evidence; pointers must remain valid through export.
    struct SceneExportOptions
    {
        /// ExistingNavmesh selects original/neighbor groups and bars; DiagnosticMarkers selects exit markers.
        std::vector<SceneLayer> layers{SceneLayer::ExistingNavmesh, SceneLayer::Terrain, SceneLayer::Collision,
                                       SceneLayer::RenderFallback, SceneLayer::DiagnosticMarkers};
        std::optional<SceneBounds> bounds;
        bool detailedProvenance{true};
        /// Borrowed generated mesh; water/preferred flags select distinct inspection materials.
        const NavMesh *candidateNavmesh{};
        /// Placed exits to another area, including physical doors and cave entrances, shown in orange.
        /// Positions use Skyrim world coordinates; optional polygon indices color the candidate's linked faces.
        /// These exits also supply inspection markers without a candidate; ownership stays with the caller.
        const std::vector<CandidateExit> *candidateEntrances{};
        /// Generated exterior connections; invalid or unavailable destination triangles are omitted.
        /// Displaying a candidate suppresses authored bars involving the selected cell's original NAVMs.
        const std::vector<CandidateBorderLink> *candidateBorderLinks{};
        /// Multi-cell candidates, including per-cell exit indices and generated-neighbor connections.
        /// Used instead of the single-candidate fields when nonempty; candidate pointers must be nonnull.
        std::vector<SceneCandidate> candidates;
    };
    struct SceneExportResult
    {
        std::size_t objects{};
        std::size_t triangles{};
        std::size_t culledTriangles{};
        /// True only when the GLB and adjacent provenance JSON both finish writing successfully.
        bool written{};
    };

    /** Write a world-coordinate inspection GLB and adjacent provenance JSON.
     * @param outputPath Destination file, overwritten when it can be opened.
     * @param scene Terrain, collision, and display geometry with source provenance.
     * @param navmeshes Authored meshes; metadata.selectedCell identifies meshes grouped as original.
     * @param markers Analysis evidence for polygon colors, without rendered diagnostic pyramids.
     * @param metadata Run provenance and selected CELL(s); their NAVM identities select original meshes.
     * @param options Layer/bounds selection and borrowed generated connection evidence.
     * @return Emitted counts and written status; false written indicates an output failure.
     * Solid connection bars follow recorded source edges, lifted above the NAVM in Skyrim Z-up space.
     * Invalid triangles and unresolved connections are omitted. Requested empty geometry groups are retained.
     */
    [[nodiscard]] SceneExportResult WriteCombinedGlb(const std::filesystem::path &outputPath, const Scene &scene,
                                                     const std::vector<NavMesh> &navmeshes,
                                                     const std::vector<DiagnosticMarker> &markers,
                                                     const reproducibility::ExportMetadata &metadata,
                                                     const SceneExportOptions &options = {});
} // namespace navmesh::core
