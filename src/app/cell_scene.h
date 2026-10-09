#pragma once

#include "app/options.h"
#include "core/scene/scene_exporter.h"

#include <map>
#include <set>
#include <tuple>

namespace navmesh::app::detail
{
    /// Decode the shared comma-separated inspection layer selection; unknown tokens are ignored.
    [[nodiscard]] std::vector<core::SceneLayer> ParseSceneLayers(const std::string &value);

    /// Accumulates requested inspection geometry without duplicate overlapping neighborhoods.
    /// Storage is owned for the run; all vertices retain their native Skyrim world coordinates.
    class CellScene
    {
      public:
        /** Append extracted geometry and visible CELL navigation/exit evidence.
         * @param scene World-space triangles with matching provenance arrays; invalid triangles are omitted.
         * @param cells Scene neighborhood suppliers; authored meshes and enabled exits are deduplicated by Form ID.
         * @param markers Optional authored polygon classifications in resolved Form ID space.
         * Copies retained evidence. Identical source triangles at identical positions are emitted once,
         * while different placed references remain distinct even when their geometry overlaps.
         */
        void Append(const core::Scene &scene, const std::vector<core::Cell> &cells,
                    const std::vector<core::DiagnosticMarker> &markers = {});

        /** Write the combined GLB and its provenance/metadata sidecars.
         * @param path Standard destination in the output folder; existing scene files are overwritten.
         * @param options Shared layer and detail choices; scene storage is independent of batch artifact policy.
         * @param selected Complete resolved Cell selection, borrowed until this call finishes.
         * @param candidates Finalized generated candidates, borrowed until this call finishes.
         * @return False if the GLB or either sidecar cannot be written; exceptions propagate to the run boundary.
         * Native coordinates are preserved across interiors/worldspaces; unrelated spaces may overlap.
         */
        [[nodiscard]] bool Write(const std::filesystem::path &path, const Options &options,
                                 const std::vector<const core::Cell *> &selected,
                                 const std::vector<core::SceneCandidate> &candidates = {}) const;

      private:
        using SourceKey = std::tuple<std::string, std::uint32_t, std::string, std::uint32_t, std::string,
                                     core::GeometrySourceType, core::MaterialCollisionClass, std::string, bool>;
        using TriangleKey = std::tuple<std::size_t, std::size_t, std::uint32_t, std::array<std::uint32_t, 9>, bool>;
        /// Retained geometry and indices refer only to this owned scene.
        core::Scene scene_;
        std::map<SourceKey, std::size_t> sources_;
        std::set<TriangleKey> triangles_;
        std::map<std::uint32_t, core::NavMesh> navmeshes_;
        std::map<std::uint32_t, core::CandidateExit> doors_;
        std::vector<core::DiagnosticMarker> markers_;

        /// Copy one support/display mesh, rebasing both vertex and geometry-source indices after deduplication.
        void AppendMesh(const core::Scene &scene, bool render);
    };
} // namespace navmesh::app::detail
