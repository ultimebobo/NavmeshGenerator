#pragma once

#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/parser/affected_cells.h"

namespace navmesh::skyrim
{
    /// Resolved asset inputs for comparing navigation collision across a rebuild scope.
    struct CollisionImpactInput
    {
        /// Active plugin filename, case insensitive; empty compares plugins after the first baseline input.
        std::string plugin;
        /// Game Data and writable asset snapshot directories; game files are read only.
        std::filesystem::path dataDirectory, cacheDirectory, previousCacheDirectory;
        /// Winning assets and preceding asset providers for selected model replacements; null uses game Data.
        const ModelAssetSources *assets{}, *previousAssets{};
        /// Normalized logical model paths contributed by the selected asset scope.
        std::set<std::string> changedModels;
        /// Archive indexing could not identify replacements; compare all model uses against preceding providers.
        bool archiveModelsChanged{};
        /// Restrict selection to changed terrain and water inputs when models are excluded from generation.
        bool terrainOnly{};
        /// Optional cooperative cancellation; cancellation stops selection before generation or writing.
        GeometryCancellationCallback cancelled;
        /// Optional synchronous activity sink identifying the reference and model about to be compared.
        std::function<void(std::string_view)> progress;
        /// Optional recovery sink for required geometry at a historical ownership context.
        /// Returning true permits comparison to continue only after the caller excludes every potentially affected target.
        /// Without this sink any required-geometry failure throws; input, allocation and storage exceptions always propagate.
        std::function<bool(const GeometryFailure &, const RecordOrigin &)> failedGeometry;
    };

    /** Select cells whose supported terrain, water or placed collision inputs change.
     * Selected record transitions compare collision before/after the contributing plugin,
     * including base-object uses, moves, deletion and disabled state. Unchanged triangles,
     * visual-only models and NAVM-only edits do not enlarge the rebuild scope. Actual
     * world-space collision triangles determine exterior footprints without a whole-cell halo.
     * New terrain and collision can select cells without authored NAVM, including new
     * worldspaces and submerged terrain. Water-only transitions require supported winning
     * terrain or placed collision because water classifies geometry rather than supplying floor.
     * Exterior teleport boundaries participate in collision comparisons; return-side edits also
     * select the physically owning cells of affected entrance anchors at both historical cutoffs.
     * @param resolved Immutable full load order, retained for the duration of selection.
     * @param index Spatial index built from resolved; provides physical ownership and exterior queries.
     * @param input Scope, asset providers and extraction/cancellation policy.
     * @param cache Bounded model/placement cache shared with subsequent generation.
     * @param statistics Selection counters, reset before processing.
     * @return Unique targets in worldspace/coordinate order, followed by interiors in FormID order.
     * @throws std::invalid_argument if the named plugin is inactive.
     * Missing current models are recorded in statistics. Required-geometry failures throw unless failedGeometry
     * accepts responsibility for excluding their complete uncertain historical ownership scopes.
     * A new asset's absent preceding provider is a valid empty comparison input.
     * @throws std::runtime_error For unrecovered geometry failures, invalid ownership or cancellation.
     * Allocation and storage exceptions always propagate; no patch may be published after those failures.
     */
    [[nodiscard]] std::vector<const core::Cell *> SelectCollisionAffectedCells(const ResolvedLoadOrder &resolved,
                                                                               const CellImpactIndex &index,
                                                                               const CollisionImpactInput &input,
                                                                               ModelGeometryCache &cache,
                                                                               ImpactSelectionStatistics &statistics);
} // namespace navmesh::skyrim
