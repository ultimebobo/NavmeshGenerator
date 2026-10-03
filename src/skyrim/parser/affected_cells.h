#pragma once

#include "skyrim/parser/plugin_parser.h"

#include <map>
#include <set>
#include <tuple>

namespace navmesh::skyrim
{
    /// Selection diagnostics count records and placed uses; categories can overlap in their resulting CELLs.
    struct ImpactSelectionStatistics
    {
        /// Navigation-changing selected records, grouped by Bethesda record type.
        std::map<std::string, std::size_t> changedRecords;
        /// Selected records whose origins contain only equivalent navigation inputs.
        std::size_t equivalentRecords{};
        /// Placed records selected by a changed base-object dependency.
        std::size_t baseObjectUses{};
        /// Winning placements selected by loose/indexed archive assets or archive fallback.
        std::size_t assetUses{};
    };

    /** Spatial/dependency index shared by all targets in one resolved run.
     * Reference positions use Skyrim world units. The load order must outlive
     * the index and keep its cells and records unchanged.
     */
    class CellImpactIndex
    {
      public:
        /// Index exterior coordinates, winning reference placements, and cell ownership.
        explicit CellImpactIndex(const ResolvedLoadOrder &resolved);
        /** Select conservative navigation targets from record edit history.
         * @param plugin Active filename, case insensitive; empty selects all plugins
         * after the first input plugin, which supplies the baseline.
         * @param radius Exterior impact halo in cells; at least the adjacent ring is used.
         * @param changedModels Logical model paths replaced by enabled loose assets.
         * @param archiveModelsChanged Conservatively include every model-bearing
         * reference when mod archives can replace models without a plugin edit.
         * @param statistics Optional output counters; reset before selection.
         * @return Unique cells ordered by worldspace/coordinates, then interior FormID.
         * @throws std::invalid_argument for an unknown plugin or negative radius.
         * Deleted/moved placements include historical and winning locations. Changes
         * to base records select their placed uses, even when the REFR is untouched.
         */
        [[nodiscard]] std::vector<const core::Cell *> AffectedCells(
            const std::string &plugin, int radius, const std::set<std::string> &changedModels = {},
            bool archiveModelsChanged = false, ImpactSelectionStatistics *statistics = nullptr) const;
        /// Existing exterior cells within a Chebyshev radius, including the target.
        /// Interior targets return only themselves; negative radius throws.
        [[nodiscard]] std::vector<const core::Cell *> Neighbors(const core::Cell &cell, int radius) const;
        /// Geometry source cells for a target halo, including oversized model bounds
        /// and distant persistent placements. Unknown model bounds conservatively
        /// include source cells throughout the worldspace.
        [[nodiscard]] std::vector<const core::Cell *> GeometryNeighbors(const core::Cell &cell, int radius) const;
        /// Winning references bucketed by physical exterior position, including
        /// persistent references stored in a distant worldspace parent CELL.
        /// Returns copied cell metadata and references without NAVM geometry.
        /** Copy winning placements, optionally excluding models whose conservative bounds miss a world-space area.
         * @param cell Source CELL; persistent placements use physical ownership.
         * @param bounds Optional Skyrim-world AABB. Unknown bounds remain included.
         * @return Metadata and eligible reference copies without NAVM geometry, in source order.
         */
        [[nodiscard]] core::Cell GeometryCell(const core::Cell &cell,
                                              std::optional<core::AABB> bounds = std::nullopt) const;

      private:
        using Key = std::tuple<std::uint32_t, std::int32_t, std::int32_t>;
        const ResolvedLoadOrder &resolved_;
        std::map<Key, const core::Cell *> exterior_;
        std::map<std::uint32_t, const core::Cell *> cells_;
        std::map<std::uint32_t, std::uint32_t> worlds_;
        std::map<std::uint32_t, std::vector<const core::Reference *>> references_;
        std::map<std::uint32_t, std::set<std::uint32_t>> geometrySources_;
        std::map<std::uint32_t, std::set<std::uint32_t>> unboundedSources_;
        [[nodiscard]] const core::Cell *PhysicalCell(const RecordOrigin &origin) const;
        [[nodiscard]] std::vector<const core::Cell *> Footprint(const RecordOrigin &origin, int radius) const;
    };
} // namespace navmesh::skyrim
