#pragma once

#include "skyrim/parser/plugin_parser.h"

#include <span>

namespace navmesh::skyrim::detail
{
    /// Authored metadata and its original vertex/FormID spaces, borrowed for a generated primary NAVM.
    struct NavmeshMetadataSource
    {
        const PluginRecord *record{};
        const core::NavMesh *mesh{};
        /// Convert a non-null source-local FormID to the output master table, or reject it.
        std::function<std::optional<std::uint32_t>(std::uint32_t)> rebase;
    };

    /** Rebuild NAVM auxiliary arrays against generated vertex and master indices.
     * @param sources Authored meshes being combined into the generated primary record.
     * @param generated Final world-space vertices; triangle rotation does not change their indices.
     * @param tolerance Maximum world-space displacement allowed for a connector constraint.
     * @param payload Receives encoded ONAM, PNAM and NNAM, omitting empty arrays.
     * @param error Receives a malformed array, conflicting constraint, or unmappable vertex reason.
     * @param warnings Optional sink for preferred connectors omitted when no generated vertex is in reach.
     * @return False for malformed data, contradictory constraints or an unmappable exclusion.
     */
    [[nodiscard]] bool RebuildNavmeshMetadata(std::span<const NavmeshMetadataSource> sources,
                                              const core::NavMesh &generated, float tolerance,
                                              std::vector<std::uint8_t> &payload, std::string &error,
                                              std::vector<std::string> *warnings = nullptr);
} // namespace navmesh::skyrim::detail
