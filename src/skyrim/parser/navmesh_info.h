#pragma once

#include "skyrim/parser/plugin_parser.h"

#include <map>

namespace navmesh::skyrim::detail
{
    /** Build NAVI subrecords from final, output-local NVNM geometry.
     * @param geometry NAVM FormIDs mapped to serialized NVNM bodies, including empty replacements and neighbors.
     * Vertices use Skyrim world units; all embedded FormIDs already use the output master table.
     * @param preserved Optional source-copy NAVI in the same master space. Unchanged NVMI and NVSI entries
     * are retained; the optional precomputed path cache is invalidated when geometry changes.
     * @param payload Receives the complete NAVI payload, including version and precomputed-path counts.
     * @param error Receives a reason for unsupported or malformed navigation data.
     * @return True when every mesh has a bounded, matching navigation-map entry.
     */
    [[nodiscard]] bool BuildNavmeshInfo(const std::map<std::uint32_t, std::vector<std::uint8_t>> &geometry,
                                        const PluginRecord *preserved, std::vector<std::uint8_t> &payload,
                                        std::string &error);
} // namespace navmesh::skyrim::detail
