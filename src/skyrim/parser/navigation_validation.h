#pragma once

#include "skyrim/parser/plugin_parser.h"

#include <map>

namespace navmesh::skyrim::detail
{
    /** Validate exported navigation against its effective load-order graph.
     * @param records Independently read output records, using output-local FormIDs.
     * @param masters Output TES4 master order, excluding the output's implicit self slot.
     * @param outputName Final plugin filename; source copies retain their original identity.
     * @param input Winning input snapshot, supplying unchanged portal destinations and door references.
     * @param modifiedMeshes Output-local NAVM identities whose complete graph closure must be valid.
     * @param modifiedReferences Output-local REFR identities whose XNDP changes must be reciprocal.
     * @param error Receives the malformed mesh, dangling link, or nonreciprocal connection reason.
     * @param generatedStepHeights Output-local generated NAVM identities and their allowed shared-edge climb,
     * in Skyrim world units including weld tolerance. Other meshes retain authored endpoint tolerance.
     * @return True when geometry, grid, cover, portal destinations and both directions of door associations agree.
     * Coordinates use Skyrim world units. Unrelated original records are inspected only when referenced.
     */
    [[nodiscard]] bool ValidateNavigationGraph(const std::vector<ResolvedRecord> &records,
                                               const std::vector<std::string> &masters, const std::string &outputName,
                                               const ResolvedLoadOrder &input,
                                               const std::set<std::uint32_t> &modifiedMeshes,
                                               const std::set<std::uint32_t> &modifiedReferences, std::string &error,
                                               const std::map<std::uint32_t, float> &generatedStepHeights = {});
} // namespace navmesh::skyrim::detail
