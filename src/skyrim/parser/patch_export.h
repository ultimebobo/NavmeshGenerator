#pragma once

#include "skyrim/parser/plugin_parser.h"

#include <map>

namespace navmesh::skyrim::detail
{
    /** Publish a coordinated navigation patch set from finalized records in resolved identity space.
     * @param directory Output directory; existing plugins, manifests and staging directories are refused.
     * @param filename First patch filename, distinct from active inputs.
     * @param input Immutable load-order snapshot supplying owners, source order and unchanged navigation.
     * @param groups Encoded NAVM/REFR groups with all FormIDs in resolved load-order space.
     * @param newMeshes New NAVM identities in the reserved FF prefix; they must remain in one output.
     * @param navigation Optional existing resolved NAVI identity; otherwise each file allocates its own.
     * @param nextObjectId First unallocated output object ID, after the new meshes.
     * @param stepHeights Generated NAVM identities and permitted climbs in Skyrim world units.
     * @param autoSplit Permit dependency-driven splitting for exports without new NAVMs.
     * @param written Receives every published plugin in installation order; empty on failure.
     * @param error Receives a dependency, validation or publication failure.
     * @return True after every file round-trips and the complete graph passes validation.
     * A manifest is published last as the completion marker; failed publication rolls back created outputs.
     */
    [[nodiscard]] bool WriteNavigationPatchSet(const std::filesystem::path &directory, const std::string &filename,
                                               const ResolvedLoadOrder &input, const std::vector<std::uint8_t> &groups,
                                               const std::set<std::uint32_t> &newMeshes,
                                               std::optional<std::uint32_t> navigation, std::uint32_t nextObjectId,
                                               const std::map<std::uint32_t, float> &stepHeights, bool autoSplit,
                                               std::vector<std::filesystem::path> &written, std::string &error);
} // namespace navmesh::skyrim::detail
