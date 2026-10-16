#pragma once

#include "skyrim/parser/plugin_parser.h"

namespace navmesh::skyrim::detail
{
    /// Convert a non-null source-local FormID to the output master table, or reject an unresolved dependency.
    using ReferenceFormIdRebaser = std::function<std::optional<std::uint32_t>(std::uint32_t)>;

    /** Copy a winning REFR payload while rebuilding its navigation association.
     * @param source Decoded source reference, retaining ordered subrecords and their original encodings.
     * @param rebase Source-local to output-local FormID conversion, including references in Papyrus properties.
     * @param association New output-local NAVM and zero-based triangle, or no value to remove stale XNDP.
     * @param retainsStringTable True only when localized FULL IDs retain the same filename and resources.
     * @param localizedSource Whether the source TES4 header uses localized strings.
     * @param output Receives the complete payload, preserving non-navigation fields and rebasing their FormIDs.
     * @param error Receives the unsupported field, malformed payload, or missing dependency on failure.
     * @return False rather than copying a field whose reference layout is unknown.
     */
    [[nodiscard]] bool RewriteReferenceNavigation(const PluginRecord &source, const ReferenceFormIdRebaser &rebase,
                                                  std::optional<NavmeshDoorAssociation> association,
                                                  bool retainsStringTable, bool localizedSource,
                                                  std::vector<std::uint8_t> &output, std::string &error);
} // namespace navmesh::skyrim::detail
