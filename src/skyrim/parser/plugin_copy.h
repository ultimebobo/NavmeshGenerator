#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace navmesh::skyrim::offline::detail
{
    /// Validated source bytes and identity limits for a byte-preserving plugin export.
    struct PluginCopy
    {
        /// Complete source file; unknown and compressed records remain encoded.
        std::vector<std::uint8_t> bytes;
        /// Source master order, excluding the source plugin's implicit self slot.
        std::vector<std::string> masters;
        /// Original TES4 flags, including master, light, and localization flags.
        std::uint32_t flags{};
        /// First unused local object ID, respecting HEDR and every source record type.
        std::uint32_t nextObjectId{};
    };

    /** Validate the complete record/group envelope and TES4 header for copying.
     * @param bytes Source plugin bytes; record payloads need not be decoded.
     * @param copy Receives the source snapshot, master names, flags, and allocation limit.
     * @param error Receives a reason for malformed structure or header data.
     * @return True when every byte belongs to a bounded record or group and HEDR is present.
     */
    [[nodiscard]] bool PreparePluginCopy(std::vector<std::uint8_t> bytes, PluginCopy &copy, std::string &error);

    /** Merge serialized NAVMs into a source copy without rebasing unrelated records.
     * @param copy Validated source snapshot; its master table and self index stay unchanged.
     * @param navigation Complete generated groups with source-table-local FormIDs.
     * @param nextObjectId Next available source-owned ID after NAVM allocation.
     * @param output Receives the merged plugin, including updated group sizes, HEDR
     * counts, and ONAM override entries when required by the source header.
     * @param error Receives a structural or identity failure reason.
     * @return True when generated NAVMs occur exactly once and all other records are retained.
     * @warning Navigation may refer only to the source plugin and its existing masters.
     */
    [[nodiscard]] bool MergePluginCopy(const PluginCopy &copy, const std::vector<std::uint8_t> &navigation,
                                       std::uint32_t nextObjectId, std::vector<std::uint8_t> &output,
                                       std::string &error);
} // namespace navmesh::skyrim::offline::detail
