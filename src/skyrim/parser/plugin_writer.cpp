#include "skyrim/parser/plugin_writer.h"
#include "skyrim/parser/plugin_copy.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <optional>
#include <tuple>

namespace
{
    using Bytes = std::vector<std::uint8_t>;
    using navmesh::skyrim::ResolvedRecord;
    constexpr std::uint32_t PathingDoorCrc = 0xE48B73F3U;

    void U16(Bytes &b, std::uint16_t n)
    {
        b.push_back(static_cast<std::uint8_t>(n));
        b.push_back(static_cast<std::uint8_t>(n >> 8));
    }
    void U32(Bytes &b, std::uint32_t n)
    {
        for (int shift{}; shift < 32; shift += 8)
        {
            b.push_back(static_cast<std::uint8_t>(n >> shift));
        }
    }
    std::uint32_t Get32(std::span<const std::uint8_t> b, std::size_t p)
    {
        return static_cast<std::uint32_t>(b[p]) | static_cast<std::uint32_t>(b[p + 1]) << 8 |
               static_cast<std::uint32_t>(b[p + 2]) << 16 | static_cast<std::uint32_t>(b[p + 3]) << 24;
    }
    void Set32(Bytes &b, std::size_t p, std::uint32_t n)
    {
        for (int shift{}; shift < 32; shift += 8)
        {
            b[p + shift / 8] = static_cast<std::uint8_t>(n >> shift);
        }
    }
    void Set16(Bytes &b, std::size_t p, std::uint16_t n)
    {
        b[p] = static_cast<std::uint8_t>(n);
        b[p + 1] = static_cast<std::uint8_t>(n >> 8);
    }
    void Float(Bytes &b, float n)
    {
        std::uint32_t bits{};
        std::memcpy(&bits, &n, 4);
        U32(b, bits);
    }
    void PutSubrecord(Bytes &b, const char *kind, const Bytes &data)
    {
        if (data.size() > std::numeric_limits<std::uint16_t>::max())
        {
            b.insert(b.end(), {'X', 'X', 'X', 'X'});
            U16(b, 4);
            U32(b, static_cast<std::uint32_t>(data.size()));
        }
        b.insert(b.end(), kind, kind + 4);
        U16(b, data.size() > std::numeric_limits<std::uint16_t>::max() ? 0 : static_cast<std::uint16_t>(data.size()));
        b.insert(b.end(), data.begin(), data.end());
    }
    bool SameName(std::string a, std::string b)
    {
        std::transform(a.begin(), a.end(), a.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        std::transform(b.begin(), b.end(), b.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return a == b;
    }
    struct SourceInfo
    {
        std::string name;
        Bytes bytes;
        std::vector<std::string> masters;
        std::filesystem::path path;
    };
    /// Decode only the TES4 master names; the resolved load order already supplied the winning NAVMs.
    bool ReadMasterNames(const Bytes &source, std::vector<std::string> &masters)
    {
        if (source.size() < 24 || std::memcmp(source.data(), "TES4", 4) != 0)
        {
            return false;
        }
        const auto headerSize = static_cast<std::size_t>(Get32(source, 4));
        if (headerSize > source.size() - 24)
        {
            return false;
        }
        const auto end = 24 + headerSize;
        std::optional<std::uint32_t> extended;
        for (std::size_t p = 24; p < end;)
        {
            if (end - p < 6)
            {
                return false;
            }
            const std::string type(reinterpret_cast<const char *>(source.data() + p), 4);
            const auto shortSize = static_cast<std::uint16_t>(source[p + 4] | source[p + 5] << 8);
            p += 6;
            if (type == "XXXX")
            {
                if (shortSize != 4 || end - p < 4)
                {
                    return false;
                }
                extended = Get32(source, p);
                p += 4;
                continue;
            }
            const auto size = static_cast<std::size_t>(extended.value_or(shortSize));
            if (size > end - p)
            {
                return false;
            }
            if (type == "MAST")
            {
                const auto zero = std::find(source.begin() + p, source.begin() + p + size, 0);
                if (zero == source.begin() + p || zero == source.begin() + p + size)
                {
                    return false;
                }
                masters.emplace_back(source.begin() + p, zero);
            }
            p += size;
            extended.reset();
        }
        return !extended;
    }
    /// Convert a source-local FormID index into the generated plugin's master table.
    std::optional<std::uint32_t> RebaseFormId(const SourceInfo &source, const std::vector<std::string> &outputMasters,
                                              std::uint32_t raw)
    {
        const auto index = static_cast<std::size_t>(raw >> 24);
        if (index > source.masters.size())
        {
            return std::nullopt;
        }
        const auto &owner = index == source.masters.size() ? source.name : source.masters[index];
        const auto it = std::find_if(outputMasters.begin(), outputMasters.end(),
                                     [&](const auto &master) { return SameName(master, owner); });
        if (it == outputMasters.end())
        {
            return std::nullopt;
        }
        return static_cast<std::uint32_t>(std::distance(outputMasters.begin(), it)) << 24 | (raw & 0xFFFFFFU);
    }
    /// Resolve a load-order FormID through the record that introduced it, including light-plugin IDs.
    std::optional<std::uint32_t> RebaseResolvedFormId(const navmesh::skyrim::ResolvedLoadOrder &resolved,
                                                      const std::vector<std::string> &outputMasters,
                                                      std::uint32_t global)
    {
        const auto *record = resolved.FindWinning(global);
        if (!record || record->origins.empty())
        {
            return std::nullopt;
        }
        const auto &owner = record->origins.front().plugin;
        const auto it = std::find_if(outputMasters.begin(), outputMasters.end(),
                                     [&](const auto &master) { return SameName(master, owner); });
        if (it == outputMasters.end())
        {
            return std::nullopt;
        }
        const auto local = global >> 24 == 0xFEU ? global & 0xFFFU : global & 0xFFFFFFU;
        return static_cast<std::uint32_t>(std::distance(outputMasters.begin(), it)) << 24 | local;
    }
    /// Read the verified source range directly; copied-plugin assembly retains its complete input separately.
    Bytes SourceRecord(const SourceInfo &source, const ResolvedRecord &record)
    {
        const auto start = record.raw->headerRange.offset;
        const auto size = 24 + record.raw->filePayloadRange.size;
        std::ifstream stream(source.path, std::ios::binary | std::ios::ate);
        const auto length = stream.tellg();
        if (length < 0 || start > static_cast<std::uint64_t>(length) ||
            size > static_cast<std::uint64_t>(length) - start)
        {
            return {};
        }
        Bytes bytes(static_cast<std::size_t>(size));
        stream.seekg(static_cast<std::streamoff>(start));
        stream.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        return stream ? std::move(bytes) : Bytes{};
    }
    /** Collect every authored NAVM with an incoming link to replaced geometry.
     * Source-local targets are compared through each plugin's own master table,
     * including its implicit self slot. Unmatched incoming edges also need overrides
     * because their destination triangle indices cannot survive regeneration.
     */
    bool CollectIncomingNeighbors(const navmesh::skyrim::ResolvedLoadOrder &resolved,
                                  const std::vector<std::filesystem::path> &inputPlugins,
                                  const std::vector<const ResolvedRecord *> &replacedRecords,
                                  std::map<std::uint32_t, const ResolvedRecord *> &neighborRecords,
                                  std::map<std::uint32_t, navmesh::core::NavMesh> &neighborMeshes)
    {
        if (replacedRecords.empty())
        {
            return true;
        }
        std::map<std::string, std::set<std::uint32_t>> targetsBySource;
        for (const auto &record : resolved.records)
        {
            if (record.type != "NAVM" || !record.raw || !record.navm || !record.navm->supported ||
                (record.raw->flags & 0x20U) ||
                std::any_of(replacedRecords.begin(), replacedRecords.end(),
                            [&](const auto *replaced) { return replaced->formId == record.formId; }))
            {
                continue;
            }
            const auto &data = record.raw->decodedPayload;
            const auto start = static_cast<std::size_t>(record.navm->trailingData.offset);
            const auto size = static_cast<std::size_t>(record.navm->trailingData.size);
            if (start > data.size() || size > data.size() - start || size < 4)
            {
                continue;
            }
            const auto count = Get32(data, start);
            if (!count)
            {
                continue;
            }
            if (count > (size - 4) / 10)
            {
                return false;
            }
            auto targets = targetsBySource.find(record.winning.plugin);
            if (targets == targetsBySource.end())
            {
                const auto path = std::find_if(inputPlugins.begin(), inputPlugins.end(), [&](const auto &input)
                                               { return SameName(input.filename().string(), record.winning.plugin); });
                if (path == inputPlugins.end())
                {
                    return false;
                }
                std::ifstream stream(*path, std::ios::binary);
                Bytes header(24);
                stream.read(reinterpret_cast<char *>(header.data()), 24);
                if (!stream || std::memcmp(header.data(), "TES4", 4) != 0)
                {
                    return false;
                }
                const auto payloadSize = Get32(header, 4);
                if (payloadSize > 256ULL * 1024ULL * 1024ULL)
                {
                    return false;
                }
                header.resize(24 + static_cast<std::size_t>(payloadSize));
                stream.read(reinterpret_cast<char *>(header.data() + 24), payloadSize);
                std::vector<std::string> table;
                if (!stream || !ReadMasterNames(header, table))
                {
                    return false;
                }
                table.push_back(record.winning.plugin);
                std::set<std::uint32_t> localTargets;
                for (const auto *replaced : replacedRecords)
                {
                    if (const auto local = RebaseResolvedFormId(resolved, table, replaced->formId))
                    {
                        localTargets.insert(*local);
                    }
                }
                targets = targetsBySource.emplace(record.winning.plugin, std::move(localTargets)).first;
            }
            bool incoming{};
            for (std::uint32_t index{}; index < count; ++index)
            {
                incoming |= targets->second.contains(Get32(data, start + 4 + index * 10 + 4));
            }
            if (!incoming || neighborRecords.contains(record.formId))
            {
                continue;
            }
            if (!record.cellFormId || record.groupHeaders.empty() ||
                std::any_of(record.raw->subrecords.begin(), record.raw->subrecords.end(),
                            [](const auto &sub) { return sub.type != "NVNM" && sub.type != "EDID"; }))
            {
                return false;
            }
            const auto cell = std::find_if(resolved.cells.begin(), resolved.cells.end(),
                                           [&](const auto &owner) { return owner.id == *record.cellFormId; });
            if (cell == resolved.cells.end())
            {
                return false;
            }
            const auto mesh = std::find_if(cell->navMeshes.begin(), cell->navMeshes.end(),
                                           [&](const auto &item) { return item.id == record.formId; });
            if (mesh == cell->navMeshes.end())
            {
                return false;
            }
            neighborRecords.emplace(record.formId, &record);
            neighborMeshes.emplace(record.formId, *mesh);
        }
        return true;
    }
    /// Compact rebased external entries and clear edges into replaced triangle spaces.
    /// Requires a bounds-checked external table. Geometry and subsequent trailing sections retain their bytes.
    bool RemoveIncomingPortals(Bytes &nvnm, const navmesh::skyrim::NavmLayout &layout, navmesh::core::NavMesh &mesh,
                               const std::set<std::uint32_t> &replacedIds)
    {
        const auto trailing = static_cast<std::size_t>(layout.trailingData.offset - layout.header.offset);
        const auto oldExternalCount = Get32(nvnm, trailing);
        const auto insertion = trailing + 4 + oldExternalCount * 10;
        // Retain only external entries whose destination triangle space survives.
        // Remap every consuming edge after compaction; unmatched replacement edges
        // become open boundaries, and matched edges receive fresh reciprocal links.
        std::vector<std::uint32_t> externalRemap(oldExternalCount, std::numeric_limits<std::uint32_t>::max());
        Bytes retained;
        for (std::uint32_t index{}; index < oldExternalCount; ++index)
        {
            const auto offset = trailing + 4 + index * 10;
            if (!replacedIds.contains(Get32(nvnm, offset + 4)))
            {
                externalRemap[index] = static_cast<std::uint32_t>(retained.size() / 10);
                retained.insert(retained.end(), nvnm.begin() + offset, nvnm.begin() + offset + 10);
            }
        }
        for (std::size_t triangle{}; triangle < mesh.polygons.size(); ++triangle)
        {
            auto &face = mesh.polygons[triangle];
            for (std::size_t edge{}; edge < 3; ++edge)
            {
                if (!(face.flags & (1U << edge)))
                {
                    continue;
                }
                const auto index = face.neighbors[edge];
                if (index >= externalRemap.size())
                {
                    return false;
                }
                const auto remapped = externalRemap[index];
                if (remapped == std::numeric_limits<std::uint32_t>::max())
                {
                    face.flags &= static_cast<std::uint16_t>(~(1U << edge));
                    face.neighbors[edge] = 0xffffU;
                }
                else
                {
                    face.neighbors[edge] = remapped;
                }
                const auto offset =
                    static_cast<std::size_t>(layout.triangles.offset - layout.header.offset) + triangle * 16;
                Set16(nvnm, offset + 6 + edge * 2, static_cast<std::uint16_t>(face.neighbors[edge]));
                Set16(nvnm, offset + 12, face.flags);
            }
        }
        nvnm.erase(nvnm.begin() + trailing + 4, nvnm.begin() + insertion);
        nvnm.insert(nvnm.begin() + trailing + 4, retained.begin(), retained.end());
        Set32(nvnm, trailing, static_cast<std::uint32_t>(retained.size() / 10));
        return true;
    }
    struct Group
    {
        std::array<std::uint8_t, 24> header{};
        Bytes records;
        std::vector<Group> children;
    };
    void Add(Group &root, const ResolvedRecord &record, Bytes bytes)
    {
        auto *group = &root;
        for (const auto &header : record.groupHeaders)
        {
            auto it =
                std::find_if(group->children.begin(), group->children.end(), [&](const Group &child)
                             { return std::equal(header.begin() + 8, header.begin() + 16, child.header.begin() + 8); });
            if (it == group->children.end())
            {
                group->children.push_back({.header = header});
                it = std::prev(group->children.end());
            }
            group = &*it;
        }
        group->records.insert(group->records.end(), bytes.begin(), bytes.end());
    }
    Bytes Serialize(const Group &group)
    {
        Bytes body = group.records;
        for (const auto &child : group.children)
        {
            auto bytes = Serialize(child);
            body.insert(body.end(), bytes.begin(), bytes.end());
        }
        if (group.header[0] != 'G')
        {
            return body;
        }
        Bytes output(group.header.begin(), group.header.end());
        Set32(output, 4, static_cast<std::uint32_t>(24 + body.size()));
        output.insert(output.end(), body.begin(), body.end());
        return output;
    }
    bool ReadBack(
        const std::filesystem::path &path, const std::vector<std::string> &expectedMasters, bool expectedLight,
        const std::vector<std::pair<std::uint32_t, navmesh::core::NavMesh>> &expected,
        const std::map<std::uint32_t, std::vector<std::pair<std::uint16_t, std::uint32_t>>> &doorsByMesh,
        const std::vector<std::pair<std::uint32_t, std::uint32_t>> &expectedCells,
        const std::vector<std::tuple<std::uint32_t, std::uint16_t, std::uint32_t, std::uint16_t>> &expectedLinks,
        bool copiedPlugin = false)
    {
        std::vector<ResolvedRecord> records;
        std::vector<std::string> masters;
        std::vector<navmesh::skyrim::Diagnostic> diagnostics;
        bool light{};
        if (!navmesh::skyrim::DirectPluginReader{}.Read(path, records, masters, light, diagnostics) ||
            std::any_of(
                diagnostics.begin(), diagnostics.end(), [&](const auto &diagnostic)
                { return !copiedPlugin || diagnostic.kind != navmesh::skyrim::DiagnosticKind::UnsupportedVersion; }))
        {
            return false;
        }
        if (masters != expectedMasters || light != expectedLight)
        {
            return false;
        }
        if (!copiedPlugin &&
            static_cast<std::size_t>(std::count_if(records.begin(), records.end(), [](const auto &record)
                                                   { return record.type == "NAVM"; })) != expected.size())
        {
            return false;
        }
        for (const auto &[originalId, mesh] : expected)
        {
            const auto it = std::find_if(records.begin(), records.end(), [&](const auto &record)
                                         { return record.type == "NAVM" && record.winning.formId == originalId; });
            const auto owner = std::find_if(expectedCells.begin(), expectedCells.end(),
                                            [&](const auto &item) { return item.first == originalId; });
            if (it == records.end() || owner == expectedCells.end() || it->cellFormId != owner->second || !it->raw ||
                !it->navm || !it->navm->supported || it->navm->vertexCount != mesh.vertices.size() ||
                it->navm->triangleCount != mesh.polygons.size())
            {
                return false;
            }
            const auto &data = it->raw->decodedPayload;
            for (std::size_t i{}; i < mesh.vertices.size(); ++i)
            {
                if (std::memcmp(data.data() + it->navm->vertices.offset + i * 12, &mesh.vertices[i], 12) != 0)
                {
                    return false;
                }
            }
            for (std::size_t i{}; i < mesh.polygons.size(); ++i)
            {
                for (int j{}; j < 3; ++j)
                {
                    const auto offset = static_cast<std::size_t>(it->navm->triangles.offset) + i * 16;
                    const auto vertex =
                        static_cast<std::uint16_t>(data[offset + j * 2] | data[offset + j * 2 + 1] << 8);
                    if (vertex != mesh.polygons[i].vertices[j])
                    {
                        return false;
                    }
                    const auto neighbor =
                        static_cast<std::uint16_t>(data[offset + 6 + j * 2] | data[offset + 7 + j * 2] << 8);
                    if (neighbor != (mesh.polygons[i].neighbors[j] == std::numeric_limits<std::uint32_t>::max()
                                         ? 0xffffU
                                         : mesh.polygons[i].neighbors[j]))
                    {
                        return false;
                    }
                }
            }
            for (std::size_t i{}; i < mesh.polygons.size(); ++i)
            {
                const auto offset = static_cast<std::size_t>(it->navm->triangles.offset) + i * 16;
                if (static_cast<std::uint16_t>(data[offset + 12] | data[offset + 13] << 8) != mesh.polygons[i].flags)
                {
                    return false;
                }
            }
            const auto trailing = static_cast<std::size_t>(it->navm->trailingData.offset);
            const auto end = trailing + static_cast<std::size_t>(it->navm->trailingData.size);
            if (end > data.size() || end - trailing < 8)
            {
                return false;
            }
            const auto externalCount = Get32(data, trailing);
            if (externalCount > (end - trailing - 4) / 10)
            {
                return false;
            }
            // Check the edges that actually consume the external table, including
            // retained authored entries. Emitted targets use their final triangle space.
            for (const auto &polygon : mesh.polygons)
            {
                for (std::size_t edge{}; edge < 3; ++edge)
                {
                    if (!(polygon.flags & (1U << edge)))
                    {
                        continue;
                    }
                    const auto index = polygon.neighbors[edge];
                    if (index >= externalCount)
                    {
                        return false;
                    }
                    const auto offset = trailing + 4 + index * 10;
                    const auto targetId = Get32(data, offset + 4);
                    const auto targetTriangle = static_cast<std::uint16_t>(data[offset + 8] | data[offset + 9] << 8);
                    const auto target = std::find_if(expected.begin(), expected.end(),
                                                     [&](const auto &item) { return item.first == targetId; });
                    if (target != expected.end() && targetTriangle >= target->second.polygons.size())
                    {
                        return false;
                    }
                }
            }
            const auto doorAt = trailing + 4 + externalCount * 10;
            if (end - doorAt < 4)
            {
                return false;
            }
            const auto doorCount = Get32(data, doorAt);
            const auto doors = doorsByMesh.find(originalId);
            const auto &expectedDoors =
                doors == doorsByMesh.end() ? std::vector<std::pair<std::uint16_t, std::uint32_t>>{} : doors->second;
            if (doors != doorsByMesh.end() && doorCount != expectedDoors.size())
            {
                return false;
            }
            if (doorCount > (end - doorAt - 4) / 10)
            {
                return false;
            }
            for (std::size_t i{}; i < expectedDoors.size(); ++i)
            {
                const auto offset = doorAt + 4 + i * 10;
                const auto triangle = static_cast<std::uint16_t>(data[offset] | data[offset + 1] << 8);
                if (triangle != expectedDoors[i].first || Get32(data, offset + 2) != PathingDoorCrc ||
                    Get32(data, offset + 6) != expectedDoors[i].second)
                {
                    return false;
                }
            }
            for (const auto &[meshId, externalIndex, targetId, targetTriangle] : expectedLinks)
            {
                if (meshId == originalId)
                {
                    if (externalIndex >= externalCount)
                    {
                        return false;
                    }
                    const auto offset = trailing + 4 + externalIndex * 10;
                    if (Get32(data, offset) != 0 || Get32(data, offset + 4) != targetId ||
                        static_cast<std::uint16_t>(data[offset + 8] | data[offset + 9] << 8) != targetTriangle)
                    {
                        return false;
                    }
                }
            }
        }
        return true;
    }
} // namespace

bool navmesh::skyrim::WriteNavmeshOverrides(const std::filesystem::path &outputDirectory,
                                            const std::vector<std::filesystem::path> &inputPlugins,
                                            const ResolvedLoadOrder &resolved,
                                            const std::vector<NavmeshReplacement> &replacements,
                                            std::filesystem::path &writtenPath, std::string &error,
                                            const std::string &copyPlugin)
{
    const auto fail = [&](const char *reason)
    {
        error = reason;
        return false;
    };
    if (replacements.empty())
    {
        return fail("No NAVM replacements were supplied.");
    }
    std::map<std::uint32_t, const NavmeshReplacement *> byCell;
    std::map<std::uint32_t, core::NavMesh> generatedMeshes;
    const auto existingNavmeshCells = CellsWithExistingNavmesh(resolved);
    std::size_t newNavmeshCount{};
    for (const auto &item : replacements)
    {
        if (!item.cell || !item.candidate || !byCell.emplace(item.cell->id, &item).second)
        {
            return fail("Batch contains an invalid or duplicate CELL.");
        }
        if (item.cell->navMeshes.empty())
        {
            if (existingNavmeshCells.contains(item.cell->id))
            {
                return fail("A selected CELL has existing NAVM records without supported geometry.");
            }
            const auto *cellRecord = resolved.FindWinning(item.cell->id);
            if (!cellRecord || cellRecord->type != "CELL" || !cellRecord->raw || cellRecord->groupHeaders.empty() ||
                (cellRecord->raw->flags & 0x20U))
            {
                return fail("A new NAVM requires a live CELL with supported group placement.");
            }
            ++newNavmeshCount;
            continue;
        }
        const auto primary =
            std::max_element(item.cell->navMeshes.begin(), item.cell->navMeshes.end(),
                             [](const auto &a, const auto &b) { return a.polygons.size() < b.polygons.size(); });
        generatedMeshes.emplace(primary->id, item.candidate->mesh);
    }
    std::vector<const ResolvedRecord *> records;
    std::map<std::uint32_t, const ResolvedRecord *> neighborRecords;
    std::map<std::uint32_t, core::NavMesh> neighborMeshes;
    std::vector<std::string> doorOwners;
    std::map<std::uint32_t, std::vector<std::pair<std::uint16_t, std::uint32_t>>> doorsByCell;
    for (const auto &replacement : replacements)
    {
        const auto &cell = *replacement.cell;
        const auto &candidate = *replacement.candidate;
        if (!candidate.topology.valid || candidate.mesh.vertices.empty() || candidate.mesh.polygons.empty())
        {
            return fail("Generated NAVM is empty or has invalid topology.");
        }
        if (candidate.mesh.vertices.size() > 65535 || candidate.mesh.polygons.size() > 65535)
        {
            return fail("Generated NAVM exceeds 16-bit vertex or triangle indices.");
        }
        std::vector<std::pair<std::uint16_t, std::uint32_t>> doorEntries;
        for (const auto &exit : candidate.exits)
        {
            if (!exit.polygon)
            {
                continue;
            }
            if (*exit.polygon >= candidate.mesh.polygons.size())
            {
                return fail("An entrance names an invalid candidate triangle.");
            }
            const auto *reference = resolved.FindWinning(exit.referenceId);
            if (!reference || reference->type != "REFR" || reference->origins.empty() || !reference->raw ||
                (reference->raw->flags & ((1U << 5) | (1U << 11))) != 0 || reference->referencedFormIds.empty())
            {
                return fail("A matched entrance is not an enabled placed door.");
            }
            const auto *base = resolved.FindWinning(reference->referencedFormIds.front());
            if (!base || base->type != "DOOR")
            {
                return fail("A matched entrance does not reference a DOOR base.");
            }
            doorEntries.emplace_back(static_cast<std::uint16_t>(*exit.polygon), exit.referenceId);
            const auto &owner = reference->origins.front().plugin;
            if (std::none_of(doorOwners.begin(), doorOwners.end(),
                             [&](const auto &name) { return SameName(name, owner); }))
            {
                doorOwners.push_back(owner);
            }
        }
        std::sort(doorEntries.begin(), doorEntries.end());
        doorEntries.erase(std::unique(doorEntries.begin(), doorEntries.end()), doorEntries.end());
        if (cell.exteriorCoordinates)
        {
            // Authored portals may lie just beyond the nominal CELL boundary.
            // Preserve their exact endpoints, but never relax bounds for other vertices.
            std::set<std::uint32_t> portalVertices;
            for (const auto &link : candidate.borderLinks)
            {
                if (link.polygon < candidate.mesh.polygons.size() && link.edge < 3)
                {
                    const auto &polygon = candidate.mesh.polygons[link.polygon];
                    portalVertices.insert(polygon.vertices[link.edge]);
                    portalVertices.insert(polygon.vertices[(link.edge + 1) % 3]);
                }
            }
            for (std::size_t index{}; index < candidate.mesh.vertices.size(); ++index)
            {
                const auto &vertex = candidate.mesh.vertices[index];
                const auto [x, y] = *cell.exteriorCoordinates;
                const auto tolerance =
                    portalVertices.contains(static_cast<std::uint32_t>(index)) ? core::AuthoredBorderTolerance : 0.0F;
                if (vertex.x < static_cast<float>(x) * 4096.0F - tolerance ||
                    vertex.x > (static_cast<float>(x) + 1.0F) * 4096.0F + tolerance ||
                    vertex.y < static_cast<float>(y) * 4096.0F - tolerance ||
                    vertex.y > (static_cast<float>(y) + 1.0F) * 4096.0F + tolerance)
                {
                    return fail("Generated NAVM extends outside the selected exterior CELL.");
                }
            }
        }
        for (const auto &mesh : cell.navMeshes)
        {
            const auto *navm = resolved.FindWinning(mesh.id);
            if (!navm || navm->type != "NAVM" || !navm->raw || !navm->navm || !navm->navm->supported)
            {
                return fail("A selected NAVM has an unsupported format.");
            }
            if (navm->groupHeaders.empty() || navm->cellFormId != cell.id)
            {
                return fail("A selected NAVM has no supported cell group placement.");
            }
            for (const auto &sub : navm->raw->subrecords)
            {
                if (sub.type != "NVNM" && sub.type != "EDID")
                {
                    return fail("An existing NAVM has additional authored subrecords that cannot be remapped.");
                }
            }
            records.push_back(navm);
        }
        const auto *selectedCellRecord = resolved.FindWinning(cell.id);
        for (const auto &link : candidate.borderLinks)
        {
            if (link.polygon >= candidate.mesh.polygons.size() || link.edge >= 3 || link.neighborEdge >= 3)
            {
                return fail("A border portal names an invalid triangle edge.");
            }
            const auto *record = resolved.FindWinning(link.neighborNavmeshId);
            if (!record || record->type != "NAVM" || !record->raw || !record->navm || !record->navm->supported ||
                !record->cellFormId || *record->cellFormId == cell.id || record->groupHeaders.empty())
            {
                return fail("A border portal has no supported neighboring NAVM.");
            }
            for (const auto &sub : record->raw->subrecords)
            {
                if (sub.type != "NVNM" && sub.type != "EDID")
                {
                    return fail("A neighboring NAVM has additional authored subrecords that cannot be remapped.");
                }
            }
            const auto *neighborCell = resolved.FindWinning(*record->cellFormId);
            if (!selectedCellRecord || !neighborCell || !selectedCellRecord->worldspaceFormId ||
                neighborCell->worldspaceFormId != selectedCellRecord->worldspaceFormId)
            {
                return fail("A border portal crosses worldspaces.");
            }
            if (!neighborMeshes.contains(link.neighborNavmeshId))
            {
                const auto owner = std::find_if(resolved.cells.begin(), resolved.cells.end(),
                                                [&](const auto &item) { return item.id == *record->cellFormId; });
                if (owner == resolved.cells.end())
                {
                    return fail("A border portal has no resolved neighboring CELL.");
                }
                if (!cell.exteriorCoordinates || !owner->exteriorCoordinates ||
                    std::abs((*cell.exteriorCoordinates)[0] - (*owner->exteriorCoordinates)[0]) +
                            std::abs((*cell.exteriorCoordinates)[1] - (*owner->exteriorCoordinates)[1]) !=
                        1)
                {
                    return fail("A border portal does not target an adjacent exterior CELL.");
                }
                const auto mesh = std::find_if(owner->navMeshes.begin(), owner->navMeshes.end(),
                                               [&](const auto &item) { return item.id == link.neighborNavmeshId; });
                if (mesh == owner->navMeshes.end())
                {
                    return fail("A border portal has no resolved neighboring mesh.");
                }
                neighborMeshes.emplace(link.neighborNavmeshId, *mesh);
                neighborRecords.emplace(link.neighborNavmeshId, record);
            }
            const auto generated = generatedMeshes.find(link.neighborNavmeshId);
            if (byCell.contains(*record->cellFormId) && generated == generatedMeshes.end())
            {
                return fail("A generated border targets a replaced secondary NAVM instead of the generated primary.");
            }
            const auto &neighbor =
                generated != generatedMeshes.end() ? generated->second : neighborMeshes.at(link.neighborNavmeshId);
            if (link.neighborPolygon >= neighbor.polygons.size() || link.neighborPolygon > 65535)
            {
                return fail("A border portal names an invalid neighboring triangle.");
            }
            const auto &target = neighbor.polygons[link.neighborPolygon];
            const auto &source = candidate.mesh.polygons[link.polygon];
            if (source.vertices[link.edge] >= candidate.mesh.vertices.size() ||
                source.vertices[(link.edge + 1) % 3] >= candidate.mesh.vertices.size() ||
                target.vertices[link.neighborEdge] >= neighbor.vertices.size() ||
                target.vertices[(link.neighborEdge + 1) % 3] >= neighbor.vertices.size())
            {
                return fail("A neighboring border triangle has invalid vertices.");
            }
            const auto a = candidate.mesh.vertices[source.vertices[link.edge]];
            const auto b = candidate.mesh.vertices[source.vertices[(link.edge + 1) % 3]];
            const auto c = neighbor.vertices[target.vertices[link.neighborEdge]];
            const auto d = neighbor.vertices[target.vertices[(link.neighborEdge + 1) % 3]];
            const auto near = [](core::Vec3 p, core::Vec3 q)
            { return std::abs(p.x - q.x) <= 1.0F && std::abs(p.y - q.y) <= 1.0F && std::abs(p.z - q.z) <= 1.0F; };
            if (!near(a, d) || !near(b, c))
            {
                return fail("A border portal does not share its neighboring edge.");
            }
            const auto borderX = (*cell.exteriorCoordinates)[0] * 4096.0F;
            const auto borderXMax = ((*cell.exteriorCoordinates)[0] + 1) * 4096.0F;
            const auto borderY = (*cell.exteriorCoordinates)[1] * 4096.0F;
            const auto borderYMax = ((*cell.exteriorCoordinates)[1] + 1) * 4096.0F;
            const auto onBorder = [](float first, float second, float boundary)
            {
                return std::abs(first - boundary) <= core::AuthoredBorderTolerance &&
                       std::abs(second - boundary) <= core::AuthoredBorderTolerance;
            };
            if (!(onBorder(a.x, b.x, borderX) || onBorder(a.x, b.x, borderXMax) || onBorder(a.y, b.y, borderY) ||
                  onBorder(a.y, b.y, borderYMax)))
            {
                return fail("A border portal is not on the selected CELL boundary.");
            }
        }
        doorsByCell.emplace(cell.id, std::move(doorEntries));
    }
    // Generated targets are serialized once. Their reciprocal portals must name
    // generated triangle indices, never triangle indices in replaced geometry.
    for (const auto &replacement : replacements)
    {
        for (const auto &link : replacement.candidate->borderLinks)
        {
            const auto target = generatedMeshes.find(link.neighborNavmeshId);
            if (target == generatedMeshes.end())
            {
                continue;
            }
            const auto *record = resolved.FindWinning(link.neighborNavmeshId);
            const auto other = record && record->cellFormId ? byCell.find(*record->cellFormId) : byCell.end();
            if (replacement.cell->navMeshes.empty())
            {
                return fail("A rebuilt neighbor cannot link back to a new NAVM without a generated identity.");
            }
            const auto primary =
                std::max_element(replacement.cell->navMeshes.begin(), replacement.cell->navMeshes.end(),
                                 [](const auto &a, const auto &b) { return a.polygons.size() < b.polygons.size(); });
            if (other == byCell.end() ||
                std::none_of(other->second->candidate->borderLinks.begin(), other->second->candidate->borderLinks.end(),
                             [&](const auto &reverse)
                             {
                                 return reverse.neighborNavmeshId == primary->id &&
                                        reverse.polygon == link.neighborPolygon && reverse.edge == link.neighborEdge &&
                                        reverse.neighborPolygon == link.polygon && reverse.neighborEdge == link.edge;
                             }))
            {
                return fail("Generated border portal has no reciprocal generated target.");
            }
        }
    }
    for (const auto &[id, mesh] : generatedMeshes)
    {
        neighborMeshes.erase(id);
        neighborRecords.erase(id);
    }
    if (!CollectIncomingNeighbors(resolved, inputPlugins, records, neighborRecords, neighborMeshes))
    {
        return fail("Cannot safely collect authored portals targeting replaced NAVMs.");
    }
    std::vector<SourceInfo> sources;
    std::vector<std::string> requiredSources = doorOwners;
    if (!copyPlugin.empty())
    {
        requiredSources.push_back(copyPlugin);
    }
    for (const auto &replacement : replacements)
    {
        if (replacement.cell->navMeshes.empty())
        {
            requiredSources.push_back(resolved.FindWinning(replacement.cell->id)->winning.plugin);
        }
    }
    for (const auto *navm : records)
    {
        requiredSources.push_back(navm->winning.plugin);
    }
    for (const auto &[id, navm] : neighborRecords)
    {
        requiredSources.push_back(navm->winning.plugin);
    }
    for (const auto &name : requiredSources)
    {
        if (std::any_of(sources.begin(), sources.end(),
                        [&](const auto &source) { return SameName(source.name, name); }))
        {
            continue;
        }
        const auto path = std::find_if(inputPlugins.begin(), inputPlugins.end(),
                                       [&](const auto &p) { return SameName(p.filename().string(), name); });
        if (path == inputPlugins.end())
        {
            return fail("Winning NAVM source plugin is absent from the input paths.");
        }
        std::ifstream file(*path, std::ios::binary);
        Bytes bytes(24);
        file.read(reinterpret_cast<char *>(bytes.data()), 24);
        if (!file || std::memcmp(bytes.data(), "TES4", 4) != 0)
        {
            return fail("Cannot read winning NAVM source plugin.");
        }
        const auto payloadSize = Get32(bytes, 4);
        std::error_code sizeError;
        const auto fileSize = std::filesystem::file_size(*path, sizeError);
        if (sizeError || fileSize < 24 || payloadSize > fileSize - 24 || payloadSize > 256ULL * 1024ULL * 1024ULL)
        {
            return fail("Source TES4 payload exceeds its file or the safe decoded record size.");
        }
        bytes.resize(24 + static_cast<std::size_t>(payloadSize));
        file.read(reinterpret_cast<char *>(bytes.data() + 24), payloadSize);
        if (!file)
        {
            return fail("Cannot read source TES4 payload.");
        }
        std::vector<std::string> sourceMasters;
        if (!ReadMasterNames(bytes, sourceMasters))
        {
            return fail("Cannot read source master list.");
        }
        sources.push_back({name, std::move(bytes), std::move(sourceMasters), *path});
    }
    std::vector<std::string> masters;
    for (const auto &path : inputPlugins)
    {
        const auto name = path.filename().string();
        const auto required =
            std::any_of(sources.begin(), sources.end(),
                        [&](const auto &source)
                        {
                            return SameName(source.name, name) ||
                                   std::any_of(source.masters.begin(), source.masters.end(),
                                               [&](const auto &master) { return SameName(master, name); });
                        });
        if (required &&
            std::none_of(masters.begin(), masters.end(), [&](const auto &master) { return SameName(master, name); }))
        {
            masters.push_back(name);
        }
    }
    for (const auto &source : sources)
    {
        if (std::none_of(masters.begin(), masters.end(),
                         [&](const auto &master) { return SameName(master, source.name); }))
        {
            return fail("A NAVM source is absent from the input load order.");
        }
        for (const auto &required : source.masters)
        {
            if (std::none_of(masters.begin(), masters.end(),
                             [&](const auto &master) { return SameName(master, required); }))
            {
                return fail("A required source master is absent from the input load order.");
            }
        }
    }
    std::optional<detail::PluginCopy> copy;
    std::string copiedName;
    if (!copyPlugin.empty())
    {
        const auto selected = std::find_if(sources.begin(), sources.end(),
                                           [&](const auto &source) { return SameName(source.name, copyPlugin); });
        if (selected != sources.end())
        {
            std::ifstream stream(selected->path, std::ios::binary | std::ios::ate);
            const auto size = stream.tellg();
            if (size < 0)
            {
                return fail("Cannot read selected plugin copy input.");
            }
            selected->bytes.resize(static_cast<std::size_t>(size));
            stream.seekg(0);
            stream.read(reinterpret_cast<char *>(selected->bytes.data()), size);
            if (!stream)
            {
                return fail("Cannot read selected plugin copy input.");
            }
        }
        detail::PluginCopy prepared;
        if (selected == sources.end() || !detail::PreparePluginCopy(selected->bytes, prepared, error))
        {
            return false;
        }
        // Keep the self slot virtual while reusing NAVM rebasing. It is never emitted
        // as a TES4 master, and arbitrary copied payloads retain their original indices.
        copiedName = selected->name;
        const auto selectedPath = std::find_if(inputPlugins.begin(), inputPlugins.end(), [&](const auto &inputPath)
                                               { return SameName(inputPath.filename().string(), copiedName); });
        copiedName = selectedPath->filename().string();
        masters = prepared.masters;
        masters.push_back(copiedName);
        copy = std::move(prepared);
        // Record identities must remain meaningful when the copy replaces the source.
        // Winning overrides may come from other plugins, but their introducing owners
        // must be addressable through this unchanged table before serialization starts.
        const auto representable = [&](std::uint32_t id)
        { return RebaseResolvedFormId(resolved, masters, id).has_value(); };
        for (const auto *record : records)
        {
            if (!representable(record->formId))
            {
                return fail("Copy export requires NAVM owners in the selected plugin's existing master table. "
                            "Use the NAVM-only patch export for additional dependencies.");
            }
        }
        for (const auto &[id, record] : neighborRecords)
        {
            if (!representable(id))
            {
                return fail("Copy export requires neighboring NAVM owners in the selected plugin's existing master "
                            "table. Use the NAVM-only patch export for additional dependencies.");
            }
        }
    }
    for (auto &[cellId, entries] : doorsByCell)
    {
        for (auto &[triangle, doorId] : entries)
        {
            const auto rebased = RebaseResolvedFormId(resolved, masters, doorId);
            if (!rebased)
            {
                return fail("Cannot rebase an entrance door reference.");
            }
            doorId = *rebased;
        }
    }
    if (masters.size() > (copy ? 255U : 254U))
    {
        return fail("Too many masters for a generated plugin.");
    }
    // New identities belong to the output's self slot. Copies keep their allocation
    // cursor and format; patches choose a full/light format based on their new records.
    const std::uint32_t firstNewObjectId = copy ? copy->nextObjectId : 0x800U;
    const bool light =
        copy ? (copy->flags & 0x200U) != 0 : masters.size() <= 253 && newNavmeshCount <= 0x1000U - firstNewObjectId;
    const auto objectIdLimit = light ? 0x1000U : 0x1000000U;
    if (newNavmeshCount && (firstNewObjectId >= objectIdLimit || newNavmeshCount > objectIdLimit - firstNewObjectId))
    {
        return fail("New NAVM identities do not fit the output plugin's full/light format.");
    }
    const auto selfIndex = static_cast<std::uint32_t>(masters.size() - (copy ? 1 : 0));
    std::uint32_t nextObjectId = firstNewObjectId;
    for (const auto &master : masters)
    {
        const auto it = std::find_if(inputPlugins.begin(), inputPlugins.end(),
                                     [&](const auto &p) { return SameName(p.filename().string(), master); });
        if (it == inputPlugins.end())
        {
            return fail("A copied plugin master is absent from the input load order.");
        }
        std::ifstream file(*it, std::ios::binary);
        std::array<std::uint8_t, 12> header{};
        file.read(reinterpret_cast<char *>(header.data()), static_cast<std::streamsize>(header.size()));
        if (!file || std::memcmp(header.data(), "TES4", 4) != 0)
        {
            return fail("Cannot read a required source master.");
        }
    }
    const auto path = outputDirectory / (copy ? copiedName : "generated-navmesh.esp");
    if (std::filesystem::exists(path))
    {
        return fail("Plugin output already exists; choose an empty output folder.");
    }
    if (!copy && std::filesystem::exists(outputDirectory / "generated-navmesh.esl"))
    {
        return fail("An older generated-navmesh.esl exists in the output folder; choose an empty output folder.");
    }
    Group root;
    std::vector<std::pair<std::uint32_t, core::NavMesh>> expected;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> expectedCells;
    std::vector<std::tuple<std::uint32_t, std::uint16_t, std::uint32_t, std::uint16_t>> expectedLinks;
    std::map<std::uint32_t, std::vector<std::pair<std::uint16_t, std::uint32_t>>> doorsByMesh;
    std::map<std::uint32_t, std::uint32_t> outputPrimaryByCell;
    for (const auto &replacement : replacements)
    {
        const auto &cell = *replacement.cell;
        const auto &candidate = *replacement.candidate;
        const auto &doorEntries = doorsByCell.at(cell.id);
        const auto primary = static_cast<std::size_t>(
            std::distance(cell.navMeshes.begin(), std::max_element(cell.navMeshes.begin(), cell.navMeshes.end(),
                                                                   [](const auto &a, const auto &b)
                                                                   { return a.polygons.size() < b.polygons.size(); })));
        core::NavMesh linkedMesh = candidate.mesh;
        std::vector<std::pair<std::uint32_t, std::uint16_t>> outgoing;
        for (const auto &link : candidate.borderLinks)
        {
            const auto target = RebaseResolvedFormId(resolved, masters, link.neighborNavmeshId);
            if (!target)
            {
                return fail("Cannot rebase a neighboring NAVM FormID.");
            }
            auto &face = linkedMesh.polygons[link.polygon];
            if (face.neighbors[link.edge] != std::numeric_limits<std::uint32_t>::max() || outgoing.size() >= 65535)
            {
                return fail("A candidate border edge has more than one portal.");
            }
            face.neighbors[link.edge] = static_cast<std::uint32_t>(outgoing.size());
            face.flags |= static_cast<std::uint16_t>(1U << link.edge);
            outgoing.emplace_back(*target, static_cast<std::uint16_t>(link.neighborPolygon));
        }
        // A candidate uses one triangle-index space; keeping it in one NAVM avoids invalid cross-record indices.
        std::optional<std::uint32_t> outputCellGroup;
        const bool newNavmesh = cell.navMeshes.empty();
        for (std::size_t index{}; index < std::max<std::size_t>(1, cell.navMeshes.size()); ++index)
        {
            const auto *navm =
                newNavmesh ? resolved.FindWinning(cell.id) : resolved.FindWinning(cell.navMeshes[index].id);
            const auto sourceIt = std::find_if(sources.begin(), sources.end(), [&](const auto &source)
                                               { return SameName(source.name, navm->winning.plugin); });
            const auto &source = *sourceIt;
            const core::NavMesh emptyMesh;
            const core::NavMesh &mesh = index == primary ? linkedMesh : emptyMesh;
            Bytes nvnm;
            if (newNavmesh)
            {
                // Encode the same world-space location prefix as authored NAVM.
                // CELL group labels remain source-local until placement is rebased below.
                U32(nvnm, 12);
                U32(nvnm, 0);
                const auto world = navm->worldspaceFormId
                                       ? RebaseResolvedFormId(resolved, masters, *navm->worldspaceFormId)
                                       : std::optional<std::uint32_t>{0};
                if (!world || (cell.exteriorCoordinates && !*world && !navm->worldspaceFormId))
                {
                    return fail("Cannot resolve new NAVM worldspace.");
                }
                U32(nvnm, *world);
                if (cell.exteriorCoordinates)
                {
                    const auto [x, y] = *cell.exteriorCoordinates;
                    if (x < std::numeric_limits<std::int16_t>::min() || x > std::numeric_limits<std::int16_t>::max() ||
                        y < std::numeric_limits<std::int16_t>::min() || y > std::numeric_limits<std::int16_t>::max())
                    {
                        return fail("New NAVM exterior coordinates exceed the format range.");
                    }
                    U16(nvnm, static_cast<std::uint16_t>(y));
                    U16(nvnm, static_cast<std::uint16_t>(x));
                }
                else
                {
                    const auto cellId = RebaseResolvedFormId(resolved, masters, cell.id);
                    if (!cellId)
                    {
                        return fail("Cannot resolve new NAVM interior CELL.");
                    }
                    U32(nvnm, *cellId);
                }
            }
            else
            {
                const auto &old = navm->raw->decodedPayload;
                const auto &layout = *navm->navm;
                nvnm.insert(nvnm.end(), old.begin() + layout.header.offset, old.begin() + layout.header.offset + 16);
            }
            if (!newNavmesh && Get32(nvnm, 8))
            {
                const auto world = RebaseFormId(source, masters, Get32(nvnm, 8));
                if (!world)
                {
                    return fail("Cannot rebase NAVM worldspace FormID.");
                }
                Set32(nvnm, 8, *world);
            }
            else if (!newNavmesh && !cell.exteriorCoordinates)
            {
                const auto interiorCell = RebaseFormId(source, masters, Get32(nvnm, 12));
                if (!interiorCell)
                {
                    return fail("Cannot rebase NAVM interior CELL FormID.");
                }
                Set32(nvnm, 12, *interiorCell);
            }
            U32(nvnm, static_cast<std::uint32_t>(mesh.vertices.size()));
            for (const auto &vertex : mesh.vertices)
            {
                if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) || !std::isfinite(vertex.z))
                {
                    return fail("Generated NAVM contains non-finite coordinates.");
                }
                Float(nvnm, vertex.x);
                Float(nvnm, vertex.y);
                Float(nvnm, vertex.z);
            }
            U32(nvnm, static_cast<std::uint32_t>(mesh.polygons.size()));
            for (const auto &polygon : mesh.polygons)
            {
                for (const auto vertex : polygon.vertices)
                {
                    if (vertex >= mesh.vertices.size())
                    {
                        return fail("Generated NAVM has an invalid vertex index.");
                    }
                    U16(nvnm, static_cast<std::uint16_t>(vertex));
                }
                for (std::size_t side{}; side < 3; ++side)
                {
                    const auto neighbor = polygon.neighbors[side];
                    if (neighbor != std::numeric_limits<std::uint32_t>::max() && !(polygon.flags & (1U << side)) &&
                        neighbor >= mesh.polygons.size())
                    {
                        return fail("Generated NAVM has an invalid neighbor index.");
                    }
                    U16(nvnm, neighbor == std::numeric_limits<std::uint32_t>::max()
                                  ? 0xffffU
                                  : static_cast<std::uint16_t>(neighbor));
                }
                U16(nvnm, polygon.flags);
                U16(nvnm, 0);
            }
            U32(nvnm, index == primary ? static_cast<std::uint32_t>(outgoing.size()) : 0U);
            if (index == primary)
            {
                for (const auto &[target, triangle] : outgoing)
                {
                    U32(nvnm, 0);
                    U32(nvnm, target);
                    U16(nvnm, triangle);
                }
            }
            U32(nvnm, index == primary ? static_cast<std::uint32_t>(doorEntries.size()) : 0U);
            if (index == primary)
            {
                for (const auto &[triangle, doorId] : doorEntries)
                {
                    U16(nvnm, triangle);
                    U32(nvnm, PathingDoorCrc);
                    U32(nvnm, doorId);
                }
            }
            U32(nvnm, 0);
            float minX{}, minY{}, minZ{}, maxX{}, maxY{}, maxZ{};
            if (!mesh.vertices.empty())
            {
                minX = minY = minZ = std::numeric_limits<float>::max();
                maxX = maxY = maxZ = std::numeric_limits<float>::lowest();
                for (const auto &v : mesh.vertices)
                {
                    minX = std::min(minX, v.x);
                    minY = std::min(minY, v.y);
                    minZ = std::min(minZ, v.z);
                    maxX = std::max(maxX, v.x);
                    maxY = std::max(maxY, v.y);
                    maxZ = std::max(maxZ, v.z);
                }
            }
            U32(nvnm, 1);
            Float(nvnm, maxX - minX);
            Float(nvnm, maxY - minY);
            for (float bound : {minX, minY, minZ, maxX, maxY, maxZ})
            {
                Float(nvnm, bound);
            }
            U32(nvnm, static_cast<std::uint32_t>(mesh.polygons.size()));
            for (std::size_t triangle{}; triangle < mesh.polygons.size(); ++triangle)
            {
                U16(nvnm, static_cast<std::uint16_t>(triangle));
            }
            Bytes payload;
            if (newNavmesh)
            {
                PutSubrecord(payload, "NVNM", nvnm);
            }
            else
            {
                for (const auto &sub : navm->raw->subrecords)
                {
                    if (sub.type == "NVNM")
                    {
                        PutSubrecord(payload, "NVNM", nvnm);
                    }
                    else
                    {
                        payload.insert(payload.end(), sub.encodedBytes.begin(), sub.encodedBytes.end());
                    }
                }
            }
            Bytes bytes = SourceRecord(source, *navm);
            if (bytes.size() < 24)
            {
                return fail("Source NAVM record range is invalid.");
            }
            bytes.resize(24);
            if (newNavmesh)
            {
                std::copy_n("NAVM", 4, bytes.begin());
                Set32(bytes, 8, 0);
            }
            Set32(bytes, 4, static_cast<std::uint32_t>(payload.size()));
            Set32(bytes, 8, Get32(bytes, 8) & ~(0x40000U | 0x20U));
            const auto rebasedId = newNavmesh ? std::optional<std::uint32_t>{selfIndex << 24 | nextObjectId++}
                                              : RebaseFormId(source, masters, Get32(bytes, 12));
            if (!rebasedId)
            {
                return fail("Cannot rebase source NAVM FormID.");
            }
            Set32(bytes, 12, *rebasedId);
            bytes.insert(bytes.end(), payload.begin(), payload.end());
            expected.emplace_back(Get32(bytes, 12), mesh);
            const auto cellId = RebaseResolvedFormId(resolved, masters, cell.id);
            if (!cellId)
            {
                return fail("Cannot rebase selected CELL FormID.");
            }
            expectedCells.emplace_back(Get32(bytes, 12), *cellId);
            if (index == primary)
            {
                for (std::size_t i{}; i < outgoing.size(); ++i)
                {
                    expectedLinks.emplace_back(Get32(bytes, 12), static_cast<std::uint16_t>(i), outgoing[i].first,
                                               outgoing[i].second);
                }
            }
            ResolvedRecord placement;
            placement.groupHeaders = navm->groupHeaders;
            if (newNavmesh)
            {
                // The winning CELL's ancestors provide exterior block/sub-block or
                // interior placement. New NAVMs live in its temporary child group.
                for (const auto groupType : {6U, 9U})
                {
                    Bytes header{'G', 'R', 'U', 'P'};
                    U32(header, 24);
                    U32(header, navm->winning.formId);
                    U32(header, groupType);
                    header.resize(24);
                    std::array<std::uint8_t, 24> groupHeader{};
                    std::copy(header.begin(), header.end(), groupHeader.begin());
                    placement.groupHeaders.push_back(groupHeader);
                }
            }
            for (auto &header : placement.groupHeaders)
            {
                Bytes group(header.begin(), header.end());
                const auto groupType = Get32(group, 12);
                if (groupType == 1 || groupType == 6 || groupType == 8 || groupType == 9 || groupType == 10)
                {
                    const auto label = RebaseFormId(source, masters, Get32(group, 8));
                    if (!label)
                    {
                        return fail("Cannot rebase NAVM group placement.");
                    }
                    Set32(group, 8, *label);
                    if (groupType == 6)
                    {
                        if (outputCellGroup && *outputCellGroup != *label)
                        {
                            return fail("NAVM records do not share the selected CELL group.");
                        }
                        outputCellGroup = *label;
                    }
                    std::copy(group.begin(), group.end(), header.begin());
                }
            }
            Add(root, placement, std::move(bytes));
        }
        const auto primaryFormId = newNavmesh ? std::optional<std::uint32_t>{expected.back().first}
                                              : RebaseResolvedFormId(resolved, masters, cell.navMeshes[primary].id);
        if (!primaryFormId)
        {
            return fail("Cannot rebase generated NAVM FormID.");
        }
        if (!outputCellGroup)
        {
            return fail("Generated NAVM has no CELL group.");
        }
        doorsByMesh.emplace(*primaryFormId, doorEntries);
        outputPrimaryByCell.emplace(cell.id, *primaryFormId);
    }
    std::set<std::uint32_t> replacedIds;
    for (const auto *replaced : records)
    {
        const auto id = RebaseResolvedFormId(resolved, masters, replaced->formId);
        if (!id)
        {
            return fail("Cannot rebase an incoming portal destination.");
        }
        replacedIds.insert(*id);
    }
    for (auto &[neighborId, mesh] : neighborMeshes)
    {
        const auto *navm = neighborRecords.at(neighborId);
        const auto sourceIt = std::find_if(sources.begin(), sources.end(), [&](const auto &source)
                                           { return SameName(source.name, navm->winning.plugin); });
        const auto &source = *sourceIt;
        const auto &layout = *navm->navm;
        const auto &old = navm->raw->decodedPayload;
        const auto end = static_cast<std::size_t>(layout.trailingData.offset + layout.trailingData.size);
        if (end > old.size())
        {
            return fail("A neighboring NAVM has an invalid trailing section.");
        }
        Bytes nvnm(old.begin() + layout.header.offset, old.begin() + end);
        const auto trailing = static_cast<std::size_t>(layout.trailingData.offset - layout.header.offset);
        if (nvnm.size() - trailing < 8)
        {
            return fail("A neighboring NAVM has no connection counts.");
        }
        auto oldExternalCount = Get32(nvnm, trailing);
        if (oldExternalCount > (nvnm.size() - trailing - 4) / 10)
        {
            return fail("A neighboring NAVM has invalid external links.");
        }
        auto insertion = trailing + 4 + oldExternalCount * 10;
        for (std::uint32_t i{}; i < oldExternalCount; ++i)
        {
            const auto offset = trailing + 4 + i * 10 + 4;
            const auto rebased = RebaseFormId(source, masters, Get32(nvnm, offset));
            if (!rebased)
            {
                return fail("Cannot rebase a neighboring NAVM external link.");
            }
            Set32(nvnm, offset, *rebased);
        }
        if (nvnm.size() - insertion < 4)
        {
            return fail("A neighboring NAVM has no door count.");
        }
        const auto oldDoorCount = Get32(nvnm, insertion);
        if (oldDoorCount > (nvnm.size() - insertion - 4) / 10)
        {
            return fail("A neighboring NAVM has invalid door links.");
        }
        for (std::uint32_t i{}; i < oldDoorCount; ++i)
        {
            const auto offset = insertion + 4 + i * 10 + 6;
            const auto rebased = RebaseFormId(source, masters, Get32(nvnm, offset));
            if (!rebased)
            {
                return fail("Cannot rebase a neighboring NAVM door link.");
            }
            Set32(nvnm, offset, *rebased);
        }
        if (Get32(nvnm, 8))
        {
            const auto world = RebaseFormId(source, masters, Get32(nvnm, 8));
            if (!world)
            {
                return fail("Cannot rebase neighboring NAVM worldspace.");
            }
            Set32(nvnm, 8, *world);
        }
        else
        {
            const auto interior = RebaseFormId(source, masters, Get32(nvnm, 12));
            if (!interior)
            {
                return fail("Cannot rebase neighboring NAVM CELL.");
            }
            Set32(nvnm, 12, *interior);
        }
        if (!RemoveIncomingPortals(nvnm, layout, mesh, replacedIds))
        {
            return fail("An incoming NAVM edge has an invalid authored portal index.");
        }
        oldExternalCount = Get32(nvnm, trailing);
        insertion = trailing + 4 + oldExternalCount * 10;
        Bytes appended;
        for (const auto &replacement : replacements)
        {
            const auto primaryFormId = outputPrimaryByCell.at(replacement.cell->id);
            for (const auto &link : replacement.candidate->borderLinks)
            {
                if (link.neighborNavmeshId == neighborId)
                {
                    auto &face = mesh.polygons[link.neighborPolygon];
                    if (face.flags & (1U << link.neighborEdge))
                    {
                        if (face.neighbors[link.neighborEdge] >= oldExternalCount)
                        {
                            return fail("A neighboring border edge has an invalid authored portal.");
                        }
                        return fail("A neighboring border edge is linked to another NAVM.");
                    }
                    else if (face.neighbors[link.neighborEdge] != 0xffffU &&
                             face.neighbors[link.neighborEdge] != std::numeric_limits<std::uint32_t>::max())
                    {
                        return fail("A neighboring border edge already has an internal link.");
                    }
                    const auto index = oldExternalCount + static_cast<std::uint32_t>(appended.size() / 10);
                    if (index >= 65535)
                    {
                        return fail("A neighboring NAVM has too many external links.");
                    }
                    face.flags |= static_cast<std::uint16_t>(1U << link.neighborEdge);
                    face.neighbors[link.neighborEdge] = index;
                    const auto triangleOffset =
                        static_cast<std::size_t>(layout.triangles.offset - layout.header.offset) +
                        link.neighborPolygon * 16;
                    Set16(nvnm, triangleOffset + 6 + link.neighborEdge * 2, static_cast<std::uint16_t>(index));
                    Set16(nvnm, triangleOffset + 12, face.flags);
                    U32(appended, 0);
                    U32(appended, primaryFormId);
                    U16(appended, static_cast<std::uint16_t>(link.polygon));
                }
            }
        }
        Set32(nvnm, trailing, oldExternalCount + static_cast<std::uint32_t>(appended.size() / 10));
        nvnm.insert(nvnm.begin() + insertion, appended.begin(), appended.end());
        Bytes payload;
        for (const auto &sub : navm->raw->subrecords)
        {
            if (sub.type == "NVNM")
            {
                PutSubrecord(payload, "NVNM", nvnm);
            }
            else
            {
                payload.insert(payload.end(), sub.encodedBytes.begin(), sub.encodedBytes.end());
            }
        }
        Bytes bytes = SourceRecord(source, *navm);
        if (bytes.size() < 24)
        {
            return fail("Neighboring NAVM source range is invalid.");
        }
        bytes.resize(24);
        Set32(bytes, 4, static_cast<std::uint32_t>(payload.size()));
        Set32(bytes, 8, Get32(bytes, 8) & ~(0x40000U | 0x20U));
        const auto rebasedId = RebaseFormId(source, masters, Get32(bytes, 12));
        if (!rebasedId)
        {
            return fail("Cannot rebase neighboring NAVM FormID.");
        }
        Set32(bytes, 12, *rebasedId);
        bytes.insert(bytes.end(), payload.begin(), payload.end());
        expected.emplace_back(*rebasedId, mesh);
        const auto cellId = RebaseResolvedFormId(resolved, masters, *navm->cellFormId);
        if (!cellId)
        {
            return fail("Cannot rebase neighboring CELL FormID.");
        }
        expectedCells.emplace_back(*rebasedId, *cellId);
        for (std::size_t i{}; i < appended.size() / 10; ++i)
        {
            const auto offset = i * 10;
            expectedLinks.emplace_back(*rebasedId, static_cast<std::uint16_t>(oldExternalCount + i),
                                       Get32(appended, offset + 4),
                                       static_cast<std::uint16_t>(appended[offset + 8] | appended[offset + 9] << 8));
        }
        ResolvedRecord placement;
        placement.groupHeaders = navm->groupHeaders;
        for (auto &header : placement.groupHeaders)
        {
            Bytes group(header.begin(), header.end());
            const auto type = Get32(group, 12);
            if (type == 1 || type == 6 || type == 8 || type == 9 || type == 10)
            {
                const auto label = RebaseFormId(source, masters, Get32(group, 8));
                if (!label)
                {
                    return fail("Cannot rebase neighboring NAVM group.");
                }
                Set32(group, 8, *label);
                std::copy(group.begin(), group.end(), header.begin());
            }
        }
        Add(root, placement, std::move(bytes));
    }
    const auto &source = sources.front().bytes;
    Bytes hedr;
    if (source.size() >= 42 && std::memcmp(source.data() + 24, "HEDR", 4) == 0 && source[28] == 12 && source[29] == 0)
    {
        hedr.assign(source.begin() + 30, source.begin() + 42);
    }
    else
    {
        Float(hedr, 1.7F);
        U32(hedr, 0);
        U32(hedr, 0x800);
    }
    Set32(hedr, 4, static_cast<std::uint32_t>(expected.size()));
    if (newNavmeshCount)
    {
        Set32(hedr, 8, nextObjectId);
    }
    Bytes headerPayload;
    PutSubrecord(headerPayload, "HEDR", hedr);
    for (const auto &master : masters)
    {
        Bytes name(master.begin(), master.end());
        name.push_back(0);
        PutSubrecord(headerPayload, "MAST", name);
        PutSubrecord(headerPayload, "DATA", Bytes(8));
    }
    Bytes output(source.begin(), source.begin() + 24);
    Set32(output, 4, static_cast<std::uint32_t>(headerPayload.size()));
    Set32(output, 8, light ? 0x200U : 0U);
    output.insert(output.end(), headerPayload.begin(), headerPayload.end());
    auto groups = Serialize(root);
    output.insert(output.end(), groups.begin(), groups.end());
    auto emittedMasters = masters;
    if (copy)
    {
        if (!detail::MergePluginCopy(*copy, groups, nextObjectId, output, error))
        {
            return false;
        }
        emittedMasters = copy->masters;
    }
    if (!outputDirectory.empty())
    {
        std::filesystem::create_directories(outputDirectory);
    }
    auto temporary = path;
    temporary += ".tmp";
    if (std::filesystem::exists(temporary))
    {
        return fail("Temporary plugin path already exists.");
    }
    {
        std::ofstream file(temporary, std::ios::binary);
        file.write(reinterpret_cast<const char *>(output.data()), static_cast<std::streamsize>(output.size()));
        if (!file)
        {
            std::filesystem::remove(temporary);
            return fail("Cannot write generated plugin.");
        }
    }
    if (!ReadBack(temporary, emittedMasters, light, expected, doorsByMesh, expectedCells, expectedLinks,
                  copy.has_value()))
    {
        std::filesystem::remove(temporary);
        return fail("Generated plugin failed NAVM read-back verification.");
    }
    std::error_code renameError;
    std::filesystem::rename(temporary, path, renameError);
    if (renameError)
    {
        std::filesystem::remove(temporary);
        return fail("Cannot finalize generated plugin.");
    }
    writtenPath = path;
    return true;
}

bool navmesh::skyrim::WriteNavmeshOverride(const std::filesystem::path &outputDirectory,
                                           const std::vector<std::filesystem::path> &inputPlugins,
                                           const ResolvedLoadOrder &resolved, const core::Cell &cell,
                                           const core::CandidateNavMesh &candidate, std::filesystem::path &writtenPath,
                                           std::string &error)
{
    return WriteNavmeshOverrides(outputDirectory, inputPlugins, resolved, {{&cell, &candidate}}, writtenPath, error);
}
