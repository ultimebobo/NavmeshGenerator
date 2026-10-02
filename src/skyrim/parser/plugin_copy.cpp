#include "skyrim/parser/plugin_copy.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <set>
#include <span>

namespace
{
    using Bytes = std::vector<std::uint8_t>;

    std::uint32_t Read32(const Bytes &bytes, std::size_t offset)
    {
        return static_cast<std::uint32_t>(bytes[offset]) | static_cast<std::uint32_t>(bytes[offset + 1]) << 8 |
               static_cast<std::uint32_t>(bytes[offset + 2]) << 16 |
               static_cast<std::uint32_t>(bytes[offset + 3]) << 24;
    }

    void Write32(Bytes &bytes, std::size_t offset, std::uint32_t value)
    {
        for (int shift{}; shift < 32; shift += 8)
        {
            bytes[offset + shift / 8] = static_cast<std::uint8_t>(value >> shift);
        }
    }

    /// Ordered raw records and groups preserve unknown payloads and prerequisite parent placement.
    struct Node
    {
        Bytes bytes;
        std::vector<Node> children;

        bool IsGroup() const
        {
            return std::memcmp(bytes.data(), "GRUP", 4) == 0;
        }
    };

    bool ParseNodes(const Bytes &bytes, std::size_t begin, std::size_t end, std::vector<Node> &nodes,
                    unsigned depth = 0)
    {
        if (depth > 64)
        {
            return false;
        }
        for (auto offset = begin; offset < end;)
        {
            if (end - offset < 24)
            {
                return false;
            }
            const bool group = std::memcmp(bytes.data() + offset, "GRUP", 4) == 0;
            const auto size = static_cast<std::size_t>(Read32(bytes, offset + 4));
            if ((group && size < 24) || size > end - offset - (group ? 0 : 24))
            {
                return false;
            }
            const auto next = offset + size + (group ? 0 : 24);
            Node node{.bytes = Bytes(bytes.begin() + offset, bytes.begin() + (group ? offset + 24 : next))};
            if (group && !ParseNodes(bytes, offset + 24, next, node.children, depth + 1))
            {
                return false;
            }
            nodes.push_back(std::move(node));
            offset = next;
        }
        return true;
    }

    /// Inspect all record headers, including types outside the extraction reader's index.
    bool InspectIdentities(std::span<const Node> nodes, std::uint32_t selfIndex, std::uint32_t &nextObjectId,
                           std::set<std::uint32_t> &identities)
    {
        for (const auto &node : nodes)
        {
            if (node.IsGroup())
            {
                if (!InspectIdentities(node.children, selfIndex, nextObjectId, identities))
                {
                    return false;
                }
                continue;
            }
            const auto id = Read32(node.bytes, 12);
            if (std::memcmp(node.bytes.data(), "TES4", 4) == 0)
            {
                return false;
            }
            if (!identities.insert(id).second || (id >> 24) > selfIndex)
            {
                return false;
            }
            if ((id >> 24) == selfIndex)
            {
                nextObjectId = std::max(nextObjectId, (id & 0xFFFFFFU) + 1);
            }
        }
        return true;
    }

    /// Decode field boundaries, including XXXX sizes, while retaining their encoded byte ranges.
    bool ReadHeaderField(const Bytes &header, std::size_t &offset, std::string &type, std::size_t &dataOffset,
                         std::size_t &size)
    {
        if (header.size() - offset < 6)
        {
            return false;
        }
        type.assign(reinterpret_cast<const char *>(header.data() + offset), 4);
        size = static_cast<std::size_t>(header[offset + 4] | header[offset + 5] << 8);
        offset += 6;
        if (type == "XXXX")
        {
            if (size != 4 || header.size() - offset < 10)
            {
                return false;
            }
            size = Read32(header, offset);
            offset += 4;
            type.assign(reinterpret_cast<const char *>(header.data() + offset), 4);
            offset += 6;
            if (type == "XXXX")
            {
                return false;
            }
        }
        if (size > header.size() - offset)
        {
            return false;
        }
        dataOffset = offset;
        offset += size;
        return true;
    }

    /// Locate encoded TES4 fields without changing their order, strings, or unknown subrecords.
    bool ReadHeader(const Bytes &header, std::vector<std::string> &masters, std::size_t &hedrOffset)
    {
        bool foundHedr{};
        for (std::size_t offset = 24; offset < header.size();)
        {
            std::string type;
            std::size_t dataOffset{}, size{};
            if (!ReadHeaderField(header, offset, type, dataOffset, size))
            {
                return false;
            }
            if (type == "HEDR")
            {
                if (foundHedr || size != 12)
                {
                    return false;
                }
                foundHedr = true;
                hedrOffset = dataOffset;
            }
            else if (type == "MAST")
            {
                const auto end = std::find(header.begin() + dataOffset, header.begin() + dataOffset + size, 0);
                if (end == header.begin() + dataOffset || end == header.begin() + dataOffset + size)
                {
                    return false;
                }
                masters.emplace_back(header.begin() + dataOffset, end);
            }
        }
        return foundHedr;
    }

    /// Master plugins use ONAM to discover overridden CELL children. Keep existing
    /// entries and register generated overrides without changing their master indices.
    bool UpdateOverrideList(Bytes &header, const std::set<std::uint32_t> &generatedIds, std::uint32_t selfIndex)
    {
        Bytes forms;
        std::set<std::uint32_t> registered;
        std::size_t fieldStart = header.size(), fieldEnd = header.size();
        bool found{};
        for (std::size_t offset = 24; offset < header.size();)
        {
            const auto start = offset;
            std::string type;
            std::size_t dataOffset{}, size{};
            if (!ReadHeaderField(header, offset, type, dataOffset, size))
            {
                return false;
            }
            if (type != "ONAM")
            {
                continue;
            }
            if (found || size % 4 != 0)
            {
                return false;
            }
            found = true;
            fieldStart = start;
            fieldEnd = offset;
            forms.assign(header.begin() + dataOffset, header.begin() + offset);
            for (std::size_t index{}; index < forms.size(); index += 4)
            {
                registered.insert(Read32(forms, index));
            }
        }
        if (!found && !(Read32(header, 8) & 1U))
        {
            return true;
        }
        const auto originalSize = forms.size();
        for (const auto id : generatedIds)
        {
            if ((id >> 24) < selfIndex && registered.insert(id).second)
            {
                const auto offset = forms.size();
                forms.resize(offset + 4);
                Write32(forms, offset, id);
            }
        }
        if (forms.size() == originalSize)
        {
            return true;
        }
        Bytes encoded;
        if (forms.size() > std::numeric_limits<std::uint16_t>::max())
        {
            encoded = {'X', 'X', 'X', 'X', 4, 0};
            encoded.resize(10);
            Write32(encoded, 6, static_cast<std::uint32_t>(forms.size()));
        }
        encoded.insert(encoded.end(), {'O', 'N', 'A', 'M'});
        const auto shortSize = forms.size() <= std::numeric_limits<std::uint16_t>::max() ? forms.size() : 0;
        encoded.push_back(static_cast<std::uint8_t>(shortSize));
        encoded.push_back(static_cast<std::uint8_t>(shortSize >> 8));
        encoded.insert(encoded.end(), forms.begin(), forms.end());
        header.erase(header.begin() + fieldStart, header.begin() + fieldEnd);
        header.insert(header.begin() + fieldStart, encoded.begin(), encoded.end());
        Write32(header, 4, static_cast<std::uint32_t>(header.size() - 24));
        return true;
    }

    bool CollectNavigation(const std::vector<Node> &nodes, std::set<std::uint32_t> &ids)
    {
        for (const auto &node : nodes)
        {
            if (node.IsGroup())
            {
                if (!CollectNavigation(node.children, ids))
                {
                    return false;
                }
            }
            else if (std::memcmp(node.bytes.data(), "NAVM", 4) != 0 || !ids.insert(Read32(node.bytes, 12)).second)
            {
                return false;
            }
        }
        return true;
    }

    /// Remove only replaced NAVMs globally, so relocated winning groups cannot leave duplicate identities.
    bool RemoveNavigation(std::vector<Node> &nodes, const std::set<std::uint32_t> &ids)
    {
        for (auto it = nodes.begin(); it != nodes.end();)
        {
            if (it->IsGroup())
            {
                if (!RemoveNavigation(it->children, ids))
                {
                    return false;
                }
                ++it;
            }
            else if (ids.contains(Read32(it->bytes, 12)))
            {
                if (std::memcmp(it->bytes.data(), "NAVM", 4) != 0)
                {
                    return false;
                }
                it = nodes.erase(it);
            }
            else
            {
                ++it;
            }
        }
        return true;
    }

    /// Merge by group label/type; raw parent records stay in their original relative order.
    void MergeNodes(std::vector<Node> &source, std::vector<Node> navigation)
    {
        for (auto &node : navigation)
        {
            const auto match = node.IsGroup()
                                   ? std::find_if(source.begin(), source.end(),
                                                  [&](const auto &existing)
                                                  {
                                                      return existing.IsGroup() &&
                                                             std::equal(node.bytes.begin() + 8, node.bytes.begin() + 16,
                                                                        existing.bytes.begin() + 8);
                                                  })
                                   : source.end();
            if (match == source.end())
            {
                source.push_back(std::move(node));
            }
            else
            {
                MergeNodes(match->children, std::move(node.children));
            }
        }
    }

    bool SerializeNodes(const std::vector<Node> &nodes, Bytes &output, std::uint32_t &recordCount)
    {
        for (const auto &node : nodes)
        {
            const auto start = output.size();
            output.insert(output.end(), node.bytes.begin(), node.bytes.end());
            ++recordCount;
            if (node.IsGroup())
            {
                if (!SerializeNodes(node.children, output, recordCount) ||
                    output.size() - start > std::numeric_limits<std::uint32_t>::max())
                {
                    return false;
                }
                Write32(output, start + 4, static_cast<std::uint32_t>(output.size() - start));
            }
        }
        return true;
    }
} // namespace

bool navmesh::skyrim::offline::detail::PreparePluginCopy(std::vector<std::uint8_t> bytes, PluginCopy &copy,
                                                         std::string &error)
{
    std::vector<Node> nodes;
    std::size_t hedrOffset{};
    PluginCopy prepared;
    if (!ParseNodes(bytes, 0, bytes.size(), nodes) || nodes.empty() || nodes.front().IsGroup() ||
        std::memcmp(nodes.front().bytes.data(), "TES4", 4) != 0 ||
        !ReadHeader(nodes.front().bytes, prepared.masters, hedrOffset) || prepared.masters.size() > 254)
    {
        error = "Cannot copy a plugin with malformed record/group boundaries or TES4 header.";
        return false;
    }
    prepared.flags = Read32(bytes, 8);
    prepared.nextObjectId = std::max(0x800U, Read32(nodes.front().bytes, hedrOffset + 8));
    std::set<std::uint32_t> ids;
    if (!InspectIdentities(std::span<const Node>(nodes).subspan(1), static_cast<std::uint32_t>(prepared.masters.size()),
                           prepared.nextObjectId, ids))
    {
        error = "Cannot copy a plugin with duplicate or unresolved record identities.";
        return false;
    }
    prepared.bytes = std::move(bytes);
    copy = std::move(prepared);
    return true;
}

bool navmesh::skyrim::offline::detail::MergePluginCopy(const PluginCopy &copy,
                                                       const std::vector<std::uint8_t> &navigation,
                                                       std::uint32_t nextObjectId, std::vector<std::uint8_t> &output,
                                                       std::string &error)
{
    std::vector<Node> sourceNodes, generatedNodes;
    std::set<std::uint32_t> ids;
    if (!ParseNodes(copy.bytes, 0, copy.bytes.size(), sourceNodes) ||
        !ParseNodes(navigation, 0, navigation.size(), generatedNodes) || !CollectNavigation(generatedNodes, ids) ||
        !RemoveNavigation(sourceNodes, ids))
    {
        error = "Cannot merge generated navigation into the source plugin structure.";
        return false;
    }
    MergeNodes(sourceNodes, std::move(generatedNodes));
    if (!UpdateOverrideList(sourceNodes.front().bytes, ids, static_cast<std::uint32_t>(copy.masters.size())))
    {
        error = "Cannot update the copied plugin's ONAM override list.";
        return false;
    }
    output.clear();
    std::uint32_t recordCount{};
    if (!SerializeNodes(sourceNodes, output, recordCount))
    {
        error = "Copied plugin exceeds group size limits.";
        return false;
    }
    std::vector<std::string> masters;
    std::size_t hedrOffset{};
    if (!ReadHeader(sourceNodes.front().bytes, masters, hedrOffset))
    {
        error = "Cannot update the copied TES4 header.";
        return false;
    }
    Write32(output, hedrOffset + 4, recordCount - 1);
    Write32(output, hedrOffset + 8, nextObjectId);
    return true;
}
