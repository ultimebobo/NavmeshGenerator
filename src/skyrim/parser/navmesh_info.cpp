#include "skyrim/parser/navmesh_info.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <span>

namespace
{
    using Bytes = std::vector<std::uint8_t>;
    constexpr std::uint32_t PathingCellCrc = 0xA5E9A03CU;

    std::uint16_t Read16(std::span<const std::uint8_t> data, std::size_t offset)
    {
        return static_cast<std::uint16_t>(data[offset] | data[offset + 1] << 8);
    }

    std::uint32_t Read32(std::span<const std::uint8_t> data, std::size_t offset)
    {
        return static_cast<std::uint32_t>(Read16(data, offset)) | static_cast<std::uint32_t>(Read16(data, offset + 2))
                                                                      << 16;
    }

    void Put32(Bytes &data, std::uint32_t value)
    {
        for (int shift{}; shift < 32; shift += 8)
        {
            data.push_back(static_cast<std::uint8_t>(value >> shift));
        }
    }

    void PutFloat(Bytes &data, float value)
    {
        std::uint32_t bits{};
        std::memcpy(&bits, &value, sizeof(bits));
        Put32(data, bits);
    }

    void PutSubrecord(Bytes &payload, const char *type, const Bytes &data)
    {
        const bool extended = data.size() > std::numeric_limits<std::uint16_t>::max();
        if (extended)
        {
            payload.insert(payload.end(), {'X', 'X', 'X', 'X', 4, 0});
            Put32(payload, static_cast<std::uint32_t>(data.size()));
        }
        payload.insert(payload.end(), type, type + 4);
        const auto size = extended ? 0U : static_cast<std::uint32_t>(data.size());
        payload.push_back(static_cast<std::uint8_t>(size));
        payload.push_back(static_cast<std::uint8_t>(size >> 8));
        payload.insert(payload.end(), data.begin(), data.end());
    }

    /// Consume a count-prefixed fixed-width array without trusting a source count or allocating from it.
    bool Array(std::span<const std::uint8_t> data, std::size_t &offset, std::size_t width,
               std::span<const std::uint8_t> &elements)
    {
        if (offset > data.size() || data.size() - offset < 4)
        {
            return false;
        }
        const auto count = Read32(data, offset);
        offset += 4;
        if (count > (data.size() - offset) / width)
        {
            return false;
        }
        elements = data.subspan(offset, count * width);
        offset += elements.size();
        return true;
    }

    void FormArray(Bytes &data, const std::set<std::uint32_t> &forms)
    {
        Put32(data, static_cast<std::uint32_t>(forms.size()));
        for (const auto form : forms)
        {
            Put32(data, form);
        }
    }

    /// NVMI's variable arrays precede an optional island and a fixed PathingCell tail.
    bool ValidEntry(std::span<const std::uint8_t> data)
    {
        if (data.size() < 49)
        {
            return false;
        }
        std::size_t offset = 24;
        std::span<const std::uint8_t> elements;
        if (!Array(data, offset, 4, elements) || !Array(data, offset, 4, elements) ||
            !Array(data, offset, 8, elements) || offset >= data.size())
        {
            return false;
        }
        const auto island = data[offset++];
        if (island > 1)
        {
            return false;
        }
        if (island)
        {
            if (data.size() - offset < 24)
            {
                return false;
            }
            offset += 24;
            if (!Array(data, offset, 6, elements) || !Array(data, offset, 12, elements))
            {
                return false;
            }
        }
        return data.size() - offset == 12 && Read32(data, offset) == PathingCellCrc;
    }

    /// Summarize only links consumed by final triangle edges. NVMI contains unique mesh identities,
    /// whereas NVNM contains one external entry per portal and uses triangle-local edge indices.
    bool BuildEntry(std::uint32_t id, std::span<const std::uint8_t> data, Bytes &entry)
    {
        if (data.size() < 20 || Read32(data, 0) != 12)
        {
            return false;
        }
        std::size_t offset = 16;
        std::span<const std::uint8_t> vertices, triangles, external, doors;
        if (!Array(data, offset, 12, vertices) || !Array(data, offset, 16, triangles) ||
            !Array(data, offset, 10, external) || !Array(data, offset, 10, doors))
        {
            return false;
        }
        std::vector<navmesh::core::Vec3> points(vertices.size() / 12);
        if (!vertices.empty())
        {
            std::memcpy(points.data(), vertices.data(), vertices.size());
        }
        navmesh::core::Vec3 minimum{}, maximum{};
        if (!points.empty())
        {
            minimum = maximum = points.front();
        }
        for (const auto &point : points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                return false;
            }
            minimum.x = std::min(minimum.x, point.x);
            minimum.y = std::min(minimum.y, point.y);
            minimum.z = std::min(minimum.z, point.z);
            maximum.x = std::max(maximum.x, point.x);
            maximum.y = std::max(maximum.y, point.y);
            maximum.z = std::max(maximum.z, point.z);
        }

        std::set<std::uint32_t> links, preferredLinks;
        std::set<std::pair<std::uint32_t, std::uint32_t>> doorLinks;
        double area{}, preferredArea{}, centerX{}, centerY{}, centerZ{};
        for (std::size_t triangle{}; triangle < triangles.size(); triangle += 16)
        {
            const auto a = Read16(triangles, triangle);
            const auto b = Read16(triangles, triangle + 2);
            const auto c = Read16(triangles, triangle + 4);
            if (a >= points.size() || b >= points.size() || c >= points.size())
            {
                return false;
            }
            const auto flags = Read16(triangles, triangle + 12);
            const auto ab = points[b] - points[a];
            const auto ac = points[c] - points[a];
            const double x = static_cast<double>(ab.y) * ac.z - static_cast<double>(ab.z) * ac.y;
            const double y = static_cast<double>(ab.z) * ac.x - static_cast<double>(ab.x) * ac.z;
            const double z = static_cast<double>(ab.x) * ac.y - static_cast<double>(ab.y) * ac.x;
            const double weight = std::sqrt(x * x + y * y + z * z);
            area += weight;
            centerX += weight * (static_cast<double>(points[a].x) + points[b].x + points[c].x) / 3;
            centerY += weight * (static_cast<double>(points[a].y) + points[b].y + points[c].y) / 3;
            centerZ += weight * (static_cast<double>(points[a].z) + points[b].z + points[c].z) / 3;
            if (flags & navmesh::core::PreferredPathFlag)
            {
                preferredArea += weight;
            }
            for (std::size_t edge{}; edge < 3; ++edge)
            {
                if (!(flags & (1U << edge)))
                {
                    continue;
                }
                const auto index = Read16(triangles, triangle + 6 + edge * 2);
                if (index >= external.size() / 10 || !Read32(external, index * 10 + 4))
                {
                    return false;
                }
                const auto target = Read32(external, index * 10 + 4);
                links.insert(target);
                if (flags & navmesh::core::PreferredPathFlag)
                {
                    preferredLinks.insert(target);
                }
            }
        }
        for (std::size_t door{}; door < doors.size(); door += 10)
        {
            if (Read16(doors, door) >= triangles.size() / 16 || !Read32(doors, door + 6))
            {
                return false;
            }
            doorLinks.emplace(Read32(doors, door + 6), Read32(doors, door + 2));
        }

        // Isolated meshes carry geometry so the unloaded-cell map can locate their floor.
        // Empty secondary overrides remain registered with no island or stale connections.
        const bool island = !triangles.empty() && links.empty() && doorLinks.empty();
        Put32(entry, id);
        Put32(entry, island ? 0x20U : 0U);
        PutFloat(entry, area > 0 ? static_cast<float>(centerX / area) : minimum.x);
        PutFloat(entry, area > 0 ? static_cast<float>(centerY / area) : minimum.y);
        PutFloat(entry, area > 0 ? static_cast<float>(centerZ / area) : minimum.z);
        PutFloat(entry, area > 0 ? static_cast<float>(preferredArea / area) : 0);
        FormArray(entry, links);
        FormArray(entry, preferredLinks);
        Put32(entry, static_cast<std::uint32_t>(doorLinks.size()));
        for (const auto &[reference, crc] : doorLinks)
        {
            Put32(entry, crc);
            Put32(entry, reference);
        }
        entry.push_back(island ? 1 : 0);
        if (island)
        {
            for (const auto value : {minimum.x, minimum.y, minimum.z, maximum.x, maximum.y, maximum.z})
            {
                PutFloat(entry, value);
            }
            Put32(entry, static_cast<std::uint32_t>(triangles.size() / 16));
            for (std::size_t triangle{}; triangle < triangles.size(); triangle += 16)
            {
                entry.insert(entry.end(), triangles.begin() + triangle, triangles.begin() + triangle + 6);
            }
            Put32(entry, static_cast<std::uint32_t>(points.size()));
            entry.insert(entry.end(), vertices.begin(), vertices.end());
        }
        Put32(entry, PathingCellCrc);
        entry.insert(entry.end(), data.begin() + 8, data.begin() + 16);
        return true;
    }
} // namespace

bool navmesh::skyrim::detail::BuildNavmeshInfo(const std::map<std::uint32_t, std::vector<std::uint8_t>> &geometry,
                                               const PluginRecord *preserved, std::vector<std::uint8_t> &payload,
                                               std::string &error)
{
    std::map<std::uint32_t, Bytes> entries;
    Bytes deleted;
    payload.clear();
    Bytes version;
    Put32(version, 12);
    if (preserved)
    {
        bool foundVersion{};
        for (const auto &sub : preserved->subrecords)
        {
            if (sub.type == "NVMI" && ValidEntry(sub.data))
            {
                const auto id = Read32(sub.data, 0);
                if (!entries.emplace(id, Bytes(sub.data.begin(), sub.data.end())).second)
                {
                    error = "Copied NAVI has duplicate navigation-map entries.";
                    return false;
                }
            }
            else if (sub.type == "NVSI" && sub.data.size() % 4 == 0)
            {
                for (std::size_t index{}; index < sub.data.size(); index += 4)
                {
                    const auto id = Read32(sub.data, index);
                    if (!geometry.contains(id))
                    {
                        Put32(deleted, id);
                    }
                }
            }
            else if (sub.type == "NVER" && !foundVersion && sub.data.size() == 4 && Read32(sub.data, 0) == 12)
            {
                foundVersion = true;
                continue;
            }
            else if (sub.type == "EDID" || sub.type == "NVPP")
            {
                continue;
            }
            else
            {
                error = "Copied NAVI has unsupported or malformed subrecords.";
                return false;
            }
        }
        if (!foundVersion)
        {
            error = "Copied NAVI has no supported version.";
            return false;
        }
    }
    for (const auto &[id, data] : geometry)
    {
        Bytes entry;
        if (!BuildEntry(id, data, entry))
        {
            error = "Cannot build NAVI from malformed serialized NAVM geometry or connections.";
            return false;
        }
        entries[id] = std::move(entry);
    }
    PutSubrecord(payload, "NVER", version);
    for (const auto &[id, entry] : entries)
    {
        PutSubrecord(payload, "NVMI", entry);
    }
    // NVPP is an optional acceleration cache. Empty path and marker arrays allow
    // navigation to use the rebuilt graph without retaining routes through replaced triangles.
    PutSubrecord(payload, "NVPP", Bytes(8));
    if (!deleted.empty())
    {
        PutSubrecord(payload, "NVSI", deleted);
    }
    return true;
}
