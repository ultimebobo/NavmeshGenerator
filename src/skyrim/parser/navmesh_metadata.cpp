#include "skyrim/parser/navmesh_metadata.h"

#include <cmath>
#include <cstring>
#include <set>

namespace
{
    using Bytes = std::vector<std::uint8_t>;

    void Put(Bytes &bytes, std::uint32_t value, unsigned width)
    {
        for (unsigned index{}; index < width; ++index)
        {
            bytes.push_back(static_cast<std::uint8_t>(value >> (index * 8)));
        }
    }

    void Array(Bytes &bytes, const char *type, const std::set<std::uint32_t> &forms, unsigned width)
    {
        if (forms.empty())
        {
            return;
        }
        const auto size = static_cast<std::uint32_t>(forms.size() * width);
        if (size > 65535)
        {
            bytes.insert(bytes.end(), {'X', 'X', 'X', 'X', 4, 0});
            Put(bytes, size, 4);
        }
        bytes.insert(bytes.end(), type, type + 4);
        Put(bytes, size > 65535 ? 0 : size, 2);
        for (const auto form : forms)
        {
            Put(bytes, form, width);
        }
    }
} // namespace

bool navmesh::skyrim::detail::RebuildNavmeshMetadata(std::span<const NavmeshMetadataSource> sources,
                                                     const core::NavMesh &generated, float tolerance,
                                                     std::vector<std::uint8_t> &payload, std::string &error,
                                                     std::vector<std::string> *warnings)
{
    if (!std::isfinite(tolerance) || tolerance < 0 || generated.vertices.size() > 65535)
    {
        error = "NAVM connector remapping requires finite displacement limits and representable vertex indices.";
        return false;
    }
    std::set<std::uint32_t> objects, preferred, excluded;
    for (const auto &source : sources)
    {
        if (!source.record || !source.mesh || !source.rebase)
        {
            error = "NAVM auxiliary metadata has no source geometry or master mapping.";
            return false;
        }
        for (const auto &sub : source.record->subrecords)
        {
            if (sub.type != "ONAM" && sub.type != "PNAM" && sub.type != "NNAM")
            {
                continue;
            }
            const unsigned width = sub.type == "ONAM" ? 4 : 2;
            if (sub.data.size() % width != 0)
            {
                error = "NAVM has a malformed " + sub.type + " array.";
                return false;
            }
            for (std::size_t offset{}; offset < sub.data.size(); offset += width)
            {
                std::uint32_t value{};
                std::memcpy(&value, sub.data.data() + offset, width);
                if (sub.type == "ONAM")
                {
                    const auto form = value ? source.rebase(value) : std::optional<std::uint32_t>{0};
                    if (!form)
                    {
                        error = "Cannot rebase a NAVM base-object dependency.";
                        return false;
                    }
                    objects.insert(*form);
                    continue;
                }
                if (value >= source.mesh->vertices.size())
                {
                    error = "NAVM connector constraint refers to an invalid source vertex.";
                    return false;
                }
                const auto point = source.mesh->vertices[value];
                std::optional<std::uint32_t> matched;
                double nearest = tolerance;
                for (std::size_t index{}; index < generated.vertices.size(); ++index)
                {
                    const auto delta = point - generated.vertices[index];
                    const auto distance = std::hypot(std::hypot(delta.x, delta.y), delta.z);
                    if (distance <= nearest)
                    {
                        nearest = distance;
                        matched = static_cast<std::uint32_t>(index);
                    }
                }
                if (!matched)
                {
                    if (sub.type == "PNAM")
                    {
                        if (warnings)
                        {
                            warnings->push_back("Dropped authored PNAM preferred connector on NAVM " +
                                                std::to_string(source.mesh->id) + " at vertex " +
                                                std::to_string(value) + ": no generated vertex within reach.");
                        }
                        continue;
                    }
                    error = "Generated navigation cannot retain an authored " + sub.type + " connector constraint.";
                    return false;
                }
                (sub.type == "PNAM" ? preferred : excluded).insert(*matched);
            }
        }
    }
    for (const auto index : preferred)
    {
        if (excluded.contains(index))
        {
            error = "Authored preferred and excluded connector constraints map to the same generated vertex.";
            return false;
        }
    }
    payload.clear();
    Array(payload, "ONAM", objects, 4);
    Array(payload, "PNAM", preferred, 2);
    Array(payload, "NNAM", excluded, 2);
    return true;
}
