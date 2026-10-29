#include "skyrim/parser/navigation_validation.h"
#include "core/navmesh/candidate.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <format>
#include <map>

namespace
{
    using namespace navmesh::skyrim;

    std::string Lower(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
        return value;
    }

    std::uint16_t Read16(std::span<const std::uint8_t> data, std::size_t at)
    {
        return static_cast<std::uint16_t>(data[at] | data[at + 1] << 8);
    }

    std::uint32_t Read32(std::span<const std::uint8_t> data, std::size_t at)
    {
        std::uint32_t value{};
        std::memcpy(&value, data.data() + at, 4);
        return value;
    }

    bool SamePoint(navmesh::core::Vec3 a, navmesh::core::Vec3 b)
    {
        return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z) <= navmesh::core::AuthoredBorderTolerance;
    }

    /// Establish exact triangle identity and geometry for preservation in a connection-only override.
    bool UnchangedAuthoredTriangle(const navmesh::core::NavMesh &mesh, std::size_t index,
                                   const navmesh::core::NavMesh *authored)
    {
        if (!authored || index >= authored->polygons.size())
        {
            return false;
        }
        const auto &before = authored->polygons[index];
        const auto &after = mesh.polygons[index];
        if ((before.flags & (1U << 3)) || before.vertices != after.vertices)
        {
            return false;
        }
        for (const auto vertex : before.vertices)
        {
            if (vertex >= authored->vertices.size())
            {
                return false;
            }
            const auto a = authored->vertices[vertex];
            const auto b = mesh.vertices[vertex];
            if (a.x != b.x || a.y != b.y || a.z != b.z)
            {
                return false;
            }
        }
        return true;
    }

    /// Preserve a source's one-sided indexed shared edge without inventing a reverse path.
    /// Both faces, the claimed local edge and the opposite open edge must retain their original identity.
    bool UnchangedAuthoredLocalAdjacency(const navmesh::core::NavMesh &mesh, std::size_t index, std::size_t edge,
                                         const navmesh::core::NavMesh *authored)
    {
        if (!UnchangedAuthoredTriangle(mesh, index, authored))
        {
            return false;
        }
        const auto &face = mesh.polygons[index];
        const auto &before = authored->polygons[index];
        const auto neighborIndex = face.neighbors[edge];
        if ((before.flags & (1U << edge)) || before.neighbors[edge] != neighborIndex ||
            (before.flags & ~7U) != (face.flags & ~7U) || !UnchangedAuthoredTriangle(mesh, neighborIndex, authored))
        {
            return false;
        }
        const auto &neighbor = mesh.polygons[neighborIndex];
        const auto &originalNeighbor = authored->polygons[neighborIndex];
        if ((originalNeighbor.flags & ~7U) != (neighbor.flags & ~7U))
        {
            return false;
        }
        for (std::size_t side{}; side < 3; ++side)
        {
            if (face.vertices[edge] == neighbor.vertices[(side + 1) % 3] &&
                face.vertices[(edge + 1) % 3] == neighbor.vertices[side] && !(neighbor.flags & (1U << side)) &&
                !(originalNeighbor.flags & (1U << side)) && neighbor.neighbors[side] == 0xffffU &&
                originalNeighbor.neighbors[side] == 0xffffU)
            {
                return true;
            }
        }
        return false;
    }

    bool Decode(const ResolvedRecord &record, navmesh::core::NavMesh &mesh, float stepHeight,
                const navmesh::core::NavMesh *authored, std::string &stage)
    {
        stage = "record layout";
        if (!record.raw || !record.navm || !record.navm->supported || (record.raw->flags & 0x20U))
        {
            return false;
        }
        const auto &layout = *record.navm;
        const auto &data = record.raw->decodedPayload;
        mesh.id = record.winning.formId;
        const auto sameLocalEndpoint = [&](navmesh::core::Vec3 a, navmesh::core::Vec3 b)
        {
            return std::hypot(a.x - b.x, a.y - b.y) <= navmesh::core::AuthoredBorderTolerance &&
                   std::abs(a.z - b.z) <= stepHeight;
        };
        mesh.vertices.resize(layout.vertexCount);
        if (!mesh.vertices.empty())
        {
            std::memcpy(mesh.vertices.data(), data.data() + layout.vertices.offset, mesh.vertices.size() * 12);
        }
        for (std::uint32_t index{}; index < layout.triangleCount; ++index)
        {
            const auto at = static_cast<std::size_t>(layout.triangles.offset) + index * 16;
            navmesh::core::NavPolygon face;
            for (std::size_t edge{}; edge < 3; ++edge)
            {
                face.vertices[edge] = Read16(data, at + edge * 2);
                face.neighbors[edge] = Read16(data, at + 6 + edge * 2);
                if (face.vertices[edge] >= mesh.vertices.size())
                {
                    return false;
                }
            }
            face.flags = Read16(data, at + 12);
            face.coverFlags = Read16(data, at + 14);
            mesh.polygons.push_back(face);
        }
        stage = "finite coordinates";
        if (std::any_of(mesh.vertices.begin(), mesh.vertices.end(), [](const auto &point)
                        { return !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z); }))
        {
            return false;
        }
        for (std::size_t index{}; index < mesh.polygons.size(); ++index)
        {
            const auto &face = mesh.polygons[index];
            if (face.flags & (1U << 3))
            {
                continue;
            }
            const auto a = mesh.vertices[face.vertices[0]];
            const auto b = mesh.vertices[face.vertices[1]];
            const auto c = mesh.vertices[face.vertices[2]];
            const auto winding = (static_cast<double>(b.x) - a.x) * (static_cast<double>(c.y) - a.y) -
                                 (static_cast<double>(b.y) - a.y) * (static_cast<double>(c.x) - a.x);
            stage = std::format("live triangle {} winding", index);
            if (!std::isfinite(winding) || winding < 0 ||
                (winding == 0 && !UnchangedAuthoredTriangle(mesh, index, authored)))
            {
                return false;
            }
            for (std::size_t edge{}; edge < 3; ++edge)
            {
                stage = std::format("triangle {} edge {} adjacency", index, edge);
                if ((face.flags & (1U << edge)) || face.neighbors[edge] == 0xffffU)
                {
                    continue;
                }
                if (face.neighbors[edge] >= mesh.polygons.size() || face.neighbors[edge] == index)
                {
                    return false;
                }
                const auto &neighbor = mesh.polygons[face.neighbors[edge]];
                bool reciprocal{};
                for (std::size_t side{}; side < 3; ++side)
                {
                    if (!(neighbor.flags & (1U << side)) && neighbor.neighbors[side] == index &&
                        sameLocalEndpoint(mesh.vertices[face.vertices[edge]],
                                          mesh.vertices[neighbor.vertices[(side + 1) % 3]]) &&
                        sameLocalEndpoint(mesh.vertices[face.vertices[(edge + 1) % 3]],
                                          mesh.vertices[neighbor.vertices[side]]))
                    {
                        reciprocal = true;
                    }
                }
                if (!reciprocal && !UnchangedAuthoredLocalAdjacency(mesh, index, edge, authored))
                {
                    return false;
                }
            }
        }
        stage = "auxiliary vertex arrays";
        for (const auto &sub : record.raw->subrecords)
        {
            if (sub.type == "ONAM" && sub.data.size() % 4 != 0)
            {
                return false;
            }
            if (sub.type != "PNAM" && sub.type != "NNAM")
            {
                continue;
            }
            if (sub.data.size() % 2 != 0)
            {
                return false;
            }
            for (std::size_t offset{}; offset < sub.data.size(); offset += 2)
            {
                if (Read16(sub.data, offset) >= mesh.vertices.size())
                {
                    return false;
                }
            }
        }
        const std::span<const std::uint8_t> trailing{data.data() + layout.trailingData.offset,
                                                     static_cast<std::size_t>(layout.trailingData.size)};
        stage = "connection arrays";
        if (!DecodeNavmeshConnections(trailing, mesh))
        {
            return false;
        }
        // Walk every trailing array and grid bucket independently of its writer.
        // All live triangles must appear in the grid; cover indices must refer to marked triangles.
        stage = "cover indices";
        const auto externalCount = Read32(trailing, 0);
        const auto doorAt = 4 + externalCount * 10;
        const auto coverAt = doorAt + 4 + Read32(trailing, doorAt) * 10;
        if (coverAt > trailing.size() || trailing.size() - coverAt < 4)
        {
            return false;
        }
        const auto covers = Read32(trailing, coverAt);
        if (covers > (trailing.size() - coverAt - 4) / 2)
        {
            return false;
        }
        std::set<std::uint16_t> coverTriangles;
        for (std::uint32_t index{}; index < covers; ++index)
        {
            const auto triangle = Read16(trailing, coverAt + 4 + index * 2);
            if (triangle >= mesh.polygons.size() || !coverTriangles.insert(triangle).second ||
                !(mesh.polygons[triangle].coverFlags & 0xfffU))
            {
                return false;
            }
        }
        for (std::size_t index{}; index < mesh.polygons.size(); ++index)
        {
            if ((mesh.polygons[index].coverFlags & 0xfffU) &&
                !coverTriangles.contains(static_cast<std::uint16_t>(index)))
            {
                return false;
            }
        }
        stage = "spatial grid";
        auto at = coverAt + 4 + covers * 2;
        if (trailing.size() - at < 36)
        {
            return false;
        }
        const auto divisor = Read32(trailing, at);
        for (std::size_t field = at + 4; field < at + 36; field += 4)
        {
            float value{};
            std::memcpy(&value, trailing.data() + field, sizeof(value));
            if (!std::isfinite(value))
            {
                return false;
            }
        }
        at += 36;
        if (!divisor || divisor > (trailing.size() - at) / 4 / divisor)
        {
            return false;
        }
        std::set<std::uint16_t> indexed;
        for (std::uint64_t bucket{}; bucket < static_cast<std::uint64_t>(divisor) * divisor; ++bucket)
        {
            if (trailing.size() - at < 4)
            {
                return false;
            }
            const auto count = Read32(trailing, at);
            at += 4;
            if (count > (trailing.size() - at) / 2)
            {
                return false;
            }
            for (std::uint32_t entry{}; entry < count; ++entry)
            {
                const auto triangle = Read16(trailing, at + entry * 2);
                if (triangle >= mesh.polygons.size())
                {
                    return false;
                }
                indexed.insert(triangle);
            }
            at += count * 2;
        }
        if (at != trailing.size())
        {
            return false;
        }
        for (std::size_t index{}; index < mesh.polygons.size(); ++index)
        {
            if (!(mesh.polygons[index].flags & (1U << 3)) && !indexed.contains(static_cast<std::uint16_t>(index)))
            {
                return false;
            }
        }
        return true;
    }

} // namespace

bool navmesh::skyrim::detail::ValidateNavigationGraph(
    const std::vector<ResolvedRecord> &records, const std::vector<std::string> &masters, const std::string &outputName,
    const ResolvedLoadOrder &input, const std::set<std::uint32_t> &modifiedMeshes,
    const std::set<std::uint32_t> &modifiedReferences, std::string &error,
    const std::map<std::uint32_t, float> &generatedStepHeights, bool resolvedIdentities)
{
    auto names = masters;
    names.push_back(outputName);
    const auto outputId = [&](std::uint32_t global) -> std::optional<std::uint32_t>
    {
        if (resolvedIdentities)
        {
            return global;
        }
        const auto *record = input.FindWinning(global);
        if (!record || record->origins.empty())
        {
            return std::nullopt;
        }
        const auto owner = Lower(record->origins.front().plugin);
        const auto found =
            std::find_if(names.begin(), names.end(), [&](const auto &name) { return Lower(name) == owner; });
        if (found == names.end())
        {
            return std::nullopt;
        }
        const auto object = global >> 24 == 0xfeU ? global & 0xfffU : global & 0xffffffU;
        return static_cast<std::uint32_t>(found - names.begin()) << 24 | object;
    };
    std::map<std::uint32_t, core::NavMesh> meshes;
    std::map<std::uint32_t, const core::NavMesh *> sourceMeshes;
    std::map<std::uint32_t, std::optional<NavmeshDoorAssociation>> doors;
    for (const auto &cell : input.cells)
    {
        for (const auto &source : cell.navMeshes)
        {
            const auto id = outputId(source.id);
            if (!id)
            {
                continue;
            }
            sourceMeshes.emplace(*id, &source);
        }
    }
    const auto getMesh = [&](std::uint32_t id) -> const core::NavMesh *
    {
        if (const auto found = meshes.find(id); found != meshes.end())
        {
            return &found->second;
        }
        const auto source = sourceMeshes.find(id);
        if (source == sourceMeshes.end())
        {
            return nullptr;
        }
        auto mesh = *source->second;
        mesh.id = id;
        for (auto &link : mesh.externalLinks)
        {
            link.navmeshId = outputId(link.navmeshId).value_or(0);
        }
        for (auto &link : mesh.doorLinks)
        {
            link.referenceId = outputId(link.referenceId).value_or(0);
        }
        return &meshes.emplace(id, std::move(mesh)).first->second;
    };
    // Untouched authored portals can have intentional endpoint offsets. Preserve their
    // complete relationship and geometry; newly created or moved portal edges must match.
    const auto unchangedPortal = [&](const core::NavMesh &mesh, const core::NavmeshExternalLink &portal)
    {
        const auto source = sourceMeshes.find(mesh.id);
        if (source == sourceMeshes.end() || portal.polygon >= source->second->polygons.size())
        {
            return false;
        }
        const auto &original = *source->second;
        const auto matched = std::any_of(original.externalLinks.begin(), original.externalLinks.end(),
                                         [&](const auto &link)
                                         {
                                             return link.polygon == portal.polygon && link.edge == portal.edge &&
                                                    link.type == portal.type &&
                                                    link.targetPolygon == portal.targetPolygon &&
                                                    outputId(link.navmeshId) == portal.navmeshId;
                                         });
        const auto &before = original.polygons[portal.polygon];
        const auto &after = mesh.polygons[portal.polygon];
        for (const auto edge : {portal.edge, static_cast<std::uint8_t>((portal.edge + 1) % 3)})
        {
            const auto a = original.vertices.at(before.vertices[edge]);
            const auto b = mesh.vertices.at(after.vertices[edge]);
            if (a.x != b.x || a.y != b.y || a.z != b.z)
            {
                return false;
            }
        }
        return matched;
    };
    for (const auto &record : input.records)
    {
        if (record.type != "REFR" || !record.navmeshDoor)
        {
            continue;
        }
        const auto id = outputId(record.formId);
        if (!id)
        {
            continue;
        }
        auto association = record.navmeshDoor;
        if (association)
        {
            association->navmeshId = outputId(association->navmeshId).value_or(0);
        }
        doors.emplace(*id, association);
    }
    for (const auto &record : records)
    {
        if (record.type == "NAVM" && modifiedMeshes.contains(record.winning.formId))
        {
            core::NavMesh mesh;
            std::string stage;
            const auto settings = generatedStepHeights.find(record.winning.formId);
            const auto stepHeight =
                settings == generatedStepHeights.end() ? core::AuthoredBorderTolerance : settings->second;
            const auto source = sourceMeshes.find(record.winning.formId);
            const auto *authored =
                settings == generatedStepHeights.end() && source != sourceMeshes.end() ? source->second : nullptr;
            if (!std::isfinite(stepHeight) || stepHeight < 0 || !Decode(record, mesh, stepHeight, authored, stage))
            {
                error = std::format("Exported NAVM {:08X} has invalid {}.", record.winning.formId, stage);
                return false;
            }
            meshes[mesh.id] = std::move(mesh);
        }
        else if (record.type == "REFR")
        {
            doors[record.winning.formId] = record.navmeshDoor;
        }
    }
    for (const auto id : modifiedMeshes)
    {
        const auto found = meshes.find(id);
        if (found == meshes.end())
        {
            error = "An exported NAVM is missing from the effective navigation graph.";
            return false;
        }
        const auto &mesh = found->second;
        for (const auto &portal : mesh.externalLinks)
        {
            const auto *target = getMesh(portal.navmeshId);
            if (!portal.navmeshId || !target || portal.targetPolygon >= target->polygons.size() || portal.type > 3)
            {
                error = "An exported portal targets a missing NAVM or triangle.";
                return false;
            }
            const auto reverseType = portal.type == 1 ? 2U : portal.type == 2 ? 1U : portal.type;
            const auto matches =
                std::count_if(target->externalLinks.begin(), target->externalLinks.end(),
                              [&](const auto &reverse)
                              {
                                  return reverse.polygon == portal.targetPolygon && reverse.navmeshId == id &&
                                         reverse.targetPolygon == portal.polygon && reverse.type == reverseType;
                              });
            if (matches != 1)
            {
                error = "An exported portal has no unique reciprocal connection in the effective load order.";
                return false;
            }
            if (portal.type == 0)
            {
                const auto &reverse = *std::find_if(target->externalLinks.begin(), target->externalLinks.end(),
                                                    [&](const auto &link)
                                                    {
                                                        return link.polygon == portal.targetPolygon &&
                                                               link.navmeshId == id &&
                                                               link.targetPolygon == portal.polygon && link.type == 0;
                                                    });
                const auto &a = mesh.polygons[portal.polygon];
                const auto &b = target->polygons[reverse.polygon];
                const auto endpointsMatch = SamePoint(mesh.vertices.at(a.vertices[portal.edge]),
                                                      target->vertices.at(b.vertices[(reverse.edge + 1) % 3])) &&
                                            SamePoint(mesh.vertices.at(a.vertices[(portal.edge + 1) % 3]),
                                                      target->vertices.at(b.vertices[reverse.edge]));
                if (!endpointsMatch && !(unchangedPortal(mesh, portal) && unchangedPortal(*target, reverse)))
                {
                    error =
                        std::format("An exported portal's endpoints disagree with its reciprocal edge: "
                                    "NAVM {:08X}:{}:{} -> NAVM {:08X}:{}:{}.",
                                    mesh.id, portal.polygon, portal.edge, target->id, reverse.polygon, reverse.edge);
                    return false;
                }
            }
        }
        for (const auto &door : mesh.doorLinks)
        {
            const auto reference = doors.find(door.referenceId);
            if (reference == doors.end() || !reference->second || reference->second->navmeshId != id ||
                reference->second->triangle != door.polygon || !(mesh.polygons.at(door.polygon).flags & (1U << 10)))
            {
                error = "An exported door portal has no matching REFR XNDP association and door triangle flag.";
                return false;
            }
        }
    }
    for (const auto &record : records)
    {
        if (record.type != "REFR" || !record.navmeshDoor || !modifiedReferences.contains(record.winning.formId))
        {
            continue;
        }
        const auto *target = getMesh(record.navmeshDoor->navmeshId);
        if (!target || record.navmeshDoor->triangle >= target->polygons.size() ||
            std::count_if(target->doorLinks.begin(), target->doorLinks.end(),
                          [&](const auto &door)
                          {
                              return door.referenceId == record.winning.formId &&
                                     door.polygon == record.navmeshDoor->triangle;
                          }) != 1)
        {
            error = "An exported REFR XNDP association has no matching NAVM door portal.";
            return false;
        }
    }
    return true;
}
