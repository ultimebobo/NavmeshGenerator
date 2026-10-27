#include "skyrim/parser/navigation_audit.h"
#include "core/navmesh/candidate.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

namespace
{
    using namespace navmesh;

    bool Live(const skyrim::ResolvedRecord *record)
    {
        return !record || (!record->winning.deleted && (!record->raw || !(record->raw->flags & 0x20U)));
    }

    bool Active(const core::NavPolygon &polygon)
    {
        return !(polygon.flags & (1U << 3));
    }

    bool Finite(core::Vec3 point)
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

    bool ValidVertices(const core::NavMesh &mesh, const core::NavPolygon &polygon)
    {
        return std::all_of(polygon.vertices.begin(), polygon.vertices.end(),
                           [&](auto index) { return index < mesh.vertices.size() && Finite(mesh.vertices[index]); });
    }

    bool SameEndpoint(core::Vec3 a, core::Vec3 b, float stepHeight)
    {
        return std::hypot(a.x - b.x, a.y - b.y) <= core::AuthoredBorderTolerance && std::abs(a.z - b.z) <= stepHeight;
    }

    /// Check only claimed adjacency. Open boundaries and separate authored islands can be intentional.
    bool ReciprocalNeighbor(const core::NavMesh &mesh, std::uint32_t index, std::size_t edge, float stepHeight)
    {
        const auto &face = mesh.polygons[index];
        const auto neighborIndex = face.neighbors[edge];
        if (neighborIndex >= mesh.polygons.size() || neighborIndex == index)
        {
            return false;
        }
        const auto &neighbor = mesh.polygons[neighborIndex];
        if (!Active(neighbor) || !ValidVertices(mesh, neighbor))
        {
            return false;
        }
        for (std::size_t side{}; side < 3; ++side)
        {
            if (!(neighbor.flags & (1U << side)) && neighbor.neighbors[side] == index &&
                SameEndpoint(mesh.vertices[face.vertices[edge]], mesh.vertices[neighbor.vertices[(side + 1) % 3]],
                             stepHeight) &&
                SameEndpoint(mesh.vertices[face.vertices[(edge + 1) % 3]], mesh.vertices[neighbor.vertices[side]],
                             stepHeight))
            {
                return true;
            }
        }
        return false;
    }

    using MeshIndex = std::map<std::uint32_t, const core::NavMesh *>;

    /// Resolution omits undecodable tables and null door destinations from scene evidence.
    /// Recheck their retained bytes so omission cannot hide a structural fault from repair selection.
    const char *ConnectionFault(const skyrim::ResolvedRecord *record, const core::NavMesh &mesh)
    {
        if (!record || !record->raw || !record->navm || !record->navm->supported)
        {
            return nullptr;
        }
        const auto &data = record->raw->decodedPayload;
        const auto range = record->navm->trailingData;
        if (range.size < 8 || range.offset > data.size() || range.size > data.size() - range.offset)
        {
            return "malformed_connections";
        }
        core::NavMesh decoded{.polygons = mesh.polygons};
        if (!skyrim::DecodeNavmeshConnections({data.data() + range.offset, static_cast<std::size_t>(range.size)},
                                              decoded))
        {
            return "malformed_connections";
        }
        return decoded.doorLinks.size() != mesh.doorLinks.size() ? "invalid_door_link" : nullptr;
    }

    /// Resolved destinations and unique reverse links are required; authored endpoint offsets remain permissible.
    bool ValidPortal(const core::NavmeshExternalLink &link, const core::NavMesh &mesh, const MeshIndex &meshes,
                     const skyrim::ResolvedLoadOrder &resolved)
    {
        const auto destination = meshes.find(link.navmeshId);
        if (link.type > 3)
        {
            return false;
        }
        if (destination == meshes.end())
        {
            const auto *record = resolved.FindWinning(link.navmeshId);
            // An unsupported live destination supplies no evidence either way; never infer a missing triangle.
            return record && record->type == "NAVM" && Live(record) && record->navm && !record->navm->supported;
        }
        const auto &target = *destination->second;
        if (link.targetPolygon >= target.polygons.size() || !Active(target.polygons[link.targetPolygon]))
        {
            return false;
        }
        const auto reverseType = link.type == 1 ? 2U : link.type == 2 ? 1U : link.type;
        return std::count_if(target.externalLinks.begin(), target.externalLinks.end(),
                             [&](const auto &reverse)
                             {
                                 return reverse.polygon == link.targetPolygon && reverse.navmeshId == mesh.id &&
                                        reverse.targetPolygon == link.polygon && reverse.type == reverseType &&
                                        reverse.edge < 3;
                             }) == 1;
    }
} // namespace

namespace navmesh::skyrim
{
    NavigationAudit AuditNavigation(const ResolvedLoadOrder &resolved, float stepHeight,
                                    const std::function<bool()> &cancelled)
    {
        if (!std::isfinite(stepHeight) || stepHeight < 0)
        {
            throw std::invalid_argument("Navigation audit step height must be finite and nonnegative.");
        }
        NavigationAudit report;
        MeshIndex meshes;
        for (const auto &cell : resolved.cells)
        {
            if (!Live(resolved.FindWinning(cell.id)))
            {
                continue;
            }
            for (const auto &mesh : cell.navMeshes)
            {
                if (Live(resolved.FindWinning(mesh.id)))
                {
                    meshes.emplace(mesh.id, &mesh);
                }
            }
        }
        for (const auto &record : resolved.records)
        {
            if (record.type == "NAVM" && Live(&record) && (!record.navm || !record.navm->supported))
            {
                ++report.unsupportedNavmeshes;
            }
        }
        for (const auto &cell : resolved.cells)
        {
            for (const auto &mesh : cell.navMeshes)
            {
                if (cancelled && cancelled())
                {
                    return report;
                }
                if (!meshes.contains(mesh.id))
                {
                    continue;
                }
                ++report.scannedNavmeshes;
                std::set<std::string> kinds;
                const auto finding = [&](const char *kind, std::optional<std::uint32_t> polygon = std::nullopt)
                {
                    if (kinds.insert(kind).second)
                    {
                        report.findings.push_back({cell.id, mesh.id, polygon, kind});
                    }
                };
                if (const auto *fault = ConnectionFault(resolved.FindWinning(mesh.id), mesh))
                {
                    finding(fault);
                }
                // Index active, valid geometry once. Disabled triangles keep their source indices but
                // do not participate in duplicate/manifold tests or supply usable adjacency.
                std::set<std::array<std::uint32_t, 3>> faces;
                std::map<std::pair<std::uint32_t, std::uint32_t>, std::size_t> edgeUses;
                std::map<std::pair<std::uint32_t, std::uint8_t>, const core::NavmeshExternalLink *> portals;
                for (const auto &link : mesh.externalLinks)
                {
                    if (!portals.emplace(std::pair{link.polygon, link.edge}, &link).second)
                    {
                        finding("invalid_external_link", link.polygon);
                    }
                }
                for (std::uint32_t index{}; index < mesh.polygons.size(); ++index)
                {
                    if (index % 1024 == 0 && cancelled && cancelled())
                    {
                        return report;
                    }
                    const auto &face = mesh.polygons[index];
                    if (!Active(face))
                    {
                        continue;
                    }
                    if (!ValidVertices(mesh, face))
                    {
                        const auto outOfRange = std::any_of(face.vertices.begin(), face.vertices.end(), [&](auto vertex)
                                                            { return vertex >= mesh.vertices.size(); });
                        finding(outOfRange ? "invalid_triangle_vertices" : "non_finite_vertex", index);
                        continue;
                    }
                    const auto a = mesh.vertices[face.vertices[0]];
                    const auto b = mesh.vertices[face.vertices[1]];
                    const auto c = mesh.vertices[face.vertices[2]];
                    const double ux = static_cast<double>(b.x) - a.x, uy = static_cast<double>(b.y) - a.y,
                                 uz = static_cast<double>(b.z) - a.z;
                    const double vx = static_cast<double>(c.x) - a.x, vy = static_cast<double>(c.y) - a.y,
                                 vz = static_cast<double>(c.z) - a.z;
                    if (ux * vy - uy * vx == 0 && uy * vz - uz * vy == 0 && uz * vx - ux * vz == 0)
                    {
                        finding("degenerate_triangle", index);
                    }
                    if (ux * vy - uy * vx < 0)
                    {
                        finding("inverted_triangle", index);
                    }
                    auto vertices = face.vertices;
                    std::sort(vertices.begin(), vertices.end());
                    if (!faces.insert(vertices).second)
                    {
                        finding("duplicate_triangle", index);
                    }
                    for (std::uint8_t edge{}; edge < 3; ++edge)
                    {
                        const auto endpoints = std::minmax(face.vertices[edge], face.vertices[(edge + 1) % 3]);
                        if (++edgeUses[{endpoints.first, endpoints.second}] > 2)
                        {
                            finding("non_manifold_edge", index);
                        }
                        if (face.flags & (1U << edge))
                        {
                            const auto link = portals.find({index, edge});
                            if (link == portals.end() || !ValidPortal(*link->second, mesh, meshes, resolved))
                            {
                                finding("invalid_external_link", index);
                            }
                        }
                        else if (face.neighbors[edge] != 0xffffU && !ReciprocalNeighbor(mesh, index, edge, stepHeight))
                        {
                            finding("invalid_adjacency", index);
                        }
                    }
                }
                for (const auto &door : mesh.doorLinks)
                {
                    const auto *reference = resolved.FindWinning(door.referenceId);
                    if (door.polygon >= mesh.polygons.size() || !Active(mesh.polygons[door.polygon]) || !reference ||
                        reference->type != "REFR" || !Live(reference))
                    {
                        finding("invalid_door_link", door.polygon);
                    }
                }
            }
        }
        return report;
    }
} // namespace navmesh::skyrim
