#include "core/navmesh/cover.h"
#include "analysis/navmesh_analysis.h"
#include "core/navmesh/triangle_tagging.h"

#include <algorithm>
#include <cmath>
#include <map>

namespace navmesh::core
{
    namespace
    {
        constexpr float CoverStep = 16.0F;
        constexpr unsigned MaximumCoverCode = 15;

        bool Boundary(const NavPolygon &face, std::size_t edge)
        {
            return !(face.flags & (1U << edge)) && (face.neighbors[edge] == 0xffffU ||
                                                    face.neighbors[edge] == std::numeric_limits<std::uint32_t>::max());
        }

        Vec3 Cross(Vec3 a, Vec3 b)
        {
            return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
        }

        double Dot(Vec3 a, Vec3 b)
        {
            return static_cast<double>(a.x) * b.x + static_cast<double>(a.y) * b.y + static_cast<double>(a.z) * b.z;
        }

        bool HitsTriangle(Vec3 origin, Vec3 direction, float distance, Vec3 a, Vec3 b, Vec3 c)
        {
            const auto first = b - a;
            const auto second = c - a;
            const auto perpendicular = Cross(direction, second);
            const auto determinant = Dot(first, perpendicular);
            if (std::abs(determinant) < 1e-8)
            {
                return false;
            }
            const auto relative = origin - a;
            const auto u = Dot(relative, perpendicular) / determinant;
            const auto across = Cross(relative, first);
            const auto v = Dot(direction, across) / determinant;
            const auto along = Dot(second, across) / determinant;
            return u >= -1e-7 && v >= -1e-7 && u + v <= 1 + 1e-7 && along >= 0 && along <= distance;
        }

        /// Require solid obstruction at several points along an edge and successive height bands.
        /// This excludes open rims, isolated posts, windows and overhead surfaces from full-edge cover.
        std::uint16_t PhysicalCover(Vec3 a, Vec3 b, Vec3 opposite, const Mesh &geometry,
                                    const analysis::SpatialIndex &walls, float distance, float epsilon)
        {
            const auto delta = b - a;
            const auto length = std::hypot(delta.x, delta.y);
            if (length <= epsilon)
            {
                return 0;
            }
            const auto sign = delta.x * (opposite.y - a.y) - delta.y * (opposite.x - a.x) >= 0 ? 1.0F : -1.0F;
            const Vec3 outward{sign * delta.y / length, -sign * delta.x / length, 0};
            AABB probeBounds;
            for (const auto point : {a, b, a + outward * distance, b + outward * distance})
            {
                probeBounds.Expand(point);
            }
            probeBounds.min.z += CoverStep;
            probeBounds.max.z += MaximumCoverCode * CoverStep;
            // Spatial rejection bounds the repeated height probes to walls beside this edge.
            const auto nearby = walls.QueryAABB(probeBounds);
            if (nearby.empty())
            {
                return 0;
            }
            unsigned height{};
            for (unsigned band = 1; band <= MaximumCoverCode; ++band)
            {
                bool blocked = true;
                for (const float fraction : {0.25F, 0.5F, 0.75F})
                {
                    auto origin = a + delta * fraction + outward * epsilon;
                    origin.z += band * CoverStep;
                    const bool hit = std::any_of(nearby.begin(), nearby.end(),
                                                 [&](const auto index)
                                                 {
                                                     const auto &face = geometry.triangles.at(index);
                                                     return HitsTriangle(origin, outward, distance,
                                                                         geometry.vertices.at(face.vertices[0]),
                                                                         geometry.vertices.at(face.vertices[1]),
                                                                         geometry.vertices.at(face.vertices[2]));
                                                 });
                    if (!hit)
                    {
                        blocked = false;
                        break;
                    }
                }
                if (!blocked)
                {
                    break;
                }
                height = band;
            }
            return static_cast<std::uint16_t>(height < 4 ? (height ? 1 : 0) : height);
        }

        std::uint16_t RetainedCover(Vec3 a, Vec3 b, std::span<const NavMesh> authored, float tolerance)
        {
            const auto close = [&](Vec3 x, Vec3 y)
            { return std::hypot(std::hypot(x.x - y.x, x.y - y.y), x.z - y.z) <= tolerance; };
            for (const auto &source : authored)
            {
                for (const auto &face : source.polygons)
                {
                    if (face.flags & (1U << 3))
                    {
                        continue;
                    }
                    for (std::size_t edge{}; edge < 2; ++edge)
                    {
                        const auto code = static_cast<std::uint16_t>((face.coverFlags >> (edge * 6)) & 0x3fU);
                        if (!(code & 0xfU) || face.vertices[edge] >= source.vertices.size() ||
                            face.vertices[(edge + 1) % 3] >= source.vertices.size())
                        {
                            continue;
                        }
                        const auto c = source.vertices[face.vertices[edge]];
                        const auto d = source.vertices[face.vertices[(edge + 1) % 3]];
                        if (close(a, c) && close(b, d))
                        {
                            return code;
                        }
                        if (close(a, d) && close(b, c))
                        {
                            return static_cast<std::uint16_t>((code & 0xfU) | ((code & 0x10U) << 1) |
                                                              ((code & 0x20U) >> 1));
                        }
                    }
                }
            }
            return 0;
        }

        /// Cyclic rotation preserves winding and triangle identity while moving its non-cover edge to slot two.
        void EncodeCover(NavPolygon &face, std::array<std::uint16_t, 3> cover)
        {
            if (cover[0] && cover[1] && cover[2])
            {
                const auto weakest = std::min_element(cover.begin(), cover.end(),
                                                      [](auto a, auto b) { return (a & 0xfU) < (b & 0xfU); });
                *weakest = 0;
            }
            std::size_t rotation{};
            while (rotation < 2 && cover[(rotation + 2) % 3])
            {
                ++rotation;
            }
            const auto original = face;
            face.flags &= static_cast<std::uint16_t>(~7U);
            for (std::size_t edge{}; edge < 3; ++edge)
            {
                const auto source = (edge + rotation) % 3;
                face.vertices[edge] = original.vertices[source];
                face.neighbors[edge] = original.neighbors[source];
                face.flags |= static_cast<std::uint16_t>(((original.flags >> source) & 1U) << edge);
            }
            face.coverFlags = static_cast<std::uint16_t>(cover[rotation] | (cover[(rotation + 1) % 3] << 6));
            if (face.coverFlags)
            {
                face.coverFlags |= 1U << 14;
            }
        }
    } // namespace

    void CaptureCoverGeometry(CandidateNavMesh &candidate, const Scene &scene)
    {
        candidate.coverGeometry = {};
        if (candidate.mesh.vertices.empty() || !scene.HasCompleteTriangleProvenance())
        {
            return;
        }
        AABB footprint;
        for (const auto &point : candidate.mesh.vertices)
        {
            footprint.Expand(point);
        }
        const auto margin = candidate.profile.agentRadius + 2 * candidate.recastSettings.cellSize;
        footprint.min = footprint.min - Vec3{margin, margin, candidate.profile.stepHeight};
        footprint.max = footprint.max + Vec3{margin, margin, MaximumCoverCode * CoverStep};
        std::map<std::uint32_t, std::uint32_t> remap;
        for (std::size_t index{}; index < scene.mesh.triangles.size(); ++index)
        {
            const auto &provenance = scene.triangleProvenance[index];
            if (provenance.geometrySource >= scene.geometrySources.size())
            {
                continue;
            }
            const auto &source = scene.geometrySources[provenance.geometrySource];
            if (source.sourceType != GeometrySourceType::Collision && source.sourceType != GeometrySourceType::Terrain)
            {
                continue;
            }
            const auto &face = scene.mesh.triangles[index];
            if (std::any_of(face.vertices.begin(), face.vertices.end(),
                            [&](auto vertex) { return vertex >= scene.mesh.vertices.size(); }))
            {
                continue;
            }
            const auto a = scene.mesh.vertices[face.vertices[0]];
            const auto b = scene.mesh.vertices[face.vertices[1]];
            const auto c = scene.mesh.vertices[face.vertices[2]];
            const auto normal = Cross(b - a, c - a);
            const auto magnitude = std::hypot(std::hypot(normal.x, normal.y), normal.z);
            AABB triangleBounds;
            for (const auto point : {a, b, c})
            {
                triangleBounds.Expand(point);
            }
            if (magnitude <= 0 || std::abs(normal.z) > magnitude * 0.7F || !triangleBounds.Intersects(footprint))
            {
                continue;
            }
            Triangle copied;
            for (std::size_t vertex{}; vertex < 3; ++vertex)
            {
                const auto old = face.vertices[vertex];
                const auto [entry, inserted] = remap.emplace(old, static_cast<std::uint32_t>(remap.size()));
                if (inserted)
                {
                    candidate.coverGeometry.vertices.push_back(scene.mesh.vertices[old]);
                }
                copied.vertices[vertex] = entry->second;
            }
            candidate.coverGeometry.triangles.push_back(copied);
        }
    }

    void BuildNavmeshCover(NavMesh &mesh, const CandidateNavMesh &candidate, std::span<const NavMesh> authored)
    {
        analysis::SpatialIndex walls;
        walls.Build(candidate.coverGeometry.triangles, candidate.coverGeometry.vertices);
        PreserveAuthoredRestrictions(mesh, authored,
                                     std::max(candidate.profile.stepHeight, candidate.recastSettings.cellHeight * 2));
        const auto distance = candidate.profile.agentRadius + 2 * candidate.recastSettings.cellSize;
        for (auto &face : mesh.polygons)
        {
            std::array<std::uint16_t, 3> cover{};
            for (std::size_t edge{}; edge < 3; ++edge)
            {
                if (!Boundary(face, edge))
                {
                    continue;
                }
                const auto a = mesh.vertices.at(face.vertices[edge]);
                const auto b = mesh.vertices.at(face.vertices[(edge + 1) % 3]);
                const auto opposite = mesh.vertices.at(face.vertices[(edge + 2) % 3]);
                cover[edge] = PhysicalCover(a, b, opposite, candidate.coverGeometry, walls, distance,
                                            candidate.profile.weldTolerance);
                if (!cover[edge])
                {
                    cover[edge] = RetainedCover(a, b, authored, candidate.profile.weldTolerance);
                }
            }
            EncodeCover(face, cover);
        }
    }
} // namespace navmesh::core
