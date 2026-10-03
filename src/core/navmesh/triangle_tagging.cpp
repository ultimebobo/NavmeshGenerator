#include "core/navmesh/triangle_tagging.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace navmesh::core
{
    namespace
    {
        struct AuthoredSurface
        {
            std::array<Vec3, 3> vertices;
            float minimumX, maximumX, minimumY, maximumY;
            double determinant;
            std::uint16_t flags;
        };

        bool Finite(Vec3 point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

        std::vector<AuthoredSurface> CollectSurfaces(std::span<const NavMesh> authored)
        {
            std::vector<AuthoredSurface> surfaces;
            for (const auto &mesh : authored)
            {
                for (const auto &polygon : mesh.polygons)
                {
                    if ((polygon.flags & (1U << 3)) ||
                        std::any_of(polygon.vertices.begin(), polygon.vertices.end(),
                                    [&](auto index) { return index >= mesh.vertices.size(); }))
                    {
                        continue;
                    }
                    const auto a = mesh.vertices[polygon.vertices[0]];
                    const auto b = mesh.vertices[polygon.vertices[1]];
                    const auto c = mesh.vertices[polygon.vertices[2]];
                    const double determinant =
                        (static_cast<double>(b.y) - c.y) * (a.x - c.x) + (static_cast<double>(c.x) - b.x) * (a.y - c.y);
                    if (!Finite(a) || !Finite(b) || !Finite(c) || std::abs(determinant) < 1e-8)
                    {
                        continue;
                    }
                    surfaces.push_back({{a, b, c},
                                        std::min({a.x, b.x, c.x}),
                                        std::max({a.x, b.x, c.x}),
                                        std::min({a.y, b.y, c.y}),
                                        std::max({a.y, b.y, c.y}),
                                        determinant,
                                        polygon.flags});
                }
            }
            std::stable_sort(surfaces.begin(), surfaces.end(),
                             [](const auto &a, const auto &b) { return a.minimumX < b.minimumX; });
            return surfaces;
        }

        std::uint16_t MatchedFlags(Vec3 point, const std::vector<AuthoredSurface> &surfaces, float tolerance)
        {
            double closest = std::numeric_limits<double>::infinity();
            std::uint16_t flags{};
            // Bounds reject unrelated footprints; barycentric height matching separates stacked floors.
            // Unmarked floors participate so a nearby marked lower floor cannot tag a bridge above it.
            for (const auto &surface : surfaces)
            {
                if (surface.minimumX > point.x)
                {
                    break;
                }
                if (point.x > surface.maximumX || point.y < surface.minimumY || point.y > surface.maximumY)
                {
                    continue;
                }
                const auto &[a, b, c] = surface.vertices;
                const double u = ((static_cast<double>(b.y) - c.y) * (point.x - c.x) +
                                  (static_cast<double>(c.x) - b.x) * (point.y - c.y)) /
                                 surface.determinant;
                const double v = ((static_cast<double>(c.y) - a.y) * (point.x - c.x) +
                                  (static_cast<double>(a.x) - c.x) * (point.y - c.y)) /
                                 surface.determinant;
                const double w = 1.0 - u - v;
                if (std::min({u, v, w}) < -1e-6)
                {
                    continue;
                }
                const auto distance = std::abs(point.z - (u * a.z + v * b.z + w * c.z));
                if (distance <= tolerance && distance + 1e-6 < closest)
                {
                    closest = distance;
                    flags = surface.flags;
                }
                else if (distance <= tolerance && std::abs(distance - closest) < 1e-6)
                {
                    // Equally close contradictory evidence is classified conservatively.
                    flags &= surface.flags;
                    closest = std::min(closest, distance);
                }
            }
            return flags;
        }
    } // namespace

    void TagCandidateTriangles(CandidateNavMesh &candidate, std::span<const NavMesh> authored,
                               std::optional<float> waterHeight, bool enabled)
    {
        if (waterHeight && !std::isfinite(*waterHeight))
        {
            throw std::invalid_argument("Triangle tagging requires a finite water height");
        }
        candidate.triangleTagging = enabled;
        const auto surfaces = enabled ? CollectSurfaces(authored) : std::vector<AuthoredSurface>{};
        const auto heightTolerance = std::max(candidate.profile.stepHeight, candidate.recastSettings.cellHeight * 2);
        for (auto &polygon : candidate.mesh.polygons)
        {
            polygon.flags &= static_cast<std::uint16_t>(~(WaterFlag | PreferredPathFlag));
            if (!enabled)
            {
                continue;
            }
            const auto centroid =
                (candidate.mesh.vertices.at(polygon.vertices[0]) + candidate.mesh.vertices.at(polygon.vertices[1]) +
                 candidate.mesh.vertices.at(polygon.vertices[2])) /
                3.0F;
            const auto matched = MatchedFlags(centroid, surfaces, heightTolerance);
            polygon.flags |= matched & PreferredPathFlag;
            const bool water = waterHeight ? centroid.z < *waterHeight : (matched & WaterFlag) != 0;
            if (water)
            {
                polygon.flags |= WaterFlag;
            }
        }
    }
} // namespace navmesh::core
