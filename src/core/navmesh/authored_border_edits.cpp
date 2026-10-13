#include "core/navmesh/authored_border_edits.h"
#include "core/navmesh/candidate.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <limits>

namespace navmesh::core
{
    namespace
    {
        /// The edit solver requires finite movement limits and a decoded finite triangle mesh.
        bool ValidEditInputs(const NavMesh &mesh, const NavigationProfile &profile)
        {
            if (!std::isfinite(profile.agentRadius) || profile.agentRadius < 0 || !std::isfinite(profile.stepHeight) ||
                profile.stepHeight < 0 || !std::isfinite(profile.maxSlopeDegrees) || profile.maxSlopeDegrees < 0 ||
                profile.maxSlopeDegrees >= 90)
            {
                return false;
            }
            const auto gap = profile.agentRadius * 2 + 16;
            const auto height =
                profile.stepHeight + gap * std::tan(profile.maxSlopeDegrees * 3.14159265358979323846F / 180.0F);
            if (!std::isfinite(gap) || !std::isfinite(height))
            {
                return false;
            }
            if (std::any_of(mesh.vertices.begin(), mesh.vertices.end(), [](Vec3 point)
                            { return !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z); }))
            {
                return false;
            }
            return std::none_of(mesh.polygons.begin(), mesh.polygons.end(),
                                [&](const auto &face)
                                {
                                    return std::any_of(face.vertices.begin(), face.vertices.end(),
                                                       [&](auto v) { return v >= mesh.vertices.size(); });
                                });
        }
    } // namespace

    bool ApplyAuthoredBorderVertexEdits(NavMesh &mesh, std::span<const AuthoredBorderVertexEdit> edits,
                                        const NavigationProfile &profile)
    {
        if (std::none_of(edits.begin(), edits.end(), [&](const auto &edit) { return edit.navmeshId == mesh.id; }))
        {
            return true;
        }
        if (!ValidEditInputs(mesh, profile))
        {
            return false;
        }
        const auto equal = [](Vec3 a, Vec3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; };
        const auto gap = profile.agentRadius * 2.0F + 16.0F;
        const auto height =
            profile.stepHeight + gap * std::tan(profile.maxSlopeDegrees * 3.14159265358979323846F / 180.0F);
        auto revised = mesh;
        std::map<std::uint32_t, Vec3> moved;
        for (const auto &edit : edits)
        {
            if (edit.navmeshId != mesh.id)
            {
                continue;
            }
            if (edit.vertex >= mesh.vertices.size() || !equal(mesh.vertices[edit.vertex], edit.original) ||
                !std::isfinite(edit.position.x) || !std::isfinite(edit.position.y) || !std::isfinite(edit.position.z) ||
                std::hypot(edit.position.x - edit.original.x, edit.position.y - edit.original.y) > gap ||
                std::abs(edit.position.z - edit.original.z) > height)
            {
                return false;
            }
            const auto [entry, inserted] = moved.emplace(edit.vertex, edit.position);
            if (!inserted && !equal(entry->second, edit.position))
            {
                return false;
            }
            revised.vertices[edit.vertex] = edit.position;
        }
        // Shared vertex identities propagate each change through its complete authored fan.
        // Compare each face against its own immutable slope, rather than a distant steep face.
        const auto shape = [](Vec3 a, Vec3 b, Vec3 c)
        {
            const auto u = b - a;
            const auto v = c - a;
            const auto area = static_cast<double>(u.x) * v.y - static_cast<double>(u.y) * v.x;
            const auto slope = std::atan2(std::hypot(static_cast<double>(u.y) * v.z - static_cast<double>(u.z) * v.y,
                                                     static_cast<double>(u.z) * v.x - static_cast<double>(u.x) * v.z),
                                          std::abs(area)) *
                               180.0 / 3.14159265358979323846;
            return std::pair{area, slope};
        };
        for (const auto &face : mesh.polygons)
        {
            if (std::none_of(face.vertices.begin(), face.vertices.end(), [&](auto v) { return moved.contains(v); }))
            {
                continue;
            }
            if (std::any_of(face.vertices.begin(), face.vertices.end(),
                            [&](auto v) { return v >= mesh.vertices.size(); }))
            {
                return false;
            }
            const auto [oldArea, oldSlope] = shape(mesh.vertices[face.vertices[0]], mesh.vertices[face.vertices[1]],
                                                   mesh.vertices[face.vertices[2]]);
            const auto [area, slope] = shape(revised.vertices[face.vertices[0]], revised.vertices[face.vertices[1]],
                                             revised.vertices[face.vertices[2]]);
            if (std::abs(area) <= 0.01 || area * oldArea <= 0 ||
                slope > std::max<double>(profile.maxSlopeDegrees, oldSlope) + 0.001)
            {
                return false;
            }
        }
        mesh = std::move(revised);
        return true;
    }
    bool CompleteAuthoredBorderFanEdits(const NavMesh &mesh, std::vector<AuthoredBorderVertexEdit> &edits,
                                        std::span<const std::uint32_t> pinned, const NavigationProfile &profile)
    {
        if (!ValidEditInputs(mesh, profile))
        {
            return false;
        }
        auto plan = edits;
        std::set<std::uint32_t> locked(pinned.begin(), pinned.end());
        auto revised = mesh;
        std::set<std::uint32_t> editedVertices;
        for (const auto &edit : plan)
        {
            if (edit.navmeshId != mesh.id)
            {
                continue;
            }
            if (edit.vertex >= mesh.vertices.size())
            {
                return false;
            }
            revised.vertices[edit.vertex] = edit.position;
            editedVertices.insert(edit.vertex);
        }
        std::set<std::uint32_t> fanVertices;
        for (const auto &face : mesh.polygons)
        {
            if (std::any_of(face.vertices.begin(), face.vertices.end(),
                            [&](auto v) { return editedVertices.contains(v); }))
            {
                fanVertices.insert(face.vertices.begin(), face.vertices.end());
            }
        }
        const auto gap = profile.agentRadius * 2 + 16;
        const auto heightLimit =
            profile.stepHeight + gap * std::tan(profile.maxSlopeDegrees * 3.14159265358979323846F / 180.0F);
        const auto slope = [](Vec3 a, Vec3 b, Vec3 c)
        {
            const auto u = b - a;
            const auto v = c - a;
            return std::atan2(std::hypot(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z),
                              std::abs(u.x * v.y - u.y * v.x));
        };
        const auto profileSlope = profile.maxSlopeDegrees * 3.14159265358979323846F / 180.0F;
        // Relax only local free heights. Each step strictly reduces total incident-face
        // slope excess, so a conflicting pinned rim cannot produce oscillating edits.
        const auto violation = [&](const NavMesh &target)
        {
            double error{};
            for (const auto &face : mesh.polygons)
            {
                const auto limit =
                    std::max(profileSlope, slope(mesh.vertices[face.vertices[0]], mesh.vertices[face.vertices[1]],
                                                 mesh.vertices[face.vertices[2]]));
                const auto current = slope(target.vertices[face.vertices[0]], target.vertices[face.vertices[1]],
                                           target.vertices[face.vertices[2]]);
                error += std::max(0.0F, current - limit - 0.00001F);
            }
            return error;
        };
        for (std::size_t iteration{}; iteration < mesh.vertices.size(); ++iteration)
        {
            auto checked = mesh;
            if (ApplyAuthoredBorderVertexEdits(checked, plan, profile))
            {
                edits = std::move(plan);
                return true;
            }
            const auto currentError = violation(revised);
            auto bestError = currentError;
            std::optional<std::pair<std::uint32_t, float>> best;
            for (std::uint32_t vertex{}; vertex < mesh.vertices.size(); ++vertex)
            {
                if (locked.contains(vertex) || !fanVertices.contains(vertex))
                {
                    continue;
                }
                auto low = mesh.vertices[vertex].z - heightLimit;
                auto high = mesh.vertices[vertex].z + heightLimit;
                bool incident{};
                for (const auto &face : mesh.polygons)
                {
                    const auto found = std::find(face.vertices.begin(), face.vertices.end(), vertex);
                    if (found == face.vertices.end())
                    {
                        continue;
                    }
                    incident = true;
                    const auto e = static_cast<std::size_t>(found - face.vertices.begin());
                    const auto a = revised.vertices[face.vertices[(e + 1) % 3]];
                    const auto b = revised.vertices[face.vertices[(e + 2) % 3]];
                    const auto point = revised.vertices[vertex];
                    const auto dx = b.x - a.x;
                    const auto dy = b.y - a.y;
                    const auto lengthSquared = dx * dx + dy * dy;
                    if (lengthSquared <= 0.01F)
                    {
                        low = 1;
                        high = 0;
                        break;
                    }
                    const auto length = std::sqrt(lengthSquared);
                    const auto limit = std::tan(
                        std::max(profileSlope, slope(mesh.vertices[face.vertices[0]], mesh.vertices[face.vertices[1]],
                                                     mesh.vertices[face.vertices[2]])));
                    const auto along = (b.z - a.z) / length;
                    if (std::abs(along) > limit + 0.00001F)
                    {
                        low = 1;
                        high = 0;
                        break;
                    }
                    const auto normal = std::sqrt(std::max(0.0F, limit * limit - along * along));
                    const auto fraction = ((point.x - a.x) * dx + (point.y - a.y) * dy) / lengthSquared;
                    const auto center = a.z + (b.z - a.z) * fraction;
                    const auto distance = std::abs(dx * (point.y - a.y) - dy * (point.x - a.x)) / length;
                    low = std::max(low, center - distance * normal);
                    high = std::min(high, center + distance * normal);
                }
                if (!incident || low > high)
                {
                    continue;
                }
                const auto height = std::clamp(mesh.vertices[vertex].z, low, high);
                auto trial = revised;
                trial.vertices[vertex].z = height;
                const auto error = violation(trial);
                if (error < bestError)
                {
                    bestError = error;
                    best = std::pair{vertex, height};
                }
            }
            if (!best)
            {
                return false;
            }
            const auto [vertex, height] = *best;
            revised.vertices[vertex].z = height;
            const auto found = std::find_if(plan.begin(), plan.end(), [&](const auto &edit)
                                            { return edit.navmeshId == mesh.id && edit.vertex == vertex; });
            if (found == plan.end())
            {
                plan.push_back({mesh.id, vertex, mesh.vertices[vertex], revised.vertices[vertex]});
            }
            else
            {
                found->position = revised.vertices[vertex];
            }
        }
        return false;
    }

} // namespace navmesh::core
