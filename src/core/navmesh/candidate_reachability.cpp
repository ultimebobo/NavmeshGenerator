#include "core/navmesh/candidate_reachability.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <tuple>

namespace
{
    using namespace navmesh::core;
    constexpr auto Removed = std::numeric_limits<std::uint32_t>::max();

    /// Disjoint-set roots represent either selected CELL areas or connected floor networks.
    struct Connectivity
    {
        std::vector<std::size_t> parent;

        explicit Connectivity(std::size_t count) : parent(count)
        {
            std::iota(parent.begin(), parent.end(), 0);
        }

        std::size_t Root(std::size_t index)
        {
            while (parent[index] != index)
            {
                parent[index] = parent[parent[index]];
                index = parent[index];
            }
            return index;
        }

        void Join(std::size_t left, std::size_t right)
        {
            left = Root(left);
            right = Root(right);
            parent[std::max(left, right)] = std::min(left, right);
        }
    };

    /// Group adjacent selected CELLs without treating stacked floors as connected navigation.
    Connectivity SelectionAreas(std::span<CandidateReachabilityTarget> targets)
    {
        Connectivity areas(targets.size());
        std::map<std::tuple<std::uint32_t, float, float>, std::size_t> positions;
        for (std::size_t index{}; index < targets.size(); ++index)
        {
            const auto &target = targets[index];
            if (target.bounds &&
                !positions.emplace(std::tuple{*target.worldspaceId, target.bounds->min.x, target.bounds->min.y}, index)
                     .second)
            {
                throw std::runtime_error("Island filtering found duplicate exterior CELL ownership");
            }
        }
        for (std::size_t index{}; index < targets.size(); ++index)
        {
            const auto &target = targets[index];
            if (!target.bounds)
            {
                continue;
            }
            const auto &bounds = *target.bounds;
            for (const bool vertical : {true, false})
            {
                const auto neighbor = positions.find({*target.worldspaceId, vertical ? bounds.max.x : bounds.min.x,
                                                      vertical ? bounds.min.y : bounds.max.y});
                if (neighbor == positions.end())
                {
                    continue;
                }
                const auto &other = *targets[neighbor->second].bounds;
                if ((vertical && bounds.max.y == other.max.y) || (!vertical && bounds.max.x == other.max.x))
                {
                    areas.Join(index, neighbor->second);
                }
            }
        }
        return areas;
    }

    /// Stable compaction preserves flags and provenance; portal destinations are remapped in a later pass.
    void CompactCandidate(CandidateNavMesh &candidate, const std::vector<std::uint32_t> &polygonRemap)
    {
        std::vector<NavPolygon> polygons;
        std::vector<std::size_t> sources;
        std::vector<std::vector<std::size_t>> contributors;
        for (std::size_t index{}; index < polygonRemap.size(); ++index)
        {
            if (polygonRemap[index] == Removed)
            {
                continue;
            }
            auto polygon = candidate.mesh.polygons[index];
            for (auto &neighbor : polygon.neighbors)
            {
                if (neighbor != Removed)
                {
                    neighbor = polygonRemap.at(neighbor);
                }
            }
            polygons.push_back(polygon);
            sources.push_back(candidate.polygonSourceTriangles[index]);
            contributors.push_back(std::move(candidate.polygonContributingTriangles[index]));
        }
        candidate.statistics.rejectedUnreachable += polygonRemap.size() - polygons.size();
        candidate.mesh.polygons = std::move(polygons);
        candidate.polygonSourceTriangles = std::move(sources);
        candidate.polygonContributingTriangles = std::move(contributors);

        std::vector<CandidateRegion> regions;
        std::vector<std::uint32_t> polygonRegion(candidate.mesh.polygons.size(), Removed);
        for (auto &region : candidate.regions)
        {
            std::erase_if(region.polygons, [&](auto polygon) { return polygonRemap.at(polygon) == Removed; });
            if (region.polygons.empty())
            {
                continue;
            }
            region.id = static_cast<std::uint32_t>(regions.size());
            region.sourceTriangles.clear();
            region.exitFormIds.clear();
            region.reachesBorder = false;
            for (auto &polygon : region.polygons)
            {
                polygon = polygonRemap[polygon];
                polygonRegion[polygon] = region.id;
                const auto &values = candidate.polygonContributingTriangles[polygon];
                region.sourceTriangles.insert(region.sourceTriangles.end(), values.begin(), values.end());
            }
            std::sort(region.sourceTriangles.begin(), region.sourceTriangles.end());
            region.sourceTriangles.erase(std::unique(region.sourceTriangles.begin(), region.sourceTriangles.end()),
                                         region.sourceTriangles.end());
            regions.push_back(std::move(region));
        }
        candidate.regions = std::move(regions);
        std::erase_if(candidate.borderLinks,
                      [&](const auto &link) { return polygonRemap.at(link.polygon) == Removed; });
        for (auto &link : candidate.borderLinks)
        {
            link.polygon = polygonRemap[link.polygon];
            candidate.regions.at(polygonRegion.at(link.polygon)).reachesBorder = true;
        }
        for (auto &exit : candidate.exits)
        {
            exit.region.reset();
            if (exit.polygon && polygonRemap.at(*exit.polygon) != Removed)
            {
                exit.polygon = polygonRemap[*exit.polygon];
                exit.region = polygonRegion.at(*exit.polygon);
                candidate.regions.at(*exit.region).exitFormIds.push_back(exit.referenceId);
            }
            else
            {
                exit.polygon.reset();
            }
        }

        std::vector<Vec3> vertices;
        std::vector<std::uint32_t> vertexRemap(candidate.mesh.vertices.size(), Removed);
        for (auto &polygon : candidate.mesh.polygons)
        {
            for (auto &vertex : polygon.vertices)
            {
                if (vertexRemap[vertex] == Removed)
                {
                    vertexRemap[vertex] = static_cast<std::uint32_t>(vertices.size());
                    vertices.push_back(candidate.mesh.vertices[vertex]);
                }
                vertex = vertexRemap[vertex];
            }
        }
        candidate.mesh.vertices = std::move(vertices);
    }
} // namespace

namespace navmesh::core
{
    std::size_t RemoveCandidateIslands(std::span<CandidateReachabilityTarget> targets)
    {
        std::map<std::uint32_t, std::size_t> byCell;
        std::set<const CandidateNavMesh *> meshes;
        using PortalKey = std::tuple<std::uint32_t, std::uint32_t, std::uint8_t>;
        std::map<PortalKey, const CandidateBorderLink *> generatedPortals;
        std::vector<std::size_t> offsets{0};
        for (std::size_t index{}; index < targets.size(); ++index)
        {
            const auto &target = targets[index];
            if (!target.candidate || target.bounds.has_value() != target.worldspaceId.has_value() ||
                !byCell.emplace(target.cellId, index).second || !meshes.insert(target.candidate).second)
            {
                throw std::runtime_error("Island filtering requires unique valid CELL candidates");
            }
            if (target.bounds &&
                (!std::isfinite(target.bounds->min.x) || !std::isfinite(target.bounds->min.y) ||
                 !std::isfinite(target.bounds->max.x) || !std::isfinite(target.bounds->max.y) ||
                 target.bounds->max.x <= target.bounds->min.x || target.bounds->max.y <= target.bounds->min.y))
            {
                throw std::runtime_error("Island filtering requires finite positive exterior CELL extents");
            }
            RefreshCandidateTopology(*target.candidate);
            if (!target.candidate->topology.valid)
            {
                throw std::runtime_error(
                    std::format("CELL {:08X}: {}", target.cellId, target.candidate->topology.findings.front()));
            }
            for (const auto &exit : target.candidate->exits)
            {
                if (exit.polygon && *exit.polygon >= target.candidate->mesh.polygons.size())
                {
                    throw std::runtime_error("Island filtering found an invalid matched door polygon");
                }
            }
            for (const auto &link : target.candidate->borderLinks)
            {
                if (link.polygon >= target.candidate->mesh.polygons.size() || link.edge >= 3 || link.neighborEdge >= 3)
                {
                    throw std::runtime_error("Island filtering found an invalid portal edge");
                }
                if (link.generatedNeighborCell &&
                    !generatedPortals.emplace(PortalKey{target.cellId, link.polygon, link.edge}, &link).second)
                {
                    throw std::runtime_error("Island filtering found duplicate generated portal consumers");
                }
            }
            offsets.push_back(offsets.back() + target.candidate->mesh.polygons.size());
        }
        auto areas = SelectionAreas(targets);
        Connectivity floors(offsets.back());

        // Generated portals join floors across CELLs, but only doors and untouched
        // authored destinations supply external access. A roof spanning CELLs is still one island.
        for (std::size_t index{}; index < targets.size(); ++index)
        {
            const auto &target = targets[index];
            const auto &candidate = *target.candidate;
            for (std::size_t polygon{}; polygon < candidate.mesh.polygons.size(); ++polygon)
            {
                for (const auto neighbor : candidate.mesh.polygons[polygon].neighbors)
                {
                    if (neighbor != Removed)
                    {
                        floors.Join(offsets[index] + polygon, offsets[index] + neighbor);
                    }
                }
            }
            for (const auto &link : candidate.borderLinks)
            {
                if (!link.generatedNeighborCell)
                {
                    continue;
                }
                const auto destination = byCell.find(*link.generatedNeighborCell);
                if (destination == byCell.end())
                {
                    throw std::runtime_error("Island filtering found a missing generated portal destination");
                }
                const auto otherIndex = destination->second;
                const auto &other = *targets[otherIndex].candidate;
                const auto reverse =
                    generatedPortals.find({*link.generatedNeighborCell, link.neighborPolygon, link.neighborEdge});
                if (areas.Root(index) != areas.Root(otherIndex) || link.neighborPolygon >= other.mesh.polygons.size() ||
                    reverse == generatedPortals.end() || reverse->second->generatedNeighborCell != target.cellId ||
                    reverse->second->neighborPolygon != link.polygon || reverse->second->neighborEdge != link.edge)
                {
                    throw std::runtime_error("Island filtering found an invalid reciprocal generated portal");
                }
                floors.Join(offsets[index] + link.polygon, offsets[otherIndex] + link.neighborPolygon);
            }
        }

        std::vector<double> floorAreas(offsets.back());
        std::vector<bool> retained(offsets.back());
        std::map<std::size_t, std::size_t> largest;
        for (std::size_t index{}; index < targets.size(); ++index)
        {
            const auto &candidate = *targets[index].candidate;
            for (std::size_t polygon{}; polygon < candidate.mesh.polygons.size(); ++polygon)
            {
                const auto &face = candidate.mesh.polygons[polygon];
                const auto a = candidate.mesh.vertices[face.vertices[0]];
                const auto b = candidate.mesh.vertices[face.vertices[1]];
                const auto c = candidate.mesh.vertices[face.vertices[2]];
                floorAreas[floors.Root(offsets[index] + polygon)] +=
                    std::abs((static_cast<double>(b.x) - a.x) * (static_cast<double>(c.y) - a.y) -
                             (static_cast<double>(b.y) - a.y) * (static_cast<double>(c.x) - a.x)) *
                    0.5;
            }
            for (const auto &exit : candidate.exits)
            {
                if (exit.polygon)
                {
                    retained[floors.Root(offsets[index] + *exit.polygon)] = true;
                }
            }
            for (const auto &link : candidate.borderLinks)
            {
                if (!link.generatedNeighborCell)
                {
                    retained[floors.Root(offsets[index] + link.polygon)] = true;
                }
            }
        }
        // Area is summed over the complete network, independent of tessellation.
        // Separate selected areas and interiors each retain their primary floor.
        for (std::size_t index{}; index < targets.size(); ++index)
        {
            for (auto polygon = offsets[index]; polygon < offsets[index + 1]; ++polygon)
            {
                const auto root = floors.Root(polygon);
                const auto [found, inserted] = largest.emplace(areas.Root(index), root);
                if (!inserted && floorAreas[root] > floorAreas[found->second])
                {
                    found->second = root;
                }
            }
        }
        for (const auto &[area, root] : largest)
        {
            retained[root] = true;
        }

        // Plan all polygon identities before editing any candidate, then update
        // reciprocal destinations using the same maps so no kept portal dangles.
        std::size_t removed{};
        std::vector<std::vector<std::uint32_t>> remaps(targets.size());
        for (std::size_t index{}; index < targets.size(); ++index)
        {
            auto &remap = remaps[index];
            remap.resize(offsets[index + 1] - offsets[index], Removed);
            std::uint32_t next{};
            for (std::size_t polygon{}; polygon < remap.size(); ++polygon)
            {
                if (retained[floors.Root(offsets[index] + polygon)])
                {
                    remap[polygon] = next++;
                }
            }
            removed += remap.size() - next;
        }
        for (std::size_t index{}; index < targets.size(); ++index)
        {
            CompactCandidate(*targets[index].candidate, remaps[index]);
        }
        for (const auto &target : targets)
        {
            for (auto &link : target.candidate->borderLinks)
            {
                if (link.generatedNeighborCell)
                {
                    link.neighborPolygon = remaps[byCell.at(*link.generatedNeighborCell)].at(link.neighborPolygon);
                }
            }
            RefreshCandidateTopology(*target.candidate);
            if (!target.candidate->topology.valid)
            {
                throw std::runtime_error(
                    std::format("CELL {:08X}: {}", target.cellId, target.candidate->topology.findings.front()));
            }
        }
        return removed;
    }
} // namespace navmesh::core
