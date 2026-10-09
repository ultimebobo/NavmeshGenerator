#include "core/scene/scene_exporter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <format>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <tuple>
#include <unordered_map>

namespace
{
    using namespace navmesh::core;
    // This order controls both the scene tree and the flattened mesh list in viewers.
    constexpr std::array LayerOrder{SceneLayer::OriginalNavmesh,   SceneLayer::ExistingNavmesh,
                                    SceneLayer::CandidateNavmesh,  SceneLayer::NavmeshLinks,
                                    SceneLayer::DiagnosticMarkers, SceneLayer::Terrain,
                                    SceneLayer::Collision,         SceneLayer::RenderFallback};
    struct Object
    {
        SceneLayer layer{};
        std::string name;
        std::string provenance;
        std::string classification;
        std::vector<Vec3> vertices;
        std::vector<std::array<std::uint32_t, 3>> triangles;
    };
    [[nodiscard]] bool Contains(const std::vector<SceneLayer> &layers, SceneLayer layer)
    {
        return std::find(layers.begin(), layers.end(), layer) != layers.end();
    }
    [[nodiscard]] const char *LayerName(SceneLayer layer)
    {
        switch (layer)
        {
        case SceneLayer::ExistingNavmesh:
            return "Neighboring NAVM";
        case SceneLayer::OriginalNavmesh:
            return "Original NAVM (current cell)";
        case SceneLayer::NavmeshLinks:
            return "NAVM links";
        case SceneLayer::Terrain:
            return "Terrain";
        case SceneLayer::Collision:
            return "Collision";
        case SceneLayer::RenderFallback:
            return "Render fallback";
        case SceneLayer::DiagnosticMarkers:
            return "Doors";
        case SceneLayer::CandidateNavmesh:
            return "Candidate NAVM";
        }
        return "Unknown";
    }
    [[nodiscard]] std::size_t MaterialIndex(SceneLayer layer, const std::string &classification = {})
    {
        if (classification == "water")
        {
            return 15;
        }
        if (classification == "preferred_path")
        {
            return 16;
        }
        if (classification == "water_preferred_path")
        {
            return 17;
        }
        if (classification == "door_linked" || classification == "entrance")
        {
            return 13;
        }
        if (layer == SceneLayer::NavmeshLinks)
        {
            return 14;
        }
        if (layer == SceneLayer::Terrain)
        {
            return 1; // brown/green
        }
        if (layer == SceneLayer::Collision)
        {
            return 2; // gray
        }
        if (layer == SceneLayer::RenderFallback)
        {
            return 3; // purple
        }
        if (layer == SceneLayer::CandidateNavmesh)
        {
            return 12;
        }
        if (classification.empty())
        {
            return 0; // unclassified NAVM
        }
        if (classification == "supported")
        {
            return 4;
        }
        if (classification == "floating")
        {
            return 5;
        }
        if (classification == "buried")
        {
            return 6;
        }
        if (classification == "too_steep")
        {
            return 7;
        }
        if (classification == "blocked")
        {
            return 8;
        }
        if (classification == "out_of_coverage")
        {
            return 9;
        }
        if (classification == "ambiguous")
        {
            return 10;
        }
        return 11; // unsupported or unknown
    }
    [[nodiscard]] std::string Escape(const std::string &value)
    {
        std::string escaped;
        for (const char c : value)
        {
            if (c == '\\')
            {
                escaped += "\\\\";
            }
            else if (c == '\"')
            {
                escaped += "\\\"";
            }
            else if (c == '\n')
            {
                escaped += "\\n";
            }
            else if (c == '\r')
            {
                escaped += "\\r";
            }
            else
            {
                escaped += c;
            }
        }
        return escaped;
    }
    [[nodiscard]] AABB TriangleBounds(const Mesh &mesh, const Triangle &triangle)
    {
        AABB result;
        for (const auto index : triangle.vertices)
        {
            if (index < mesh.vertices.size())
            {
                result.Expand(mesh.vertices[index]);
            }
        }
        return result;
    }
    // A coarse grid makes bounds selection proportional to the local selection,
    // not to every triangle. Source triangles remain streamed into GLB objects.
    class TriangleGrid
    {
      public:
        explicit TriangleGrid(const Mesh &mesh) : mesh_(mesh)
        {
            for (std::size_t index = 0; index < mesh.triangles.size(); ++index)
            {
                const auto box = TriangleBounds(mesh, mesh.triangles[index]);
                if (!box.IsValid())
                {
                    continue;
                }
                const auto minX = Cell(box.min.x), maxX = Cell(box.max.x), minY = Cell(box.min.y),
                           maxY = Cell(box.max.y);
                for (int x = minX; x <= maxX; ++x)
                {
                    for (int y = minY; y <= maxY; ++y)
                    {
                        cells_[{x, y}].push_back(index);
                    }
                }
            }
        }
        [[nodiscard]] std::vector<std::size_t> Query(const std::optional<SceneBounds> &bounds) const
        {
            if (!bounds)
            {
                std::vector<std::size_t> all(mesh_.triangles.size());
                for (std::size_t i{}; i < all.size(); ++i)
                {
                    all[i] = i;
                }
                return all;
            }
            std::set<std::size_t> selected;
            const auto &box = bounds->world;
            for (int x = Cell(box.min.x); x <= Cell(box.max.x); ++x)
            {
                for (int y = Cell(box.min.y); y <= Cell(box.max.y); ++y)
                {
                    const auto found = cells_.find({x, y});
                    if (found == cells_.end())
                    {
                        continue;
                    }
                    for (const auto index : found->second)
                    {
                        if (TriangleBounds(mesh_, mesh_.triangles[index]).Intersects(box))
                        {
                            selected.insert(index);
                        }
                    }
                }
            }
            return {selected.begin(), selected.end()};
        }

      private:
        [[nodiscard]] static int Cell(float value)
        {
            return static_cast<int>(std::floor(value / 4096.0F));
        }
        struct Key
        {
            int x{}, y{};
            [[nodiscard]] bool operator<(const Key &other) const noexcept
            {
                return x != other.x ? x < other.x : y < other.y;
            }
        };
        const Mesh &mesh_;
        std::map<Key, std::vector<std::size_t>> cells_;
    };
    void AppendDoorMarker(Object &object, const Vec3 &position)
    {
        // Small world-space pyramid, readable in viewers that hide POINTS.
        constexpr float r = 12.0F, h = 24.0F;
        const auto base = static_cast<std::uint32_t>(object.vertices.size());
        object.vertices.insert(object.vertices.end(), {{position.x - r, position.y - r, position.z},
                                                       {position.x + r, position.y - r, position.z},
                                                       {position.x + r, position.y + r, position.z},
                                                       {position.x - r, position.y + r, position.z},
                                                       {position.x, position.y, position.z + h}});
        object.triangles.insert(object.triangles.end(), {{base, base + 1, base + 4},
                                                         {base + 1, base + 2, base + 4},
                                                         {base + 2, base + 3, base + 4},
                                                         {base + 3, base, base + 4}});
    }
    /// Resolve only valid, selected triangles so door colors and link endpoints obey identical culling.
    [[nodiscard]] std::optional<std::array<Vec3, 3>> SelectedPolygon(const NavMesh &mesh, std::size_t index,
                                                                     const std::optional<SceneBounds> &selection)
    {
        if (index >= mesh.polygons.size())
        {
            return std::nullopt;
        }
        std::array<Vec3, 3> points;
        AABB bounds;
        for (std::size_t vertex{}; vertex < points.size(); ++vertex)
        {
            const auto source = mesh.polygons[index].vertices[vertex];
            if (source >= mesh.vertices.size())
            {
                return std::nullopt;
            }
            points[vertex] = mesh.vertices[source];
            bounds.Expand(points[vertex]);
        }
        if (selection && !bounds.Intersects(selection->world))
        {
            return std::nullopt;
        }
        return points;
    }
    void AppendPolygon(Object &object, const std::array<Vec3, 3> &points)
    {
        const auto base = static_cast<std::uint32_t>(object.vertices.size());
        object.vertices.insert(object.vertices.end(), points.begin(), points.end());
        object.triangles.push_back({base, base + 1, base + 2});
    }
    /// A solid prism follows the recorded portal edge in Skyrim world coordinates,
    /// lifted in Z so triangle-only viewers can distinguish it from the NAVM surface.
    void AppendLinkBar(Object &object, Vec3 start, Vec3 end)
    {
        start.z += 8.0F;
        end.z += 8.0F;
        const auto delta = end - start;
        const auto length = std::hypot(delta.x, delta.y, delta.z);
        if (!std::isfinite(length) || length <= 0.001F)
        {
            return;
        }
        const auto direction = delta / length;
        const auto horizontalLength = std::hypot(direction.x, direction.y);
        const Vec3 side = horizontalLength > 0.001F
                              ? Vec3{-direction.y / horizontalLength, direction.x / horizontalLength, 0}
                              : Vec3{1, 0, 0};
        const Vec3 up{direction.y * side.z - direction.z * side.y, direction.z * side.x - direction.x * side.z,
                      direction.x * side.y - direction.y * side.x};
        const auto base = static_cast<std::uint32_t>(object.vertices.size());
        for (const auto &center : {start, end})
        {
            object.vertices.push_back(center - side * 4.0F - up * 4.0F);
            object.vertices.push_back(center + side * 4.0F - up * 4.0F);
            object.vertices.push_back(center + side * 4.0F + up * 4.0F);
            object.vertices.push_back(center - side * 4.0F + up * 4.0F);
        }
        constexpr std::array<std::array<std::uint32_t, 3>, 12> faces{{{0, 2, 1},
                                                                      {0, 3, 2},
                                                                      {4, 5, 6},
                                                                      {4, 6, 7},
                                                                      {0, 1, 5},
                                                                      {0, 5, 4},
                                                                      {1, 2, 6},
                                                                      {1, 6, 5},
                                                                      {2, 3, 7},
                                                                      {2, 7, 6},
                                                                      {3, 0, 4},
                                                                      {3, 4, 7}}};
        for (const auto &face : faces)
        {
            object.triangles.push_back({base + face[0], base + face[1], base + face[2]});
        }
    }
    template <typename T> void Append(std::vector<std::uint8_t> &bytes, const T &value)
    {
        const auto start = bytes.size();
        bytes.resize(start + sizeof(T));
        std::memcpy(bytes.data() + start, &value, sizeof(T));
    }
    void Align(std::vector<std::uint8_t> &bytes)
    {
        while (bytes.size() % 4 != 0)
        {
            bytes.push_back(0);
        }
    }
} // namespace

namespace navmesh::core
{
    SceneExportResult WriteCombinedGlb(const std::filesystem::path &outputPath, const Scene &scene,
                                       const std::vector<NavMesh> &navmeshes,
                                       const std::vector<DiagnosticMarker> &markers,
                                       const reproducibility::ExportMetadata &metadata,
                                       const SceneExportOptions &options)
    {
        SceneExportResult result;
        std::vector<Object> objects;
        std::map<std::pair<SceneLayer, std::size_t>, std::size_t> objectForSource;
        const auto appendMesh = [&](const Mesh &mesh, const std::vector<TriangleProvenance> &triangleProvenance)
        {
            const TriangleGrid index(mesh);
            const auto selected = index.Query(options.bounds);
            result.culledTriangles += mesh.triangles.size() - selected.size();
            for (const auto triangleIndex : selected)
            {
                if (triangleIndex >= triangleProvenance.size())
                {
                    continue;
                }
                const auto &provenance = triangleProvenance[triangleIndex];
                if (provenance.geometrySource >= scene.geometrySources.size())
                {
                    continue;
                }
                const auto &source = scene.geometrySources[provenance.geometrySource];
                const auto layer = source.sourceType == GeometrySourceType::Terrain     ? SceneLayer::Terrain
                                   : source.sourceType == GeometrySourceType::Collision ? SceneLayer::Collision
                                                                                        : SceneLayer::RenderFallback;
                if (!Contains(options.layers, layer))
                {
                    continue;
                }
                const auto key = std::pair{layer, provenance.geometrySource};
                auto [where, added] = objectForSource.emplace(key, objects.size());
                if (added)
                {
                    const auto name =
                        std::format("{}: {:08X} {}", LayerName(layer), source.reference.formId,
                                    source.modelPath.empty() ? source.reference.recordType : source.modelPath);
                    const auto provenanceJson =
                        std::format("{{\"plugin\":\"{}\",\"formId\":\"{:08X}\",\"recordType\":\"{}\",\"model\":\"{}\","
                                    "\"sourceType\":\"{}\",\"confidence\":{},\"navigationObstacle\":{}}}",
                                    Escape(source.reference.plugin), source.reference.formId,
                                    Escape(source.reference.recordType), Escape(source.modelPath), LayerName(layer),
                                    source.confidence, source.navigationObstacle ? "true" : "false");
                    objects.push_back({layer, name, provenanceJson});
                }
                auto &object = objects[where->second];
                const auto &triangle = mesh.triangles[triangleIndex];
                const auto base = static_cast<std::uint32_t>(object.vertices.size());
                bool valid = true;
                for (const auto vertex : triangle.vertices)
                {
                    if (vertex >= mesh.vertices.size())
                    {
                        valid = false;
                    }
                }
                if (!valid)
                {
                    continue;
                }
                for (const auto vertex : triangle.vertices)
                {
                    object.vertices.push_back(mesh.vertices[vertex]);
                }
                object.triangles.push_back({base, base + 1, base + 2});
            }
        };
        appendMesh(scene.mesh, scene.triangleProvenance);
        appendMesh(scene.renderFallbackMesh, scene.renderFallbackTriangleProvenance);
        std::map<std::pair<std::uint32_t, std::size_t>, std::string> polygonClassifications;
        for (const auto &marker : markers)
        {
            if (marker.navmeshFormId)
            {
                polygonClassifications.try_emplace(std::pair{*marker.navmeshFormId, marker.navmeshPolygon},
                                                   marker.classification);
            }
        }
        if (Contains(options.layers, SceneLayer::ExistingNavmesh))
        {
            for (const auto &navmesh : navmeshes)
            {
                const bool isOriginal =
                    std::any_of(metadata.selectedCells.begin(), metadata.selectedCells.end(),
                                [&](const auto *cell)
                                {
                                    return std::any_of(cell->navMeshes.begin(), cell->navMeshes.end(),
                                                       [&](const auto &mesh) { return mesh.id == navmesh.id; });
                                }) ||
                    metadata.selectedCell &&
                        std::any_of(metadata.selectedCell->navMeshes.begin(), metadata.selectedCell->navMeshes.end(),
                                    [&](const auto &mesh) { return mesh.id == navmesh.id; });
                std::set<std::uint32_t> doorPolygons;
                for (const auto &door : navmesh.doorLinks)
                {
                    doorPolygons.insert(door.polygon);
                }
                std::map<std::string, Object> classifiedObjects;
                for (std::size_t polygonIndex{}; polygonIndex < navmesh.polygons.size(); ++polygonIndex)
                {
                    const auto points = SelectedPolygon(navmesh, polygonIndex, options.bounds);
                    if (!points)
                    {
                        continue;
                    }
                    const auto found = polygonClassifications.find({navmesh.id, polygonIndex});
                    std::string classification;
                    if (found != polygonClassifications.end())
                    {
                        classification = found->second;
                    }
                    if (doorPolygons.contains(static_cast<std::uint32_t>(polygonIndex)))
                    {
                        classification = "door_linked";
                    }
                    auto [it, inserted] = classifiedObjects.try_emplace(classification);
                    auto &object = it->second;
                    if (inserted)
                    {
                        object.layer = isOriginal ? SceneLayer::OriginalNavmesh : SceneLayer::ExistingNavmesh;
                        object.classification = classification;
                        object.name = std::format("Existing NAVM {:08X}: {}", navmesh.id,
                                                  classification.empty() ? "unclassified" : classification);
                        object.provenance =
                            std::format("{{\"navmeshFormId\":\"{:08X}\",\"classification\":\"{}\"}}", navmesh.id,
                                        Escape(classification.empty() ? "unclassified" : classification));
                    }
                    AppendPolygon(object, *points);
                }
                for (auto &[_, object] : classifiedObjects)
                {
                    objects.push_back(std::move(object));
                }
            }
        }
        const auto appendCandidate =
            [&](const NavMesh &candidate, const std::vector<CandidateExit> *entrances, std::uint32_t cellId)
        {
            const auto name = cellId ? std::format("Candidate NAVM CELL {:08X}", cellId) : "Candidate NAVM";
            const auto provenance =
                cellId ? std::format("{{\"kind\":\"neutral_candidate\",\"cellFormId\":\"{:08X}\"}}", cellId)
                       : "{\"kind\":\"neutral_candidate\"}";
            Object object{SceneLayer::CandidateNavmesh, name, provenance};
            Object doors{SceneLayer::CandidateNavmesh, name + ": door_linked",
                         "{\"kind\":\"neutral_candidate\",\"classification\":\"door_linked\"}", "door_linked"};
            std::map<std::string, Object> tagged;
            std::set<std::uint32_t> doorPolygons;
            if (entrances)
            {
                for (const auto &door : *entrances)
                {
                    if (door.polygon)
                    {
                        doorPolygons.insert(*door.polygon);
                    }
                }
            }
            for (std::size_t polygonIndex{}; polygonIndex < candidate.polygons.size(); ++polygonIndex)
            {
                const auto points = SelectedPolygon(candidate, polygonIndex, options.bounds);
                if (!points)
                {
                    continue;
                }
                const auto flags = candidate.polygons[polygonIndex].flags;
                const bool water = (flags & WaterFlag) != 0;
                const bool preferred = (flags & PreferredPathFlag) != 0;
                if (water || preferred)
                {
                    const std::string classification = water && preferred ? "water_preferred_path"
                                                       : water            ? "water"
                                                                          : "preferred_path";
                    auto [entry, added] = tagged.try_emplace(classification);
                    if (added)
                    {
                        entry->second = {
                            SceneLayer::CandidateNavmesh, name + ": " + classification,
                            std::format("{{\"kind\":\"neutral_candidate\",\"classification\":\"{}\"}}", classification),
                            classification};
                    }
                    AppendPolygon(entry->second, *points);
                }
                else
                {
                    AppendPolygon(doorPolygons.contains(static_cast<std::uint32_t>(polygonIndex)) ? doors : object,
                                  *points);
                }
            }
            objects.push_back(std::move(object));
            objects.push_back(std::move(doors));
            for (auto &[_, group] : tagged)
            {
                objects.push_back(std::move(group));
            }
        };
        if (Contains(options.layers, SceneLayer::CandidateNavmesh))
        {
            if (options.candidates.empty() && options.candidateNavmesh)
            {
                appendCandidate(*options.candidateNavmesh, options.candidateEntrances, 0);
            }
            for (const auto &view : options.candidates)
            {
                appendCandidate(view.candidate->mesh, &view.candidate->exits, view.cellFormId);
            }
        }
        if (options.candidateEntrances && Contains(options.layers, SceneLayer::DiagnosticMarkers))
        {
            for (const auto &entrance : *options.candidateEntrances)
            {
                if (options.bounds && !options.bounds->world.Contains(entrance.position))
                {
                    continue;
                }
                Object object{SceneLayer::DiagnosticMarkers, std::format("Door {:08X}", entrance.referenceId),
                              std::format("{{\"kind\":\"entrance\",\"referenceFormId\":\"{:08X}\",\"region\":{}}}",
                                          entrance.referenceId,
                                          entrance.region ? std::to_string(*entrance.region) : "null"),
                              "entrance"};
                AppendDoorMarker(object, entrance.position);
                objects.push_back(std::move(object));
            }
        }

        // Resolve destinations against displayed authored geometry. Reciprocal
        // authored edges share a bar; candidate triangles have separate numbering.
        std::unordered_map<std::uint32_t, const NavMesh *> meshById;
        for (const auto &mesh : navmeshes)
        {
            meshById.emplace(mesh.id, &mesh);
        }
        std::set<std::tuple<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t>> authoredPairs;
        const bool showCandidateLinks = (options.candidateNavmesh || !options.candidates.empty()) &&
                                        Contains(options.layers, SceneLayer::CandidateNavmesh);
        const auto isReplaced = [&](std::uint32_t id)
        {
            for (const auto &view : options.candidates)
            {
                for (const auto *cell : metadata.selectedCells)
                {
                    if (showCandidateLinks && cell->id == view.cellFormId &&
                        std::any_of(cell->navMeshes.begin(), cell->navMeshes.end(),
                                    [&](const auto &mesh) { return mesh.id == id; }))
                    {
                        return true;
                    }
                }
            }
            return showCandidateLinks && metadata.selectedCell &&
                   std::any_of(metadata.selectedCell->navMeshes.begin(), metadata.selectedCell->navMeshes.end(),
                               [&](const auto &mesh) { return mesh.id == id; });
        };
        const auto appendLink = [&](const NavMesh &source, std::uint32_t sourcePolygon, std::uint8_t sourceEdge,
                                    std::uint32_t targetId, std::uint32_t targetPolygon, bool generated,
                                    const NavMesh *generatedTarget = nullptr, std::uint32_t sourceCell = 0)
        {
            const auto target = meshById.find(targetId);
            if (!generatedTarget && target == meshById.end())
            {
                return;
            }
            const auto sourcePoints = SelectedPolygon(source, sourcePolygon, options.bounds);
            const auto targetPoints =
                SelectedPolygon(generatedTarget ? *generatedTarget : *target->second, targetPolygon, options.bounds);
            if (!sourcePoints || !targetPoints)
            {
                return;
            }
            if (!generated)
            {
                const auto first = std::pair{source.id, sourcePolygon};
                const auto second = std::pair{targetId, targetPolygon};
                const auto [a, b] = std::minmax(first, second);
                if (a == b || !authoredPairs.emplace(a.first, a.second, b.first, b.second).second)
                {
                    return;
                }
            }
            Object object{SceneLayer::NavmeshLinks,
                          generated ? (sourceCell ? std::format("Candidate link CELL {:08X}:{} -> {:08X}:{}",
                                                                sourceCell, sourcePolygon, targetId, targetPolygon)
                                                  : std::format("Candidate link {} -> {:08X}:{}", sourcePolygon,
                                                                targetId, targetPolygon))
                                    : std::format("Authored link {:08X}:{} -> {:08X}:{}", source.id, sourcePolygon,
                                                  targetId, targetPolygon),
                          std::format("{{\"kind\":\"{}\",\"sourceNavmeshFormId\":\"{:08X}\",\"sourcePolygon\":{},"
                                      "\"sourceEdge\":{},\"targetNavmeshFormId\":\"{:08X}\",\"targetPolygon\":{}}}",
                                      generated ? "candidate_border_link" : "authored_external_link", source.id,
                                      sourcePolygon, sourceEdge, targetId, targetPolygon)};
            if (sourceCell)
            {
                object.provenance = std::format(
                    "{{\"kind\":\"candidate_border_link\",\"sourceCellFormId\":\"{:08X}\",\"sourcePolygon\":{},"
                    "\"sourceEdge\":{},\"{}\":\"{:08X}\",\"targetPolygon\":{},\"targetKind\":\"{}\"}}",
                    sourceCell, sourcePolygon, sourceEdge, generatedTarget ? "targetCellFormId" : "targetNavmeshFormId",
                    targetId, targetPolygon, generatedTarget ? "generated_cell" : "authored_navmesh");
            }
            // The consuming edge fixes the portal's exact endpoints, including
            // authored border drift and slope. The target remains required for culling.
            AppendLinkBar(object, (*sourcePoints)[sourceEdge], (*sourcePoints)[(sourceEdge + 1) % 3]);
            objects.push_back(std::move(object));
        };
        if (Contains(options.layers, SceneLayer::ExistingNavmesh))
        {
            for (const auto &mesh : navmeshes)
            {
                for (const auto &link : mesh.externalLinks)
                {
                    if (link.edge < 3 && !isReplaced(mesh.id) && !isReplaced(link.navmeshId))
                    {
                        appendLink(mesh, link.polygon, link.edge, link.navmeshId, link.targetPolygon, false);
                    }
                }
            }
        }
        if (options.candidates.empty() && options.candidateNavmesh && options.candidateBorderLinks &&
            Contains(options.layers, SceneLayer::CandidateNavmesh) &&
            Contains(options.layers, SceneLayer::ExistingNavmesh))
        {
            for (const auto &link : *options.candidateBorderLinks)
            {
                if (link.edge < 3 && link.neighborEdge < 3)
                {
                    appendLink(*options.candidateNavmesh, link.polygon, link.edge, link.neighborNavmeshId,
                               link.neighborPolygon, true);
                }
            }
        }
        if (Contains(options.layers, SceneLayer::CandidateNavmesh) &&
            Contains(options.layers, SceneLayer::ExistingNavmesh))
        {
            std::map<std::uint32_t, const CandidateNavMesh *> candidatesByCell;
            for (const auto &view : options.candidates)
            {
                candidatesByCell.emplace(view.cellFormId, view.candidate);
            }
            std::set<std::tuple<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t>> generatedPairs;
            // Generated destinations use CELL identities until the writer allocates NAVM records.
            // Resolve them directly to finalized candidate geometry, and draw reciprocal seams once.
            for (const auto &view : options.candidates)
            {
                for (const auto &link : view.candidate->borderLinks)
                {
                    if (link.edge >= 3 || link.neighborEdge >= 3)
                    {
                        continue;
                    }
                    const NavMesh *target{};
                    auto targetId = link.neighborNavmeshId;
                    if (link.generatedNeighborCell)
                    {
                        const auto found = candidatesByCell.find(*link.generatedNeighborCell);
                        if (found == candidatesByCell.end())
                        {
                            continue;
                        }
                        target = &found->second->mesh;
                        targetId = found->first;
                        const auto sourcePair = std::pair{view.cellFormId, link.polygon};
                        const auto targetPair = std::pair{targetId, link.neighborPolygon};
                        const auto [a, b] = std::minmax(sourcePair, targetPair);
                        if (!generatedPairs.emplace(a.first, a.second, b.first, b.second).second)
                        {
                            continue;
                        }
                    }
                    appendLink(view.candidate->mesh, link.polygon, link.edge, targetId, link.neighborPolygon, true,
                               target, view.cellFormId);
                }
            }
        }
        std::stable_sort(objects.begin(), objects.end(),
                         [](const Object &a, const Object &b)
                         {
                             return std::find(LayerOrder.begin(), LayerOrder.end(), a.layer) <
                                    std::find(LayerOrder.begin(), LayerOrder.end(), b.layer);
                         });
        std::vector<std::uint8_t> binary;
        std::vector<std::string> bufferViews, accessors, meshes, nodes, provenanceObjects;
        std::array<std::vector<std::size_t>, LayerOrder.size()> layerChildren;
        for (const auto &object : objects)
        {
            if (object.triangles.empty())
            {
                continue;
            }
            Align(binary);
            const auto positionOffset = binary.size();
            AABB bounds;
            for (const auto &vertex : object.vertices)
            {
                Append(binary, vertex.x);
                Append(binary, vertex.y);
                Append(binary, vertex.z);
                bounds.Expand(vertex);
            }
            const auto positionView = bufferViews.size();
            bufferViews.push_back(std::format("{{\"buffer\":0,\"byteOffset\":{},\"byteLength\":{},\"target\":34962}}",
                                              positionOffset, object.vertices.size() * sizeof(float) * 3));
            const auto positionAccessor = accessors.size();
            accessors.push_back(std::format("{{\"bufferView\":{},\"componentType\":5126,\"count\":{},\"type\":\"VEC3\","
                                            "\"min\":[{},{},{}],\"max\":[{},{},{}]}}",
                                            positionView, object.vertices.size(), bounds.min.x, bounds.min.y,
                                            bounds.min.z, bounds.max.x, bounds.max.y, bounds.max.z));
            Align(binary);
            const auto indexOffset = binary.size();
            for (const auto &tri : object.triangles)
            {
                for (const auto value : tri)
                {
                    Append(binary, value);
                }
            }
            const auto indexView = bufferViews.size();
            bufferViews.push_back(std::format("{{\"buffer\":0,\"byteOffset\":{},\"byteLength\":{},\"target\":34963}}",
                                              indexOffset, object.triangles.size() * 3 * sizeof(std::uint32_t)));
            const auto indexAccessor = accessors.size();
            accessors.push_back(
                std::format("{{\"bufferView\":{},\"componentType\":5125,\"count\":{},\"type\":\"SCALAR\"}}", indexView,
                            object.triangles.size() * 3));
            const auto meshIndex = meshes.size();
            meshes.push_back(std::format("{{\"name\":\"{}\",\"primitives\":[{{\"attributes\":{{\"POSITION\":{}}},"
                                         "\"indices\":{},\"material\":{}}}]}}",
                                         Escape(object.name), positionAccessor, indexAccessor,
                                         MaterialIndex(object.layer, object.classification)));
            layerChildren[static_cast<std::size_t>(object.layer)].push_back(nodes.size());
            nodes.push_back(std::format("{{\"name\":\"{}\",\"mesh\":{},\"extras\":{{\"provenance\":{}}}}}",
                                        Escape(object.name), meshIndex, object.provenance));
            if (options.detailedProvenance)
            {
                provenanceObjects.push_back(std::format(
                    "{{\"name\":\"{}\",\"layer\":\"{}\",\"triangles\":{},\"provenance\":{}}}", Escape(object.name),
                    LayerName(object.layer), object.triangles.size(), object.provenance));
            }
            else
            {
                provenanceObjects.push_back(std::format("{{\"name\":\"{}\",\"layer\":\"{}\",\"triangles\":{}}}",
                                                        Escape(object.name), LayerName(object.layer),
                                                        object.triangles.size()));
            }
            result.triangles += object.triangles.size();
            ++result.objects;
        }
        const std::array<const char *, 18> materialNames{"Unclassified NAVM (cyan)",
                                                         "Terrain (brown-green)",
                                                         "Collision (gray)",
                                                         "Render fallback (purple)",
                                                         "Supported (green)",
                                                         "Floating (orange)",
                                                         "Buried (red)",
                                                         "Too steep (yellow)",
                                                         "Blocked (magenta)",
                                                         "Out of coverage (blue)",
                                                         "Ambiguous (violet)",
                                                         "Unsupported or unknown (dark gray)",
                                                         "Candidate NAVM (blue-green)",
                                                         "Door / door-linked NAVM (orange)",
                                                         "NAVM link (green)",
                                                         "Water NAVM (blue)",
                                                         "Preferred path NAVM (yellow)",
                                                         "Water preferred path NAVM (teal)"};
        const std::array<std::array<float, 4>, 18> colors{{{{0.0F, 0.85F, 0.95F, 0.70F}},
                                                           {{0.35F, 0.48F, 0.16F, 1.0F}},
                                                           {{0.46F, 0.46F, 0.50F, 1.0F}},
                                                           {{0.58F, 0.25F, 0.75F, 0.80F}},
                                                           {{0.10F, 0.70F, 0.25F, 0.75F}},
                                                           {{1.0F, 0.50F, 0.05F, 0.75F}},
                                                           {{0.90F, 0.10F, 0.10F, 0.75F}},
                                                           {{0.95F, 0.82F, 0.08F, 0.75F}},
                                                           {{0.85F, 0.05F, 0.60F, 0.75F}},
                                                           {{0.08F, 0.35F, 0.95F, 0.75F}},
                                                           {{0.48F, 0.20F, 0.90F, 0.75F}},
                                                           {{0.25F, 0.28F, 0.32F, 0.75F}},
                                                           {{0.02F, 0.78F, 0.72F, 0.8F}},
                                                           {{1.0F, 0.45F, 0.0F, 1.0F}},
                                                           {{0.05F, 1.0F, 0.10F, 1.0F}},
                                                           {{0.05F, 0.30F, 1.0F, 0.8F}},
                                                           {{1.0F, 0.85F, 0.10F, 0.8F}},
                                                           {{0.10F, 0.85F, 0.85F, 0.8F}}}};
        std::vector<std::string> materials;
        for (std::size_t i{}; i < materialNames.size(); ++i)
        {
            materials.push_back(
                std::format("{{\"name\":\"{}\",\"doubleSided\":true,\"alphaMode\":\"{}\",\"pbrMetallicRoughness\":{{"
                            "\"baseColorFactor\":[{},{},{},{}],\"metallicFactor\":0,\"roughnessFactor\":0.82}}}}",
                            materialNames[i], colors[i][3] < 1.0F ? "BLEND" : "OPAQUE", colors[i][0], colors[i][1],
                            colors[i][2], colors[i][3]));
        }
        const auto join = [](const auto &values)
        {
            std::ostringstream out;
            for (std::size_t i{}; i < values.size(); ++i)
            {
                out << (i ? "," : "") << values[i];
            }
            return out.str();
        };
        // Keep a stable, visible hierarchy even when a requested layer has no
        // extracted triangles. A viewer can then distinguish unavailable
        // terrain/collision/NAVM from an exporter silently losing that layer.
        std::vector<std::size_t> rootNodes;
        auto exportedLayers = options.layers;
        if (Contains(options.layers, SceneLayer::ExistingNavmesh))
        {
            exportedLayers.push_back(SceneLayer::OriginalNavmesh);
            exportedLayers.push_back(SceneLayer::NavmeshLinks);
        }
        std::vector<std::string> provenanceLayers;
        for (const auto layer : LayerOrder)
        {
            if (!Contains(exportedLayers, layer))
            {
                continue;
            }
            std::ostringstream children;
            const auto &values = layerChildren[static_cast<std::size_t>(layer)];
            for (std::size_t childIndex{}; childIndex < values.size(); ++childIndex)
            {
                children << (childIndex ? "," : "") << values[childIndex];
            }
            provenanceLayers.push_back(std::format("\"{}\"", LayerName(layer)));
            rootNodes.push_back(nodes.size());
            nodes.push_back(
                std::format("{{\"name\":\"{}\",\"children\":[{}],\"extras\":{{\"layer\":\"{}\",\"objectCount\":{}}}}}",
                            LayerName(layer), children.str(), LayerName(layer), values.size()));
        }
        std::ostringstream sceneNodes;
        for (std::size_t i{}; i < rootNodes.size(); ++i)
        {
            sceneNodes << (i ? "," : "") << rootNodes[i];
        }
        std::string json =
            std::format("{{\"asset\":{{\"version\":\"2.0\",\"generator\":\"navmesh-generator\"}},\"scene\":0,"
                        "\"scenes\":[{{\"name\":\"Skyrim navmesh "
                        "inspection\",\"nodes\":[{}]}}],\"nodes\":[{}],\"meshes\":[{}],\"materials\":[{}],\"buffers\":["
                        "{{\"byteLength\":{}}}],\"bufferViews\":[{}],\"accessors\":[{}]}}",
                        sceneNodes.str(), join(nodes), join(meshes), join(materials), binary.size(), join(bufferViews),
                        join(accessors));
        while (json.size() % 4 != 0)
        {
            json.push_back(' ');
        }
        Align(binary);
        std::ofstream glb(outputPath, std::ios::binary | std::ios::trunc);
        if (!glb)
        {
            return result;
        }
        const std::uint32_t magic = 0x46546C67, version = 2,
                            length = static_cast<std::uint32_t>(12 + 8 + json.size() + 8 + binary.size()),
                            jsonLength = static_cast<std::uint32_t>(json.size()), jsonType = 0x4E4F534A,
                            binaryLength = static_cast<std::uint32_t>(binary.size()), binaryType = 0x004E4942;
        glb.write(reinterpret_cast<const char *>(&magic), sizeof(magic));
        glb.write(reinterpret_cast<const char *>(&version), sizeof(version));
        glb.write(reinterpret_cast<const char *>(&length), sizeof(length));
        glb.write(reinterpret_cast<const char *>(&jsonLength), sizeof(jsonLength));
        glb.write(reinterpret_cast<const char *>(&jsonType), sizeof(jsonType));
        glb.write(json.data(), static_cast<std::streamsize>(json.size()));
        glb.write(reinterpret_cast<const char *>(&binaryLength), sizeof(binaryLength));
        glb.write(reinterpret_cast<const char *>(&binaryType), sizeof(binaryType));
        glb.write(reinterpret_cast<const char *>(binary.data()), static_cast<std::streamsize>(binary.size()));
        std::ofstream provenance(outputPath.string() + ".provenance.json", std::ios::trunc);
        if (provenance)
        {
            provenance << "{\n  \"metadata\": " << reproducibility::ToJson(metadata, "    ") << ",\n  \"layers\": ["
                       << join(provenanceLayers)
                       << "],\n  \"selection\": {\"culledTriangles\": " << result.culledTriangles << ", \"detail\": \""
                       << (options.detailedProvenance ? "full" : "summary") << "\"},\n  \"objects\": ["
                       << join(provenanceObjects) << "]\n}\n";
        }
        glb.close();
        provenance.close();
        result.written = glb.good() && provenance.good();
        return result;
    }
} // namespace navmesh::core
