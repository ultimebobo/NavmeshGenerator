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
#include <unordered_map>

namespace
{
    using namespace navmesh::core;
    struct Object
    {
        SceneLayer layer{};
        std::string name;
        std::string provenance;
        std::string classification;
        std::vector<Vec3> vertices;
        std::vector<std::array<std::uint32_t, 3>> triangles;
    };
    [[nodiscard]] bool Contains(const std::vector<SceneLayer>& layers, SceneLayer layer) { return std::find(layers.begin(), layers.end(), layer) != layers.end(); }
    [[nodiscard]] const char* LayerName(SceneLayer layer)
    {
        switch (layer) {
        case SceneLayer::ExistingNavmesh: return "Existing NAVM";
        case SceneLayer::Terrain: return "Terrain";
        case SceneLayer::Collision: return "Collision";
        case SceneLayer::RenderFallback: return "Render fallback";
        case SceneLayer::DiagnosticMarkers: return "Diagnostic markers";
        }
        return "Unknown";
    }
    [[nodiscard]] std::size_t MaterialIndex(SceneLayer layer, const std::string& classification = {})
    {
        if (layer == SceneLayer::Terrain) return 1; // brown/green
        if (layer == SceneLayer::Collision) return 2; // gray
        if (layer == SceneLayer::RenderFallback) return 3; // purple
        if (classification.empty()) return 0; // unclassified NAVM
        if (classification == "supported") return 4;
        if (classification == "floating") return 5;
        if (classification == "buried") return 6;
        if (classification == "too_steep") return 7;
        if (classification == "blocked") return 8;
        if (classification == "out_of_coverage") return 9;
        if (classification == "ambiguous") return 10;
        return 11; // unsupported or unknown
    }
    [[nodiscard]] std::string Escape(const std::string& value)
    {
        std::string escaped;
        for (const char c : value) { if (c == '\\') escaped += "\\\\"; else if (c == '\"') escaped += "\\\""; else if (c == '\n') escaped += "\\n"; else if (c == '\r') escaped += "\\r"; else escaped += c; }
        return escaped;
    }
    [[nodiscard]] AABB TriangleBounds(const Mesh& mesh, const Triangle& triangle)
    {
        AABB result;
        for (const auto index : triangle.vertices) if (index < mesh.vertices.size()) result.Expand(mesh.vertices[index]);
        return result;
    }
    // A coarse grid makes bounds selection proportional to the local selection,
    // not to every triangle. Source triangles remain streamed into GLB objects.
    class TriangleGrid
    {
    public:
        explicit TriangleGrid(const Mesh& mesh) : mesh_(mesh)
        {
            for (std::size_t index = 0; index < mesh.triangles.size(); ++index) {
                const auto box = TriangleBounds(mesh, mesh.triangles[index]);
                if (!box.IsValid()) continue;
                const auto minX = Cell(box.min.x), maxX = Cell(box.max.x), minY = Cell(box.min.y), maxY = Cell(box.max.y);
                for (int x = minX; x <= maxX; ++x) for (int y = minY; y <= maxY; ++y) cells_[{ x, y }].push_back(index);
            }
        }
        [[nodiscard]] std::vector<std::size_t> Query(const std::optional<SceneBounds>& bounds) const
        {
            if (!bounds) { std::vector<std::size_t> all(mesh_.triangles.size()); for (std::size_t i{}; i < all.size(); ++i) all[i] = i; return all; }
            std::set<std::size_t> selected;
            const auto& box = bounds->world;
            for (int x = Cell(box.min.x); x <= Cell(box.max.x); ++x) for (int y = Cell(box.min.y); y <= Cell(box.max.y); ++y) {
                const auto found = cells_.find({ x, y }); if (found == cells_.end()) continue;
                for (const auto index : found->second) if (TriangleBounds(mesh_, mesh_.triangles[index]).Intersects(box)) selected.insert(index);
            }
            return { selected.begin(), selected.end() };
        }
    private:
        [[nodiscard]] static int Cell(float value) { return static_cast<int>(std::floor(value / 4096.0F)); }
        struct Key { int x{}, y{}; [[nodiscard]] bool operator<(const Key& other) const noexcept { return x != other.x ? x < other.x : y < other.y; } };
        const Mesh& mesh_; std::map<Key, std::vector<std::size_t>> cells_;
    };
    void AppendMarker(Object& object, const DiagnosticMarker& marker)
    {
        // Small world-space pyramid, readable in viewers that hide POINTS.
        constexpr float r = 12.0F, h = 24.0F;
        const auto base = static_cast<std::uint32_t>(object.vertices.size());
        object.vertices.insert(object.vertices.end(), { { marker.position.x-r, marker.position.y-r, marker.position.z }, { marker.position.x+r, marker.position.y-r, marker.position.z }, { marker.position.x+r, marker.position.y+r, marker.position.z }, { marker.position.x-r, marker.position.y+r, marker.position.z }, { marker.position.x, marker.position.y, marker.position.z+h } });
        object.triangles.insert(object.triangles.end(), { { base, base+1, base+4 }, { base+1, base+2, base+4 }, { base+2, base+3, base+4 }, { base+3, base, base+4 } });
    }
    template <typename T> void Append(std::vector<std::uint8_t>& bytes, const T& value)
    {
        const auto start = bytes.size(); bytes.resize(start + sizeof(T)); std::memcpy(bytes.data() + start, &value, sizeof(T));
    }
    void Align(std::vector<std::uint8_t>& bytes) { while (bytes.size() % 4 != 0) bytes.push_back(0); }
}

namespace navmesh::core
{
    SceneExportResult WriteCombinedGlb(const std::filesystem::path& outputPath, const Scene& scene, const std::vector<NavMesh>& navmeshes, const std::vector<DiagnosticMarker>& markers, const reproducibility::ExportMetadata& metadata, const SceneExportOptions& options)
    {
        SceneExportResult result;
        std::vector<Object> objects;
        std::map<std::pair<SceneLayer, std::size_t>, std::size_t> objectForSource;
        const auto appendMesh = [&](const Mesh& mesh, const std::vector<TriangleProvenance>& triangleProvenance) {
            const TriangleGrid index(mesh);
            const auto selected = index.Query(options.bounds);
            result.culledTriangles += mesh.triangles.size() - selected.size();
            for (const auto triangleIndex : selected) {
                if (triangleIndex >= triangleProvenance.size()) continue;
                const auto& provenance = triangleProvenance[triangleIndex];
                if (provenance.geometrySource >= scene.geometrySources.size()) continue;
                const auto& source = scene.geometrySources[provenance.geometrySource];
                const auto layer = source.sourceType == GeometrySourceType::Terrain ? SceneLayer::Terrain : source.sourceType == GeometrySourceType::Collision ? SceneLayer::Collision : SceneLayer::RenderFallback;
                if (!Contains(options.layers, layer)) continue;
                const auto key = std::pair{ layer, provenance.geometrySource };
                auto [where, added] = objectForSource.emplace(key, objects.size());
                if (added) {
                    const auto name = std::format("{}: {:08X} {}", LayerName(layer), source.reference.formId, source.modelPath.empty() ? source.reference.recordType : source.modelPath);
                    const auto provenanceJson = std::format("{{\"plugin\":\"{}\",\"formId\":\"{:08X}\",\"recordType\":\"{}\",\"model\":\"{}\",\"sourceType\":\"{}\",\"confidence\":{}}}", Escape(source.reference.plugin), source.reference.formId, Escape(source.reference.recordType), Escape(source.modelPath), LayerName(layer), source.confidence);
                    objects.push_back({ layer, name, provenanceJson });
                }
                auto& object = objects[where->second]; const auto& triangle = mesh.triangles[triangleIndex];
                const auto base = static_cast<std::uint32_t>(object.vertices.size());
                bool valid = true; for (const auto vertex : triangle.vertices) if (vertex >= mesh.vertices.size()) valid = false;
                if (!valid) continue;
                for (const auto vertex : triangle.vertices) object.vertices.push_back(mesh.vertices[vertex]);
                object.triangles.push_back({ base, base + 1, base + 2 });
            }
        };
        appendMesh(scene.mesh, scene.triangleProvenance);
        appendMesh(scene.renderFallbackMesh, scene.renderFallbackTriangleProvenance);
        std::map<std::pair<std::uint32_t, std::size_t>, std::string> polygonClassifications;
        for (const auto& marker : markers) if (marker.navmeshFormId) polygonClassifications.try_emplace(std::pair{ *marker.navmeshFormId, marker.navmeshPolygon }, marker.classification);
        if (Contains(options.layers, SceneLayer::ExistingNavmesh)) for (const auto& navmesh : navmeshes) {
            std::map<std::string, Object> classifiedObjects;
            for (std::size_t polygonIndex{}; polygonIndex < navmesh.polygons.size(); ++polygonIndex) {
                const auto& polygon = navmesh.polygons[polygonIndex];
                if (polygon.vertices[0] >= navmesh.vertices.size() || polygon.vertices[1] >= navmesh.vertices.size() || polygon.vertices[2] >= navmesh.vertices.size()) continue;
                AABB polygonBounds; for (const auto vertex : polygon.vertices) polygonBounds.Expand(navmesh.vertices[vertex]);
                if (options.bounds && !polygonBounds.Intersects(options.bounds->world)) continue;
                const auto found = polygonClassifications.find({ navmesh.id, polygonIndex });
                const std::string classification = found == polygonClassifications.end() ? "" : found->second;
                auto [it, inserted] = classifiedObjects.try_emplace(classification);
                auto& object = it->second;
                if (inserted) {
                    object.layer = SceneLayer::ExistingNavmesh;
                    object.classification = classification;
                    object.name = std::format("Existing NAVM {:08X}: {}", navmesh.id, classification.empty() ? "unclassified" : classification);
                    object.provenance = std::format("{{\"navmeshFormId\":\"{:08X}\",\"classification\":\"{}\"}}", navmesh.id, Escape(classification.empty() ? "unclassified" : classification));
                }
                const auto base = static_cast<std::uint32_t>(object.vertices.size()); for (const auto vertex : polygon.vertices) object.vertices.push_back(navmesh.vertices[vertex]); object.triangles.push_back({ base, base+1, base+2 });
            }
            for (auto& [_, object] : classifiedObjects) objects.push_back(std::move(object));
        }
        if (Contains(options.layers, SceneLayer::DiagnosticMarkers)) {
            std::map<std::string, Object> markerObjects;
            for (const auto& marker : markers) {
                if (options.bounds && !options.bounds->world.Contains(marker.position)) continue;
                auto [it, inserted] = markerObjects.try_emplace(marker.classification, SceneLayer::DiagnosticMarkers, std::format("Diagnostic: {}", marker.classification), std::format("{{\"classification\":\"{}\"}}", Escape(marker.classification)), marker.classification);
                AppendMarker(it->second, marker);
            }
            for (auto& [_, object] : markerObjects) objects.push_back(std::move(object));
        }

        std::vector<std::uint8_t> binary; std::vector<std::string> bufferViews, accessors, meshes, nodes, provenanceObjects;
        std::array<std::vector<std::size_t>, 5> layerChildren;
        for (const auto& object : objects) {
            if (object.triangles.empty()) continue;
            Align(binary); const auto positionOffset = binary.size(); AABB bounds;
            for (const auto& vertex : object.vertices) { Append(binary, vertex.x); Append(binary, vertex.y); Append(binary, vertex.z); bounds.Expand(vertex); }
            const auto positionView = bufferViews.size(); bufferViews.push_back(std::format("{{\"buffer\":0,\"byteOffset\":{},\"byteLength\":{},\"target\":34962}}", positionOffset, object.vertices.size() * sizeof(float) * 3));
            const auto positionAccessor = accessors.size(); accessors.push_back(std::format("{{\"bufferView\":{},\"componentType\":5126,\"count\":{},\"type\":\"VEC3\",\"min\":[{},{},{}],\"max\":[{},{},{}]}}", positionView, object.vertices.size(), bounds.min.x, bounds.min.y, bounds.min.z, bounds.max.x, bounds.max.y, bounds.max.z));
            Align(binary); const auto indexOffset = binary.size(); for (const auto& tri : object.triangles) for (const auto value : tri) Append(binary, value);
            const auto indexView = bufferViews.size(); bufferViews.push_back(std::format("{{\"buffer\":0,\"byteOffset\":{},\"byteLength\":{},\"target\":34963}}", indexOffset, object.triangles.size() * 3 * sizeof(std::uint32_t)));
            const auto indexAccessor = accessors.size(); accessors.push_back(std::format("{{\"bufferView\":{},\"componentType\":5125,\"count\":{},\"type\":\"SCALAR\"}}", indexView, object.triangles.size() * 3));
            const auto meshIndex = meshes.size(); meshes.push_back(std::format("{{\"name\":\"{}\",\"primitives\":[{{\"attributes\":{{\"POSITION\":{}}},\"indices\":{},\"material\":{}}}]}}", Escape(object.name), positionAccessor, indexAccessor, MaterialIndex(object.layer, object.classification)));
            layerChildren[static_cast<std::size_t>(object.layer)].push_back(nodes.size());
            nodes.push_back(std::format("{{\"name\":\"{}\",\"mesh\":{},\"extras\":{{\"provenance\":{}}}}}", Escape(object.name), meshIndex, object.provenance));
            if (options.detailedProvenance) provenanceObjects.push_back(std::format("{{\"name\":\"{}\",\"layer\":\"{}\",\"triangles\":{},\"provenance\":{}}}", Escape(object.name), LayerName(object.layer), object.triangles.size(), object.provenance));
            else provenanceObjects.push_back(std::format("{{\"name\":\"{}\",\"layer\":\"{}\",\"triangles\":{}}}", Escape(object.name), LayerName(object.layer), object.triangles.size()));
            result.triangles += object.triangles.size(); ++result.objects;
        }
        const std::array<const char*, 12> materialNames{ "Unclassified NAVM (cyan)", "Terrain (brown-green)", "Collision (gray)", "Render fallback (purple)", "Supported (green)", "Floating (orange)", "Buried (red)", "Too steep (yellow)", "Blocked (magenta)", "Out of coverage (blue)", "Ambiguous (violet)", "Unsupported or unknown (dark gray)" };
        const std::array<std::array<float, 4>, 12> colors{{ {{0.0F,0.85F,0.95F,0.70F}}, {{0.35F,0.48F,0.16F,1.0F}}, {{0.46F,0.46F,0.50F,1.0F}}, {{0.58F,0.25F,0.75F,0.80F}}, {{0.10F,0.70F,0.25F,0.75F}}, {{1.0F,0.50F,0.05F,0.75F}}, {{0.90F,0.10F,0.10F,0.75F}}, {{0.95F,0.82F,0.08F,0.75F}}, {{0.85F,0.05F,0.60F,0.75F}}, {{0.08F,0.35F,0.95F,0.75F}}, {{0.48F,0.20F,0.90F,0.75F}}, {{0.25F,0.28F,0.32F,0.75F}} }};
        std::vector<std::string> materials; for (std::size_t i{}; i < materialNames.size(); ++i) materials.push_back(std::format("{{\"name\":\"{}\",\"doubleSided\":true,\"alphaMode\":\"{}\",\"pbrMetallicRoughness\":{{\"baseColorFactor\":[{},{},{},{}],\"metallicFactor\":0,\"roughnessFactor\":0.82}}}}", materialNames[i], colors[i][3] < 1.0F ? "BLEND" : "OPAQUE", colors[i][0], colors[i][1], colors[i][2], colors[i][3]));
        const auto join = [](const auto& values) { std::ostringstream out; for (std::size_t i{}; i < values.size(); ++i) out << (i ? "," : "") << values[i]; return out.str(); };
        // Keep a stable, visible hierarchy even when a requested layer has no
        // extracted triangles. A viewer can then distinguish unavailable
        // terrain/collision/NAVM from an exporter silently losing that layer.
        std::vector<std::size_t> rootNodes; std::set<SceneLayer> emittedLayers;
        for (const auto layer : options.layers) {
            if (!emittedLayers.insert(layer).second) continue;
            std::ostringstream children; const auto& values = layerChildren[static_cast<std::size_t>(layer)];
            for (std::size_t childIndex{}; childIndex < values.size(); ++childIndex) children << (childIndex ? "," : "") << values[childIndex];
            rootNodes.push_back(nodes.size());
            nodes.push_back(std::format("{{\"name\":\"{}\",\"children\":[{}],\"extras\":{{\"layer\":\"{}\",\"objectCount\":{}}}}}", LayerName(layer), children.str(), LayerName(layer), values.size()));
        }
        std::ostringstream sceneNodes; for (std::size_t i{}; i < rootNodes.size(); ++i) sceneNodes << (i ? "," : "") << rootNodes[i];
        std::string json = std::format("{{\"asset\":{{\"version\":\"2.0\",\"generator\":\"navmesh-generator\"}},\"scene\":0,\"scenes\":[{{\"name\":\"Skyrim navmesh inspection\",\"nodes\":[{}]}}],\"nodes\":[{}],\"meshes\":[{}],\"materials\":[{}],\"buffers\":[{{\"byteLength\":{}}}],\"bufferViews\":[{}],\"accessors\":[{}]}}", sceneNodes.str(), join(nodes), join(meshes), join(materials), binary.size(), join(bufferViews), join(accessors));
        while (json.size() % 4 != 0) json.push_back(' '); Align(binary);
        std::ofstream glb(outputPath, std::ios::binary | std::ios::trunc); if (!glb) return result;
        const std::uint32_t magic = 0x46546C67, version = 2, length = static_cast<std::uint32_t>(12 + 8 + json.size() + 8 + binary.size()), jsonLength = static_cast<std::uint32_t>(json.size()), jsonType = 0x4E4F534A, binaryLength = static_cast<std::uint32_t>(binary.size()), binaryType = 0x004E4942;
        glb.write(reinterpret_cast<const char*>(&magic), sizeof(magic)); glb.write(reinterpret_cast<const char*>(&version), sizeof(version)); glb.write(reinterpret_cast<const char*>(&length), sizeof(length)); glb.write(reinterpret_cast<const char*>(&jsonLength), sizeof(jsonLength)); glb.write(reinterpret_cast<const char*>(&jsonType), sizeof(jsonType)); glb.write(json.data(), static_cast<std::streamsize>(json.size())); glb.write(reinterpret_cast<const char*>(&binaryLength), sizeof(binaryLength)); glb.write(reinterpret_cast<const char*>(&binaryType), sizeof(binaryType)); glb.write(reinterpret_cast<const char*>(binary.data()), static_cast<std::streamsize>(binary.size()));
        std::ofstream provenance(outputPath.string() + ".provenance.json", std::ios::trunc);
        if (provenance) provenance << "{\n  \"metadata\": " << reproducibility::ToJson(metadata, "    ") << ",\n  \"layers\": [\"Existing NAVM\", \"Terrain\", \"Collision\", \"Render fallback\", \"Diagnostic markers\"],\n  \"selection\": {\"culledTriangles\": " << result.culledTriangles << ", \"detail\": \"" << (options.detailedProvenance ? "full" : "summary") << "\"},\n  \"objects\": [" << join(provenanceObjects) << "]\n}\n";
        return result;
    }
}
