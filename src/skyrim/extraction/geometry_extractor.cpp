#include "skyrim/extraction/geometry_extractor.h"

#include <NifFile.hpp>
#include <bhk.hpp>

#include <cmath>
#include <fstream>
#include <format>
#include <cstdlib>
#include <functional>
#include <unordered_set>

namespace
{
    [[nodiscard]] std::string EscapeJson(std::string value)
    {
        std::string escaped;
        for (const auto character : value) {
            if (character == '\\') escaped += "\\\\";
            else if (character == '"') escaped += "\\\"";
            else if (static_cast<unsigned char>(character) < 0x20 || static_cast<unsigned char>(character) >= 0x80) escaped += std::format("\\u00{:02X}", static_cast<unsigned char>(character));
            else escaped += character;
        }
        return escaped;
    }
    [[nodiscard]] std::string JsonStringArray(const std::vector<std::string>& values)
    {
        std::string result = "[";
        for (std::size_t index = 0; index < values.size(); ++index) {
            if (index != 0) result += ",";
            result += "\"" + EscapeJson(values[index]) + "\"";
        }
        result += "]";
        return result;
    }
    [[nodiscard]] const char* CoverageName(const navmesh::core::GeometryCoverage coverage)
    {
        switch (coverage) {
        case navmesh::core::GeometryCoverage::Found: return "found";
        case navmesh::core::GeometryCoverage::Excluded: return "excluded";
        case navmesh::core::GeometryCoverage::Missing: return "missing";
        case navmesh::core::GeometryCoverage::Unreadable: return "unreadable";
        case navmesh::core::GeometryCoverage::Unsupported: return "unsupported";
        }
        return "unknown";
    }
    [[nodiscard]] const char* SourceTypeName(const navmesh::core::GeometrySourceType sourceType)
    {
        switch (sourceType) {
        case navmesh::core::GeometrySourceType::Terrain: return "terrain";
        case navmesh::core::GeometrySourceType::Collision: return "collision";
        case navmesh::core::GeometrySourceType::RenderFallback: return "render_fallback";
        }
        return "unknown";
    }
    [[nodiscard]] bool IsFiniteVec3(const navmesh::core::Vec3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z) && std::abs(value.x) < 1.0e7F && std::abs(value.y) < 1.0e7F && std::abs(value.z) < 1.0e7F;
    }
    struct TriangleGeometry {
        std::vector<navmesh::core::Vec3> vertices;
        std::vector<navmesh::core::Triangle> triangles;
        std::size_t invalidIndices{};
        std::size_t degenerateTriangles{};
        std::vector<std::string> shapes;
    };
    struct NifGeometry {
        TriangleGeometry render;
        TriangleGeometry collision;
        bool nonSolidCollision{};
        std::string version;
    };

    [[nodiscard]] navmesh::core::Vec3 ApplyRigidBodyTransform(const nifly::bhkRigidBody& body, const nifly::Vector3& vertex)
    {
        // Havok stores rigid-body rotation as xyzw quaternion and translation
        // separately.  Apply it before the reference/NIF transforms below.
        const auto& q = body.rotation; const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
        const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z, wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
        return { (1 - 2 * (yy + zz)) * vertex.x + 2 * (xy - wz) * vertex.y + 2 * (xz + wy) * vertex.z + body.translation.x,
                 2 * (xy + wz) * vertex.x + (1 - 2 * (xx + zz)) * vertex.y + 2 * (yz - wx) * vertex.z + body.translation.y,
                 2 * (xz - wy) * vertex.x + 2 * (yz + wx) * vertex.y + (1 - 2 * (xx + yy)) * vertex.z + body.translation.z };
    }
    void AddPackedTriangles(const nifly::hkPackedNiTriStripsData& data, const nifly::bhkRigidBody& body, TriangleGeometry& result)
    {
        const auto base = static_cast<std::uint32_t>(result.vertices.size());
        result.vertices.reserve(result.vertices.size() + data.compressedVertData.size());
        for (const auto& vertex : data.compressedVertData) result.vertices.push_back(ApplyRigidBodyTransform(body, vertex));
        const auto& triangles = data.triData;
        for (const auto& triangle : triangles) {
            if (triangle.tri.p1 >= data.compressedVertData.size() || triangle.tri.p2 >= data.compressedVertData.size() || triangle.tri.p3 >= data.compressedVertData.size()) { ++result.invalidIndices; continue; }
            if (triangle.tri.p1 == triangle.tri.p2 || triangle.tri.p1 == triangle.tri.p3 || triangle.tri.p2 == triangle.tri.p3) { ++result.degenerateTriangles; continue; }
            result.triangles.push_back({ { base + triangle.tri.p1, base + triangle.tri.p2, base + triangle.tri.p3 } });
        }
    }

    // SE static-world collision is normally bhkMoppBvTreeShape ->
    // bhkPackedNiTriStripsShape -> hkPackedNiTriStripsData.  Nifly retains the
    // packed vertices/triangles losslessly, so support that compact path first.
    // Other Havok primitives are intentionally reported as unsupported rather
    // than approximated with decorative render geometry.
    void LoadPackedCollision(const nifly::NifFile& nif, TriangleGeometry& result, bool& nonSolidCollision)
    {
        const auto& header = nif.GetHeader();
        std::unordered_set<std::uint32_t> reachablePacked;
        std::function<void(nifly::bhkShape*, const nifly::bhkRigidBody&)> visit = [&](nifly::bhkShape* shape, const nifly::bhkRigidBody& body) {
            if (!shape) return;
            if (auto* mopp = dynamic_cast<nifly::bhkMoppBvTreeShape*>(shape)) { visit(header.GetBlock(mopp->shapeRef), body); return; }
            if (auto* list = dynamic_cast<nifly::bhkListShape*>(shape)) { for (const auto& child : list->subShapeRefs) visit(header.GetBlock(child), body); return; }
            if (auto* packed = dynamic_cast<nifly::bhkPackedNiTriStripsShape*>(shape)) {
                const auto id = header.GetBlockID(packed);
                if (reachablePacked.insert(id).second) if (const auto* data = header.GetBlock(packed->dataRef)) AddPackedTriangles(*data, body, result);
            }
        };
        for (std::uint32_t block = 0; block < header.GetNumBlocks(); ++block) {
            const auto* collision = header.GetBlock<nifly::bhkNiCollisionObject>(block);
            if (!collision) continue;
            const auto* body = dynamic_cast<nifly::bhkRigidBody*>(header.GetBlock<nifly::NiObject>(collision->bodyRef));
            if (body && body->collisionResponse == nifly::RESPONSE_NONE) { nonSolidCollision = true; continue; }
            if (body) visit(header.GetBlock(body->shapeRef), *body);
        }
        if (!result.triangles.empty()) result.shapes.push_back("hkPackedNiTriStripsData");
    }

    [[nodiscard]] NifGeometry LoadNif(const std::filesystem::path& path)
    {
        NifGeometry result;
        nifly::NifFile nif;
        if (nif.Load(path) != 0 || !nif.IsValid()) return result;
        result.version = nif.GetHeader().GetVersion().String();
        LoadPackedCollision(nif, result.collision, result.nonSolidCollision);
        for (auto* shape : nif.GetShapes()) {
            std::vector<nifly::Vector3> vertices;
            std::vector<nifly::Triangle> triangles;
            if (!nif.GetVertsForShape(shape, vertices) || !shape->GetTriangles(triangles)) continue;
            nifly::MatTransform nodeTransform;
            nodeTransform.Clear();
            if (const auto* parentNode = nif.GetParentNode(shape); parentNode != nullptr) nif.GetNodeTransformToGlobal(parentNode->name.get(), nodeTransform);
            const auto base = static_cast<std::uint32_t>(result.render.vertices.size());
            result.render.shapes.push_back(shape->name.get());
            result.render.vertices.reserve(result.render.vertices.size() + vertices.size());
            for (const auto& vertex : vertices) {
                const auto transformed = nodeTransform.ApplyTransform(vertex);
                result.render.vertices.push_back({ transformed.x, transformed.y, transformed.z });
            }
            for (const auto& triangle : triangles) {
                if (triangle.p1 >= vertices.size() || triangle.p2 >= vertices.size() || triangle.p3 >= vertices.size()) {
                    ++result.render.invalidIndices;
                    continue;
                }
                if (triangle.p1 == triangle.p2 || triangle.p1 == triangle.p3 || triangle.p2 == triangle.p3) {
                    ++result.render.degenerateTriangles;
                    continue;
                }
                result.render.triangles.push_back({ { base + triangle.p1, base + triangle.p2, base + triangle.p3 } });
            }
        }
        return result;
    }

    [[nodiscard]] std::string QuoteShell(const std::filesystem::path& path)
    {
        return "\"" + path.string() + "\"";
    }

    [[nodiscard]] std::filesystem::path ModelRelativePath(const std::string& modelPath)
    {
        auto normalized = modelPath;
        for (auto& character : normalized) if (character == '\\') character = '/';
        auto relativePath = std::filesystem::path(normalized);
        const auto first = relativePath.begin();
        if (first == relativePath.end() || first->string() != "meshes") relativePath = std::filesystem::path("meshes") / relativePath;
        return relativePath;
    }

    [[nodiscard]] bool IsVisualEffectModel(const std::string& modelPath)
    {
        auto normalized = modelPath;
        for (auto& character : normalized) {
            character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
            if (character == '/') character = '\\';
        }
        return normalized.rfind("effects\\", 0) == 0;
    }
    [[nodiscard]] bool IsFilteredReference(const navmesh::core::Reference& reference)
    {
        // These classes are not stable solid scene support.  The policy is
        // intentionally conservative until their collision semantics are
        // modelled explicitly.
        if (reference.recordType == "ACHR" || reference.baseRecordType == "FURN") return true;
        auto path = reference.modelPath;
        std::transform(path.begin(), path.end(), path.begin(), [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return path.find("\\effects\\") != std::string::npos || path.find("\\animated\\") != std::string::npos || path.find("\\fx\\") != std::string::npos;
    }

    void ExtractBsaModels(const std::filesystem::path& dataDirectory, const std::filesystem::path& cacheDirectory, const navmesh::core::Cell& cell)
    {
        if (cacheDirectory.empty()) return;
        std::filesystem::create_directories(cacheDirectory);
        const auto manifest = cacheDirectory / "requested_models.txt";
        std::ofstream manifestStream(manifest, std::ios::trunc);
        if (!manifestStream) return;
        std::size_t modelCount = 0;
        for (const auto& reference : cell.references) {
            if (!reference.modelPath.empty()) { manifestStream << ModelRelativePath(reference.modelPath).string() << "\n"; ++modelCount; }
        }
        manifestStream.close();
        if (modelCount == 0) return;
        const auto script = std::filesystem::current_path() / "tools" / "extract_bsa_models.py";
        if (!std::filesystem::exists(script)) return;
        const auto command = "python " + QuoteShell(script) + " --data " + QuoteShell(dataDirectory) + " --output " + QuoteShell(cacheDirectory) + " --manifest " + QuoteShell(manifest);
        std::system(command.c_str());
    }
}

namespace navmesh::skyrim::offline
{
    GeometryExtraction ExtractGeometry(const std::filesystem::path& dataDirectory, const core::Cell& cell, const std::filesystem::path& cacheDirectory)
    {
        GeometryExtraction output;
        ExtractBsaModels(dataDirectory, cacheDirectory, cell);
        for (const auto& reference : cell.references) {
            GeometryReferenceReport report{ .formId = reference.id, .baseFormId = reference.baseObjectId, .recordType = reference.recordType, .modelPath = reference.modelPath, .position = reference.position, .rotation = reference.rotation, .scale = reference.scale };
            core::GeometrySource source{ .modelPath = reference.modelPath, .reference = { reference.sourcePlugin, reference.id, reference.recordType }, .baseObject = { reference.basePlugin, reference.baseObjectId, reference.baseRecordType } };
            if (reference.modelPath.empty()) { report.failure = "reference has no model path"; output.scene.coverage.push_back({ core::GeometryCoverage::Missing, std::move(source), report.failure }); output.references.push_back(std::move(report)); continue; }
            if (IsFilteredReference(reference)) { report.failure = "excluded by navigation policy (effect, furniture, animated, or actor reference)"; output.scene.coverage.push_back({ core::GeometryCoverage::Excluded, std::move(source), report.failure }); ++output.modelsExcluded; output.references.push_back(std::move(report)); continue; }
            ++output.referencesWithModels; const auto relativePath = ModelRelativePath(reference.modelPath);
            const auto loosePath = dataDirectory / relativePath;
            const auto cachedPath = cacheDirectory.empty() ? loosePath : cacheDirectory / relativePath;
            const auto modelPath = std::filesystem::exists(loosePath) ? loosePath : cachedPath;
            const auto nifGeometry = LoadNif(modelPath);
            const auto& mesh = !nifGeometry.collision.triangles.empty() ? nifGeometry.collision : nifGeometry.render;
            const bool hasCollision = !nifGeometry.collision.triangles.empty();
            if (!hasCollision && nifGeometry.nonSolidCollision) { report.failure = "excluded non-solid Havok collision; render fallback is forbidden"; output.scene.coverage.push_back({ core::GeometryCoverage::Excluded, std::move(source), report.failure }); ++output.modelsExcluded; output.references.push_back(std::move(report)); continue; }
            if (nifGeometry.version.empty() || mesh.vertices.empty() || mesh.triangles.empty()) {
                const auto exists = std::filesystem::exists(modelPath);
                const auto status = !exists ? core::GeometryCoverage::Missing : nifGeometry.version.empty() ? core::GeometryCoverage::Unreadable : core::GeometryCoverage::Unsupported;
                report.failure = status == core::GeometryCoverage::Missing ? "missing loose NIF" : status == core::GeometryCoverage::Unreadable ? "NIF could not be read" : "NIF contains no supported triangle geometry";
                if (status == core::GeometryCoverage::Unsupported) ++output.modelsUnsupported;
                else if (status == core::GeometryCoverage::Unreadable) ++output.modelsUnreadable;
                else ++output.modelsMissing;
                output.scene.coverage.push_back({ status, std::move(source), report.failure }); output.references.push_back(std::move(report)); continue;
            }
            if (IsVisualEffectModel(reference.modelPath)) {
                report.failure = "excluded visual effect from support geometry";
                report.vertices = mesh.vertices.size();
                report.triangles = mesh.triangles.size();
                ++output.modelsLoaded;
                output.scene.coverage.push_back({ core::GeometryCoverage::Excluded, std::move(source), report.failure }); ++output.modelsExcluded;
                output.references.push_back(std::move(report));
                continue;
            }
            source.sourceType = hasCollision ? core::GeometrySourceType::Collision : core::GeometrySourceType::RenderFallback;
            source.materialClass = hasCollision ? core::MaterialCollisionClass::HavokPackedTriangles : core::MaterialCollisionClass::RenderVisual;
            source.collisionType = hasCollision ? "hkPackedNiTriStripsData" : "render mesh (no supported collision)";
            source.confidence = hasCollision ? 1.0F : 0.35F;
            const auto sourceIndex = output.scene.geometrySources.size();
            output.scene.geometrySources.push_back(source);
            output.scene.nodes.push_back({ .name = reference.editorId.empty() ? reference.modelPath : reference.editorId, .localTransform = core::Transform::FromEulerXYZ(reference.position, reference.rotation, reference.scale), .geometrySource = sourceIndex });
            const auto base = static_cast<std::uint32_t>(output.mesh.vertices.size());
            report.meshVertexOffset = base;
            report.meshTriangleOffset = output.mesh.triangles.size();
            const auto transform = output.scene.nodes.back().localTransform;
            for (const auto& vertex : mesh.vertices) { const auto transformed = transform.ApplyPoint(vertex); if (!std::isfinite(transformed.x) || !std::isfinite(transformed.y) || !std::isfinite(transformed.z) || std::abs(transformed.x) > 1.0e7F || std::abs(transformed.y) > 1.0e7F || std::abs(transformed.z) > 1.0e7F) ++output.invalidVertices; output.mesh.vertices.push_back(transformed); }
            for (std::size_t triangleIndex = 0; triangleIndex < mesh.triangles.size(); ++triangleIndex) { const auto& triangle = mesh.triangles[triangleIndex]; output.mesh.triangles.push_back({ { base + triangle.vertices[0], base + triangle.vertices[1], base + triangle.vertices[2] } }); output.scene.triangleProvenance.push_back({ sourceIndex, triangleIndex }); }
            output.invalidIndices += mesh.invalidIndices;
            report.vertices = mesh.vertices.size(); report.triangles = mesh.triangles.size(); report.invalidIndices = mesh.invalidIndices; report.degenerateTriangles = mesh.degenerateTriangles; report.shapes = mesh.shapes; report.nifVersion = nifGeometry.version; report.sourceType = SourceTypeName(source.sourceType); report.collisionType = source.collisionType; report.usedRenderFallback = !hasCollision; ++output.modelsLoaded; output.references.push_back(std::move(report));
            if (hasCollision) { ++output.collisionModelsLoaded; output.collisionTriangles += mesh.triangles.size(); }
            else { ++output.renderFallbackModels; output.renderFallbackTriangles += mesh.triangles.size(); }
            output.scene.coverage.push_back({ core::GeometryCoverage::Found, std::move(source), hasCollision ? "packed Havok collision loaded" : "render NIF loaded as low-confidence fallback; no supported collision" });
        }
        output.scene.mesh = output.mesh;
        return output;
    }
    bool WriteGeometryObj(const std::filesystem::path& outputPath, const GeometryExtraction& geometry)
    {
        std::ofstream output(outputPath, std::ios::trunc);
        if (!output) return false;
        for (const auto& vertex : geometry.mesh.vertices) output << std::format("v {} {} {}\n", vertex.x, vertex.y, vertex.z);
        for (const auto& reference : geometry.references) {
            if (reference.triangles == 0) continue;
            output << std::format("g REF_{:08X}_BASE_{:08X}\n", reference.formId, reference.baseFormId);
            for (std::size_t index = reference.meshTriangleOffset; index < reference.meshTriangleOffset + reference.triangles; ++index) {
                const auto& triangle = geometry.mesh.triangles[index];
                output << std::format("f {} {} {}\n", triangle.vertices[0] + 1, triangle.vertices[1] + 1, triangle.vertices[2] + 1);
            }
        }
        return true;
    }

    bool WriteGeometryJson(const std::filesystem::path& outputPath, const core::Cell& cell, const GeometryExtraction& geometry, const reproducibility::ExportMetadata& metadata)
    {
        std::ofstream output(outputPath, std::ios::trunc); if (!output) return false;
        output << "{\n  \"metadata\": " << reproducibility::ToJson(metadata, "    ") << ",\n";
        output << std::format("  \"cell\": \"{:08X}\",\n  \"references\": {},\n  \"referencesWithModels\": {},\n  \"modelsLoaded\": {},\n  \"modelsMissing\": {},\n  \"modelsExcluded\": {},\n  \"modelsUnreadable\": {},\n  \"modelsUnsupported\": {},\n  \"collisionModelsLoaded\": {},\n  \"collisionTriangles\": {},\n  \"renderFallbackModels\": {},\n  \"renderFallbackTriangles\": {},\n  \"vertices\": {},\n  \"triangles\": {},\n  \"terrainSupported\": {},\n  \"collisionGeometrySupported\": {},\n  \"invalidVertices\": {},\n  \"invalidIndices\": {},\n  \"referenceDetails\": [\n", cell.id, cell.references.size(), geometry.referencesWithModels, geometry.modelsLoaded, geometry.modelsMissing, geometry.modelsExcluded, geometry.modelsUnreadable, geometry.modelsUnsupported, geometry.collisionModelsLoaded, geometry.collisionTriangles, geometry.renderFallbackModels, geometry.renderFallbackTriangles, geometry.mesh.vertices.size(), geometry.mesh.triangles.size(), geometry.terrainSupported ? "true" : "false", geometry.collisionGeometrySupported ? "true" : "false", geometry.invalidVertices, geometry.invalidIndices);
        for (std::size_t index = 0; index < geometry.references.size(); ++index) {
            const auto& reference = geometry.references[index];
            output << std::format("    {{\"formId\":\"{:08X}\",\"baseFormId\":\"{:08X}\",\"recordType\":\"{}\",\"model\":\"{}\",\"nifVersion\":\"{}\",\"sourceType\":\"{}\",\"collisionType\":\"{}\",\"usedRenderFallback\":{},\"shapes\":{},\"position\":[{},{},{}],\"rotation\":[{},{},{}],\"scale\":{},\"vertices\":{},\"triangles\":{},\"invalidIndices\":{},\"degenerateTriangles\":{},\"failure\":\"{}\"}}{}\n", reference.formId, reference.baseFormId, EscapeJson(reference.recordType), EscapeJson(reference.modelPath), EscapeJson(reference.nifVersion), EscapeJson(reference.sourceType), EscapeJson(reference.collisionType), reference.usedRenderFallback ? "true" : "false", JsonStringArray(reference.shapes), reference.position.x, reference.position.y, reference.position.z, reference.rotation.x, reference.rotation.y, reference.rotation.z, reference.scale, reference.vertices, reference.triangles, reference.invalidIndices, reference.degenerateTriangles, EscapeJson(reference.failure), index + 1 == geometry.references.size() ? "" : ",");
        }
        output << "  ],\n  \"coverage\": [\n";
        for (std::size_t index = 0; index < geometry.scene.coverage.size(); ++index) {
            const auto& entry = geometry.scene.coverage[index]; const auto& source = entry.source;
            output << std::format("    {{\"status\":\"{}\",\"detail\":\"{}\",\"model\":\"{}\",\"reference\":{{\"plugin\":\"{}\",\"formId\":\"{:08X}\",\"recordType\":\"{}\"}},\"baseObject\":{{\"plugin\":\"{}\",\"formId\":\"{:08X}\",\"recordType\":\"{}\"}}}}{}\n", CoverageName(entry.status), EscapeJson(entry.detail), EscapeJson(source.modelPath), EscapeJson(source.reference.plugin), source.reference.formId, EscapeJson(source.reference.recordType), EscapeJson(source.baseObject.plugin), source.baseObject.formId, EscapeJson(source.baseObject.recordType), index + 1 == geometry.scene.coverage.size() ? "" : ",");
        }
        output << "  ],\n  \"triangleProvenance\": [\n";
        for (std::size_t index = 0; index < geometry.scene.triangleProvenance.size(); ++index) {
            const auto& provenance = geometry.scene.triangleProvenance[index]; const auto& source = geometry.scene.geometrySources[provenance.geometrySource];
            const auto terrain = provenance.terrain ? std::format(",\"terrain\":{{\"cell\":[{},{}],\"landFormId\":\"{:08X}\",\"sample\":[{},{}]}}", provenance.terrain->cellX, provenance.terrain->cellY, provenance.terrain->landFormId, provenance.terrain->sampleX, provenance.terrain->sampleY) : "";
            output << std::format("    {{\"triangle\":{},\"sourceTriangle\":{},\"sourceType\":\"{}\",\"collisionType\":\"{}\",\"confidence\":{},\"model\":\"{}\",\"reference\":{{\"plugin\":\"{}\",\"formId\":\"{:08X}\",\"recordType\":\"{}\"}},\"baseObject\":{{\"plugin\":\"{}\",\"formId\":\"{:08X}\",\"recordType\":\"{}\"}}{}}}{}\n", index, provenance.sourceTriangle, SourceTypeName(source.sourceType), EscapeJson(source.collisionType), source.confidence, EscapeJson(source.modelPath), EscapeJson(source.reference.plugin), source.reference.formId, EscapeJson(source.reference.recordType), EscapeJson(source.baseObject.plugin), source.baseObject.formId, EscapeJson(source.baseObject.recordType), terrain, index + 1 == geometry.scene.triangleProvenance.size() ? "" : ",");
        }
        output << "  ]\n}\n"; return true;
    }
}
