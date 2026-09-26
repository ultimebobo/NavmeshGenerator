#include "skyrim/extraction/geometry_extractor.h"

#include <NifFile.hpp>

#include <cmath>
#include <fstream>
#include <format>
#include <cstdlib>

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
    [[nodiscard]] navmesh::core::Vec3 Rotate(const navmesh::core::Vec3& value, const navmesh::core::Vec3& rotation)
    {
        const auto cx = std::cos(rotation.x); const auto sx = std::sin(rotation.x); const auto cy = std::cos(rotation.y); const auto sy = std::sin(rotation.y); const auto cz = std::cos(rotation.z); const auto sz = std::sin(rotation.z);
        const auto x1 = value.x; const auto y1 = value.y * cx - value.z * sx; const auto z1 = value.y * sx + value.z * cx; const auto x2 = x1 * cy + z1 * sy; const auto y2 = y1; const auto z2 = -x1 * sy + z1 * cy;
        return { x2 * cz - y2 * sz, x2 * sz + y2 * cz, z2 };
    }
    [[nodiscard]] bool IsFiniteVec3(const navmesh::core::Vec3& value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z) && std::abs(value.x) < 1.0e7F && std::abs(value.y) < 1.0e7F && std::abs(value.z) < 1.0e7F;
    }
    struct NifGeometry {
        std::vector<navmesh::core::Vec3> vertices;
        std::vector<navmesh::core::Triangle> triangles;
        std::size_t invalidIndices{};
        std::size_t degenerateTriangles{};
        std::vector<std::string> shapes;
        std::string version;
    };

    [[nodiscard]] NifGeometry LoadNif(const std::filesystem::path& path)
    {
        NifGeometry result;
        nifly::NifFile nif;
        if (nif.Load(path) != 0 || !nif.IsValid()) return result;
        result.version = nif.GetHeader().GetVersion().String();
        for (auto* shape : nif.GetShapes()) {
            std::vector<nifly::Vector3> vertices;
            std::vector<nifly::Triangle> triangles;
            if (!nif.GetVertsForShape(shape, vertices) || !shape->GetTriangles(triangles)) continue;
            nifly::MatTransform nodeTransform;
            nodeTransform.Clear();
            if (const auto* parentNode = nif.GetParentNode(shape); parentNode != nullptr) nif.GetNodeTransformToGlobal(parentNode->name.get(), nodeTransform);
            const auto base = static_cast<std::uint32_t>(result.vertices.size());
            result.shapes.push_back(shape->name.get());
            result.vertices.reserve(result.vertices.size() + vertices.size());
            for (const auto& vertex : vertices) {
                const auto transformed = nodeTransform.ApplyTransform(vertex);
                result.vertices.push_back({ transformed.x, transformed.y, transformed.z });
            }
            for (const auto& triangle : triangles) {
                if (triangle.p1 >= vertices.size() || triangle.p2 >= vertices.size() || triangle.p3 >= vertices.size()) {
                    ++result.invalidIndices;
                    continue;
                }
                if (triangle.p1 == triangle.p2 || triangle.p1 == triangle.p3 || triangle.p2 == triangle.p3) {
                    ++result.degenerateTriangles;
                    continue;
                }
                result.triangles.push_back({ { base + triangle.p1, base + triangle.p2, base + triangle.p3 } });
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
            if (reference.modelPath.empty()) { report.failure = "reference has no model path"; output.references.push_back(std::move(report)); continue; }
            ++output.referencesWithModels; const auto relativePath = ModelRelativePath(reference.modelPath);
            const auto loosePath = dataDirectory / relativePath;
            const auto cachedPath = cacheDirectory.empty() ? loosePath : cacheDirectory / relativePath;
            const auto modelPath = std::filesystem::exists(loosePath) ? loosePath : cachedPath;
            const auto mesh = LoadNif(modelPath);
            if (mesh.version.empty() || mesh.vertices.empty() || mesh.triangles.empty()) { report.failure = std::filesystem::exists(modelPath) ? "unsupported or empty NIF" : "missing loose NIF"; ++output.modelsMissing; output.references.push_back(std::move(report)); continue; }
            if (IsVisualEffectModel(reference.modelPath)) {
                report.failure = "excluded visual effect from support geometry";
                report.vertices = mesh.vertices.size();
                report.triangles = mesh.triangles.size();
                ++output.modelsLoaded;
                output.references.push_back(std::move(report));
                continue;
            }
            const auto base = static_cast<std::uint32_t>(output.mesh.vertices.size());
            report.meshVertexOffset = base;
            report.meshTriangleOffset = output.mesh.triangles.size();
            for (const auto& vertex : mesh.vertices) { auto transformed = Rotate({ vertex.x * reference.scale, vertex.y * reference.scale, vertex.z * reference.scale }, reference.rotation); transformed.x += reference.position.x; transformed.y += reference.position.y; transformed.z += reference.position.z; if (!std::isfinite(transformed.x) || !std::isfinite(transformed.y) || !std::isfinite(transformed.z) || std::abs(transformed.x) > 1.0e7F || std::abs(transformed.y) > 1.0e7F || std::abs(transformed.z) > 1.0e7F) ++output.invalidVertices; output.mesh.vertices.push_back(transformed); }
            for (const auto& triangle : mesh.triangles) output.mesh.triangles.push_back({ { base + triangle.vertices[0], base + triangle.vertices[1], base + triangle.vertices[2] } });
            output.invalidIndices += mesh.invalidIndices;
            report.vertices = mesh.vertices.size(); report.triangles = mesh.triangles.size(); report.invalidIndices = mesh.invalidIndices; report.degenerateTriangles = mesh.degenerateTriangles; report.shapes = mesh.shapes; report.nifVersion = mesh.version; ++output.modelsLoaded; output.references.push_back(std::move(report));
        }
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
        output << std::format("  \"cell\": \"{:08X}\",\n  \"references\": {},\n  \"referencesWithModels\": {},\n  \"modelsLoaded\": {},\n  \"modelsMissing\": {},\n  \"vertices\": {},\n  \"triangles\": {},\n  \"terrainSupported\": false,\n  \"collisionGeometrySupported\": false,\n  \"invalidVertices\": {},\n  \"invalidIndices\": {},\n  \"referenceDetails\": [\n", cell.id, cell.references.size(), geometry.referencesWithModels, geometry.modelsLoaded, geometry.modelsMissing, geometry.mesh.vertices.size(), geometry.mesh.triangles.size(), geometry.invalidVertices, geometry.invalidIndices);
        for (std::size_t index = 0; index < geometry.references.size(); ++index) {
            const auto& reference = geometry.references[index];
            output << std::format("    {{\"formId\":\"{:08X}\",\"baseFormId\":\"{:08X}\",\"recordType\":\"{}\",\"model\":\"{}\",\"nifVersion\":\"{}\",\"shapes\":{},\"position\":[{},{},{}],\"rotation\":[{},{},{}],\"scale\":{},\"vertices\":{},\"triangles\":{},\"invalidIndices\":{},\"degenerateTriangles\":{},\"failure\":\"{}\"}}{}\n", reference.formId, reference.baseFormId, EscapeJson(reference.recordType), EscapeJson(reference.modelPath), EscapeJson(reference.nifVersion), JsonStringArray(reference.shapes), reference.position.x, reference.position.y, reference.position.z, reference.rotation.x, reference.rotation.y, reference.rotation.z, reference.scale, reference.vertices, reference.triangles, reference.invalidIndices, reference.degenerateTriangles, EscapeJson(reference.failure), index + 1 == geometry.references.size() ? "" : ",");
        }
        output << "  ]\n}\n"; return true;
    }
}
