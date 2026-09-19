#include "skyrim/extraction/geometry_extractor.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <format>
#include <optional>
#include <string_view>
#include <cstdlib>

namespace
{
    [[nodiscard]] std::string EscapeJson(std::string value)
    {
        std::string escaped;
        for (const auto character : value) {
            if (character == '\\') escaped += "\\\\";
            else if (character == '"') escaped += "\\\"";
            else escaped += character;
        }
        return escaped;
    }
    [[nodiscard]] std::uint16_t ReadU16(const std::vector<std::uint8_t>& bytes, std::size_t offset)
    {
        return static_cast<std::uint16_t>(bytes[offset]) | static_cast<std::uint16_t>(bytes[offset + 1] << 8);
    }
    [[nodiscard]] std::uint32_t ReadU32(const std::vector<std::uint8_t>& bytes, std::size_t offset)
    {
        return static_cast<std::uint32_t>(bytes[offset]) | (static_cast<std::uint32_t>(bytes[offset + 1]) << 8) | (static_cast<std::uint32_t>(bytes[offset + 2]) << 16) | (static_cast<std::uint32_t>(bytes[offset + 3]) << 24);
    }
    [[nodiscard]] float ReadFloat(const std::vector<std::uint8_t>& bytes, std::size_t offset)
    {
        float result{}; std::memcpy(&result, bytes.data() + offset, sizeof(result)); return result;
    }
    [[nodiscard]] navmesh::core::Vec3 Rotate(const navmesh::core::Vec3& value, const navmesh::core::Vec3& rotation)
    {
        const auto cx = std::cos(rotation.x); const auto sx = std::sin(rotation.x); const auto cy = std::cos(rotation.y); const auto sy = std::sin(rotation.y); const auto cz = std::cos(rotation.z); const auto sz = std::sin(rotation.z);
        const auto x1 = value.x; const auto y1 = value.y * cx - value.z * sx; const auto z1 = value.y * sx + value.z * cx; const auto x2 = x1 * cy + z1 * sy; const auto y2 = y1; const auto z2 = -x1 * sy + z1 * cy;
        return { x2 * cz - y2 * sz, x2 * sz + y2 * cz, z2 };
    }
    [[nodiscard]] std::optional<navmesh::core::Mesh> LoadNif(const std::filesystem::path& path)
    {
        std::ifstream input(path, std::ios::binary); if (!input) return std::nullopt;
        const std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        if (bytes.size() < 16 || std::string_view(reinterpret_cast<const char*>(bytes.data()), 4) != "Game") return std::nullopt;
        const auto text = std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size()); navmesh::core::Mesh result; std::size_t cursor = 0;
        while ((cursor = text.find("NiTriShapeData", cursor)) != std::string::npos) {
            cursor += 13; const auto dataStart = text.find('\0', cursor); if (dataStart == std::string::npos) break; auto offset = dataStart + 1;
            if (offset + 3 > bytes.size()) break; const auto vertexCount = ReadU16(bytes, offset); offset += 2; const auto hasVertices = bytes[offset++] != 0;
            if (!hasVertices || vertexCount == 0 || vertexCount > 1000000 || offset + vertexCount * 12 > bytes.size()) continue;
            std::vector<navmesh::core::Vec3> vertices; vertices.reserve(vertexCount);
            for (std::uint16_t index = 0; index < vertexCount; ++index) { vertices.push_back({ ReadFloat(bytes, offset), ReadFloat(bytes, offset + 4), ReadFloat(bytes, offset + 8) }); offset += 12; }
            if (offset >= bytes.size()) continue; if (bytes[offset++] != 0) offset += static_cast<std::size_t>(vertexCount) * 12; if (offset + 12 > bytes.size()) continue; offset += 12;
            if (offset >= bytes.size()) continue; if (bytes[offset++] != 0) offset += static_cast<std::size_t>(vertexCount) * 4; if (offset + 2 > bytes.size()) continue;
            const auto uvSets = ReadU16(bytes, offset); offset += 2; offset += static_cast<std::size_t>(uvSets) * vertexCount * 8; if (offset + 6 > bytes.size()) continue; offset += 2;
            const auto triangleCount = ReadU16(bytes, offset); offset += 2; if (offset + 4 > bytes.size()) continue; const auto pointCount = ReadU32(bytes, offset); offset += 4;
            if (pointCount != static_cast<std::uint32_t>(triangleCount) * 3 || offset + pointCount * 2 > bytes.size()) continue;
            const auto base = static_cast<std::uint32_t>(result.vertices.size()); result.vertices.insert(result.vertices.end(), vertices.begin(), vertices.end());
            for (std::uint16_t triangle = 0; triangle < triangleCount; ++triangle) { const auto a = ReadU16(bytes, offset); const auto b = ReadU16(bytes, offset + 2); const auto c = ReadU16(bytes, offset + 4); offset += 6; if (a < vertexCount && b < vertexCount && c < vertexCount) result.triangles.push_back({ { base + a, base + b, base + c } }); }
        }
        return result.vertices.empty() || result.triangles.empty() ? std::nullopt : std::optional{ std::move(result) };
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
            if (!mesh) { report.failure = std::filesystem::exists(modelPath) ? "unsupported or empty NIF" : "missing loose NIF"; ++output.modelsMissing; output.references.push_back(std::move(report)); continue; }
            const auto base = static_cast<std::uint32_t>(output.mesh.vertices.size());
            for (const auto& vertex : mesh->vertices) { auto transformed = Rotate({ vertex.x * reference.scale, vertex.y * reference.scale, vertex.z * reference.scale }, reference.rotation); transformed.x += reference.position.x; transformed.y += reference.position.y; transformed.z += reference.position.z; if (!std::isfinite(transformed.x) || !std::isfinite(transformed.y) || !std::isfinite(transformed.z) || std::abs(transformed.x) > 1.0e7F || std::abs(transformed.y) > 1.0e7F || std::abs(transformed.z) > 1.0e7F) ++output.invalidVertices; output.mesh.vertices.push_back(transformed); }
            for (const auto& triangle : mesh->triangles) output.mesh.triangles.push_back({ { base + triangle.vertices[0], base + triangle.vertices[1], base + triangle.vertices[2] } });
            report.vertices = mesh->vertices.size(); report.triangles = mesh->triangles.size(); ++output.modelsLoaded; output.references.push_back(std::move(report));
        }
        return output;
    }
    bool WriteGeometryObj(const std::filesystem::path& outputPath, const GeometryExtraction& geometry)
    {
        std::ofstream output(outputPath, std::ios::trunc); if (!output) return false; for (const auto& vertex : geometry.mesh.vertices) output << std::format("v {} {} {}\n", vertex.x, vertex.y, vertex.z); for (const auto& triangle : geometry.mesh.triangles) output << std::format("f {} {} {}\n", triangle.vertices[0] + 1, triangle.vertices[1] + 1, triangle.vertices[2] + 1); return true;
    }

    bool WriteGeometryJson(const std::filesystem::path& outputPath, const core::Cell& cell, const GeometryExtraction& geometry)
    {
        std::ofstream output(outputPath, std::ios::trunc); if (!output) return false;
        output << std::format("{{\n  \"cell\": \"{:08X}\",\n  \"references\": {},\n  \"referencesWithModels\": {},\n  \"modelsLoaded\": {},\n  \"modelsMissing\": {},\n  \"vertices\": {},\n  \"triangles\": {},\n  \"terrainSupported\": false,\n  \"collisionGeometrySupported\": false,\n  \"invalidVertices\": {},\n  \"invalidIndices\": {},\n  \"referenceDetails\": [\n", cell.id, cell.references.size(), geometry.referencesWithModels, geometry.modelsLoaded, geometry.modelsMissing, geometry.mesh.vertices.size(), geometry.mesh.triangles.size(), geometry.invalidVertices, geometry.invalidIndices);
        for (std::size_t index = 0; index < geometry.references.size(); ++index) {
            const auto& reference = geometry.references[index];
            output << std::format("    {{\"formId\":\"{:08X}\",\"baseFormId\":\"{:08X}\",\"recordType\":\"{}\",\"model\":\"{}\",\"position\":[{},{},{}],\"rotation\":[{},{},{}],\"scale\":{},\"vertices\":{},\"triangles\":{},\"failure\":\"{}\"}}{}\n", reference.formId, reference.baseFormId, EscapeJson(reference.recordType), EscapeJson(reference.modelPath), reference.position.x, reference.position.y, reference.position.z, reference.rotation.x, reference.rotation.y, reference.rotation.z, reference.scale, reference.vertices, reference.triangles, EscapeJson(reference.failure), index + 1 == geometry.references.size() ? "" : ",");
        }
        output << "  ]\n}\n"; return true;
    }
}