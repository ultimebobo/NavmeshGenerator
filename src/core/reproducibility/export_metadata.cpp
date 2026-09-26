#include "core/reproducibility/export_metadata.h"

#include <fstream>
#include <format>

namespace navmesh::reproducibility
{
    std::string EscapeJson(const std::string& value) { std::string result; for (const char c : value) { switch (c) { case '\\': result += "\\\\"; break; case '"': result += "\\\""; break; case '\n': result += "\\n"; break; case '\r': result += "\\r"; break; case '\t': result += "\\t"; break; default: result += c; break; } } return result; }
    std::string ToJson(const ExportMetadata& metadata, const std::string_view indent)
    {
        const auto& cell = metadata.selectedCell; std::string selectedCell = "null";
        if (cell) { selectedCell = std::format("{{\"form_id\": \"{:08X}\", \"editor_id\": \"{}\", \"is_interior\": {}", cell->id, EscapeJson(cell->editorId), cell->isInterior ? "true" : "false"); if (cell->exteriorCoordinates) selectedCell += std::format(", \"exterior_coordinates\": [{}, {}]", (*cell->exteriorCoordinates)[0], (*cell->exteriorCoordinates)[1]); selectedCell += "}"; }
        std::string warnings; for (std::size_t i = 0; i < metadata.warnings.size(); ++i) warnings += std::format("\"{}\"{}", EscapeJson(metadata.warnings[i]), i + 1 == metadata.warnings.size() ? "" : ", ");
        return std::format("{{\n{}\"schema\": \"navmesh-generator/export-metadata\",\n{}\"schema_version\": \"{}\",\n{}\"tool\": {{\"name\": \"navmesh-generator\", \"version\": \"{}\"}},\n{}\"input_plugins\": [\"{}\"],\n{}\"selected_cell\": {},\n{}\"coordinates\": {{\"convention\": \"skyrim-world-z-up-v1\", \"documentation\": \"docs/coordinate-system.md\"}},\n{}\"source_coverage\": {{\"references\": {}, \"references_with_models\": {}, \"models_loaded\": {}, \"models_missing\": {}, \"geometry_vertices\": {}, \"geometry_triangles\": {}, \"terrain_land_records\": {}, \"terrain_land_decoded\": {}, \"terrain_land_missing\": {}, \"terrain_supported\": {}, \"collision_geometry_supported\": {}}},\n{}\"warnings\": [{}]\n{}}}", indent, indent, kExportMetadataSchemaVersion, indent, kToolVersion, indent, EscapeJson(metadata.inputPlugin.generic_string()), indent, selectedCell, indent, indent, metadata.coverage.references, metadata.coverage.referencesWithModels, metadata.coverage.modelsLoaded, metadata.coverage.modelsMissing, metadata.coverage.geometryVertices, metadata.coverage.geometryTriangles, metadata.coverage.terrainLandRecords, metadata.coverage.terrainLandDecoded, metadata.coverage.terrainLandMissing, metadata.coverage.terrainSupported ? "true" : "false", metadata.coverage.collisionGeometrySupported ? "true" : "false", indent, warnings, indent);
    }
    bool WriteSidecar(const std::filesystem::path& path, const ExportMetadata& metadata) { std::ofstream output(std::filesystem::path(path.string() + ".metadata.json"), std::ios::trunc | std::ios::binary); if (!output) return false; output << "{\n  \"metadata\": " << ToJson(metadata, "    ") << "\n}\n"; return static_cast<bool>(output); }
}
