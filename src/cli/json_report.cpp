#include "cli/json_report.h"

#include <format>
#include <sstream>

namespace
{
    std::string Escape(const std::string& value) {
        std::string result; result.reserve(value.size());
        for (const char c : value) switch (c) { case '\\': result += "\\\\"; break; case '"': result += "\\\""; break; case '\n': result += "\\n"; break; case '\r': result += "\\r"; break; case '\t': result += "\\t"; break; default: if (static_cast<unsigned char>(c) < 0x20 || static_cast<unsigned char>(c) >= 0x80) result += std::format("\\u00{:02X}", static_cast<unsigned char>(c)); else result += c; break; }
        return result;
    }
    void WriteVec3(std::ostream& output, const navmesh::core::Vec3& value) { output << std::format("[{}, {}, {}]", value.x, value.y, value.z); }
}

namespace navmesh::cli
{
    std::string ToJson(const core::Cell& cell, const std::vector<validation::Finding>& findings)
    {
        std::ostringstream output;
        output << "{\n  \"schema_version\": 1,\n  \"cell\": {\n";
        output << std::format("    \"id\": \"{:08X}\",\n    \"editor_id\": \"{}\",\n    \"name\": \"{}\",\n    \"is_interior\": {}", cell.id, Escape(cell.editorId), Escape(cell.name), cell.isInterior ? "true" : "false");
        if (cell.exteriorCoordinates) output << std::format(",\n    \"exterior_coordinates\": [{}, {}]", (*cell.exteriorCoordinates)[0], (*cell.exteriorCoordinates)[1]);
        output << "\n  },\n  \"references\": [";
        for (std::size_t index = 0; index < cell.references.size(); ++index) {
            const auto& reference = cell.references[index];
            output << (index ? "," : "") << "\n    {" << std::format("\"id\": \"{:08X}\", \"base_object_id\": \"{:08X}\", \"record_type\": \"{}\", \"editor_id\": \"{}\", \"model\": \"{}\", \"name\": \"{}\", \"position\": ", reference.id, reference.baseObjectId, Escape(reference.recordType), Escape(reference.editorId), Escape(reference.modelPath), Escape(reference.name)); WriteVec3(output, reference.position); output << ", \"rotation_radians\": "; WriteVec3(output, reference.rotation); output << std::format(", \"scale\": {}", reference.scale);
            if (reference.localBounds) { output << ", \"local_bounds\": {\"min\": "; WriteVec3(output, reference.localBounds->min); output << ", \"max\": "; WriteVec3(output, reference.localBounds->max); output << "}"; }
            output << "}";
        }
        output << "\n  ],\n  \"navmeshes\": [";
        for (std::size_t index = 0; index < cell.navMeshes.size(); ++index) {
            const auto& mesh = cell.navMeshes[index]; output << (index ? "," : "") << std::format("\n    {{\"id\": \"{:08X}\", \"vertex_count\": {}, \"polygon_count\": {}, \"vertices\": [", mesh.id, mesh.vertices.size(), mesh.polygons.size());
            for (std::size_t vertex = 0; vertex < mesh.vertices.size(); ++vertex) { output << (vertex ? ", " : ""); WriteVec3(output, mesh.vertices[vertex]); }
            output << "], \"polygons\": [";
            for (std::size_t polygon = 0; polygon < mesh.polygons.size(); ++polygon) { const auto& value = mesh.polygons[polygon]; output << (polygon ? ", " : "") << std::format("{{\"vertices\":[{}, {}, {}],\"neighbors\":[{}, {}, {}],\"flags\":{}}}", value.vertices[0], value.vertices[1], value.vertices[2], value.neighbors[0], value.neighbors[1], value.neighbors[2], value.flags); }
            output << "]}";
        }
        output << "\n  ],\n  \"validation\": [";
        for (std::size_t index = 0; index < findings.size(); ++index) { const auto& finding = findings[index]; output << (index ? ", " : "") << std::format("{{\"severity\":\"{}\",\"message\":\"{}\"}}", finding.severity == validation::Severity::error ? "error" : "warning", Escape(finding.message)); }
        output << "]\n}\n";
        return output.str();
    }
}
