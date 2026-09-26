#include "analysis/navmesh_analysis.h"
#include "cli/json_report.h"
#include "skyrim/parser/plugin_parser.h"
#include "skyrim/mo2/mo2_importer.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/extraction/terrain_extractor.h"
#include "validation/validation.h"

#include <filesystem>
#include <fstream>
#include <format>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace
{
    struct Options
    {
        std::filesystem::path plugin;
        std::filesystem::path data;
        std::filesystem::path loadOrder;
        std::filesystem::path mo2;
        std::filesystem::path modsDirectory;
        std::string profile;
        std::string cell;
        std::string editorId;
        std::string worldspace;
        std::optional<std::int32_t> cellX;
        std::optional<std::int32_t> cellY;
        std::optional<std::uint32_t> cellFormId;
        std::filesystem::path output;
        bool listCells{};
        std::filesystem::path exportGeometry;
        std::filesystem::path exportAnalysis;
        bool diagnostics{};
        bool terrainOnly{};
        float surfaceSearchRadius{ 64.0F };
        float maxSupportDistance{ 32.0F };
        float maxSlope{ 45.0F };
    };

    [[nodiscard]] std::string ToString(const std::optional<std::int32_t>& value)
    {
        return value ? std::to_string(*value) : "";
    }

    [[nodiscard]] Options ParseArgs(int argc, char** argv)
    {
        Options options;
        options.output = ".";
        for (int index = 1; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--plugin" && index + 1 < argc) {
                options.plugin = argv[++index];
            } else if (argument == "--data" && index + 1 < argc) {
                options.data = argv[++index];
            } else if (argument == "--load-order" && index + 1 < argc) {
                options.loadOrder = argv[++index];
            } else if (argument == "--mo2" && index + 1 < argc) {
                options.mo2 = argv[++index];
            } else if (argument == "--profile" && index + 1 < argc) {
                options.profile = argv[++index];
            } else if (argument == "--mods-dir" && index + 1 < argc) {
                options.modsDirectory = argv[++index];
            } else if (argument == "--cell" && index + 1 < argc) {
                options.cell = argv[++index];
            } else if (argument == "--cell-formid" && index + 1 < argc) {
                options.cellFormId = static_cast<std::uint32_t>(std::stoul(argv[++index], nullptr, 16));
            } else if (argument == "--editor-id" && index + 1 < argc) {
                options.editorId = argv[++index];
            } else if (argument == "--worldspace" && index + 1 < argc) {
                options.worldspace = argv[++index];
            } else if (argument == "--cell-x" && index + 1 < argc) {
                options.cellX = std::stoi(argv[++index]);
            } else if (argument == "--cell-y" && index + 1 < argc) {
                options.cellY = std::stoi(argv[++index]);
            } else if (argument == "--output" && index + 1 < argc) {
                options.output = argv[++index];
            } else if (argument == "--export-geometry" && index + 1 < argc) {
                options.exportGeometry = argv[++index];
            } else if (argument == "--export-analysis" && index + 1 < argc) {
                options.exportAnalysis = argv[++index];
            } else if (argument == "--diagnostics") {
                options.diagnostics = true;
            } else if (argument == "--terrain-only") {
                options.terrainOnly = true;
            } else if (argument == "--surface-search-radius" && index + 1 < argc) {
                options.surfaceSearchRadius = std::stof(argv[++index]);
            } else if (argument == "--max-support-distance" && index + 1 < argc) {
                options.maxSupportDistance = std::stof(argv[++index]);
            } else if (argument == "--max-slope" && index + 1 < argc) {
                options.maxSlope = std::stof(argv[++index]);
            } else if (argument == "--list-cells") {
                options.listCells = true;
            }
        }
        return options;
    }

    void WriteObj(const std::filesystem::path& outputPath, const navmesh::core::NavMesh& mesh, const std::string& name)
    {
        std::ofstream stream(outputPath, std::ios::trunc);
        if (!stream.is_open()) {
            return;
        }
        for (const auto& vertex : mesh.vertices) {
            stream << std::format("v {} {} {}\n", vertex.x, vertex.y, vertex.z);
        }
        for (const auto& polygon : mesh.polygons) {
            if (polygon.vertices[0] >= mesh.vertices.size() || polygon.vertices[1] >= mesh.vertices.size() || polygon.vertices[2] >= mesh.vertices.size()) {
                continue;
            }
            stream << std::format("f {} {} {}\n",
                static_cast<std::uint32_t>(polygon.vertices[0] + 1),
                static_cast<std::uint32_t>(polygon.vertices[1] + 1),
                static_cast<std::uint32_t>(polygon.vertices[2] + 1));
        }
        stream << "# exported from " << name << "\n";
    }

    void WriteAnalysisObj(const std::filesystem::path& outputPath, const navmesh::core::NavMesh& mesh, const navmesh::core::Mesh& geometry, const navmesh::analysis::AnalysisReport& analysisReport)
    {
        std::ofstream stream(outputPath, std::ios::trunc);
        if (!stream.is_open()) {
            return;
        }

        std::size_t vertexIndex = 0;
        auto emitVertex = [&](const navmesh::core::Vec3& vertex) -> std::size_t {
            stream << std::format("v {} {} {}\n", vertex.x, vertex.y, vertex.z);
            return ++vertexIndex;
        };

        stream << "g NAVM\n";
        for (const auto& polygon : mesh.polygons) {
            if (polygon.vertices[0] >= mesh.vertices.size() || polygon.vertices[1] >= mesh.vertices.size() || polygon.vertices[2] >= mesh.vertices.size()) {
                continue;
            }
            const auto a = mesh.vertices[polygon.vertices[0]];
            const auto b = mesh.vertices[polygon.vertices[1]];
            const auto c = mesh.vertices[polygon.vertices[2]];
            const auto va = emitVertex(a);
            const auto vb = emitVertex(b);
            const auto vc = emitVertex(c);
            stream << std::format("f {} {} {}\n", va, vb, vc);
        }

        stream << "g SUPPORT\n";
        for (const auto& polygon : analysisReport.polygons) {
            if (!polygon.support.found || polygon.support.triangleIndex >= geometry.triangles.size()) {
                continue;
            }
            const auto& triangle = geometry.triangles[polygon.support.triangleIndex];
            if (triangle.vertices[0] >= geometry.vertices.size() || triangle.vertices[1] >= geometry.vertices.size() || triangle.vertices[2] >= geometry.vertices.size()) {
                continue;
            }
            const auto va = emitVertex(geometry.vertices[triangle.vertices[0]]);
            const auto vb = emitVertex(geometry.vertices[triangle.vertices[1]]);
            const auto vc = emitVertex(geometry.vertices[triangle.vertices[2]]);
            stream << std::format("f {} {} {}\n", va, vb, vc);
        }

        stream << "# exported analysis visualization\n";
    }

    [[nodiscard]] std::string JsonEscape(const std::string& value)
    {
        std::string result;
        for (const char character : value) {
            if (character == '\\') result += "\\\\";
            else if (character == '"') result += "\\\"";
            else if (character == '\n') result += "\\n";
            else if (character == '\r') result += "\\r";
            else result += character;
        }
        return result;
    }

    void WriteAnalysisJson(const std::filesystem::path& outputPath, const navmesh::analysis::AnalysisReport& analysisReport, const navmesh::analysis::GeometrySummary& geometrySummary, const navmesh::analysis::NavMeshSummary& meshSummary, const navmesh::reproducibility::ExportMetadata& metadata)
    {
        std::ofstream stream(outputPath, std::ios::trunc);
        if (!stream.is_open()) {
            return;
        }

        stream << "{\n  \"metadata\": " << navmesh::reproducibility::ToJson(metadata, "    ") << ",\n";
        stream << "  \"geometry\": {\n";
        stream << std::format("    \"vertices\": {},\n", geometrySummary.vertexCount);
        stream << std::format("    \"triangles\": {},\n", geometrySummary.triangleCount);
        stream << "    \"bounds\": {\n";
        stream << std::format("      \"min\": [{}, {}, {}],\n", geometrySummary.bounds.min.x, geometrySummary.bounds.min.y, geometrySummary.bounds.min.z);
        stream << std::format("      \"max\": [{}, {}, {}]\n", geometrySummary.bounds.max.x, geometrySummary.bounds.max.y, geometrySummary.bounds.max.z);
        stream << "    }\n";
        stream << "  },\n";
        stream << "  \"navmesh\": {\n";
        stream << std::format("    \"vertices\": {},\n", meshSummary.vertexCount);
        stream << std::format("    \"polygons\": {},\n", meshSummary.polygonCount);
        stream << "    \"bounds\": {\n";
        stream << std::format("      \"min\": [{}, {}, {}],\n", meshSummary.bounds.min.x, meshSummary.bounds.min.y, meshSummary.bounds.min.z);
        stream << std::format("      \"max\": [{}, {}, {}]\n", meshSummary.bounds.max.x, meshSummary.bounds.max.y, meshSummary.bounds.max.z);
        stream << "    }\n";
        stream << "  },\n";
        stream << "  \"analysis\": {\n";
        stream << "    \"configuration\": {\n";
        stream << std::format("      \"surfaceSearchRadius\": {},\n", analysisReport.configuration.surfaceSearchRadius);
        stream << std::format("      \"maxSupportDistance\": {},\n", analysisReport.configuration.maxSupportDistance);
        stream << std::format("      \"maxSlope\": {}\n", analysisReport.configuration.maxSlope);
        stream << "    },\n";
        stream << "    \"summary\": {\n";
        stream << std::format("      \"polygonsAnalyzed\": {},\n", analysisReport.summary.polygonsAnalyzed);
        stream << std::format("      \"supportFound\": {},\n", analysisReport.summary.supportFound);
        stream << std::format("      \"unsupported\": {},\n", analysisReport.summary.unsupported);
        stream << std::format("      \"supported\": {},\n", analysisReport.summary.supported);
        stream << std::format("      \"floating\": {},\n", analysisReport.summary.floating);
        stream << std::format("      \"buried\": {},\n", analysisReport.summary.buried);
        stream << std::format("      \"tooSteep\": {}\n", analysisReport.summary.tooSteep);
        stream << "    },\n";
        stream << "    \"heightDelta\": {\n";
        stream << std::format("      \"min\": {},\n", analysisReport.heightDeltaStats.min);
        stream << std::format("      \"max\": {},\n", analysisReport.heightDeltaStats.max);
        stream << std::format("      \"mean\": {},\n", analysisReport.heightDeltaStats.mean);
        stream << std::format("      \"median\": {},\n", analysisReport.heightDeltaStats.median);
        stream << std::format("      \"p95\": {}\n", analysisReport.heightDeltaStats.p95);
        stream << "    },\n";
        stream << "    \"slope\": {\n";
        stream << std::format("      \"min\": {},\n", analysisReport.slopeStats.min);
        stream << std::format("      \"max\": {},\n", analysisReport.slopeStats.max);
        stream << std::format("      \"mean\": {},\n", analysisReport.slopeStats.mean);
        stream << std::format("      \"median\": {},\n", analysisReport.slopeStats.median);
        stream << std::format("      \"p95\": {}\n", analysisReport.slopeStats.p95);
        stream << "    },\n";
        stream << "    \"polygons\": [\n";
        for (std::size_t index = 0; index < analysisReport.polygons.size(); ++index) {
            const auto& polygon = analysisReport.polygons[index];
            stream << "      {\n";
            stream << std::format("        \"index\": {},\n", polygon.index);
            stream << "        \"centroid\": [" << polygon.centroid.x << ", " << polygon.centroid.y << ", " << polygon.centroid.z << "],\n";
            stream << "        \"normal\": [" << polygon.normal.x << ", " << polygon.normal.y << ", " << polygon.normal.z << "],\n";
            stream << "        \"support\": {\n";
            stream << std::format("          \"found\": {}{}\n", polygon.support.found ? "true" : "false", polygon.support.found ? "," : "");
            if (polygon.support.found) {
                stream << "          \"point\": [" << polygon.support.point.x << ", " << polygon.support.point.y << ", " << polygon.support.point.z << "],\n";
                stream << std::format("          \"distance\": {},\n", polygon.support.distance);
                stream << std::format("          \"heightDelta\": {},\n", polygon.support.heightDelta);
                stream << "          \"normal\": [" << polygon.support.normal.x << ", " << polygon.support.normal.y << ", " << polygon.support.normal.z << "],\n";
                stream << std::format("          \"slopeDegrees\": {},\n", polygon.support.slopeDegrees);
                stream << std::format("          \"triangleIndex\": {},\n", polygon.support.triangleIndex);
                    stream << std::format("          \"sourceNifPath\": \"{}\",\n", JsonEscape(polygon.support.sourceNifPath));
                if (polygon.support.sourceTriangleIndex) stream << std::format("          \"sourceTriangleIndex\": {}\n", *polygon.support.sourceTriangleIndex);
                else stream << "          \"sourceTriangleIndex\": null\n";
            }
            stream << "        },\n";
            stream << std::format("        \"classification\": \"{}\"\n", polygon.classification);
            stream << "      }" << (index + 1 == analysisReport.polygons.size() ? "" : ",") << "\n";
        }
        stream << "    ]\n";
        stream << "  }\n";
        stream << "}\n";
    }

    [[nodiscard]] std::string HtmlEscape(const std::string& value)
    {
        std::string result;
        for (const char character : value) {
            if (character == '&') result += "&amp;";
            else if (character == '<') result += "&lt;";
            else if (character == '>') result += "&gt;";
            else if (character == '"') result += "&quot;";
            else result += character;
        }
        return result;
    }

    [[nodiscard]] std::string Vec3Text(const navmesh::core::Vec3& value)
    {
        return std::format("({:.2f}, {:.2f}, {:.2f})", value.x, value.y, value.z);
    }

    void AnnotateSupportSources(navmesh::analysis::AnalysisReport& report, const navmesh::skyrim::offline::GeometryExtraction& geometry)
    {
        for (auto& polygon : report.polygons) {
            if (!polygon.support.found) continue;
            for (const auto& reference : geometry.references) {
                if (polygon.support.triangleIndex < reference.meshTriangleOffset || polygon.support.triangleIndex >= reference.meshTriangleOffset + reference.triangles) continue;
                polygon.support.sourceNifPath = reference.modelPath;
                polygon.support.sourceTriangleIndex = polygon.support.triangleIndex - reference.meshTriangleOffset;
                break;
            }
        }
    }

    [[nodiscard]] std::string ProjectionSvg(const navmesh::core::NavMesh& mesh, const navmesh::core::Mesh& geometry, const navmesh::analysis::PolygonAnalysisResult& polygon)
    {
        std::vector<navmesh::core::Vec3> points;
        if (polygon.index < mesh.polygons.size()) {
            const auto& navPolygon = mesh.polygons[polygon.index];
            for (const auto vertex : navPolygon.vertices) if (vertex < mesh.vertices.size()) points.push_back(mesh.vertices[vertex]);
        }
        if (polygon.support.found && polygon.support.triangleIndex < geometry.triangles.size()) {
            const auto& triangle = geometry.triangles[polygon.support.triangleIndex];
            for (const auto vertex : triangle.vertices) if (vertex < geometry.vertices.size()) points.push_back(geometry.vertices[vertex]);
        }
        if (points.empty()) return "<svg viewBox=\"0 0 360 90\"><text x=\"8\" y=\"20\">no geometry</text></svg>";
        float minX = points.front().x, maxX = points.front().x, minY = points.front().y, maxY = points.front().y, minZ = points.front().z, maxZ = points.front().z;
        for (const auto& point : points) { minX = std::min(minX, point.x); maxX = std::max(maxX, point.x); minY = std::min(minY, point.y); maxY = std::max(maxY, point.y); minZ = std::min(minZ, point.z); maxZ = std::max(maxZ, point.z); }
        const auto scale = [](float value, float low, float high, float outputLow, float outputHigh) { return outputLow + (high - low > 1.0e-4F ? (value - low) / (high - low) : 0.5F) * (outputHigh - outputLow); };
        const auto topX = [&](const navmesh::core::Vec3& point) { return scale(point.x, minX, maxX, 12.0F, 168.0F); };
        const auto topY = [&](const navmesh::core::Vec3& point) { return scale(point.y, minY, maxY, 74.0F, 12.0F); };
        const auto sideX = [&](const navmesh::core::Vec3& point) { return scale(point.x, minX, maxX, 192.0F, 348.0F); };
        const auto sideY = [&](const navmesh::core::Vec3& point) { return scale(point.z, minZ, maxZ, 74.0F, 12.0F); };
        std::ostringstream svg;
        svg << "<svg viewBox=\"0 0 360 90\" role=\"img\"><rect width=\"360\" height=\"90\" fill=\"#f7f4ed\"/><text x=\"12\" y=\"9\" font-size=\"6\">top</text><text x=\"192\" y=\"9\" font-size=\"6\">side</text>";
        if (points.size() >= 3) svg << std::format("<polygon points=\"{},{} {},{} {},{}\" fill=\"#386641\" fill-opacity=\".35\" stroke=\"#386641\"/>", topX(points[0]), topY(points[0]), topX(points[1]), topY(points[1]), topX(points[2]), topY(points[2]));
        if (points.size() >= 3) svg << std::format("<polygon points=\"{},{} {},{} {},{}\" fill=\"#386641\" fill-opacity=\".35\" stroke=\"#386641\"/>", sideX(points[0]), sideY(points[0]), sideX(points[1]), sideY(points[1]), sideX(points[2]), sideY(points[2]));
        if (points.size() >= 6) svg << std::format("<polygon points=\"{},{} {},{} {},{}\" fill=\"#bc4749\" fill-opacity=\".35\" stroke=\"#bc4749\"/>", topX(points[3]), topY(points[3]), topX(points[4]), topY(points[4]), topX(points[5]), topY(points[5]));
        for (std::size_t index = 0; index < std::min<std::size_t>(points.size(), 6); ++index) svg << std::format("<circle cx=\"{}\" cy=\"{}\" r=\"1.5\" fill=\"{}\"/>", topX(points[index]), topY(points[index]), index < 3 ? "#386641" : "#bc4749");
        if (polygon.support.found) svg << std::format("<circle cx=\"{}\" cy=\"{}\" r=\"2.5\" fill=\"#f08a24\"/><circle cx=\"{}\" cy=\"{}\" r=\"2.5\" fill=\"#f08a24\"/>", topX(polygon.centroid), topY(polygon.centroid), sideX(polygon.centroid), sideY(polygon.centroid));
        if (points.size() >= 6) svg << std::format("<polygon points=\"{},{} {},{} {},{}\" fill=\"none\" stroke=\"#bc4749\" stroke-dasharray=\"2 2\"/>", sideX(points[3]), sideY(points[3]), sideX(points[4]), sideY(points[4]), sideX(points[5]), sideY(points[5]));
        svg << "</svg>";
        return svg.str();
    }

    void WriteDiagnosticHtml(const std::filesystem::path& outputPath, const navmesh::core::Cell& cell, const navmesh::core::NavMesh& mesh, const navmesh::core::Mesh& geometry, const navmesh::skyrim::offline::GeometryExtraction& extraction, const navmesh::analysis::AnalysisReport& report)
    {
        std::ofstream stream(outputPath, std::ios::trunc);
        if (!stream) return;
        std::vector<const navmesh::analysis::PolygonAnalysisResult*> selected;
        const std::vector<std::string> classes = { "supported", "floating", "buried", "too_steep", "unsupported" };
        for (const auto& classification : classes) {
            std::size_t limit = classification == "buried" ? 10U : 1U;
            for (const auto& polygon : report.polygons) if (polygon.classification == classification && limit-- > 0) selected.push_back(&polygon);
        }
        const auto missingModels = extraction.modelsMissing > 0 || !extraction.terrainSupported || !extraction.collisionGeometrySupported;
        stream << "<!doctype html><html><head><meta charset=\"utf-8\"><title>NAVM support diagnostics</title><style>body{font:14px system-ui,sans-serif;color:#252422;background:#f1eee7;margin:2rem}h1{font-family:Georgia,serif}table{border-collapse:collapse;background:#fff}th,td{border:1px solid #d8d2c6;padding:.45rem;text-align:left;vertical-align:top}th{background:#283618;color:#fff}tr.buried{background:#fff0e1}svg{width:360px;max-width:100%;height:auto}code{white-space:nowrap}</style></head><body>";
        stream << std::format("<h1>NAVM support diagnostics</h1><p>Cell <code>{:08X}</code> &middot; {} &middot; polygons analyzed: {} &middot; geometry triangles: {}</p>", cell.id, HtmlEscape(cell.editorId.empty() ? cell.name : cell.editorId), report.summary.polygonsAnalyzed, geometry.triangles.size());
        stream << std::format("<p><strong>Counts:</strong> supported {} &middot; floating {} &middot; buried {} &middot; too steep {} &middot; unsupported {}.</p>", report.summary.supported, report.summary.floating, report.summary.buried, report.summary.tooSteep, report.summary.unsupported);
        stream << "<h2>Diagnostic assessment</h2><p>Each support shown is the nearest upward-facing triangle selected by the existing downward-centroid ray. No thresholds were changed.</p>";
        stream << (missingModels ? "<p><strong>Geometry limitation:</strong> terrain or collision geometry is not included, or one or more model NIFs could not be loaded; buried counts may be incomplete or misleading where those surfaces are missing.</p>" : "<p><strong>Geometry coverage:</strong> extracted model geometry is available for the analyzed cell; the examples below can be inspected as actual world-space support triangles.</p>");
        stream << "<p><strong>Interpretation:</strong> a buried result is supported by a concrete selected triangle and a negative height delta. When the triangle has NIF provenance and the two views show the NAVM above that triangle, it is consistent with genuinely buried NAVM. A missing source, visibly displaced triangle, or a systematic offset is evidence for geometry/transform limitations and should be investigated before changing thresholds.</p>";
        stream << "<h2>Selected polygons</h2><table><thead><tr><th>Index</th><th>Class</th><th>Centroid</th><th>Support point</th><th>Delta</th><th>Slope</th><th>Support triangle</th><th>Source</th><th>Views</th></tr></thead><tbody>";
        for (const auto* polygon : selected) {
            std::string triangleText = "none";
            std::string sourceText = "none";
            if (polygon->support.found && polygon->support.triangleIndex < geometry.triangles.size()) {
                const auto& triangle = geometry.triangles[polygon->support.triangleIndex];
                triangleText = std::format("world #{}", polygon->support.triangleIndex);
                if (triangle.vertices[0] < geometry.vertices.size() && triangle.vertices[1] < geometry.vertices.size() && triangle.vertices[2] < geometry.vertices.size()) triangleText += " " + Vec3Text(geometry.vertices[triangle.vertices[0]]) + " / " + Vec3Text(geometry.vertices[triangle.vertices[1]]) + " / " + Vec3Text(geometry.vertices[triangle.vertices[2]]);
                sourceText = HtmlEscape(polygon->support.sourceNifPath.empty() ? "unknown" : polygon->support.sourceNifPath);
                if (polygon->support.sourceTriangleIndex) sourceText += std::format(" #{}", *polygon->support.sourceTriangleIndex);
            }
            stream << std::format("<tr class=\"{}\"><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td><small>{}</small></td><td><small>{}</small></td><td>{}</td></tr>", polygon->classification == "buried" ? "buried" : "", polygon->index, polygon->classification, Vec3Text(polygon->centroid), polygon->support.found ? Vec3Text(polygon->support.point) : "none", polygon->support.found ? std::format("{:.2f}", polygon->support.heightDelta) : "none", polygon->support.found ? std::format("{:.2f} deg", polygon->support.slopeDegrees) : "none", HtmlEscape(triangleText), sourceText, ProjectionSvg(mesh, geometry, *polygon));
        }
        stream << "</tbody></table><p><strong>Legend:</strong> green triangle = NAVM polygon; red triangle = world-geometry support triangle in the top view; dotted red triangle = the same world-geometry support triangle in the side view; orange markers = NAVM centroid and selected support point.</p></body></html>\n";
    }

    void PrintAnalysisSummary(const navmesh::analysis::AnalysisReport& analysis)
    {
        std::cout << "\nNAVM analysis\n";
        std::cout << std::format("Polygons analyzed: {}\n", analysis.summary.polygonsAnalyzed);
        std::cout << std::format("Support found:     {}\n", analysis.summary.supportFound);
        std::cout << std::format("Supported:         {}\n", analysis.summary.supported);
        std::cout << std::format("Unsupported:       {}\n", analysis.summary.unsupported);
        std::cout << std::format("Floating:          {}\n", analysis.summary.floating);
        std::cout << std::format("Buried:            {}\n", analysis.summary.buried);
        std::cout << std::format("Too steep:         {}\n", analysis.summary.tooSteep);
        std::cout << "\nHeight delta:\n";
        std::cout << std::format("  min:    {}\n  max:    {}\n  mean:   {}\n  median: {}\n  p95:    {}\n", analysis.heightDeltaStats.min, analysis.heightDeltaStats.max, analysis.heightDeltaStats.mean, analysis.heightDeltaStats.median, analysis.heightDeltaStats.p95);
        std::cout << "\nSlope:\n";
        std::cout << std::format("  min:    {}\n  max:    {}\n  mean:   {}\n  median: {}\n  p95:    {}\n", analysis.slopeStats.min, analysis.slopeStats.max, analysis.slopeStats.mean, analysis.slopeStats.median, analysis.slopeStats.p95);
    }

    void WriteLoadOrderJson(const std::filesystem::path& path, const navmesh::skyrim::offline::ResolvedLoadOrder& loadOrder)
    {
        std::ofstream stream(path, std::ios::trunc);
        if (!stream) return;
        stream << "{\n  \"records\": [\n";
        for (std::size_t index = 0; index < loadOrder.records.size(); ++index) {
            const auto& record = loadOrder.records[index];
            stream << std::format("    {{\"form_id\": \"{:08X}\", \"type\": \"{}\", \"winning_plugin\": \"{}\", \"origin_chain\": [", record.formId, JsonEscape(record.type), JsonEscape(record.winning.plugin));
            for (std::size_t origin = 0; origin < record.origins.size(); ++origin) stream << std::format("\"{}\"{}", JsonEscape(record.origins[origin].plugin), origin + 1 == record.origins.size() ? "" : ", ");
            stream << "]}" << (index + 1 == loadOrder.records.size() ? "" : ",") << "\n";
        }
        stream << "  ]\n}\n";
    }

    void WriteCellsJson(const std::filesystem::path& path, const std::vector<navmesh::core::Cell>& cells)
    {
        std::ofstream stream(path, std::ios::trunc); if (!stream) return;
        stream << "{\n  \"cells\": [\n";
        for (std::size_t index = 0; index < cells.size(); ++index) {
            const auto& cell = cells[index]; stream << std::format("    {{\"form_id\": \"{:08X}\", \"editor_id\": \"{}\", \"name\": \"{}\", \"interior\": {}", cell.id, JsonEscape(cell.editorId), JsonEscape(cell.name), cell.isInterior ? "true" : "false");
            if (cell.exteriorCoordinates) stream << std::format(", \"coordinates\": [{}, {}]", (*cell.exteriorCoordinates)[0], (*cell.exteriorCoordinates)[1]);
            stream << "}" << (index + 1 == cells.size() ? "" : ",") << "\n";
        }
        stream << "  ]\n}\n";
    }
}

int main(int argc, char** argv)
{
    auto options = ParseArgs(argc, argv);
    if (options.mo2.empty() && options.plugin.empty() && options.loadOrder.empty()) {
        std::cerr << "Usage: navmesh-offline --mo2 <instance-or-portable-root> --profile <existing-profile> [--mods-dir <moved-mods-root>] [--list-cells] [--cell-formid <hex>] --output <dir>\nDeveloper/test override: --data <Data> --load-order <plugins.txt>.\n";
        return 1;
    }
    if (!options.mo2.empty() && options.profile.empty()) { std::cerr << "--mo2 requires --profile naming an existing MO2 profile.\n"; return 1; }

    std::optional<navmesh::skyrim::offline::ResolvedLoadOrder> resolved;
    std::optional<navmesh::skyrim::offline::Mo2ProfileInput> mo2Input;
    if (!options.mo2.empty()) {
        std::filesystem::create_directories(options.output);
        std::cerr << "Importing MO2 profile and virtual-file winners...\n";
        if (!options.modsDirectory.empty()) std::cerr << "Using --mods-dir override: " << options.modsDirectory.string() << "\n";
        try {
            mo2Input = navmesh::skyrim::offline::ImportMo2Profile(options.mo2, options.profile,
                options.modsDirectory.empty() ? std::nullopt : std::optional(options.modsDirectory));
        } catch (const std::exception& error) {
            mo2Input = navmesh::skyrim::offline::Mo2ProfileInput{ .instanceRoot = options.mo2, .profile = options.profile };
            mo2Input->diagnostics.push_back({ navmesh::skyrim::offline::DiagnosticKind::InvalidPlugin, options.mo2.string(), std::string("MO2 import failed: ") + error.what() });
        }
        if (!navmesh::skyrim::offline::WriteInputReport(options.output / "input-report.json", *mo2Input)) std::cerr << "Failed to write input-report.json.\n";
        std::cerr << "Effective MO2 mods directory: " << mo2Input->modsDirectory.string() << "\n";
        for (const auto& diagnostic : mo2Input->diagnostics) std::cerr << diagnostic.plugin << ": " << diagnostic.message << "\n";
        if (mo2Input->pluginPaths.empty()) { std::cerr << "MO2 import found no usable active plugin paths; see input-report.json.\n"; return 2; }
        mo2Input->looseAssetWinners.clear(); mo2Input->looseAssetWinners.shrink_to_fit(); mo2Input->enabledMods.clear(); mo2Input->enabledMods.shrink_to_fit();
        options.data = mo2Input->gameData;
        std::cerr << "Resolving worldspace and cell records from " << mo2Input->pluginPaths.size() << " active plugins...\n";
        try {
            resolved = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = options.data, .plugins = mo2Input->pluginPaths, .indexReferencesAndNavmeshes = !options.listCells });
        } catch (const std::exception& error) {
            std::cerr << "Load-order resolution failed: " << error.what() << ". See input-report.json for the imported profile inputs.\n";
            return 2;
        }
    } else if (!options.loadOrder.empty()) {
        try { resolved = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = options.data, .plugins = navmesh::skyrim::offline::ReadLoadOrderManifest(options.loadOrder), .indexReferencesAndNavmeshes = !options.listCells }); }
        catch (const std::exception& error) { std::cerr << "Developer load-order resolution failed: " << error.what() << "\n"; return 2; }
    }
    if (resolved) {
        std::size_t unsupported{};
        for (const auto& diagnostic : resolved->diagnostics) {
            if (diagnostic.kind == navmesh::skyrim::offline::DiagnosticKind::UnsupportedRecord) { ++unsupported; continue; }
            std::cerr << diagnostic.plugin << ": " << diagnostic.message << "\n";
        }
        if (unsupported != 0) std::cerr << unsupported << " unsupported record variants were recorded in input-report.json/load-order.json.\n";
    }
    if (resolved && std::any_of(resolved->diagnostics.begin(), resolved->diagnostics.end(), [](const auto& d) { return d.kind == navmesh::skyrim::offline::DiagnosticKind::MissingMaster || d.kind == navmesh::skyrim::offline::DiagnosticKind::Cycle || d.kind == navmesh::skyrim::offline::DiagnosticKind::InvalidPlugin; })) return 2;
    if (mo2Input && !navmesh::skyrim::offline::ProfileSnapshotMatches(*mo2Input)) {
        std::cerr << "MO2 profile inputs changed while resolving the load order; rerun so the snapshot is coherent.\n";
        return 2;
    }

    if (options.listCells) {
        if (resolved) { std::filesystem::create_directories(options.output); WriteLoadOrderJson(options.output / "load-order.json", *resolved); }
        const auto cells = resolved ? resolved->cells : navmesh::skyrim::offline::ListCells(options.plugin);
        if (!options.output.empty()) { std::filesystem::create_directories(options.output); WriteCellsJson(options.output / "cells.json", cells); }
        for (const auto& cell : cells) {
            std::cout << std::format("{:08X} editor_id=\"{}\" name=\"{}\" type={} coords=",
                cell.id,
                cell.editorId,
                cell.name,
                cell.isInterior ? "interior" : "exterior");
            if (cell.exteriorCoordinates) {
                std::cout << std::format("{},{}", (*cell.exteriorCoordinates)[0], (*cell.exteriorCoordinates)[1]);
            } else {
                std::cout << "-";
            }
            std::cout << "\n";
        }
        return cells.empty() ? 2 : 0;
    }

    std::filesystem::create_directories(options.output);
    if (resolved) WriteLoadOrderJson(options.output / "load-order.json", *resolved);
    auto cell = resolved ? std::optional<navmesh::core::Cell>{} : navmesh::skyrim::offline::LoadCell(options.plugin, options.cell, options.worldspace, options.cellX, options.cellY, options.cellFormId, options.editorId);
    if (resolved) for (const auto& candidate : resolved->cells) {
        const auto formMatch = options.cellFormId && candidate.id == *options.cellFormId;
        const auto editorMatch = !options.editorId.empty() && candidate.editorId == options.editorId;
        const auto coordinateMatch = candidate.exteriorCoordinates && options.cellX && options.cellY && (*candidate.exteriorCoordinates)[0] == *options.cellX && (*candidate.exteriorCoordinates)[1] == *options.cellY;
        if ((!options.cellFormId && options.editorId.empty() && !options.cellX && !options.cellY) || formMatch || editorMatch || coordinateMatch) { cell = candidate; break; }
    }
    if (!cell) {
        std::cerr << "No matching Skyrim cell was found in " << (resolved ? options.loadOrder : options.plugin) << "\n";
        return 2;
    }
    if (resolved) if (const auto* winning = resolved->FindWinning(cell->id)) {
        std::cout << "Winning CELL: " << winning->winning.plugin << "\nOverride chain:";
        for (const auto& origin : winning->origins) std::cout << " " << origin.plugin;
        std::cout << "\n";
    }

    auto geometry = options.terrainOnly ? navmesh::skyrim::offline::GeometryExtraction{} : navmesh::skyrim::offline::ExtractGeometry(resolved ? options.data : options.plugin.parent_path(), *cell, options.output / ".bsa-cache");
    if (resolved) {
        const auto terrain = navmesh::skyrim::offline::ExtractTerrain(*resolved, *cell);
        const auto sourceOffset = geometry.scene.geometrySources.size();
        const auto vertexOffset = static_cast<std::uint32_t>(geometry.mesh.vertices.size());
        geometry.scene.geometrySources.insert(geometry.scene.geometrySources.end(), terrain.scene.geometrySources.begin(), terrain.scene.geometrySources.end());
        geometry.mesh.vertices.insert(geometry.mesh.vertices.end(), terrain.mesh.vertices.begin(), terrain.mesh.vertices.end());
        for (auto triangle : terrain.mesh.triangles) { for (auto& vertex : triangle.vertices) vertex += vertexOffset; geometry.mesh.triangles.push_back(triangle); }
        for (auto provenance : terrain.scene.triangleProvenance) { provenance.geometrySource += sourceOffset; geometry.scene.triangleProvenance.push_back(provenance); }
        geometry.scene.mesh = geometry.mesh;
        geometry.terrainSupported = terrain.landRecordsDecoded != 0;
        geometry.terrainLandRecords = terrain.landRecordsFound; geometry.terrainLandDecoded = terrain.landRecordsDecoded; geometry.terrainLandMissing = terrain.landRecordsMissing;
        for (const auto& warning : terrain.warnings) std::cerr << warning << "\n";
    }
    navmesh::reproducibility::ExportMetadata metadata{
        .inputPlugin = options.plugin,
        .selectedCell = &*cell,
        .coverage = {
            .references = cell->references.size(), .referencesWithModels = geometry.referencesWithModels,
            .modelsLoaded = geometry.modelsLoaded, .modelsMissing = geometry.modelsMissing,
            .geometryVertices = geometry.mesh.vertices.size(), .geometryTriangles = geometry.mesh.triangles.size(),
            .terrainLandRecords = geometry.terrainLandRecords, .terrainLandDecoded = geometry.terrainLandDecoded, .terrainLandMissing = geometry.terrainLandMissing,
            .terrainSupported = geometry.terrainSupported, .collisionGeometrySupported = geometry.collisionGeometrySupported },
        .warnings = { geometry.terrainLandMissing ? "Terrain coverage is missing for this exterior CELL; no flat substitute was emitted." : "Terrain is not applicable to this interior CELL.", "Collision geometry is not supported by the current extractor." } };
    const auto findings = navmesh::validation::Validate(*cell);
    const auto report = navmesh::cli::ToJson(*cell, findings, metadata);
    const auto reportPath = options.output / "report.json";
    std::ofstream reportStream(reportPath, std::ios::trunc | std::ios::binary);
    reportStream << report;
    const auto geometryPath = options.exportGeometry.empty() ? options.output / "geometry.obj" : options.exportGeometry;
    if (!navmesh::skyrim::offline::WriteGeometryObj(geometryPath, geometry)) std::cerr << "Failed to write geometry OBJ to " << geometryPath << "\n";
    if (!navmesh::reproducibility::WriteSidecar(geometryPath, metadata)) std::cerr << "Failed to write geometry metadata sidecar\n";
    if (!navmesh::skyrim::offline::WriteGeometryJson(options.output / "geometry.json", *cell, geometry, metadata)) std::cerr << "Failed to write geometry JSON\n";

    const auto geometrySummary = navmesh::analysis::AnalyzeGeometry(geometry.mesh);
    const auto meshSummary = navmesh::analysis::AnalyzeNavMesh(cell->navMeshes.empty() ? navmesh::core::NavMesh{} : cell->navMeshes.front());
    const auto analysisConfig = navmesh::analysis::AnalysisConfiguration{ .surfaceSearchRadius = options.surfaceSearchRadius, .maxSupportDistance = options.maxSupportDistance, .maxSlope = options.maxSlope };
    auto analysisReport = cell->navMeshes.empty() ? navmesh::analysis::AnalysisReport{} : navmesh::analysis::AnalyzeNavMeshPolygons(cell->navMeshes.front(), geometry.mesh, analysisConfig);
    AnnotateSupportSources(analysisReport, geometry);

    if (!options.exportAnalysis.empty() && !cell->navMeshes.empty()) {
        WriteAnalysisObj(options.exportAnalysis, cell->navMeshes.front(), geometry.mesh, analysisReport);
        if (!navmesh::reproducibility::WriteSidecar(options.exportAnalysis, metadata)) std::cerr << "Failed to write analysis metadata sidecar\n";
        std::cout << "Exported analysis OBJ to " << options.exportAnalysis << "\n";
    }

    for (std::size_t index = 0; index < cell->navMeshes.size(); ++index) {
        const auto& mesh = cell->navMeshes[index];
        const auto meshPath = options.output / std::format("navmesh_{}.obj", index);
        WriteObj(meshPath, mesh, std::format("NAVM {:08X}", mesh.id));
        if (!navmesh::reproducibility::WriteSidecar(meshPath, metadata)) std::cerr << "Failed to write navmesh metadata sidecar\n";
    }

    std::cout << std::format("Geometry bounds:\n  X: {} -> {}\n  Y: {} -> {}\n  Z: {} -> {}\n", geometrySummary.bounds.min.x, geometrySummary.bounds.max.x, geometrySummary.bounds.min.y, geometrySummary.bounds.max.y, geometrySummary.bounds.min.z, geometrySummary.bounds.max.z);
    std::cout << std::format("NAVM bounds:\n  X: {} -> {}\n  Y: {} -> {}\n  Z: {} -> {}\n", meshSummary.bounds.min.x, meshSummary.bounds.max.x, meshSummary.bounds.min.y, meshSummary.bounds.max.y, meshSummary.bounds.min.z, meshSummary.bounds.max.z);
    std::cout << std::format("Geometry center: ({}, {}, {})\nNAVM center: ({}, {}, {})\nGeometry extent: ({}, {}, {})\nNAVM extent: ({}, {}, {})\n",
        geometrySummary.center.x, geometrySummary.center.y, geometrySummary.center.z,
        meshSummary.center.x, meshSummary.center.y, meshSummary.center.z,
        geometrySummary.extent.x, geometrySummary.extent.y, geometrySummary.extent.z,
        meshSummary.extent.x, meshSummary.extent.y, meshSummary.extent.z);
    std::cout << std::format("Analysis thresholds: surfaceSearchRadius={} maxSupportDistance={} maxSlope={}\n", options.surfaceSearchRadius, options.maxSupportDistance, options.maxSlope);

    WriteAnalysisJson(options.output / "analysis.json", analysisReport, geometrySummary, meshSummary, metadata);
    if (options.diagnostics && !cell->navMeshes.empty()) {
        const auto diagnosticPath = options.output / "navmesh_diagnostics.html";
        WriteDiagnosticHtml(diagnosticPath, *cell, cell->navMeshes.front(), geometry.mesh, geometry, analysisReport);
        if (!navmesh::reproducibility::WriteSidecar(diagnosticPath, metadata)) std::cerr << "Failed to write diagnostic metadata sidecar\n";
        std::cout << "Wrote diagnostic report to " << diagnosticPath << "\n";
    }
    PrintAnalysisSummary(analysisReport);

    std::cout << std::format("Resolved cell: {:08X} editor_id=\"{}\" name=\"{}\" type={}",
        cell->id,
        cell->editorId,
        cell->name,
        cell->isInterior ? "interior" : "exterior");
    if (cell->exteriorCoordinates) {
        std::cout << std::format(" coords=({}, {})", (*cell->exteriorCoordinates)[0], (*cell->exteriorCoordinates)[1]);
    }
    std::cout << "\n";

    if (!cell->navMeshes.empty()) {
        const auto analysis = navmesh::analysis::Analyze(cell->navMeshes.front());
        std::cout << std::format("Cell: {:08X}\nWorldspace: {}\nNAVM vertices: {}\nNAVM polygons: {}\nConnected regions: {}\nIsolated polygons: {}\nDegenerate polygons: {}\nAverage polygon area: {}\n",
            cell->id,
            options.worldspace.empty() ? "<unspecified>" : options.worldspace,
            analysis.vertexCount,
            analysis.polygonCount,
            analysis.connectedComponents,
            analysis.isolatedPolygonCount,
            analysis.degeneratePolygonCount,
            analysis.averagePolygonArea);
    }

    std::cout << "Wrote report to " << reportPath << "\n";
    std::cout << std::format("References: {}\nReferences with models: {}\nModels loaded: {}\nModels missing: {}\nGeometry vertices: {}\nGeometry triangles: {}\nExported geometry: {}\n", cell->references.size(), geometry.referencesWithModels, geometry.modelsLoaded, geometry.modelsMissing, geometry.mesh.vertices.size(), geometry.mesh.triangles.size(), geometryPath.string());
    return 0;
}
