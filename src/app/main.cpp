#include "analysis/navmesh_analysis.h"
#include "app/run.h"
#include "cli/json_report.h"
#include "skyrim/parser/plugin_parser.h"
#include "skyrim/parser/plugin_writer.h"
#include "skyrim/parser/affected_cells.h"
#include <deque>
#include <set>
#include "skyrim/mo2/mo2_importer.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/extraction/terrain_extractor.h"
#include "core/scene/scene_exporter.h"
#include "core/navmesh/candidate.h"
#include "core/navmesh/generator.h"
#include "validation/validation.h"

#include <filesystem>
#include <fstream>
#include <format>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <iostream>
#include <map>
#include <optional>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
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
        stream << std::format("      \"maxSlope\": {},\n", analysisReport.configuration.maxSlope);
        stream << std::format("      \"minimumCoverage\": {},\n", analysisReport.configuration.minimumCoverage);
        stream << std::format("      \"obstructionClearance\": {},\n", analysisReport.configuration.obstructionClearance);
        stream << std::format("      \"ambiguityHeightDelta\": {}\n", analysisReport.configuration.ambiguityHeightDelta);
        stream << "    },\n";
        stream << "    \"summary\": {\n";
        stream << std::format("      \"polygonsAnalyzed\": {},\n", analysisReport.summary.polygonsAnalyzed);
        stream << std::format("      \"supportFound\": {},\n", analysisReport.summary.supportFound);
        stream << std::format("      \"supported\": {},\n", analysisReport.summary.supported);
        stream << std::format("      \"floating\": {},\n", analysisReport.summary.floating);
        stream << std::format("      \"buried\": {},\n", analysisReport.summary.buried);
        stream << std::format("      \"tooSteep\": {},\n", analysisReport.summary.tooSteep);
        stream << std::format("      \"blocked\": {},\n", analysisReport.summary.blocked);
        stream << std::format("      \"outOfCoverage\": {},\n", analysisReport.summary.outOfCoverage);
        stream << std::format("      \"ambiguous\": {},\n", analysisReport.summary.ambiguous);
        stream << std::format("      \"repairCandidates\": {},\n", analysisReport.summary.repairCandidates);
        stream << std::format("      \"topologyFindings\": {}\n", analysisReport.summary.topologyFindings);
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
                stream << std::format("          \"sourceType\": \"{}\",\n", JsonEscape(polygon.support.sourceType));
                stream << std::format("          \"collisionType\": \"{}\",\n", JsonEscape(polygon.support.collisionType));
                stream << std::format("          \"sourceConfidence\": {},\n", polygon.support.sourceConfidence);
                stream << std::format("          \"confidence\": {},\n", polygon.support.confidence);
                stream << std::format("          \"samplesTotal\": {},\n", polygon.support.samplesTotal);
                stream << std::format("          \"samplesCovered\": {},\n", polygon.support.samplesCovered);
                stream << std::format("          \"sampleAgreement\": {},\n", polygon.support.sampleAgreement);
                stream << std::format("          \"sourceNifPath\": \"{}\",\n", JsonEscape(polygon.support.sourceNifPath));
                if (polygon.support.sourceTriangleIndex) stream << std::format("          \"sourceTriangleIndex\": {}\n", *polygon.support.sourceTriangleIndex);
                else stream << "          \"sourceTriangleIndex\": null\n";
            }
            stream << "        },\n";
            stream << std::format("        \"classification\": \"{}\"\n", polygon.classification);
            stream << "      }" << (index + 1 == analysisReport.polygons.size() ? "" : ",") << "\n";
        }
        stream << "    ],\n";
        stream << "    \"topology\": [\n";
        for (std::size_t index{}; index < analysisReport.topology.size(); ++index) {
            const auto& finding = analysisReport.topology[index];
            stream << std::format("      {{\"kind\":\"{}\",\"confidence\":{},\"evidence\":\"{}\",\"polygons\":[", JsonEscape(finding.kind), finding.confidence, JsonEscape(finding.evidence));
            for (std::size_t polygon{}; polygon < finding.polygons.size(); ++polygon) stream << finding.polygons[polygon] << (polygon + 1 == finding.polygons.size() ? "" : ",");
            stream << "]}" << (index + 1 == analysisReport.topology.size() ? "" : ",") << "\n";
        }
        stream << "    ],\n    \"repairCandidates\": [\n";
        for (std::size_t index{}; index < analysisReport.repairCandidates.size(); ++index) {
            const auto& candidate = analysisReport.repairCandidates[index];
            stream << std::format("      {{\"id\":\"{}\",\"kind\":\"{}\",\"confidence\":{},\"disposition\":\"{}\",\"evidence\":\"{}\",\"polygons\":[", JsonEscape(candidate.id), JsonEscape(candidate.kind), candidate.confidence, JsonEscape(candidate.disposition), JsonEscape(candidate.evidence));
            for (std::size_t polygon{}; polygon < candidate.polygons.size(); ++polygon) stream << candidate.polygons[polygon] << (polygon + 1 == candidate.polygons.size() ? "" : ",");
            stream << "]}" << (index + 1 == analysisReport.repairCandidates.size() ? "" : ",") << "\n";
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
            if (polygon.support.triangleIndex < geometry.scene.triangleProvenance.size()) {
                const auto& provenance = geometry.scene.triangleProvenance[polygon.support.triangleIndex];
                if (provenance.geometrySource < geometry.scene.geometrySources.size()) {
                    const auto& source = geometry.scene.geometrySources[provenance.geometrySource];
                    switch (source.sourceType) {
                    case navmesh::core::GeometrySourceType::Terrain: polygon.support.sourceType = "terrain"; break;
                    case navmesh::core::GeometrySourceType::Collision: polygon.support.sourceType = "collision"; break;
                    case navmesh::core::GeometrySourceType::RenderFallback: polygon.support.sourceType = "render_fallback"; break;
                    }
                    polygon.support.collisionType = source.collisionType;
                    polygon.support.sourceConfidence = source.confidence;
                    polygon.support.sourceTriangleIndex = provenance.sourceTriangle;
                }
            }
            for (const auto& reference : geometry.references) {
                if (polygon.support.triangleIndex < reference.meshTriangleOffset || polygon.support.triangleIndex >= reference.meshTriangleOffset + reference.triangles) continue;
                polygon.support.sourceNifPath = reference.modelPath;
                if (!polygon.support.sourceTriangleIndex) polygon.support.sourceTriangleIndex = polygon.support.triangleIndex - reference.meshTriangleOffset;
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
        const std::vector<std::string> classes = { "supported", "floating", "buried", "too_steep", "blocked", "ambiguous", "out_of_coverage" };
        for (const auto& classification : classes) {
            std::size_t limit = classification == "buried" ? 10U : 1U;
            for (const auto& polygon : report.polygons) if (polygon.classification == classification && limit-- > 0) selected.push_back(&polygon);
        }
        const auto missingModels = extraction.modelsMissing > 0 || !extraction.terrainSupported || !extraction.collisionGeometrySupported;
        stream << "<!doctype html><html><head><meta charset=\"utf-8\"><title>NAVM support diagnostics</title><style>body{font:14px system-ui,sans-serif;color:#252422;background:#f1eee7;margin:2rem}h1{font-family:Georgia,serif}table{border-collapse:collapse;background:#fff}th,td{border:1px solid #d8d2c6;padding:.45rem;text-align:left;vertical-align:top}th{background:#283618;color:#fff}tr.buried{background:#fff0e1}svg{width:360px;max-width:100%;height:auto}code{white-space:nowrap}</style></head><body>";
        stream << std::format("<h1>NAVM support diagnostics</h1><p>Cell <code>{:08X}</code> &middot; {} &middot; polygons analyzed: {} &middot; geometry triangles: {}</p>", cell.id, HtmlEscape(cell.editorId.empty() ? cell.name : cell.editorId), report.summary.polygonsAnalyzed, geometry.triangles.size());
        stream << std::format("<p><strong>Counts:</strong> supported {} &middot; floating {} &middot; buried {} &middot; too steep {} &middot; blocked {} &middot; ambiguous {} &middot; out of coverage {}.</p>", report.summary.supported, report.summary.floating, report.summary.buried, report.summary.tooSteep, report.summary.blocked, report.summary.ambiguous, report.summary.outOfCoverage);
        stream << "<h2>Diagnostic assessment</h2><p>Seven interior and edge-aware samples are queried through the spatial index. Collision takes priority over terrain, which takes priority over render fallback. Ambiguous and out-of-coverage states are evidence limitations, never defects.</p>";
        stream << (missingModels ? "<p><strong>Geometry limitation:</strong> terrain or collision geometry is not included, or one or more model NIFs could not be loaded; buried counts may be incomplete or misleading where those surfaces are missing.</p>" : "<p><strong>Geometry coverage:</strong> extracted model geometry is available for the analyzed cell; the examples below can be inspected as actual world-space support triangles.</p>");
        stream << "<p><strong>Interpretation:</strong> a buried result is supported by a concrete selected triangle and a negative height delta. When the triangle has NIF provenance and the two views show the NAVM above that triangle, it is consistent with genuinely buried NAVM. A missing source, visibly displaced triangle, or a systematic offset is evidence for geometry/transform limitations and should be investigated before changing thresholds.</p>";
        std::map<std::string, std::size_t> supportSources, coverage;
        for (const auto& polygon : report.polygons) if (polygon.support.found) ++supportSources[polygon.support.sourceType.empty() ? "unknown" : polygon.support.sourceType];
        for (const auto& entry : extraction.scene.coverage) {
            const auto name = entry.status == navmesh::core::GeometryCoverage::Found ? "found" : entry.status == navmesh::core::GeometryCoverage::Excluded ? "excluded" : entry.status == navmesh::core::GeometryCoverage::Missing ? "missing" : entry.status == navmesh::core::GeometryCoverage::Unreadable ? "unreadable" : "unsupported";
            ++coverage[name];
        }
        stream << "<h2>Source and coverage groups</h2><table><thead><tr><th>Support source</th><th>Classified NAVM polygons linked to source triangles</th></tr></thead><tbody>";
        for (const auto& [name, count] : supportSources) stream << std::format("<tr><td>{}</td><td>{}</td></tr>", HtmlEscape(name), count);
        if (supportSources.empty()) stream << "<tr><td>none</td><td>0</td></tr>";
        stream << "</tbody></table><table><thead><tr><th>Geometry coverage status</th><th>Sources</th></tr></thead><tbody>";
        for (const auto& [name, count] : coverage) stream << std::format("<tr><td>{}</td><td>{}</td></tr>", HtmlEscape(name), count);
        stream << "</tbody></table><p>Every selected row below identifies its world support-triangle index; use <code>analysis.json</code> to join that index to the full <code>geometry.json</code> triangle provenance.</p>";
        stream << "<h2>Selected polygons</h2><table><thead><tr><th>Index</th><th>Class</th><th>Centroid</th><th>Support point</th><th>Delta</th><th>Slope</th><th>Support triangle</th><th>Source</th><th>Views</th></tr></thead><tbody>";
        for (const auto* polygon : selected) {
            std::string triangleText = "none";
            std::string sourceText = "none";
            if (polygon->support.found && polygon->support.triangleIndex < geometry.triangles.size()) {
                const auto& triangle = geometry.triangles[polygon->support.triangleIndex];
                triangleText = std::format("world #{}", polygon->support.triangleIndex);
                if (triangle.vertices[0] < geometry.vertices.size() && triangle.vertices[1] < geometry.vertices.size() && triangle.vertices[2] < geometry.vertices.size()) triangleText += " " + Vec3Text(geometry.vertices[triangle.vertices[0]]) + " / " + Vec3Text(geometry.vertices[triangle.vertices[1]]) + " / " + Vec3Text(geometry.vertices[triangle.vertices[2]]);
                sourceText = HtmlEscape(polygon->support.sourceNifPath.empty() ? "unknown" : polygon->support.sourceNifPath);
                sourceText = HtmlEscape(polygon->support.sourceType) + " (" + std::format("{:.2f}", polygon->support.sourceConfidence) + ") " + sourceText;
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
        std::cout << std::format("Out of coverage:   {}\n", analysis.summary.outOfCoverage);
        std::cout << std::format("Ambiguous:         {}\n", analysis.summary.ambiguous);
        std::cout << std::format("Floating:          {}\n", analysis.summary.floating);
        std::cout << std::format("Buried:            {}\n", analysis.summary.buried);
        std::cout << std::format("Too steep:         {}\n", analysis.summary.tooSteep);
        std::cout << std::format("Blocked:           {}\n", analysis.summary.blocked);
        std::cout << std::format("Repair candidates: {} (report-only)\n", analysis.summary.repairCandidates);
        std::cout << "\nHeight delta:\n";
        std::cout << std::format("  min:    {}\n  max:    {}\n  mean:   {}\n  median: {}\n  p95:    {}\n", analysis.heightDeltaStats.min, analysis.heightDeltaStats.max, analysis.heightDeltaStats.mean, analysis.heightDeltaStats.median, analysis.heightDeltaStats.p95);
        std::cout << "\nSlope:\n";
        std::cout << std::format("  min:    {}\n  max:    {}\n  mean:   {}\n  median: {}\n  p95:    {}\n", analysis.slopeStats.min, analysis.slopeStats.max, analysis.slopeStats.mean, analysis.slopeStats.median, analysis.slopeStats.p95);
    }

    [[nodiscard]] std::vector<navmesh::analysis::TriangleSource> BuildTriangleSources(const navmesh::skyrim::offline::GeometryExtraction& geometry)
    {
        std::vector<navmesh::analysis::TriangleSource> result(geometry.mesh.triangles.size());
        for (std::size_t index{}; index < result.size() && index < geometry.scene.triangleProvenance.size(); ++index) {
            const auto& provenance = geometry.scene.triangleProvenance[index];
            if (provenance.geometrySource >= geometry.scene.geometrySources.size()) continue;
            const auto& source = geometry.scene.geometrySources[provenance.geometrySource];
            using Type = navmesh::core::GeometrySourceType;
            result[index] = {
                .type = source.sourceType == Type::Collision ? navmesh::analysis::SupportSourceType::Collision : source.sourceType == Type::Terrain ? navmesh::analysis::SupportSourceType::Terrain : navmesh::analysis::SupportSourceType::RenderFallback,
                .confidence = source.confidence,
                .id = std::format("{}:{:08X}:{}", source.reference.plugin, source.reference.formId, source.modelPath)
            };
        }
        return result;
    }

    [[nodiscard]] bool EqualsIgnoreCase(const std::string& left, const std::string& right)
    {
        return left.size() == right.size() && std::equal(left.begin(), left.end(), right.begin(), [](unsigned char a, unsigned char b) { return std::tolower(a) == std::tolower(b); });
    }

    [[nodiscard]] std::vector<navmesh::core::SceneLayer> ParseSceneLayers(const std::string& value)
    {
        std::vector<navmesh::core::SceneLayer> result;
        std::stringstream input(value); std::string token;
        while (std::getline(input, token, ',')) {
            std::transform(token.begin(), token.end(), token.begin(), [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (token == "navmesh" || token == "navm") result.push_back(navmesh::core::SceneLayer::ExistingNavmesh);
            else if (token == "terrain") result.push_back(navmesh::core::SceneLayer::Terrain);
            else if (token == "collision") result.push_back(navmesh::core::SceneLayer::Collision);
            else if (token == "render" || token == "render_fallback") result.push_back(navmesh::core::SceneLayer::RenderFallback);
            else if (token == "diagnostics" || token == "markers") result.push_back(navmesh::core::SceneLayer::DiagnosticMarkers);
            else if (token == "candidate") result.push_back(navmesh::core::SceneLayer::CandidateNavmesh);
        }
        return result;
    }

    void AppendGeometry(navmesh::skyrim::offline::GeometryExtraction& destination, navmesh::skyrim::offline::GeometryExtraction&& source)
    {
        const auto sourceOffset = destination.scene.geometrySources.size();
        const auto vertexOffset = static_cast<std::uint32_t>(destination.mesh.vertices.size());
        const auto renderVertexOffset = static_cast<std::uint32_t>(destination.scene.renderFallbackMesh.vertices.size());
        const auto triangleOffset = destination.mesh.triangles.size();
        const auto nodeStart = destination.scene.nodes.size();
        destination.scene.geometrySources.insert(destination.scene.geometrySources.end(), std::make_move_iterator(source.scene.geometrySources.begin()), std::make_move_iterator(source.scene.geometrySources.end()));
        destination.scene.nodes.insert(destination.scene.nodes.end(), std::make_move_iterator(source.scene.nodes.begin()), std::make_move_iterator(source.scene.nodes.end()));
        for (std::size_t index = nodeStart; index < destination.scene.nodes.size(); ++index) if (destination.scene.nodes[index].geometrySource) *destination.scene.nodes[index].geometrySource += sourceOffset;
        destination.mesh.vertices.insert(destination.mesh.vertices.end(), source.mesh.vertices.begin(), source.mesh.vertices.end());
        for (auto triangle : source.mesh.triangles) { for (auto& vertex : triangle.vertices) vertex += vertexOffset; destination.mesh.triangles.push_back(triangle); }
        for (auto provenance : source.scene.triangleProvenance) { provenance.geometrySource += sourceOffset; destination.scene.triangleProvenance.push_back(provenance); }
        destination.scene.renderFallbackMesh.vertices.insert(destination.scene.renderFallbackMesh.vertices.end(), source.scene.renderFallbackMesh.vertices.begin(), source.scene.renderFallbackMesh.vertices.end());
        for (auto triangle : source.scene.renderFallbackMesh.triangles) { for (auto& vertex : triangle.vertices) vertex += renderVertexOffset; destination.scene.renderFallbackMesh.triangles.push_back(triangle); }
        for (auto provenance : source.scene.renderFallbackTriangleProvenance) { provenance.geometrySource += sourceOffset; destination.scene.renderFallbackTriangleProvenance.push_back(provenance); }
        destination.scene.coverage.insert(destination.scene.coverage.end(), std::make_move_iterator(source.scene.coverage.begin()), std::make_move_iterator(source.scene.coverage.end()));
        for (auto reference : source.references) { reference.meshVertexOffset += vertexOffset; reference.meshTriangleOffset += triangleOffset; destination.references.push_back(std::move(reference)); }
        destination.referencesWithModels += source.referencesWithModels; destination.modelsLoaded += source.modelsLoaded; destination.modelsMissing += source.modelsMissing; destination.invalidVertices += source.invalidVertices; destination.invalidIndices += source.invalidIndices; destination.modelsExcluded += source.modelsExcluded; destination.modelsUnreadable += source.modelsUnreadable; destination.modelsUnsupported += source.modelsUnsupported; destination.collisionModelsLoaded += source.collisionModelsLoaded; destination.collisionTriangles += source.collisionTriangles; destination.renderFallbackModels += source.renderFallbackModels; destination.renderFallbackTriangles += source.renderFallbackTriangles; destination.terrainLandRecords += source.terrainLandRecords; destination.terrainLandDecoded += source.terrainLandDecoded; destination.terrainLandMissing += source.terrainLandMissing;
        destination.terrainSupported = destination.terrainSupported || source.terrainSupported; destination.collisionGeometrySupported = destination.collisionGeometrySupported || source.collisionGeometrySupported; destination.scene.mesh = destination.mesh;
    }

    void CullGeometryToBounds(navmesh::skyrim::offline::GeometryExtraction& geometry, const navmesh::core::AABB& bounds)
    {
        navmesh::core::Mesh selected;
        std::vector<navmesh::core::TriangleProvenance> provenance;
        std::vector<std::pair<std::size_t, std::size_t>> oldRanges; oldRanges.reserve(geometry.references.size());
        for (const auto& reference : geometry.references) oldRanges.push_back({ reference.meshTriangleOffset, reference.triangles });
        for (auto& reference : geometry.references) { reference.meshVertexOffset = 0; reference.meshTriangleOffset = 0; reference.vertices = 0; reference.triangles = 0; }
        std::size_t referenceIndex{};
        for (std::size_t triangleIndex{}; triangleIndex < geometry.mesh.triangles.size(); ++triangleIndex) {
            const auto& triangle = geometry.mesh.triangles[triangleIndex]; navmesh::core::AABB triangleBounds;
            bool valid = true; for (const auto vertex : triangle.vertices) { if (vertex >= geometry.mesh.vertices.size()) { valid = false; break; } triangleBounds.Expand(geometry.mesh.vertices[vertex]); }
            if (!valid || !triangleBounds.Intersects(bounds)) continue;
            while (referenceIndex < oldRanges.size() && triangleIndex >= oldRanges[referenceIndex].first + oldRanges[referenceIndex].second) ++referenceIndex;
            const auto base = static_cast<std::uint32_t>(selected.vertices.size()); for (const auto vertex : triangle.vertices) selected.vertices.push_back(geometry.mesh.vertices[vertex]); selected.triangles.push_back({ { base, base + 1, base + 2 } });
            if (triangleIndex < geometry.scene.triangleProvenance.size()) provenance.push_back(geometry.scene.triangleProvenance[triangleIndex]);
            if (referenceIndex < geometry.references.size() && triangleIndex >= oldRanges[referenceIndex].first) {
                auto& reference = geometry.references[referenceIndex]; if (reference.triangles == 0) { reference.meshVertexOffset = base; reference.meshTriangleOffset = selected.triangles.size() - 1; } reference.vertices += 3; ++reference.triangles;
            }
        }
        geometry.mesh = std::move(selected); geometry.scene.mesh = geometry.mesh; geometry.scene.triangleProvenance = std::move(provenance);
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

namespace
{
    /// Generate one CELL at a time, retaining only candidate evidence and a bounded
    /// geometry cache. Full scene meshes never accumulate across the load order.
    int RunBatch(const navmesh::app::Options& options, const navmesh::skyrim::offline::ResolvedLoadOrder& resolved,
        const navmesh::skyrim::offline::ModelAssetSources* assets, const std::vector<std::filesystem::path>& paths,
        const navmesh::app::ProgressCallback& progress, const navmesh::app::CancellationCallback& cancelled)
    {
        using namespace navmesh;
        const auto stop = [&] { return cancelled && cancelled(); };
        const skyrim::offline::CellImpactIndex index(resolved);
        std::set<std::string> changedModels;
        bool archiveModelsChanged{};
        if (assets) {
            const auto inside = [](const std::filesystem::path& path, const std::filesystem::path& root) {
                const auto relative = path.lexically_relative(root);
                return !relative.empty() && *relative.begin() != "..";
            };
            if (options.rebuildScope == app::RebuildScope::LoadOrder) {
                for (const auto& [logical,physical] : assets->looseModels) if (!inside(physical,options.data)) changedModels.insert(logical);
                archiveModelsChanged = std::any_of(assets->archives.begin(),assets->archives.end(),[&](const auto& archive) { return !inside(archive,options.data); });
            } else {
                const auto selected = std::find_if(paths.begin(),paths.end(),[&](const auto& path) { return EqualsIgnoreCase(path.filename().string(),options.affectedPlugin); });
                if (selected != paths.end() && !inside(*selected,options.data))
                    for (const auto& [logical,physical] : assets->looseModels) if (inside(physical,selected->parent_path())) changedModels.insert(logical);
                const auto pluginStem = std::filesystem::path(options.affectedPlugin).stem().string();
                archiveModelsChanged = std::any_of(assets->archives.begin(),assets->archives.end(),[&](const auto& archive) {
                    const auto stem = archive.stem().string();
                    return EqualsIgnoreCase(stem,pluginStem) || (stem.size() > pluginStem.size() && stem[pluginStem.size()] == ' '
                        && EqualsIgnoreCase(stem.substr(0,pluginStem.size()),pluginStem));
                });
            }
        }
        const auto targets = index.AffectedCells(options.rebuildScope == app::RebuildScope::Plugin ? options.affectedPlugin : "",
            options.neighboringCellRadius,changedModels,archiveModelsChanged);
        struct Result {
            const core::Cell* cell{};
            core::CandidateNavMesh candidate;
            core::Scene evidence;
            std::string metadata, status;
        };
        std::vector<Result> results;
        results.reserve(targets.size());
        for (const auto* target : targets) results.push_back({ .cell=target, .status="pending" });
        std::map<std::uint32_t, skyrim::offline::GeometryExtraction> cache;
        std::deque<std::uint32_t> cacheOrder;
        std::size_t cacheTriangles{}, cacheHits{}, extractedCells{}, originalPolygons{}, generatedPolygons{};
        const auto batchMetadata = reproducibility::ToJson(reproducibility::ExportMetadata{
            .inputPlugin=options.affectedPlugin,
            .warnings={"Load-order scope treats plugins after the first active baseline plugin as changes.",
                "Skipped targets and writer limitations are recorded in the batch report and docs/batch-rebuilding.md."}},"    ");
        const auto summaryPath = options.output / "batch-report.json";
        const auto summary = [&](const std::string& state, const std::string& error = "") {
            std::ofstream out(summaryPath, std::ios::trunc);
            out << "{\n  \"metadata\": " << batchMetadata << ",\n";
            out << std::format("  \"scope\":\"{}\",\"plugin\":\"{}\",\"status\":\"{}\",\"error\":\"{}\",\n",
                options.rebuildScope == app::RebuildScope::Plugin ? "plugin" : "load_order", JsonEscape(options.affectedPlugin), state, JsonEscape(error));
            out << std::format("  \"selected_cells\":{},\"geometry_cells_extracted\":{},\"geometry_cache_hits\":{},\n  \"original_polygons\":{},\"generated_polygons\":{},\n  \"cells\":[\n",
                targets.size(), extractedCells, cacheHits, originalPolygons, generatedPolygons);
            for (std::size_t i{}; i < results.size(); ++i) out << std::format("    {{\"form_id\":\"{:08X}\",\"status\":\"{}\",\"polygons\":{}}}{}\n",
                results[i].cell->id, JsonEscape(results[i].status), results[i].candidate.mesh.polygons.size(), i+1 == results.size() ? "" : ",");
            out << "  ]\n}\n";
            return out.good();
        };
        const auto exports = [&] {
            for (const auto& result : results) if (!result.metadata.empty()) {
                const auto directory = options.output/"cells"/std::format("{:08X}",result.cell->id);
                std::filesystem::create_directories(directory);
                if (!core::WriteCandidateJson(directory/"candidate-navm.json",result.candidate,result.evidence,result.metadata)
                    || !core::WriteCandidateObj(directory/"candidate-navm.obj",result.candidate)) return false;
                std::ofstream sidecar(directory/"candidate-navm.obj.metadata.json",std::ios::trunc);
                sidecar << "{\n  \"metadata\": " << result.metadata << "\n}\n";
                if (!sidecar) return false;
            }
            return true;
        };
        const auto fail = [&](const std::string& error) { exports(); summary("failed",error); std::cerr << error << '\n'; return 2; };
        if (!summary("running")) return fail("Cannot write batch-report.json");
        for (std::size_t targetIndex{}; targetIndex < targets.size(); ++targetIndex) {
            if (stop()) { summary("cancelled"); return 3; }
            const auto& cell = *targets[targetIndex];
            auto& result = results[targetIndex];
            for (const auto& mesh : cell.navMeshes) originalPolygons += mesh.polygons.size();
            if (cell.navMeshes.empty()) { result.status = "skipped_no_existing_navm"; continue; }
            if (const auto* record = resolved.FindWinning(cell.id); record && record->raw && (record->raw->flags & 0x20U)) {
                result.status = "skipped_deleted_cell"; continue;
            }
            const auto update = [&](std::string_view status) {
                if (progress) progress(30+static_cast<int>(60*targetIndex/std::max<std::size_t>(1,targets.size())),
                    std::format("CELL {}/{} {:08X}: {}",targetIndex+1,targets.size(),cell.id,status));
            };
            update("Extracting neighboring geometry");
            skyrim::offline::GeometryExtraction geometry;
            for (const auto* neighbor : index.GeometryNeighbors(cell,std::max(1,options.neighboringCellRadius))) {
                auto found = cache.find(neighbor->id);
                if (found == cache.end()) {
                    auto geometryCell = index.GeometryCell(*neighbor);
                    auto extracted = options.terrainOnly ? skyrim::offline::GeometryExtraction{} : skyrim::offline::ExtractGeometry(
                        options.data,geometryCell,options.output/".bsa-cache",
                        [&](std::size_t done,std::size_t total) { update(std::format("Geometry {:08X}: reference {}/{}",neighbor->id,done,total)); },stop,assets);
                    if (stop()) { result.status = "cancelled"; summary("cancelled"); return 3; }
                    auto terrain = skyrim::offline::ExtractTerrain(resolved,*neighbor);
                    skyrim::offline::GeometryExtraction terrainGeometry;
                    terrainGeometry.scene = std::move(terrain.scene); terrainGeometry.mesh = std::move(terrain.mesh);
                    terrainGeometry.terrainSupported = terrain.landRecordsDecoded != 0;
                    terrainGeometry.terrainLandRecords = terrain.landRecordsFound;
                    terrainGeometry.terrainLandDecoded = terrain.landRecordsDecoded;
                    terrainGeometry.terrainLandMissing = terrain.landRecordsMissing;
                    AppendGeometry(extracted,std::move(terrainGeometry));
                    cacheTriangles += extracted.mesh.triangles.size()+extracted.scene.renderFallbackMesh.triangles.size();
                    found = cache.emplace(neighbor->id,std::move(extracted)).first;
                    cacheOrder.push_back(neighbor->id); ++extractedCells;
                } else {
                    ++cacheHits;
                    std::erase(cacheOrder,neighbor->id); cacheOrder.push_back(neighbor->id);
                }
                AppendGeometry(geometry,skyrim::offline::GeometryExtraction(found->second));
                // LRU is bounded both by entry count and triangle volume. A single
                // oversized entry may be used for this target but is not retained.
                while (cache.size() > 16 || cacheTriangles > 2000000) {
                    const auto id = cacheOrder.front(); cacheOrder.pop_front();
                    const auto& entry = cache.at(id);
                    cacheTriangles -= entry.mesh.triangles.size()+entry.scene.renderFallbackMesh.triangles.size(); cache.erase(id);
                }
            }
            std::optional<core::AABB> bounds;
            if (cell.exteriorCoordinates) {
                const auto [x,y] = *cell.exteriorCoordinates;
                bounds = core::AABB{ .min = {x*4096.0F,y*4096.0F,std::numeric_limits<float>::lowest()},
                    .max = {(static_cast<float>(x)+1)*4096.0F,(static_cast<float>(y)+1)*4096.0F,std::numeric_limits<float>::max()} };
            }
            std::vector<core::CandidateExit> exits;
            // Physical bucketing includes DOORs from persistent worldspace parents.
            for (const auto& reference : index.GeometryCell(cell).references)
                if (reference.baseRecordType == "DOOR" && !reference.deleted && !reference.initiallyDisabled)
                    exits.push_back({ .referenceId = reference.id, .position = reference.position });
            update("Generating NAVM");
            try {
                result.candidate = core::RecastCandidateGenerator{}.Generate(geometry.scene,core::NavigationProfile{},bounds,std::move(exits),options.partitioningAlgorithm);
                if (bounds) {
                    std::vector<core::NavMesh> adjacent;
                    for (const auto* neighbor : index.Neighbors(cell,1)) {
                        if (!neighbor->exteriorCoordinates || neighbor->id == cell.id) continue;
                        const auto [x,y] = *neighbor->exteriorCoordinates;
                        if (std::abs(static_cast<std::int64_t>(x)-(*cell.exteriorCoordinates)[0])
                            +std::abs(static_cast<std::int64_t>(y)-(*cell.exteriorCoordinates)[1]) != 1) continue;
                        adjacent.insert(adjacent.end(),neighbor->navMeshes.begin(),neighbor->navMeshes.end());
                    }
                    (void)core::StitchCandidateBorders(result.candidate,*bounds,adjacent);
                }
            } catch (const std::exception& error) { result.status = "failed"; return fail(std::format("CELL {:08X}: {}",cell.id,error.what())); }
            if (!result.candidate.topology.valid) { result.status = "invalid_topology"; return fail("Candidate topology validation failed"); }
            result.status = result.candidate.mesh.polygons.empty() ? "skipped_empty_candidate" : "generated";
            generatedPolygons += result.candidate.mesh.polygons.size();
            reproducibility::ExportMetadata metadata{ .inputPlugin=options.affectedPlugin, .selectedCell = &cell,
                .coverage = { .references = cell.references.size(), .geometryVertices = geometry.mesh.vertices.size(), .geometryTriangles = geometry.mesh.triangles.size(),
                    .terrainSupported = geometry.terrainSupported, .collisionGeometrySupported = geometry.collisionModelsLoaded != 0 },
                .warnings = { "Batch candidates use neighboring geometry but remain clipped to their target CELL.",
                    "Cells without existing NAVM or a nonempty supported candidate are skipped; see batch-report.json." } };
            result.metadata = reproducibility::ToJson(metadata,"    ");
            // Compact source evidence after generation; retain no scene mesh and
            // only provenance entries actually used by candidate polygons.
            result.evidence.geometrySources = std::move(geometry.scene.geometrySources);
            std::map<std::size_t,std::size_t> remap;
            const auto source = [&](std::size_t old) {
                const auto [it,added] = remap.emplace(old,result.evidence.triangleProvenance.size());
                if (added) result.evidence.triangleProvenance.push_back(geometry.scene.triangleProvenance.at(old));
                return it->second;
            };
            for (auto& value : result.candidate.polygonSourceTriangles) value = source(value);
            for (auto& values : result.candidate.polygonContributingTriangles) for (auto& value : values) value = source(value);
            for (auto& region : result.candidate.regions) for (auto& value : region.sourceTriangles) value = source(value);
            if (stop()) { result.status = "cancelled"; summary("cancelled"); return 3; }
        }
        // Replace authored neighbor triangle identities with reciprocal generated
        // edges. Endpoint equality is required; incompatible partitions fail closed.
        std::map<std::uint32_t,Result*> generatedByCell;
        for (auto& result : results) if (result.status == "generated") generatedByCell.emplace(result.cell->id,&result);
        const auto near = [](core::Vec3 a,core::Vec3 b) { return std::abs(a.x-b.x)<=1 && std::abs(a.y-b.y)<=1 && std::abs(a.z-b.z)<=1; };
        for (auto& result : results) for (auto& link : result.candidate.borderLinks) {
            const auto* record = resolved.FindWinning(link.neighborNavmeshId);
            if (!record || !record->cellFormId) return fail("Border target has no CELL ownership");
            const auto other = generatedByCell.find(*record->cellFormId);
            if (other == generatedByCell.end()) continue;
            const auto& face = result.candidate.mesh.polygons.at(link.polygon);
            const auto a = result.candidate.mesh.vertices.at(face.vertices[link.edge]);
            const auto b = result.candidate.mesh.vertices.at(face.vertices[(link.edge+1)%3]);
            bool matched{};
            for (const auto& reverse : other->second->candidate.borderLinks) {
                const auto& target = other->second->candidate.mesh.polygons.at(reverse.polygon);
                const auto c = other->second->candidate.mesh.vertices.at(target.vertices[reverse.edge]);
                const auto d = other->second->candidate.mesh.vertices.at(target.vertices[(reverse.edge+1)%3]);
                if (!near(a,d) || !near(b,c)) continue;
                const auto& meshes = other->second->cell->navMeshes;
                link.neighborNavmeshId = std::max_element(meshes.begin(),meshes.end(),[](const auto& x,const auto& y) { return x.polygons.size()<y.polygons.size(); })->id;
                link.neighborPolygon = reverse.polygon; link.neighborEdge = reverse.edge; matched = true; break;
            }
            if (!matched) return fail(std::format("Generated border partitions do not match between CELL {:08X} and {:08X}",result.cell->id,other->second->cell->id));
        }
        if (!exports()) return fail("Cannot write batch candidate exports");
        if (stop()) { summary("cancelled"); return 3; }
        if (options.generatePlugin && !generatedByCell.empty()) {
            std::vector<skyrim::offline::NavmeshReplacement> replacements;
            for (const auto& result : results) if (result.status == "generated") replacements.push_back({result.cell,&result.candidate});
            std::filesystem::path written; std::string error;
            if (progress) progress(92,"Writing batch NAVM override plugin");
            if (!skyrim::offline::WriteNavmeshOverrides(options.output,paths,resolved,replacements,written,error)) return fail(error);
            std::cout << "Generated batch NAVM override plugin: " << written.string() << '\n';
        }
        if (!summary("complete")) return fail("Cannot finalize batch-report.json");
        const auto status = std::format("Affected cells: {}; rebuilt: {}; skipped: {}; original polygons: {}; generated polygons: {}",
            targets.size(),generatedByCell.size(),targets.size()-generatedByCell.size(),originalPolygons,generatedPolygons);
        std::cout << status << '\n';
        if (progress) progress(100,status);
        return 0;
    }
}

int navmesh::app::Run(const Options& input, const ProgressCallback& progress, const CancellationCallback& cancelled)
{
    auto options = input;
    const auto update = [&](int percent, std::string_view status) { if (progress) progress(percent, status); };
    const auto wasCancelled = [&] { return cancelled && cancelled(); };
    update(0, "Validating inputs");
    if (options.mo2.empty() && options.plugin.empty() && options.loadOrder.empty()) {
        std::cerr << "Usage: navmesh-offline --mo2 <instance-or-portable-root> --profile <existing-profile> [--mods-dir <moved-mods-root>] [--list-cells] [--cell-formid <hex> | --rebuild-plugin <active filename> | --rebuild-load-order] [--generate-plugin] --output <dir>\nDeveloper/test override: --data <Data> --load-order <plugins.txt>.\n";
        return 1;
    }
    if (!options.mo2.empty() && options.profile.empty()) { std::cerr << "--mo2 requires --profile naming an existing MO2 profile.\n"; return 1; }
    if (options.generatePlugin && (options.listCells || (options.mo2.empty() && options.loadOrder.empty()))) {
        std::cerr << "Plugin generation requires a resolved MO2/load-order input and a rebuild selection.\n";
        return 1;
    }
    if (options.rebuildScope != RebuildScope::Cell) {
        if (options.mo2.empty() && options.loadOrder.empty()) { std::cerr << "Batch rebuilding requires MO2 or --load-order input.\n"; return 1; }
        if (options.listCells || !options.cell.empty() || options.cellFormId || !options.editorId.empty() || options.cellX || options.cellY
            || !options.exportGeometry.empty() || !options.exportAnalysis.empty() || !options.exportScene.empty() || options.sceneBounds) {
            std::cerr << "Batch rebuilding cannot combine cell selectors, listing, scene bounds, or custom export paths.\n"; return 1;
        }
        if (options.rebuildScope == RebuildScope::Plugin && options.affectedPlugin.empty()) { std::cerr << "Select an active plugin filename.\n"; return 1; }
        options.generateCandidate = true;
    }
    if (options.neighboringCellRadius < 0) { std::cerr << "Neighboring-cell radius cannot be negative.\n"; return 1; }
    if (options.generatePlugin) options.generateCandidate = true;

    std::optional<navmesh::skyrim::offline::ResolvedLoadOrder> resolved;
    std::optional<navmesh::skyrim::offline::Mo2ProfileInput> mo2Input;
    navmesh::skyrim::offline::ModelAssetSources modelAssets;
    if (!options.mo2.empty()) {
        std::filesystem::create_directories(options.output);
        update(5, "Reading MO2 profile");
        std::cerr << "Importing MO2 profile and virtual-file winners...\n";
        if (!options.modsDirectory.empty()) std::cerr << "Using --mods-dir override: " << options.modsDirectory.string() << "\n";
        try {
            mo2Input = navmesh::skyrim::offline::ImportMo2Profile(options.mo2, options.profile,
                options.modsDirectory.empty() ? std::nullopt : std::optional(options.modsDirectory), options.output / ".mo2-cache");
        } catch (const std::exception& error) {
            mo2Input = navmesh::skyrim::offline::Mo2ProfileInput{ .instanceRoot = options.mo2, .profile = options.profile };
            mo2Input->diagnostics.push_back({ navmesh::skyrim::offline::DiagnosticKind::InvalidPlugin, options.mo2.string(), std::string("MO2 import failed: ") + error.what() });
        }
        if (!navmesh::skyrim::offline::WriteInputReport(options.output / "input-report.json", *mo2Input)) std::cerr << "Failed to write input-report.json.\n";
        update(15, mo2Input->looseAssetCacheUsed ? "MO2 loose-asset cache loaded" : "MO2 loose-asset cache created");
        std::cerr << (mo2Input->looseAssetCacheUsed ? "Using" : "Created") << " MO2 loose-asset cache in " << (options.output / ".mo2-cache").string() << "\n";
        std::cerr << "Effective MO2 mods directory: " << mo2Input->modsDirectory.string() << "\n";
        for (const auto& diagnostic : mo2Input->diagnostics) std::cerr << diagnostic.plugin << ": " << diagnostic.message << "\n";
        if (mo2Input->pluginPaths.empty()) { std::cerr << "MO2 import found no usable active plugin paths; see input-report.json.\n"; return 2; }
        for (const auto& file : mo2Input->looseAssetWinners) {
            if (std::filesystem::path(file.logicalPath).extension() == ".nif") modelAssets.looseModels.emplace(file.logicalPath, file.physicalPath);
        }
        modelAssets.archives = std::move(mo2Input->archivePaths);
        mo2Input->looseAssetWinners.clear(); mo2Input->looseAssetWinners.shrink_to_fit(); mo2Input->enabledMods.clear(); mo2Input->enabledMods.shrink_to_fit();
        options.data = mo2Input->gameData;
        update(20, "Resolving active plugin load order");
        std::cerr << "Resolving worldspace and cell records from " << mo2Input->pluginPaths.size() << " active plugins...\n";
        try {
            resolved = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = options.data, .plugins = mo2Input->pluginPaths, .indexReferencesAndNavmeshes = !options.listCells,
                .progress = [&](std::size_t completed, std::size_t total, const std::filesystem::path& plugin) {
                    const auto percent = total == 0 ? 30 : 20 + static_cast<int>((10 * completed) / total);
                    update(percent, completed == total ? "Load order records resolved" : std::format("Resolving plugin {} of {}: {}", completed + 1, total, plugin.filename().string()));
                } });
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
    update(30, resolved ? "Load order resolved" : "Plugin input ready");
    if (wasCancelled()) return 3;

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
        update(100, cells.empty() ? "No cells found" : "Cell list complete");
        return cells.empty() ? 2 : 0;
    }

    std::filesystem::create_directories(options.output);
    if (resolved) WriteLoadOrderJson(options.output / "load-order.json", *resolved);
    if (options.rebuildScope != RebuildScope::Cell) {
        if (resolved->plugins.empty()) { std::cerr << "The load order contains no active plugin inputs.\n"; return 2; }
        std::vector<std::filesystem::path> paths;
        if (mo2Input) paths = mo2Input->pluginPaths;
        else for (auto path : navmesh::skyrim::offline::ReadLoadOrderManifest(options.loadOrder))
            paths.push_back(path.is_absolute() ? path : options.data / path);
        return RunBatch(options, *resolved, mo2Input ? &modelAssets : nullptr, paths, progress, cancelled);
    }
    auto cell = resolved ? std::optional<navmesh::core::Cell>{} : navmesh::skyrim::offline::LoadCell(options.plugin, options.cell, options.worldspace, options.cellX, options.cellY, options.cellFormId, options.editorId);
    if (resolved) for (const auto& candidate : resolved->cells) {
        const auto formMatch = options.cellFormId && candidate.id == *options.cellFormId;
        const auto editorMatch = !options.editorId.empty() && EqualsIgnoreCase(candidate.editorId, options.editorId);
        const auto coordinateMatch = candidate.exteriorCoordinates && options.cellX && options.cellY && (*candidate.exteriorCoordinates)[0] == *options.cellX && (*candidate.exteriorCoordinates)[1] == *options.cellY;
        if ((!options.cellFormId && options.editorId.empty() && !options.cellX && !options.cellY) || formMatch || editorMatch || coordinateMatch) { cell = candidate; break; }
    }
    if (!cell) {
        if (resolved) {
            std::cerr << "No matching Skyrim CELL was found in the resolved MO2 load order";
            if (!options.editorId.empty()) std::cerr << " for editor ID '" << options.editorId << "'";
            else if (options.cellFormId) std::cerr << " for form ID " << std::format("{:08X}", *options.cellFormId);
            else if (options.cellX && options.cellY) std::cerr << " at exterior coordinates " << *options.cellX << "," << *options.cellY;
            std::cerr << ". Use List cells to find an exact CELL editor ID or form ID.\n";
        } else std::cerr << "No matching Skyrim cell was found in " << options.plugin << "\n";
        return 2;
    }
    update(45, "Cell resolved");
    if (wasCancelled()) return 3;
    if (resolved) if (const auto* winning = resolved->FindWinning(cell->id)) {
        std::cout << "Winning CELL: " << winning->winning.plugin << "\nOverride chain:";
        for (const auto& origin : winning->origins) std::cout << " " << origin.plugin;
        std::cout << "\n";
    }

    std::vector<navmesh::core::Cell> sceneCells{ *cell };
    if (resolved) {
        const navmesh::skyrim::offline::CellImpactIndex index(*resolved);
        sceneCells.clear();
        for (const auto* source : index.GeometryNeighbors(*cell, options.generateCandidate ? std::max(1, options.neighboringCellRadius) : options.neighboringCellRadius)) {
            auto geometryCell = index.GeometryCell(*source);
            geometryCell.navMeshes = source->navMeshes;
            sceneCells.push_back(std::move(geometryCell));
        }
    }
    navmesh::skyrim::offline::GeometryExtraction geometry;
    for (std::size_t cellIndex{}; cellIndex < sceneCells.size(); ++cellIndex) {
        auto extracted = options.terrainOnly ? navmesh::skyrim::offline::GeometryExtraction{} : navmesh::skyrim::offline::ExtractGeometry(
            resolved ? options.data : options.plugin.parent_path(), sceneCells[cellIndex], options.output / ".bsa-cache",
            [&](std::size_t completed, std::size_t total) { const auto percent = total == 0 ? 65 : 45 + static_cast<int>((20 * completed) / total); update(percent, std::format("Extracting scene cell {}/{}: reference {} of {}", cellIndex + 1, sceneCells.size(), completed, total)); }, wasCancelled, mo2Input ? &modelAssets : nullptr);
        if (resolved) {
            auto terrain = navmesh::skyrim::offline::ExtractTerrain(*resolved, sceneCells[cellIndex]);
            navmesh::skyrim::offline::GeometryExtraction terrainGeometry;
            terrainGeometry.scene = std::move(terrain.scene); terrainGeometry.mesh = std::move(terrain.mesh); terrainGeometry.scene.mesh = terrainGeometry.mesh;
            terrainGeometry.terrainSupported = terrain.landRecordsDecoded != 0; terrainGeometry.terrainLandRecords = terrain.landRecordsFound; terrainGeometry.terrainLandDecoded = terrain.landRecordsDecoded; terrainGeometry.terrainLandMissing = terrain.landRecordsMissing;
            for (const auto& warning : terrain.warnings) std::cerr << warning << "\n";
            AppendGeometry(extracted, std::move(terrainGeometry));
        }
        if (options.sceneBounds) {
            const auto& bounds = *options.sceneBounds;
            if (bounds[0] > bounds[2] || bounds[1] > bounds[3]) { std::cerr << "--scene-bounds requires minX minY maxX maxY.\n"; return 1; }
            CullGeometryToBounds(extracted, { .min = { bounds[0], bounds[1], std::numeric_limits<float>::lowest() }, .max = { bounds[2], bounds[3], std::numeric_limits<float>::max() } });
        }
        AppendGeometry(geometry, std::move(extracted));
        if (wasCancelled()) { update(0, "Cancelled"); return 3; }
    }
    geometry.collisionGeometrySupported = geometry.collisionModelsLoaded != 0;
    update(65, "Geometry extracted");
    if (wasCancelled()) return 3;
    navmesh::reproducibility::ExportMetadata metadata{
        .inputPlugin = options.plugin,
        .selectedCell = &*cell,
        .coverage = {
            .references = cell->references.size(), .referencesWithModels = geometry.referencesWithModels,
            .modelsLoaded = geometry.modelsLoaded, .modelsMissing = geometry.modelsMissing,
            .geometryVertices = geometry.mesh.vertices.size(), .geometryTriangles = geometry.mesh.triangles.size(),
            .terrainLandRecords = geometry.terrainLandRecords, .terrainLandDecoded = geometry.terrainLandDecoded, .terrainLandMissing = geometry.terrainLandMissing,
            .terrainSupported = geometry.terrainSupported, .collisionGeometrySupported = geometry.collisionGeometrySupported },
        .warnings = { geometry.terrainLandMissing ? "Terrain coverage is missing for this exterior CELL; no flat substitute was emitted." : "Terrain is not applicable to this interior CELL.", geometry.collisionGeometrySupported ? "Collision support is limited to reachable bhkPackedNiTriStripsData; unsupported Havok shapes are not approximated." : "No supported packed Havok collision was found; any render triangles are low-confidence fallbacks." } };
    const auto findings = navmesh::validation::Validate(*cell);
    const auto report = navmesh::cli::ToJson(*cell, findings, metadata);
    const auto reportPath = options.output / "report.json";
    std::ofstream reportStream(reportPath, std::ios::trunc | std::ios::binary);
    reportStream << report;
    const auto geometryPath = options.exportGeometry.empty() ? options.output / "geometry.obj" : options.exportGeometry;
    if (!navmesh::skyrim::offline::WriteGeometryObj(geometryPath, geometry)) std::cerr << "Failed to write geometry OBJ to " << geometryPath << "\n";
    if (!navmesh::reproducibility::WriteSidecar(geometryPath, metadata)) std::cerr << "Failed to write geometry metadata sidecar\n";
    if (!navmesh::skyrim::offline::WriteGeometryJson(options.output / "geometry.json", *cell, geometry, metadata)) std::cerr << "Failed to write geometry JSON\n";
    update(78, "Geometry exports written");
    if (wasCancelled()) return 3;

    update(79, "Analyzing existing NAVM support");
    const auto geometrySummary = navmesh::analysis::AnalyzeGeometry(geometry.mesh);
    const auto meshSummary = navmesh::analysis::AnalyzeNavMesh(cell->navMeshes.empty() ? navmesh::core::NavMesh{} : cell->navMeshes.front());
    auto analysisConfig = navmesh::analysis::AnalysisConfiguration{ .surfaceSearchRadius = options.surfaceSearchRadius, .maxSupportDistance = options.maxSupportDistance, .maxSlope = options.maxSlope };
    if (cell->exteriorCoordinates) {
        const auto [x, y] = *cell->exteriorCoordinates;
        analysisConfig.cellBounds = navmesh::core::AABB{ .min = { x * 4096.0F, y * 4096.0F, std::numeric_limits<float>::lowest() }, .max = { (x + 1) * 4096.0F, (y + 1) * 4096.0F, std::numeric_limits<float>::max() } };
    }
    const auto triangleSources = BuildTriangleSources(geometry);
    auto analysisReport = cell->navMeshes.empty() ? navmesh::analysis::AnalysisReport{} : navmesh::analysis::AnalyzeNavMeshPolygons(cell->navMeshes.front(), geometry.mesh, triangleSources, analysisConfig);
    update(82, "Existing NAVM analysis complete");
    if (wasCancelled()) return 3;
    AnnotateSupportSources(analysisReport, geometry);
    std::vector<navmesh::core::NavMesh> sceneNavmeshes;
    for (const auto& sceneCell : sceneCells) sceneNavmeshes.insert(sceneNavmeshes.end(), sceneCell.navMeshes.begin(), sceneCell.navMeshes.end());
    std::vector<navmesh::core::DiagnosticMarker> sceneMarkers;
    sceneMarkers.reserve(analysisReport.polygons.size());
    for (const auto& polygon : analysisReport.polygons) sceneMarkers.push_back({ polygon.centroid, polygon.classification, polygon.index, polygon.support.found ? std::optional<std::size_t>{ polygon.support.triangleIndex } : std::nullopt, cell->navMeshes.front().id });
    std::optional<navmesh::core::CandidateNavMesh> candidate;
    if (options.generateCandidate) {
        std::vector<navmesh::core::CandidateExit> exits;
        std::optional<navmesh::core::AABB> candidateBounds;
        for (const auto& sceneCell : sceneCells) {
            for (const auto& reference : sceneCell.references)
                if (reference.recordType == "REFR" && reference.baseRecordType == "DOOR"
                    && !reference.deleted && !reference.initiallyDisabled)
                    exits.push_back({ .referenceId = reference.id, .position = reference.position });
        }
        candidateBounds = analysisConfig.cellBounds;
        if (resolved && candidateBounds) {
            const auto* selectedRecord = resolved->FindWinning(cell->id);
            std::unordered_map<std::uint32_t,std::optional<std::uint32_t>> worldspaceByCell;
            for (const auto& record : resolved->records) if (record.type == "CELL")
                worldspaceByCell.emplace(record.formId,record.worldspaceFormId);
            for (const auto& worldCell : resolved->cells) {
                const auto owner = worldspaceByCell.find(worldCell.id);
                if (!selectedRecord || owner == worldspaceByCell.end() || !selectedRecord->worldspaceFormId
                    || owner->second != selectedRecord->worldspaceFormId) continue;
                for (const auto& reference : worldCell.references)
                    if (reference.recordType == "REFR" && reference.baseRecordType == "DOOR"
                        && !reference.deleted && !reference.initiallyDisabled
                        && reference.position.x >= candidateBounds->min.x && reference.position.x <= candidateBounds->max.x
                        && reference.position.y >= candidateBounds->min.y && reference.position.y <= candidateBounds->max.y)
                        exits.push_back({ .referenceId = reference.id, .position = reference.position });
            }
        }
        std::sort(exits.begin(),exits.end(),[](const auto& a, const auto& b){return a.referenceId < b.referenceId;});
        exits.erase(std::unique(exits.begin(),exits.end(),[](const auto& a, const auto& b){return a.referenceId == b.referenceId;}),exits.end());
        try {
            update(83, "Generating candidate NAVM");
            candidate = navmesh::core::RecastCandidateGenerator{}.Generate(geometry.scene, navmesh::core::NavigationProfile{},candidateBounds,
                std::move(exits), options.partitioningAlgorithm);
            update(85, "Candidate NAVM generated");
            if (wasCancelled()) return 3;
            if (resolved && analysisConfig.cellBounds) {
                update(86, "Matching adjacent NAVM borders");
                std::vector<navmesh::core::NavMesh> adjacent;
                const auto* selectedRecord = resolved->FindWinning(cell->id);
                for (const auto& other : resolved->cells) {
                    if (!other.exteriorCoordinates || !cell->exteriorCoordinates || other.id == cell->id) continue;
                    const auto dx = std::abs((*other.exteriorCoordinates)[0]-(*cell->exteriorCoordinates)[0]);
                    const auto dy = std::abs((*other.exteriorCoordinates)[1]-(*cell->exteriorCoordinates)[1]);
                    if (dx+dy != 1) continue;
                    const auto* otherRecord = resolved->FindWinning(other.id);
                    if (selectedRecord && otherRecord && otherRecord->worldspaceFormId == selectedRecord->worldspaceFormId)
                        adjacent.insert(adjacent.end(),other.navMeshes.begin(),other.navMeshes.end());
                }
                const auto links = navmesh::core::StitchCandidateBorders(*candidate,*analysisConfig.cellBounds,adjacent);
                if (!links && std::any_of(candidate->regions.begin(),candidate->regions.end(),[](const auto& region) { return region.reachesBorder; }))
                    candidate->warnings.push_back("No exterior border edge matched an adjacent NAVM; cross-cell navigation is unlinked.");
            }
        } catch (const std::exception& error) {
            std::cerr << "Candidate generation failed: " << error.what() << "\n";
            return 2;
        }
        const auto jsonPath = options.output / "candidate-navm.json";
        const auto objPath = options.output / "candidate-navm.obj";
        update(87, "Writing candidate exports");
        if (!navmesh::core::WriteCandidateJson(jsonPath, *candidate, geometry.scene, navmesh::reproducibility::ToJson(metadata, "    "))
            || !navmesh::core::WriteCandidateObj(objPath, *candidate)
            || !navmesh::reproducibility::WriteSidecar(objPath, metadata)) {
            std::cerr << "Failed to write neutral candidate exports.\n"; return 2;
        }
    }
    std::size_t existingSelectedPolygons{};
    for (const auto& navmesh : cell->navMeshes) existingSelectedPolygons += navmesh.polygons.size();
    const auto generatedPolygons = candidate ? candidate->mesh.polygons.size() : 0U;
    const auto navmeshCounts = std::format(
        "Number of original navmesh polygons (selected cell): {}\n"
        "Number of generated navmesh polygons: {}\n",
        existingSelectedPolygons, generatedPolygons);
    std::ofstream countLog(options.output / "navmesh-counts.txt", std::ios::trunc);
    countLog << navmeshCounts;
    countLog.close();
    if (!countLog) { std::cerr << "Failed to write navmesh-counts.txt.\n"; return 2; }
    std::cout << navmeshCounts;
    navmesh::core::SceneExportOptions sceneOptions{ .layers = ParseSceneLayers(options.geometryLayers), .detailedProvenance = options.outputDetail != "summary" };
    if (candidate) {
        sceneOptions.layers.push_back(navmesh::core::SceneLayer::CandidateNavmesh);
        sceneOptions.candidateNavmesh = &candidate->mesh;
        sceneOptions.candidateEntrances = &candidate->exits;
    }
    if (options.sceneBounds) {
        const auto& bounds = *options.sceneBounds;
        if (bounds[0] > bounds[2] || bounds[1] > bounds[3]) { std::cerr << "--scene-bounds requires minX minY maxX maxY.\n"; return 1; }
        sceneOptions.bounds = navmesh::core::SceneBounds{ .world = { .min = { bounds[0], bounds[1], std::numeric_limits<float>::lowest() }, .max = { bounds[2], bounds[3], std::numeric_limits<float>::max() } } };
    }
    const auto scenePath = options.exportScene.empty() ? options.output / "scene.glb" : options.exportScene;
    update(88, "Writing combined scene");
    const auto sceneExport = navmesh::core::WriteCombinedGlb(scenePath, geometry.scene, sceneNavmeshes, sceneMarkers, metadata, sceneOptions);
    if (!navmesh::reproducibility::WriteSidecar(scenePath, metadata)) std::cerr << "Failed to write GLB metadata sidecar\n";
    std::cout << std::format("Exported combined GLB scene to {} ({} objects, {} triangles, {} geometry triangles culled)\n", scenePath.string(), sceneExport.objects, sceneExport.triangles, sceneExport.culledTriangles);
    if (candidate && !candidate->topology.valid) {
        std::cerr << "Candidate topology validation failed; see candidate-navm.json.\n";
        return 2;
    }
    if (options.generatePlugin) {
        std::vector<std::filesystem::path> inputPaths;
        if (mo2Input) inputPaths = mo2Input->pluginPaths;
        else for (auto path : navmesh::skyrim::offline::ReadLoadOrderManifest(options.loadOrder))
            inputPaths.push_back(path.is_absolute() ? path : options.data / path);
        std::filesystem::path pluginPath;
        std::string error;
        update(89, "Writing NAVM override plugin");
        if (!navmesh::skyrim::offline::WriteNavmeshOverride(options.output, inputPaths, *resolved, *cell, *candidate, pluginPath, error)) {
            std::cerr << "Plugin generation failed: " << error << "\n";
            update(89, std::format("Plugin generation failed: {}", error));
            return 2;
        }
        std::ifstream writtenPlugin(pluginPath, std::ios::binary);
        std::array<unsigned char, 12> pluginHeader{};
        writtenPlugin.read(reinterpret_cast<char*>(pluginHeader.data()), static_cast<std::streamsize>(pluginHeader.size()));
        const bool eslFlagged = writtenPlugin && (pluginHeader[9] & 0x02U) != 0;
        std::cout << "Generated NAVM override plugin: " << pluginPath.string()
            << (eslFlagged ? " (ESL-flagged ESP)\n" : " (regular ESP)\n");
        std::cerr << "Generated NAVM includes any matched border and door links; NAVI, teleport-door XNDP, cover, and other authored links still need independent validation.\n";
    }
    update(90, "Navmesh support analyzed");
    if (wasCancelled()) return 3;

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
    if (!cell->navMeshes.empty()) {
        const auto sceneReportPath = options.output / "scene-report.html";
        WriteDiagnosticHtml(sceneReportPath, *cell, cell->navMeshes.front(), geometry.mesh, geometry, analysisReport);
        if (!navmesh::reproducibility::WriteSidecar(sceneReportPath, metadata)) std::cerr << "Failed to write scene report metadata sidecar\n";
    }
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
    update(100, std::format("Original navmesh polygons: {}; generated navmesh polygons: {}",
        existingSelectedPolygons, generatedPolygons));
    return 0;
}
