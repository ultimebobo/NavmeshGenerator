#include "cli/inspection_report.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

namespace
{
    [[nodiscard]] std::string JsonEscape(const std::string &value)
    {
        std::string result;
        for (const char character : value)
        {
            if (character == '\\')
            {
                result += "\\\\";
            }
            else if (character == '"')
            {
                result += "\\\"";
            }
            else if (character == '\n')
            {
                result += "\\n";
            }
            else if (character == '\r')
            {
                result += "\\r";
            }
            else
            {
                result += character;
            }
        }
        return result;
    }

    [[nodiscard]] std::string HtmlEscape(const std::string &value)
    {
        std::string result;
        for (const char character : value)
        {
            if (character == '&')
            {
                result += "&amp;";
            }
            else if (character == '<')
            {
                result += "&lt;";
            }
            else if (character == '>')
            {
                result += "&gt;";
            }
            else if (character == '"')
            {
                result += "&quot;";
            }
            else
            {
                result += character;
            }
        }
        return result;
    }

    [[nodiscard]] std::string Vec3Text(const navmesh::core::Vec3 &value)
    {
        return std::format("({:.2f}, {:.2f}, {:.2f})", value.x, value.y, value.z);
    }

    [[nodiscard]] std::string ProjectionSvg(const navmesh::core::NavMesh &mesh, const navmesh::core::Mesh &geometry,
                                            const navmesh::analysis::PolygonAnalysisResult &polygon)
    {
        std::vector<navmesh::core::Vec3> points;
        if (polygon.index < mesh.polygons.size())
        {
            const auto &navPolygon = mesh.polygons[polygon.index];
            for (const auto vertex : navPolygon.vertices)
            {
                if (vertex < mesh.vertices.size())
                {
                    points.push_back(mesh.vertices[vertex]);
                }
            }
        }
        if (polygon.support.found && polygon.support.triangleIndex < geometry.triangles.size())
        {
            const auto &triangle = geometry.triangles[polygon.support.triangleIndex];
            for (const auto vertex : triangle.vertices)
            {
                if (vertex < geometry.vertices.size())
                {
                    points.push_back(geometry.vertices[vertex]);
                }
            }
        }
        if (points.empty())
        {
            return "<svg viewBox=\"0 0 360 90\"><text x=\"8\" y=\"20\">no geometry</text></svg>";
        }
        float minX = points.front().x, maxX = points.front().x, minY = points.front().y, maxY = points.front().y,
              minZ = points.front().z, maxZ = points.front().z;
        for (const auto &point : points)
        {
            minX = std::min(minX, point.x);
            maxX = std::max(maxX, point.x);
            minY = std::min(minY, point.y);
            maxY = std::max(maxY, point.y);
            minZ = std::min(minZ, point.z);
            maxZ = std::max(maxZ, point.z);
        }
        const auto scale = [](float value, float low, float high, float outputLow, float outputHigh)
        { return outputLow + (high - low > 1.0e-4F ? (value - low) / (high - low) : 0.5F) * (outputHigh - outputLow); };
        const auto topX = [&](const navmesh::core::Vec3 &point) { return scale(point.x, minX, maxX, 12.0F, 168.0F); };
        const auto topY = [&](const navmesh::core::Vec3 &point) { return scale(point.y, minY, maxY, 74.0F, 12.0F); };
        const auto sideX = [&](const navmesh::core::Vec3 &point) { return scale(point.x, minX, maxX, 192.0F, 348.0F); };
        const auto sideY = [&](const navmesh::core::Vec3 &point) { return scale(point.z, minZ, maxZ, 74.0F, 12.0F); };
        std::ostringstream svg;
        svg << "<svg viewBox=\"0 0 360 90\" role=\"img\"><rect width=\"360\" height=\"90\" fill=\"#f7f4ed\"/><text "
               "x=\"12\" y=\"9\" font-size=\"6\">top</text><text x=\"192\" y=\"9\" font-size=\"6\">side</text>";
        if (points.size() >= 3)
        {
            svg << std::format(
                "<polygon points=\"{},{} {},{} {},{}\" fill=\"#386641\" fill-opacity=\".35\" stroke=\"#386641\"/>",
                topX(points[0]), topY(points[0]), topX(points[1]), topY(points[1]), topX(points[2]), topY(points[2]));
        }
        if (points.size() >= 3)
        {
            svg << std::format(
                "<polygon points=\"{},{} {},{} {},{}\" fill=\"#386641\" fill-opacity=\".35\" stroke=\"#386641\"/>",
                sideX(points[0]), sideY(points[0]), sideX(points[1]), sideY(points[1]), sideX(points[2]),
                sideY(points[2]));
        }
        if (points.size() >= 6)
        {
            svg << std::format(
                "<polygon points=\"{},{} {},{} {},{}\" fill=\"#bc4749\" fill-opacity=\".35\" stroke=\"#bc4749\"/>",
                topX(points[3]), topY(points[3]), topX(points[4]), topY(points[4]), topX(points[5]), topY(points[5]));
        }
        for (std::size_t index = 0; index < std::min<std::size_t>(points.size(), 6); ++index)
        {
            svg << std::format("<circle cx=\"{}\" cy=\"{}\" r=\"1.5\" fill=\"{}\"/>", topX(points[index]),
                               topY(points[index]), index < 3 ? "#386641" : "#bc4749");
        }
        if (polygon.support.found)
        {
            svg << std::format("<circle cx=\"{}\" cy=\"{}\" r=\"2.5\" fill=\"#f08a24\"/><circle cx=\"{}\" cy=\"{}\" "
                               "r=\"2.5\" fill=\"#f08a24\"/>",
                               topX(polygon.centroid), topY(polygon.centroid), sideX(polygon.centroid),
                               sideY(polygon.centroid));
        }
        if (points.size() >= 6)
        {
            svg << std::format(
                "<polygon points=\"{},{} {},{} {},{}\" fill=\"none\" stroke=\"#bc4749\" stroke-dasharray=\"2 2\"/>",
                sideX(points[3]), sideY(points[3]), sideX(points[4]), sideY(points[4]), sideX(points[5]),
                sideY(points[5]));
        }
        svg << "</svg>";
        return svg.str();
    }

} // namespace

namespace navmesh::cli
{
    void WriteObj(const std::filesystem::path &outputPath, const navmesh::core::NavMesh &mesh, const std::string &name)
    {
        std::ofstream stream(outputPath, std::ios::trunc);
        if (!stream.is_open())
        {
            return;
        }
        for (const auto &vertex : mesh.vertices)
        {
            stream << std::format("v {} {} {}\n", vertex.x, vertex.y, vertex.z);
        }
        for (const auto &polygon : mesh.polygons)
        {
            if (polygon.vertices[0] >= mesh.vertices.size() || polygon.vertices[1] >= mesh.vertices.size() ||
                polygon.vertices[2] >= mesh.vertices.size())
            {
                continue;
            }
            stream << std::format("f {} {} {}\n", static_cast<std::uint32_t>(polygon.vertices[0] + 1),
                                  static_cast<std::uint32_t>(polygon.vertices[1] + 1),
                                  static_cast<std::uint32_t>(polygon.vertices[2] + 1));
        }
        stream << "# exported from " << name << "\n";
    }

    void WriteAnalysisObj(const std::filesystem::path &outputPath, const navmesh::core::NavMesh &mesh,
                          const navmesh::core::Mesh &geometry, const navmesh::analysis::AnalysisReport &analysisReport)
    {
        std::ofstream stream(outputPath, std::ios::trunc);
        if (!stream.is_open())
        {
            return;
        }

        std::size_t vertexIndex = 0;
        auto emitVertex = [&](const navmesh::core::Vec3 &vertex) -> std::size_t
        {
            stream << std::format("v {} {} {}\n", vertex.x, vertex.y, vertex.z);
            return ++vertexIndex;
        };

        stream << "g NAVM\n";
        for (const auto &polygon : mesh.polygons)
        {
            if (polygon.vertices[0] >= mesh.vertices.size() || polygon.vertices[1] >= mesh.vertices.size() ||
                polygon.vertices[2] >= mesh.vertices.size())
            {
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
        for (const auto &polygon : analysisReport.polygons)
        {
            if (!polygon.support.found || polygon.support.triangleIndex >= geometry.triangles.size())
            {
                continue;
            }
            const auto &triangle = geometry.triangles[polygon.support.triangleIndex];
            if (triangle.vertices[0] >= geometry.vertices.size() || triangle.vertices[1] >= geometry.vertices.size() ||
                triangle.vertices[2] >= geometry.vertices.size())
            {
                continue;
            }
            const auto va = emitVertex(geometry.vertices[triangle.vertices[0]]);
            const auto vb = emitVertex(geometry.vertices[triangle.vertices[1]]);
            const auto vc = emitVertex(geometry.vertices[triangle.vertices[2]]);
            stream << std::format("f {} {} {}\n", va, vb, vc);
        }

        stream << "# exported analysis visualization\n";
    }

    void WriteAnalysisJson(const std::filesystem::path &outputPath,
                           const navmesh::analysis::AnalysisReport &analysisReport,
                           const navmesh::analysis::GeometrySummary &geometrySummary,
                           const navmesh::analysis::NavMeshSummary &meshSummary,
                           const navmesh::reproducibility::ExportMetadata &metadata)
    {
        std::ofstream stream(outputPath, std::ios::trunc);
        if (!stream.is_open())
        {
            return;
        }

        stream << "{\n  \"metadata\": " << navmesh::reproducibility::ToJson(metadata, "    ") << ",\n";
        stream << "  \"geometry\": {\n";
        stream << std::format("    \"vertices\": {},\n", geometrySummary.vertexCount);
        stream << std::format("    \"triangles\": {},\n", geometrySummary.triangleCount);
        stream << "    \"bounds\": {\n";
        stream << std::format("      \"min\": [{}, {}, {}],\n", geometrySummary.bounds.min.x,
                              geometrySummary.bounds.min.y, geometrySummary.bounds.min.z);
        stream << std::format("      \"max\": [{}, {}, {}]\n", geometrySummary.bounds.max.x,
                              geometrySummary.bounds.max.y, geometrySummary.bounds.max.z);
        stream << "    }\n";
        stream << "  },\n";
        stream << "  \"navmesh\": {\n";
        stream << std::format("    \"vertices\": {},\n", meshSummary.vertexCount);
        stream << std::format("    \"polygons\": {},\n", meshSummary.polygonCount);
        stream << "    \"bounds\": {\n";
        stream << std::format("      \"min\": [{}, {}, {}],\n", meshSummary.bounds.min.x, meshSummary.bounds.min.y,
                              meshSummary.bounds.min.z);
        stream << std::format("      \"max\": [{}, {}, {}]\n", meshSummary.bounds.max.x, meshSummary.bounds.max.y,
                              meshSummary.bounds.max.z);
        stream << "    }\n";
        stream << "  },\n";
        stream << "  \"analysis\": {\n";
        stream << "    \"configuration\": {\n";
        stream << std::format("      \"surfaceSearchRadius\": {},\n", analysisReport.configuration.surfaceSearchRadius);
        stream << std::format("      \"maxSupportDistance\": {},\n", analysisReport.configuration.maxSupportDistance);
        stream << std::format("      \"maxSlope\": {},\n", analysisReport.configuration.maxSlope);
        stream << std::format("      \"minimumCoverage\": {},\n", analysisReport.configuration.minimumCoverage);
        stream << std::format("      \"obstructionClearance\": {},\n",
                              analysisReport.configuration.obstructionClearance);
        stream << std::format("      \"ambiguityHeightDelta\": {}\n",
                              analysisReport.configuration.ambiguityHeightDelta);
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
        for (std::size_t index = 0; index < analysisReport.polygons.size(); ++index)
        {
            const auto &polygon = analysisReport.polygons[index];
            stream << "      {\n";
            stream << std::format("        \"index\": {},\n", polygon.index);
            stream << "        \"centroid\": [" << polygon.centroid.x << ", " << polygon.centroid.y << ", "
                   << polygon.centroid.z << "],\n";
            stream << "        \"normal\": [" << polygon.normal.x << ", " << polygon.normal.y << ", "
                   << polygon.normal.z << "],\n";
            stream << "        \"support\": {\n";
            stream << std::format("          \"found\": {}{}\n", polygon.support.found ? "true" : "false",
                                  polygon.support.found ? "," : "");
            if (polygon.support.found)
            {
                stream << "          \"point\": [" << polygon.support.point.x << ", " << polygon.support.point.y << ", "
                       << polygon.support.point.z << "],\n";
                stream << std::format("          \"distance\": {},\n", polygon.support.distance);
                stream << std::format("          \"heightDelta\": {},\n", polygon.support.heightDelta);
                stream << "          \"normal\": [" << polygon.support.normal.x << ", " << polygon.support.normal.y
                       << ", " << polygon.support.normal.z << "],\n";
                stream << std::format("          \"slopeDegrees\": {},\n", polygon.support.slopeDegrees);
                stream << std::format("          \"triangleIndex\": {},\n", polygon.support.triangleIndex);
                stream << std::format("          \"sourceType\": \"{}\",\n", JsonEscape(polygon.support.sourceType));
                stream << std::format("          \"collisionType\": \"{}\",\n",
                                      JsonEscape(polygon.support.collisionType));
                stream << std::format("          \"sourceConfidence\": {},\n", polygon.support.sourceConfidence);
                stream << std::format("          \"confidence\": {},\n", polygon.support.confidence);
                stream << std::format("          \"samplesTotal\": {},\n", polygon.support.samplesTotal);
                stream << std::format("          \"samplesCovered\": {},\n", polygon.support.samplesCovered);
                stream << std::format("          \"sampleAgreement\": {},\n", polygon.support.sampleAgreement);
                stream << std::format("          \"sourceNifPath\": \"{}\",\n",
                                      JsonEscape(polygon.support.sourceNifPath));
                if (polygon.support.sourceTriangleIndex)
                {
                    stream << std::format("          \"sourceTriangleIndex\": {}\n",
                                          *polygon.support.sourceTriangleIndex);
                }
                else
                {
                    stream << "          \"sourceTriangleIndex\": null\n";
                }
            }
            stream << "        },\n";
            stream << std::format("        \"classification\": \"{}\"\n", polygon.classification);
            stream << "      }" << (index + 1 == analysisReport.polygons.size() ? "" : ",") << "\n";
        }
        stream << "    ],\n";
        stream << "    \"topology\": [\n";
        for (std::size_t index{}; index < analysisReport.topology.size(); ++index)
        {
            const auto &finding = analysisReport.topology[index];
            stream << std::format("      {{\"kind\":\"{}\",\"confidence\":{},\"evidence\":\"{}\",\"polygons\":[",
                                  JsonEscape(finding.kind), finding.confidence, JsonEscape(finding.evidence));
            for (std::size_t polygon{}; polygon < finding.polygons.size(); ++polygon)
            {
                stream << finding.polygons[polygon] << (polygon + 1 == finding.polygons.size() ? "" : ",");
            }
            stream << "]}" << (index + 1 == analysisReport.topology.size() ? "" : ",") << "\n";
        }
        stream << "    ],\n    \"repairCandidates\": [\n";
        for (std::size_t index{}; index < analysisReport.repairCandidates.size(); ++index)
        {
            const auto &candidate = analysisReport.repairCandidates[index];
            stream << std::format("      "
                                  "{{\"id\":\"{}\",\"kind\":\"{}\",\"confidence\":{},\"disposition\":\"{}\","
                                  "\"evidence\":\"{}\",\"polygons\":[",
                                  JsonEscape(candidate.id), JsonEscape(candidate.kind), candidate.confidence,
                                  JsonEscape(candidate.disposition), JsonEscape(candidate.evidence));
            for (std::size_t polygon{}; polygon < candidate.polygons.size(); ++polygon)
            {
                stream << candidate.polygons[polygon] << (polygon + 1 == candidate.polygons.size() ? "" : ",");
            }
            stream << "]}" << (index + 1 == analysisReport.repairCandidates.size() ? "" : ",") << "\n";
        }
        stream << "    ]\n";
        stream << "  }\n";
        stream << "}\n";
    }

    void WriteDiagnosticHtml(const std::filesystem::path &outputPath, const navmesh::core::Cell &cell,
                             const navmesh::core::NavMesh &mesh, const navmesh::core::Mesh &geometry,
                             const navmesh::skyrim::offline::GeometryExtraction &extraction,
                             const navmesh::analysis::AnalysisReport &report)
    {
        std::ofstream stream(outputPath, std::ios::trunc);
        if (!stream)
        {
            return;
        }
        std::vector<const navmesh::analysis::PolygonAnalysisResult *> selected;
        const std::vector<std::string> classes = {"supported", "floating",  "buried",         "too_steep",
                                                  "blocked",   "ambiguous", "out_of_coverage"};
        for (const auto &classification : classes)
        {
            std::size_t limit = classification == "buried" ? 10U : 1U;
            for (const auto &polygon : report.polygons)
            {
                if (polygon.classification == classification && limit-- > 0)
                {
                    selected.push_back(&polygon);
                }
            }
        }
        const auto missingModels =
            extraction.modelsMissing > 0 || !extraction.terrainSupported || !extraction.collisionGeometrySupported;
        stream << "<!doctype html><html><head><meta charset=\"utf-8\"><title>NAVM support "
                  "diagnostics</title><style>body{font:14px "
                  "system-ui,sans-serif;color:#252422;background:#f1eee7;margin:2rem}h1{font-family:Georgia,serif}"
                  "table{border-collapse:collapse;background:#fff}th,td{border:1px solid "
                  "#d8d2c6;padding:.45rem;text-align:left;vertical-align:top}th{background:#283618;color:#fff}tr."
                  "buried{background:#fff0e1}svg{width:360px;max-width:100%;height:auto}code{white-space:nowrap}</"
                  "style></head><body>";
        stream << std::format("<h1>NAVM support diagnostics</h1><p>Cell <code>{:08X}</code> &middot; {} &middot; "
                              "polygons analyzed: {} &middot; geometry triangles: {}</p>",
                              cell.id, HtmlEscape(cell.editorId.empty() ? cell.name : cell.editorId),
                              report.summary.polygonsAnalyzed, geometry.triangles.size());
        stream << std::format(
            "<p><strong>Counts:</strong> supported {} &middot; floating {} &middot; buried {} &middot; too steep {} "
            "&middot; blocked {} &middot; ambiguous {} &middot; out of coverage {}.</p>",
            report.summary.supported, report.summary.floating, report.summary.buried, report.summary.tooSteep,
            report.summary.blocked, report.summary.ambiguous, report.summary.outOfCoverage);
        stream << "<h2>Diagnostic assessment</h2><p>Seven interior and edge-aware samples are queried through the "
                  "spatial index. Collision takes priority over terrain, which takes priority over render fallback. "
                  "Ambiguous and out-of-coverage states are evidence limitations, never defects.</p>";
        stream
            << (missingModels
                    ? "<p><strong>Geometry limitation:</strong> terrain or collision geometry is not included, or one "
                      "or more model NIFs could not be loaded; buried counts may be incomplete or misleading where "
                      "those surfaces are missing.</p>"
                    : "<p><strong>Geometry coverage:</strong> extracted model geometry is available for the analyzed "
                      "cell; the examples below can be inspected as actual world-space support triangles.</p>");
        stream << "<p><strong>Interpretation:</strong> a buried result is supported by a concrete selected triangle "
                  "and a negative height delta. When the triangle has NIF provenance and the two views show the NAVM "
                  "above that triangle, it is consistent with genuinely buried NAVM. A missing source, visibly "
                  "displaced triangle, or a systematic offset is evidence for geometry/transform limitations and "
                  "should be investigated before changing thresholds.</p>";
        std::map<std::string, std::size_t> supportSources, coverage;
        for (const auto &polygon : report.polygons)
        {
            if (polygon.support.found)
            {
                ++supportSources[polygon.support.sourceType.empty() ? "unknown" : polygon.support.sourceType];
            }
        }
        for (const auto &entry : extraction.scene.coverage)
        {
            const auto name = entry.status == navmesh::core::GeometryCoverage::Found        ? "found"
                              : entry.status == navmesh::core::GeometryCoverage::Excluded   ? "excluded"
                              : entry.status == navmesh::core::GeometryCoverage::Missing    ? "missing"
                              : entry.status == navmesh::core::GeometryCoverage::Unreadable ? "unreadable"
                                                                                            : "unsupported";
            ++coverage[name];
        }
        stream << "<h2>Source and coverage groups</h2><table><thead><tr><th>Support source</th><th>Classified NAVM "
                  "polygons linked to source triangles</th></tr></thead><tbody>";
        for (const auto &[name, count] : supportSources)
        {
            stream << std::format("<tr><td>{}</td><td>{}</td></tr>", HtmlEscape(name), count);
        }
        if (supportSources.empty())
        {
            stream << "<tr><td>none</td><td>0</td></tr>";
        }
        stream << "</tbody></table><table><thead><tr><th>Geometry coverage "
                  "status</th><th>Sources</th></tr></thead><tbody>";
        for (const auto &[name, count] : coverage)
        {
            stream << std::format("<tr><td>{}</td><td>{}</td></tr>", HtmlEscape(name), count);
        }
        stream << "</tbody></table><p>Every selected row below identifies its world support-triangle index; use "
                  "<code>analysis.json</code> to join that index to the full <code>geometry.json</code> triangle "
                  "provenance.</p>";
        stream
            << "<h2>Selected polygons</h2><table><thead><tr><th>Index</th><th>Class</th><th>Centroid</th><th>Support "
               "point</th><th>Delta</th><th>Slope</th><th>Support "
               "triangle</th><th>Source</th><th>Views</th></tr></thead><tbody>";
        for (const auto *polygon : selected)
        {
            std::string triangleText = "none";
            std::string sourceText = "none";
            if (polygon->support.found && polygon->support.triangleIndex < geometry.triangles.size())
            {
                const auto &triangle = geometry.triangles[polygon->support.triangleIndex];
                triangleText = std::format("world #{}", polygon->support.triangleIndex);
                if (triangle.vertices[0] < geometry.vertices.size() &&
                    triangle.vertices[1] < geometry.vertices.size() && triangle.vertices[2] < geometry.vertices.size())
                {
                    triangleText += " " + Vec3Text(geometry.vertices[triangle.vertices[0]]) + " / " +
                                    Vec3Text(geometry.vertices[triangle.vertices[1]]) + " / " +
                                    Vec3Text(geometry.vertices[triangle.vertices[2]]);
                }
                sourceText =
                    HtmlEscape(polygon->support.sourceNifPath.empty() ? "unknown" : polygon->support.sourceNifPath);
                sourceText = HtmlEscape(polygon->support.sourceType) + " (" +
                             std::format("{:.2f}", polygon->support.sourceConfidence) + ") " + sourceText;
                if (polygon->support.sourceTriangleIndex)
                {
                    sourceText += std::format(" #{}", *polygon->support.sourceTriangleIndex);
                }
            }
            stream << std::format(
                "<tr "
                "class=\"{}\"><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td><small>{}</small></"
                "td><td><small>{}</small></td><td>{}</td></tr>",
                polygon->classification == "buried" ? "buried" : "", polygon->index, polygon->classification,
                Vec3Text(polygon->centroid), polygon->support.found ? Vec3Text(polygon->support.point) : "none",
                polygon->support.found ? std::format("{:.2f}", polygon->support.heightDelta) : "none",
                polygon->support.found ? std::format("{:.2f} deg", polygon->support.slopeDegrees) : "none",
                HtmlEscape(triangleText), sourceText, ProjectionSvg(mesh, geometry, *polygon));
        }
        stream << "</tbody></table><p><strong>Legend:</strong> green triangle = NAVM polygon; red triangle = "
                  "world-geometry support triangle in the top view; dotted red triangle = the same world-geometry "
                  "support triangle in the side view; orange markers = NAVM centroid and selected support "
                  "point.</p></body></html>\n";
    }

    void PrintAnalysisSummary(const navmesh::analysis::AnalysisReport &analysis)
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
        std::cout << std::format("  min:    {}\n  max:    {}\n  mean:   {}\n  median: {}\n  p95:    {}\n",
                                 analysis.heightDeltaStats.min, analysis.heightDeltaStats.max,
                                 analysis.heightDeltaStats.mean, analysis.heightDeltaStats.median,
                                 analysis.heightDeltaStats.p95);
        std::cout << "\nSlope:\n";
        std::cout << std::format("  min:    {}\n  max:    {}\n  mean:   {}\n  median: {}\n  p95:    {}\n",
                                 analysis.slopeStats.min, analysis.slopeStats.max, analysis.slopeStats.mean,
                                 analysis.slopeStats.median, analysis.slopeStats.p95);
    }

    void WriteLoadOrderJson(const std::filesystem::path &path,
                            const navmesh::skyrim::offline::ResolvedLoadOrder &loadOrder)
    {
        std::ofstream stream(path, std::ios::trunc);
        if (!stream)
        {
            return;
        }
        stream << "{\n  \"records\": [\n";
        for (std::size_t index = 0; index < loadOrder.records.size(); ++index)
        {
            const auto &record = loadOrder.records[index];
            stream << std::format(
                "    {{\"form_id\": \"{:08X}\", \"type\": \"{}\", \"winning_plugin\": \"{}\", \"origin_chain\": [",
                record.formId, JsonEscape(record.type), JsonEscape(record.winning.plugin));
            for (std::size_t origin = 0; origin < record.origins.size(); ++origin)
            {
                stream << std::format("\"{}\"{}", JsonEscape(record.origins[origin].plugin),
                                      origin + 1 == record.origins.size() ? "" : ", ");
            }
            stream << "]}" << (index + 1 == loadOrder.records.size() ? "" : ",") << "\n";
        }
        stream << "  ]\n}\n";
    }

    void WriteCellsJson(const std::filesystem::path &path, const std::vector<navmesh::core::Cell> &cells)
    {
        std::ofstream stream(path, std::ios::trunc);
        if (!stream)
        {
            return;
        }
        stream << "{\n  \"cells\": [\n";
        for (std::size_t index = 0; index < cells.size(); ++index)
        {
            const auto &cell = cells[index];
            stream << std::format(
                "    {{\"form_id\": \"{:08X}\", \"editor_id\": \"{}\", \"name\": \"{}\", \"interior\": {}", cell.id,
                JsonEscape(cell.editorId), JsonEscape(cell.name), cell.isInterior ? "true" : "false");
            if (cell.exteriorCoordinates)
            {
                stream << std::format(", \"coordinates\": [{}, {}]", (*cell.exteriorCoordinates)[0],
                                      (*cell.exteriorCoordinates)[1]);
            }
            stream << "}" << (index + 1 == cells.size() ? "" : ",") << "\n";
        }
        stream << "  ]\n}\n";
    }

} // namespace navmesh::cli
