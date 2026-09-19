#include "analysis/navmesh_analysis.h"
#include "cli/json_report.h"
#include "skyrim/parser/plugin_parser.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "validation/validation.h"

#include <filesystem>
#include <fstream>
#include <format>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace
{
    struct Options
    {
        std::filesystem::path plugin;
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

    void WriteAnalysisJson(const std::filesystem::path& outputPath, const navmesh::analysis::AnalysisReport& analysisReport, const navmesh::analysis::GeometrySummary& geometrySummary, const navmesh::analysis::NavMeshSummary& meshSummary)
    {
        std::ofstream stream(outputPath, std::ios::trunc);
        if (!stream.is_open()) {
            return;
        }

        stream << "{\n";
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
            stream << std::format("          \"found\": {},\n", polygon.support.found ? "true" : "false");
            if (polygon.support.found) {
                stream << "          \"point\": [" << polygon.support.point.x << ", " << polygon.support.point.y << ", " << polygon.support.point.z << "],\n";
                stream << std::format("          \"distance\": {},\n", polygon.support.distance);
                stream << std::format("          \"heightDelta\": {},\n", polygon.support.heightDelta);
                stream << "          \"normal\": [" << polygon.support.normal.x << ", " << polygon.support.normal.y << ", " << polygon.support.normal.z << "],\n";
                stream << std::format("          \"slopeDegrees\": {},\n", polygon.support.slopeDegrees);
            }
            stream << "        },\n";
            stream << std::format("        \"classification\": \"{}\"\n", polygon.classification);
            stream << "      }" << (index + 1 == analysisReport.polygons.size() ? "" : ",") << "\n";
        }
        stream << "    ]\n";
        stream << "  }\n";
        stream << "}\n";
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
}

int main(int argc, char** argv)
{
    const auto options = ParseArgs(argc, argv);
    if (options.plugin.empty()) {
        std::cerr << "Usage: navmesh-offline --plugin <plugin.esm> [--cell-formid <hex>] [--editor-id <id>] [--cell <name>] [--worldspace <name>] [--cell-x <n> --cell-y <n>] [--list-cells] [--output <dir>] [--export-geometry <path>] [--export-analysis <path>] [--surface-search-radius <n>] [--max-support-distance <n>] [--max-slope <n>]\n";
        return 1;
    }

    if (options.listCells) {
        const auto cells = navmesh::skyrim::offline::ListCells(options.plugin);
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
    const auto cell = navmesh::skyrim::offline::LoadCell(options.plugin, options.cell, options.worldspace, options.cellX, options.cellY, options.cellFormId, options.editorId);
    if (!cell) {
        std::cerr << "No matching Skyrim cell was found in " << options.plugin << "\n";
        return 2;
    }

    const auto findings = navmesh::validation::Validate(*cell);
    const auto report = navmesh::cli::ToJson(*cell, findings);
    const auto reportPath = options.output / "report.json";
    std::ofstream reportStream(reportPath, std::ios::trunc | std::ios::binary);
    reportStream << report;

    const auto geometry = navmesh::skyrim::offline::ExtractGeometry(options.plugin.parent_path(), *cell, options.output / ".bsa-cache");
    const auto geometryPath = options.exportGeometry.empty() ? options.output / "geometry.obj" : options.exportGeometry;
    if (!navmesh::skyrim::offline::WriteGeometryObj(geometryPath, geometry)) std::cerr << "Failed to write geometry OBJ to " << geometryPath << "\n";
    if (!navmesh::skyrim::offline::WriteGeometryJson(options.output / "geometry.json", *cell, geometry)) std::cerr << "Failed to write geometry JSON\n";

    const auto geometrySummary = navmesh::analysis::AnalyzeGeometry(geometry.mesh);
    const auto meshSummary = navmesh::analysis::AnalyzeNavMesh(cell->navMeshes.empty() ? navmesh::core::NavMesh{} : cell->navMeshes.front());
    const auto analysisConfig = navmesh::analysis::AnalysisConfiguration{ .surfaceSearchRadius = options.surfaceSearchRadius, .maxSupportDistance = options.maxSupportDistance, .maxSlope = options.maxSlope };
    const auto analysisReport = cell->navMeshes.empty() ? navmesh::analysis::AnalysisReport{} : navmesh::analysis::AnalyzeNavMeshPolygons(cell->navMeshes.front(), geometry.mesh, analysisConfig);

    if (!options.exportAnalysis.empty() && !cell->navMeshes.empty()) {
        WriteAnalysisObj(options.exportAnalysis, cell->navMeshes.front(), geometry.mesh, analysisReport);
        std::cout << "Exported analysis OBJ to " << options.exportAnalysis << "\n";
    }

    for (std::size_t index = 0; index < cell->navMeshes.size(); ++index) {
        const auto& mesh = cell->navMeshes[index];
        const auto meshPath = options.output / std::format("navmesh_{}.obj", index);
        WriteObj(meshPath, mesh, std::format("NAVM {:08X}", mesh.id));
    }

    std::cout << std::format("Geometry bounds:\n  X: {} -> {}\n  Y: {} -> {}\n  Z: {} -> {}\n", geometrySummary.bounds.min.x, geometrySummary.bounds.max.x, geometrySummary.bounds.min.y, geometrySummary.bounds.max.y, geometrySummary.bounds.min.z, geometrySummary.bounds.max.z);
    std::cout << std::format("NAVM bounds:\n  X: {} -> {}\n  Y: {} -> {}\n  Z: {} -> {}\n", meshSummary.bounds.min.x, meshSummary.bounds.max.x, meshSummary.bounds.min.y, meshSummary.bounds.max.y, meshSummary.bounds.min.z, meshSummary.bounds.max.z);
    std::cout << std::format("Geometry center: ({}, {}, {})\nNAVM center: ({}, {}, {})\nGeometry extent: ({}, {}, {})\nNAVM extent: ({}, {}, {})\n",
        geometrySummary.center.x, geometrySummary.center.y, geometrySummary.center.z,
        meshSummary.center.x, meshSummary.center.y, meshSummary.center.z,
        geometrySummary.extent.x, geometrySummary.extent.y, geometrySummary.extent.z,
        meshSummary.extent.x, meshSummary.extent.y, meshSummary.extent.z);
    std::cout << std::format("Analysis thresholds: surfaceSearchRadius={} maxSupportDistance={} maxSlope={}\n", options.surfaceSearchRadius, options.maxSupportDistance, options.maxSlope);

    WriteAnalysisJson(options.output / "analysis.json", analysisReport, geometrySummary, meshSummary);
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
