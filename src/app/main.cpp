#include "analysis/navmesh_analysis.h"
#include "cli/json_report.h"
#include "skyrim/parser/plugin_parser.h"
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
        std::string worldspace;
        std::optional<std::int32_t> cellX;
        std::optional<std::int32_t> cellY;
        std::filesystem::path output;
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
            } else if (argument == "--worldspace" && index + 1 < argc) {
                options.worldspace = argv[++index];
            } else if (argument == "--cell-x" && index + 1 < argc) {
                options.cellX = std::stoi(argv[++index]);
            } else if (argument == "--cell-y" && index + 1 < argc) {
                options.cellY = std::stoi(argv[++index]);
            } else if (argument == "--output" && index + 1 < argc) {
                options.output = argv[++index];
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
}

int main(int argc, char** argv)
{
    const auto options = ParseArgs(argc, argv);
    if (options.plugin.empty()) {
        std::cerr << "Usage: navmesh-offline --plugin <plugin.esm> [--cell <editor-id>] [--worldspace <name>] [--cell-x <n>] [--cell-y <n>] [--output <dir>]\n";
        return 1;
    }

    std::filesystem::create_directories(options.output);
    const auto cell = navmesh::skyrim::offline::LoadCell(options.plugin, options.cell, options.worldspace, options.cellX, options.cellY);
    if (!cell) {
        std::cerr << "No matching Skyrim cell was found in " << options.plugin << "\n";
        return 2;
    }

    const auto findings = navmesh::validation::Validate(*cell);
    const auto report = navmesh::cli::ToJson(*cell, findings);
    const auto reportPath = options.output / "report.json";
    std::ofstream reportStream(reportPath, std::ios::trunc | std::ios::binary);
    reportStream << report;

    for (std::size_t index = 0; index < cell->navMeshes.size(); ++index) {
        const auto& mesh = cell->navMeshes[index];
        const auto meshPath = options.output / std::format("navmesh_{}.obj", index);
        WriteObj(meshPath, mesh, std::format("NAVM {:08X}", mesh.id));
    }

    if (!cell->navMeshes.empty()) {
        const auto analysis = navmesh::analysis::Analyze(cell->navMeshes.front());
        std::cout << std::format("Cell: {:08X}\nWorldspace: {}\nNAVM vertices: {}\nNAVM polygons: {}\nConnected regions: {}\nIsolated polygons: {}\nDegenerate polygons: {}\nAverage polygon area: {}\n",
            cell->id,
            cell->editorId.empty() ? "<unknown>" : cell->editorId,
            analysis.vertexCount,
            analysis.polygonCount,
            analysis.connectedComponents,
            analysis.isolatedPolygonCount,
            analysis.degeneratePolygonCount,
            analysis.averagePolygonArea);
    }

    std::cout << "Wrote report to " << reportPath << "\n";
    return 0;
}
