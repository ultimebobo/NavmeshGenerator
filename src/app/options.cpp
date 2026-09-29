#include "app/options.h"

#include <algorithm>
#include <stdexcept>

namespace navmesh::app
{
    Options ParseCommandLine(int argc, char** argv)
    {
        Options options;
        for (int index = 1; index < argc; ++index) {
            const std::string argument = argv[index];
            if (argument == "--plugin" && index + 1 < argc) options.plugin = argv[++index];
            else if (argument == "--data" && index + 1 < argc) options.data = argv[++index];
            else if (argument == "--load-order" && index + 1 < argc) options.loadOrder = argv[++index];
            else if (argument == "--mo2" && index + 1 < argc) options.mo2 = argv[++index];
            else if (argument == "--profile" && index + 1 < argc) options.profile = argv[++index];
            else if (argument == "--mods-dir" && index + 1 < argc) options.modsDirectory = argv[++index];
            else if (argument == "--cell" && index + 1 < argc) options.cell = argv[++index];
            else if (argument == "--cell-formid" && index + 1 < argc) options.cellFormId = static_cast<std::uint32_t>(std::stoul(argv[++index], nullptr, 16));
            else if (argument == "--editor-id" && index + 1 < argc) options.editorId = argv[++index];
            else if (argument == "--worldspace" && index + 1 < argc) options.worldspace = argv[++index];
            else if (argument == "--cell-x" && index + 1 < argc) options.cellX = std::stoi(argv[++index]);
            else if (argument == "--cell-y" && index + 1 < argc) options.cellY = std::stoi(argv[++index]);
            else if (argument == "--output" && index + 1 < argc) options.output = argv[++index];
            else if (argument == "--export-geometry" && index + 1 < argc) options.exportGeometry = argv[++index];
            else if (argument == "--export-analysis" && index + 1 < argc) options.exportAnalysis = argv[++index];
            else if (argument == "--export-scene" && index + 1 < argc) options.exportScene = argv[++index];
            else if (argument == "--geometry-layers" && index + 1 < argc) options.geometryLayers = argv[++index];
            else if (argument == "--output-detail" && index + 1 < argc) options.outputDetail = argv[++index];
            else if (argument == "--neighboring-cell-radius" && index + 1 < argc) options.neighboringCellRadius = std::max(0, std::stoi(argv[++index]));
            else if (argument == "--scene-bounds" && index + 4 < argc) options.sceneBounds = std::array<float, 4>{ std::stof(argv[++index]), std::stof(argv[++index]), std::stof(argv[++index]), std::stof(argv[++index]) };
            else if (argument == "--diagnostics") options.diagnostics = true;
            else if (argument == "--terrain-only") options.terrainOnly = true;
            else if (argument == "--generate-candidate") options.generateCandidate = true;
            else if (argument == "--surface-search-radius" && index + 1 < argc) options.surfaceSearchRadius = std::stof(argv[++index]);
            else if (argument == "--max-support-distance" && index + 1 < argc) options.maxSupportDistance = std::stof(argv[++index]);
            else if (argument == "--max-slope" && index + 1 < argc) options.maxSlope = std::stof(argv[++index]);
            else if (argument == "--list-cells") options.listCells = true;
        }
        return options;
    }
}
