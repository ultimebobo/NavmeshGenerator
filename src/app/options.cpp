#include "app/options.h"

#include <algorithm>
#include <stdexcept>

namespace navmesh::app
{
    Options ParseCommandLine(int argc, char **argv)
    {
        Options options;
        const std::array generationInputs{
            std::pair{"--agent-radius", &options.navigationProfile.agentRadius},
            std::pair{"--agent-height", &options.navigationProfile.agentHeight},
            std::pair{"--agent-clearance", &options.navigationProfile.clearance},
            std::pair{"--agent-step-height", &options.navigationProfile.stepHeight},
            std::pair{"--agent-max-slope", &options.navigationProfile.maxSlopeDegrees},
            std::pair{"--minimum-region-area", &options.navigationProfile.minimumRegionArea},
            std::pair{"--weld-tolerance", &options.navigationProfile.weldTolerance},
            std::pair{"--recast-cell-size", &options.recastSettings.cellSize},
            std::pair{"--recast-cell-height", &options.recastSettings.cellHeight},
            std::pair{"--recast-simplification-error", &options.recastSettings.maxSimplificationError},
            std::pair{"--recast-max-edge-length", &options.recastSettings.maxEdgeLength},
            std::pair{"--recast-merge-area-multiplier", &options.recastSettings.mergeRegionAreaMultiplier}};
        for (int index = 1; index < argc; ++index)
        {
            const std::string argument = argv[index];
            const auto generationInput = std::find_if(generationInputs.begin(), generationInputs.end(),
                                                      [&](const auto &input) { return argument == input.first; });
            if (generationInput != generationInputs.end())
            {
                if (index + 1 == argc)
                {
                    throw std::invalid_argument(argument + " requires a number");
                }
                const std::string value = argv[++index];
                std::size_t end{};
                const auto number = std::stof(value, &end);
                if (end != value.size())
                {
                    throw std::invalid_argument(argument + " requires a number without trailing characters");
                }
                *generationInput->second = number;
            }
            else if (argument == "--plugin" && index + 1 < argc)
            {
                options.plugin = argv[++index];
            }
            else if (argument == "--data" && index + 1 < argc)
            {
                options.data = argv[++index];
            }
            else if (argument == "--load-order" && index + 1 < argc)
            {
                options.loadOrder = argv[++index];
            }
            else if (argument == "--mo2" && index + 1 < argc)
            {
                options.mo2 = argv[++index];
            }
            else if (argument == "--profile" && index + 1 < argc)
            {
                options.profile = argv[++index];
            }
            else if (argument == "--mods-dir" && index + 1 < argc)
            {
                options.modsDirectory = argv[++index];
            }
            else if (argument == "--cell" && index + 1 < argc)
            {
                options.cell = argv[++index];
            }
            else if (argument == "--cell-formid" && index + 1 < argc)
            {
                options.cellFormId = static_cast<std::uint32_t>(std::stoul(argv[++index], nullptr, 16));
            }
            else if (argument == "--editor-id" && index + 1 < argc)
            {
                options.editorId = argv[++index];
            }
            else if (argument == "--worldspace" && index + 1 < argc)
            {
                options.worldspace = argv[++index];
            }
            else if (argument == "--cell-x" && index + 1 < argc)
            {
                options.cellX = std::stoi(argv[++index]);
            }
            else if (argument == "--cell-y" && index + 1 < argc)
            {
                options.cellY = std::stoi(argv[++index]);
            }
            else if (argument == "--output" && index + 1 < argc)
            {
                options.output = argv[++index];
            }
            else if (argument == "--export-geometry" && index + 1 < argc)
            {
                options.exportGeometry = argv[++index];
            }
            else if (argument == "--export-analysis" && index + 1 < argc)
            {
                options.exportAnalysis = argv[++index];
            }
            else if (argument == "--export-scene" && index + 1 < argc)
            {
                options.exportScene = argv[++index];
            }
            else if (argument == "--geometry-layers" && index + 1 < argc)
            {
                options.geometryLayers = argv[++index];
            }
            else if (argument == "--output-detail" && index + 1 < argc)
            {
                options.outputDetail = argv[++index];
            }
            else if (argument == "--batch-output" && index + 1 < argc)
            {
                options.batchOutput = argv[++index];
            }
            else if (argument == "--asset-cache" && index + 1 < argc)
            {
                options.assetCache = argv[++index];
            }
            else if ((argument == "--cache-budget-mib" || argument == "--working-memory-mib" ||
                      argument == "--workers") &&
                     index + 1 < argc)
            {
                const std::string value = argv[++index];
                std::size_t end{};
                const auto number = std::stoull(value, &end);
                if (value.empty() || value.front() == '-' || end != value.size())
                {
                    throw std::invalid_argument(argument + " requires a nonnegative integer");
                }
                if (argument == "--cache-budget-mib")
                {
                    options.cacheBudgetMiB = number;
                }
                else if (argument == "--working-memory-mib")
                {
                    options.workingMemoryMiB = number;
                }
                else
                {
                    options.workers = number;
                }
            }
            else if (argument == "--neighboring-cell-radius" && index + 1 < argc)
            {
                options.neighboringCellRadius = std::stoi(argv[++index]);
            }
            else if (argument == "--scene-bounds" && index + 4 < argc)
            {
                options.sceneBounds = std::array<float, 4>{std::stof(argv[++index]), std::stof(argv[++index]),
                                                           std::stof(argv[++index]), std::stof(argv[++index])};
            }
            else if (argument == "--diagnostics")
            {
                options.diagnostics = true;
            }
            else if (argument == "--estimate-only")
            {
                options.estimateOnly = true;
            }
            else if (argument == "--terrain-only")
            {
                options.terrainOnly = true;
            }
            else if (argument == "--generate-candidate")
            {
                options.generateCandidate = true;
            }
            else if (argument == "--no-triangle-tagging")
            {
                options.tagTriangles = false;
            }
            else if (argument == "--generate-plugin")
            {
                if (index + 1 < argc &&
                    (std::string(argv[index + 1]) == "esp" || std::string(argv[index + 1]) == "esl"))
                {
                    throw std::invalid_argument(
                        "--generate-plugin selects ESL or ESP automatically; omit the format argument");
                }
                options.generatePlugin = true;
                options.generateCandidate = true;
            }
            else if (argument == "--skip-existing-navmesh")
            {
                options.skipExistingNavmesh = true;
            }
            else if (argument == "--copy-plugin")
            {
                options.copyPlugin = true;
                options.generatePlugin = true;
                options.generateCandidate = true;
            }
            else if (argument == "--surface-search-radius" && index + 1 < argc)
            {
                options.surfaceSearchRadius = std::stof(argv[++index]);
            }
            else if (argument == "--max-support-distance" && index + 1 < argc)
            {
                options.maxSupportDistance = std::stof(argv[++index]);
            }
            else if (argument == "--max-slope" && index + 1 < argc)
            {
                options.maxSlope = std::stof(argv[++index]);
            }
            else if (argument == "--partitioning-algorithm" && index + 1 < argc)
            {
                const std::string algorithm = argv[++index];
                if (algorithm == "watershed")
                {
                    options.partitioningAlgorithm = core::RegionPartitioningAlgorithm::Watershed;
                }
                else if (algorithm == "monotone")
                {
                    options.partitioningAlgorithm = core::RegionPartitioningAlgorithm::Monotone;
                }
                else if (algorithm == "layers")
                {
                    options.partitioningAlgorithm = core::RegionPartitioningAlgorithm::Layers;
                }
                else
                {
                    throw std::invalid_argument("--partitioning-algorithm must be watershed, monotone, or layers");
                }
            }
            else if (argument == "--list-cells")
            {
                options.listCells = true;
            }
            else if (argument == "--rebuild-plugin" && index + 1 < argc)
            {
                if (options.rebuildScope != RebuildScope::Cell)
                {
                    throw std::invalid_argument("Choose one rebuild scope");
                }
                options.rebuildScope = RebuildScope::Plugin;
                options.affectedPlugin = argv[++index];
                options.generateCandidate = true;
            }
            else if (argument == "--rebuild-load-order")
            {
                if (options.rebuildScope != RebuildScope::Cell)
                {
                    throw std::invalid_argument("Choose one rebuild scope");
                }
                options.rebuildScope = RebuildScope::LoadOrder;
                options.generateCandidate = true;
            }
            else if (argument == "--batch-output" || argument == "--asset-cache" || argument == "--cache-budget-mib" ||
                     argument == "--working-memory-mib" || argument == "--workers")
            {
                throw std::invalid_argument(argument + " requires a value");
            }
            else if (argument == "--rebuild-plugin")
            {
                throw std::invalid_argument("--rebuild-plugin requires an active plugin filename");
            }
        }
        return options;
    }
} // namespace navmesh::app
