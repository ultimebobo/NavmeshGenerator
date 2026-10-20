#include "app/options.h"
#include "core/io/file_path.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <stdexcept>

namespace navmesh::app
{
    namespace
    {
        /// Retain the option and original input when the standard numeric parsers reject a value.
        template <class Parse>
        auto ParseNumber(const std::string &option, const std::string &value, std::string_view expected, Parse parse)
        {
            try
            {
                std::size_t end{};
                const auto number = parse(value, end);
                if (end == value.size())
                {
                    return number;
                }
            }
            catch (const std::invalid_argument &)
            {
            }
            catch (const std::out_of_range &)
            {
            }
            throw std::invalid_argument(std::format("Invalid value '{}' for {}. Enter {}.", value, option, expected));
        }

        float ParseFloat(const std::string &option, const std::string &value)
        {
            return ParseNumber(option, value, "a finite number within the supported range",
                               [](const std::string &text, std::size_t &end)
                               {
                                   const auto number = std::stof(text, &end);
                                   if (!std::isfinite(number))
                                   {
                                       throw std::invalid_argument("nonfinite");
                                   }
                                   return number;
                               });
        }

        int ParseInteger(const std::string &option, const std::string &value)
        {
            return ParseNumber(option, value, "a whole number within the signed integer range",
                               [](const std::string &text, std::size_t &end) { return std::stoi(text, &end); });
        }
    } // namespace

    std::vector<std::string> ParseCellSelection(std::string_view selection)
    {
        std::vector<std::string> identifiers;
        std::size_t position{};
        while ((position = selection.find_first_not_of(" \t\r\n,;", position)) != std::string_view::npos)
        {
            const auto end = selection.find_first_of(" \t\r\n,;", position);
            identifiers.emplace_back(selection.substr(position, end - position));
            if (end == std::string_view::npos)
            {
                break;
            }
            position = end;
        }
        return identifiers;
    }

    bool UsesBatchGeneration(const Options &options)
    {
        return options.rebuildScope != RebuildScope::Cell || options.copyPlugin ||
               (options.makeScene && options.skipExistingNavmesh &&
                (options.generateCandidate || options.generatePlugin)) ||
               ((options.generateCandidate || options.generatePlugin) &&
                ParseCellSelection(options.cellSelection).size() > 1);
    }

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
            const std::array valueOptions{"--plugin",
                                          "--data",
                                          "--load-order",
                                          "--mo2",
                                          "--vortex",
                                          "--vortex-plugins",
                                          "--profile",
                                          "--mods-dir",
                                          "--cell",
                                          "--cells",
                                          "--cell-formid",
                                          "--editor-id",
                                          "--worldspace",
                                          "--cell-x",
                                          "--cell-y",
                                          "--output",
                                          "--export-geometry",
                                          "--export-analysis",
                                          "--export-scene",
                                          "--geometry-layers",
                                          "--output-detail",
                                          "--batch-output",
                                          "--asset-cache",
                                          "--cache-budget-mib",
                                          "--working-memory-mib",
                                          "--workers",
                                          "--neighboring-cell-radius",
                                          "--scene-bounds",
                                          "--copy-plugin-source",
                                          "--surface-search-radius",
                                          "--max-support-distance",
                                          "--max-slope",
                                          "--partitioning-algorithm",
                                          "--rebuild-plugin"};
            const bool needsValue =
                std::find(valueOptions.begin(), valueOptions.end(), argument) != valueOptions.end() ||
                std::any_of(generationInputs.begin(), generationInputs.end(),
                            [&](const auto &input) { return argument == input.first; });
            if (needsValue)
            {
                const int count = argument == "--scene-bounds" ? 4 : 1;
                for (int offset = 1; offset <= count; ++offset)
                {
                    if (index + offset >= argc || std::string_view(argv[index + offset]).starts_with("--") ||
                        std::string_view(argv[index + offset]).empty())
                    {
                        throw std::invalid_argument(
                            argument == "--scene-bounds"
                                ? "--scene-bounds requires four numbers: minX minY maxX maxY."
                                : std::format("{} requires a value. Supply it immediately after the option; quote "
                                              "paths or names containing spaces.",
                                              argument));
                    }
                }
            }
            const auto generationInput = std::find_if(generationInputs.begin(), generationInputs.end(),
                                                      [&](const auto &input) { return argument == input.first; });
            if (generationInput != generationInputs.end())
            {
                *generationInput->second = ParseFloat(argument, argv[++index]);
            }
            else if (argument == "--plugin" && index + 1 < argc)
            {
                options.plugin = core::PathFromUtf8(argv[++index]);
            }
            else if (argument == "--data" && index + 1 < argc)
            {
                options.data = core::PathFromUtf8(argv[++index]);
            }
            else if (argument == "--load-order" && index + 1 < argc)
            {
                options.loadOrder = core::PathFromUtf8(argv[++index]);
            }
            else if (argument == "--mo2" && index + 1 < argc)
            {
                options.mo2 = core::PathFromUtf8(argv[++index]);
            }
            else if (argument == "--vortex" && index + 1 < argc)
            {
                options.modManager = ModManager::Vortex;
                options.vortex = core::PathFromUtf8(argv[++index]);
            }
            else if (argument == "--vortex-plugins" && index + 1 < argc)
            {
                options.vortexPlugins = core::PathFromUtf8(argv[++index]);
            }
            else if (argument == "--profile" && index + 1 < argc)
            {
                options.profile = argv[++index];
            }
            else if (argument == "--mods-dir" && index + 1 < argc)
            {
                options.modsDirectory = core::PathFromUtf8(argv[++index]);
            }
            else if (argument == "--cell" && index + 1 < argc)
            {
                options.cell = argv[++index];
            }
            else if (argument == "--cells" && index + 1 < argc)
            {
                if (options.rebuildScope != RebuildScope::Cell)
                {
                    throw std::invalid_argument(
                        "Choose one rebuild scope: use --cells, --rebuild-plugin, or --rebuild-load-order separately.");
                }
                options.cellSelection += std::string(argv[++index]) + "\n";
            }
            else if (argument == "--cell-formid" && index + 1 < argc)
            {
                options.cellFormId = ParseNumber(argument, argv[++index], "a hexadecimal CELL Form ID from cells.json",
                                                 [](const std::string &text, std::size_t &end)
                                                 {
                                                     const auto number = std::stoull(text, &end, 16);
                                                     if (text.find('-') != std::string::npos ||
                                                         number > std::numeric_limits<std::uint32_t>::max())
                                                     {
                                                         throw std::out_of_range("Form ID");
                                                     }
                                                     return static_cast<std::uint32_t>(number);
                                                 });
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
                options.cellX = ParseInteger(argument, argv[++index]);
            }
            else if (argument == "--cell-y" && index + 1 < argc)
            {
                options.cellY = ParseInteger(argument, argv[++index]);
            }
            else if (argument == "--output" && index + 1 < argc)
            {
                options.output = core::PathFromUtf8(argv[++index]);
            }
            else if (argument == "--export-geometry" && index + 1 < argc)
            {
                options.exportGeometry = core::PathFromUtf8(argv[++index]);
            }
            else if (argument == "--export-analysis" && index + 1 < argc)
            {
                options.exportAnalysis = core::PathFromUtf8(argv[++index]);
            }
            else if (argument == "--export-scene" && index + 1 < argc)
            {
                options.exportScene = core::PathFromUtf8(argv[++index]);
            }
            else if (argument == "--make-scene")
            {
                options.makeScene = true;
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
                options.assetCache = core::PathFromUtf8(argv[++index]);
            }
            else if ((argument == "--cache-budget-mib" || argument == "--working-memory-mib" ||
                      argument == "--workers") &&
                     index + 1 < argc)
            {
                const std::string value = argv[++index];
                const auto number =
                    ParseNumber(argument, value, "a nonnegative whole number within the unsigned integer range",
                                [](const std::string &text, std::size_t &end)
                                {
                                    if (text.find('-') != std::string::npos)
                                    {
                                        throw std::invalid_argument("negative");
                                    }
                                    return std::stoull(text, &end);
                                });
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
                options.neighboringCellRadius = ParseInteger(argument, argv[++index]);
            }
            else if (argument == "--scene-bounds" && index + 4 < argc)
            {
                options.sceneBounds =
                    std::array<float, 4>{ParseFloat(argument, argv[++index]), ParseFloat(argument, argv[++index]),
                                         ParseFloat(argument, argv[++index]), ParseFloat(argument, argv[++index])};
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
            else if (argument == "--copy-plugin-source" && index + 1 < argc)
            {
                options.copySourcePlugin = argv[++index];
            }
            else if (argument == "--surface-search-radius" && index + 1 < argc)
            {
                options.surfaceSearchRadius = ParseFloat(argument, argv[++index]);
            }
            else if (argument == "--max-support-distance" && index + 1 < argc)
            {
                options.maxSupportDistance = ParseFloat(argument, argv[++index]);
            }
            else if (argument == "--max-slope" && index + 1 < argc)
            {
                options.maxSlope = ParseFloat(argument, argv[++index]);
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
                if (options.rebuildScope != RebuildScope::Cell || !options.cellSelection.empty())
                {
                    throw std::invalid_argument(
                        "Choose one rebuild scope: use --cells, --rebuild-plugin, or --rebuild-load-order separately.");
                }
                options.rebuildScope = RebuildScope::Plugin;
                options.affectedPlugin = argv[++index];
                options.generateCandidate = true;
            }
            else if (argument == "--rebuild-load-order")
            {
                if (options.rebuildScope != RebuildScope::Cell || !options.cellSelection.empty())
                {
                    throw std::invalid_argument(
                        "Choose one rebuild scope: use --cells, --rebuild-plugin, or --rebuild-load-order separately.");
                }
                options.rebuildScope = RebuildScope::LoadOrder;
                options.generateCandidate = true;
            }
            else
            {
                throw std::invalid_argument(std::format("Unknown option or unexpected argument '{}'. Check the option "
                                                        "spelling and quote paths or names containing spaces.",
                                                        argument));
            }
        }
        return options;
    }
} // namespace navmesh::app
