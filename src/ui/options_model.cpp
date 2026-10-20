#include "ui/options_model.h"

#include <cmath>
#include <format>
#include <stdexcept>

namespace navmesh::ui
{
    void ResetAdvancedNumericalOptions(app::Options &draft)
    {
        const app::Options defaults;
        draft.navigationProfile.agentRadius = defaults.navigationProfile.agentRadius;
        draft.navigationProfile.agentHeight = defaults.navigationProfile.agentHeight;
        draft.navigationProfile.clearance = defaults.navigationProfile.clearance;
        draft.navigationProfile.maxSlopeDegrees = defaults.navigationProfile.maxSlopeDegrees;
        draft.navigationProfile.stepHeight = defaults.navigationProfile.stepHeight;
        draft.navigationProfile.minimumRegionArea = defaults.navigationProfile.minimumRegionArea;
        draft.navigationProfile.weldTolerance = defaults.navigationProfile.weldTolerance;
        draft.navigationProfile.contourSimplificationTolerance =
            defaults.navigationProfile.contourSimplificationTolerance;
        draft.recastSettings = defaults.recastSettings;
        draft.surfaceSearchRadius = defaults.surfaceSearchRadius;
        draft.maxSupportDistance = defaults.maxSupportDistance;
        draft.maxSlope = defaults.maxSlope;
        draft.neighboringCellRadius = defaults.neighboringCellRadius;
        draft.cacheBudgetMiB = defaults.cacheBudgetMiB;
        draft.workingMemoryMiB = defaults.workingMemoryMiB;
        draft.workers = defaults.workers;
    }

    app::Options PrepareDesktopOptions(const app::Options &draft, bool listOnly)
    {
        auto result = draft;
        if (result.modManager != app::ModManager::Mo2 && result.modManager != app::ModManager::Vortex)
        {
            throw std::invalid_argument("Choose MO2 or Vortex as the mod manager.");
        }
        if (result.output.empty())
        {
            throw std::invalid_argument("Choose an output folder.");
        }
        if (result.modManager == app::ModManager::Mo2)
        {
            if (result.mo2.empty() || result.profile.empty())
            {
                throw std::invalid_argument("Choose an MO2 folder and an existing profile.");
            }
            result.vortex.clear();
            result.vortexPlugins.clear();
        }
        else
        {
            if (result.vortex.empty())
            {
                throw std::invalid_argument("Choose the Skyrim installation folder deployed by Vortex.");
            }
            result.mo2.clear();
            result.profile.clear();
        }

        // The selected manager resolves inputs and the desktop writes standard artifact paths.
        // Clearing escape-hatch and hidden selectors prevents persisted state leaking into another scope.
        result.plugin.clear();
        result.data.clear();
        result.loadOrder.clear();
        result.modsDirectory.clear();
        result.exportGeometry.clear();
        result.exportAnalysis.clear();
        result.exportScene.clear();
        result.cell.clear();
        result.cellX.reset();
        result.cellY.reset();
        result.sceneBounds.reset();
        result.terrainOnly = false;
        result.diagnostics = false;
        result.listCells = listOnly;
        if (listOnly)
        {
            result.rebuildScope = app::RebuildScope::Cell;
            result.generateCandidate = false;
            result.generatePlugin = false;
            result.skipExistingNavmesh = false;
            result.copyPlugin = false;
            result.estimateOnly = false;
            result.batchOutput = "auto";
            result.navigationProfile = {};
            result.recastSettings = {};
            result.surfaceSearchRadius = app::Options{}.surfaceSearchRadius;
            result.maxSupportDistance = app::Options{}.maxSupportDistance;
            result.maxSlope = app::Options{}.maxSlope;
            result.neighboringCellRadius = app::Options{}.neighboringCellRadius;
            result.cacheBudgetMiB = app::Options{}.cacheBudgetMiB;
            result.workingMemoryMiB = app::Options{}.workingMemoryMiB;
        }
        const bool cellScope = result.rebuildScope == app::RebuildScope::Cell;
        if (listOnly || !cellScope || !result.cellSelection.empty())
        {
            result.cellFormId.reset();
            result.editorId.clear();
        }
        if (listOnly || !cellScope)
        {
            result.cellSelection.clear();
        }
        if (listOnly || result.rebuildScope == app::RebuildScope::LoadOrder)
        {
            result.makeScene = false;
        }
        if (listOnly || result.rebuildScope != app::RebuildScope::Plugin)
        {
            result.affectedPlugin.clear();
        }
        if (listOnly || result.rebuildScope == app::RebuildScope::LoadOrder)
        {
            result.copyPlugin = false;
        }
        if (listOnly || !cellScope || !result.copyPlugin)
        {
            result.copySourcePlugin.clear();
        }
        result.generatePlugin = result.generatePlugin || result.copyPlugin;
        result.generateCandidate = result.generateCandidate || result.generatePlugin || !cellScope;
        if (result.makeScene)
        {
            result.estimateOnly = false;
        }
        const bool batchGeneration = app::UsesBatchGeneration(result);
        if (cellScope && !batchGeneration)
        {
            result.estimateOnly = false;
            result.batchOutput = "auto";
            result.workers = app::Options{}.workers;
        }
        else
        {
            result.surfaceSearchRadius = app::Options{}.surfaceSearchRadius;
            result.maxSupportDistance = app::Options{}.maxSupportDistance;
            result.maxSlope = app::Options{}.maxSlope;
        }
        result.worldspace.clear();
        if (!result.generateCandidate)
        {
            result.skipExistingNavmesh = false;
            result.navigationProfile = {};
            result.recastSettings = {};
        }
        if (result.generateCandidate)
        {
            if (result.partitioningAlgorithm == core::RegionPartitioningAlgorithm::Layers)
            {
                result.recastSettings.mergeRegionAreaMultiplier = core::RecastSettings{}.mergeRegionAreaMultiplier;
            }
            core::ValidateRecastSettings(result.navigationProfile, result.recastSettings);
        }
        if (result.neighboringCellRadius < 0)
        {
            throw std::invalid_argument(
                std::format("Invalid Neighboring cells / --neighboring-cell-radius: {}. Enter a nonnegative whole "
                            "number in Advanced settings or the CLI option.",
                            result.neighboringCellRadius));
        }
        if (result.workers == 0 || result.workers > 64)
        {
            throw std::invalid_argument(std::format("Invalid Workers / --workers: {}. Enter a whole number from 1 "
                                                    "through 64 in Advanced settings or the CLI option.",
                                                    result.workers));
        }
        if (result.workingMemoryMiB < 64 || result.workingMemoryMiB > 1048576)
        {
            throw std::invalid_argument(
                std::format("Invalid Working memory / --working-memory-mib: {}. Enter a value from 64 through 1048576 "
                            "MiB in Advanced settings or the CLI option.",
                            result.workingMemoryMiB));
        }
        if (result.cacheBudgetMiB > 1048576)
        {
            throw std::invalid_argument(
                std::format("Invalid Cache disk budget / --cache-budget-mib: {}. Enter a value from 0 through 1048576 "
                            "MiB in Advanced settings or the CLI option.",
                            result.cacheBudgetMiB));
        }
        if (!std::isfinite(result.surfaceSearchRadius) || result.surfaceSearchRadius <= 0)
        {
            throw std::invalid_argument(
                std::format("Invalid Surface search radius / --surface-search-radius: {}. Enter a finite positive "
                            "distance in Skyrim units in Advanced settings or the CLI option.",
                            result.surfaceSearchRadius));
        }
        if (!std::isfinite(result.maxSupportDistance) || result.maxSupportDistance < 0)
        {
            throw std::invalid_argument(
                std::format("Invalid Maximum support distance / --max-support-distance: {}. Enter a finite nonnegative "
                            "distance in Skyrim units in Advanced settings or the CLI option.",
                            result.maxSupportDistance));
        }
        if (!std::isfinite(result.maxSlope) || result.maxSlope < 0 || result.maxSlope >= 90)
        {
            throw std::invalid_argument(
                std::format("Invalid Analysis slope / --max-slope: {}. Enter a finite angle from 0 up to, but "
                            "excluding, 90 degrees in Advanced settings or the CLI option.",
                            result.maxSlope));
        }
        if (!listOnly && cellScope && result.cellSelection.empty() && !result.cellFormId && result.editorId.empty())
        {
            throw std::invalid_argument("Enter a cell Form ID or editor ID from cells.json.");
        }
        if (!listOnly && result.rebuildScope == app::RebuildScope::Plugin && result.affectedPlugin.empty())
        {
            throw std::invalid_argument("Enter the active plugin filename for Plugin scope.");
        }
        if (!listOnly && cellScope && !result.cellSelection.empty() &&
            app::ParseCellSelection(result.cellSelection).empty())
        {
            throw std::invalid_argument("Enter at least one cell Form ID or editor ID from cells.json.");
        }
        if (!listOnly && cellScope && result.copyPlugin && result.copySourcePlugin.empty())
        {
            throw std::invalid_argument("Enter the active plugin filename to copy for the selected cells.");
        }
        // A report-only batch cannot consume a plugin-only artifact policy.
        if (!result.generatePlugin && !result.estimateOnly && result.batchOutput == "plugin_only")
        {
            result.batchOutput = "auto";
        }
        return result;
    }
} // namespace navmesh::ui
