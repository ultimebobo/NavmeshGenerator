// Keep checks active in optimized builds.
#ifdef NDEBUG
#undef NDEBUG
#endif

#include "ui/options_model.h"

#include <cassert>
#include <limits>
#include <stdexcept>

void TestDesktopOptionsAndRecastSettings()
{
    using namespace navmesh;
    app::Options draft;
    draft.mo2 = "fixture-mo2";
    draft.profile = "fixture-profile";
    draft.output = "fixture-output";
    draft.cellFormId = 0x100;
    draft.editorId = "FixtureCell";
    draft.affectedPlugin = "Fixture.esp";
    draft.cellX = 2;
    draft.cellY = 3;
    draft.plugin = "developer.esp";
    draft.terrainOnly = true;
    draft.diagnostics = true;
    draft.copyPlugin = true;
    draft.estimateOnly = true;
    draft.skipExistingNavmesh = true;
    draft.navigationProfile.agentRadius = 7;
    draft.recastSettings.cellSize = 8;
    draft.rebuildScope = app::RebuildScope::Plugin;
    const auto plugin = ui::PrepareDesktopOptions(draft, ui::CellIdentification::FormId, false);
    assert(plugin.copyPlugin && plugin.generatePlugin && plugin.generateCandidate && plugin.estimateOnly);
    assert(!plugin.cellFormId && plugin.editorId.empty() && !plugin.cellX && !plugin.cellY);
    assert(plugin.plugin.empty() && !plugin.terrainOnly && !plugin.diagnostics);
    assert(plugin.navigationProfile.agentRadius == 7 && plugin.recastSettings.cellSize == 8);

    draft.rebuildScope = app::RebuildScope::LoadOrder;
    const auto loadOrder = ui::PrepareDesktopOptions(draft, ui::CellIdentification::EditorId, false);
    assert(!loadOrder.copyPlugin && !loadOrder.generatePlugin && loadOrder.affectedPlugin.empty());
    assert(loadOrder.generateCandidate && loadOrder.estimateOnly);

    draft.rebuildScope = app::RebuildScope::Cell;
    const auto cell = ui::PrepareDesktopOptions(draft, ui::CellIdentification::FormId, false);
    assert(cell.cellFormId == 0x100 && cell.editorId.empty());
    assert(!cell.copyPlugin && !cell.generateCandidate && !cell.estimateOnly && !cell.skipExistingNavmesh);
    assert(cell.recastSettings.cellSize == core::RecastSettings{}.cellSize);
    const auto editor = ui::PrepareDesktopOptions(draft, ui::CellIdentification::EditorId, false);
    assert(!editor.cellFormId && editor.editorId == "FixtureCell");

    // Catalog export remains usable when unrelated advanced settings and selectors are invalid.
    draft.rebuildScope = app::RebuildScope::Plugin;
    draft.navigationProfile.agentRadius = -1;
    draft.surfaceSearchRadius = -1;
    draft.neighboringCellRadius = -1;
    draft.workingMemoryMiB = 0;
    draft.workers = 0;
    draft.affectedPlugin.clear();
    const auto listing = ui::PrepareDesktopOptions(draft, ui::CellIdentification::FormId, true);
    assert(listing.listCells && !listing.generatePlugin && !listing.generateCandidate && !listing.copyPlugin);
    assert(!listing.estimateOnly && !listing.cellFormId && listing.editorId.empty());
    assert(listing.rebuildScope == app::RebuildScope::Cell);

    // Reset also repairs hidden values before the draft enters the shared run path.
    draft.generateCandidate = true;
    draft.rebuildScope = app::RebuildScope::Cell;
    draft.partitioningAlgorithm = core::RegionPartitioningAlgorithm::Monotone;
    draft.navigationProfile.name = "fixture-profile";
    draft.navigationProfile.cellBorderPolicy = "fixture-policy";
    draft.navigationProfile.agentHeight = 200;
    draft.navigationProfile.clearance = 200;
    draft.navigationProfile.maxSlopeDegrees = 60;
    draft.navigationProfile.stepHeight = 50;
    draft.navigationProfile.minimumRegionArea = 1;
    draft.navigationProfile.weldTolerance = 1;
    draft.navigationProfile.contourSimplificationTolerance = 1;
    draft.recastSettings = {8, 8, 1, 128, 2};
    draft.maxSupportDistance = 100;
    draft.maxSlope = 80;
    draft.cacheBudgetMiB = 123;
    ui::ResetAdvancedNumericalOptions(draft);
    const app::Options defaults;
    assert(draft.navigationProfile.agentRadius == defaults.navigationProfile.agentRadius);
    assert(draft.navigationProfile.agentHeight == defaults.navigationProfile.agentHeight);
    assert(draft.navigationProfile.clearance == defaults.navigationProfile.clearance);
    assert(draft.navigationProfile.maxSlopeDegrees == defaults.navigationProfile.maxSlopeDegrees);
    assert(draft.navigationProfile.stepHeight == defaults.navigationProfile.stepHeight);
    assert(draft.navigationProfile.minimumRegionArea == defaults.navigationProfile.minimumRegionArea);
    assert(draft.navigationProfile.weldTolerance == defaults.navigationProfile.weldTolerance);
    assert(draft.navigationProfile.contourSimplificationTolerance ==
           defaults.navigationProfile.contourSimplificationTolerance);
    assert(draft.recastSettings.cellSize == defaults.recastSettings.cellSize);
    assert(draft.recastSettings.cellHeight == defaults.recastSettings.cellHeight);
    assert(draft.recastSettings.maxSimplificationError == defaults.recastSettings.maxSimplificationError);
    assert(draft.recastSettings.maxEdgeLength == defaults.recastSettings.maxEdgeLength);
    assert(draft.recastSettings.mergeRegionAreaMultiplier == defaults.recastSettings.mergeRegionAreaMultiplier);
    assert(draft.surfaceSearchRadius == defaults.surfaceSearchRadius);
    assert(draft.maxSupportDistance == defaults.maxSupportDistance && draft.maxSlope == defaults.maxSlope);
    assert(draft.neighboringCellRadius == defaults.neighboringCellRadius);
    assert(draft.cacheBudgetMiB == defaults.cacheBudgetMiB && draft.workingMemoryMiB == defaults.workingMemoryMiB);
    assert(draft.workers == defaults.workers);
    assert(draft.mo2 == "fixture-mo2" && draft.profile == "fixture-profile" && draft.output == "fixture-output");
    assert(draft.partitioningAlgorithm == core::RegionPartitioningAlgorithm::Monotone && draft.generateCandidate);
    assert(draft.navigationProfile.name == "fixture-profile" &&
           draft.navigationProfile.cellBorderPolicy == "fixture-policy");
    const auto resetRun = ui::PrepareDesktopOptions(draft, ui::CellIdentification::FormId, false);
    assert(resetRun.navigationProfile.stepHeight == defaults.navigationProfile.stepHeight);
    assert(resetRun.recastSettings.cellHeight == defaults.recastSettings.cellHeight);

    core::NavigationProfile profile;
    core::RecastSettings settings;
    core::ValidateRecastSettings(profile, settings);
    const auto rejected = [&](float value)
    {
        settings.cellSize = value;
        try
        {
            core::ValidateRecastSettings(profile, settings);
            return false;
        }
        catch (const std::invalid_argument &)
        {
            return true;
        }
    };
    assert(rejected(0) && rejected(-1));
    assert(rejected(std::numeric_limits<float>::quiet_NaN()));
    assert(rejected(std::numeric_limits<float>::infinity()));
    assert(rejected(std::numeric_limits<float>::denorm_min()));
}
