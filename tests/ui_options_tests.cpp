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
    draft.tagTriangles = false;
    draft.makeScene = true;
    draft.navigationProfile.agentRadius = 7;
    draft.recastSettings.cellSize = 8;
    draft.rebuildScope = app::RebuildScope::Plugin;
    const auto plugin = ui::PrepareDesktopOptions(draft, false);
    assert(plugin.copyPlugin && plugin.generatePlugin && plugin.generateCandidate && plugin.estimateOnly);
    assert(!plugin.tagTriangles);
    assert(!plugin.makeScene);
    assert(!plugin.cellFormId && plugin.editorId.empty() && !plugin.cellX && !plugin.cellY);
    assert(plugin.plugin.empty() && !plugin.terrainOnly && !plugin.diagnostics);
    assert(plugin.navigationProfile.agentRadius == 7 && plugin.recastSettings.cellSize == 8);

    draft.rebuildScope = app::RebuildScope::Cell;
    draft.copySourcePlugin = "Fixture.esp";
    draft.cellSelection = "FixtureCell\n00000100; OtherCell,0x101";
    const auto selected = ui::PrepareDesktopOptions(draft, false);
    assert(selected.cellSelection == draft.cellSelection && selected.copySourcePlugin == "Fixture.esp" &&
           selected.affectedPlugin.empty());
    assert(selected.copyPlugin && selected.generatePlugin && selected.generateCandidate);
    assert(!selected.cellFormId && selected.editorId.empty() && !selected.estimateOnly && selected.makeScene);
    assert((app::ParseCellSelection(selected.cellSelection) ==
            std::vector<std::string>{"FixtureCell", "00000100", "OtherCell", "0x101"}));
    const auto rejectsSelection = [&](const app::Options &invalid)
    {
        try
        {
            (void)ui::PrepareDesktopOptions(invalid, false);
            return false;
        }
        catch (const std::invalid_argument &)
        {
            return true;
        }
    };
    auto invalidSelection = draft;
    invalidSelection.cellSelection = " \r\n,;\t";
    assert(rejectsSelection(invalidSelection));
    invalidSelection = draft;
    invalidSelection.copySourcePlugin.clear();
    assert(rejectsSelection(invalidSelection));
    invalidSelection.copyPlugin = false;
    const auto selectedPreview = ui::PrepareDesktopOptions(invalidSelection, false);
    assert(!selectedPreview.generateCandidate && !selectedPreview.generatePlugin);
    assert(selectedPreview.copySourcePlugin.empty());
    assert(selectedPreview.cellSelection == draft.cellSelection && !app::UsesBatchGeneration(selectedPreview));
    invalidSelection.generateCandidate = true;
    const auto multiplePreview = ui::PrepareDesktopOptions(invalidSelection, false);
    assert(multiplePreview.rebuildScope == app::RebuildScope::Cell && app::UsesBatchGeneration(multiplePreview));
    assert(!multiplePreview.copyPlugin && !multiplePreview.generatePlugin);
    invalidSelection.cellSelection = "FixtureCell";
    invalidSelection.makeScene = false;
    const auto singlePreview = ui::PrepareDesktopOptions(invalidSelection, false);
    assert(!app::UsesBatchGeneration(singlePreview) && singlePreview.generateCandidate && !singlePreview.estimateOnly);
    const auto selectedListing = ui::PrepareDesktopOptions(draft, true);
    assert(selectedListing.cellSelection.empty() && !selectedListing.copyPlugin);
    assert(!selectedListing.makeScene);

    draft.rebuildScope = app::RebuildScope::LoadOrder;
    const auto loadOrder = ui::PrepareDesktopOptions(draft, false);
    assert(!loadOrder.copyPlugin && !loadOrder.generatePlugin && loadOrder.affectedPlugin.empty());
    assert(loadOrder.generateCandidate && loadOrder.estimateOnly);
    assert(loadOrder.cellSelection.empty());
    assert(!loadOrder.makeScene);

    draft.rebuildScope = app::RebuildScope::Cell;
    draft.cellSelection.clear();
    draft.copyPlugin = false;
    draft.editorId.clear();
    const auto cell = ui::PrepareDesktopOptions(draft, false);
    assert(cell.cellFormId == 0x100 && cell.editorId.empty());
    assert(!cell.copyPlugin && !cell.generateCandidate && !cell.estimateOnly && !cell.skipExistingNavmesh);
    assert(cell.recastSettings.cellSize == core::RecastSettings{}.cellSize);
    draft.cellFormId.reset();
    draft.editorId = "FixtureCell";
    const auto editor = ui::PrepareDesktopOptions(draft, false);
    assert(!editor.cellFormId && editor.editorId == "FixtureCell");

    // Catalog export remains usable when unrelated advanced settings and selectors are invalid.
    draft.rebuildScope = app::RebuildScope::Plugin;
    draft.navigationProfile.agentRadius = -1;
    draft.surfaceSearchRadius = -1;
    draft.neighboringCellRadius = -1;
    draft.workingMemoryMiB = 0;
    draft.workers = 0;
    draft.affectedPlugin.clear();
    const auto listing = ui::PrepareDesktopOptions(draft, true);
    assert(listing.listCells && !listing.generatePlugin && !listing.generateCandidate && !listing.copyPlugin);
    assert(!listing.estimateOnly && !listing.cellFormId && listing.editorId.empty());
    assert(listing.rebuildScope == app::RebuildScope::Cell);

    // Reset also repairs hidden values before the draft enters the shared run path.
    draft.cellFormId = 0x100;
    draft.editorId.clear();
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
    const auto resetRun = ui::PrepareDesktopOptions(draft, false);
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
