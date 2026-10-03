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
