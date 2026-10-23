#pragma once

#include "app/options.h"

namespace navmesh::ui
{
    /** Restore the advanced numerical inputs from the shared application defaults.
     * @param draft Desktop choices updated in place, including hidden numerical controls.
     * Resets movement, voxel, contour, analysis, neighboring-cell and resource-budget
     * values. Paths, target selection, output choices, toggles and partitioning remain
     * selected. The desktop saves the resulting draft through its normal persistence path.
     */
    void ResetAdvancedNumericalOptions(app::Options &draft);

    /** Prepare a desktop action using only settings relevant to its current scope.
     * @param draft Persisted desktop choices; hidden choices remain in the draft for later use.
     * @param listOnly Export the cell catalog without generation, selection or estimation.
     * @return Shared-run inputs for the selected manager, with dependent generation flags resolved.
     * MO2 resolves mod storage from its INI; Vortex reads the active deployment and game plugin list.
     * Desktop actions clear developer directory overrides and the unselected manager's inputs.
     * Cell scope retains its explicit list and enabled copy source; hidden cell lists are cleared in other scopes.
     * Make scene is retained for Cell and Plugin runs and clears cost estimation; shared-run selection validates
     * the Plugin scene size against app::MaximumPluginSceneCells before extraction.
     * Generation retains and validates the persisted cell-error policy for the shared run path; listing resets it.
     * @throws std::invalid_argument When the manager is invalid or required visible source, output or target inputs are missing.
     */
    [[nodiscard]] app::Options PrepareDesktopOptions(const app::Options &draft, bool listOnly);
} // namespace navmesh::ui
