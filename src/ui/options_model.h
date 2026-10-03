#pragma once

#include "app/options.h"

namespace navmesh::ui
{
    /// Supported MO2 CELL identifiers in the desktop workflow.
    enum class CellIdentification
    {
        FormId,
        EditorId
    };

    /** Prepare a desktop action using only settings relevant to its current scope.
     * @param draft Persisted desktop choices; hidden choices remain in the draft for later use.
     * @param identification Identifier to use for Cell scope; the other identifier is cleared.
     * @param listOnly Export the cell catalog without generation, selection or estimation.
     * @return MO2-only shared-run inputs, with dependent generation flags resolved.
     * @throws std::invalid_argument When required visible MO2, output or target inputs are missing.
     */
    [[nodiscard]] app::Options PrepareDesktopOptions(const app::Options &draft, CellIdentification identification,
                                                     bool listOnly);
} // namespace navmesh::ui
