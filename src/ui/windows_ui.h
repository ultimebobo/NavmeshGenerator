#pragma once
#include "app/options.h"
namespace navmesh::ui
{
    /** Open the dark desktop workspace and run MO2 operations through app::Run.
     * @param initialOptions Initial choices, overridden by persisted desktop settings when present.
     * @return Process-style status; returns nonzero if the Windows/DirectX host cannot initialize.
     * @warning Closing during a run requests cancellation and waits for a safe completion boundary.
     */
    int RunWindowsUi(const app::Options &initialOptions);
} // namespace navmesh::ui
