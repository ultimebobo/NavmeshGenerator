#pragma once

#include <cmath>

namespace navmesh::skyrim::detail
{
    /// Finite CELL/WRLD water elevations in Skyrim world units exclude the engine's default-height sentinels.
    inline bool IsExplicitWaterHeight(float height)
    {
        return std::isfinite(height) && height > -2147483648.0F && height < 2147483648.0F;
    }

    /// A finite sentinel requests the owning world's water; malformed non-finite data supplies no plane.
    inline bool UsesDefaultWaterHeight(float height)
    {
        return std::isfinite(height) && !IsExplicitWaterHeight(height);
    }
} // namespace navmesh::skyrim::detail
