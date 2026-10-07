#pragma once

#include "core/geometry/types.h"

#include <array>
#include <span>
#include <vector>

namespace navmesh::core
{
    /** Triangulate a sampled floor without changing its boundary or sample positions.
     * @param points Finite floor samples with unique XY positions in a shared Z-up world coordinate space.
     * @param boundary Simple counterclockwise boundary indices, including collinear samples.
     * Other points must lie inside this boundary; heights belong to the same floor layer.
     * @return Counterclockwise triangles indexing points, with every sample represented.
     * @throws std::runtime_error For an untriangulable boundary, duplicate or nonfinite samples,
     * or unsupported sample placement.
     */
    [[nodiscard]] std::vector<std::array<std::uint32_t, 3>> TriangulateDetailSamples(
        std::span<const Vec3> points, std::span<const std::uint32_t> boundary);
} // namespace navmesh::core
