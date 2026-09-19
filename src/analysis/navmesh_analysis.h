#pragma once

#include "core/world/types.h"

#include <cstdint>

namespace navmesh::analysis
{
    struct NavMeshAnalysis
    {
        std::uint32_t vertexCount{};
        std::uint32_t polygonCount{};
        std::uint32_t connectedComponents{};
        std::uint32_t isolatedPolygonCount{};
        std::uint32_t degeneratePolygonCount{};
        core::AABB boundingBox{};
        double minPolygonArea{};
        double maxPolygonArea{};
        double averagePolygonArea{};
    };

    [[nodiscard]] NavMeshAnalysis Analyze(const core::NavMesh& mesh);
}
