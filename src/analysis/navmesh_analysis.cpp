#include "analysis/navmesh_analysis.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace
{
    [[nodiscard]] double TriangleArea(const navmesh::core::Vec3& a, const navmesh::core::Vec3& b, const navmesh::core::Vec3& c)
    {
        const auto ab = navmesh::core::Vec3{ b.x - a.x, b.y - a.y, b.z - a.z };
        const auto ac = navmesh::core::Vec3{ c.x - a.x, c.y - a.y, c.z - a.z };
        const auto crossX = ab.y * ac.z - ab.z * ac.y;
        const auto crossY = ab.z * ac.x - ab.x * ac.z;
        const auto crossZ = ab.x * ac.y - ab.y * ac.x;
        return 0.5 * std::sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ);
    }

    [[nodiscard]] int FindRoot(std::vector<int>& parents, int index)
    {
        while (parents[index] != index) {
            parents[index] = parents[parents[index]];
            index = parents[index];
        }
        return index;
    }

    void Union(std::vector<int>& parents, int a, int b)
    {
        const auto rootA = FindRoot(parents, a);
        const auto rootB = FindRoot(parents, b);
        if (rootA == rootB) {
            return;
        }
        parents[rootB] = rootA;
    }
}

namespace navmesh::analysis
{
    NavMeshAnalysis Analyze(const core::NavMesh& mesh)
    {
        NavMeshAnalysis analysis{};
        analysis.vertexCount = static_cast<std::uint32_t>(mesh.vertices.size());
        analysis.polygonCount = static_cast<std::uint32_t>(mesh.polygons.size());

        if (!mesh.vertices.empty()) {
            analysis.boundingBox.min = mesh.vertices.front();
            analysis.boundingBox.max = mesh.vertices.front();
            for (const auto& vertex : mesh.vertices) {
                analysis.boundingBox.min.x = std::min(analysis.boundingBox.min.x, vertex.x);
                analysis.boundingBox.min.y = std::min(analysis.boundingBox.min.y, vertex.y);
                analysis.boundingBox.min.z = std::min(analysis.boundingBox.min.z, vertex.z);
                analysis.boundingBox.max.x = std::max(analysis.boundingBox.max.x, vertex.x);
                analysis.boundingBox.max.y = std::max(analysis.boundingBox.max.y, vertex.y);
                analysis.boundingBox.max.z = std::max(analysis.boundingBox.max.z, vertex.z);
            }
        }

        std::vector<int> parents(analysis.polygonCount > 0 ? static_cast<std::size_t>(analysis.polygonCount) : 0);
        std::iota(parents.begin(), parents.end(), 0);

        for (std::uint32_t polygonIndex = 0; polygonIndex < analysis.polygonCount; ++polygonIndex) {
            const auto& polygon = mesh.polygons[polygonIndex];
            for (std::uint32_t compareIndex = polygonIndex + 1; compareIndex < analysis.polygonCount; ++compareIndex) {
                const auto& other = mesh.polygons[compareIndex];
                const bool sharesVertex = std::any_of(polygon.vertices.begin(), polygon.vertices.end(), [&](std::uint32_t value) {
                    return std::any_of(other.vertices.begin(), other.vertices.end(), [value](std::uint32_t otherValue) {
                        return value == otherValue;
                    });
                });
                if (sharesVertex) {
                    Union(parents, static_cast<int>(polygonIndex), static_cast<int>(compareIndex));
                }
            }
        }

        std::vector<int> componentSizes(analysis.polygonCount > 0 ? static_cast<std::size_t>(analysis.polygonCount) : 0, 0);
        for (std::uint32_t polygonIndex = 0; polygonIndex < analysis.polygonCount; ++polygonIndex) {
            const auto root = FindRoot(parents, static_cast<int>(polygonIndex));
            ++componentSizes[root];
        }

        analysis.connectedComponents = static_cast<std::uint32_t>(std::count_if(componentSizes.begin(), componentSizes.end(), [](int size) { return size > 0; }));
        analysis.isolatedPolygonCount = static_cast<std::uint32_t>(std::count_if(componentSizes.begin(), componentSizes.end(), [](int size) { return size == 1; }));

        double totalArea = 0.0;
        double minArea = std::numeric_limits<double>::max();
        double maxArea = 0.0;
        for (const auto& polygon : mesh.polygons) {
            const auto a = mesh.vertices[polygon.vertices[0]];
            const auto b = mesh.vertices[polygon.vertices[1]];
            const auto c = mesh.vertices[polygon.vertices[2]];
            const auto area = TriangleArea(a, b, c);
            if (polygon.vertices[0] >= mesh.vertices.size() || polygon.vertices[1] >= mesh.vertices.size() || polygon.vertices[2] >= mesh.vertices.size()) {
                ++analysis.degeneratePolygonCount;
                continue;
            }

            const bool degenerate = area <= 1.0e-6 || (a.x == b.x && a.y == b.y && a.z == b.z) || (a.x == c.x && a.y == c.y && a.z == c.z) || (b.x == c.x && b.y == c.y && b.z == c.z);
            if (degenerate) {
                ++analysis.degeneratePolygonCount;
            }
            minArea = std::min(minArea, area);
            maxArea = std::max(maxArea, area);
            totalArea += area;
        }

        if (analysis.polygonCount > 0) {
            analysis.minPolygonArea = minArea == std::numeric_limits<double>::max() ? 0.0 : minArea;
            analysis.maxPolygonArea = maxArea;
            analysis.averagePolygonArea = totalArea / static_cast<double>(analysis.polygonCount);
        }

        return analysis;
    }
}
