#include "validation/validation.h"

#include <format>

namespace navmesh::validation
{
    std::vector<Finding> Validate(const core::Cell &cell)
    {
        std::vector<Finding> findings;
        for (const auto &mesh : cell.navMeshes)
        {
            for (std::size_t index = 0; index < mesh.polygons.size(); ++index)
            {
                for (const auto vertex : mesh.polygons[index].vertices)
                {
                    if (vertex >= mesh.vertices.size())
                    {
                        findings.push_back(
                            {Severity::error,
                             std::format("NAVM {:08X} polygon {} references vertex {} outside vertex array", mesh.id,
                                         index, vertex)});
                    }
                }
            }
        }
        if (cell.navMeshes.empty())
        {
            findings.push_back({Severity::warning, "No runtime NAVM meshes are attached to this cell."});
        }
        return findings;
    }
} // namespace navmesh::validation
