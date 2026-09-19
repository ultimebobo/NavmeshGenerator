#include "skyrim/extraction/cell_extractor.h"

#include <RE/Skyrim.h>

namespace
{
    [[nodiscard]] navmesh::core::Vec3 ToVec3(const RE::NiPoint3& point) { return { point.x, point.y, point.z }; }
    [[nodiscard]] const char* NonNull(const char* value) { return value ? value : ""; }
}

namespace navmesh::skyrim
{
    core::Cell ExtractCell(const RE::TESObjectCELL& gameCell)
    {
        core::Cell cell{ .id = gameCell.GetFormID(), .editorId = NonNull(gameCell.GetFormEditorID()), .name = NonNull(gameCell.GetName()), .isInterior = gameCell.IsInteriorCell() };
        if (!gameCell.IsInteriorCell()) {
            if (const auto* coordinates = gameCell.cellData.exterior) cell.exteriorCoordinates = { coordinates->cellX, coordinates->cellY };
        }
        gameCell.ForEachReference([&cell](RE::TESObjectREFR* reference) {
            if (!reference) return RE::BSContainer::ForEachResult::kContinue;
            const auto* base = reference->GetBaseObject();
            RE::NiTransform transform; reference->GetTransform(transform);
            core::Reference result{ .id = reference->GetFormID(), .baseObjectId = base ? base->GetFormID() : 0, .name = NonNull(reference->GetDisplayFullName()), .position = ToVec3(transform.translate), .rotation = { reference->GetAngleX(), reference->GetAngleY(), reference->GetAngleZ() }, .scale = transform.scale };
            if (base) {
                const auto& bounds = base->boundData;
                result.localBounds = core::AABB{ .min = { static_cast<float>(bounds.boundMin.x), static_cast<float>(bounds.boundMin.y), static_cast<float>(bounds.boundMin.z) }, .max = { static_cast<float>(bounds.boundMax.x), static_cast<float>(bounds.boundMax.y), static_cast<float>(bounds.boundMax.z) } };
            }
            cell.references.push_back(std::move(result));
            return RE::BSContainer::ForEachResult::kContinue;
        });
        if (!gameCell.navMeshes) return cell;
        for (const auto& meshPointer : gameCell.navMeshes->navMeshes) {
            const auto* mesh = meshPointer.get(); if (!mesh) continue;
            core::NavMesh result{ .id = mesh->GetFormID() };
            result.vertices.reserve(mesh->vertices.size());
            for (const auto& vertex : mesh->vertices) result.vertices.push_back(ToVec3(vertex.location));
            result.polygons.reserve(mesh->triangles.size());
            for (const auto& triangle : mesh->triangles)
                result.polygons.push_back({ .vertices = { triangle.vertices[0], triangle.vertices[1], triangle.vertices[2] }, .neighbors = { triangle.triangles[0], triangle.triangles[1], triangle.triangles[2] }, .flags = triangle.triangleFlags.underlying() });
            cell.navMeshes.push_back(std::move(result));
        }
        return cell;
    }
}
