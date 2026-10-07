#include "skyrim/parser/affected_cells.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace navmesh::skyrim
{
    namespace
    {
        std::string Lower(std::string text)
        {
            std::transform(text.begin(), text.end(), text.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            return text;
        }
    } // namespace
    CellImpactIndex::CellImpactIndex(const ResolvedLoadOrder &resolved) : resolved_(resolved)
    {
        for (const auto &cell : resolved.cells)
        {
            cells_.emplace(cell.id, &cell);
            const auto *record = resolved.FindWinning(cell.id);
            if (record && record->worldspaceFormId)
            {
                worlds_.emplace(cell.id, *record->worldspaceFormId);
                if (cell.exteriorCoordinates)
                {
                    exterior_.emplace(
                        Key{*record->worldspaceFormId, (*cell.exteriorCoordinates)[0], (*cell.exteriorCoordinates)[1]},
                        &cell);
                }
            }
        }
        for (const auto &cell : resolved.cells)
        {
            for (const auto &reference : cell.references)
            {
                const auto *record = resolved.FindWinning(reference.id);
                const auto *physical = record ? PhysicalCell(record->winning) : nullptr;
                references_[physical ? physical->id : cell.id].push_back(&reference);
                if (record && physical && physical->exteriorCoordinates && !reference.modelPath.empty() &&
                    !reference.deleted && !reference.initiallyDisabled)
                {
                    const auto *base = resolved.FindWinning(reference.baseObjectId);
                    const bool bounded = base && base->winning.modelRadius &&
                                         std::isfinite(*base->winning.modelRadius) && *base->winning.modelRadius >= 0 &&
                                         std::isfinite(reference.scale) && base->modelPath &&
                                         Lower(*base->modelPath) == Lower(reference.modelPath);
                    if (!bounded && worlds_.contains(physical->id))
                    {
                        unboundedSources_[worlds_.at(physical->id)].insert(physical->id);
                    }
                    else if (bounded)
                    {
                        const auto radius = static_cast<int>(std::min<double>(
                            std::numeric_limits<int>::max(),
                            std::ceil(*base->winning.modelRadius * std::abs(reference.scale) / 4096.0)));
                        for (const auto *target : Neighbors(*physical, radius))
                        {
                            geometrySources_[target->id].insert(physical->id);
                        }
                    }
                }
            }
        }
    }
    const core::Cell *CellImpactIndex::PhysicalCell(const RecordOrigin &origin) const
    {
        auto world = origin.worldspaceFormId;
        if (!world && origin.cellFormId)
        {
            if (const auto it = worlds_.find(*origin.cellFormId); it != worlds_.end())
            {
                world = it->second;
            }
        }
        if (world && origin.position && std::isfinite(origin.position->x) && std::isfinite(origin.position->y))
        {
            const auto x = std::floor(static_cast<double>(origin.position->x) / 4096.0);
            const auto y = std::floor(static_cast<double>(origin.position->y) / 4096.0);
            if (x >= std::numeric_limits<std::int32_t>::min() && x <= std::numeric_limits<std::int32_t>::max() &&
                y >= std::numeric_limits<std::int32_t>::min() && y <= std::numeric_limits<std::int32_t>::max())
            {
                if (const auto it =
                        exterior_.find({*world, static_cast<std::int32_t>(x), static_cast<std::int32_t>(y)});
                    it != exterior_.end())
                {
                    return it->second;
                }
            }
        }
        if (origin.cellFormId)
        {
            if (const auto it = cells_.find(*origin.cellFormId); it != cells_.end())
            {
                return it->second;
            }
        }
        return nullptr;
    }
    std::vector<const core::Cell *> CellImpactIndex::Neighbors(const core::Cell &cell, int radius) const
    {
        if (radius < 0)
        {
            throw std::invalid_argument("Neighboring-cell radius cannot be negative");
        }
        std::vector<const core::Cell *> result{&cell};
        const auto world = worlds_.find(cell.id);
        if (!cell.exteriorCoordinates || world == worlds_.end())
        {
            return result;
        }
        const auto [x, y] = *cell.exteriorCoordinates;
        const auto low = static_cast<std::int32_t>(
            std::max<std::int64_t>(std::numeric_limits<std::int32_t>::min(), static_cast<std::int64_t>(x) - radius));
        const auto high = static_cast<std::int32_t>(
            std::min<std::int64_t>(std::numeric_limits<std::int32_t>::max(), static_cast<std::int64_t>(x) + radius));
        for (auto it = exterior_.lower_bound({world->second, low, std::numeric_limits<std::int32_t>::min()});
             it != exterior_.end(); ++it)
        {
            const auto [owner, cx, cy] = it->first;
            if (owner != world->second || cx > high)
            {
                break;
            }
            if (it->second->id != cell.id && std::abs(static_cast<std::int64_t>(cy) - y) <= radius)
            {
                result.push_back(it->second);
            }
        }
        return result;
    }
    core::Cell CellImpactIndex::GeometryCell(const core::Cell &cell, std::optional<core::AABB> bounds) const
    {
        core::Cell result{.id = cell.id,
                          .editorId = cell.editorId,
                          .name = cell.name,
                          .isInterior = cell.isInterior,
                          .exteriorCoordinates = cell.exteriorCoordinates};
        if (const auto it = references_.find(cell.id); it != references_.end())
        {
            for (const auto *reference : it->second)
            {
                const auto *base = resolved_.FindWinning(reference->baseObjectId);
                if (bounds && base && base->winning.modelRadius && std::isfinite(*base->winning.modelRadius) &&
                    *base->winning.modelRadius >= 0 && std::isfinite(reference->scale) &&
                    std::isfinite(reference->position.x) && std::isfinite(reference->position.y) &&
                    std::isfinite(reference->position.z) && base->modelPath &&
                    Lower(*base->modelPath) == Lower(reference->modelPath))
                {
                    const float radius = *base->winning.modelRadius * std::abs(reference->scale);
                    const core::Vec3 extent{radius, radius, radius};
                    const core::AABB placed{.min = reference->position - extent, .max = reference->position + extent};
                    if (!placed.Intersects(*bounds))
                    {
                        continue;
                    }
                }
                result.references.push_back(*reference);
            }
        }
        return result;
    }
    std::vector<const core::Cell *> CellImpactIndex::IntersectingCells(const RecordOrigin &origin,
                                                                       const core::AABB &bounds) const
    {
        const auto *physical = PhysicalCell(origin);
        if (!physical || !bounds.IsValid() || !std::isfinite(bounds.min.x) || !std::isfinite(bounds.min.y) ||
            !std::isfinite(bounds.max.x) || !std::isfinite(bounds.max.y))
        {
            return {};
        }
        if (!physical->exteriorCoordinates || !worlds_.contains(physical->id))
        {
            return {physical};
        }
        // Query the horizontal projection of actual collision; vertical size does not enlarge the grid footprint.
        const auto world = worlds_.at(physical->id);
        const auto lowX =
            static_cast<std::int32_t>(std::clamp(std::ceil(static_cast<double>(bounds.min.x) / 4096) - 1,
                                                 static_cast<double>(std::numeric_limits<std::int32_t>::min()),
                                                 static_cast<double>(std::numeric_limits<std::int32_t>::max())));
        std::vector<const core::Cell *> result;
        for (auto entry = exterior_.lower_bound({world, lowX, std::numeric_limits<std::int32_t>::min()});
             entry != exterior_.end(); ++entry)
        {
            const auto &[key, cell] = *entry;
            const auto [owner, x, y] = key;
            if (owner != world || static_cast<double>(x) * 4096 > bounds.max.x)
            {
                break;
            }
            if ((static_cast<double>(y) + 1) * 4096 >= bounds.min.y && static_cast<double>(y) * 4096 <= bounds.max.y)
            {
                result.push_back(cell);
            }
        }
        return result;
    }
    std::vector<const core::Cell *> CellImpactIndex::Footprint(const RecordOrigin &origin, int radius) const
    {
        const auto *physical = PhysicalCell(origin);
        if (!physical)
        {
            return {};
        }
        if (!physical->exteriorCoordinates || !origin.baseFormId || !origin.position)
        {
            return Neighbors(*physical, radius);
        }
        const auto *base = resolved_.FindWinning(*origin.baseFormId);
        const bool hasModel =
            origin.hasModel || (base && ((base->modelPath && !base->modelPath->empty()) ||
                                         std::any_of(base->origins.begin(), base->origins.end(),
                                                     [](const auto &version) { return version.hasModel; })));
        if (!hasModel)
        {
            return Neighbors(*physical, radius);
        }
        auto modelRadius = origin.modelRadius;
        if (base)
        {
            for (const auto &version : base->origins)
            {
                if (version.modelRadius)
                {
                    modelRadius = std::max(modelRadius.value_or(0.0F), *version.modelRadius);
                }
            }
        }
        if (base && std::any_of(base->origins.begin(), base->origins.end(),
                                [](const auto &version) { return version.hasModel && !version.modelRadius; }))
        {
            modelRadius.reset();
        }
        if (modelRadius && std::isfinite(*modelRadius) && *modelRadius >= 0 && std::isfinite(origin.scale))
        {
            const auto extra = std::ceil(static_cast<double>(*modelRadius) * std::abs(origin.scale) / 4096.0);
            const auto total = std::min<double>(std::numeric_limits<int>::max(), static_cast<double>(radius) + extra);
            return Neighbors(*physical, static_cast<int>(total));
        }
        // Without model bounds no finite influence radius can be justified.
        // Retain the whole worldspace rather than silently omit possible targets.
        std::vector<const core::Cell *> result;
        const auto world = worlds_.find(physical->id);
        if (world != worlds_.end())
        {
            for (const auto &[key, cell] : exterior_)
            {
                if (std::get<0>(key) == world->second)
                {
                    result.push_back(cell);
                }
            }
        }
        return result;
    }
    std::vector<const core::Cell *> CellImpactIndex::GeometryNeighbors(const core::Cell &cell, int radius) const
    {
        const auto neighbors = Neighbors(cell, radius);
        std::set<std::uint32_t> sources;
        for (const auto *neighbor : neighbors)
        {
            sources.insert(neighbor->id);
            if (const auto it = geometrySources_.find(neighbor->id); it != geometrySources_.end())
            {
                sources.insert(it->second.begin(), it->second.end());
            }
        }
        if (const auto world = worlds_.find(cell.id); world != worlds_.end())
        {
            if (const auto it = unboundedSources_.find(world->second); it != unboundedSources_.end())
            {
                sources.insert(it->second.begin(), it->second.end());
            }
        }
        std::vector<const core::Cell *> result;
        for (const auto id : sources)
        {
            result.push_back(cells_.at(id));
        }
        return result;
    }
    std::vector<const core::Cell *> CellImpactIndex::AffectedCells(const std::string &plugin, int radius,
                                                                   const std::set<std::string> &changedModels,
                                                                   bool archiveModelsChanged,
                                                                   ImpactSelectionStatistics *statistics) const
    {
        ImpactSelectionStatistics counts;
        if (radius < 0)
        {
            throw std::invalid_argument("Impact radius cannot be negative");
        }
        const auto name = Lower(plugin);
        std::unordered_set<std::string> selected;
        if (!name.empty())
        {
            for (const auto &active : resolved_.plugins)
            {
                if (Lower(active) == name)
                {
                    selected.insert(name);
                }
            }
            if (selected.empty())
            {
                throw std::invalid_argument("Rebuild plugin is not active in the resolved load order: " + plugin);
            }
        }
        else
        {
            for (std::size_t i = 1; i < resolved_.plugins.size(); ++i)
            {
                selected.insert(Lower(resolved_.plugins[i]));
            }
        }
        std::unordered_set<std::uint32_t> changedBases;
        std::set<std::uint32_t> affected;
        const auto add = [&](const core::Cell *cell)
        {
            if (cell)
            {
                for (const auto *neighbor : Neighbors(*cell, std::max(1, radius)))
                {
                    affected.insert(neighbor->id);
                }
            }
        };
        for (const auto &record : resolved_.records)
        {
            if (std::none_of(record.origins.begin(), record.origins.end(), [&](const auto &origin)
                             { return origin.navigationChanged && selected.contains(Lower(origin.plugin)); }))
            {
                counts.equivalentRecords +=
                    std::any_of(record.origins.begin(), record.origins.end(),
                                [&](const auto &origin) { return selected.contains(Lower(origin.plugin)); })
                        ? 1
                        : 0;
                continue;
            }
            ++counts.changedRecords[record.type];
            if (record.type == "CELL")
            {
                if (const auto it = cells_.find(record.formId); it != cells_.end())
                {
                    add(it->second);
                }
            }
            else if (record.type == "WRLD")
            {
                for (const auto &[id, world] : worlds_)
                {
                    if (world == record.formId)
                    {
                        add(cells_.at(id));
                    }
                }
            }
            else if (record.type == "LAND" || record.type == "NAVM" || record.type == "REFR" || record.type == "ACHR")
            {
                // All versions are needed: deleting or moving a placed object affects
                // its old footprint as well as the winning physical location.
                for (const auto &origin : record.origins)
                {
                    for (const auto *target : Footprint(origin, std::max(1, radius)))
                    {
                        affected.insert(target->id);
                    }
                }
            }
            else
            {
                changedBases.insert(record.formId);
            }
        }
        for (const auto &record : resolved_.records)
        {
            if (record.type == "REFR" || record.type == "ACHR")
            {
                if (std::any_of(record.origins.begin(), record.origins.end(), [&](const auto &origin)
                                { return origin.baseFormId && changedBases.contains(*origin.baseFormId); }))
                {
                    ++counts.baseObjectUses;
                    for (const auto &origin : record.origins)
                    {
                        for (const auto *target : Footprint(origin, std::max(1, radius)))
                        {
                            affected.insert(target->id);
                        }
                    }
                }
            }
        }
        for (const auto &cell : resolved_.cells)
        {
            for (const auto &reference : cell.references)
            {
                auto model = Lower(reference.modelPath);
                std::replace(model.begin(), model.end(), '\\', '/');
                if (!model.starts_with("meshes/"))
                {
                    model = "meshes/" + model;
                }
                if (!reference.modelPath.empty() && (archiveModelsChanged || changedModels.contains(model)))
                {
                    ++counts.assetUses;
                    if (const auto *record = resolved_.FindWinning(reference.id))
                    {
                        for (const auto &origin : record->origins)
                        {
                            for (const auto *target : Footprint(origin, std::max(1, radius)))
                            {
                                affected.insert(target->id);
                            }
                        }
                    }
                }
            }
        }
        std::vector<const core::Cell *> result;
        for (const auto &[key, cell] : exterior_)
        {
            if (affected.erase(cell->id))
            {
                result.push_back(cell);
            }
        }
        for (const auto id : affected)
        {
            result.push_back(cells_.at(id));
        }
        if (statistics)
        {
            *statistics = std::move(counts);
        }
        return result;
    }
} // namespace navmesh::skyrim
