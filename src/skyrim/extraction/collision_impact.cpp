#include "skyrim/extraction/collision_impact.h"

#include "skyrim/extraction/terrain_extractor.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <format>
#include <iterator>
#include <limits>
#include <map>
#include <stdexcept>

namespace navmesh::skyrim
{
    namespace
    {
        std::string Normalize(std::string text)
        {
            std::transform(text.begin(), text.end(), text.begin(),
                           [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
            std::replace(text.begin(), text.end(), '\\', '/');
            return text;
        }

        using CollisionTriangle = std::array<float, 9>;

        struct PlacedCollision
        {
            const RecordOrigin *origin{};
            std::vector<CollisionTriangle> triangles;
            bool obstacle{};
        };

        // Canonical cyclic ordering preserves winding while ignoring vertex indices and source triangle order.
        CollisionTriangle CanonicalTriangle(const core::Mesh &mesh, const core::Triangle &triangle)
        {
            CollisionTriangle best;
            for (std::size_t start{}; start < 3; ++start)
            {
                CollisionTriangle candidate;
                for (std::size_t corner{}; corner < 3; ++corner)
                {
                    const auto &vertex = mesh.vertices.at(triangle.vertices[(start + corner) % 3]);
                    candidate[corner * 3] = vertex.x;
                    candidate[corner * 3 + 1] = vertex.y;
                    candidate[corner * 3 + 2] = vertex.z;
                }
                if (start == 0 || candidate < best)
                {
                    best = candidate;
                }
            }
            return best;
        }

        /// Selection state owns plugin cutoffs, compared transitions and accumulated unique CELL identities.
        class Selection
        {
          public:
            Selection(const ResolvedLoadOrder &resolved, const CellImpactIndex &index,
                      const CollisionImpactInput &input, ModelGeometryCache &cache,
                      ImpactSelectionStatistics &statistics)
                : resolved_(resolved), index_(index), input_(input), cache_(cache), statistics_(statistics)
            {
                for (std::size_t rank{}; rank < resolved.plugins.size(); ++rank)
                {
                    ranks_.emplace(Normalize(resolved.plugins[rank]), rank);
                    if ((!input.plugin.empty() && Normalize(input.plugin) == Normalize(resolved.plugins[rank])) ||
                        (input.plugin.empty() && rank > 0))
                    {
                        selected_.insert(rank);
                    }
                }
                if (!input.plugin.empty() && selected_.empty())
                {
                    throw std::invalid_argument("Rebuild plugin is not active in the resolved load order: " +
                                                input.plugin);
                }
                statistics_ = {};
            }

            std::vector<const core::Cell *> Run()
            {
                // Terrain changes select their owning CELL. Water changes compare effective CELL inputs separately.
                std::map<std::uint32_t, std::set<std::size_t>> changedBases;
                std::set<std::size_t> waterCutoffs;
                for (const auto &record : resolved_.records)
                {
                    CheckCancellation();
                    bool changed{};
                    bool contributed{};
                    for (const auto &origin : record.origins)
                    {
                        const auto rank = Rank(origin);
                        if (!selected_.contains(rank))
                        {
                            continue;
                        }
                        contributed = true;
                        if (!origin.navigationChanged)
                        {
                            continue;
                        }
                        changed = true;
                        if (record.type == "LAND")
                        {
                            const auto *previous = OriginAt(record, rank == 0 ? std::nullopt : std::optional(rank - 1));
                            if ((!previous && !origin.terrainHeights.empty()) ||
                                (previous &&
                                 (origin.terrainHeights != previous->terrainHeights ||
                                  origin.deleted != previous->deleted || origin.cellFormId != previous->cellFormId)))
                            {
                                AddTerrainOwner(origin);
                                if (previous)
                                {
                                    AddTerrainOwner(*previous);
                                }
                            }
                        }
                        else if (record.type == "CELL" || record.type == "WRLD")
                        {
                            waterCutoffs.insert(rank);
                        }
                        else if (record.type == "REFR" || record.type == "ACHR")
                        {
                            if (!input_.terrainOnly)
                            {
                                Compare(record, rank);
                            }
                        }
                        else if (record.type != "NAVM")
                        {
                            changedBases[record.formId].insert(rank);
                        }
                    }
                    if (changed)
                    {
                        ++statistics_.changedRecords[record.type];
                    }
                    else if (contributed)
                    {
                        ++statistics_.equivalentRecords;
                    }
                }

                // Base models and replacement assets can change collision without an overriding placed record.
                for (const auto &record : resolved_.records)
                {
                    if (input_.terrainOnly || (record.type != "REFR" && record.type != "ACHR"))
                    {
                        continue;
                    }
                    CheckCancellation();
                    std::set<std::size_t> cutoffs;
                    for (const auto &origin : record.origins)
                    {
                        if (origin.baseFormId && changedBases.contains(*origin.baseFormId))
                        {
                            const auto &ranks = changedBases.at(*origin.baseFormId);
                            cutoffs.insert(ranks.begin(), ranks.end());
                        }
                    }
                    if (!cutoffs.empty())
                    {
                        ++statistics_.baseObjectUses;
                        for (const auto cutoff : cutoffs)
                        {
                            Compare(record, cutoff);
                        }
                    }
                    auto placement = ReferenceAt(record, resolved_.plugins.size());
                    const auto model = placement ? Normalize(placement->modelPath) : "";
                    const auto logical = model.starts_with("meshes/") ? model : "meshes/" + model;
                    if (!model.empty() && (input_.archiveModelsChanged || input_.changedModels.contains(logical)))
                    {
                        ++statistics_.assetUses;
                        const auto *origin = OriginAt(record, resolved_.plugins.size());
                        auto before = Extract(record, origin, resolved_.plugins.size(), true);
                        auto after = Extract(record, origin, resolved_.plugins.size(), false);
                        AddDifference(before, after);
                    }
                }

                for (const auto cutoff : waterCutoffs)
                {
                    for (const auto &cell : resolved_.cells)
                    {
                        CheckCancellation();
                        const auto *record = resolved_.FindWinning(cell.id);
                        if (!cell.isInterior && record &&
                            WaterAt(*record, cutoff == 0 ? std::nullopt : std::optional(cutoff - 1)) !=
                                WaterAt(*record, cutoff))
                        {
                            // Water is a triangle classification input; it does not supply a navigable floor.
                            if (targets_.contains(cell.id) || HasWinningGeometry(cell))
                            {
                                waterCells_.insert(cell.id);
                                targets_.insert(cell.id);
                            }
                            else
                            {
                                ignoredWaterCells_.insert(cell.id);
                            }
                        }
                    }
                }
                std::vector<const core::Cell *> result;
                for (const auto &cell : resolved_.cells)
                {
                    if (targets_.contains(cell.id))
                    {
                        result.push_back(&cell);
                    }
                }
                std::sort(result.begin(), result.end(),
                          [&](const auto *left, const auto *right)
                          {
                              const auto *a = resolved_.FindWinning(left->id);
                              const auto *b = resolved_.FindWinning(right->id);
                              return std::tuple{!left->exteriorCoordinates.has_value(), a->worldspaceFormId,
                                                left->exteriorCoordinates, left->id} <
                                     std::tuple{!right->exteriorCoordinates.has_value(), b->worldspaceFormId,
                                                right->exteriorCoordinates, right->id};
                          });
                statistics_.terrainCells = terrainCells_.size();
                statistics_.waterCells = waterCells_.size();
                statistics_.collisionCells = collisionCells_.size();
                statistics_.ignoredWaterCells = ignoredWaterCells_.size();
                return result;
            }

          private:
            std::size_t Rank(const RecordOrigin &origin) const
            {
                return ranks_.at(Normalize(origin.plugin));
            }

            const RecordOrigin *OriginAt(const ResolvedRecord &record, std::optional<std::size_t> cutoff) const
            {
                if (cutoff)
                {
                    for (auto origin = record.origins.rbegin(); origin != record.origins.rend(); ++origin)
                    {
                        if (Rank(*origin) <= *cutoff)
                        {
                            return &*origin;
                        }
                    }
                }
                return nullptr;
            }

            std::optional<core::Reference> ReferenceAt(const ResolvedRecord &record, std::size_t cutoff) const
            {
                const auto *origin = OriginAt(record, cutoff);
                if (!origin)
                {
                    return std::nullopt;
                }
                core::Reference reference{.id = record.formId,
                                          .recordType = record.type,
                                          .modelPath = origin->modelPath,
                                          .position = origin->position.value_or(core::Vec3{}),
                                          .rotation = origin->rotation,
                                          .scale = origin->scale,
                                          .sourcePlugin = origin->plugin,
                                          .initiallyDisabled = origin->initiallyDisabled,
                                          .deleted = origin->deleted};
                if (origin->baseFormId)
                {
                    reference.baseObjectId = *origin->baseFormId;
                    if (const auto *base = resolved_.FindWinning(*origin->baseFormId))
                    {
                        reference.baseRecordType = base->type;
                        if (const auto *version = OriginAt(*base, cutoff))
                        {
                            reference.modelPath = version->modelPath.empty() ? reference.modelPath : version->modelPath;
                            reference.basePlugin = version->plugin;
                            reference.deleted = reference.deleted || version->deleted;
                        }
                    }
                }
                return reference;
            }

            PlacedCollision Extract(const ResolvedRecord &record, const RecordOrigin *origin, std::size_t cutoff,
                                    bool previousAssets)
            {
                PlacedCollision result{.origin = origin};
                const auto reference = ReferenceAt(record, cutoff);
                if (!origin || !reference || reference->modelPath.empty() || reference->deleted ||
                    reference->initiallyDisabled)
                {
                    return result;
                }
                const core::Cell source{.references = {*reference}};
                auto geometry = ExtractGeometry(input_.dataDirectory, source,
                                                previousAssets ? input_.previousCacheDirectory : input_.cacheDirectory,
                                                {}, input_.cancelled,
                                                previousAssets ? input_.previousAssets : input_.assets, &cache_, true);
                CheckCancellation();
                if (geometry.modelsUnreadable || !geometry.archiveSearchComplete)
                {
                    throw std::runtime_error(std::format("Cannot determine collision impact for REFR {:08X}: {} ({})",
                                                         record.formId, reference->modelPath,
                                                         geometry.archiveSearchComplete
                                                             ? geometry.references.front().failure
                                                             : "archive model search incomplete"));
                }
                if (geometry.modelsMissing)
                {
                    // A completed provider search establishes absence; missing models supply no collision.
                    if (!previousAssets)
                    {
                        auto path = reference->modelPath;
                        std::replace(path.begin(), path.end(), '\\', '/');
                        std::transform(path.begin(), path.end(), path.begin(),
                                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                        if (!path.starts_with("meshes/"))
                        {
                            path.insert(0, "meshes/");
                        }
                        statistics_.missingModels.insert(std::move(path));
                    }
                    return result;
                }
                for (std::size_t triangle{}; triangle < geometry.scene.mesh.triangles.size(); ++triangle)
                {
                    const auto &geometrySource = geometry.scene.geometrySources.at(
                        geometry.scene.triangleProvenance.at(triangle).geometrySource);
                    if (geometrySource.sourceType == core::GeometrySourceType::Collision)
                    {
                        result.obstacle = geometrySource.navigationObstacle;
                        result.triangles.push_back(
                            CanonicalTriangle(geometry.scene.mesh, geometry.scene.mesh.triangles[triangle]));
                    }
                }
                std::sort(result.triangles.begin(), result.triangles.end());
                return result;
            }

            void Compare(const ResolvedRecord &record, std::size_t cutoff)
            {
                if (!compared_.emplace(record.formId, cutoff).second)
                {
                    return;
                }
                const auto *previous = OriginAt(record, cutoff == 0 ? std::nullopt : std::optional(cutoff - 1));
                const auto *current = OriginAt(record, cutoff);
                auto before = previous ? Extract(record, previous, cutoff - 1, false) : PlacedCollision{};
                auto after = Extract(record, current, cutoff, false);
                if (AddDifference(before, after))
                {
                    // Later winners can move the surviving collider to another physical CELL.
                    const auto winning = Extract(record, &record.winning, resolved_.plugins.size(), false);
                    if (winning.triangles != after.triangles || winning.obstacle != after.obstacle ||
                        !SameSpace(winning.origin, after.origin))
                    {
                        Add(winning);
                    }
                }
            }

            bool AddDifference(const PlacedCollision &before, const PlacedCollision &after)
            {
                ++statistics_.collisionComparisons;
                const bool sameOwner = SameSpace(before.origin, after.origin);
                if (before.obstacle == after.obstacle && sameOwner && before.triangles == after.triangles)
                {
                    ++statistics_.equivalentCollisionComparisons;
                    return false;
                }
                if (before.obstacle != after.obstacle || !sameOwner)
                {
                    Add(before);
                    Add(after);
                }
                else
                {
                    // Only changed triangles contribute footprints; retained geometry cannot enlarge selection.
                    AddRemoved(before, after);
                    AddRemoved(after, before);
                }
                return true;
            }

            void AddRemoved(const PlacedCollision &from, const PlacedCollision &other)
            {
                std::vector<CollisionTriangle> difference;
                std::set_difference(from.triangles.begin(), from.triangles.end(), other.triangles.begin(),
                                    other.triangles.end(), std::back_inserter(difference));
                Add({.origin = from.origin, .triangles = std::move(difference)});
            }

            void Add(const PlacedCollision &collision)
            {
                const auto cells = CollisionFootprint(collision);
                collisionCells_.insert(cells.begin(), cells.end());
                targets_.insert(cells.begin(), cells.end());
            }

            /// Project supported placed triangles into CELLs without using model bounds as selection evidence.
            std::set<std::uint32_t> CollisionFootprint(const PlacedCollision &collision) const
            {
                std::set<std::uint32_t> cells;
                if (!collision.origin)
                {
                    return cells;
                }
                for (const auto &triangle : collision.triangles)
                {
                    core::AABB bounds;
                    for (std::size_t corner{}; corner < 3; ++corner)
                    {
                        bounds.Expand({triangle[corner * 3], triangle[corner * 3 + 1], triangle[corner * 3 + 2]});
                    }
                    for (const auto *cell : index_.IntersectingCells(*collision.origin, bounds))
                    {
                        if (TriangleTouchesCell(triangle, *cell))
                        {
                            cells.insert(cell->id);
                        }
                    }
                }
                return cells;
            }

            /// Heightfield transitions select their owner even when no navigation has been authored.
            void AddTerrainOwner(const RecordOrigin &origin)
            {
                if (!origin.cellFormId)
                {
                    return;
                }
                const auto *cell = resolved_.FindWinning(*origin.cellFormId);
                if (!cell)
                {
                    return;
                }
                terrainCells_.insert(*origin.cellFormId);
                targets_.insert(*origin.cellFormId);
            }

            /** Water tags existing support geometry; a water plane alone cannot create a floor.
             * Terrain evidence uses the shared LAND decoder rather than CELL or NAVM metadata.
             * Model bounds discover suppliers only. Each winning reference is decoded once, and its exact
             * horizontal collision footprint is retained for other water candidates in the same selection.
             */
            bool HasWinningGeometry(const core::Cell &cell)
            {
                if (ExtractTerrain(resolved_, cell).landRecordsDecoded != 0)
                {
                    return true;
                }
                if (input_.terrainOnly || !cell.exteriorCoordinates)
                {
                    return false;
                }
                if (supportCells_.contains(cell.id))
                {
                    return true;
                }
                const auto [x, y] = *cell.exteriorCoordinates;
                const core::AABB bounds{.min = {x * 4096.0F, y * 4096.0F, std::numeric_limits<float>::lowest()},
                                        .max = {(static_cast<float>(x) + 1) * 4096.0F,
                                                (static_cast<float>(y) + 1) * 4096.0F,
                                                std::numeric_limits<float>::max()}};
                for (const auto *supplier : index_.GeometryNeighbors(cell, 0))
                {
                    for (const auto &reference : index_.GeometryCell(*supplier, bounds).references)
                    {
                        CheckCancellation();
                        if (!supportReferences_.insert(reference.id).second)
                        {
                            continue;
                        }
                        const auto *record = resolved_.FindWinning(reference.id);
                        if (!record)
                        {
                            continue;
                        }
                        const auto collision = Extract(*record, &record->winning, resolved_.plugins.size(), false);
                        const auto cells = CollisionFootprint(collision);
                        supportCells_.insert(cells.begin(), cells.end());
                        if (supportCells_.contains(cell.id))
                        {
                            return true;
                        }
                    }
                }
                return false;
            }

            std::optional<float> WaterAt(const ResolvedRecord &cell, std::optional<std::size_t> cutoff) const
            {
                const auto *origin = OriginAt(cell, cutoff);
                if (!origin || origin->deleted || !origin->hasWater)
                {
                    return std::nullopt;
                }
                if (!origin->usesWorldWater)
                {
                    return origin->waterHeight;
                }
                auto worldId = origin->worldspaceFormId;
                std::set<std::uint32_t> visited;
                while (worldId && visited.insert(*worldId).second)
                {
                    const auto *world = resolved_.FindWinning(*worldId);
                    const auto *version = world ? OriginAt(*world, cutoff) : nullptr;
                    if (!version || version->deleted)
                    {
                        return std::nullopt;
                    }
                    if (!version->inheritsParentWater || !version->parentWorldFormId)
                    {
                        return version->waterHeight;
                    }
                    worldId = version->parentWorldFormId;
                }
                return std::nullopt;
            }

            bool SameSpace(const RecordOrigin *left, const RecordOrigin *right) const
            {
                if (!left || !right)
                {
                    return true;
                }
                const auto space = [&](const RecordOrigin &origin)
                {
                    auto world = origin.worldspaceFormId;
                    if (!world && origin.cellFormId)
                    {
                        const auto *cell = resolved_.FindWinning(*origin.cellFormId);
                        world = cell ? cell->worldspaceFormId : std::nullopt;
                    }
                    return std::pair{world.has_value(), world ? world : origin.cellFormId};
                };
                return space(*left) == space(*right);
            }

            static bool TriangleTouchesCell(const CollisionTriangle &triangle, const core::Cell &cell)
            {
                if (!cell.exteriorCoordinates)
                {
                    return true;
                }
                const auto [x, y] = *cell.exteriorCoordinates;
                const double centerX = (static_cast<double>(x) + 0.5) * 4096;
                const double centerY = (static_cast<double>(y) + 0.5) * 4096;
                // The bounds query supplies the rectangle axes. Edge-normal separation rejects empty AABB corners,
                // including degenerate horizontal projections of vertical collision walls.
                for (std::size_t edge{}; edge < 3; ++edge)
                {
                    const auto next = (edge + 1) % 3;
                    const double axisX = triangle[next * 3 + 1] - triangle[edge * 3 + 1];
                    const double axisY = triangle[edge * 3] - triangle[next * 3];
                    double low = std::numeric_limits<double>::max();
                    double high = std::numeric_limits<double>::lowest();
                    for (std::size_t corner{}; corner < 3; ++corner)
                    {
                        const double projection = triangle[corner * 3] * axisX + triangle[corner * 3 + 1] * axisY;
                        low = std::min(low, projection);
                        high = std::max(high, projection);
                    }
                    const double center = centerX * axisX + centerY * axisY;
                    const double extent = 2048 * (std::abs(axisX) + std::abs(axisY));
                    if (low > center + extent || high < center - extent)
                    {
                        return false;
                    }
                }
                return true;
            }

            void CheckCancellation() const
            {
                if (input_.cancelled && input_.cancelled())
                {
                    throw std::runtime_error("Collision impact selection cancelled");
                }
            }

            const ResolvedLoadOrder &resolved_;
            const CellImpactIndex &index_;
            const CollisionImpactInput &input_;
            ModelGeometryCache &cache_;
            ImpactSelectionStatistics &statistics_;
            std::map<std::string, std::size_t> ranks_;
            std::set<std::size_t> selected_;
            std::set<std::pair<std::uint32_t, std::size_t>> compared_;
            std::set<std::uint32_t> targets_;
            std::set<std::uint32_t> terrainCells_, waterCells_, collisionCells_, ignoredWaterCells_;
            std::set<std::uint32_t> supportCells_, supportReferences_;
        };
    } // namespace

    std::vector<const core::Cell *> SelectCollisionAffectedCells(const ResolvedLoadOrder &resolved,
                                                                 const CellImpactIndex &index,
                                                                 const CollisionImpactInput &input,
                                                                 ModelGeometryCache &cache,
                                                                 ImpactSelectionStatistics &statistics)
    {
        return Selection(resolved, index, input, cache, statistics).Run();
    }
} // namespace navmesh::skyrim
