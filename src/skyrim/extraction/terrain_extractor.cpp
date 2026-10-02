#include "skyrim/extraction/terrain_extractor.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <format>

namespace
{
    [[nodiscard]] std::optional<std::vector<float>> DecodeVhgt(const navmesh::skyrim::offline::ResolvedRecord &record,
                                                               std::string &error)
    {
        if (!record.raw)
        {
            error = "LAND record payload was not retained";
            return std::nullopt;
        }
        const auto it = std::find_if(record.raw->subrecords.begin(), record.raw->subrecords.end(),
                                     [](const auto &sub) { return sub.type == "VHGT"; });
        if (it == record.raw->subrecords.end())
        {
            error = "LAND record has no VHGT height subrecord";
            return std::nullopt;
        }
        // VHGT stores a height offset followed by one signed delta for every
        // sample in its 33x33 grid.  The remaining three bytes in an on-disk
        // record are alignment padding.
        constexpr std::size_t kRequiredSize = 4 + 33 * 33;
        if (it->data.size() < kRequiredSize)
        {
            error = std::format("VHGT has {} bytes; requires at least {}", it->data.size(), kRequiredSize);
            return std::nullopt;
        }
        float base{};
        std::memcpy(&base, it->data.data(), sizeof(base));
        if (!std::isfinite(base))
        {
            error = "VHGT base height is not finite";
            return std::nullopt;
        }
        std::vector<float> heights(33 * 33);
        for (std::size_t index{}; index < heights.size(); ++index)
        {
            const auto previous = index == 0 ? 0 : index % 33 == 0 ? index - 33 : index - 1;
            const auto baseHeight = index == 0 ? base * 8.0F : heights[previous];
            heights[index] = baseHeight + static_cast<float>(static_cast<std::int8_t>(it->data[4 + index])) * 8.0F;
        }
        return heights;
    }
} // namespace

namespace navmesh::skyrim::offline
{
    TerrainExtraction ExtractTerrain(const ResolvedLoadOrder &loadOrder, const core::Cell &cell)
    {
        TerrainExtraction result;
        if (cell.isInterior)
        {
            return result;
        }
        if (!cell.exteriorCoordinates)
        {
            result.warnings.push_back(
                "Exterior CELL has no XCLC coordinates; terrain cannot be placed in world space.");
            return result;
        }
        const auto [cellX, cellY] = *cell.exteriorCoordinates;
        bool sawLand = false;
        for (const auto *sourceRecord : loadOrder.LandRecords(cell.id))
        {
            const auto &record = *sourceRecord;
            sawLand = true;
            ++result.landRecordsFound;
            std::string error;
            const auto heights = DecodeVhgt(record, error);
            if (!heights)
            {
                result.warnings.push_back(std::format("LAND {:08X} from {} was not decoded: {}.", record.formId,
                                                      record.winning.plugin, error));
                continue;
            }
            const auto sourceIndex = result.scene.geometrySources.size();
            result.scene.geometrySources.push_back({.modelPath = "",
                                                    .materialClass = core::MaterialCollisionClass::Terrain,
                                                    .sourceType = core::GeometrySourceType::Terrain,
                                                    .collisionType = "LAND heightfield",
                                                    .confidence = 1.0F,
                                                    .reference = {record.winning.plugin, record.formId, "LAND"},
                                                    .baseObject = {}});
            const auto vertexBase = static_cast<std::uint32_t>(result.scene.mesh.vertices.size());
            result.scene.mesh.vertices.reserve(result.scene.mesh.vertices.size() + 33 * 33);
            for (std::size_t y = 0; y < 33; ++y)
            {
                for (std::size_t x = 0; x < 33; ++x)
                {
                    result.scene.mesh.vertices.push_back(
                        {static_cast<float>(cellX) * kLandCellSize + static_cast<float>(x) * kLandSampleSpacing,
                         static_cast<float>(cellY) * kLandCellSize + static_cast<float>(y) * kLandSampleSpacing,
                         (*heights)[y * 33 + x]});
                }
            }
            for (std::size_t y = 0; y < 32; ++y)
            {
                for (std::size_t x = 0; x < 32; ++x)
                {
                    const auto a = vertexBase + static_cast<std::uint32_t>(y * 33 + x);
                    const auto b = a + 1;
                    const auto c = a + 33;
                    const auto d = c + 1;
                    const auto terrain = core::TerrainTriangleProvenance{
                        cellX, cellY, record.formId, static_cast<std::uint8_t>(x), static_cast<std::uint8_t>(y)};
                    const auto sourceTriangle = (y * 32 + x) * 2;
                    result.scene.mesh.triangles.push_back({{a, b, c}});
                    result.scene.triangleProvenance.push_back({sourceIndex, sourceTriangle, terrain});
                    result.scene.mesh.triangles.push_back({{b, d, c}});
                    result.scene.triangleProvenance.push_back({sourceIndex, sourceTriangle + 1, terrain});
                }
            }
            ++result.landRecordsDecoded;
        }
        if (!sawLand)
        {
            ++result.landRecordsMissing;
            result.warnings.push_back(std::format(
                "Exterior CELL ({}, {}) has no winning LAND record; no terrain was substituted.", cellX, cellY));
        }
        return result;
    }
} // namespace navmesh::skyrim::offline
