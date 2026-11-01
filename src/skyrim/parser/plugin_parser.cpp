#include "skyrim/parser/plugin_parser.h"
#include "skyrim/parser/water_height.h"
#include "skyrim/parser/text_encoding.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>
#include <zlib.h>

namespace
{
    [[nodiscard]] std::uint16_t ReadU16LE(const std::vector<std::uint8_t> &buffer, std::size_t offset)
    {
        return static_cast<std::uint16_t>(buffer[offset]) | (static_cast<std::uint16_t>(buffer[offset + 1]) << 8);
    }

    [[nodiscard]] std::uint32_t ReadU32LE(const std::vector<std::uint8_t> &buffer, std::size_t offset)
    {
        return static_cast<std::uint32_t>(buffer[offset]) | (static_cast<std::uint32_t>(buffer[offset + 1]) << 8) |
               (static_cast<std::uint32_t>(buffer[offset + 2]) << 16) |
               (static_cast<std::uint32_t>(buffer[offset + 3]) << 24);
    }

    [[nodiscard]] std::int32_t ReadI32LE(const std::vector<std::uint8_t> &buffer, std::size_t offset)
    {
        return static_cast<std::int32_t>(ReadU32LE(buffer, offset));
    }

    [[nodiscard]] std::string ReadAscii(const std::vector<std::uint8_t> &buffer, std::size_t offset, std::size_t length)
    {
        if (offset + length > buffer.size())
        {
            return {};
        }
        return std::string(reinterpret_cast<const char *>(&buffer[offset]), length);
    }

    [[nodiscard]] std::string TrimNulls(std::string value)
    {
        while (!value.empty() && value.back() == '\0')
        {
            value.pop_back();
        }
        return value;
    }

    [[nodiscard]] std::string ToLower(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return value;
    }

    struct RecordHeader
    {
        std::uint32_t dataSize{};
        std::uint32_t flags{};
        std::uint32_t formId{};
    };

    struct BaseRecord
    {
        std::string type;
        std::string editorId;
        std::string modelPath;
        std::optional<navmesh::core::AABB> localBounds;
        bool automaticDoor{};
        bool editorMarker{};
        bool navigationObstacle{};
    };

    struct ParsedReference
    {
        navmesh::core::Reference reference;
        std::uint32_t cellId{};
        std::uint32_t teleportDestinationId{};
        std::optional<navmesh::core::Vec3> teleportDestinationPosition;
    };

    void ParseReferencePayload(const std::vector<std::uint8_t> &payload, const RecordHeader &header,
                               const std::string &type, std::vector<ParsedReference> &references)
    {
        navmesh::core::Reference reference{};
        reference.id = header.formId;
        reference.recordType = type;
        reference.initiallyDisabled = (header.flags & (1U << 11)) != 0;
        reference.deleted = (header.flags & (1U << 5)) != 0;
        std::uint32_t teleportDestinationId{};
        std::optional<navmesh::core::Vec3> teleportDestinationPosition;
        std::size_t offset = 0;
        while (offset + 6 <= payload.size())
        {
            const auto subType = ReadAscii(payload, offset, 4);
            const auto subSize = ReadU16LE(payload, offset + 4);
            const auto dataStart = offset + 6;
            const auto dataEnd = dataStart + subSize;
            if (dataEnd > payload.size())
            {
                break;
            }
            if (subType == "NAME" && subSize >= 4)
            {
                reference.baseObjectId = ReadU32LE(payload, dataStart);
            }
            else if (subType == "DATA" && subSize >= 24)
            {
                std::memcpy(&reference.position.x, payload.data() + dataStart, 4);
                std::memcpy(&reference.position.y, payload.data() + dataStart + 4, 4);
                std::memcpy(&reference.position.z, payload.data() + dataStart + 8, 4);
                std::memcpy(&reference.rotation.x, payload.data() + dataStart + 12, 4);
                std::memcpy(&reference.rotation.y, payload.data() + dataStart + 16, 4);
                std::memcpy(&reference.rotation.z, payload.data() + dataStart + 20, 4);
            }
            else if (subType == "XSCL" && subSize >= 4)
            {
                std::memcpy(&reference.scale, payload.data() + dataStart, 4);
            }
            else if (subType == "XTEL" && subSize >= 4)
            {
                reference.teleportExit = ReadU32LE(payload, dataStart) != 0;
                teleportDestinationId = ReadU32LE(payload, dataStart);
                if (subSize >= 28)
                {
                    navmesh::core::Vec3 position;
                    std::memcpy(&position, payload.data() + dataStart + 4, sizeof(position));
                    if (std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z))
                    {
                        teleportDestinationPosition = position;
                    }
                }
            }
            offset = dataEnd;
        }
        references.push_back({std::move(reference), 0, teleportDestinationId, teleportDestinationPosition});
    }

    [[nodiscard]] std::optional<RecordHeader> ParseRecordHeader(const std::vector<std::uint8_t> &buffer,
                                                                std::size_t offset)
    {
        if (offset + 24 > buffer.size())
        {
            return std::nullopt;
        }
        return RecordHeader{ReadU32LE(buffer, offset + 4), ReadU32LE(buffer, offset + 8),
                            ReadU32LE(buffer, offset + 12)};
    }

    [[nodiscard]] bool MatchesTargetCell(const std::string &targetCell, const std::string &editorId,
                                         const std::string &fullName)
    {
        if (targetCell.empty())
        {
            return true;
        }
        const auto normalizedTarget = ToLower(targetCell);
        const auto normalizedEditor = ToLower(editorId);
        const auto normalizedName = ToLower(fullName);
        return normalizedEditor == normalizedTarget || normalizedName == normalizedTarget ||
               normalizedEditor.find(normalizedTarget) != std::string::npos ||
               normalizedName.find(normalizedTarget) != std::string::npos;
    }

    [[nodiscard]] std::optional<std::vector<std::uint8_t>> DecompressNavMesh(const std::vector<std::uint8_t> &buffer,
                                                                             std::size_t dataStart, std::size_t dataEnd,
                                                                             std::uint32_t flags)
    {
        if ((flags & 0x00040000U) == 0)
        {
            return std::vector<std::uint8_t>(buffer.begin() + dataStart, buffer.begin() + dataEnd);
        }
        if (dataStart + 4 > dataEnd)
        {
            return std::nullopt;
        }

        std::vector<std::uint8_t> result(ReadU32LE(buffer, dataStart));
        auto resultSize = static_cast<uLongf>(result.size());
        const auto status = uncompress(result.data(), &resultSize, buffer.data() + dataStart + 4,
                                       static_cast<uLong>(dataEnd - dataStart - 4));
        if (status != Z_OK)
        {
            return std::nullopt;
        }
        result.resize(static_cast<std::size_t>(resultSize));
        return result;
    }

    [[nodiscard]] std::optional<navmesh::core::NavMesh> ParseNavMesh(const std::vector<std::uint8_t> &payload,
                                                                     std::uint32_t formId)
    {
        if (payload.size() < 6 || ReadAscii(payload, 0, 4) != "NVNM")
        {
            return std::nullopt;
        }
        const auto bodySize = ReadU16LE(payload, 4);
        const std::size_t bodyStart = 6;
        const auto bodyEnd = bodyStart + bodySize;
        if (bodyEnd > payload.size() || bodySize < 0x14)
        {
            return std::nullopt;
        }

        const auto vertexCount = ReadU32LE(payload, bodyStart + 0x10);
        const auto verticesStart = bodyStart + 0x14;
        const auto trianglesCountOffset = verticesStart + static_cast<std::size_t>(vertexCount) * 12;
        if (trianglesCountOffset + 4 > bodyEnd)
        {
            return std::nullopt;
        }

        navmesh::core::NavMesh mesh{};
        mesh.id = formId;
        mesh.vertices.reserve(vertexCount);
        for (std::uint32_t index = 0; index < vertexCount; ++index)
        {
            const auto vertexOffset = verticesStart + static_cast<std::size_t>(index) * 12;
            if (vertexOffset + 12 > bodyEnd)
            {
                return std::nullopt;
            }
            navmesh::core::Vec3 vertex{};
            std::memcpy(&vertex.x, payload.data() + vertexOffset, sizeof(float));
            std::memcpy(&vertex.y, payload.data() + vertexOffset + 4, sizeof(float));
            std::memcpy(&vertex.z, payload.data() + vertexOffset + 8, sizeof(float));
            mesh.vertices.push_back(vertex);
        }

        const auto polygonCount = ReadU32LE(payload, trianglesCountOffset);
        const auto trianglesStart = trianglesCountOffset + 4;
        if (trianglesStart + static_cast<std::size_t>(polygonCount) * 16 > bodyEnd)
        {
            return std::nullopt;
        }
        mesh.polygons.reserve(polygonCount);
        for (std::uint32_t index = 0; index < polygonCount; ++index)
        {
            const auto triangleOffset = trianglesStart + static_cast<std::size_t>(index) * 16;
            navmesh::core::NavPolygon polygon{};
            for (std::size_t vertex = 0; vertex < 3; ++vertex)
            {
                polygon.vertices[vertex] = ReadU16LE(payload, triangleOffset + vertex * 2);
                polygon.neighbors[vertex] = ReadU16LE(payload, triangleOffset + 6 + vertex * 2);
            }
            polygon.flags = ReadU16LE(payload, triangleOffset + 12);
            polygon.coverFlags = ReadU16LE(payload, triangleOffset + 14);
            mesh.polygons.push_back(polygon);
        }
        if (ReadU32LE(payload, bodyStart) == 12)
        {
            const auto trailingStart = trianglesStart + static_cast<std::size_t>(polygonCount) * 16;
            (void)navmesh::skyrim::DecodeNavmeshConnections({payload.data() + trailingStart, bodyEnd - trailingStart},
                                                            mesh);
        }
        return mesh;
    }
} // namespace

namespace navmesh::skyrim
{
    bool IsEditorMarker(std::string_view recordType, std::uint32_t flags)
    {
        return (flags & (1U << 23)) != 0 &&
               (recordType == "STAT" || recordType == "ACTI" || recordType == "FURN" || recordType == "DOOR");
    }

    bool IsNavigationObstacle(std::string_view recordType, std::uint32_t flags)
    {
        // Supported flag definitions: https://github.com/TES5Edit/TES5Edit/blob/dev-4.1.6/Core/wbDefinitionsTES5.pas
        return (flags & (1U << 25)) != 0 && (recordType == "STAT" || recordType == "ACTI" || recordType == "MSTT" ||
                                             recordType == "CONT" || recordType == "FURN" || recordType == "DOOR" ||
                                             recordType == "TREE" || recordType == "LIGH" || recordType == "SCOL");
    }

    std::vector<core::Cell> ListCells(const std::filesystem::path &pluginPath)
    {
        if (!std::filesystem::exists(pluginPath))
        {
            return {};
        }

        std::ifstream input(pluginPath, std::ios::binary);
        std::vector<std::uint8_t> buffer((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        if (buffer.size() < 24)
        {
            return {};
        }

        const auto fileStart = buffer.size() >= 8 && ReadAscii(buffer, 0, 4) == "TES4" ? 24 + ReadU32LE(buffer, 4) : 0;
        std::vector<core::Cell> cells;
        std::unordered_map<std::uint32_t, std::vector<core::NavMesh>> navMeshesByGroup;
        std::unordered_map<std::uint32_t, BaseRecord> baseRecords;
        std::vector<ParsedReference> references;

        const auto walk = [&](const auto &self, std::size_t begin, std::size_t end, std::uint32_t groupId) -> void
        {
            auto current = begin;
            while (current + 24 <= end)
            {
                const auto type = ReadAscii(buffer, current, 4);
                if (type == "GRUP")
                {
                    const auto groupSize = ReadU32LE(buffer, current + 4);
                    const auto groupEnd = current + groupSize;
                    if (groupSize < 24 || groupEnd > end)
                    {
                        return;
                    }
                    self(self, current + 24, groupEnd, ReadU32LE(buffer, current + 8));
                    current = groupEnd;
                    continue;
                }

                const auto header = ParseRecordHeader(buffer, current);
                if (!header)
                {
                    return;
                }
                const auto dataStart = current + 24;
                const auto dataEnd = dataStart + header->dataSize;
                if (dataEnd > end)
                {
                    return;
                }

                if (type == "CELL")
                {
                    const auto payload = DecompressNavMesh(buffer, dataStart, dataEnd, header->flags);
                    if (!payload)
                    {
                        current = dataEnd;
                        continue;
                    }
                    core::Cell cell{};
                    cell.id = header->formId;
                    cell.name = "Tamriel";
                    bool hasWater{};
                    std::size_t subOffset = 0;
                    while (subOffset + 6 <= payload->size())
                    {
                        const auto subType = ReadAscii(*payload, subOffset, 4);
                        const auto subSize = ReadU16LE(*payload, subOffset + 4);
                        const auto subDataStart = subOffset + 6;
                        const auto subDataEnd = subDataStart + subSize;
                        if (subDataEnd > payload->size())
                        {
                            break;
                        }
                        if (subType == "EDID")
                        {
                            cell.editorId = navmesh::skyrim::DecodePluginText(
                                TrimNulls(ReadAscii(*payload, subDataStart, subSize)));
                        }
                        else if (subType == "FULL")
                        {
                            auto name = navmesh::skyrim::DecodePluginText(
                                TrimNulls(ReadAscii(*payload, subDataStart, subSize)));
                            if (!name.empty() && std::all_of(name.begin(), name.end(), [](unsigned char value)
                                                             { return std::isprint(value) != 0; }))
                            {
                                cell.name = std::move(name);
                            }
                        }
                        else if (subType == "XCLL")
                        {
                            cell.isInterior = true;
                        }
                        else if (subType == "DATA" && subSize >= 1)
                        {
                            cell.isInterior = ((*payload)[subDataStart] & 1U) != 0;
                            hasWater = ((*payload)[subDataStart] & 2U) != 0;
                        }
                        else if (subType == "XCLW" && subSize >= sizeof(float))
                        {
                            float height;
                            std::memcpy(&height, payload->data() + subDataStart, sizeof(height));
                            if (navmesh::skyrim::detail::IsExplicitWaterHeight(height))
                            {
                                cell.waterHeight = height;
                            }
                        }
                        else if (subType == "XCLC" && subSize >= 8)
                        {
                            // XCLC's coordinates define an exterior CELL. The
                            // optional flag DWORD is not an interior marker.
                            cell.isInterior = false;
                            cell.exteriorCoordinates = {ReadI32LE(*payload, subDataStart),
                                                        ReadI32LE(*payload, subDataStart + 4)};
                        }
                        subOffset = subDataEnd;
                    }
                    if (!hasWater)
                    {
                        cell.waterHeight.reset();
                    }
                    cells.push_back(std::move(cell));
                }
                else if (type == "REFR" || type == "ACHR")
                {
                    const auto payload = DecompressNavMesh(buffer, dataStart, dataEnd, header->flags);
                    if (payload)
                    {
                        ParseReferencePayload(*payload, *header, type, references);
                        if (!references.empty())
                        {
                            references.back().cellId = groupId;
                        }
                    }
                }
                else if (type == "NAVM")
                {
                    const auto payload = DecompressNavMesh(buffer, dataStart, dataEnd, header->flags);
                    if (payload)
                    {
                        const auto mesh = ParseNavMesh(*payload, header->formId);
                        if (mesh)
                        {
                            navMeshesByGroup[groupId].push_back(*mesh);
                        }
                    }
                }
                else
                {
                    BaseRecord record{.type = type,
                                      .editorMarker = IsEditorMarker(type, header->flags),
                                      .navigationObstacle = IsNavigationObstacle(type, header->flags)};
                    std::string femaleArmorModel;
                    const auto payload = DecompressNavMesh(buffer, dataStart, dataEnd, header->flags);
                    if (payload)
                    {
                        std::size_t subOffset = 0;
                        while (subOffset + 6 <= payload->size())
                        {
                            const auto subType = ReadAscii(*payload, subOffset, 4);
                            const auto subSize = ReadU16LE(*payload, subOffset + 4);
                            const auto subDataStart = subOffset + 6;
                            const auto subDataEnd = subDataStart + subSize;
                            if (subDataEnd > payload->size())
                            {
                                break;
                            }
                            if (subType == "EDID")
                            {
                                record.editorId = navmesh::skyrim::DecodePluginText(
                                    TrimNulls(ReadAscii(*payload, subDataStart, subSize)));
                            }
                            else if ((subType == "MODL" && record.type != "ARMO") ||
                                     (subType == "MOD2" && record.type == "ARMO"))
                            {
                                record.modelPath = navmesh::skyrim::DecodePluginText(
                                    TrimNulls(ReadAscii(*payload, subDataStart, subSize)));
                            }
                            else if (subType == "MOD4" && record.type == "ARMO")
                            {
                                femaleArmorModel = navmesh::skyrim::DecodePluginText(
                                    TrimNulls(ReadAscii(*payload, subDataStart, subSize)));
                            }
                            else if (record.type == "DOOR" && subType == "FNAM" && subSize >= 1)
                            {
                                record.automaticDoor = ((*payload)[subDataStart] & (1U << 1)) != 0;
                            }
                            else if (subType == "OBND" && subSize == 12)
                            {
                                std::array<std::int16_t, 6> values;
                                std::memcpy(values.data(), payload->data() + subDataStart, sizeof(values));
                                if (values[0] < values[3] && values[1] < values[4] && values[2] < values[5])
                                {
                                    record.localBounds =
                                        core::AABB{{static_cast<float>(values[0]), static_cast<float>(values[1]),
                                                    static_cast<float>(values[2])},
                                                   {static_cast<float>(values[3]), static_cast<float>(values[4]),
                                                    static_cast<float>(values[5])}};
                                }
                            }
                            subOffset = subDataEnd;
                        }
                    }
                    if (record.type == "ARMO" && record.modelPath.empty())
                    {
                        record.modelPath = std::move(femaleArmorModel);
                    }
                    baseRecords[header->formId] = std::move(record);
                }
                current = dataEnd;
            }
        };

        walk(walk, fileStart, buffer.size(), 0);
        // Reciprocal lookups must not rescan every placement when inspecting a whole plugin.
        std::unordered_map<std::uint32_t, const ParsedReference *> teleportReferences;
        for (const auto &parsed : references)
        {
            if (parsed.teleportDestinationId != 0)
            {
                teleportReferences.emplace(parsed.reference.id, &parsed);
            }
        }
        for (auto &cell : cells)
        {
            const auto meshes = navMeshesByGroup.find(cell.id);
            if (meshes != navMeshesByGroup.end())
            {
                cell.navMeshes = meshes->second;
            }
            for (auto &parsed : references)
            {
                if (parsed.cellId != cell.id)
                {
                    continue;
                }
                if (const auto base = baseRecords.find(parsed.reference.baseObjectId); base != baseRecords.end())
                {
                    parsed.reference.baseRecordType = base->second.type;
                    parsed.reference.editorId = base->second.editorId;
                    parsed.reference.modelPath = base->second.modelPath;
                    parsed.reference.localBounds = base->second.localBounds;
                    parsed.reference.automaticDoor = base->second.automaticDoor;
                    parsed.reference.editorMarker = base->second.editorMarker;
                    parsed.reference.navigationObstacle = base->second.navigationObstacle;
                }
                const auto partner = teleportReferences.find(parsed.teleportDestinationId);
                if (partner != teleportReferences.end() &&
                    partner->second->teleportDestinationId == parsed.reference.id &&
                    !partner->second->reference.deleted)
                {
                    parsed.reference.teleportArrival = partner->second->teleportDestinationPosition;
                }
                cell.references.push_back(parsed.reference);
            }
        }
        return cells;
    }

    std::optional<core::Cell> LoadCell(const std::filesystem::path &pluginPath, const std::string &targetCell,
                                       const std::string &targetWorldspace, const std::optional<std::int32_t> &cellX,
                                       const std::optional<std::int32_t> &cellY,
                                       const std::optional<std::uint32_t> &cellFormId,
                                       const std::string &targetEditorId)
    {
        static_cast<void>(targetWorldspace);
        for (auto &cell : ListCells(pluginPath))
        {
            const bool matchesByFormId = cellFormId && cell.id == *cellFormId;
            const bool matchesByEditorId = !targetEditorId.empty() && ToLower(cell.editorId) == ToLower(targetEditorId);
            const bool matchesByCellName =
                !targetCell.empty() && MatchesTargetCell(targetCell, cell.editorId, cell.name);
            const bool matchesByCoords = cell.exteriorCoordinates && cellX && cellY &&
                                         (*cell.exteriorCoordinates)[0] == *cellX &&
                                         (*cell.exteriorCoordinates)[1] == *cellY;
            const bool hasSelector = !targetCell.empty() || !targetEditorId.empty() || cellFormId || cellX || cellY;
            const bool selected =
                !hasSelector || matchesByFormId || matchesByEditorId || matchesByCellName || matchesByCoords;
            if (selected)
            {
                return cell;
            }
        }
        return std::nullopt;
    }
} // namespace navmesh::skyrim
