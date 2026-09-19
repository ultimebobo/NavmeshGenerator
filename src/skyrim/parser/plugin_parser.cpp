#include "skyrim/parser/plugin_parser.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
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
    [[nodiscard]] std::uint16_t ReadU16LE(const std::vector<std::uint8_t>& buffer, std::size_t offset)
    {
        return static_cast<std::uint16_t>(buffer[offset]) |
            (static_cast<std::uint16_t>(buffer[offset + 1]) << 8);
    }

    [[nodiscard]] std::uint32_t ReadU32LE(const std::vector<std::uint8_t>& buffer, std::size_t offset)
    {
        return static_cast<std::uint32_t>(buffer[offset]) |
            (static_cast<std::uint32_t>(buffer[offset + 1]) << 8) |
            (static_cast<std::uint32_t>(buffer[offset + 2]) << 16) |
            (static_cast<std::uint32_t>(buffer[offset + 3]) << 24);
    }

    [[nodiscard]] std::int32_t ReadI32LE(const std::vector<std::uint8_t>& buffer, std::size_t offset)
    {
        return static_cast<std::int32_t>(ReadU32LE(buffer, offset));
    }

    [[nodiscard]] std::string ReadAscii(const std::vector<std::uint8_t>& buffer, std::size_t offset, std::size_t length)
    {
        if (offset + length > buffer.size()) {
            return {};
        }
        return std::string(reinterpret_cast<const char*>(&buffer[offset]), length);
    }

    [[nodiscard]] std::string TrimNulls(std::string value)
    {
        while (!value.empty() && value.back() == '\0') {
            value.pop_back();
        }
        return value;
    }

    [[nodiscard]] std::string ToLower(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return value;
    }

    struct RecordHeader
    {
        std::uint32_t dataSize{};
        std::uint32_t flags{};
        std::uint32_t formId{};
    };

    [[nodiscard]] std::optional<RecordHeader> ParseRecordHeader(const std::vector<std::uint8_t>& buffer, std::size_t offset)
    {
        if (offset + 24 > buffer.size()) {
            return std::nullopt;
        }
        return RecordHeader{ ReadU32LE(buffer, offset + 4), ReadU32LE(buffer, offset + 8), ReadU32LE(buffer, offset + 12) };
    }

    [[nodiscard]] bool MatchesTargetCell(const std::string& targetCell, const std::string& editorId, const std::string& fullName)
    {
        if (targetCell.empty()) {
            return true;
        }
        const auto normalizedTarget = ToLower(targetCell);
        const auto normalizedEditor = ToLower(editorId);
        const auto normalizedName = ToLower(fullName);
        return normalizedEditor == normalizedTarget || normalizedName == normalizedTarget ||
            normalizedEditor.find(normalizedTarget) != std::string::npos || normalizedName.find(normalizedTarget) != std::string::npos;
    }

    [[nodiscard]] std::optional<std::vector<std::uint8_t>> DecompressNavMesh(
        const std::vector<std::uint8_t>& buffer,
        std::size_t dataStart,
        std::size_t dataEnd,
        std::uint32_t flags)
    {
        if ((flags & 0x00040000U) == 0) {
            return std::vector<std::uint8_t>(buffer.begin() + dataStart, buffer.begin() + dataEnd);
        }
        if (dataStart + 4 > dataEnd) {
            return std::nullopt;
        }

        std::vector<std::uint8_t> result(ReadU32LE(buffer, dataStart));
        auto resultSize = static_cast<uLongf>(result.size());
        const auto status = uncompress(
            result.data(),
            &resultSize,
            buffer.data() + dataStart + 4,
            static_cast<uLong>(dataEnd - dataStart - 4));
        if (status != Z_OK) {
            return std::nullopt;
        }
        result.resize(static_cast<std::size_t>(resultSize));
        return result;
    }

    [[nodiscard]] std::optional<navmesh::core::NavMesh> ParseNavMesh(
        const std::vector<std::uint8_t>& payload,
        std::uint32_t formId)
    {
        if (payload.size() < 6 || ReadAscii(payload, 0, 4) != "NVNM") {
            return std::nullopt;
        }
        const auto bodySize = ReadU16LE(payload, 4);
        const std::size_t bodyStart = 6;
        const auto bodyEnd = bodyStart + bodySize;
        if (bodyEnd > payload.size() || bodySize < 0x14) {
            return std::nullopt;
        }

        const auto vertexCount = ReadU32LE(payload, bodyStart + 0x10);
        const auto verticesStart = bodyStart + 0x14;
        const auto trianglesCountOffset = verticesStart + static_cast<std::size_t>(vertexCount) * 12;
        if (trianglesCountOffset + 4 > bodyEnd) {
            return std::nullopt;
        }

        navmesh::core::NavMesh mesh{};
        mesh.id = formId;
        mesh.vertices.reserve(vertexCount);
        for (std::uint32_t index = 0; index < vertexCount; ++index) {
            const auto vertexOffset = verticesStart + static_cast<std::size_t>(index) * 12;
            if (vertexOffset + 12 > bodyEnd) {
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
        if (trianglesStart + static_cast<std::size_t>(polygonCount) * 16 > bodyEnd) {
            return std::nullopt;
        }
        mesh.polygons.reserve(polygonCount);
        for (std::uint32_t index = 0; index < polygonCount; ++index) {
            const auto triangleOffset = trianglesStart + static_cast<std::size_t>(index) * 16;
            navmesh::core::NavPolygon polygon{};
            for (std::size_t vertex = 0; vertex < 3; ++vertex) {
                polygon.vertices[vertex] = ReadU16LE(payload, triangleOffset + vertex * 2);
                polygon.neighbors[vertex] = ReadU16LE(payload, triangleOffset + 6 + vertex * 2);
            }
            polygon.flags = ReadU16LE(payload, triangleOffset + 12);
            mesh.polygons.push_back(polygon);
        }
        return mesh;
    }
}

namespace navmesh::skyrim::offline
{
    std::optional<core::Cell> LoadCell(
        const std::filesystem::path& pluginPath,
        const std::string& targetCell,
        const std::string& targetWorldspace,
        const std::optional<std::int32_t>& cellX,
        const std::optional<std::int32_t>& cellY)
    {
        if (!std::filesystem::exists(pluginPath)) {
            return std::nullopt;
        }

        std::ifstream input(pluginPath, std::ios::binary);
        std::vector<std::uint8_t> buffer((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
        if (buffer.size() < 24) {
            return std::nullopt;
        }

        const auto fileStart = buffer.size() >= 8 && ReadAscii(buffer, 0, 4) == "TES4" ? 24 + ReadU32LE(buffer, 4) : 0;
        std::vector<core::Cell> cells;
        std::unordered_map<std::uint32_t, std::vector<core::NavMesh>> navMeshesByGroup;

        const auto walk = [&](const auto& self, std::size_t begin, std::size_t end, std::uint32_t groupId) -> void {
            auto current = begin;
            while (current + 24 <= end) {
                const auto type = ReadAscii(buffer, current, 4);
                if (type == "GRUP") {
                    if (current + 24 > end) {
                        return;
                    }
                    const auto groupSize = ReadU32LE(buffer, current + 4);
                    const auto groupEnd = current + groupSize;
                    if (groupSize < 24 || groupEnd > end) {
                        return;
                    }
                    self(self, current + 24, groupEnd, ReadU32LE(buffer, current + 8));
                    current = groupEnd;
                    continue;
                }

                const auto header = ParseRecordHeader(buffer, current);
                if (!header) {
                    return;
                }
                const auto dataStart = current + 24;
                const auto dataEnd = dataStart + header->dataSize;
                if (dataEnd > end) {
                    return;
                }

                if (type == "CELL") {
                    core::Cell cell{};
                    cell.id = header->formId;
                    cell.name = targetWorldspace.empty() ? "Tamriel" : targetWorldspace;
                    std::size_t subOffset = dataStart;
                    while (subOffset + 6 <= dataEnd) {
                        const auto subSize = ReadU16LE(buffer, subOffset);
                        const auto subType = ReadAscii(buffer, subOffset + 2, 4);
                        const auto subDataStart = subOffset + 6;
                        const auto subDataEnd = subDataStart + subSize;
                        if (subDataEnd > dataEnd) {
                            break;
                        }
                        if (subType == "EDID") {
                            cell.editorId = TrimNulls(ReadAscii(buffer, subDataStart, subSize));
                        } else if (subType == "FULL") {
                            cell.name = TrimNulls(ReadAscii(buffer, subDataStart, subSize));
                        } else if (subType == "XCLC" && subSize >= 12) {
                            cell.isInterior = ReadU32LE(buffer, subDataStart + 8) != 0;
                            if (!cell.isInterior) {
                                cell.exteriorCoordinates = { ReadI32LE(buffer, subDataStart), ReadI32LE(buffer, subDataStart + 4) };
                            }
                        }
                        subOffset = subDataEnd;
                    }
                    cells.push_back(std::move(cell));
                } else if (type == "NAVM") {
                    const auto payload = DecompressNavMesh(buffer, dataStart, dataEnd, header->flags);
                    if (payload) {
                        const auto mesh = ParseNavMesh(*payload, header->formId);
                        if (mesh) {
                            navMeshesByGroup[groupId].push_back(*mesh);
                        }
                    }
                }
                current = dataEnd;
            }
        };

        walk(walk, fileStart, buffer.size(), 0);

        std::optional<core::Cell> firstCell;
        for (auto& cell : cells) {
            if (!firstCell) {
                firstCell = cell;
            }
            const bool matchesByCellName = MatchesTargetCell(targetCell, cell.editorId, cell.name);
            const bool matchesByCoords = cell.exteriorCoordinates && cellX && cellY &&
                (*cell.exteriorCoordinates)[0] == *cellX && (*cell.exteriorCoordinates)[1] == *cellY;
            const bool selected = (targetCell.empty() && targetWorldspace.empty() && !cellX && !cellY) || matchesByCellName || matchesByCoords;
            if (selected) {
                const auto meshes = navMeshesByGroup.find(cell.id);
                if (meshes != navMeshesByGroup.end()) {
                    cell.navMeshes = meshes->second;
                }
                return cell;
            }
        }

        if (firstCell) {
            const auto meshes = navMeshesByGroup.find(firstCell->id);
            if (meshes != navMeshesByGroup.end()) {
                firstCell->navMeshes = meshes->second;
            }
        }
        return firstCell;
    }
}
