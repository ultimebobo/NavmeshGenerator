#include "skyrim/parser/plugin_parser.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <cmath>
#include <fstream>
#include <iterator>
#include <unordered_map>
#include <zlib.h>

namespace
{
    using namespace navmesh::skyrim;
    constexpr std::size_t kMaxDecodedRecordSize = 256ULL * 1024ULL * 1024ULL;
    bool Has(std::span<const std::uint8_t> b, std::size_t p, std::size_t n)
    {
        return p <= b.size() && n <= b.size() - p;
    }
    std::uint16_t U16(std::span<const std::uint8_t> b, std::size_t p)
    {
        return static_cast<std::uint16_t>(b[p] | b[p + 1] << 8);
    }
    std::uint32_t U32(std::span<const std::uint8_t> b, std::size_t p)
    {
        return static_cast<std::uint32_t>(b[p]) | static_cast<std::uint32_t>(b[p + 1]) << 8 |
               static_cast<std::uint32_t>(b[p + 2]) << 16 | static_cast<std::uint32_t>(b[p + 3]) << 24;
    }
    std::int32_t I32(std::span<const std::uint8_t> b, std::size_t p)
    {
        return static_cast<std::int32_t>(U32(b, p));
    }
    std::string Text(std::span<const std::uint8_t> b, std::size_t p, std::size_t n)
    {
        return p + n <= b.size() ? std::string(reinterpret_cast<const char *>(b.data() + p), n) : "";
    }
    std::string Trim(std::string s)
    {
        while (!s.empty() && (s.back() == '\0' || std::isspace(static_cast<unsigned char>(s.back()))))
        {
            s.pop_back();
        }
        return s;
    }
    std::string Lower(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    }
    bool IndexedRecordType(const std::string &type)
    {
        // Keep the full roadmap subset, including base forms that provide a model
        // and doors/links needed by later navigation work.  Other record types
        // remain outside this bounded in-memory reader.
        return type == "WRLD" || type == "CELL" || type == "LAND" || type == "REFR" || type == "ACHR" ||
               type == "NAVM" || type == "STAT" || type == "MSTT" || type == "ACTI" || type == "DOOR" ||
               type == "FURN" || type == "TREE" || type == "FLOR" || type == "CONT";
    }
    bool DecodePayload(const std::vector<std::uint8_t> &fileBytes, std::size_t begin, std::size_t end, bool compressed,
                       std::vector<std::uint8_t> &decoded, std::string &error)
    {
        if (!Has(fileBytes, begin, end - begin))
        {
            error = "record payload extends outside the plugin";
            return false;
        }
        if (!compressed)
        {
            decoded.assign(fileBytes.begin() + begin, fileBytes.begin() + end);
            return true;
        }
        if (!Has(fileBytes, begin, 4))
        {
            error = "compressed record lacks its uncompressed-size prefix";
            return false;
        }
        const auto expected = U32(fileBytes, begin);
        if (expected == 0 || expected > kMaxDecodedRecordSize)
        {
            error = "compressed record declares an unsafe uncompressed size";
            return false;
        }
        decoded.resize(expected);
        auto actual = static_cast<uLongf>(expected);
        const auto status =
            uncompress(decoded.data(), &actual, fileBytes.data() + begin + 4, static_cast<uLong>(end - begin - 4));
        if (status != Z_OK || actual != expected)
        {
            error = "zlib payload does not decode to its declared size";
            return false;
        }
        return true;
    }
    bool ParseSubrecords(const PluginRecord &raw, ResolvedRecord &record, std::vector<std::string> *masters,
                         std::string &error)
    {
        const auto &payload = raw.decodedPayload;
        std::size_t p = 0;
        std::optional<std::uint32_t> extendedSize;
        while (p < payload.size())
        {
            if (!Has(payload, p, 6))
            {
                error = "truncated subrecord header at decoded offset " + std::to_string(p);
                return false;
            }
            const auto encodedStart = p;
            const auto kind = Text(payload, p, 4);
            const auto shortSize = U16(payload, p + 4);
            p += 6;
            if (kind == "XXXX")
            {
                if (shortSize != 4 || !Has(payload, p, 4))
                {
                    error = "malformed XXXX extended-size marker";
                    return false;
                }
                extendedSize = U32(payload, p);
                p += 4;
                continue;
            }
            const auto size = extendedSize.value_or(shortSize);
            const auto data = p;
            if (!Has(payload, data, size))
            {
                error = "subrecord " + kind + " exceeds decoded payload";
                return false;
            }
            Subrecord sub{.type = kind,
                          .encodedRange = {encodedStart, p + size - encodedStart},
                          .dataRange = {data, size},
                          .encodedBytes = payload.Slice(encodedStart, data + size - encodedStart),
                          .data = payload.Slice(data, size),
                          .extendedSize = extendedSize.has_value()};
            record.raw->subrecords.push_back(std::move(sub));
            if (kind == "MAST" && masters)
            {
                masters->push_back(Trim(Text(payload, data, size)));
            }
            else if (kind == "EDID")
            {
                record.editorId = Trim(Text(payload, data, size));
            }
            else if (kind == "FULL")
            {
                record.name = Trim(Text(payload, data, size));
            }
            else if (kind == "MODL")
            {
                record.modelPath = Trim(Text(payload, data, size));
            }
            else if (kind == "NAME" && (record.type == "REFR" || record.type == "ACHR") && size >= 4)
            {
                record.referencedFormIds.push_back(U32(payload, data));
            }
            else if ((kind == "XLKR" || kind == "XESP" || kind == "XNDP" || kind == "XTEL") && size >= 4)
            {
                record.linkedFormIds.push_back(U32(payload, data));
            }
            else if (kind == "DATA" && (record.type == "REFR" || record.type == "ACHR") && size >= 24)
            {
                std::array<float, 6> transform{};
                std::memcpy(transform.data(), payload.data() + data, sizeof(transform));
                record.transform = transform;
            }
            else if (kind == "XSCL" && (record.type == "REFR" || record.type == "ACHR") && size >= 4)
            {
                float scale{};
                std::memcpy(&scale, payload.data() + data, sizeof(scale));
                record.referenceScale = scale;
            }
            // XCLC is present on exterior CELL records. Its optional third
            // DWORD is a cell flag field, not an interior/exterior discriminator
            // (Riverwood, for example, legitimately has non-zero flags).
            else if (kind == "XCLC" && size >= 8)
            {
                record.exteriorCoordinates = {I32(payload, data), I32(payload, data + 4)};
            }
            else if (kind == "NVNM" && record.type == "NAVM")
            {
                if (size < 0x18)
                {
                    error = "NVNM is shorter than the verified vertex-count header";
                    return false;
                }
                const auto vertexCount = U32(payload, data + 0x10);
                const auto vertices = data + 0x14;
                if (vertexCount > (size - 0x14) / 12 ||
                    !Has(payload, vertices, static_cast<std::size_t>(vertexCount) * 12))
                {
                    error = "NVNM vertex array exceeds subrecord";
                    return false;
                }
                const auto triangleCountAt = vertices + static_cast<std::size_t>(vertexCount) * 12;
                if (!Has(payload, triangleCountAt, 4))
                {
                    error = "NVNM omits its triangle count";
                    return false;
                }
                const auto triangleCount = U32(payload, triangleCountAt);
                const auto triangles = triangleCountAt + 4;
                if (triangleCount > (data + size - triangles) / 16 ||
                    !Has(payload, triangles, static_cast<std::size_t>(triangleCount) * 16))
                {
                    error = "NVNM triangle array exceeds subrecord";
                    return false;
                }
                const auto version = U32(payload, data);
                record.navm = NavmLayout{
                    .version = version,
                    .declaredBodySize = static_cast<std::uint16_t>(size),
                    .vertexCount = vertexCount,
                    .triangleCount = triangleCount,
                    .header = {data, 0x14},
                    .vertices = {vertices, static_cast<std::uint64_t>(vertexCount) * 12},
                    .triangles = {triangles, static_cast<std::uint64_t>(triangleCount) * 16},
                    .trailingData = {triangles + static_cast<std::size_t>(triangleCount) * 16,
                                     data + size - (triangles + static_cast<std::size_t>(triangleCount) * 16)},
                    .supported = version == 12};
            }
            extendedSize.reset();
            p = data + size;
        }
        if (extendedSize)
        {
            error = "XXXX extended-size marker has no following subrecord";
            return false;
        }
        return true;
    }
    /// Ignore only classified display/streaming fields; unknown subrecords remain exact dependencies.
    bool NavigationEquivalent(const ResolvedRecord &left, const ResolvedRecord &right)
    {
        if (!left.raw || !right.raw || left.type != right.type ||
            (left.raw->flags & ~0x40000U) != (right.raw->flags & ~0x40000U) || left.cellFormId != right.cellFormId ||
            left.worldspaceFormId != right.worldspaceFormId || left.referencedFormIds != right.referencedFormIds ||
            left.transform != right.transform ||
            left.referenceScale.value_or(1.0F) != right.referenceScale.value_or(1.0F))
        {
            return false;
        }
        const auto relevant = [&](const Subrecord &sub)
        {
            if (sub.type == "EDID" || sub.type == "FULL")
            {
                return false;
            }
            if ((left.type == "REFR" || left.type == "ACHR") && sub.type == "NAME")
            {
                return false;
            }
            if (left.type == "WRLD")
            {
                // Cached height/streaming tables and display textures do not supply LAND or collision.
                return sub.type != "MHDT" && sub.type != "OFST" && sub.type != "RNAM" && sub.type != "TNAM" &&
                       sub.type != "UNAM" && sub.type != "MNAM";
            }
            if (left.type == "LAND")
            {
                return sub.type != "VNML" && sub.type != "VCLR" && sub.type != "BTXT" && sub.type != "ATXT" &&
                       sub.type != "VTXT" && sub.type != "VTEX" && sub.type != "MPCD";
            }
            return true;
        };
        auto a = left.raw->subrecords.begin();
        auto b = right.raw->subrecords.begin();
        for (;;)
        {
            a = std::find_if(a, left.raw->subrecords.end(), relevant);
            b = std::find_if(b, right.raw->subrecords.end(), relevant);
            if (a == left.raw->subrecords.end() || b == right.raw->subrecords.end())
            {
                return a == left.raw->subrecords.end() && b == right.raw->subrecords.end();
            }
            if (a->type != b->type || !(a->data == b->data))
            {
                return false;
            }
            ++a;
            ++b;
        }
    }

    std::optional<navmesh::core::NavMesh> DecodeNavMesh(const ResolvedRecord &record)
    {
        if (!record.raw || !record.navm || !record.navm->supported)
        {
            return std::nullopt;
        }
        const auto &layout = *record.navm;
        const auto &payload = record.raw->decodedPayload;
        if (!Has(payload, static_cast<std::size_t>(layout.vertices.offset),
                 static_cast<std::size_t>(layout.vertices.size)) ||
            !Has(payload, static_cast<std::size_t>(layout.triangles.offset),
                 static_cast<std::size_t>(layout.triangles.size)))
        {
            return std::nullopt;
        }
        navmesh::core::NavMesh mesh{.id = record.formId};
        mesh.vertices.reserve(layout.vertexCount);
        mesh.polygons.reserve(layout.triangleCount);
        for (std::size_t index{}; index < layout.vertexCount; ++index)
        {
            navmesh::core::Vec3 vertex{};
            std::memcpy(&vertex, payload.data() + layout.vertices.offset + index * 12, sizeof(vertex));
            mesh.vertices.push_back(vertex);
        }
        for (std::size_t index{}; index < layout.triangleCount; ++index)
        {
            const auto offset = static_cast<std::size_t>(layout.triangles.offset) + index * 16;
            navmesh::core::NavPolygon polygon{};
            for (std::size_t edge{}; edge < 3; ++edge)
            {
                polygon.vertices[edge] = U16(payload, offset + edge * 2);
                polygon.neighbors[edge] = U16(payload, offset + 6 + edge * 2);
            }
            polygon.flags = U16(payload, offset + 12);
            mesh.polygons.push_back(polygon);
        }
        const auto trailing = static_cast<std::size_t>(layout.trailingData.offset);
        const auto size = static_cast<std::size_t>(layout.trailingData.size);
        if (Has(payload, trailing, size))
        {
            (void)navmesh::skyrim::DecodeNavmeshConnections({payload.data() + trailing, size}, mesh);
        }
        return mesh;
    }
} // namespace

namespace navmesh::skyrim
{
    bool DecodeNavmeshConnections(std::span<const std::uint8_t> trailing, core::NavMesh &mesh)
    {
        mesh.externalLinks.clear();
        mesh.doorLinks.clear();
        const auto read16 = [&](std::size_t offset)
        {
            std::uint16_t value{};
            std::memcpy(&value, trailing.data() + offset, sizeof(value));
            return value;
        };
        const auto read32 = [&](std::size_t offset)
        {
            std::uint32_t value{};
            std::memcpy(&value, trailing.data() + offset, sizeof(value));
            return value;
        };
        if (trailing.size() < 4)
        {
            return false;
        }
        const auto externalCount = read32(0);
        if (externalCount > (trailing.size() - 4) / 10)
        {
            return false;
        }
        const auto doorOffset = 4 + static_cast<std::size_t>(externalCount) * 10;
        if (trailing.size() - doorOffset < 4)
        {
            return false;
        }
        const auto doorCount = read32(doorOffset);
        if (doorCount > (trailing.size() - doorOffset - 4) / 10)
        {
            return false;
        }
        // Only flagged edges consume the external table. Ordinary neighbor values
        // are local triangle indices, even when they happen to fit that table.
        std::vector<core::NavmeshExternalLink> externalLinks;
        for (std::size_t polygonIndex{}; polygonIndex < mesh.polygons.size(); ++polygonIndex)
        {
            const auto &polygon = mesh.polygons[polygonIndex];
            for (std::uint8_t edge{}; edge < 3; ++edge)
            {
                if (!(polygon.flags & (1U << edge)))
                {
                    continue;
                }
                if (polygon.neighbors[edge] >= externalCount)
                {
                    return false;
                }
                const auto offset = 4 + static_cast<std::size_t>(polygon.neighbors[edge]) * 10;
                externalLinks.push_back(
                    {static_cast<std::uint32_t>(polygonIndex), edge, read32(offset + 4), read16(offset + 8)});
            }
        }
        std::vector<core::NavmeshDoorLink> doorLinks;
        for (std::uint32_t index{}; index < doorCount; ++index)
        {
            const auto offset = doorOffset + 4 + static_cast<std::size_t>(index) * 10;
            const auto polygon = read16(offset);
            if (polygon >= mesh.polygons.size())
            {
                return false;
            }
            doorLinks.push_back({polygon, read32(offset + 6)});
        }
        mesh.externalLinks = std::move(externalLinks);
        mesh.doorLinks = std::move(doorLinks);
        return true;
    }

    std::set<std::uint32_t> CellsWithExistingNavmesh(const ResolvedLoadOrder &resolved)
    {
        std::set<std::uint32_t> cells;
        for (const auto &record : resolved.records)
        {
            if (record.type == "NAVM" && record.cellFormId)
            {
                cells.insert(*record.cellFormId);
            }
        }
        return cells;
    }

    bool IPluginReader::ReadMetadata(const std::filesystem::path &path, std::vector<std::string> &masters,
                                     bool &isLight, std::vector<Diagnostic> &diagnostics) const
    {
        std::vector<ResolvedRecord> ignored;
        return Read(path, ignored, masters, isLight, diagnostics, false);
    }

    bool DirectPluginReader::ReadMetadata(const std::filesystem::path &path, std::vector<std::string> &masters,
                                          bool &isLight, std::vector<Diagnostic> &diagnostics) const
    {
        std::ifstream stream(path, std::ios::binary | std::ios::ate);
        const auto length = stream.tellg();
        std::vector<std::uint8_t> bytes(24);
        stream.seekg(0);
        stream.read(reinterpret_cast<char *>(bytes.data()), 24);
        if (!stream || Text(bytes, 0, 4) != "TES4")
        {
            diagnostics.push_back(
                {DiagnosticKind::InvalidPlugin, path.filename().string(), "Expected a TES4 plugin header."});
            return false;
        }
        const auto payloadSize = U32(bytes, 4);
        if (length < 24 || payloadSize > static_cast<std::uint64_t>(length) - 24 || payloadSize > kMaxDecodedRecordSize)
        {
            diagnostics.push_back({DiagnosticKind::InvalidPlugin, path.filename().string(),
                                   "TES4 header extends beyond the file or exceeds the safe decoded record size."});
            return false;
        }
        isLight = (U32(bytes, 8) & 0x200U) != 0;
        std::vector<std::uint8_t> payload(payloadSize);
        stream.read(reinterpret_cast<char *>(payload.data()), payloadSize);
        if (!stream)
        {
            diagnostics.push_back(
                {DiagnosticKind::InvalidPlugin, path.filename().string(), "Cannot read TES4 payload."});
            return false;
        }
        ResolvedRecord header{.type = "TES4"};
        header.raw = PluginRecord{.type = "TES4"};
        header.raw->filePayload = std::move(payload);
        header.raw->decodedPayload = header.raw->filePayload;
        std::string error;
        if (!ParseSubrecords(*header.raw, header, &masters, error))
        {
            diagnostics.push_back({DiagnosticKind::MalformedInput, path.filename().string(), "TES4 " + error});
            return false;
        }
        return true;
    }

    bool DirectPluginReader::Read(const std::filesystem::path &path, std::vector<ResolvedRecord> &records,
                                  std::vector<std::string> &masters, bool &isLight,
                                  std::vector<Diagnostic> &diagnostics, bool includeReferencesAndNavmeshes) const
    {
        std::error_code sizeError;
        if (!includeReferencesAndNavmeshes && std::filesystem::file_size(path, sizeError) > 512ULL * 1024ULL * 1024ULL)
        {
            diagnostics.push_back({DiagnosticKind::UnsupportedRecord, path.filename().string(),
                                   "Plugin exceeds the 512 MiB list-cells reader limit; it was skipped rather than "
                                   "risking an out-of-memory failure."});
            return false;
        }
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        const auto length = file.tellg();
        std::vector<std::uint8_t> b;
        if (length >= 0)
        {
            b.resize(static_cast<std::size_t>(length));
            file.seekg(0);
            file.read(reinterpret_cast<char *>(b.data()), length);
            if (!file)
            {
                b.clear();
            }
        }
        if (b.size() < 24 || Text(b, 0, 4) != "TES4")
        {
            diagnostics.push_back(
                {DiagnosticKind::InvalidPlugin, path.filename().string(), "Expected a TES4 plugin header."});
            return false;
        }
        const auto headerEnd = 24ULL + U32(b, 4);
        if (headerEnd > b.size())
        {
            diagnostics.push_back(
                {DiagnosticKind::InvalidPlugin, path.filename().string(), "TES4 header extends beyond the file."});
            return false;
        }
        isLight = (U32(b, 8) & 0x200U) != 0;
        ResolvedRecord header{.type = "TES4"};
        header.raw = PluginRecord{.type = "TES4", .headerRange = {0, 24}, .filePayloadRange = {24, headerEnd - 24}};
        header.raw->filePayload = std::vector<std::uint8_t>(b.begin() + 24, b.begin() + headerEnd);
        header.raw->decodedPayload = header.raw->filePayload;
        std::string headerError;
        if (!ParseSubrecords(*header.raw, header, &masters, headerError))
        {
            diagnostics.push_back({DiagnosticKind::MalformedInput, path.filename().string(), "TES4 " + headerError});
            return false;
        }
        const auto plugin = path.filename().string();
        const auto walk = [&](const auto &self, std::size_t begin, std::size_t end, std::uint32_t cell,
                              std::uint32_t world, bool persistent, bool temporary,
                              std::vector<std::array<std::uint8_t, 24>> groups) -> void
        {
            for (auto p = begin; p + 24 <= end;)
            {
                const auto kind = Text(b, p, 4);
                const auto size = U32(b, p + 4);
                const auto next = p + static_cast<std::size_t>(kind == "GRUP" ? size : 24ULL + size);
                if ((kind == "GRUP" && size < 24) || next > end)
                {
                    diagnostics.push_back(
                        {DiagnosticKind::InvalidPlugin, plugin, "A record/group size exceeds its containing group."});
                    return;
                }
                if (kind == "GRUP")
                {
                    const auto label = U32(b, p + 8);
                    const auto groupType = U32(b, p + 12);
                    std::array<std::uint8_t, 24> groupHeader{};
                    std::copy_n(b.begin() + p, 24, groupHeader.begin());
                    groups.push_back(groupHeader);
                    self(self, p + 24, next,
                         groupType == 6 || groupType == 8 || groupType == 9 || groupType == 10 ? label : cell,
                         groupType == 1 ? label : world, persistent || groupType == 8, temporary || groupType == 9,
                         groups);
                    groups.pop_back();
                }
                else
                {
                    const auto flags = U32(b, p + 8);
                    const auto payload = p + 24;
                    ResolvedRecord r{.type = kind,
                                     .formId = U32(b, p + 12),
                                     .winning = {plugin, U32(b, p + 12), {p, 24}},
                                     .cellFormId = cell ? std::optional(cell) : std::nullopt,
                                     .worldspaceFormId = world ? std::optional(world) : std::nullopt,
                                     .persistent = persistent,
                                     .temporary = temporary};
                    const auto retain =
                        IndexedRecordType(kind) && (includeReferencesAndNavmeshes || kind == "CELL" || kind == "WRLD");
                    if (retain)
                    {
                        if (kind == "CELL" || kind == "WRLD" || kind == "NAVM")
                        {
                            r.groupHeaders = groups;
                        }
                        r.raw = PluginRecord{.type = kind,
                                             .flags = flags,
                                             .headerRange = {p, 24},
                                             .filePayloadRange = {payload, size},
                                             .compressed = (flags & 0x40000U) != 0};
                        r.raw->filePayload = std::vector<std::uint8_t>(b.begin() + payload, b.begin() + next);
                        std::vector<std::uint8_t> decoded;
                        std::string error;
                        const bool decodedOk =
                            !r.raw->compressed || DecodePayload(b, payload, next, true, decoded, error);
                        r.raw->decodedPayload =
                            r.raw->compressed ? core::SharedBytes(std::move(decoded)) : r.raw->filePayload;
                        if (!decodedOk)
                        {
                            diagnostics.push_back({r.raw->compressed ? DiagnosticKind::DecompressionFailure
                                                                     : DiagnosticKind::MalformedInput,
                                                   plugin, kind + " " + error});
                        }
                        else if (!ParseSubrecords(*r.raw, r, nullptr, error))
                        {
                            diagnostics.push_back({DiagnosticKind::MalformedInput, plugin, kind + " " + error});
                        }
                        else if (r.navm && !r.navm->supported)
                        {
                            diagnostics.push_back(
                                {DiagnosticKind::UnsupportedVersion, plugin,
                                 kind + " NVNM version " + std::to_string(r.navm->version) +
                                     " is preserved but not decoded beyond verified arrays (expected 12)."});
                        }
                    }
                    // Milestone 1 indexes the record families that establish world/cell ownership and
                    // reference/NAVM overrides. Retaining every Skyrim record exhausts memory on large MO2 lists.
                    if (retain)
                    {
                        records.push_back(std::move(r));
                    }
                }
                p = next;
            }
        };
        walk(walk, headerEnd, b.size(), 0, 0, false, false, {});
        return true;
    }

    std::vector<std::filesystem::path> ReadLoadOrderManifest(const std::filesystem::path &manifest)
    {
        std::ifstream in(manifest);
        std::vector<std::filesystem::path> result;
        std::string line;
        while (std::getline(in, line))
        {
            line = Trim(line);
            if (!line.empty() && line[0] != '#')
            {
                if (line[0] == '*')
                {
                    line.erase(0, 1);
                }
                result.emplace_back(Trim(line));
            }
        }
        return result;
    }

    ResolvedLoadOrder ResolveLoadOrder(const LoadOrderInput &input, const IPluginReader &reader)
    {
        struct Source
        {
            std::filesystem::path path;
            std::vector<std::string> masters;
            bool light{}, metadataValid{};
            std::vector<Diagnostic> diagnostics;
        };
        struct ReadEvent
        {
            std::size_t source{};
            std::optional<Diagnostic> duplicate;
        };
        ResolvedLoadOrder result;
        std::vector<Source> sources;
        std::vector<ReadEvent> readEvents;
        std::unordered_map<std::string, std::size_t> byName;
        for (std::size_t requestedIndex = 0; requestedIndex < input.plugins.size(); ++requestedIndex)
        {
            const auto &requested = input.plugins[requestedIndex];
            auto path = requested.is_absolute() ? requested : input.dataDirectory / requested;
            const auto name = Lower(path.filename().string());
            if (byName.contains(name))
            {
                readEvents.push_back(
                    {.duplicate = Diagnostic{DiagnosticKind::DuplicatePlugin, path.filename().string(),
                                             "The load-order manifest names this plugin more than once."}});
                continue;
            }
            Source source{.path = path};
            source.metadataValid = reader.ReadMetadata(path, source.masters, source.light, source.diagnostics);
            byName.emplace(name, sources.size());
            result.plugins.push_back(path.filename().string());
            readEvents.push_back({.source = sources.size()});
            sources.push_back(std::move(source));
        }
        for (const auto &source : sources)
        {
            for (const auto &master : source.masters)
            {
                if (!byName.contains(Lower(master)))
                {
                    result.diagnostics.push_back(
                        {DiagnosticKind::MissingMaster, source.path.filename().string(),
                         "Missing master '" + master + "'. Add it before this plugin in --load-order."});
                }
            }
        }
        for (std::size_t i = 0; i < sources.size(); ++i)
        {
            for (const auto &master : sources[i].masters)
            {
                if (const auto it = byName.find(Lower(master)); it != byName.end() && it->second >= i)
                {
                    result.diagnostics.push_back({DiagnosticKind::Cycle, sources[i].path.filename().string(),
                                                  "Master '" + master + "' must occur earlier in the load order."});
                }
            }
        }
        std::vector<std::uint32_t> full(sources.size()), light(sources.size());
        std::uint32_t nextFull = 0, nextLight = 0;
        for (std::size_t i = 0; i < sources.size(); ++i)
        {
            if (sources[i].light)
            {
                light[i] = nextLight++;
            }
            else
            {
                full[i] = nextFull++;
            }
        }
        const auto resolve = [&](std::size_t owner, std::uint32_t raw) -> std::optional<std::uint32_t>
        {
            const auto hi = raw >> 24;
            // A file's FormID prefix indexes its master table from zero. New
            // records use the index immediately after the last master.
            std::size_t target = owner;
            if (hi < sources[owner].masters.size())
            {
                const auto it = byName.find(Lower(sources[owner].masters[hi]));
                if (it == byName.end())
                {
                    return std::nullopt;
                }
                target = it->second;
            }
            else if (hi != sources[owner].masters.size())
            {
                return std::nullopt;
            }
            if (sources[target].light)
            {
                return 0xFE000000U | (light[target] << 12) | (raw & 0xFFFU);
            }
            return full[target] << 24 | (raw & 0xFFFFFFU);
        };
        std::unordered_map<std::uint32_t, std::size_t> winners;
        for (std::size_t i = 0; i < sources.size(); ++i)
        {
            auto &source = sources[i];
            if (!source.metadataValid)
            {
                continue;
            }
            if (input.progress)
            {
                input.progress(i, sources.size(), source.path);
            }
            // Metadata fixes master indices first; transfer records from just one plugin into winners at a time.
            std::vector<std::string> masters;
            std::vector<Diagnostic> diagnostics;
            bool isLight{};
            std::vector<ResolvedRecord> records;
            const auto read =
                reader.Read(source.path, records, masters, isLight, diagnostics, input.indexReferencesAndNavmeshes);
            source.diagnostics = std::move(diagnostics);
            if (read && (masters != source.masters || isLight != source.light))
            {
                result.diagnostics.push_back(
                    {DiagnosticKind::InvalidPlugin, source.path.filename().string(),
                     "Plugin master metadata changed while resolving its records; rerun with coherent inputs."});
                continue;
            }
            for (auto &sourceRecord : records)
            {
                auto record = std::move(sourceRecord);
                const auto global = resolve(i, record.formId);
                if (!global)
                {
                    result.diagnostics.push_back({DiagnosticKind::UnresolvedFormId, sources[i].path.filename().string(),
                                                  "Cannot resolve FormID " + std::to_string(record.formId) + "."});
                    continue;
                }
                record.formId = *global;
                record.winning.formId = *global;
                if (record.cellFormId)
                {
                    record.cellFormId = resolve(i, *record.cellFormId);
                }
                if (record.worldspaceFormId)
                {
                    record.worldspaceFormId = resolve(i, *record.worldspaceFormId);
                }
                for (auto &reference : record.referencedFormIds)
                {
                    const auto resolvedReference = resolve(i, reference);
                    if (!resolvedReference)
                    {
                        result.diagnostics.push_back({DiagnosticKind::UnresolvedFormId,
                                                      sources[i].path.filename().string(),
                                                      "Cannot resolve referenced FormID " + std::to_string(reference) +
                                                          " in " + record.type + "."});
                    }
                    else
                    {
                        reference = *resolvedReference;
                    }
                }
                record.winning.cellFormId = record.cellFormId;
                record.winning.worldspaceFormId = record.worldspaceFormId;
                if (!record.referencedFormIds.empty())
                {
                    record.winning.baseFormId = record.referencedFormIds.front();
                }
                if (record.transform)
                {
                    record.winning.position =
                        core::Vec3{(*record.transform)[0], (*record.transform)[1], (*record.transform)[2]};
                }
                record.winning.scale = record.referenceScale.value_or(1.0F);
                record.winning.hasModel = record.modelPath && !record.modelPath->empty();
                if (record.raw)
                {
                    for (const auto &sub : record.raw->subrecords)
                    {
                        if (sub.type == "OBND" && sub.data.size() == 12)
                        {
                            std::array<std::int16_t, 6> bounds{};
                            std::memcpy(bounds.data(), sub.data.data(), 12);
                            const auto x = static_cast<float>(
                                std::max(std::abs(static_cast<int>(bounds[0])), std::abs(static_cast<int>(bounds[3]))));
                            const auto y = static_cast<float>(
                                std::max(std::abs(static_cast<int>(bounds[1])), std::abs(static_cast<int>(bounds[4]))));
                            const auto z = static_cast<float>(
                                std::max(std::abs(static_cast<int>(bounds[2])), std::abs(static_cast<int>(bounds[5]))));
                            record.winning.modelRadius = std::hypot(std::hypot(x, y), z);
                        }
                    }
                }
                if (const auto found = winners.find(*global); found != winners.end())
                {
                    // LAND overrides commonly change texture/colour data without
                    // repeating VHGT.  Preserve the inherited height subrecord in
                    // the resolved representation so terrain remains the winning
                    // LAND surface rather than being misreported as absent.
                    if (record.type == "LAND" && record.raw && result.records[found->second].raw)
                    {
                        const auto hasVhgt = [](const PluginRecord &raw)
                        {
                            return std::any_of(raw.subrecords.begin(), raw.subrecords.end(),
                                               [](const auto &sub) { return sub.type == "VHGT"; });
                        };
                        if (!hasVhgt(*record.raw))
                        {
                            const auto inherited = std::find_if(result.records[found->second].raw->subrecords.begin(),
                                                                result.records[found->second].raw->subrecords.end(),
                                                                [](const auto &sub) { return sub.type == "VHGT"; });
                            if (inherited != result.records[found->second].raw->subrecords.end())
                            {
                                record.raw->subrecords.push_back(*inherited);
                            }
                        }
                    }
                    record.winning.navigationChanged = !NavigationEquivalent(record, result.records[found->second]);
                    auto origins = std::move(result.records[found->second].origins);
                    origins.push_back(record.winning);
                    record.origins = std::move(origins);
                    result.records[found->second] = std::move(record);
                }
                else
                {
                    record.origins.push_back(record.winning);
                    winners.emplace(*global, result.records.size());
                    result.records.push_back(std::move(record));
                }
            }
        }
        if (input.progress)
        {
            input.progress(sources.size(), sources.size(), {});
        }
        std::unordered_map<std::uint32_t, std::size_t> cellsById;
        for (const auto &record : result.records)
        {
            if (record.type == "CELL")
            {
                cellsById.emplace(record.formId, result.cells.size());
                result.cells.push_back({.id = record.formId,
                                        .editorId = record.editorId,
                                        .name = record.name,
                                        .isInterior = !record.exteriorCoordinates.has_value(),
                                        .exteriorCoordinates = record.exteriorCoordinates});
            }
        }
        for (const auto &record : result.records)
        {
            if (record.type != "NAVM" || !record.cellFormId)
            {
                continue;
            }
            const auto cell = cellsById.find(*record.cellFormId);
            if (cell == cellsById.end())
            {
                continue;
            }
            if (auto mesh = DecodeNavMesh(record))
            {
                // Connection identities use the winning file's master table, which
                // can differ from both the NAVM's origin and global load-order slots.
                const auto owner = byName.find(Lower(record.winning.plugin));
                if (owner != byName.end())
                {
                    for (auto &link : mesh->externalLinks)
                    {
                        link.navmeshId = link.navmeshId ? resolve(owner->second, link.navmeshId).value_or(0) : 0;
                    }
                    for (auto &link : mesh->doorLinks)
                    {
                        link.referenceId = link.referenceId ? resolve(owner->second, link.referenceId).value_or(0) : 0;
                    }
                    std::erase_if(mesh->externalLinks, [](const auto &link) { return link.navmeshId == 0; });
                    std::erase_if(mesh->doorLinks, [](const auto &link) { return link.referenceId == 0; });
                }
                result.cells[cell->second].navMeshes.push_back(*mesh);
            }
        }
        // Scene assembly intentionally uses only winning records: the reference and
        // its base object may originate in different plugins, both of which remain
        // attached as provenance for the neutral scene boundary.
        for (const auto &record : result.records)
        {
            if ((record.type != "REFR" && record.type != "ACHR") || !record.cellFormId)
            {
                continue;
            }
            const auto cell = cellsById.find(*record.cellFormId);
            if (cell == cellsById.end())
            {
                continue;
            }
            core::Reference reference{.id = record.formId,
                                      .recordType = record.type,
                                      .editorId = record.editorId,
                                      .sourcePlugin = record.winning.plugin};
            if (record.raw)
            {
                reference.initiallyDisabled = (record.raw->flags & (1U << 11)) != 0;
                reference.deleted = (record.raw->flags & (1U << 5)) != 0;
                reference.teleportExit =
                    std::any_of(record.raw->subrecords.begin(), record.raw->subrecords.end(), [](const auto &sub)
                                { return sub.type == "XTEL" && sub.data.size() >= 4 && U32(sub.data, 0) != 0; });
            }
            if (!record.referencedFormIds.empty())
            {
                reference.baseObjectId = record.referencedFormIds.front();
            }
            if (record.transform)
            {
                const auto &transform = *record.transform;
                reference.position = {transform[0], transform[1], transform[2]};
                reference.rotation = {transform[3], transform[4], transform[5]};
            }
            if (record.referenceScale)
            {
                reference.scale = *record.referenceScale;
            }
            if (const auto baseIndex = winners.find(reference.baseObjectId); baseIndex != winners.end())
            {
                const auto *base = &result.records[baseIndex->second];
                reference.modelPath = base->modelPath.value_or("");
                reference.basePlugin = base->winning.plugin;
                reference.baseRecordType = base->type;
            }
            // A reference-local MODL is unusual, but remains a valid resolved source.
            if (reference.modelPath.empty() && record.modelPath)
            {
                reference.modelPath = *record.modelPath;
            }
            result.cells[cell->second].references.push_back(std::move(reference));
        }
        for (std::size_t index{}; index < result.records.size(); ++index)
        {
            const auto &record = result.records[index];
            if (record.type == "LAND" && record.cellFormId)
            {
                result.landIndex[*record.cellFormId].push_back(index);
            }
        }
        result.recordIndex = std::move(winners);
        std::vector<Diagnostic> inputDiagnostics;
        for (const auto &event : readEvents)
        {
            if (event.duplicate)
            {
                inputDiagnostics.push_back(*event.duplicate);
            }
            else
            {
                auto &diagnostics = sources[event.source].diagnostics;
                inputDiagnostics.insert(inputDiagnostics.end(), std::make_move_iterator(diagnostics.begin()),
                                        std::make_move_iterator(diagnostics.end()));
            }
        }
        result.diagnostics.insert(result.diagnostics.begin(), std::make_move_iterator(inputDiagnostics.begin()),
                                  std::make_move_iterator(inputDiagnostics.end()));
        return result;
    }

    std::vector<const ResolvedRecord *> ResolvedLoadOrder::LandRecords(std::uint32_t cellId) const
    {
        std::vector<const ResolvedRecord *> result;
        if (const auto found = landIndex.find(cellId); found != landIndex.end())
        {
            for (const auto offset : found->second)
            {
                result.push_back(&records.at(offset));
            }
        }
        else if (recordIndex.empty())
        {
            for (const auto &record : records)
            {
                if (record.type == "LAND" && record.cellFormId == cellId)
                {
                    result.push_back(&record);
                }
            }
        }
        return result;
    }

    const ResolvedRecord *ResolvedLoadOrder::FindWinning(std::uint32_t formId) const
    {
        if (const auto it = recordIndex.find(formId); it != recordIndex.end())
        {
            return &records[it->second];
        }
        if (!recordIndex.empty())
        {
            return nullptr;
        }
        const auto it = std::find_if(records.begin(), records.end(), [=](const auto &r) { return r.formId == formId; });
        return it == records.end() ? nullptr : &*it;
    }
} // namespace navmesh::skyrim
