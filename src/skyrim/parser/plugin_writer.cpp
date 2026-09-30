#include "skyrim/parser/plugin_writer.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>

namespace
{
    using Bytes = std::vector<std::uint8_t>;
    using navmesh::skyrim::offline::ResolvedRecord;

    void U16(Bytes& b, std::uint16_t n) { b.push_back(static_cast<std::uint8_t>(n)); b.push_back(static_cast<std::uint8_t>(n >> 8)); }
    void U32(Bytes& b, std::uint32_t n) { for (int shift{}; shift < 32; shift += 8) b.push_back(static_cast<std::uint8_t>(n >> shift)); }
    std::uint32_t Get32(const Bytes& b, std::size_t p) { return static_cast<std::uint32_t>(b[p]) | static_cast<std::uint32_t>(b[p + 1]) << 8 | static_cast<std::uint32_t>(b[p + 2]) << 16 | static_cast<std::uint32_t>(b[p + 3]) << 24; }
    void Set32(Bytes& b, std::size_t p, std::uint32_t n) { for (int shift{}; shift < 32; shift += 8) b[p + shift / 8] = static_cast<std::uint8_t>(n >> shift); }
    void Float(Bytes& b, float n) { std::uint32_t bits{}; std::memcpy(&bits, &n, 4); U32(b, bits); }
    void PutSubrecord(Bytes& b, const char* kind, const Bytes& data)
    {
        if (data.size() > std::numeric_limits<std::uint16_t>::max()) {
            b.insert(b.end(), {'X','X','X','X'}); U16(b, 4); U32(b, static_cast<std::uint32_t>(data.size()));
        }
        b.insert(b.end(), kind, kind + 4); U16(b, data.size() > std::numeric_limits<std::uint16_t>::max() ? 0 : static_cast<std::uint16_t>(data.size()));
        b.insert(b.end(), data.begin(), data.end());
    }
    bool SameName(std::string a, std::string b)
    {
        std::transform(a.begin(), a.end(), a.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        std::transform(b.begin(), b.end(), b.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return a == b;
    }
    struct SourceInfo
    {
        std::string name;
        Bytes bytes;
        std::vector<std::string> masters;
    };
    /// Decode only the TES4 master names; the resolved load order already supplied the winning NAVMs.
    bool ReadMasterNames(const Bytes& source, std::vector<std::string>& masters)
    {
        if (source.size() < 24 || std::memcmp(source.data(), "TES4", 4) != 0) return false;
        const auto headerSize = static_cast<std::size_t>(Get32(source, 4));
        if (headerSize > source.size() - 24) return false;
        const auto end = 24 + headerSize;
        std::optional<std::uint32_t> extended;
        for (std::size_t p = 24; p < end;) {
            if (end - p < 6) return false;
            const std::string type(reinterpret_cast<const char*>(source.data() + p), 4);
            const auto shortSize = static_cast<std::uint16_t>(source[p + 4] | source[p + 5] << 8);
            p += 6;
            if (type == "XXXX") {
                if (shortSize != 4 || end - p < 4) return false;
                extended = Get32(source, p); p += 4; continue;
            }
            const auto size = static_cast<std::size_t>(extended.value_or(shortSize));
            if (size > end - p) return false;
            if (type == "MAST") {
                const auto zero = std::find(source.begin() + p, source.begin() + p + size, 0);
                if (zero == source.begin() + p || zero == source.begin() + p + size) return false;
                masters.emplace_back(source.begin() + p, zero);
            }
            p += size; extended.reset();
        }
        return !extended;
    }
    /// Convert a source-local FormID index into the generated plugin's master table.
    std::optional<std::uint32_t> RebaseFormId(const SourceInfo& source, const std::vector<std::string>& outputMasters, std::uint32_t raw)
    {
        const auto index = static_cast<std::size_t>(raw >> 24);
        if (index > source.masters.size()) return std::nullopt;
        const auto& owner = index == source.masters.size() ? source.name : source.masters[index];
        const auto it = std::find_if(outputMasters.begin(), outputMasters.end(), [&](const auto& master) { return SameName(master, owner); });
        if (it == outputMasters.end()) return std::nullopt;
        return static_cast<std::uint32_t>(std::distance(outputMasters.begin(), it)) << 24 | (raw & 0xFFFFFFU);
    }
    Bytes SourceRecord(const Bytes& source, const ResolvedRecord& record)
    {
        const auto start = static_cast<std::size_t>(record.raw->headerRange.offset);
        const auto size = 24 + static_cast<std::size_t>(record.raw->filePayloadRange.size);
        if (start > source.size() || size > source.size() - start) return {};
        return Bytes(source.begin() + start, source.begin() + start + size);
    }
    struct Group
    {
        std::array<std::uint8_t, 24> header{};
        Bytes records;
        std::vector<Group> children;
    };
    void Add(Group& root, const ResolvedRecord& record, Bytes bytes)
    {
        auto* group = &root;
        for (const auto& header : record.groupHeaders) {
            auto it = std::find_if(group->children.begin(), group->children.end(), [&](const Group& child) {
                return std::equal(header.begin() + 8, header.begin() + 16, child.header.begin() + 8);
            });
            if (it == group->children.end()) { group->children.push_back({ .header = header }); it = std::prev(group->children.end()); }
            group = &*it;
        }
        group->records.insert(group->records.end(), bytes.begin(), bytes.end());
    }
    Bytes Serialize(const Group& group)
    {
        Bytes body = group.records;
        for (const auto& child : group.children) { auto bytes = Serialize(child); body.insert(body.end(), bytes.begin(), bytes.end()); }
        if (group.header[0] != 'G') return body;
        Bytes output(group.header.begin(), group.header.end());
        Set32(output, 4, static_cast<std::uint32_t>(24 + body.size()));
        output.insert(output.end(), body.begin(), body.end());
        return output;
    }
    bool ReadBack(const std::filesystem::path& path, std::uint32_t cellId,
        const std::vector<std::string>& expectedMasters, bool expectedLight,
        const std::vector<std::pair<std::uint32_t, navmesh::core::NavMesh>>& expected)
    {
        std::vector<ResolvedRecord> records; std::vector<std::string> masters;
        std::vector<navmesh::skyrim::offline::Diagnostic> diagnostics; bool light{};
        if (!navmesh::skyrim::offline::DirectPluginReader{}.Read(path, records, masters, light, diagnostics) || !diagnostics.empty()) return false;
        if (masters != expectedMasters || light != expectedLight) return false;
        if (static_cast<std::size_t>(std::count_if(records.begin(), records.end(), [](const auto& record) { return record.type == "NAVM"; })) != expected.size()) return false;
        for (const auto& [originalId, mesh] : expected) {
        const auto it = std::find_if(records.begin(), records.end(), [&](const auto& record) { return record.type == "NAVM" && record.winning.formId == originalId; });
        if (it == records.end() || it->cellFormId != cellId || !it->raw || !it->navm || !it->navm->supported || it->navm->vertexCount != mesh.vertices.size() || it->navm->triangleCount != mesh.polygons.size()) return false;
        const auto& data = it->raw->decodedPayload;
        for (std::size_t i{}; i < mesh.vertices.size(); ++i)
            if (std::memcmp(data.data() + it->navm->vertices.offset + i * 12, &mesh.vertices[i], 12) != 0) return false;
        for (std::size_t i{}; i < mesh.polygons.size(); ++i) for (int j{}; j < 3; ++j) {
            const auto offset = static_cast<std::size_t>(it->navm->triangles.offset) + i * 16;
            const auto vertex = static_cast<std::uint16_t>(data[offset + j * 2] | data[offset + j * 2 + 1] << 8);
            if (vertex != mesh.polygons[i].vertices[j]) return false;
            const auto neighbor = static_cast<std::uint16_t>(data[offset + 6 + j * 2] | data[offset + 7 + j * 2] << 8);
            if (neighbor != (mesh.polygons[i].neighbors[j] == std::numeric_limits<std::uint32_t>::max() ? 0xffffU : mesh.polygons[i].neighbors[j])) return false;
        }
        for (std::size_t i{}; i < mesh.polygons.size(); ++i) {
            const auto offset = static_cast<std::size_t>(it->navm->triangles.offset) + i * 16;
            if (static_cast<std::uint16_t>(data[offset + 12] | data[offset + 13] << 8) != mesh.polygons[i].flags) return false;
        }
        }
        return true;
    }
}

bool navmesh::skyrim::offline::WriteNavmeshOverride(const std::filesystem::path& outputDirectory,
    const std::vector<std::filesystem::path>& inputPlugins, const ResolvedLoadOrder& resolved,
    const core::Cell& cell, const core::CandidateNavMesh& candidate,
    std::filesystem::path& writtenPath, std::string& error)
{
    const auto fail = [&](const char* reason) { error = reason; return false; };
    if (!candidate.topology.valid || candidate.mesh.vertices.empty() || candidate.mesh.polygons.empty()) return fail("Generated NAVM is empty or has invalid topology.");
    if (candidate.mesh.vertices.size() > 65535 || candidate.mesh.polygons.size() > 65535) return fail("Generated NAVM exceeds 16-bit vertex or triangle indices.");
    if (cell.navMeshes.empty()) return fail("The selected cell has no existing NAVM records to override.");
    if (cell.exteriorCoordinates) for (const auto& vertex : candidate.mesh.vertices) {
        const auto [x, y] = *cell.exteriorCoordinates;
        if (vertex.x < static_cast<float>(x) * 4096.0F || vertex.x > (static_cast<float>(x) + 1.0F) * 4096.0F
            || vertex.y < static_cast<float>(y) * 4096.0F || vertex.y > (static_cast<float>(y) + 1.0F) * 4096.0F)
            return fail("Generated NAVM extends outside the selected exterior CELL.");
    }
    std::vector<const ResolvedRecord*> records;
    for (const auto& mesh : cell.navMeshes) {
        const auto* navm = resolved.FindWinning(mesh.id);
        if (!navm || navm->type != "NAVM" || !navm->raw || !navm->navm || !navm->navm->supported)
            return fail("A selected NAVM has an unsupported format.");
        if (navm->groupHeaders.empty() || navm->cellFormId != cell.id)
            return fail("A selected NAVM has no supported cell group placement.");
        for (const auto& sub : navm->raw->subrecords) if (sub.type != "NVNM" && sub.type != "EDID")
            return fail("An existing NAVM has additional authored subrecords that cannot be remapped.");
        records.push_back(navm);
    }
    std::vector<SourceInfo> sources;
    for (const auto* navm : records) {
        const auto name = navm->winning.plugin;
        if (std::any_of(sources.begin(), sources.end(), [&](const auto& source) { return SameName(source.name, name); })) continue;
        const auto path = std::find_if(inputPlugins.begin(), inputPlugins.end(), [&](const auto& p) { return SameName(p.filename().string(), name); });
        if (path == inputPlugins.end()) return fail("Winning NAVM source plugin is absent from the input paths.");
        std::ifstream file(*path, std::ios::binary);
        Bytes bytes((std::istreambuf_iterator<char>(file)), {});
        if (bytes.size() < 24 || std::memcmp(bytes.data(), "TES4", 4) != 0) return fail("Cannot read winning NAVM source plugin.");
        std::vector<std::string> sourceMasters;
        if (!ReadMasterNames(bytes, sourceMasters)) return fail("Cannot read source master list.");
        sources.push_back({ name, std::move(bytes), std::move(sourceMasters) });
    }
    std::vector<std::string> masters;
    for (const auto& path : inputPlugins) {
        const auto name = path.filename().string();
        const auto required = std::any_of(sources.begin(), sources.end(), [&](const auto& source) {
            return SameName(source.name, name) || std::any_of(source.masters.begin(), source.masters.end(), [&](const auto& master) { return SameName(master, name); });
        });
        if (required && std::none_of(masters.begin(), masters.end(), [&](const auto& master) { return SameName(master, name); })) masters.push_back(name);
    }
    for (const auto& source : sources) {
        if (std::none_of(masters.begin(), masters.end(), [&](const auto& master) { return SameName(master, source.name); })) return fail("A NAVM source is absent from the input load order.");
        for (const auto& required : source.masters)
            if (std::none_of(masters.begin(), masters.end(), [&](const auto& master) { return SameName(master, required); }))
                return fail("A required source master is absent from the input load order.");
    }
    if (masters.size() > 254) return fail("Too many masters for a generated plugin.");
    // Every emitted NAVM overrides a FormID from a listed master, so no new light-plugin FormIDs are allocated.
    const bool light = masters.size() <= 253;
    for (const auto& master : masters) {
        const auto it = std::find_if(inputPlugins.begin(), inputPlugins.end(), [&](const auto& p) { return SameName(p.filename().string(), master); });
        std::ifstream file(*it, std::ios::binary);
        std::array<std::uint8_t, 12> header{}; file.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(header.size()));
        if (!file || std::memcmp(header.data(), "TES4", 4) != 0) return fail("Cannot read a required source master.");
    }
    const auto path = outputDirectory / "generated-navmesh.esp";
    if (std::filesystem::exists(path)) return fail("Plugin output already exists; choose an empty output folder.");
    if (std::filesystem::exists(outputDirectory / "generated-navmesh.esl"))
        return fail("An older generated-navmesh.esl exists in the output folder; choose an empty output folder.");
    const auto primary = static_cast<std::size_t>(std::distance(cell.navMeshes.begin(),
        std::max_element(cell.navMeshes.begin(), cell.navMeshes.end(), [](const auto& a, const auto& b) {
            return a.polygons.size() < b.polygons.size();
        })));
    // A candidate uses one triangle-index space; keeping it in one NAVM avoids invalid cross-record indices.
    Group root;
    std::vector<std::pair<std::uint32_t, core::NavMesh>> expected;
    std::optional<std::uint32_t> outputCellGroup;
    for (std::size_t index{}; index < records.size(); ++index) {
        const auto* navm = records[index];
        const auto sourceIt = std::find_if(sources.begin(), sources.end(), [&](const auto& source) { return SameName(source.name, navm->winning.plugin); });
        const auto& source = *sourceIt;
        const core::NavMesh emptyMesh;
        const core::NavMesh& mesh = index == primary ? candidate.mesh : emptyMesh;
        Bytes nvnm;
        const auto& old = navm->raw->decodedPayload;
        const auto& layout = *navm->navm;
        nvnm.insert(nvnm.end(), old.begin() + layout.header.offset, old.begin() + layout.header.offset + 16);
        if (Get32(nvnm, 8)) {
            const auto world = RebaseFormId(source, masters, Get32(nvnm, 8));
            if (!world) return fail("Cannot rebase NAVM worldspace FormID.");
            Set32(nvnm, 8, *world);
        } else if (!cell.exteriorCoordinates) {
            const auto interiorCell = RebaseFormId(source, masters, Get32(nvnm, 12));
            if (!interiorCell) return fail("Cannot rebase NAVM interior CELL FormID.");
            Set32(nvnm, 12, *interiorCell);
        }
        U32(nvnm, static_cast<std::uint32_t>(mesh.vertices.size()));
        for (const auto& vertex : mesh.vertices) {
            if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) || !std::isfinite(vertex.z)) return fail("Generated NAVM contains non-finite coordinates.");
            Float(nvnm, vertex.x); Float(nvnm, vertex.y); Float(nvnm, vertex.z);
        }
        U32(nvnm, static_cast<std::uint32_t>(mesh.polygons.size()));
        for (const auto& polygon : mesh.polygons) {
            for (const auto vertex : polygon.vertices) { if (vertex >= mesh.vertices.size()) return fail("Generated NAVM has an invalid vertex index."); U16(nvnm, static_cast<std::uint16_t>(vertex)); }
            for (const auto neighbor : polygon.neighbors) {
                if (neighbor != std::numeric_limits<std::uint32_t>::max() && neighbor >= mesh.polygons.size()) return fail("Generated NAVM has an invalid neighbor index.");
                U16(nvnm, neighbor == std::numeric_limits<std::uint32_t>::max() ? 0xffffU : static_cast<std::uint16_t>(neighbor));
            }
            U16(nvnm, polygon.flags); U16(nvnm, 0);
        }
        U32(nvnm, 0); U32(nvnm, 0); U32(nvnm, 0);
        float minX{}, minY{}, minZ{}, maxX{}, maxY{}, maxZ{};
        if (!mesh.vertices.empty()) {
            minX = minY = minZ = std::numeric_limits<float>::max();
            maxX = maxY = maxZ = std::numeric_limits<float>::lowest();
            for (const auto& v : mesh.vertices) { minX = std::min(minX, v.x); minY = std::min(minY, v.y); minZ = std::min(minZ, v.z); maxX = std::max(maxX, v.x); maxY = std::max(maxY, v.y); maxZ = std::max(maxZ, v.z); }
        }
        U32(nvnm, 1); Float(nvnm, maxX - minX); Float(nvnm, maxY - minY);
        for (float bound : { minX, minY, minZ, maxX, maxY, maxZ }) Float(nvnm, bound);
        U32(nvnm, static_cast<std::uint32_t>(mesh.polygons.size()));
        for (std::size_t triangle{}; triangle < mesh.polygons.size(); ++triangle) U16(nvnm, static_cast<std::uint16_t>(triangle));
        Bytes payload;
        for (const auto& sub : navm->raw->subrecords) {
            if (sub.type == "NVNM") PutSubrecord(payload, "NVNM", nvnm);
            else payload.insert(payload.end(), sub.encodedBytes.begin(), sub.encodedBytes.end());
        }
        Bytes bytes = SourceRecord(source.bytes, *navm);
        if (bytes.size() < 24) return fail("Source NAVM record range is invalid.");
        bytes.resize(24); Set32(bytes, 4, static_cast<std::uint32_t>(payload.size()));
        Set32(bytes, 8, Get32(bytes, 8) & ~(0x40000U | 0x20U));
        const auto rebasedId = RebaseFormId(source, masters, Get32(bytes, 12));
        if (!rebasedId) return fail("Cannot rebase source NAVM FormID.");
        Set32(bytes, 12, *rebasedId);
        bytes.insert(bytes.end(), payload.begin(), payload.end());
        expected.emplace_back(Get32(bytes, 12), mesh);
        ResolvedRecord placement;
        placement.groupHeaders = navm->groupHeaders;
        for (auto& header : placement.groupHeaders) {
            Bytes group(header.begin(), header.end());
            const auto groupType = Get32(group, 12);
            if (groupType == 1 || groupType == 6 || groupType == 8 || groupType == 9 || groupType == 10) {
                const auto label = RebaseFormId(source, masters, Get32(group, 8));
                if (!label) return fail("Cannot rebase NAVM group placement.");
                Set32(group, 8, *label);
                if (groupType == 6) {
                    if (outputCellGroup && *outputCellGroup != *label) return fail("NAVM records do not share the selected CELL group.");
                    outputCellGroup = *label;
                }
                std::copy(group.begin(), group.end(), header.begin());
            }
        }
        Add(root, placement, std::move(bytes));
    }
    const auto& source = sources.front().bytes;
    Bytes hedr;
    if (source.size() >= 42 && std::memcmp(source.data() + 24, "HEDR", 4) == 0 && source[28] == 12 && source[29] == 0)
        hedr.assign(source.begin() + 30, source.begin() + 42);
    else { Float(hedr, 1.7F); U32(hedr, 0); U32(hedr, 0x800); }
    Set32(hedr, 4, static_cast<std::uint32_t>(records.size()));
    Bytes headerPayload; PutSubrecord(headerPayload, "HEDR", hedr);
    for (const auto& master : masters) { Bytes name(master.begin(), master.end()); name.push_back(0); PutSubrecord(headerPayload, "MAST", name); PutSubrecord(headerPayload, "DATA", Bytes(8)); }
    Bytes output(source.begin(), source.begin() + 24);
    Set32(output, 4, static_cast<std::uint32_t>(headerPayload.size()));
    Set32(output, 8, light ? 0x200U : 0U);
    output.insert(output.end(), headerPayload.begin(), headerPayload.end());
    auto groups = Serialize(root); output.insert(output.end(), groups.begin(), groups.end());
    if (!outputDirectory.empty()) std::filesystem::create_directories(outputDirectory);
    auto temporary = path; temporary += ".tmp";
    if (std::filesystem::exists(temporary)) return fail("Temporary plugin path already exists.");
    { std::ofstream file(temporary, std::ios::binary); file.write(reinterpret_cast<const char*>(output.data()), static_cast<std::streamsize>(output.size())); if (!file) { std::filesystem::remove(temporary); return fail("Cannot write generated plugin."); } }
    if (!outputCellGroup || !ReadBack(temporary, *outputCellGroup, masters, light, expected)) {
        std::filesystem::remove(temporary); return fail("Generated plugin failed NAVM read-back verification.");
    }
    std::error_code renameError; std::filesystem::rename(temporary, path, renameError);
    if (renameError) { std::filesystem::remove(temporary); return fail("Cannot finalize generated plugin."); }
    writtenPath = path;
    return true;
}

