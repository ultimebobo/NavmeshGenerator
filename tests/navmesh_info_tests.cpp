#ifdef NDEBUG
#undef NDEBUG
#endif

#include "skyrim/parser/navmesh_info.h"
#include "skyrim/parser/plugin_writer.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <fstream>
#include <iterator>
#include <span>

#include <zlib.h>

namespace
{
    using Bytes = std::vector<std::uint8_t>;
    using namespace navmesh::skyrim;

    void Put32(Bytes &data, std::uint32_t value)
    {
        for (int shift{}; shift < 32; shift += 8)
        {
            data.push_back(static_cast<std::uint8_t>(value >> shift));
        }
    }

    void Put16(Bytes &data, std::uint16_t value)
    {
        data.push_back(static_cast<std::uint8_t>(value));
        data.push_back(static_cast<std::uint8_t>(value >> 8));
    }

    std::uint32_t Get32(std::span<const std::uint8_t> data, std::size_t offset)
    {
        assert(offset + 4 <= data.size());
        std::uint32_t value{};
        std::memcpy(&value, data.data() + offset, 4);
        return value;
    }

    float GetFloat(std::span<const std::uint8_t> data, std::size_t offset)
    {
        const auto bits = Get32(data, offset);
        float value{};
        std::memcpy(&value, &bits, 4);
        return value;
    }

    void Sub(Bytes &data, const char *type, const Bytes &payload)
    {
        data.insert(data.end(), type, type + 4);
        Put16(data, static_cast<std::uint16_t>(payload.size()));
        data.insert(data.end(), payload.begin(), payload.end());
    }

    void Record(Bytes &data, const char *type, std::uint32_t id, const Bytes &payload, std::uint32_t flags = 0)
    {
        data.insert(data.end(), type, type + 4);
        Put32(data, static_cast<std::uint32_t>(payload.size()));
        Put32(data, flags);
        Put32(data, id);
        Put32(data, 0);
        Put32(data, 44);
        data.insert(data.end(), payload.begin(), payload.end());
    }

    void Group(Bytes &data, std::uint32_t label, std::uint32_t type, const Bytes &payload)
    {
        data.insert(data.end(), {'G', 'R', 'U', 'P'});
        Put32(data, static_cast<std::uint32_t>(payload.size() + 24));
        Put32(data, label);
        Put32(data, type);
        Put32(data, 0);
        Put32(data, 0);
        data.insert(data.end(), payload.begin(), payload.end());
    }

    Bytes Geometry(std::uint32_t cell, bool connected = false)
    {
        Bytes data;
        for (const auto value : {12U, 0xA5E9A03CU, 0U, cell, 3U})
        {
            Put32(data, value);
        }
        const navmesh::core::Vec3 points[] = {{0, 0, 6}, {12, 0, 6}, {0, 12, 6}};
        const auto *bytes = reinterpret_cast<const std::uint8_t *>(points);
        data.insert(data.end(), bytes, bytes + sizeof(points));
        Put32(data, 1);
        for (const auto value : {0U, 1U, 2U, connected ? 0U : 65535U, 65535U, 65535U, connected ? 65U : 0U, 0U})
        {
            Put16(data, static_cast<std::uint16_t>(value));
        }
        Put32(data, connected ? 1 : 0);
        if (connected)
        {
            Put32(data, 0);
            Put32(data, 0x01000901);
            Put16(data, 0);
        }
        Put32(data, connected ? 1 : 0);
        if (connected)
        {
            Put16(data, 0);
            Put32(data, 0xE48B73F3);
            Put32(data, 0x01000200);
        }
        Put32(data, 0);
        Put32(data, 1);
        data.resize(data.size() + 32);
        Put32(data, 1);
        Put16(data, 0);
        return data;
    }

    std::vector<ResolvedRecord> Read(const std::filesystem::path &path)
    {
        std::vector<ResolvedRecord> records;
        std::vector<std::string> masters;
        std::vector<Diagnostic> diagnostics;
        bool light{};
        assert(DirectPluginReader{}.Read(path, records, masters, light, diagnostics));
        assert(diagnostics.empty());
        return records;
    }

    const PluginRecord &Navigation(const std::vector<ResolvedRecord> &records)
    {
        const auto record =
            std::find_if(records.begin(), records.end(), [](const auto &item) { return item.type == "NAVI"; });
        assert(record != records.end() && record->raw && record->winning.formId == 0x12FB4);
        assert(std::count_if(records.begin(), records.end(), [](const auto &item) { return item.type == "NAVI"; }) ==
               1);
        return *record->raw;
    }

    const Subrecord &Info(const PluginRecord &navigation, std::uint32_t id)
    {
        const auto entry = std::find_if(navigation.subrecords.begin(), navigation.subrecords.end(), [&](const auto &sub)
                                        { return sub.type == "NVMI" && Get32(sub.data, 0) == id; });
        assert(entry != navigation.subrecords.end());
        return *entry;
    }

    void Write(const std::filesystem::path &path, const Bytes &data)
    {
        std::ofstream file(path, std::ios::binary);
        file.write(reinterpret_cast<const char *>(data.data()), static_cast<std::streamsize>(data.size()));
        assert(file.good());
    }

    void TestInfoEntries(const std::filesystem::path &root)
    {
        Bytes payload;
        std::string error;
        assert(detail::BuildNavmeshInfo({{0x01000900, Geometry(0x01000100, true)}}, nullptr, payload, error));
        Bytes plugin;
        Record(plugin, "TES4", 0, {});
        Record(plugin, "NAVI", 0x12FB4, payload);
        Write(root / "entries.esp", plugin);
        const auto records = Read(root / "entries.esp");
        const auto &entry = Info(Navigation(records), 0x01000900).data;
        assert(entry.size() == 65 && Get32(entry, 4) == 0);
        assert(GetFloat(entry, 8) == 4 && GetFloat(entry, 12) == 4 && GetFloat(entry, 16) == 6);
        assert(GetFloat(entry, 20) == 1);
        assert(Get32(entry, 24) == 1 && Get32(entry, 28) == 0x01000901);
        assert(Get32(entry, 32) == 1 && Get32(entry, 36) == 0x01000901);
        assert(Get32(entry, 40) == 1 && Get32(entry, 44) == 0xE48B73F3 && Get32(entry, 48) == 0x01000200);
        assert(entry[52] == 0 && Get32(entry, 53) == 0xA5E9A03C && Get32(entry, 61) == 0x01000100);
        auto invalid = Geometry(0x100, true);
        invalid.resize(25);
        assert(!detail::BuildNavmeshInfo({{0x900, invalid}}, nullptr, payload, error));
        invalid = Geometry(0x100, true);
        invalid[66] = 3;
        assert(!detail::BuildNavmeshInfo({{0x900, invalid}}, nullptr, payload, error));

        auto exterior = Geometry(0x0004FFFE);
        exterior.insert(exterior.begin() + 56, 7000 * 12, 0);
        const std::uint32_t count = 7003, world = 0x01000400;
        std::memcpy(exterior.data() + 16, &count, 4);
        std::memcpy(exterior.data() + 8, &world, 4);
        assert(detail::BuildNavmeshInfo({{0x01000900, exterior}}, nullptr, payload, error));
        plugin.clear();
        Record(plugin, "TES4", 0, {});
        Record(plugin, "NAVI", 0x12FB4, payload);
        Write(root / "large.esp", plugin);
        const auto large = Read(root / "large.esp");
        const auto &island = Info(Navigation(large), 0x01000900);
        assert(island.extendedSize && island.data.size() > 65535);
        assert(Get32(island.data, 4) == 0x20 && island.data[36] == 1);
        assert(Get32(island.data, 61) == 1 && Get32(island.data, 71) == count);
        assert(Get32(island.data, island.data.size() - 12) == 0xA5E9A03C);
        assert(Get32(island.data, island.data.size() - 8) == world);
        assert(Get32(island.data, island.data.size() - 4) == 0x0004FFFE);
        auto unsupported = Navigation(large);
        unsupported.subrecords.front().data = Bytes(4);
        assert(!detail::BuildNavmeshInfo({}, &unsupported, payload, error));
    }

    void TestSourceCopy(const std::filesystem::path &root)
    {
        // The extra full plugin shifts global FormIDs relative to the output master table.
        Bytes padding;
        Record(padding, "TES4", 0, {});
        Write(root / "Padding.esm", padding);
        Bytes version;
        Put32(version, 12);
        Bytes baseInfo;
        Sub(baseInfo, "NVER", version);
        Sub(baseInfo, "NVPP", Bytes(8));
        Bytes base;
        Record(base, "TES4", 0, {});
        Record(base, "NAVI", 0x12FB4, baseInfo);
        Write(root / "Base.esm", base);

        Bytes sourceInfo;
        std::string error;
        const auto geometry = Geometry(0x01000100);
        assert(detail::BuildNavmeshInfo(
            {{0x01000900, geometry}, {0x01000901, Geometry(0x01000101)}, {0x01000902, geometry}}, nullptr, sourceInfo,
            error));
        Bytes removed;
        Put32(removed, 0x01000900);
        Put32(removed, 0x01000903);
        Sub(sourceInfo, "NVSI", removed);
        uLongf compressedSize = compressBound(static_cast<uLong>(sourceInfo.size()));
        Bytes compressed(4 + compressedSize);
        const auto sourceSize = static_cast<std::uint32_t>(sourceInfo.size());
        std::memcpy(compressed.data(), &sourceSize, 4);
        assert(compress(compressed.data() + 4, &compressedSize, sourceInfo.data(), sourceSize) == Z_OK);
        compressed.resize(4 + compressedSize);

        Bytes header, hedr;
        Put32(hedr, 0x3FD9999A);
        Put32(hedr, 0);
        Put32(hedr, 0xA00);
        Sub(header, "HEDR", hedr);
        Sub(header, "MAST", {'B', 'a', 's', 'e', '.', 'e', 's', 'm', 0});
        Bytes cells;
        for (std::uint32_t index{}; index < 3; ++index)
        {
            Record(cells, "CELL", 0x01000100 + index, {});
            if (index == 2)
            {
                continue;
            }
            Bytes mesh, navm;
            Sub(mesh, "NVNM", Geometry(0x01000100 + index));
            Record(navm, "NAVM", 0x01000900 + index, mesh);
            if (index == 0)
            {
                Record(navm, "NAVM", 0x01000902, mesh);
            }
            Bytes temporary;
            Group(temporary, 0x01000100 + index, 9, navm);
            Group(cells, 0x01000100 + index, 6, temporary);
        }
        Bytes source;
        Record(source, "TES4", 0, header);
        Group(source, 0x4C4C4543, 0, cells);
        Bytes navi;
        Record(navi, "NAVI", 0x12FB4, compressed, 0x40000);
        Group(source, 0x4956414E, 0, navi);
        const auto sourcePath = root / "Source.esp";
        Write(sourcePath, source);
        const std::vector<std::filesystem::path> inputs = {root / "Padding.esm", root / "Base.esm", sourcePath};
        const auto resolved = ResolveLoadOrder({.dataDirectory = root, .plugins = inputs});
        assert(resolved.diagnostics.empty() && resolved.cells.size() == 3);
        navmesh::core::CandidateNavMesh candidate;
        candidate.mesh.vertices = {{0, 0, 6}, {24, 0, 6}, {0, 24, 6}};
        navmesh::core::NavPolygon triangle;
        triangle.vertices = {0, 1, 2};
        triangle.neighbors.fill(std::numeric_limits<std::uint32_t>::max());
        candidate.mesh.polygons.push_back(triangle);
        const std::vector<NavmeshReplacement> replacements = {{&resolved.cells[0], &candidate},
                                                              {&resolved.cells[2], &candidate}};
        std::filesystem::path target;
        assert(WriteNavmeshOverrides(root / "patch", inputs, resolved, replacements, target, error));
        const auto patch = Read(target);
        const auto &patchInfo = Navigation(patch);
        assert(Info(patchInfo, 0x01000900).data.size() == 123);
        assert(Get32(Info(patchInfo, 0x01000900).data, 4) == 0x20);
        assert(GetFloat(Info(patchInfo, 0x01000900).data, 8) == 8);
        assert(Info(patchInfo, 0x01000902).data.size() == 49);
        assert(Get32(Info(patchInfo, 0x01000902).data, 4) == 0);
        assert(Info(patchInfo, 0x02000800).data.size() == 123);

        assert(WriteNavmeshOverrides(root / "copy", inputs, resolved, replacements, target, error, "Source.esp"));
        const auto copy = Read(target);
        const auto &copyInfo = Navigation(copy);
        assert(Info(copyInfo, 0x01000900).data == Info(patchInfo, 0x01000900).data);
        assert(Info(copyInfo, 0x01000A00).data.size() == 123);
        const auto originalRecords = Read(sourcePath);
        assert(Info(copyInfo, 0x01000901).data == Info(Navigation(originalRecords), 0x01000901).data);
        const auto deleted = std::find_if(copyInfo.subrecords.begin(), copyInfo.subrecords.end(),
                                          [](const auto &sub) { return sub.type == "NVSI"; });
        assert(deleted != copyInfo.subrecords.end() && deleted->data.size() == 4 &&
               Get32(deleted->data, 0) == 0x01000903);
        const auto cache = std::find_if(copyInfo.subrecords.begin(), copyInfo.subrecords.end(),
                                        [](const auto &sub) { return sub.type == "NVPP"; });
        assert(cache != copyInfo.subrecords.end() && cache->data == Bytes(8));
        std::ifstream unchanged(sourcePath, std::ios::binary);
        assert(Bytes(std::istreambuf_iterator<char>(unchanged), {}) == source);
    }
} // namespace

void TestNavmeshInfoExport()
{
    const auto root = std::filesystem::temp_directory_path() / "navmesh-info-export-test";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    TestInfoEntries(root);
    TestSourceCopy(root);
}
