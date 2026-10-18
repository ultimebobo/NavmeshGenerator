#ifdef NDEBUG
#undef NDEBUG
#endif

#include "skyrim/parser/navmesh_info.h"
#include "skyrim/parser/plugin_writer.h"
#include "skyrim/parser/navmesh_metadata.h"
#include "skyrim/parser/reference_navigation.h"
#include "skyrim/parser/navigation_validation.h"

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

    void TestAuxiliaryMetadata()
    {
        PluginRecord source;
        Bytes objects, preferred, excluded;
        Put32(objects, 0x600);
        Put16(preferred, 0);
        Put16(excluded, 1);
        source.subrecords = {
            {.type = "ONAM", .data = objects}, {.type = "PNAM", .data = preferred}, {.type = "NNAM", .data = excluded}};
        navmesh::core::NavMesh authored, generated;
        authored.vertices = {{0, 0, 0}, {10, 0, 0}, {0, 10, 0}};
        generated.vertices = {authored.vertices[1], authored.vertices[2], authored.vertices[0]};
        const auto rebase = [](std::uint32_t id) -> std::optional<std::uint32_t> { return id | 0x01000000; };
        std::array sources{detail::NavmeshMetadataSource{&source, &authored, rebase}};
        Bytes output;
        std::string error;
        assert(detail::RebuildNavmeshMetadata(sources, generated, 0, output, error));
        assert(output.size() == 26 && Get32(output, 6) == 0x01000600);
        assert(output[16] == 2 && output[17] == 0 && output[24] == 0 && output[25] == 0);
        source.subrecords.back().data = preferred;
        assert(!detail::RebuildNavmeshMetadata(sources, generated, 0, output, error));
        assert(error.find("same generated vertex") != std::string::npos);
        source.subrecords.back().data = Bytes{4, 0};
        assert(!detail::RebuildNavmeshMetadata(sources, generated, 0, output, error));
    }

    void TestReferenceSafety(const std::filesystem::path &root)
    {
        Bytes reference, name, vmad;
        Put32(name, 0x600);
        Sub(reference, "NAME", name);
        Put16(vmad, 5);
        Put16(vmad, 1);
        Put16(vmad, 1);
        Put16(vmad, 1);
        vmad.push_back('S');
        vmad.push_back(0);
        Put16(vmad, 2);
        Put16(vmad, 1);
        vmad.push_back('O');
        vmad.push_back(11);
        vmad.push_back(1);
        Put32(vmad, 2);
        Put32(vmad, 0x00000200);
        Put32(vmad, 0x0000FFFF);
        Put32(vmad, 0x01000201);
        Put32(vmad, 0x0000FFFF);
        Put16(vmad, 1);
        vmad.push_back('B');
        vmad.push_back(15);
        vmad.push_back(1);
        Put32(vmad, 70000);
        vmad.insert(vmad.end(), 70000, 1);
        reference.insert(reference.end(), {'X', 'X', 'X', 'X', 4, 0});
        Put32(reference, static_cast<std::uint32_t>(vmad.size()));
        reference.insert(reference.end(), {'V', 'M', 'A', 'D', 0, 0});
        reference.insert(reference.end(), vmad.begin(), vmad.end());
        Bytes topic;
        Put32(topic, 0);
        Put32(topic, 0x01000800);
        Sub(reference, "PDTO", topic);
        Bytes subtype;
        Put32(subtype, 1);
        subtype.insert(subtype.end(), {'H', 'E', 'L', 'O'});
        Sub(reference, "PDTO", subtype);
        Sub(reference, "DATA", Bytes(24));
        Bytes source;
        Record(source, "TES4", 0, {});
        Record(source, "REFR", 0x200, reference);
        Write(root / "reference-source.esp", source);
        const auto original = Read(root / "reference-source.esp");
        assert(original.size() == 1 && original[0].raw);
        const auto &raw = *original[0].raw;
        assert(raw.subrecords[1].extendedSize && raw.subrecords[1].encodedBytes.size() == vmad.size() + 16);
        const auto rebase = [](std::uint32_t id) -> std::optional<std::uint32_t> { return id + 0x01000000; };
        Bytes rewritten;
        std::string error;
        assert(detail::RewriteReferenceNavigation(raw, rebase, NavmeshDoorAssociation{0x02000900, 7}, false, false,
                                                  rewritten, error));
        Bytes output;
        Record(output, "TES4", 0, {});
        Record(output, "REFR", 0x01000200, rewritten);
        Write(root / "reference-output.esp", output);
        const auto copied = Read(root / "reference-output.esp");
        assert(copied[0].navmeshDoor && copied[0].navmeshDoor->navmeshId == 0x02000900 &&
               copied[0].navmeshDoor->triangle == 7);
        const auto &script = copied[0].raw->subrecords[1];
        assert(script.extendedSize && script.data.size() == vmad.size());
        assert(Get32(script.data, 21) == 0x01000200 && Get32(script.data, 29) == 0x02000201);
        assert(script.data[script.data.size() - 1] == 1);
        assert(Get32(copied[0].raw->subrecords[2].data, 4) == 0x02000800);
        assert(copied[0].raw->subrecords[3].data == subtype);
        const auto &copiedFields = copied[0].raw->subrecords;
        assert(copiedFields[copiedFields.size() - 2].type == "XNDP");
        assert(copiedFields.back().type == "DATA" && copiedFields.back().data == Bytes(24));
        const auto identity = [](std::uint32_t id) -> std::optional<std::uint32_t> { return id; };
        Bytes replaced;
        assert(detail::RewriteReferenceNavigation(*copied[0].raw, identity, NavmeshDoorAssociation{0x02000901, 8},
                                                  false, false, replaced, error));
        Bytes expectedReplacement = rewritten;
        // The association is the only field changed; placement must remain last.
        constexpr std::size_t placementBytes = 6 + 24;
        constexpr std::size_t associationBytes = 6 + 8;
        const auto associationAt = expectedReplacement.size() - placementBytes - 8;
        expectedReplacement[associationAt] = 1;
        expectedReplacement[associationAt + 4] = 8;
        assert(replaced == expectedReplacement);
        Bytes cleared;
        assert(
            detail::RewriteReferenceNavigation(*copied[0].raw, identity, std::nullopt, false, false, cleared, error));
        Bytes expectedCleared = rewritten;
        expectedCleared.erase(expectedCleared.end() - placementBytes - associationBytes,
                              expectedCleared.end() - placementBytes);
        assert(cleared == expectedCleared);
        auto unknown = raw;
        unknown.subrecords.push_back({.type = "ZZZZ", .data = Bytes{1}});
        assert(!detail::RewriteReferenceNavigation(unknown, rebase, std::nullopt, false, false, rewritten, error));
        assert(error.find("ZZZZ") != std::string::npos);
        auto localized = raw;
        localized.subrecords.push_back({.type = "FULL", .data = Bytes(4)});
        assert(!detail::RewriteReferenceNavigation(localized, rebase, std::nullopt, false, true, rewritten, error));
    }

    void TestReplacedLedge(const std::filesystem::path &root)
    {
        std::filesystem::create_directories(root);
        Bytes geometry;
        for (const auto value : {12U, 0xA5E9A03CU, 0U, 0x100U, 6U})
        {
            Put32(geometry, value);
        }
        navmesh::core::NavMesh authored;
        authored.vertices = {{0, 0, 0}, {100, 0, 0}, {0, 100, 0}, {100, 0, 40}, {100, 100, 40}, {0, 100, 40}};
        const auto *points = reinterpret_cast<const std::uint8_t *>(authored.vertices.data());
        geometry.insert(geometry.end(), points, points + authored.vertices.size() * 12);
        Put32(geometry, 2);
        for (const auto value : {0U, 1U, 2U, 65535U, 0U, 65535U, 2U, 0U, 3U, 4U, 5U, 65535U, 65535U, 1U, 4U, 0U})
        {
            Put16(geometry, static_cast<std::uint16_t>(value));
        }
        Put32(geometry, 2);
        for (const auto type : {1U, 2U})
        {
            Put32(geometry, type);
            Put32(geometry, 0x200);
            Put16(geometry, static_cast<std::uint16_t>(2 - type));
        }
        Put32(geometry, 0);
        Put32(geometry, 0);
        Put32(geometry, 1);
        geometry.resize(geometry.size() + 32);
        Put32(geometry, 2);
        Put16(geometry, 0);
        Put16(geometry, 1);
        Bytes navm, child, children, cells, plugin;
        Sub(navm, "NVNM", geometry);
        Record(child, "NAVM", 0x200, navm);
        Group(children, 0x100, 9, child);
        Record(cells, "CELL", 0x100, {});
        Group(cells, 0x100, 6, children);
        Record(plugin, "TES4", 0, {});
        Group(plugin, 0x4C4C4543, 0, cells);
        const auto source = root / "Traversal.esm";
        Write(source, plugin);
        const auto resolved = ResolveLoadOrder({.dataDirectory = root, .plugins = {source}});
        assert(resolved.diagnostics.empty() && resolved.cells.size() == 1);
        navmesh::core::CandidateNavMesh candidate;
        candidate.mesh.vertices = {{0, 0, 0}, {100, 0, 0}, {100, 100, 40}, {0, 100, 40}};
        candidate.mesh.polygons = {{.vertices = {0, 1, 3}, .neighbors = {0xffffffffU, 1, 0xffffffffU}},
                                   {.vertices = {1, 2, 3}, .neighbors = {0xffffffffU, 0xffffffffU, 0}}};
        std::filesystem::path output;
        std::string error;
        assert(WriteNavmeshOverride(root / "connected", {source}, resolved, resolved.cells.front(), candidate, output,
                                    error));
        const auto written = Read(output);
        const auto mesh = std::find_if(written.begin(), written.end(), [](const auto &r) { return r.type == "NAVM"; });
        assert(mesh != written.end() && mesh->navm->triangleCount == 2);
        assert(Get32(mesh->raw->decodedPayload, mesh->navm->trailingData.offset) == 0);
        candidate.mesh.vertices = {{0, 0, 0}, {100, 0, 0}, {0, 100, 0}, {100, 0, 20}, {100, 100, 20}, {0, 100, 20}};
        candidate.mesh.polygons = {{.vertices = {0, 1, 2}, .neighbors = {0xffffffffU, 1, 0xffffffffU}},
                                   {.vertices = {3, 4, 5}, .neighbors = {0xffffffffU, 0xffffffffU, 0}}};
        assert(
            WriteNavmeshOverride(root / "step", {source}, resolved, resolved.cells.front(), candidate, output, error));
        candidate.profile.stepHeight = 10;
        assert(!WriteNavmeshOverride(root / "excessive-step", {source}, resolved, resolved.cells.front(), candidate,
                                     output, error));
        assert(error.find("adjacency") != std::string::npos);
        assert(!std::filesystem::exists(root / "excessive-step" / "generated-navmesh.esp"));
        candidate.profile = {};
        candidate.mesh = resolved.cells.front().navMeshes.front();
        for (auto &face : candidate.mesh.polygons)
        {
            face.flags &= static_cast<std::uint16_t>(~7U);
            face.neighbors.fill(0xffffffffU);
        }
        assert(!WriteNavmeshOverride(root / "disconnected", {source}, resolved, resolved.cells.front(), candidate,
                                     output, error));
        assert(error.find("ledge") != std::string::npos);
        assert(!std::filesystem::exists(root / "disconnected" / "generated-navmesh.esp"));
    }

    void TestUnsupportedNavigationClosure(const std::filesystem::path &root)
    {
        std::filesystem::create_directories(root);
        for (const unsigned variant : {0U, 1U, 2U})
        {
            const bool sameCell = variant != 1;
            Bytes navm, children, cells, plugin;
            Sub(navm, "NVNM", Geometry(0x100));
            Record(children, "NAVM", 0x200, navm);
            if (sameCell)
            {
                auto unsupported = Geometry(0x100);
                unsupported[0] = 13;
                navm.clear();
                if (variant != 2)
                {
                    Sub(navm, "NVNM", unsupported);
                }
                Record(children, "NAVM", 0x201, navm);
            }
            Bytes temporary;
            Group(temporary, 0x100, 9, children);
            Record(cells, "CELL", 0x100, {});
            Group(cells, 0x100, 6, temporary);
            if (!sameCell)
            {
                // A different layout version can retain incoming triangle references.
                auto unsupported = Geometry(0x101);
                unsupported[0] = 13;
                unsupported[66] = 0;
                unsupported[67] = 0;
                unsupported[72] = 1;
                unsupported[76] = 1;
                Bytes incoming;
                Put32(incoming, 0);
                Put32(incoming, 0x200);
                Put16(incoming, 0);
                unsupported.insert(unsupported.begin() + 80, incoming.begin(), incoming.end());
                navm.clear();
                Sub(navm, "NVNM", unsupported);
                children.clear();
                Record(children, "NAVM", 0x201, navm);
                temporary.clear();
                Group(temporary, 0x101, 9, children);
                Record(cells, "CELL", 0x101, {});
                Group(cells, 0x101, 6, temporary);
            }
            Record(plugin, "TES4", 0, {});
            Group(plugin, 0x4C4C4543, 0, cells);
            const auto source = root / (variant == 0   ? "Mixed.esm"
                                        : variant == 1 ? "UnknownNeighbor.esm"
                                                       : "Missing.esm");
            Write(source, plugin);
            const auto resolved = ResolveLoadOrder({.dataDirectory = root, .plugins = {source}});
            assert(resolved.cells.front().navMeshes.size() == 1);
            assert(variant == 2 ? resolved.diagnostics.empty()
                                : resolved.diagnostics.size() == 1 &&
                                      resolved.diagnostics.front().kind == DiagnosticKind::UnsupportedVersion);
            navmesh::core::CandidateNavMesh candidate;
            candidate.mesh = resolved.cells.front().navMeshes.front();
            std::filesystem::path output;
            std::string error;
            for (const bool copy : {false, true})
            {
                const auto directory = root / (source.stem().string() + (copy ? "-copy" : "-patch"));
                assert(!WriteNavmeshOverrides(directory, {source}, resolved, {{&resolved.cells.front(), &candidate}},
                                              output, error, copy ? source.filename().string() : ""));
                assert(!error.empty());
                assert(!std::filesystem::exists(directory / (copy ? source.filename() : "generated-navmesh.esp")));
            }
        }
    }

    void TestSourceCopy(const std::filesystem::path &root, bool light = false)
    {
        std::filesystem::create_directories(root);
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
        Record(base, "DOOR", 0x600, {});
        Record(base, "KEYM", 0x700, {});
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
            if (index == 0)
            {
                Bytes reference, baseId, association;
                Put32(baseId, 0x600);
                Sub(reference, "NAME", baseId);
                Put32(association, 0x01000902);
                Put32(association, 0);
                Sub(reference, "XNDP", association);
                Record(navm, "REFR", 0x01000200, reference);
            }
            Group(temporary, 0x01000100 + index, 9, navm);
            Group(cells, 0x01000100 + index, 6, temporary);
        }
        Bytes source;
        Record(source, "TES4", 0, header, light ? 0x200U : 0U);
        Group(source, 0x4C4C4543, 0, cells);
        Bytes navi;
        Record(navi, "NAVI", 0x12FB4, compressed, 0x40000);
        Group(source, 0x4956414E, 0, navi);
        const auto sourcePath = root / "Source.esp";
        Write(sourcePath, source);
        // A later winner uses a different master order. Its non-navigation data must
        // survive both patch rebasing and replacement of the earlier source file.
        Bytes winnerHeader, winnerPayload, baseId, lock, enableParent, scripts;
        Sub(winnerHeader, "MAST", {'S', 'o', 'u', 'r', 'c', 'e', '.', 'e', 's', 'p', 0});
        Sub(winnerHeader, "MAST", {'B', 'a', 's', 'e', '.', 'e', 's', 'm', 0});
        Put32(baseId, 0x01000600);
        Sub(winnerPayload, "NAME", baseId);
        Sub(winnerPayload, "EDID", {'W', 'i', 'n', 'n', 'i', 'n', 'g', 'D', 'o', 'o', 'r', 0});
        lock.resize(20);
        const std::uint32_t keyId = 0x01000700;
        std::memcpy(lock.data() + 4, &keyId, 4);
        lock[0] = 37;
        lock[8] = 4;
        Sub(winnerPayload, "XLOC", lock);
        Put32(enableParent, 0x00000200);
        Put32(enableParent, 1);
        Sub(winnerPayload, "XESP", enableParent);
        Put16(scripts, 5);
        Put16(scripts, 2);
        Put16(scripts, 1);
        Put16(scripts, 1);
        scripts.push_back('S');
        scripts.push_back(0);
        Put16(scripts, 1);
        Put16(scripts, 1);
        scripts.push_back('P');
        scripts.push_back(1);
        scripts.push_back(1);
        Put32(scripts, 0xFFFF0000);
        Put32(scripts, 0x00000200);
        Sub(winnerPayload, "VMAD", scripts);
        Bytes winnerRecord, winnerTemporary, winnerCell, winner;
        Record(winnerRecord, "REFR", 0x00000200, winnerPayload, 1U << 11);
        Group(winnerTemporary, 0x00000100, 9, winnerRecord);
        Group(winnerCell, 0x00000100, 6, winnerTemporary);
        Record(winner, "TES4", 0, winnerHeader);
        Group(winner, 0x4C4C4543, 0, winnerCell);
        const auto winnerPath = root / "Winner.esp";
        Write(winnerPath, winner);
        const std::vector<std::filesystem::path> inputs = {root / "Padding.esm", root / "Base.esm", sourcePath,
                                                           winnerPath};
        const auto resolved = ResolveLoadOrder({.dataDirectory = root, .plugins = inputs});
        assert(resolved.diagnostics.empty() && resolved.cells.size() == 3);
        navmesh::core::CandidateNavMesh candidate;
        candidate.mesh.vertices = {{0, 0, 6}, {24, 0, 6}, {0, 24, 6}};
        navmesh::core::NavPolygon triangle;
        triangle.vertices = {0, 1, 2};
        triangle.neighbors.fill(std::numeric_limits<std::uint32_t>::max());
        candidate.mesh.polygons.push_back(triangle);
        auto doorCandidate = candidate;
        const auto winningDoor = std::find_if(resolved.records.begin(), resolved.records.end(),
                                              [](const auto &record) { return record.editorId == "WinningDoor"; });
        assert(winningDoor != resolved.records.end());
        doorCandidate.exits.push_back(
            {.referenceId = winningDoor->formId, .position = {8, 8, 6}, .region = 0, .polygon = 0});
        doorCandidate.coverGeometry.vertices = {{0, -4, 6}, {24, -4, 6}, {24, -4, 134}, {0, -4, 134}};
        doorCandidate.coverGeometry.triangles = {{{0, 1, 2}}, {{0, 2, 3}}};
        const std::vector<NavmeshReplacement> replacements = {{&resolved.cells[0], &doorCandidate},
                                                              {&resolved.cells[2], &candidate}};
        std::filesystem::path target;
        assert(WriteNavmeshOverrides(root / "patch", inputs, resolved, replacements, target, error));
        const auto patch = Read(target);
        const auto &patchInfo = Navigation(patch);
        assert(Info(patchInfo, 0x01000900).data.size() == 57);
        assert(Get32(Info(patchInfo, 0x01000900).data, 4) == 0);
        assert(GetFloat(Info(patchInfo, 0x01000900).data, 8) == 8);
        assert(Info(patchInfo, 0x01000902).data.size() == 49);
        assert(Get32(Info(patchInfo, 0x01000902).data, 4) == 0);
        assert(Info(patchInfo, 0x03000800).data.size() == 123);
        const auto reference =
            std::find_if(patch.begin(), patch.end(), [](const auto &record) { return record.type == "REFR"; });
        assert(reference != patch.end() && reference->navmeshDoor && reference->raw);
        assert(reference->winning.formId == 0x01000200 && reference->navmeshDoor->navmeshId == 0x01000900);
        assert(reference->navmeshDoor->triangle == 0 && reference->editorId == "WinningDoor");
        assert(reference->raw->flags & (1U << 11));
        for (const auto &sub : reference->raw->subrecords)
        {
            if (sub.type == "XLOC")
            {
                assert(Get32(sub.data, 4) == 0x700 && sub.data[0] == 37 && sub.data[8] == 4);
            }
            if (sub.type == "XESP")
            {
                assert(Get32(sub.data, 0) == 0x01000200 && Get32(sub.data, 4) == 1);
            }
            if (sub.type == "VMAD")
            {
                assert(Get32(sub.data, sub.data.size() - 4) == 0x01000200);
            }
        }
        const auto generated = std::find_if(patch.begin(), patch.end(),
                                            [](const auto &record) { return record.winning.formId == 0x01000900; });
        assert(generated != patch.end() && generated->navm && generated->raw);
        const auto triangleAt = static_cast<std::size_t>(generated->navm->triangles.offset);
        assert(Get32(generated->raw->decodedPayload, triangleAt + 12) == (0x4008U << 16 | 0x400U));
        auto broken = patch;
        const auto brokenReference =
            std::find_if(broken.begin(), broken.end(), [](const auto &record) { return record.type == "REFR"; });
        brokenReference->navmeshDoor->triangle = 1;
        assert(!detail::ValidateNavigationGraph(broken, {"Base.esm", "Source.esp", "Winner.esp"},
                                                "generated-navmesh.esp", resolved, {0x01000900, 0x01000902, 0x03000800},
                                                {0x01000200}, error));
        assert(error.find("door") != std::string::npos);

        assert(WriteNavmeshOverrides(root / "copy", inputs, resolved, replacements, target, error, "Source.esp"));
        const auto copy = Read(target);
        const auto &copyInfo = Navigation(copy);
        const auto copiedReference =
            std::find_if(copy.begin(), copy.end(), [](const auto &record) { return record.type == "REFR"; });
        assert(copiedReference != copy.end() && copiedReference->raw->decodedPayload == reference->raw->decodedPayload);
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
        unchanged.close();
        auto changed = source;
        changed.push_back(0);
        Write(sourcePath, changed);
        assert(!WriteNavmeshOverrides(root / "stale", inputs, resolved, replacements, target, error));
        assert(error.find("changed") != std::string::npos);
        assert(!std::filesystem::exists(root / "stale" / "generated-navmesh.esp"));
    }
} // namespace

void TestNavmeshInfoExport()
{
    const auto root = std::filesystem::temp_directory_path() / "navmesh-info-export-test";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    TestInfoEntries(root);
    TestAuxiliaryMetadata();
    TestReferenceSafety(root);
    TestReplacedLedge(root / "traversal");
    TestUnsupportedNavigationClosure(root / "unsupported");
    TestSourceCopy(root);
    TestSourceCopy(root / "light-source", true);
}
