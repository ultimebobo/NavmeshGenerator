#include "analysis/navmesh_analysis.h"
#include "core/geometry/types.h"
#include "core/reproducibility/export_metadata.h"
#include "core/scene/scene.h"
#include "skyrim/parser/plugin_parser.h"
#include "skyrim/mo2/mo2_importer.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/extraction/terrain_extractor.h"
#include "core/scene/scene_exporter.h"
#include "validation/validation.h"

#include <array>

#include <NifFile.hpp>
#include <bhk.hpp>

#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <source_location>
#include <vector>
#include <tuple>

#include <zlib.h>

namespace
{
    void Require(const bool condition, const std::source_location location = std::source_location::current()) { if (!condition) { std::fprintf(stderr, "Requirement failed at %s:%u\n", location.file_name(), location.line()); std::abort(); } }
    void PutU16(std::vector<std::uint8_t>& bytes, std::uint16_t value) { bytes.push_back(static_cast<std::uint8_t>(value)); bytes.push_back(static_cast<std::uint8_t>(value >> 8)); }
    void PutU32(std::vector<std::uint8_t>& bytes, std::uint32_t value) { for (auto shift = 0; shift < 32; shift += 8) bytes.push_back(static_cast<std::uint8_t>(value >> shift)); }
    void PutFloat(std::vector<std::uint8_t>& bytes, float value) { const auto* raw = reinterpret_cast<const std::uint8_t*>(&value); bytes.insert(bytes.end(), raw, raw + sizeof(value)); }
    void PutText(std::vector<std::uint8_t>& bytes, const char* type, const std::vector<std::uint8_t>& payload) { bytes.insert(bytes.end(), type, type + 4); PutU16(bytes, static_cast<std::uint16_t>(payload.size())); bytes.insert(bytes.end(), payload.begin(), payload.end()); }
    void PutRecord(std::vector<std::uint8_t>& bytes, const char* type, std::uint32_t formId, const std::vector<std::uint8_t>& payload, std::uint32_t flags = 0) { bytes.insert(bytes.end(), type, type + 4); PutU32(bytes, static_cast<std::uint32_t>(payload.size())); PutU32(bytes, flags); PutU32(bytes, formId); bytes.insert(bytes.end(), 8, 0); bytes.insert(bytes.end(), payload.begin(), payload.end()); }
    void PutGroup(std::vector<std::uint8_t>& bytes, std::uint32_t label, std::uint32_t groupType, const std::vector<std::uint8_t>& payload) { bytes.insert(bytes.end(), { 'G', 'R', 'U', 'P' }); PutU32(bytes, static_cast<std::uint32_t>(24 + payload.size())); PutU32(bytes, label); PutU32(bytes, groupType); bytes.insert(bytes.end(), 8, 0); bytes.insert(bytes.end(), payload.begin(), payload.end()); }
    void WritePlugin(const std::filesystem::path& path, const std::vector<std::string>& masters, bool light, const std::vector<std::tuple<std::string, std::uint32_t, std::vector<std::uint8_t>>>& records)
    {
        std::vector<std::uint8_t> header; for (const auto& master : masters) { std::vector<std::uint8_t> value(master.begin(), master.end()); value.push_back(0); PutText(header, "MAST", value); }
        std::vector<std::uint8_t> bytes; bytes.insert(bytes.end(), { 'T', 'E', 'S', '4' }); PutU32(bytes, static_cast<std::uint32_t>(header.size())); PutU32(bytes, light ? 0x200U : 0U); bytes.insert(bytes.end(), 12, 0); bytes.insert(bytes.end(), header.begin(), header.end());
        for (const auto& [type, id, payload] : records) PutRecord(bytes, type.c_str(), id, payload);
        std::ofstream output(path, std::ios::binary); output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    }
    std::vector<std::uint8_t> CellPayload(const char* editorId, bool exterior = false, std::uint32_t exteriorFlags = 0)
    {
        std::vector<std::uint8_t> payload; std::vector<std::uint8_t> edid(editorId, editorId + std::strlen(editorId)); edid.push_back(0); PutText(payload, "EDID", edid);
        if (exterior) { std::vector<std::uint8_t> xclc; PutU32(xclc, 12); PutU32(xclc, static_cast<std::uint32_t>(-4)); PutU32(xclc, exteriorFlags); PutText(payload, "XCLC", xclc); } return payload;
    }
    std::vector<std::uint8_t> NavmPayload(std::uint32_t version = 12)
    {
        std::vector<std::uint8_t> body; PutU32(body, version); body.insert(body.end(), 12, 0); PutU32(body, 1); // NVNM header is 0x14 bytes.
        const float point[] = { 1.0F, 2.0F, 3.0F }; const auto* raw = reinterpret_cast<const std::uint8_t*>(point); body.insert(body.end(), raw, raw + sizeof(point));
        PutU32(body, 1); PutU16(body, 0); PutU16(body, 0); PutU16(body, 0); PutU16(body, 0xFFFF); PutU16(body, 0xFFFF); PutU16(body, 0xFFFF); PutU16(body, 0); PutU16(body, 0);
        std::vector<std::uint8_t> payload; PutText(payload, "NVNM", body); PutText(payload, "ZZZZ", { 0xA1, 0xB2, 0xC3 }); return payload;
    }
    std::vector<std::uint8_t> LandPayload(float base, std::int8_t northWestDelta = 0, std::int8_t eastDelta = 0, std::int8_t southDelta = 0)
    {
        std::vector<std::uint8_t> vhgt; PutFloat(vhgt, base); vhgt.resize(4 + 33 * 33, 0); vhgt[4] = static_cast<std::uint8_t>(northWestDelta); vhgt[5] = static_cast<std::uint8_t>(eastDelta); vhgt[4 + 33] = static_cast<std::uint8_t>(southDelta);
        std::vector<std::uint8_t> payload; PutText(payload, "VHGT", vhgt); return payload;
    }
    std::vector<std::uint8_t> Compressed(const std::vector<std::uint8_t>& input)
    {
        uLongf outputSize = compressBound(static_cast<uLong>(input.size())); std::vector<std::uint8_t> output(4 + outputSize); const auto originalSize = static_cast<std::uint32_t>(input.size());
        for (int shift = 0; shift < 32; shift += 8) output[shift / 8] = static_cast<std::uint8_t>(originalSize >> shift);
        Require(compress(output.data() + 4, &outputSize, input.data(), static_cast<uLong>(input.size())) == Z_OK); output.resize(4 + outputSize); return output;
    }
    void TestLossAwareRecordReader()
    {
        const auto root = std::filesystem::temp_directory_path() / "navmesh-loss-aware-reader-test"; std::filesystem::remove_all(root); std::filesystem::create_directories(root);
        const auto cell = CellPayload("CompressedCell", true); std::vector<std::uint8_t> base; PutText(base, "MODL", { 'm', 'e', 's', 'h', 'e', 's', '/', 't', 'e', 's', 't', '.', 'n', 'i', 'f', 0 });
        WritePlugin(root / "Records.esp", {}, false, { { "CELL", 0x100, Compressed(cell) }, { "NAVM", 0x101, NavmPayload() }, { "STAT", 0x102, base } });
        // Mark the first non-TES4 record compressed in-place; this keeps the fixture builder intentionally small.
        std::fstream compressedFile(root / "Records.esp", std::ios::in | std::ios::out | std::ios::binary); compressedFile.seekp(24 + 8); const std::uint32_t compressedFlag = 0x40000; compressedFile.write(reinterpret_cast<const char*>(&compressedFlag), sizeof(compressedFlag)); compressedFile.close();
        const auto parsed = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { "Records.esp" } });
        const auto* compressedCell = parsed.FindWinning(0x100); Require(compressedCell && compressedCell->editorId == "CompressedCell" && compressedCell->raw && compressedCell->raw->compressed);
        const auto* navm = parsed.FindWinning(0x101); Require(navm && navm->navm && navm->navm->supported && navm->navm->vertexCount == 1 && navm->navm->triangleCount == 1);
        const auto unknown = std::find_if(navm->raw->subrecords.begin(), navm->raw->subrecords.end(), [](const auto& sub) { return sub.type == "ZZZZ"; }); Require(unknown != navm->raw->subrecords.end() && unknown->data == std::vector<std::uint8_t>({ 0xA1, 0xB2, 0xC3 }));
        const auto* baseRecord = parsed.FindWinning(0x102); Require(baseRecord && baseRecord->modelPath == "meshes/test.nif" && baseRecord->raw);

        WritePlugin(root / "Bad.esp", {}, false, { { "NAVM", 0x200, { 'N', 'V', 'N', 'M', 0x40, 0x00 } }, { "CELL", 0x201, { 'E', 'D', 'I', 'D', 0x08, 0x00, 'x' } }, { "NAVM", 0x202, NavmPayload(99) } });
        const auto malformed = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { "Bad.esp" } });
        Require(std::any_of(malformed.diagnostics.begin(), malformed.diagnostics.end(), [](const auto& d) { return d.kind == navmesh::skyrim::offline::DiagnosticKind::MalformedInput; }));
        Require(std::any_of(malformed.diagnostics.begin(), malformed.diagnostics.end(), [](const auto& d) { return d.kind == navmesh::skyrim::offline::DiagnosticKind::UnsupportedVersion; }));

        WritePlugin(root / "BadCompression.esp", {}, false, { { "CELL", 0x203, { 0, 0, 0, 0 } } });
        std::fstream badCompressedFile(root / "BadCompression.esp", std::ios::in | std::ios::out | std::ios::binary); badCompressedFile.seekp(24 + 8); badCompressedFile.write(reinterpret_cast<const char*>(&compressedFlag), sizeof(compressedFlag)); badCompressedFile.close();
        const auto badCompression = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { "BadCompression.esp" } });
        Require(std::any_of(badCompression.diagnostics.begin(), badCompression.diagnostics.end(), [](const auto& d) { return d.kind == navmesh::skyrim::offline::DiagnosticKind::DecompressionFailure; }));
    }
    void TestResolvedLoadOrder()
    {
        const auto root = std::filesystem::temp_directory_path() / "navmesh-load-order-test"; std::filesystem::remove_all(root); std::filesystem::create_directories(root);
        WritePlugin(root / "Base.esm", {}, false, { { "CELL", 0x123, CellPayload("BaseCell", true, 0x10) }, { "NAVM", 0x456, {} }, { "LAND", 0x789, LandPayload(42.0F) } });
        WritePlugin(root / "Light.esl", {}, true, { { "CELL", 0x800, CellPayload("LightCell") } });
        WritePlugin(root / "Patch.esp", { "Base.esm", "Light.esl" }, false, { { "CELL", 0x00000123, CellPayload("PatchedCell", true, 0x10) }, { "NAVM", 0x00000456, {} }, { "LAND", 0x00000789, {} }, { "REFR", 0x02000800, {} } });
        const auto resolved = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { "Base.esm", "Light.esl", "Patch.esp" } });
        assert(resolved.diagnostics.empty());
        const auto* cell = resolved.FindWinning(0x123); assert(cell && cell->editorId == "PatchedCell" && cell->origins.size() == 2 && cell->winning.plugin == "Patch.esp");
        const auto* navm = resolved.FindWinning(0x456); assert(navm && navm->origins.size() == 2 && navm->winning.plugin == "Patch.esp");
        const auto* inheritedLand = resolved.FindWinning(0x789); Require(inheritedLand && inheritedLand->winning.plugin == "Patch.esp" && inheritedLand->raw && std::any_of(inheritedLand->raw->subrecords.begin(), inheritedLand->raw->subrecords.end(), [](const auto& sub) { return sub.type == "VHGT"; }));
        Require(resolved.FindWinning(0x01000800) != nullptr); Require(!resolved.cells.front().isInterior && resolved.cells.front().exteriorCoordinates->at(0) == 12);
        WritePlugin(root / "Broken.esp", { "Absent.esm" }, false, {});
        const auto broken = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { "Broken.esp" } });
        assert(std::any_of(broken.diagnostics.begin(), broken.diagnostics.end(), [](const auto& d) { return d.kind == navmesh::skyrim::offline::DiagnosticKind::MissingMaster; }));

        std::vector<std::uint8_t> persistent; PutRecord(persistent, "REFR", 0x401, {}); std::vector<std::uint8_t> temporary; PutRecord(temporary, "NAVM", 0x402, {}); std::vector<std::uint8_t> navmeshGroup; PutRecord(navmeshGroup, "NAVM", 0x403, NavmPayload());
        std::vector<std::uint8_t> world; std::vector<std::uint8_t> indexedCell; PutRecord(indexedCell, "CELL", 0x400, CellPayload("IndexedCell")); PutGroup(indexedCell, 0x400, 8, persistent); PutGroup(indexedCell, 0x400, 9, temporary); PutGroup(indexedCell, 0x400, 10, navmeshGroup); PutGroup(world, 0x300, 1, indexedCell);
        std::vector<std::uint8_t> grouped = { 'T', 'E', 'S', '4' }; PutU32(grouped, 0); PutU32(grouped, 0); grouped.insert(grouped.end(), 12, 0); grouped.insert(grouped.end(), world.begin(), world.end());
        std::ofstream groupedFile(root / "Grouped.esm", std::ios::binary); groupedFile.write(reinterpret_cast<const char*>(grouped.data()), static_cast<std::streamsize>(grouped.size())); groupedFile.close();
        const auto indexed = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { "Grouped.esm" } });
        const auto* persistentRecord = indexed.FindWinning(0x401); const auto* temporaryRecord = indexed.FindWinning(0x402);
        assert(persistentRecord && persistentRecord->cellFormId == 0x400 && persistentRecord->worldspaceFormId == 0x300 && persistentRecord->persistent);
        assert(temporaryRecord && temporaryRecord->cellFormId == 0x400 && temporaryRecord->worldspaceFormId == 0x300 && temporaryRecord->temporary);
        Require(indexed.cells.size() == 1 && indexed.cells.front().navMeshes.size() == 1 && indexed.cells.front().navMeshes.front().polygons.size() == 1);
    }
    void TestWinterholdCellOverride()
    {
        const auto root = std::filesystem::temp_directory_path() / "navmesh-winterhold-override-test";
        std::filesystem::create_directories(root);
        WritePlugin(root / "Skyrim.esm", {}, false, { { "CELL", 0x123, CellPayload("WinterholdExterior01", true) } });

        std::vector<std::uint8_t> header;
        PutText(header, "MAST", { 'S', 'k', 'y', 'r', 'i', 'm', '.', 'e', 's', 'm', 0 });
        std::vector<std::uint8_t> patch;
        PutRecord(patch, "CELL", 0x00000123, CellPayload("WinterholdExterior01", true));
        std::vector<std::uint8_t> model;
        PutText(model, "MODL", { 'm', 'e', 's', 'h', 'e', 's', '/', 'w', 'i', 'n', 't', 'e', 'r', 'h', 'o', 'l', 'd', '.', 'n', 'i', 'f', 0 });
        PutRecord(patch, "STAT", 0x01000300, model);
        std::vector<std::uint8_t> reference;
        std::vector<std::uint8_t> baseId; PutU32(baseId, 0x01000300); PutText(reference, "NAME", baseId);
        std::vector<std::uint8_t> child; PutRecord(child, "REFR", 0x01000400, reference);
        PutGroup(patch, 0x00000123, 6, child);
        std::vector<std::uint8_t> bytes;
        PutRecord(bytes, "TES4", 0, header);
        bytes.insert(bytes.end(), patch.begin(), patch.end());
        std::ofstream output(root / "WinterholdPatch.esp", std::ios::binary);
        output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size())); output.close();

        const auto resolved = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { "Skyrim.esm", "WinterholdPatch.esp" } });
        Require(resolved.diagnostics.empty());
        const auto* winner = resolved.FindWinning(0x123);
        Require(winner && winner->winning.plugin == "WinterholdPatch.esp" && winner->origins.size() == 2);
        Require(resolved.cells.size() == 1 && resolved.cells.front().references.size() == 1);
        const auto& placed = resolved.cells.front().references.front();
        Require(placed.id == 0x01000400 && placed.baseObjectId == 0x01000300 && placed.sourcePlugin == "WinterholdPatch.esp");
        Require(placed.modelPath == "meshes/winterhold.nif");
    }
    void TestExteriorLandTerrain()
    {
        const auto root = std::filesystem::temp_directory_path() / "navmesh-land-test"; std::filesystem::remove_all(root); std::filesystem::create_directories(root);
        std::vector<std::uint8_t> grouped; PutRecord(grouped, "CELL", 0x700, CellPayload("TerrainCell", true));
        std::vector<std::uint8_t> land; PutRecord(land, "LAND", 0x701, LandPayload(100.0F, 2, 3, 5)); PutGroup(grouped, 0x700, 6, land);
        std::vector<std::uint8_t> file = { 'T', 'E', 'S', '4' }; PutU32(file, 0); PutU32(file, 0); file.insert(file.end(), 12, 0); file.insert(file.end(), grouped.begin(), grouped.end());
        std::ofstream output(root / "Terrain.esm", std::ios::binary); output.write(reinterpret_cast<const char*>(file.data()), static_cast<std::streamsize>(file.size())); output.close();
        const auto resolved = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { "Terrain.esm" } }); Require(resolved.diagnostics.empty());
        const auto terrain = navmesh::skyrim::offline::ExtractTerrain(resolved, resolved.cells.front());
        Require(terrain.landRecordsDecoded == 1 && terrain.mesh.vertices.size() == 1089 && terrain.mesh.triangles.size() == 2048);
        // LAND's stored horizontal basis is reversed from the emitted
        // world-space terrain basis. These distinct corners catch either
        // missed axis reversal, a transpose, or a skipped first delta.
        Require(terrain.mesh.vertices[0].x == 12 * 4096.0F && terrain.mesh.vertices[0].y == -4 * 4096.0F && terrain.mesh.vertices[0].z == 856.0F);
        Require(terrain.mesh.vertices[32].x == 12 * 4096.0F + 32 * 128.0F && terrain.mesh.vertices[32].z == 856.0F);
        Require(terrain.mesh.vertices[1056].x == 12 * 4096.0F && terrain.mesh.vertices[1056].y == -4 * 4096.0F + 32 * 128.0F && terrain.mesh.vertices[1056].z == 840.0F);
        Require(terrain.mesh.vertices[1088].x == 12 * 4096.0F + 32 * 128.0F && terrain.mesh.vertices[1088].z == 816.0F);
        Require(terrain.scene.HasCompleteTriangleProvenance()); const auto& provenance = terrain.scene.triangleProvenance.front(); Require(provenance.terrain && provenance.terrain->landFormId == 0x701 && provenance.terrain->sampleX == 0 && provenance.terrain->sampleY == 0);
        navmesh::core::Cell missing = resolved.cells.front(); missing.id = 0x799; const auto none = navmesh::skyrim::offline::ExtractTerrain(resolved, missing); Require(none.mesh.triangles.empty() && none.landRecordsMissing == 1);
    }
    void WriteTextFile(const std::filesystem::path& path, const std::string& text) { std::ofstream output(path, std::ios::binary); output << text; }
    void TestMo2ProfileImport()
    {
        const auto root = std::filesystem::temp_directory_path() / "navmesh-mo2-import-test"; std::filesystem::remove_all(root);
        std::filesystem::create_directories(root / "Game" / "Data"); std::filesystem::create_directories(root / "mods" / "Patch"); std::filesystem::create_directories(root / "profiles" / "Default"); std::filesystem::create_directories(root / "overwrite" / "meshes");
        WriteTextFile(root / "ModOrganizer.ini", "gamePath=Game\nmods_directory=mods\nprofiles_directory=profiles\noverwrite_directory=overwrite\n");
        WritePlugin(root / "Game" / "Data" / "Base.esm", {}, false, {}); WritePlugin(root / "mods" / "Patch" / "Patch.esp", { "Base.esm" }, false, {});
        WriteTextFile(root / "profiles" / "Default" / "modlist.txt", "+Patch\n"); WriteTextFile(root / "profiles" / "Default" / "plugins.txt", "*Base.esm\n*Patch.esp\n"); WriteTextFile(root / "profiles" / "Default" / "loadorder.txt", "Base.esm\nPatch.esp\n");
        WriteTextFile(root / "Game" / "Data" / "meshes.txt", "base"); WriteTextFile(root / "overwrite" / "meshes" / "marker.nif", "winner");
        const auto imported = navmesh::skyrim::offline::ImportMo2Profile(root, "Default");
        assert(imported.diagnostics.empty()); assert(imported.pluginPaths.size() == 2u && imported.pluginPaths[1].filename() == "Patch.esp"); assert(imported.enabledMods.size() == 1u && imported.enabledMods.front().priority == 0u); assert(!imported.snapshotHash.empty());
        assert(std::any_of(imported.looseAssetWinners.begin(), imported.looseAssetWinners.end(), [](const auto& file) { return file.logicalPath == "meshes/marker.nif" && file.source == "Overwrite"; }));
        const auto overridden = navmesh::skyrim::offline::ImportMo2Profile(root, "Default", root / "mods");
        Require(overridden.diagnostics.empty() && overridden.modsDirectory == root / "mods" && overridden.pluginPaths.size() == 2u);
        WriteTextFile(root / "profiles" / "Default" / "plugins.txt", "*Patch.esp\n"); WriteTextFile(root / "profiles" / "Default" / "loadorder.txt", "Patch.esp\n");
        const auto implicitMasters = navmesh::skyrim::offline::ImportMo2Profile(root, "Default");
        Require(implicitMasters.diagnostics.empty() && implicitMasters.pluginPaths.size() == 2u && implicitMasters.pluginPaths.front().filename() == "Base.esm");

        const auto splitRoot = root / "SplitInstance"; const auto mo2 = splitRoot / "MO2"; const auto storage = splitRoot / "MODS"; std::filesystem::create_directories(mo2); std::filesystem::create_directories(storage / "profiles" / "Nolvus Awakening"); std::filesystem::create_directories(storage / "mods" / "Patch"); std::filesystem::create_directories(splitRoot / "STOCK GAME" / "Data");
        WriteTextFile(mo2 / "ModOrganizer.ini", "base_directory=@ByteArray(../MODS)\ngamePath=@ByteArray(../STOCK GAME)\nprofiles_directory=profiles\nmods_directory=mods\n");
        WritePlugin(splitRoot / "STOCK GAME" / "Data" / "Base.esm", {}, false, {}); WritePlugin(storage / "mods" / "Patch" / "Patch.esp", { "Base.esm" }, false, {});
        WriteTextFile(storage / "profiles" / "Nolvus Awakening" / "modlist.txt", "+Patch\n"); WriteTextFile(storage / "profiles" / "Nolvus Awakening" / "plugins.txt", "*Base.esm\n*Patch.esp\n"); WriteTextFile(storage / "profiles" / "Nolvus Awakening" / "loadorder.txt", "Base.esm\nPatch.esp\n");
        const auto split = navmesh::skyrim::offline::ImportMo2Profile(mo2, "Nolvus Awakening");
        assert(split.diagnostics.empty() && split.profileDirectory == storage / "profiles" / "Nolvus Awakening" && split.gameData == splitRoot / "STOCK GAME" / "Data");

        const auto qtRoot = root / "QtPathInstance"; std::filesystem::create_directories(qtRoot / "profiles" / "Default");
        WriteTextFile(qtRoot / "ModOrganizer.ini", "gamePath=@ByteArray(D:\\\\SteamLibrary\\\\steamapps\\\\common\\\\Skyrim Special Edition)\nprofiles_directory=profiles\nmods_directory=mods\n");
        WriteTextFile(qtRoot / "profiles" / "Default" / "modlist.txt", ""); WriteTextFile(qtRoot / "profiles" / "Default" / "plugins.txt", ""); WriteTextFile(qtRoot / "profiles" / "Default" / "loadorder.txt", "");
        const auto qtPaths = navmesh::skyrim::offline::ImportMo2Profile(qtRoot, "Default");
        const auto expectedGame = std::filesystem::path("D:/SteamLibrary/steamapps/common/Skyrim Special Edition");
        const auto expectedData = std::filesystem::is_directory(expectedGame / "Data") ? expectedGame / "Data" : expectedGame;
        Require(qtPaths.gameData == expectedData);

        const auto utf16Root = root / "Utf16PathInstance"; std::filesystem::create_directories(utf16Root / "profiles" / "Default");
        const std::string utf16Ini = "[General]\r\ngamePath=@ByteArray(D:\\\\SteamLibrary\\\\steamapps\\\\common\\\\Skyrim Special Edition)\r\nprofiles_directory=profiles\r\nmods_directory=mods\r\n";
        std::vector<std::uint8_t> utf16Bytes{ 0xFF, 0xFE }; for (const auto character : utf16Ini) { utf16Bytes.push_back(static_cast<std::uint8_t>(character)); utf16Bytes.push_back(0); }
        std::ofstream utf16Output(utf16Root / "ModOrganizer.ini", std::ios::binary); utf16Output.write(reinterpret_cast<const char*>(utf16Bytes.data()), static_cast<std::streamsize>(utf16Bytes.size())); utf16Output.close();
        WriteTextFile(utf16Root / "profiles" / "Default" / "modlist.txt", ""); WriteTextFile(utf16Root / "profiles" / "Default" / "plugins.txt", ""); WriteTextFile(utf16Root / "profiles" / "Default" / "loadorder.txt", "");
        const auto utf16Paths = navmesh::skyrim::offline::ImportMo2Profile(utf16Root, "Default");
        Require(utf16Paths.gameData == expectedData);
    }
}

namespace
{
    void TestExportMetadata()
    {
        navmesh::core::Cell cell{ .id = 0x1234, .editorId = "SyntheticCell", .isInterior = true };
        const navmesh::reproducibility::ExportMetadata metadata{ .inputPlugin = "fixtures/Synthetic.esp", .selectedCell = &cell, .coverage = { .references = 2, .modelsLoaded = 1 }, .warnings = { "synthetic warning" } };
        const auto json = navmesh::reproducibility::ToJson(metadata);
        assert(json.contains("navmesh-generator/export-metadata"));
        assert(json.contains("\"schema_version\": \"1.0.0\""));
        assert(json.contains("00001234"));
        assert(json.contains("skyrim-world-z-up-v1"));
    }

    void TestOptionalLocalGameData()
    {
        char* configuredData{};
        std::size_t configuredDataLength{};
        if (_dupenv_s(&configuredData, &configuredDataLength, "SKYRIM_DATA_DIR") != 0 || !configuredData || !*configuredData) { std::free(configuredData); return; }
        const auto plugin = std::filesystem::path(configuredData) / "Skyrim.esm";
        std::free(configuredData);
        if (!std::filesystem::is_regular_file(plugin)) return;
        const auto cells = navmesh::skyrim::offline::ListCells(plugin);
        assert(!cells.empty());
    }

    void TestBstTriShapeExtraction()
    {
        const auto tempRoot = std::filesystem::temp_directory_path() / "navmesh-bstrishape-test";
        std::filesystem::remove_all(tempRoot);
        std::filesystem::create_directories(tempRoot / "meshes");
        const auto modelPath = tempRoot / "meshes" / "MarkerX.nif";

        nifly::NifFile nif;
        nif.Create({ nifly::V20_2_0_7, 12, 130 });
        const std::vector<nifly::Vector3> vertices = { { 0.0F, 0.0F, 0.0F }, { 1.0F, 0.0F, 0.0F }, { 0.0F, 1.0F, 0.0F } };
        const std::vector<nifly::Triangle> triangles = { { 0, 1, 2 } };
        assert(nif.CreateShapeFromData("MarkerX", &vertices, &triangles, nullptr) != nullptr);
        assert(nif.Save(modelPath) == 0);

        navmesh::core::Cell cell;
        cell.references.push_back({ .id = 1, .baseObjectId = 2, .recordType = "REFR", .modelPath = "MarkerX.nif", .position = { 10.0F, 20.0F, 30.0F }, .scale = 1.0F, .sourcePlugin = "Patch.esp", .basePlugin = "Base.esm", .baseRecordType = "STAT" });

        const auto geometry = navmesh::skyrim::offline::ExtractGeometry(tempRoot, cell, tempRoot);
        assert(geometry.modelsLoaded == 1u);
        assert(geometry.mesh.vertices.size() == 3u);
        assert(geometry.mesh.triangles.size() == 1u);
        assert(geometry.mesh.vertices[0].x == 10.0F && geometry.mesh.vertices[0].y == 20.0F && geometry.mesh.vertices[0].z == 30.0F);
        assert(geometry.scene.HasCompleteTriangleProvenance());
        const auto& provenance = geometry.scene.triangleProvenance.front();
        assert(provenance.sourceTriangle == 0u && geometry.scene.geometrySources[provenance.geometrySource].reference.plugin == "Patch.esp");
    }

    // Legal synthetic fixture: the render triangle is at z=0 while the packed
    // Havok collision triangle is at z=7. Collision is the only support mesh,
    // while render is retained separately for GLB inspection.
    void TestPackedCollisionPreferredOverRenderFixture()
    {
        const auto root = std::filesystem::temp_directory_path() / "navmesh-packed-collision-fixture";
        std::filesystem::remove_all(root); std::filesystem::create_directories(root / "meshes");
        const auto modelPath = root / "meshes" / "CollisionWins.nif";
        nifly::NifFile nif; nif.Create({ nifly::V20_2_0_7, 12, 130 });
        const std::vector<nifly::Vector3> renderVertices = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
        const std::vector<nifly::Triangle> renderTriangles = { { 0, 1, 2 } };
        Require(nif.CreateShapeFromData("DecorativeRender", &renderVertices, &renderTriangles, nullptr) != nullptr);
        auto packedData = std::make_unique<nifly::hkPackedNiTriStripsData>();
        packedData->numVerts = 3; packedData->compressedVertData = { { 0, 0, 7 }, { 1, 0, 7 }, { 0, 1, 7 } };
        packedData->keyCount = 1; packedData->triData.resize(1); packedData->triData[0].tri = { 0, 1, 2 };
        const auto packedDataId = nif.GetHeader().AddBlock(std::move(packedData));
        auto packedShape = std::make_unique<nifly::bhkPackedNiTriStripsShape>(); packedShape->dataRef.index = packedDataId;
        const auto packedShapeId = nif.GetHeader().AddBlock(std::move(packedShape));
        auto body = std::make_unique<nifly::bhkRigidBody>(); body->shapeRef.index = packedShapeId; body->translation = { 0, 0, 2, 0 };
        const auto bodyId = nif.GetHeader().AddBlock(std::move(body));
        auto collision = std::make_unique<nifly::bhkCollisionObject>(); collision->bodyRef.index = bodyId; collision->targetRef.index = nif.GetBlockID(nif.GetRootNode());
        nif.GetRootNode()->collisionRef.index = nif.GetHeader().AddBlock(std::move(collision));
        Require(nif.Save(modelPath) == 0);

        navmesh::core::Cell cell; cell.references.push_back({ .id = 3, .baseObjectId = 4, .recordType = "REFR", .modelPath = "CollisionWins.nif", .sourcePlugin = "Fixture.esp", .basePlugin = "Fixture.esm", .baseRecordType = "STAT" });
        const auto geometry = navmesh::skyrim::offline::ExtractGeometry(root, cell, root);
        Require(geometry.collisionModelsLoaded == 1 && geometry.renderFallbackModels == 1 && geometry.mesh.triangles.size() == 1 && geometry.scene.renderFallbackMesh.triangles.size() == 1);
        Require(geometry.mesh.vertices.front().z == 9.0F); // Packed z=7 plus Havok body translation.
        const auto& source = geometry.scene.geometrySources.at(geometry.scene.triangleProvenance.front().geometrySource);
        Require(source.sourceType == navmesh::core::GeometrySourceType::Collision && source.confidence == 1.0F && source.collisionType == "hkPackedNiTriStripsData");
        const auto& renderSource = geometry.scene.geometrySources.at(geometry.scene.renderFallbackTriangleProvenance.front().geometrySource);
        Require(renderSource.sourceType == navmesh::core::GeometrySourceType::RenderFallback && geometry.scene.renderFallbackMesh.vertices.front().z == 0.0F);
        navmesh::core::Cell metadataCell{ .id = 0x5, .editorId = "CollisionWins" }; const navmesh::reproducibility::ExportMetadata metadata{ .selectedCell = &metadataCell };
        const auto output = root / "scene.glb";
        const auto exported = navmesh::core::WriteCombinedGlb(output, geometry.scene, {}, {}, metadata);
        Require(exported.objects == 2 && exported.triangles == 2);
        std::ifstream glb(output, std::ios::binary); glb.seekg(12); std::uint32_t jsonLength{}; glb.read(reinterpret_cast<char*>(&jsonLength), sizeof(jsonLength)); glb.seekg(4, std::ios::cur); std::string gltf(jsonLength, '\0'); glb.read(gltf.data(), jsonLength);
        Require(gltf.contains("Collision:") && gltf.contains("Render fallback:"));
    }

    void TestSceneTransforms()
    {
        using navmesh::core::Transform;
        constexpr float halfPi = 1.57079632679F;
        const auto translated = Transform::FromEulerXYZ({ 5, -2, 7 }, {}, 1.0F).ApplyPoint({ 1, 2, 3 });
        Require(translated.x == 6 && translated.y == 0 && translated.z == 10);
        const auto scaled = Transform::FromEulerXYZ({}, {}, 2.0F).ApplyPoint({ 1, -2, 3 });
        Require(scaled.x == 2 && scaled.y == -4 && scaled.z == 6);
        const auto rotated = Transform::FromEulerXYZ({}, { 0, 0, halfPi }).ApplyPoint({ 1, 0, 0 });
        Require(std::abs(rotated.x) < 1.0e-4F && std::abs(rotated.y - 1) < 1.0e-4F);
        navmesh::core::Scene scene;
        scene.nodes.push_back({ .name = "parent", .localTransform = Transform::FromEulerXYZ({ 10, 0, 0 }, { 0, 0, halfPi }) });
        scene.nodes.push_back({ .name = "child", .parent = 0u, .localTransform = Transform::FromEulerXYZ({ 2, 0, 0 }, {}, 1.0F) });
        const auto world = scene.WorldTransform(1); Require(world.has_value());
        const auto point = world->ApplyPoint({});
        Require(std::abs(point.x - 10) < 1.0e-4F && std::abs(point.y - 2) < 1.0e-4F);
    }

    void TestCombinedColorLayeredGlb()
    {
        const auto root = std::filesystem::temp_directory_path() / "navmesh-combined-glb-test"; std::filesystem::remove_all(root); std::filesystem::create_directories(root);
        navmesh::core::Scene scene;
        scene.geometrySources.push_back({ .modelPath = "meshes/fixture.nif", .sourceType = navmesh::core::GeometrySourceType::Collision, .collisionType = "hkPackedNiTriStripsData", .confidence = 1.0F, .reference = { "Fixture.esp", 0x42, "REFR" } });
        scene.mesh = { .vertices = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } }, .triangles = { { { 0, 1, 2 } } } }; scene.triangleProvenance.push_back({ 0, 0 });
        navmesh::core::NavMesh navmesh{ .id = 0x99, .vertices = scene.mesh.vertices, .polygons = { { .vertices = { 0, 1, 2 } } } };
        navmesh::core::Cell cell{ .id = 0x1234, .editorId = "Fixture" }; const navmesh::reproducibility::ExportMetadata metadata{ .selectedCell = &cell };
        const auto output = root / "scene.glb";
        const auto exported = navmesh::core::WriteCombinedGlb(output, scene, { navmesh }, { { .position = { 0, 0, 2 }, .classification = "floating", .navmeshPolygon = 0 } }, metadata);
        Require(exported.objects == 3 && exported.triangles == 6); Require(std::filesystem::file_size(output) > 100);
        std::ifstream glb(output, std::ios::binary); std::uint32_t magic{}; glb.read(reinterpret_cast<char*>(&magic), sizeof(magic)); Require(magic == 0x46546C67);
        std::uint32_t version{}, length{}, jsonLength{}, jsonType{}; glb.read(reinterpret_cast<char*>(&version), sizeof(version)); glb.read(reinterpret_cast<char*>(&length), sizeof(length)); glb.read(reinterpret_cast<char*>(&jsonLength), sizeof(jsonLength)); glb.read(reinterpret_cast<char*>(&jsonType), sizeof(jsonType)); std::string gltf(jsonLength, '\0'); glb.read(gltf.data(), jsonLength); Require(gltf.contains("\"name\":\"Terrain\",\"children\":[]") && gltf.contains("\"name\":\"Collision\",\"children\":[]"));
        std::ifstream provenance(output.string() + ".provenance.json"); std::string text((std::istreambuf_iterator<char>(provenance)), {}); Require(text.contains("Collision") && text.contains("Fixture.esp") && text.contains("Diagnostic: floating"));
        navmesh::core::SceneExportOptions cull{ .layers = { navmesh::core::SceneLayer::Collision }, .bounds = navmesh::core::SceneBounds{ .world = { .min = { 100, 100, -1 }, .max = { 101, 101, 1 } } } };
        const auto culled = navmesh::core::WriteCombinedGlb(root / "culled.glb", scene, {}, {}, metadata, cull); Require(culled.triangles == 0 && culled.culledTriangles == 1);
    }
}

int main()
{
    TestResolvedLoadOrder();
    TestWinterholdCellOverride();
    TestMo2ProfileImport();
    TestLossAwareRecordReader();
    TestExteriorLandTerrain();
    TestExportMetadata();
    TestOptionalLocalGameData();
    TestBstTriShapeExtraction();
    TestPackedCollisionPreferredOverRenderFixture();
    TestSceneTransforms();
    TestCombinedColorLayeredGlb();
    navmesh::core::Mesh mesh{ .vertices = { { -1.0F, 2.0F, 3.0F }, { 4.0F, -5.0F, 6.0F } } };
    const auto bounds = mesh.Bounds();
    assert(bounds.IsValid());
    assert(bounds.min.x == -1.0F && bounds.min.y == -5.0F && bounds.max.z == 6.0F);

    navmesh::core::Cell cell;
    cell.navMeshes.push_back({ .id = 1, .vertices = { { 0.0F, 0.0F, 0.0F }, { 1.0F, 0.0F, 0.0F }, { 0.0F, 1.0F, 0.0F }, { 2.0F, 2.0F, 0.0F } }, .polygons = { { .vertices = { 0, 1, 2 }, .neighbors = { 0, 0, 0 }, .flags = 1 }, { .vertices = { 1, 3, 2 }, .neighbors = { 0, 0, 0 }, .flags = 2 } } });
    const auto findings = navmesh::validation::Validate(cell);
    assert(findings.empty());

    const auto analysis = navmesh::analysis::Analyze(cell.navMeshes.front());
    assert(analysis.vertexCount == 4u);
    assert(analysis.polygonCount == 2u);
    assert(analysis.connectedComponents == 1u);
    assert(analysis.boundingBox.min.x == 0.0F);
    assert(analysis.boundingBox.max.x == 2.0F);
    assert(analysis.isolatedPolygonCount == 0u);
    assert(analysis.degeneratePolygonCount == 0u);
    assert(analysis.averagePolygonArea > 0.0F);

    const auto box = navmesh::core::AABB{ .min = { -1.0F, -2.0F, -3.0F }, .max = { 3.0F, 4.0F, 5.0F } };
    const auto center = box.Center();
    const auto extent = box.Extent();
    assert(center.x == 1.0F && center.y == 1.0F && center.z == 1.0F);
    assert(extent.x == 4.0F && extent.y == 6.0F && extent.z == 8.0F);

    navmesh::analysis::SpatialIndex spatial;
    spatial.Build({ navmesh::core::Triangle{ { 0u, 1u, 2u } } }, {
        { 0.0F, 0.0F, 0.0F },
        { 1.0F, 0.0F, 0.0F },
        { 0.0F, 1.0F, 0.0F }
    });
    const auto nearest = spatial.NearestSurface({ 0.5F, 0.5F, -1.0F }, 5.0F);
    assert(nearest.has_value());
    assert(fabs(nearest->point.z) < 1.0e-4F);
    assert(nearest->distance >= 0.0F);
    const auto overlap = spatial.QueryAABB({ .min = { -0.1F, -0.1F, -0.1F }, .max = { 0.5F, 0.5F, 0.5F } });
    assert(!overlap.empty());

    const auto horizontalNormal = navmesh::core::Vec3{ 0.0F, 0.0F, 1.0F };
    assert(navmesh::analysis::SurfaceSlopeDegrees(horizontalNormal) < 1.0e-3F);

    const auto slopedNormal = navmesh::core::Vec3{ 0.0F, 0.5F, 0.8660254F };
    const auto slopeDegrees = navmesh::analysis::SurfaceSlopeDegrees(slopedNormal);
    assert(slopeDegrees > 29.0F && slopeDegrees < 31.0F);

    assert(navmesh::analysis::ClassifySupport(-3.0F, 10.0F, 2.0F, 45.0F) == "buried");
    assert(navmesh::analysis::ClassifySupport(3.0F, 10.0F, 2.0F, 45.0F) == "floating");
    assert(navmesh::analysis::ClassifySupport(1.0F, 50.0F, 2.0F, 45.0F) == "too_steep");
    assert(navmesh::analysis::ClassifySupport(1.0F, 10.0F, 2.0F, 45.0F) == "supported");

    navmesh::core::NavMesh syntheticMesh{ .vertices = { { 0.0F, 0.0F, 3.0F }, { 1.0F, 0.0F, 3.0F }, { 0.0F, 1.0F, 3.0F } }, .polygons = { { .vertices = { 0, 1, 2 }, .neighbors = { 0, 0, 0 }, .flags = 1 } } };
    navmesh::core::Mesh syntheticGeometry{ .vertices = { { 0.0F, 0.0F, 1.0F }, { 1.0F, 0.0F, 1.0F }, { 0.0F, 1.0F, 1.0F } }, .triangles = { { { 0u, 1u, 2u } } } };
    const auto polygonReport = navmesh::analysis::AnalyzeNavMeshPolygons(syntheticMesh, syntheticGeometry, { .surfaceSearchRadius = 64.0F, .maxSupportDistance = 2.0F, .maxSlope = 45.0F });
    assert(polygonReport.summary.polygonsAnalyzed == 1u);
    assert(polygonReport.polygons.front().support.found);
    assert(polygonReport.polygons.front().classification == "supported");

    navmesh::core::NavMesh centroidMesh{ .vertices = { { 0.5F, 0.5F, 3.0F }, { 1.5F, 0.5F, 3.0F }, { 0.5F, 1.5F, 3.0F } }, .polygons = { { .vertices = { 0, 1, 2 }, .neighbors = { 0, 0, 0 }, .flags = 1 } } };
    navmesh::core::Mesh wallScene{ .vertices = {
        { 0.0F, 0.0F, 0.0F }, { 2.0F, 0.0F, 0.0F }, { 0.0F, 2.0F, 0.0F },
        { 0.45F, 0.0F, 0.0F }, { 0.55F, 0.0F, 1.5F }, { 0.50F, 1.0F, 0.0F }
    }, .triangles = { { { 0u, 1u, 2u } }, { { 3u, 4u, 5u } } } };
    const auto wallSelection = navmesh::analysis::AnalyzeNavMeshPolygons(centroidMesh, wallScene, { .surfaceSearchRadius = 64.0F, .maxSupportDistance = 2.0F, .maxSlope = 45.0F });
    assert(wallSelection.polygons.front().support.found);
    assert(wallSelection.polygons.front().support.triangleIndex == 0u);
    assert(wallSelection.polygons.front().classification == "floating");

    // Milestone 7: collision wins over terrain for a bridge/floor, and the
    // result carries agreement/confidence rather than centroid-only evidence.
    navmesh::core::Mesh priorityGeometry{ .vertices = {
        { 0, 0, 0 }, { 2, 0, 0 }, { 0, 2, 0 },
        { 0, 0, 2 }, { 2, 0, 2 }, { 0, 2, 2 }
    }, .triangles = { { { 0, 1, 2 } }, { { 3, 4, 5 } } } };
    const std::vector<navmesh::analysis::TriangleSource> prioritySources{
        { navmesh::analysis::SupportSourceType::Terrain, 1.0F, "land" },
        { navmesh::analysis::SupportSourceType::Collision, 1.0F, "bridge" }
    };
    const auto priorityReport = navmesh::analysis::AnalyzeNavMeshPolygons(syntheticMesh, priorityGeometry, prioritySources, { .surfaceSearchRadius = 64, .maxSupportDistance = 2, .maxSlope = 45 });
    assert(priorityReport.polygons.front().classification == "supported");
    assert(priorityReport.polygons.front().support.sourceType == "collision");
    assert(priorityReport.polygons.front().support.samplesTotal == 7u);
    assert(priorityReport.polygons.front().support.samplesCovered >= 5u);

    const auto noCoverage = navmesh::analysis::AnalyzeNavMeshPolygons(syntheticMesh, {}, { .surfaceSearchRadius = 64, .maxSupportDistance = 2, .maxSlope = 45 });
    assert(noCoverage.polygons.front().classification == "out_of_coverage");
    assert(noCoverage.summary.outOfCoverage == 1u && noCoverage.repairCandidates.empty());

    navmesh::core::Mesh competingGeometry{ .vertices = {
        { 0, 0, 0 }, { 2, 0, 0 }, { 0, 2, 0 },
        { 0, 0, 20 }, { 2, 0, 20 }, { 0, 2, 20 }
    }, .triangles = { { { 0, 1, 2 } }, { { 3, 4, 5 } } } };
    navmesh::core::NavMesh highMesh = syntheticMesh; for (auto& vertex : highMesh.vertices) vertex.z = 25;
    const auto ambiguous = navmesh::analysis::AnalyzeNavMeshPolygons(highMesh, competingGeometry, {
        { navmesh::analysis::SupportSourceType::Collision, 1.0F, "floor-a" },
        { navmesh::analysis::SupportSourceType::Collision, 1.0F, "floor-b" }
    }, { .surfaceSearchRadius = 64, .maxSupportDistance = 32, .maxSlope = 45 });
    assert(ambiguous.polygons.front().classification == "ambiguous");
    assert(ambiguous.repairCandidates.empty());
    return 0;
}
