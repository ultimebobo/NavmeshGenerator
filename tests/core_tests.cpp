#include "analysis/navmesh_analysis.h"
#include "core/geometry/types.h"
#include "core/reproducibility/export_metadata.h"
#include "core/scene/scene.h"
#include "skyrim/parser/plugin_parser.h"
#include "skyrim/mo2/mo2_importer.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/extraction/terrain_extractor.h"
#include "core/scene/scene_exporter.h"
#include "core/navmesh/candidate.h"
#include "validation/validation.h"

#include <array>
#include <algorithm>

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
#include <string_view>
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
    void TestCellOverrideAcrossPlugins()
    {
        const auto root = std::filesystem::temp_directory_path() / "navmesh-cell-override-test";
        std::filesystem::create_directories(root);
        std::vector<std::uint8_t> baseChildren;
        PutRecord(baseChildren, "REFR", 0x401, {});
        std::vector<std::uint8_t> baseRecords;
        PutRecord(baseRecords, "CELL", 0x123, CellPayload("ExampleExteriorCell", true));
        PutGroup(baseRecords, 0x123, 6, baseChildren);
        std::vector<std::uint8_t> baseBytes;
        PutRecord(baseBytes, "TES4", 0, {});
        baseBytes.insert(baseBytes.end(), baseRecords.begin(), baseRecords.end());
        std::ofstream baseOutput(root / "Base.esm", std::ios::binary);
        baseOutput.write(reinterpret_cast<const char*>(baseBytes.data()), static_cast<std::streamsize>(baseBytes.size())); baseOutput.close();

        std::vector<std::uint8_t> header;
        PutText(header, "MAST", { 'B', 'a', 's', 'e', '.', 'e', 's', 'm', 0 });
        std::vector<std::uint8_t> patch;
        PutRecord(patch, "CELL", 0x00000123, CellPayload("ExampleExteriorCell", true));
        std::vector<std::uint8_t> model;
        PutText(model, "MODL", { 'm', 'e', 's', 'h', 'e', 's', '/', 'f', 'i', 'x', 't', 'u', 'r', 'e', '.', 'n', 'i', 'f', 0 });
        PutRecord(patch, "STAT", 0x01000300, model);
        std::vector<std::uint8_t> reference;
        std::vector<std::uint8_t> baseId; PutU32(baseId, 0x01000300); PutText(reference, "NAME", baseId);
        std::vector<std::uint8_t> child; PutRecord(child, "REFR", 0x01000400, reference);
        PutRecord(child, "REFR", 0x00000401, reference, 1U << 11);
        PutRecord(child, "REFR", 0x01000402, reference, 1U << 5);
        PutGroup(patch, 0x00000123, 6, child);
        std::vector<std::uint8_t> bytes;
        PutRecord(bytes, "TES4", 0, header);
        bytes.insert(bytes.end(), patch.begin(), patch.end());
        std::ofstream output(root / "FixturePatch.esp", std::ios::binary);
        output.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size())); output.close();

        const auto resolved = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { "Base.esm", "FixturePatch.esp" } });
        Require(resolved.diagnostics.empty());
        const auto* winner = resolved.FindWinning(0x123);
        Require(winner && winner->winning.plugin == "FixturePatch.esp" && winner->origins.size() == 2);
        Require(resolved.cells.size() == 1 && resolved.cells.front().references.size() == 3);
        const auto& references = resolved.cells.front().references;
        const auto findReference = [&](std::uint32_t id) -> const navmesh::core::Reference& { return *std::find_if(references.begin(), references.end(), [=](const auto& ref) { return ref.id == id; }); };
        const auto& placed = findReference(0x01000400);
        Require(placed.id == 0x01000400 && placed.baseObjectId == 0x01000300 && placed.sourcePlugin == "FixturePatch.esp");
        Require(placed.modelPath == "meshes/fixture.nif");
        Require(!placed.initiallyDisabled && !placed.deleted);
        Require(findReference(0x401).initiallyDisabled && !findReference(0x401).deleted);
        Require(resolved.FindWinning(0x401)->origins.size() == 2);
        Require(!findReference(0x01000402).initiallyDisabled && findReference(0x01000402).deleted);
        const auto extracted = navmesh::skyrim::offline::ExtractGeometry(root, resolved.cells.front(), {});
        Require(extracted.modelsExcluded == 2 && extracted.modelsMissing == 1 && extracted.scene.coverage.size() == 3);
        const auto findReport = [&](std::uint32_t id) -> const navmesh::skyrim::offline::GeometryReferenceReport& { return *std::find_if(extracted.references.begin(), extracted.references.end(), [=](const auto& report) { return report.formId == id; }); };
        Require(findReport(0x401).failure == "winning reference record is initially disabled");
        Require(findReport(0x01000402).failure == "winning reference record is deleted");
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
        // The stored row-major LAND grid matches the world-space X/Y basis.
        // Distinct corner heights catch a transpose or a skipped first delta.
        Require(terrain.mesh.vertices[0].x == 12 * 4096.0F && terrain.mesh.vertices[0].y == -4 * 4096.0F && terrain.mesh.vertices[0].z == 816.0F);
        Require(terrain.mesh.vertices[32].x == 12 * 4096.0F + 32 * 128.0F && terrain.mesh.vertices[32].z == 840.0F);
        Require(terrain.mesh.vertices[1056].x == 12 * 4096.0F && terrain.mesh.vertices[1056].y == -4 * 4096.0F + 32 * 128.0F && terrain.mesh.vertices[1056].z == 856.0F);
        Require(terrain.mesh.vertices[1088].x == 12 * 4096.0F + 32 * 128.0F && terrain.mesh.vertices[1088].z == 856.0F);
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
        WriteTextFile(root / "mods" / "Patch" / "Patch - Meshes.bsa", "archive fixture path");
        const auto imported = navmesh::skyrim::offline::ImportMo2Profile(root, "Default");
        assert(imported.diagnostics.empty()); assert(imported.pluginPaths.size() == 2u && imported.pluginPaths[1].filename() == "Patch.esp"); assert(imported.enabledMods.size() == 1u && imported.enabledMods.front().priority == 0u); assert(!imported.snapshotHash.empty());
        assert(std::any_of(imported.looseAssetWinners.begin(), imported.looseAssetWinners.end(), [](const auto& file) { return file.logicalPath == "meshes/marker.nif" && file.source == "Overwrite"; }));
        Require(imported.archivePaths.size() == 1 && imported.archivePaths.front().filename() == "Patch - Meshes.bsa");
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
        Require(nif.CreateShapeFromData("MarkerX", &vertices, &triangles, nullptr) != nullptr);
        Require(nif.Save(modelPath) == 0);

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

        // The game Data directory lacks this mesh. An MO2 loose winner must
        // still load the placed model, including its render geometry.
        const auto emptyData = tempRoot / "empty-data";
        std::filesystem::create_directories(emptyData);
        navmesh::skyrim::offline::ModelAssetSources assets;
        assets.looseModels.emplace("meshes/markerx.nif", modelPath);
        const auto fromMo2 = navmesh::skyrim::offline::ExtractGeometry(emptyData, cell, {}, {}, {}, &assets);
        Require(fromMo2.modelsLoaded == 1 && fromMo2.mesh.triangles.size() == 1 && fromMo2.modelsMissing == 0);

        // Placed DATA angles use the game's matrix convention. A positive Z
        // angle moves local +X toward world -Y, including render fallbacks.
        cell.references.front().rotation.z = 1.57079632679F;
        const auto rotatedGeometry = navmesh::skyrim::offline::ExtractGeometry(tempRoot, cell, tempRoot);
        Require(std::abs(rotatedGeometry.mesh.vertices[1].x - 10.0F) < 1.0e-4F);
        Require(std::abs(rotatedGeometry.mesh.vertices[1].y - 19.0F) < 1.0e-4F);
    }

    // Legal synthetic fixture: Havok coordinates and rigid-body translation
    // are in Havok units; collision must be converted to Skyrim world units.
    // Render is retained separately for GLB inspection.
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
        constexpr float skyrimUnitsPerHavokUnit = 69.99125F;
        Require(std::abs(geometry.mesh.vertices.front().z - 9.0F * skyrimUnitsPerHavokUnit) < 0.01F);
        Require(std::abs(geometry.mesh.vertices[1].x - skyrimUnitsPerHavokUnit) < 0.01F);
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

        cell.references.front().rotation.z = 1.57079632679F;
        const auto rotated = navmesh::skyrim::offline::ExtractGeometry(root, cell, root);
        Require(std::abs(rotated.mesh.vertices[1].x) < 0.01F);
        Require(std::abs(rotated.mesh.vertices[1].y + skyrimUnitsPerHavokUnit) < 0.01F);
        Require(std::abs(rotated.scene.renderFallbackMesh.vertices[1].x) < 1.0e-4F);
        Require(std::abs(rotated.scene.renderFallbackMesh.vertices[1].y + 1.0F) < 1.0e-4F);
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
        const auto skyrimZ = Transform::FromSkyrimReference({}, { 0, 0, halfPi }).ApplyPoint({ 1, 0, 0 });
        Require(std::abs(skyrimZ.x) < 1.0e-4F && std::abs(skyrimZ.y + 1) < 1.0e-4F);
        const auto skyrimMixed = Transform::FromSkyrimReference({ 10, 20, 30 }, { halfPi, 0, halfPi }, 2).ApplyPoint({ 0, 1, 0 });
        Require(std::abs(skyrimMixed.x - 12) < 1.0e-4F && std::abs(skyrimMixed.y - 20) < 1.0e-4F && std::abs(skyrimMixed.z - 30) < 1.0e-4F);
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
        const auto exported = navmesh::core::WriteCombinedGlb(output, scene, { navmesh }, { { .position = { 0, 0, 2 }, .classification = "floating", .navmeshPolygon = 0, .navmeshFormId = navmesh.id } }, metadata);
        Require(exported.objects == 3 && exported.triangles == 6); Require(std::filesystem::file_size(output) > 100);
        std::ifstream glb(output, std::ios::binary); std::uint32_t magic{}; glb.read(reinterpret_cast<char*>(&magic), sizeof(magic)); Require(magic == 0x46546C67);
        std::uint32_t version{}, length{}, jsonLength{}, jsonType{}; glb.read(reinterpret_cast<char*>(&version), sizeof(version)); glb.read(reinterpret_cast<char*>(&length), sizeof(length)); glb.read(reinterpret_cast<char*>(&jsonLength), sizeof(jsonLength)); glb.read(reinterpret_cast<char*>(&jsonType), sizeof(jsonType)); std::string gltf(jsonLength, '\0'); glb.read(gltf.data(), jsonLength); Require(gltf.contains("\"name\":\"Terrain\",\"children\":[]") && gltf.contains("\"name\":\"Collision\",\"children\":["));
        Require(gltf.contains("\"name\":\"Existing NAVM 00000099: floating\"") && gltf.contains("\"material\":5"));
        Require(gltf.contains("\"name\":\"Too steep (yellow)\"") && gltf.contains("\"name\":\"Blocked (magenta)\"") && gltf.contains("\"name\":\"Out of coverage (blue)\"") && gltf.contains("\"name\":\"Ambiguous (violet)\""));
        std::ifstream provenance(output.string() + ".provenance.json"); std::string text((std::istreambuf_iterator<char>(provenance)), {}); Require(text.contains("Collision") && text.contains("Fixture.esp") && text.contains("Diagnostic: floating"));
        const std::array<std::pair<const char*, std::size_t>, 7> classifications{{ { "supported", 4 }, { "floating", 5 }, { "buried", 6 }, { "too_steep", 7 }, { "blocked", 8 }, { "out_of_coverage", 9 }, { "ambiguous", 10 } }};
        navmesh.polygons.resize(classifications.size() + 1, navmesh.polygons.front());
        std::vector<navmesh::core::DiagnosticMarker> classMarkers;
        for (std::size_t i{}; i < classifications.size(); ++i) classMarkers.push_back({ .classification = classifications[i].first, .navmeshPolygon = i, .navmeshFormId = navmesh.id });
        navmesh::core::SceneExportOptions navmeshOnly{ .layers = { navmesh::core::SceneLayer::ExistingNavmesh } };
        const auto classifiedPath = root / "classified.glb";
        const auto classified = navmesh::core::WriteCombinedGlb(classifiedPath, scene, { navmesh }, classMarkers, metadata, navmeshOnly);
        Require(classified.objects == classifications.size() + 1 && classified.triangles == classifications.size() + 1);
        std::ifstream classifiedGlb(classifiedPath, std::ios::binary); classifiedGlb.seekg(12); classifiedGlb.read(reinterpret_cast<char*>(&jsonLength), sizeof(jsonLength)); classifiedGlb.seekg(4, std::ios::cur); std::string classifiedJson(jsonLength, '\0'); classifiedGlb.read(classifiedJson.data(), jsonLength);
        for (const auto& [name, material] : classifications) {
            const auto start = classifiedJson.find(std::format("\"name\":\"Existing NAVM 00000099: {}\",\"primitives\"", name));
            Require(start != std::string::npos);
            const auto end = classifiedJson.find("}]}", start);
            Require(end != std::string::npos && classifiedJson.substr(start, end - start).contains(std::format("\"material\":{}", material)));
        }
        Require(classifiedJson.contains("\"name\":\"Existing NAVM 00000099: unclassified\""));
        navmesh::core::SceneExportOptions cull{ .layers = { navmesh::core::SceneLayer::Collision }, .bounds = navmesh::core::SceneBounds{ .world = { .min = { 100, 100, -1 }, .max = { 101, 101, 1 } } } };
        const auto culled = navmesh::core::WriteCombinedGlb(root / "culled.glb", scene, {}, {}, metadata, cull); Require(culled.triangles == 0 && culled.culledTriangles == 1);
    }
    void TestCandidateGeneration()
    {
        using namespace navmesh::core;
        Scene scene;
        scene.geometrySources.push_back({ .sourceType = GeometrySourceType::Terrain, .confidence = 1.0F,
            .reference = { "Fixture.esm", 0x100, "LAND" } });
        scene.geometrySources.push_back({ .sourceType = GeometrySourceType::Collision, .confidence = 1.0F,
            .reference = { "Fixture.esm", 0x200, "REFR" } });
        scene.geometrySources.push_back({ .sourceType = GeometrySourceType::RenderFallback, .confidence = 0.35F,
            .reference = { "Fixture.esm", 0x300, "REFR" } });
        scene.mesh.vertices = { {0,0,0},{128,0,0},{128,128,0},{0,128,0},
            {200,0,0},{328,0,0},{328,128,500}, {400,0,0},{528,0,0},{528,128,0} };
        scene.mesh.triangles = { {{0,1,2}},{{0,2,3}},{{4,5,6}},{{7,8,9}} };
        scene.triangleProvenance = { {0,0,{}},{0,1,{}},{1,0,{}},{2,0,{}} };
        auto profile = *FindNavigationProfile("human@1.0.0");
        Require(!FindNavigationProfile("human@9.0.0"));
        profile.agentRadius = 0; profile.minimumRegionArea = 0;
        const auto flat = GenerateCandidate(scene,profile);
        Require(flat.topology.valid && flat.mesh.polygons.size() == 2 && flat.regions.size() == 1);
        Require(flat.statistics.rejectedSlope == 1 && flat.statistics.rejectedSource == 1);
        Require(flat.contours.size() == 1 && flat.contours.front().closed && flat.contours.front().vertices.size() == 4);
        const auto root = std::filesystem::temp_directory_path() / "navmesh-candidate-test";
        std::filesystem::create_directories(root);
        Require(WriteCandidateJson(root / "first.json",flat,scene,"{}"));
        Require(WriteCandidateJson(root / "second.json",GenerateCandidate(scene,profile),scene,"{}"));
        std::ifstream first(root / "first.json",std::ios::binary), second(root / "second.json",std::ios::binary);
        Require(std::string(std::istreambuf_iterator<char>(first),{}) == std::string(std::istreambuf_iterator<char>(second),{}));
        Require(WriteCandidateObj(root / "candidate.obj",flat));
        auto broken = flat; broken.mesh.polygons[0].neighbors[0] = 42;
        Require(!ValidateCandidateTopology(broken).valid);

        Scene covered = scene;
        covered.mesh.vertices.insert(covered.mesh.vertices.end(), { {0,0,80},{128,0,80},{128,128,80},{0,128,80} });
        covered.mesh.triangles.insert(covered.mesh.triangles.end(), { {{10,11,12}},{{10,12,13}} });
        covered.triangleProvenance.insert(covered.triangleProvenance.end(), { {1,1,{}},{1,2,{}} });
        const auto obstructed = GenerateCandidate(covered,profile);
        Require(obstructed.statistics.rejectedClearance == 2 && obstructed.mesh.polygons.size() == 2);
        Require(obstructed.polygonSourceTriangles[0] >= 4 && obstructed.topology.valid);
        Scene walled = scene;
        walled.mesh.vertices.insert(walled.mesh.vertices.end(), { {64,-20,0},{64,150,0},{64,150,200},{64,-20,200} });
        walled.mesh.triangles.insert(walled.mesh.triangles.end(), { {{10,11,12}},{{10,12,13}} });
        walled.triangleProvenance.insert(walled.triangleProvenance.end(), { {1,1,{}},{1,2,{}} });
        const auto blocked = GenerateCandidate(walled,profile);
        Require(blocked.statistics.rejectedObstruction == 2 && blocked.mesh.polygons.empty());
        Scene bridge = scene;
        bridge.mesh.vertices.insert(bridge.mesh.vertices.end(), { {0,0,200},{128,0,200},{128,128,200},{0,128,200} });
        bridge.mesh.triangles.insert(bridge.mesh.triangles.end(), { {{10,11,12}},{{10,12,13}} });
        bridge.triangleProvenance.insert(bridge.triangleProvenance.end(), { {1,1,{}},{1,2,{}} });
        const auto stacked = GenerateCandidate(bridge,profile,std::nullopt,
            { { .referenceId = 0x401, .position = {64,64,0} }, { .referenceId = 0x402, .position = {64,64,200} } });
        Require(stacked.topology.valid && stacked.mesh.polygons.size() == 4 && stacked.regions.size() == 2);
        Require(stacked.exits[0].region && stacked.exits[1].region && stacked.exits[0].region != stacked.exits[1].region);
        profile.agentRadius = 16;
        const auto inset = GenerateCandidate(scene,profile);
        Require(inset.topology.valid && !inset.mesh.polygons.empty());
        for (const auto& vertex : inset.mesh.vertices) if (vertex.x < 130) Require(vertex.x >= 15.9F && vertex.y >= 15.9F && vertex.x <= 112.1F && vertex.y <= 112.1F);
        Scene stepScene;
        stepScene.geometrySources.push_back(scene.geometrySources[0]);
        stepScene.mesh.vertices = { {0,0,0},{128,0,0},{128,128,0}, {128,0,10},{256,0,10},{128,128,10} };
        stepScene.mesh.triangles = { {{0,1,2}},{{3,4,5}} };
        stepScene.triangleProvenance = { {0,0,{}},{0,1,{}} };
        profile.agentRadius = 0; profile.agentHeight = 5; profile.clearance = 5;
        const auto stitched = GenerateCandidate(stepScene,profile);
        Require(stitched.topology.valid && stitched.mesh.polygons.size() == 2 && stitched.regions.size() == 1);
        const auto oneCell = GenerateCandidate(stepScene,profile,AABB{ .min = {0,0,-100}, .max = {128,128,300} });
        Require(oneCell.topology.valid && oneCell.mesh.polygons.size() == 1);
        profile.stepHeight = 0;
        const auto separated = GenerateCandidate(stepScene,profile,std::nullopt,
            { { .referenceId = 0x403, .position = {80,40,0} }, { .referenceId = 0x404, .position = {170,40,10} } });
        Require(separated.topology.valid && separated.regions.size() == 2);
        Scene junction;
        junction.geometrySources.push_back(scene.geometrySources[0]);
        junction.mesh.vertices = { {0,0,0},{128,0,0},{128,128,0},{256,0,0},{128,64,0} };
        junction.mesh.triangles = { {{0,1,2}},{{1,3,4}},{{4,3,2}} };
        junction.triangleProvenance = { {0,0,{}},{0,1,{}},{0,2,{}} };
        const auto joined = GenerateCandidate(junction,profile,std::nullopt,
            { { .referenceId = 0x405, .position = {64,32,0} } });
        Require(joined.topology.valid && joined.regions.size() == 1 && joined.mesh.polygons.size() == 4);
        Require(joined.regions[0].exitFormIds == std::vector<std::uint32_t>{0x405});
        Scene seamed = junction;
        seamed.mesh.vertices = { {0,0,0},{128,0,0},{128,128,0},
            {128.08F,0,0},{256,0,0},{128.08F,64,0},{128.08F,128,0} };
        seamed.mesh.triangles = { {{0,1,2}},{{3,4,5}},{{5,4,6}} };
        const auto joinedSeam = GenerateCandidate(seamed,profile,std::nullopt,
            { { .referenceId = 0x405, .position = {64,32,0} } });
        Require(joinedSeam.topology.valid && joinedSeam.regions.size() == 1 && joinedSeam.mesh.polygons.size() == 4);
        Scene ramp;
        ramp.geometrySources.push_back(scene.geometrySources[1]);
        ramp.mesh.vertices = { {0,0,0},{128,0,0},{128,128,0},{0,128,0},
            {256,0,100},{256,128,100},{384,0,100},{384,128,100} };
        ramp.mesh.triangles = { {{0,1,2}},{{0,2,3}},{{1,4,5}},{{1,5,2}},{{4,6,7}},{{4,7,5}} };
        for (std::size_t i{}; i < ramp.mesh.triangles.size(); ++i) ramp.triangleProvenance.push_back({0,i,{}});
        profile.stepHeight = 18; profile.agentHeight = 128; profile.clearance = 128;
        const auto layered = GenerateCandidate(ramp,profile,std::nullopt,
            { { .referenceId = 0x406, .position = {320,64,100} } });
        Require(layered.topology.valid && layered.regions.size() == 1 && layered.mesh.polygons.size() == 6);
        Require(layered.exits[0].region == 0);
        Scene island = junction;
        island.mesh.vertices.insert(island.mesh.vertices.end(),{{600,0,0},{728,0,0},{600,128,0}});
        island.mesh.triangles.push_back({{5,6,7}}); island.triangleProvenance.push_back({0,3,{}});
        const auto reachable = GenerateCandidate(island,profile,std::nullopt,
            { { .referenceId = 0x407, .position = {64,32,0} } });
        Require(reachable.topology.valid && reachable.regions.size() == 1 && reachable.statistics.rejectedUnreachable == 1);
        Scene crossing;
        crossing.geometrySources.push_back(scene.geometrySources[0]);
        crossing.mesh.vertices = { {100,0,0},{150,0,0},{100,100,0} };
        crossing.mesh.triangles = { {{0,1,2}} };
        crossing.triangleProvenance = { {0,0,{}} };
        profile.agentRadius = 8;
        const auto clippedBorder = GenerateCandidate(crossing,profile,AABB{ .min = {0,0,-100}, .max = {128,128,100} });
        Require(clippedBorder.topology.valid && clippedBorder.regions.size() == 1 && clippedBorder.regions[0].reachesBorder);
        Require(std::any_of(clippedBorder.mesh.vertices.begin(),clippedBorder.mesh.vertices.end(),
            [](Vec3 vertex){ return std::abs(vertex.x-128.0F) < 0.01F; }));
        Scene overlap;
        overlap.geometrySources.push_back(scene.geometrySources[1]);
        overlap.mesh.vertices = { {0,0,0},{128,0,0},{128,128,0},{0,128,0},
            {120,0,0},{248,0,0},{248,128,0},{120,128,0} };
        overlap.mesh.triangles = { {{0,1,2}},{{0,2,3}},{{4,5,6}},{{4,6,7}} };
        for (std::size_t i{}; i < overlap.mesh.triangles.size(); ++i) overlap.triangleProvenance.push_back({0,i,{}});
        profile.agentRadius = 16;
        const auto bridged = GenerateCandidate(overlap,profile,std::nullopt,
            { { .referenceId = 0x408, .position = {64,64,0} } });
        Require(bridged.topology.valid && bridged.regions.size() == 1 && bridged.mesh.polygons.size() >= 6);
        Scene gap = overlap;
        for (std::size_t i = 4; i < 8; ++i) gap.mesh.vertices[i].x += 20;
        const auto unsupportedGap = GenerateCandidate(gap,profile,std::nullopt,
            { { .referenceId = 0x408, .position = {64,64,0} } });
        Require(unsupportedGap.topology.valid && unsupportedGap.regions.size() == 1 && unsupportedGap.statistics.rejectedUnreachable >= 2);
        Scene raised;
        raised.geometrySources = { scene.geometrySources[0], scene.geometrySources[1] };
        raised.mesh.vertices = { {0,0,0},{128,0,0},{128,128,0},{0,128,0},
            {400,400,160},{1000,400,160},{1000,600,160},{400,600,160},
            {1200,400,160},{1264,400,160},{1264,464,160},{1200,464,160} };
        raised.mesh.triangles = { {{0,1,2}},{{0,2,3}},{{4,5,6}},{{4,6,7}},{{8,9,10}},{{8,10,11}} };
        raised.triangleProvenance = { {0,0,{}},{0,1,{}},{1,0,{}},{1,1,{}},{1,2,{}},{1,3,{}} };
        const auto raisedCandidate = GenerateCandidate(raised,profile,
            AABB{ .min = {0,0,-100}, .max = {2048,2048,300} });
        Require(raisedCandidate.topology.valid && raisedCandidate.regions.size() == 2);
        Require(std::any_of(raisedCandidate.mesh.vertices.begin(),raisedCandidate.mesh.vertices.end(),
            [](Vec3 vertex){ return std::abs(vertex.z-160.0F) < 0.01F && vertex.x > 400 && vertex.x < 1000; }));
        Require(std::none_of(raisedCandidate.mesh.vertices.begin(),raisedCandidate.mesh.vertices.end(),
            [](Vec3 vertex){ return vertex.x > 1200 && vertex.x < 1264; }));
        Scene overlappingLevels;
        overlappingLevels.geometrySources.push_back(scene.geometrySources[1]);
        overlappingLevels.mesh.vertices = { {0,0,0},{128,0,0},{0,128,0},
            {0,0,10},{128,0,10},{0,128,10} };
        overlappingLevels.mesh.triangles = { {{0,1,2}},{{3,4,5}} };
        overlappingLevels.triangleProvenance = { {0,0,{}},{0,1,{}} };
        profile.agentRadius = 0;
        const auto independentLevels = GenerateCandidate(overlappingLevels,profile,std::nullopt,
            { { .referenceId = 0x410, .position = {32,32,0} },
              { .referenceId = 0x411, .position = {32,32,10} } });
        Require(independentLevels.topology.valid && independentLevels.regions.size() == 2);
        Scene steppedFan;
        steppedFan.geometrySources.push_back(scene.geometrySources[0]);
        steppedFan.mesh.vertices = { {0,0,0}, {64,0,0}, {0,64,0}, {0,64,10}, {-64,0,0},
            {-64,0,10}, {0,-64,0}, {0,-64,10}, {64,0,10} };
        steppedFan.mesh.triangles = { {{0,1,2}}, {{0,3,4}}, {{0,5,6}}, {{0,7,8}} };
        for (std::size_t i{}; i < steppedFan.mesh.triangles.size(); ++i)
            steppedFan.triangleProvenance.push_back({0,i,{}});
        profile.stepHeight = 18;
        const auto stepRing = GenerateCandidate(steppedFan,profile,std::nullopt,
            { { .referenceId = 0x412, .position = {0,0,0} } });
        Require(stepRing.topology.valid && stepRing.mesh.polygons.size() == 4 && stepRing.regions.size() == 1);
        const auto makeTerrainGrid = [&](bool peak, bool opening, bool ripple = false) {
            Scene grid;
            grid.geometrySources.push_back(scene.geometrySources[0]);
            for (std::uint32_t y{}; y <= 6; ++y) for (std::uint32_t x{}; x <= 6; ++x)
                grid.mesh.vertices.push_back({x*64.0F,y*64.0F,
                    peak && x == 3 && y == 3 ? 32.0F : ripple && (x+y)%2 ? 4.0F : 0.0F});
            for (std::uint32_t y{}; y < 6; ++y) for (std::uint32_t x{}; x < 6; ++x) {
                if (opening && x == 2 && y == 2) continue;
                const auto a = y*7+x, b = a+1, d = a+7, c = d+1;
                grid.mesh.triangles.push_back({{a,b,c}});
                grid.triangleProvenance.push_back({0,grid.triangleProvenance.size(),{}});
                grid.mesh.triangles.push_back({{a,c,d}});
                grid.triangleProvenance.push_back({0,grid.triangleProvenance.size(),{}});
            }
            return grid;
        };
        const auto flatGrid = makeTerrainGrid(false,false);
        const auto simpler = GenerateCandidate(flatGrid,profile);
        Require(simpler.topology.valid && simpler.regions.size() == 1);
        Require(simpler.mesh.polygons.size() < flatGrid.mesh.triangles.size()/2);
        Require(simpler.statistics.polygonsBeforeSimplification == flatGrid.mesh.triangles.size());
        Require(simpler.regions[0].sourceTriangles.size() == flatGrid.mesh.triangles.size());
        Require(std::abs(simpler.regions[0].area-384.0F*384.0F) < 0.1F);
        Require(std::any_of(simpler.polygonContributingTriangles.begin(),simpler.polygonContributingTriangles.end(),
            [](const auto& sources){ return sources.size() > 1; }));
        const auto rippleGrid = makeTerrainGrid(false,false,true);
        const auto smoothed = GenerateCandidate(rippleGrid,profile);
        Require(smoothed.topology.valid && smoothed.mesh.polygons.size() < rippleGrid.mesh.triangles.size());
        for (const auto& original : rippleGrid.mesh.vertices) {
            bool sampleCovered{};
            for (const auto& polygon : smoothed.mesh.polygons) {
                const auto a = smoothed.mesh.vertices[polygon.vertices[0]];
                const auto b = smoothed.mesh.vertices[polygon.vertices[1]];
                const auto c = smoothed.mesh.vertices[polygon.vertices[2]];
                const auto area = (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
                const auto u = ((b.x-original.x)*(c.y-original.y)-(b.y-original.y)*(c.x-original.x))/area;
                const auto v = ((original.x-a.x)*(c.y-a.y)-(original.y-a.y)*(c.x-a.x))/area;
                const auto w = 1.0F-u-v;
                if (u < -1.0e-4F || v < -1.0e-4F || w < -1.0e-4F) continue;
                Require(std::abs(original.z-(u*a.z+v*b.z+w*c.z)) <= 16.001F);
                sampleCovered = true;
                break;
            }
            Require(sampleCovered);
        }
        const auto peaked = GenerateCandidate(makeTerrainGrid(true,false),profile);
        Require(peaked.topology.valid && std::any_of(peaked.mesh.vertices.begin(),peaked.mesh.vertices.end(),
            [](Vec3 vertex){ return vertex.x == 192.0F && vertex.y == 192.0F && vertex.z == 32.0F; }));
        const auto withOpening = GenerateCandidate(makeTerrainGrid(false,true),profile);
        Require(withOpening.topology.valid && std::abs(withOpening.regions[0].area-35.0F*64.0F*64.0F) < 0.1F);
        Require(std::none_of(withOpening.mesh.polygons.begin(),withOpening.mesh.polygons.end(),[&](const NavPolygon& polygon) {
            const auto& a = withOpening.mesh.vertices[polygon.vertices[0]];
            const auto& b = withOpening.mesh.vertices[polygon.vertices[1]];
            const auto& c = withOpening.mesh.vertices[polygon.vertices[2]];
            const auto center = (a+b+c)/3.0F;
            return center.x > 128.0F && center.x < 192.0F && center.y > 128.0F && center.y < 192.0F;
        }));
        navmesh::core::Cell cell{ .id = 0x400, .editorId = "CandidateFixture" };
        const navmesh::reproducibility::ExportMetadata metadata{ .selectedCell = &cell };
        navmesh::core::SceneExportOptions visual{ .layers = { SceneLayer::CandidateNavmesh }, .candidateNavmesh = &flat.mesh };
        const auto visualPath = root / "candidate.glb";
        const auto exported = WriteCombinedGlb(visualPath,scene,{}, {},metadata,visual);
        Require(exported.triangles == 2);
        std::ifstream visualGlb(visualPath,std::ios::binary); std::string visualBytes(std::istreambuf_iterator<char>(visualGlb),{});
        Require(visualBytes.contains("Candidate NAVM"));
    }
}

int main(int argc, char** argv)
{
    if (argc > 1 && std::string_view(argv[1]) == "--scene-only") { TestCombinedColorLayeredGlb(); return 0; }
    if (argc > 1 && std::string_view(argv[1]) == "--candidate-only") { TestCandidateGeneration(); return 0; }
    TestResolvedLoadOrder();
    TestCellOverrideAcrossPlugins();
    TestMo2ProfileImport();
    TestLossAwareRecordReader();
    TestExteriorLandTerrain();
    TestExportMetadata();
    TestOptionalLocalGameData();
    TestBstTriShapeExtraction();
    TestPackedCollisionPreferredOverRenderFixture();
    TestSceneTransforms();
    TestCombinedColorLayeredGlb();
    TestCandidateGeneration();
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
