#include "analysis/navmesh_analysis.h"
#include "core/geometry/types.h"
#include "core/reproducibility/export_metadata.h"
#include "core/scene/scene.h"
#include "skyrim/parser/plugin_parser.h"
#include "skyrim/parser/plugin_writer.h"
#include "skyrim/parser/affected_cells.h"
#include "skyrim/mo2/mo2_importer.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "skyrim/extraction/terrain_extractor.h"
#include "core/scene/scene_exporter.h"
#include "core/navmesh/candidate.h"
#include "core/navmesh/generator.h"
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
#include <sstream>
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
        std::vector<std::uint8_t> obnd;
        for (const auto coordinate : {-3,-4,0,3,4,0}) PutU16(obnd,static_cast<std::uint16_t>(coordinate));
        PutText(base,"OBND",obnd);
        WritePlugin(root / "Records.esp", {}, false, { { "CELL", 0x100, Compressed(cell) }, { "NAVM", 0x101, NavmPayload() }, { "STAT", 0x102, base } });
        // Mark the first non-TES4 record compressed in-place; this keeps the fixture builder intentionally small.
        std::fstream compressedFile(root / "Records.esp", std::ios::in | std::ios::out | std::ios::binary); compressedFile.seekp(24 + 8); const std::uint32_t compressedFlag = 0x40000; compressedFile.write(reinterpret_cast<const char*>(&compressedFlag), sizeof(compressedFlag)); compressedFile.close();
        const auto parsed = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { "Records.esp" } });
        const auto* compressedCell = parsed.FindWinning(0x100); Require(compressedCell && compressedCell->editorId == "CompressedCell" && compressedCell->raw && compressedCell->raw->compressed);
        const auto* navm = parsed.FindWinning(0x101); Require(navm && navm->navm && navm->navm->supported && navm->navm->vertexCount == 1 && navm->navm->triangleCount == 1);
        const auto unknown = std::find_if(navm->raw->subrecords.begin(), navm->raw->subrecords.end(), [](const auto& sub) { return sub.type == "ZZZZ"; }); Require(unknown != navm->raw->subrecords.end() && unknown->data == std::vector<std::uint8_t>({ 0xA1, 0xB2, 0xC3 }));
        const auto* baseRecord = parsed.FindWinning(0x102); Require(baseRecord && baseRecord->modelPath == "meshes/test.nif" && baseRecord->raw);

        Require(baseRecord->origins.front().modelRadius == 5.0F && baseRecord->origins.front().hasModel);
        WritePlugin(root / "Bad.esp", {}, false, { { "NAVM", 0x200, { 'N', 'V', 'N', 'M', 0x40, 0x00 } }, { "CELL", 0x201, { 'E', 'D', 'I', 'D', 0x08, 0x00, 'x' } }, { "NAVM", 0x202, NavmPayload(99) } });
        const auto malformed = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { "Bad.esp" } });
        Require(std::any_of(malformed.diagnostics.begin(), malformed.diagnostics.end(), [](const auto& d) { return d.kind == navmesh::skyrim::offline::DiagnosticKind::MalformedInput; }));
        Require(std::any_of(malformed.diagnostics.begin(), malformed.diagnostics.end(), [](const auto& d) { return d.kind == navmesh::skyrim::offline::DiagnosticKind::UnsupportedVersion; }));

        WritePlugin(root / "BadCompression.esp", {}, false, { { "CELL", 0x203, { 0, 0, 0, 0 } } });
        std::fstream badCompressedFile(root / "BadCompression.esp", std::ios::in | std::ios::out | std::ios::binary); badCompressedFile.seekp(24 + 8); badCompressedFile.write(reinterpret_cast<const char*>(&compressedFlag), sizeof(compressedFlag)); badCompressedFile.close();
        const auto badCompression = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { "BadCompression.esp" } });
        Require(std::any_of(badCompression.diagnostics.begin(), badCompression.diagnostics.end(), [](const auto& d) { return d.kind == navmesh::skyrim::offline::DiagnosticKind::DecompressionFailure; }));
    }
    void TestNavmeshOverrideWriter()
    {
        const auto root = std::filesystem::temp_directory_path() / "navmesh-override-writer-test";
        std::filesystem::remove_all(root); std::filesystem::create_directories(root);
        auto navmPayload = NavmPayload();
        navmPayload.resize(navmPayload.size() - 9); // remove the fixture's unrelated ZZZZ subrecord
        PutU32(navmPayload, 1); // one authored external connection
        PutU32(navmPayload, 0); PutU32(navmPayload, 0x201); PutU16(navmPayload, 0);
        PutU32(navmPayload, 1); // one authored door triangle
        PutU16(navmPayload, 0); PutU32(navmPayload, 0); PutU32(navmPayload, 0x300);
        PutU32(navmPayload, 1); PutU16(navmPayload, 0); // one authored cover triangle
        PutU32(navmPayload, 1); // one spatial grid segment
        for (int i{}; i < 8; ++i) PutFloat(navmPayload, 0.0F);
        PutU32(navmPayload, 1); PutU16(navmPayload, 0);
        // The NVNM subrecord's short length includes the trailing link and grid sections.
        const auto nvnmSize = static_cast<std::uint16_t>(navmPayload[4] | navmPayload[5] << 8);
        const auto completeSize = static_cast<std::uint16_t>(nvnmSize + 76);
        navmPayload[4] = static_cast<std::uint8_t>(completeSize);
        navmPayload[5] = static_cast<std::uint8_t>(completeSize >> 8);
        std::vector<std::uint8_t> navmeshGroup; PutRecord(navmeshGroup, "NAVM", 0x200, navmPayload); PutRecord(navmeshGroup, "NAVM", 0x201, navmPayload);
        std::vector<std::uint8_t> cellChildren; PutGroup(cellChildren, 0x100, 10, navmeshGroup);
        std::vector<std::uint8_t> cells; PutRecord(cells, "CELL", 0x100, CellPayload("WriterCell")); PutGroup(cells, 0x100, 6, cellChildren);
        std::vector<std::uint8_t> plugin; PutRecord(plugin, "TES4", 0, {}, 1); PutGroup(plugin, 0x4c4c4543, 0, cells);
        const auto sourcePath = root / "Source.esm";
        { std::ofstream file(sourcePath, std::ios::binary); file.write(reinterpret_cast<const char*>(plugin.data()), static_cast<std::streamsize>(plugin.size())); }
        const auto resolved = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { sourcePath } });
        Require(resolved.cells.size() == 1 && resolved.cells.front().navMeshes.size() == 2);
        navmesh::core::CandidateNavMesh candidate;
        candidate.mesh.vertices = { { 0, 0, 0 }, { 128, 0, 0 }, { 0, 128, 0 } };
        navmesh::core::NavPolygon triangle; triangle.vertices = { 0, 1, 2 }; triangle.neighbors.fill(std::numeric_limits<std::uint32_t>::max());
        candidate.mesh.polygons.push_back(triangle);
        const auto eslFlagged = [](const std::filesystem::path& path) {
            std::ifstream file(path, std::ios::binary);
            std::array<std::uint8_t, 12> header{};
            file.read(reinterpret_cast<char*>(header.data()), static_cast<std::streamsize>(header.size()));
            Require(static_cast<bool>(file));
            return header[8] == 0 && (header[9] & 0x02U) != 0;
        };
        std::filesystem::path target;
        std::string error;
        const bool written = navmesh::skyrim::offline::WriteNavmeshOverride(root / "master-output", { sourcePath }, resolved, resolved.cells.front(), candidate, target, error);
        if (!written) std::fprintf(stderr, "Writer rejected fixture: %s\n", error.c_str());
        Require(written && target.extension() == ".esp" && eslFlagged(target) && std::filesystem::exists(target));
        const auto patched = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { sourcePath, target } });
        Require(patched.cells.size() == 1 && patched.cells.front().navMeshes.size() == 2);
        const auto& active = patched.cells.front().navMeshes;
        Require(std::count_if(active.begin(), active.end(), [](const auto& mesh) { return mesh.vertices.size() == 3 && mesh.polygons.size() == 1 && mesh.vertices[1].x == 128; }) == 1);
        Require(std::count_if(active.begin(), active.end(), [](const auto& mesh) { return mesh.vertices.empty() && mesh.polygons.empty(); }) == 1);
        const auto* parent = patched.FindWinning(0x100);
        Require(parent && parent->editorId == "WriterCell" && parent->winning.plugin == "Source.esm");
        Require(!navmesh::skyrim::offline::WriteNavmeshOverride(root / "master-output", { sourcePath }, resolved, resolved.cells.front(), candidate, target, error));
        std::ifstream unchanged(sourcePath, std::ios::binary); const std::vector<std::uint8_t> sourceBytes((std::istreambuf_iterator<char>(unchanged)), {});
        Require(sourceBytes == plugin);
        auto localized = plugin; localized[8] |= 0x80;
        const auto localizedPath = root / "Localized.esm";
        { std::ofstream file(localizedPath, std::ios::binary); file.write(reinterpret_cast<const char*>(localized.data()), static_cast<std::streamsize>(localized.size())); }
        const auto localizedOrder = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { localizedPath } });
        Require(navmesh::skyrim::offline::WriteNavmeshOverride(root / "localized-output", { localizedPath }, localizedOrder,
            localizedOrder.cells.front(), candidate, target, error) && target.extension() == ".esp" && eslFlagged(target));
        auto regular = plugin; regular[8] = 0;
        const auto regularPath = root / "Regular.esp";
        { std::ofstream file(regularPath, std::ios::binary); file.write(reinterpret_cast<const char*>(regular.data()), static_cast<std::streamsize>(regular.size())); }
        const auto regularOrder = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { regularPath } });
        Require(navmesh::skyrim::offline::WriteNavmeshOverride(root / "regular-output", { regularPath }, regularOrder,
            regularOrder.cells.front(), candidate, target, error) && target.extension() == ".esp" && eslFlagged(target));

        std::vector<std::uint8_t> patchHeader;
        PutText(patchHeader, "MAST", { 'S', 'o', 'u', 'r', 'c', 'e', '.', 'e', 's', 'm', 0 });
        std::vector<std::uint8_t> patchNavmesh; PutRecord(patchNavmesh, "NAVM", 0x01000300, navmPayload);
        std::vector<std::uint8_t> patchChildren; PutGroup(patchChildren, 0x100, 10, patchNavmesh);
        std::vector<std::uint8_t> patchCells; PutGroup(patchCells, 0x100, 6, patchChildren);
        std::vector<std::uint8_t> patchPlugin; PutRecord(patchPlugin, "TES4", 0, patchHeader);
        PutGroup(patchPlugin, 0x4c4c4543, 0, patchCells);
        const auto patchPath = root / "Addition.esp";
        { std::ofstream file(patchPath, std::ios::binary); file.write(reinterpret_cast<const char*>(patchPlugin.data()), static_cast<std::streamsize>(patchPlugin.size())); }
        const auto mixedOrder = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { sourcePath, patchPath } });
        Require(mixedOrder.cells.size() == 1 && mixedOrder.cells.front().navMeshes.size() == 3);
        Require(navmesh::skyrim::offline::WriteNavmeshOverride(root / "mixed-output", { sourcePath, patchPath }, mixedOrder,
            mixedOrder.cells.front(), candidate, target, error) && target.extension() == ".esp" && eslFlagged(target));
        const auto mixedPatched = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = { sourcePath, patchPath, target } });
        Require(mixedPatched.cells.size() == 1 && mixedPatched.cells.front().navMeshes.size() == 3);
        Require(mixedPatched.FindWinning(0x01000300) && mixedPatched.FindWinning(0x01000300)->winning.plugin == "generated-navmesh.esp");

        const auto batchOrder = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory=root, .plugins={sourcePath,regularPath} });
        Require(batchOrder.cells.size() == 2);
        auto secondCandidate = candidate; secondCandidate.mesh.vertices[1].x = 256;
        Require(navmesh::skyrim::offline::WriteNavmeshOverrides(root / "batch-output",{sourcePath,regularPath},batchOrder,
            {{&batchOrder.cells[0],&candidate},{&batchOrder.cells[1],&secondCandidate}},target,error));
        const auto batchPatched = navmesh::skyrim::offline::ResolveLoadOrder({.dataDirectory=root,.plugins={sourcePath,regularPath,target}});
        Require(batchPatched.cells.size() == 2 && batchPatched.FindWinning(0x200)->winning.plugin == "generated-navmesh.esp"
            && batchPatched.FindWinning(0x01000200)->winning.plugin == "generated-navmesh.esp");
        Require(batchPatched.cells[1].navMeshes[0].vertices[1].x == 256);
        Require(!navmesh::skyrim::offline::WriteNavmeshOverrides(root / "duplicate-batch",{sourcePath},resolved,
            {{&resolved.cells[0],&candidate},{&resolved.cells[0],&candidate}},target,error));

        std::vector<std::string> manyMasters;
        std::vector<std::filesystem::path> manyPaths;
        for (int index{}; index < 253; ++index) {
            const auto name = "Dependency" + std::to_string(index) + ".esm";
            WritePlugin(root / name, {}, false, {});
            manyMasters.push_back(name);
            manyPaths.push_back(root / name);
        }
        std::vector<std::uint8_t> manyHeader;
        for (const auto& name : manyMasters) {
            std::vector<std::uint8_t> bytes(name.begin(), name.end()); bytes.push_back(0);
            PutText(manyHeader, "MAST", bytes);
        }
        std::vector<std::uint8_t> manyNavm; PutRecord(manyNavm, "NAVM", 0xFD000200, navmPayload);
        std::vector<std::uint8_t> manyChildren; PutGroup(manyChildren, 0xFD000100, 10, manyNavm);
        std::vector<std::uint8_t> manyCells; PutRecord(manyCells, "CELL", 0xFD000100, CellPayload("ManyMastersCell")); PutGroup(manyCells, 0xFD000100, 6, manyChildren);
        std::vector<std::uint8_t> manyPlugin; PutRecord(manyPlugin, "TES4", 0, manyHeader); PutGroup(manyPlugin, 0x4c4c4543, 0, manyCells);
        const auto manyPath = root / "ManyMasters.esp";
        { std::ofstream file(manyPath, std::ios::binary); file.write(reinterpret_cast<const char*>(manyPlugin.data()), static_cast<std::streamsize>(manyPlugin.size())); }
        manyPaths.push_back(manyPath);
        const auto manyOrder = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory = root, .plugins = manyPaths });
        Require(manyOrder.cells.size() == 1 && manyOrder.cells.front().navMeshes.size() == 1);
        Require(navmesh::skyrim::offline::WriteNavmeshOverride(root / "full-slot-output", manyPaths, manyOrder,
            manyOrder.cells.front(), candidate, target, error) && target.extension() == ".esp" && !eslFlagged(target));
    }
    void TestReciprocalCellTransitions()
    {
        using namespace navmesh::core;
        const auto root = std::filesystem::temp_directory_path() / "navmesh-transition-test";
        std::filesystem::remove_all(root); std::filesystem::create_directories(root);
        const auto navm = [](std::array<Vec3,3> vertices, std::int16_t cellX) {
            std::vector<std::uint8_t> body;
            PutU32(body,12); PutU32(body,0); PutU32(body,0x400);
            PutU16(body,static_cast<std::uint16_t>(-4)); PutU16(body,static_cast<std::uint16_t>(cellX));
            PutU32(body,3);
            for (const auto& vertex : vertices) { PutFloat(body,vertex.x); PutFloat(body,vertex.y); PutFloat(body,vertex.z); }
            PutU32(body,1);
            for (std::uint16_t i{}; i < 3; ++i) PutU16(body,i);
            for (int i{}; i < 3; ++i) PutU16(body,0xffff);
            PutU16(body,0); PutU16(body,0);
            PutU32(body,0); PutU32(body,0); PutU32(body,0);
            PutU32(body,1);
            for (int i{}; i < 8; ++i) PutFloat(body,0);
            PutU32(body,1); PutU16(body,0);
            std::vector<std::uint8_t> payload; PutText(payload,"NVNM",body); return payload;
        };
        const auto selectedVertices = std::array<Vec3,3>{{{53120,-15936,0},{53232,-16000,0},{53232,-15872,0}}};
        const auto neighborVertices = std::array<Vec3,3>{{{53248,-15872,0},{53248,-16000,0},{53376,-15936,0}}};
        std::vector<std::uint8_t> selectedNavm; PutRecord(selectedNavm,"NAVM",0x200,navm(selectedVertices,12));
        std::vector<std::uint8_t> neighborNavm; PutRecord(neighborNavm,"NAVM",0x201,navm(neighborVertices,13));
        std::vector<std::uint8_t> selectedRefPayload, doorBase; PutU32(doorBase,0x310); PutText(selectedRefPayload,"NAME",doorBase);
        std::vector<std::uint8_t> selectedRef; PutRecord(selectedRef,"REFR",0x300,selectedRefPayload);
        std::vector<std::uint8_t> selectedCell;
        PutRecord(selectedCell,"CELL",0x100,CellPayload("TransitionCell",true));
        std::vector<std::uint8_t> selectedChildren;
        PutGroup(selectedChildren,0x100,9,selectedRef);
        PutGroup(selectedChildren,0x100,10,selectedNavm);
        PutGroup(selectedCell,0x100,6,selectedChildren);
        std::vector<std::uint8_t> neighborCellPayload = CellPayload("AdjacentCell",true);
        const std::array<std::uint8_t,4> xclcTag{'X','C','L','C'};
        const auto xclc = std::search(neighborCellPayload.begin(),neighborCellPayload.end(),xclcTag.begin(),xclcTag.end());
        Require(xclc != neighborCellPayload.end());
        neighborCellPayload[static_cast<std::size_t>(xclc-neighborCellPayload.begin())+6] = 13;
        std::vector<std::uint8_t> neighborCell;
        PutRecord(neighborCell,"CELL",0x101,neighborCellPayload);
        std::vector<std::uint8_t> neighborChildren; PutGroup(neighborChildren,0x101,10,neighborNavm);
        PutGroup(neighborCell,0x101,6,neighborChildren);
        std::vector<std::uint8_t> world;
        PutRecord(world,"WRLD",0x400,{});
        std::vector<std::uint8_t> cells = selectedCell;
        cells.insert(cells.end(),neighborCell.begin(),neighborCell.end());
        PutGroup(world,0x400,1,cells);
        std::vector<std::uint8_t> plugin;
        PutRecord(plugin,"TES4",0,{});
        PutRecord(plugin,"DOOR",0x310,{});
        PutGroup(plugin,0x444C5257,0,world);
        const auto source = root / "Transitions.esm";
        { std::ofstream file(source,std::ios::binary); file.write(reinterpret_cast<const char*>(plugin.data()),static_cast<std::streamsize>(plugin.size())); }
        const auto resolved = navmesh::skyrim::offline::ResolveLoadOrder({ .dataDirectory=root, .plugins={source} });
        const auto selected = std::find_if(resolved.cells.begin(),resolved.cells.end(),[](const auto& cell) { return cell.id == 0x100; });
        const auto adjacent = std::find_if(resolved.cells.begin(),resolved.cells.end(),[](const auto& cell) { return cell.id == 0x101; });
        Require(selected != resolved.cells.end() && adjacent != resolved.cells.end());
        Require(selected->navMeshes.size() == 1 && adjacent->navMeshes.size() == 1);
        CandidateNavMesh candidate;
        candidate.mesh.vertices.assign(selectedVertices.begin(),selectedVertices.end());
        NavPolygon triangle{.vertices={0,1,2},.neighbors={0xffffffffU,0xffffffffU,0xffffffffU}};
        candidate.mesh.polygons.push_back(triangle);
        candidate.polygonSourceTriangles = {0};
        candidate.polygonContributingTriangles = {{0}};
        candidate.regions.push_back({.id=0,.polygons={0}});
        candidate.exits.push_back({.referenceId=0x300,.position={53180,-15936,0},.region=0,.polygon=0});
        std::filesystem::path output; std::string error;
        Require(navmesh::skyrim::offline::WriteNavmeshOverride(root / "door-only",{source},resolved,*selected,candidate,output,error));
        const auto doorOnly = navmesh::skyrim::offline::ResolveLoadOrder({.dataDirectory=root,.plugins={source,output}});
        const auto* doorOnlyNavm = doorOnly.FindWinning(0x200);
        Require(doorOnlyNavm && doorOnlyNavm->navm && doorOnlyNavm->navm->triangleCount == 1);
        const AABB bounds{.min={12*4096.0F,-4*4096.0F,-100},.max={13*4096.0F,-3*4096.0F,100}};
        Require(StitchCandidateBorders(candidate,bounds,adjacent->navMeshes) == 1);
        Require(candidate.topology.valid && candidate.borderLinks[0].polygon == 2 && candidate.mesh.polygons.size() == 3);
        auto unlinked = candidate;
        unlinked.borderLinks.clear(); unlinked.exits.clear();
        Require(!navmesh::skyrim::offline::WriteNavmeshOverride(root / "unlinked",{source},resolved,*selected,unlinked,output,error));
        Require(error.find("border-reaching") != std::string::npos);
        const auto written = navmesh::skyrim::offline::WriteNavmeshOverride(root / "output",{source},resolved,*selected,candidate,output,error);
        if (!written) std::fprintf(stderr,"Transition writer rejected fixture: %s\n",error.c_str());
        Require(written);
        const auto patched = navmesh::skyrim::offline::ResolveLoadOrder({.dataDirectory=root,.plugins={source,output}});
        const auto* selectedNavmRecord = patched.FindWinning(0x200);
        const auto* adjacentNavmRecord = patched.FindWinning(0x201);
        Require(selectedNavmRecord && adjacentNavmRecord && selectedNavmRecord->navm && adjacentNavmRecord->navm);
        Require(selectedNavmRecord->navm->triangleCount == 3 && adjacentNavmRecord->navm->triangleCount == 1);
        const auto read32 = [](const auto& bytes,std::size_t offset) {
            return static_cast<std::uint32_t>(bytes[offset]) | static_cast<std::uint32_t>(bytes[offset+1]) << 8
                | static_cast<std::uint32_t>(bytes[offset+2]) << 16 | static_cast<std::uint32_t>(bytes[offset+3]) << 24;
        };
        const auto& selectedData = selectedNavmRecord->raw->decodedPayload;
        const auto selectedTail = static_cast<std::size_t>(selectedNavmRecord->navm->trailingData.offset);
        Require(read32(selectedData,selectedTail) == 1 && read32(selectedData,selectedTail+14) == 1);
        Require(read32(selectedData,selectedTail+8) == 0x201 && read32(selectedData,selectedTail+20) == 0xE48B73F3U);
        const auto& neighborData = adjacentNavmRecord->raw->decodedPayload;
        const auto neighborTail = static_cast<std::size_t>(adjacentNavmRecord->navm->trailingData.offset);
        Require(read32(neighborData,neighborTail) == 1 && read32(neighborData,neighborTail+8) == 0x200);
        CandidateNavMesh adjacentCandidate;
        adjacentCandidate.mesh = adjacent->navMeshes.front();
        adjacentCandidate.mesh.polygons.front().neighbors.fill(std::numeric_limits<std::uint32_t>::max());
        adjacentCandidate.borderLinks.push_back({0,0,0x200,2,1});
        const bool batchWritten = navmesh::skyrim::offline::WriteNavmeshOverrides(root / "batch",{source},resolved,
            {{&*selected,&candidate},{&*adjacent,&adjacentCandidate}},output,error);
        if (!batchWritten) std::fprintf(stderr,"Batch transition rejected: %s\n",error.c_str());
        Require(batchWritten);
        const auto batch = navmesh::skyrim::offline::ResolveLoadOrder({.dataDirectory=root,.plugins={source,output}});
        Require(batch.FindWinning(0x200)->navm->triangleCount == 3 && batch.FindWinning(0x201)->navm->triangleCount == 1);
        adjacentCandidate.borderLinks.clear();
        Require(!navmesh::skyrim::offline::WriteNavmeshOverrides(root / "missing-reciprocal",{source},resolved,
            {{&*selected,&candidate},{&*adjacent,&adjacentCandidate}},output,error));
    }
    void TestAdjacentBorderBridges()
    {
        using namespace navmesh::core;
        CandidateNavMesh candidate;
        candidate.mesh.vertices = {{4000,50,0},{4080,0,0},{4080,100,0},
            {4000,150,0},{4080,200,0},{4000,350,0},{4080,300,0},{4080,400,0}};
        candidate.mesh.polygons = {
            {.vertices={0,1,2},.neighbors={0xffffffffU,0xffffffffU,0xffffffffU}},
            {.vertices={3,2,4},.neighbors={0xffffffffU,0xffffffffU,0xffffffffU}},
            {.vertices={5,6,7},.neighbors={0xffffffffU,0xffffffffU,0xffffffffU}}};
        candidate.polygonSourceTriangles = {0,1,2};
        candidate.polygonContributingTriangles = {{0},{1},{2}};
        candidate.regions.push_back({.id=0,.polygons={0,1},.reachesBorder=true});
        candidate.regions.push_back({.id=1,.polygons={2},.reachesBorder=true});
        NavMesh neighbor;
        neighbor.id = 0x201;
        neighbor.vertices = {{4096,100,0},{4096,0,0},{4200,50,0},
            {4096,200,0},{4200,150,0}};
        neighbor.polygons = {
            {.vertices={0,1,2},.neighbors={0xffffffffU,0xffffffffU,0xffffffffU}},
            {.vertices={3,0,4},.neighbors={0xffffffffU,0xffffffffU,0xffffffffU}}};
        const AABB bounds{.min={0,0,-100},.max={4096,4096,100}};
        Require(StitchCandidateBorders(candidate,bounds,{neighbor}) == 2);
        Require(candidate.topology.valid && candidate.borderLinks.size() == 2
            && candidate.mesh.polygons.size() == 6 && candidate.regions.size() == 1
            && candidate.statistics.rejectedUnreachable == 1);
        Require(candidate.mesh.polygons[3].neighbors[2] == 4
            && candidate.mesh.polygons[4].neighbors[0] == 3);
    }
    void TestAffectedCells()
    {
        using namespace navmesh::skyrim::offline;
        using navmesh::core::Cell;
        ResolvedLoadOrder order;
        order.plugins = { "Baseline.esm", "Move.esp", "Models.esm", "Later.esp" };
        order.cells = { { .id=1, .exteriorCoordinates=std::array<std::int32_t,2>{0,0} },
            { .id=2, .exteriorCoordinates=std::array<std::int32_t,2>{1,0} },
            { .id=3, .exteriorCoordinates=std::array<std::int32_t,2>{-1,0} },
            { .id=4, .exteriorCoordinates=std::array<std::int32_t,2>{10,0} },
            { .id=5, .isInterior=true },
            { .id=6, .exteriorCoordinates=std::array<std::int32_t,2>{0,0} },
            { .id=7, .exteriorCoordinates=std::array<std::int32_t,2>{50,0} } };
        for (const auto& cell : order.cells) {
            ResolvedRecord record{ .type="CELL", .formId=cell.id, .exteriorCoordinates=cell.exteriorCoordinates };
            if (!cell.isInterior) record.worldspaceFormId = cell.id == 6 ? 200 : 100;
            record.winning = { "Baseline.esm",cell.id,{}, {}, record.worldspaceFormId };
            record.origins = {record.winning}; order.records.push_back(record);
        }
        ResolvedRecord moved{ .type="REFR", .formId=60, .cellFormId=4, .worldspaceFormId=100 };
        moved.origins = { {"Baseline.esm",60,{},1,100,50,navmesh::core::Vec3{100,100,0}},
            {"Move.esp",60,{},4,100,50,navmesh::core::Vec3{41000,100,0}},
            {"Later.esp",60,{},4,100,50,navmesh::core::Vec3{41100,100,0}} };
        moved.winning = moved.origins.back(); order.records.push_back(moved);
        ResolvedRecord base{ .type="STAT", .formId=50 };
        base.origins = { {"Baseline.esm",50,{}}, {"Models.esm",50,{}} }; base.winning=base.origins.back(); order.records.push_back(base);
        // An untouched REFR uses the edited base, and an interior stays independent
        // from any exterior coordinates with the same numeric values.
        ResolvedRecord interior{ .type="REFR", .formId=61, .cellFormId=5 };
        interior.winning = {"Baseline.esm",61,{},5,{},50}; interior.origins={interior.winning}; order.records.push_back(interior);
        // A persistent REFR belongs to a distant parent but physically occupies
        // the negative-coordinate cell. Its deleted winner still affects that site.
        ResolvedRecord persistent{ .type="REFR", .formId=62, .cellFormId=7, .worldspaceFormId=100 };
        persistent.winning = {"Move.esp",62,{},7,100,51,navmesh::core::Vec3{-1,100,0}};
        persistent.origins = {persistent.winning}; order.records.push_back(persistent);
        order.cells.back().references.push_back({ .id=62, .baseObjectId=51, .modelPath="Persistent.nif", .position={-1,100,0}, .deleted=true });
        const CellImpactIndex index(order);
        const auto ids = [](const auto& cells) { std::set<std::uint32_t> result; for (const auto* cell : cells) result.insert(cell->id); return result; };
        Require(ids(index.AffectedCells("mOvE.EsP",0)) == std::set<std::uint32_t>({1,2,3,4}));
        Require(ids(index.AffectedCells("Models.esm",0)) == std::set<std::uint32_t>({1,2,3,4,5}));
        Require(ids(index.AffectedCells("",0)) == std::set<std::uint32_t>({1,2,3,4,5}));
        Require(index.GeometryCell(order.cells[2]).references.size() == 1);
        Require(index.GeometryCell(order.cells.back()).references.empty());
        Require(ids(index.Neighbors(order.cells.front(),1)) == std::set<std::uint32_t>({1,2,3}));
        Require(index.Neighbors(order.cells[4],10).size() == 1);
        bool rejected{}; try { (void)index.AffectedCells("Inactive.esp",0); } catch (const std::invalid_argument&) { rejected=true; } Require(rejected);
        Require(ids(index.AffectedCells("Later.esp",0,{"meshes/persistent.nif"})) == std::set<std::uint32_t>({1,2,3,4}));
        order.plugins.push_back("Large.esp");
        ResolvedRecord largeBase{ .type="STAT", .formId=70, .modelPath="Large.nif" };
        largeBase.winning = {"Baseline.esm",70,{}};
        largeBase.winning.modelRadius = 30000.0F; largeBase.winning.hasModel = true;
        largeBase.origins = {largeBase.winning}; order.records.push_back(largeBase);
        ResolvedRecord largeRef{ .type="REFR", .formId=63, .cellFormId=1, .worldspaceFormId=100 };
        largeRef.winning = {"Large.esp",63,{},1,100,70,navmesh::core::Vec3{100,100,0}};
        largeRef.winning.scale = 2; largeRef.origins = {largeRef.winning}; order.records.push_back(largeRef);
        order.cells.front().references.push_back({.id=63,.baseObjectId=70,.modelPath="Large.nif",.position={100,100,0},.scale=2});
        const CellImpactIndex bounded(order);
        Require(ids(bounded.AffectedCells("Large.esp",0)) == std::set<std::uint32_t>({1,2,3,4}));
        Require(ids(bounded.GeometryNeighbors(order.cells[3],1)).contains(1));
        order.records[order.records.size()-2].origins.front().modelRadius.reset();
        const CellImpactIndex unbounded(order);
        Require(ids(unbounded.AffectedCells("Large.esp",0)) == std::set<std::uint32_t>({1,2,3,4,7}));
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
        auto profile = NavigationProfile{};
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
    void TestRecastSceneGeneration()
    {
        using namespace navmesh::core;
        Scene scene;
        scene.geometrySources.push_back({ .sourceType = GeometrySourceType::Terrain, .confidence = 1.0F,
            .reference = { "Fixture.esm", 0x100, "LAND" } });
        scene.mesh.vertices = {{0,0,0},{512,0,0},{512,512,0},{0,512,0}};
        scene.mesh.triangles = {{{0,1,2}},{{0,2,3}}};
        scene.triangleProvenance = {{0,0,{}},{0,1,{}}};
        const NavigationProfile profile{};
        Require(profile.name == "human" && profile.stepHeight == 28.0F);
        const CandidateGenerator& generator = RecastCandidateGenerator{};
        const AABB cellBounds{.min={0,0,-100},.max={1024,1024,100}};
        const auto generated = generator.Generate(scene,profile,cellBounds,{});
        Require(generated.mesh.polygons.size() == 2);
        Require(generated.topology.valid);
        Require(generated.statistics.eligibleTriangles == 2);
        Scene crossing = scene;
        crossing.mesh.vertices = {{-100,-100,0},{600,-100,0},{600,600,0},{-100,600,0}};
        const AABB clipBounds{.min={0,0,-100},.max={512,512,100}};
        const auto clipped = generator.Generate(crossing,profile,clipBounds,{});
        Require(clipped.topology.valid && !clipped.mesh.polygons.empty());
        for (const auto vertex : clipped.mesh.vertices)
            Require(vertex.x >= clipBounds.min.x && vertex.x <= clipBounds.max.x
                && vertex.y >= clipBounds.min.y && vertex.y <= clipBounds.max.y);
        Scene withIsland = scene;
        withIsland.mesh.vertices.insert(withIsland.mesh.vertices.end(),
            {{768,200,0},{832,200,0},{832,264,0},{768,264,0}});
        withIsland.mesh.triangles.push_back({{4,5,6}});
        withIsland.mesh.triangles.push_back({{4,6,7}});
        withIsland.triangleProvenance.push_back({0,2,{}});
        withIsland.triangleProvenance.push_back({0,3,{}});
        NavigationProfile permissive = profile;
        permissive.minimumRegionArea = 64.0F;
        const auto withOrphan = generator.Generate(withIsland,permissive,cellBounds,{});
        const auto withoutIsland = generator.Generate(withIsland,profile,cellBounds,{});
        Require(withOrphan.topology.valid && withOrphan.regions.size() == 1);
        Require(withOrphan.mesh.polygons.size() == generated.mesh.polygons.size());
        Require(withOrphan.statistics.rejectedUnreachable > 0);
        Require(withoutIsland.topology.valid && withoutIsland.regions.size() == 1);
        Require(withoutIsland.mesh.polygons.size() == generated.mesh.polygons.size());
        const auto doorLinked = generator.Generate(withIsland,permissive,cellBounds,
            {{.referenceId=0x200,.position={800,232,0}}});
        Require(doorLinked.topology.valid && doorLinked.regions.size() == 2);
        Require(doorLinked.exits[0].region && !doorLinked.regions[*doorLinked.exits[0].region].reachesBorder);
        Require(doorLinked.mesh.polygons.size() > generated.mesh.polygons.size());
        const auto noAnchor = generator.Generate(scene,profile,std::nullopt,{});
        Require(noAnchor.mesh.polygons.empty() && noAnchor.regions.empty()
            && noAnchor.statistics.rejectedUnreachable > 0 && noAnchor.topology.valid);
        const auto root = std::filesystem::temp_directory_path() / "navmesh-recast-scene-test";
        std::filesystem::create_directories(root);
        Require(WriteCandidateJson(root / "candidate.json",generated,scene,"{}"));
        std::ifstream candidateJson(root / "candidate.json",std::ios::binary);
        const std::string candidateBytes(std::istreambuf_iterator<char>(candidateJson),{});
        Require(candidateBytes.contains("\"profile\": {\"name\":\"human\",\"agent_radius\":"));
        SceneExportOptions options{ .layers = {SceneLayer::CandidateNavmesh},
            .candidateNavmesh = &doorLinked.mesh, .candidateEntrances = &doorLinked.exits };
        Cell cell{ .id = 0x100, .editorId = "Fixture" };
        const navmesh::reproducibility::ExportMetadata metadata{ .selectedCell = &cell };
        const auto exported = WriteCombinedGlb(root / "scene.glb",scene,{}, {},metadata,options);
        Require(exported.triangles == doorLinked.mesh.polygons.size()+4);
        std::ifstream entranceGlb(root / "scene.glb",std::ios::binary);
        const std::string entranceBytes(std::istreambuf_iterator<char>(entranceGlb),{});
        Require(entranceBytes.contains("Entrance (orange)") && entranceBytes.contains("Entrance 00000200"));

        Scene stairs;
        stairs.geometrySources = scene.geometrySources;
        for (std::uint32_t step = 0; step < 16; ++step) {
            const auto base = static_cast<std::uint32_t>(stairs.mesh.vertices.size());
            const float x = static_cast<float>(step) * 12.0F;
            const float z = static_cast<float>(step) * 24.0F;
            stairs.mesh.vertices.insert(stairs.mesh.vertices.end(),
                {{x,0,z},{x+12,0,z},{x+12,96,z},{x,96,z}});
            stairs.mesh.triangles.push_back({{base,base+1,base+2}});
            stairs.mesh.triangles.push_back({{base,base+2,base+3}});
            stairs.triangleProvenance.push_back({0,stairs.triangleProvenance.size(),{}});
            stairs.triangleProvenance.push_back({0,stairs.triangleProvenance.size(),{}});
        }
        const auto stepped = generator.Generate(stairs,profile,std::nullopt,
            {{.referenceId=0x201,.position={12,48,0}}});
        Require(stepped.topology.valid && stepped.regions.size() == 1);
        Require(stepped.mesh.polygons.size() == 2);
        const auto [low,high] = std::minmax_element(stepped.mesh.vertices.begin(),stepped.mesh.vertices.end(),
            [](Vec3 a, Vec3 b){ return a.z < b.z; });
        Require(high->z-low->z > 200.0F);

        Scene winding;
        winding.geometrySources = scene.geometrySources;
        for (std::uint32_t segment = 0; segment < 48; ++segment) {
            const auto x = static_cast<float>(segment) * 32.0F;
            const auto y = static_cast<float>((segment * 13) % 7) * 8.0F;
            const auto nextY = static_cast<float>(((segment + 1) * 13) % 7) * 8.0F;
            const auto base = static_cast<std::uint32_t>(winding.mesh.vertices.size());
            winding.mesh.vertices.insert(winding.mesh.vertices.end(),
                {{x,y,0},{x+32,nextY,0},{x+32,nextY+128,0},{x,y+128,0}});
            winding.mesh.triangles.push_back({{base,base+1,base+2}});
            winding.mesh.triangles.push_back({{base,base+2,base+3}});
            winding.triangleProvenance.push_back({0,winding.triangleProvenance.size(),{}});
            winding.triangleProvenance.push_back({0,winding.triangleProvenance.size(),{}});
        }
        const auto ribbon = generator.Generate(winding,profile,std::nullopt,
            {{.referenceId=0x202,.position={16,64,0}}});
        float worstQuality = 1.0F;
        for (const auto& face : ribbon.mesh.polygons) {
            const auto a = ribbon.mesh.vertices[face.vertices[0]];
            const auto b = ribbon.mesh.vertices[face.vertices[1]];
            const auto c = ribbon.mesh.vertices[face.vertices[2]];
            const auto sq = [](Vec3 p,Vec3 q){ const auto dx=p.x-q.x,dy=p.y-q.y; return dx*dx+dy*dy; };
            const auto cross = std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x));
            worstQuality = std::min(worstQuality,cross*3.464F/(sq(a,b)+sq(b,c)+sq(c,a)));
        }
        Require(ribbon.topology.valid && ribbon.regions.size() == 1);
        Require(ribbon.mesh.polygons.size() <= 40);
        Require(worstQuality > 0.3F);
    }
    bool TestLocalStairCollision(const std::filesystem::path& geometryObj, const std::string& groupName)
    {
        using namespace navmesh::core;
        Scene scene;
        scene.geometrySources.push_back({ .sourceType = GeometrySourceType::Collision, .confidence = 1.0F,
            .reference = { "Local.esm", 0x200, "REFR" } });
        std::ifstream input(geometryObj);
        Require(input.is_open());
        bool selected{};
        std::string line;
        while (std::getline(input,line)) {
            if (line.starts_with("v ")) {
                Vec3 point;
                std::istringstream values(line.substr(2));
                Require(static_cast<bool>(values >> point.x >> point.y >> point.z));
                scene.mesh.vertices.push_back(point);
            } else if (line.starts_with("g ")) {
                selected = line == "g " + groupName;
            } else if (selected && line.starts_with("f ")) {
                unsigned a{},b{},c{};
                std::istringstream values(line.substr(2));
                Require(static_cast<bool>(values >> a >> b >> c) && a && b && c);
                scene.mesh.triangles.push_back({{a-1,b-1,c-1}});
                scene.triangleProvenance.push_back({0,scene.triangleProvenance.size(),{}});
            }
        }
        if (scene.mesh.triangles.empty()) return false;
        Require(scene.mesh.triangles.size() == 308);
        const NavigationProfile current{};
        const RecastCandidateGenerator generator;
        const auto isolated = generator.Generate(scene,current,std::nullopt,
            {{.referenceId=0x200,.position=scene.mesh.vertices.front()}});
        Require(isolated.topology.valid && isolated.regions.size() == 1);

        // Include the extracted scene envelope to reproduce its coarser grid.
        {
            const auto base = static_cast<std::uint32_t>(scene.mesh.vertices.size());
            scene.mesh.vertices.insert(scene.mesh.vertices.end(),
                {{20135.55F,-55045.95F,-1835.7F},{20139.55F,-55045.95F,-1835.7F},{20135.55F,-55041.95F,-1835.7F},
                 {37318.176F,-32027.32F,10821.2F},{37322.176F,-32027.32F,10821.2F},{37318.176F,-32023.32F,10821.2F}});
            scene.mesh.triangles.push_back({{base,base+1,base+2}});
            scene.triangleProvenance.push_back({0,scene.triangleProvenance.size(),{}});
            scene.mesh.triangles.push_back({{base+3,base+4,base+5}});
            scene.triangleProvenance.push_back({0,scene.triangleProvenance.size(),{}});
        }
        const auto candidate = generator.Generate(scene,current,std::nullopt,
            {{.referenceId=0x200,.position=scene.mesh.vertices.front()}});
        Require(candidate.topology.valid && candidate.regions.size() == 1);
        const auto& vertices = candidate.mesh.vertices;
        const auto [low,high] = std::minmax_element(vertices.begin(),vertices.end(),
            [](Vec3 a, Vec3 b){ return a.z < b.z; });
        Require(high->z - low->z > 220.0F);
        return true;
    }
    void TestLocalStairs(const std::filesystem::path& geometryObj)
    {
        const std::array groups{
            "REF_0002A749_BASE_0004D7C2", "REF_0004D7E2_BASE_0004D7C2", "REF_000538A7_BASE_0004D7C2"};
        int found{};
        for (const auto* group : groups) found += TestLocalStairCollision(geometryObj,group);
        Require(found > 0);
    }
}

int main(int argc, char** argv)
{
    if (argc > 1 && std::string_view(argv[1]) == "--scene-only") { TestCombinedColorLayeredGlb(); return 0; }
    if (argc > 1 && std::string_view(argv[1]) == "--candidate-only") { TestCandidateGeneration(); return 0; }
    if (argc > 1 && std::string_view(argv[1]) == "--recast-only") { TestRecastSceneGeneration(); return 0; }
    if (argc > 2 && std::string_view(argv[1]) == "--local-stair-obj") { TestLocalStairs(argv[2]); return 0; }
    if (argc > 1 && std::string_view(argv[1]) == "--batch-only") {
        TestAffectedCells(); TestNavmeshOverrideWriter(); TestReciprocalCellTransitions(); return 0;
    }
    TestAffectedCells();
    TestResolvedLoadOrder();
    TestCellOverrideAcrossPlugins();
    TestMo2ProfileImport();
    TestLossAwareRecordReader();
    TestNavmeshOverrideWriter();
    TestReciprocalCellTransitions();
    TestAdjacentBorderBridges();
    TestExteriorLandTerrain();
    TestExportMetadata();
    TestOptionalLocalGameData();
    TestBstTriShapeExtraction();
    TestPackedCollisionPreferredOverRenderFixture();
    TestSceneTransforms();
    TestCombinedColorLayeredGlb();
    TestCandidateGeneration();
    TestRecastSceneGeneration();
    if (std::filesystem::exists("output/riverwood03-recast-repro/geometry.obj"))
        TestLocalStairs("output/riverwood03-recast-repro/geometry.obj");
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
