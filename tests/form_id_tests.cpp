#ifdef NDEBUG
#undef NDEBUG
#endif

#include "skyrim/parser/plugin_parser.h"
#include "skyrim/parser/text_encoding.h"
#include "core/io/file_path.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <zlib.h>

namespace
{
    using Bytes = std::vector<std::uint8_t>;
    using namespace navmesh::skyrim;

    void Put32(Bytes &bytes, std::uint32_t value)
    {
        for (unsigned shift{}; shift < 32; shift += 8)
        {
            bytes.push_back(static_cast<std::uint8_t>(value >> shift));
        }
    }

    void Subrecord(Bytes &bytes, const char *type, const Bytes &payload)
    {
        bytes.insert(bytes.end(), type, type + 4);
        bytes.push_back(static_cast<std::uint8_t>(payload.size()));
        bytes.push_back(static_cast<std::uint8_t>(payload.size() >> 8));
        bytes.insert(bytes.end(), payload.begin(), payload.end());
    }

    void Record(Bytes &bytes, const char *type, std::uint32_t id, const Bytes &payload, bool light = false)
    {
        bytes.insert(bytes.end(), type, type + 4);
        Put32(bytes, static_cast<std::uint32_t>(payload.size()));
        Put32(bytes, light ? 0x200U : 0U);
        Put32(bytes, id);
        bytes.insert(bytes.end(), 8, 0);
        bytes.insert(bytes.end(), payload.begin(), payload.end());
    }

    Bytes BaseReference(std::uint32_t id)
    {
        Bytes value;
        Put32(value, id);
        Bytes payload;
        Subrecord(payload, "NAME", value);
        return payload;
    }

    void WritePlugin(const std::filesystem::path &path, const std::vector<std::string> &masters, bool light,
                     const std::vector<std::tuple<std::string, std::uint32_t, Bytes>> &records)
    {
        Bytes header;
        for (const auto &master : masters)
        {
            Bytes name(master.begin(), master.end());
            name.push_back(0);
            Subrecord(header, "MAST", name);
        }
        Bytes bytes;
        Record(bytes, "TES4", 0, header, light);
        for (const auto &[type, id, payload] : records)
        {
            Record(bytes, type.c_str(), id, payload);
        }
        std::ofstream output(path, std::ios::binary);
        output.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        assert(output.good());
    }

    void TestCompressedRecordBoundaries(const std::filesystem::path &root)
    {
        const auto compressed = [](const Bytes &payload)
        {
            Bytes result;
            Put32(result, static_cast<std::uint32_t>(payload.size()));
            auto length = compressBound(static_cast<uLong>(payload.size()));
            result.resize(4 + length);
            assert(compress(result.data() + 4, &length, payload.data(), static_cast<uLong>(payload.size())) == Z_OK);
            result.resize(4 + length);
            return result;
        };
        Bytes bytes;
        Record(bytes, "TES4", 0, {});
        const auto append = [&](std::uint32_t id, const Bytes &payload, bool deleted = false)
        {
            Bytes record;
            Record(record, "CELL", id, payload);
            record[10] = 4; // Compressed record flag.
            record[8] = deleted ? 0x20 : 0;
            bytes.insert(bytes.end(), record.begin(), record.end());
        };
        append(0x800, {0, 0, 0, 0}, true);
        append(0x801, compressed({}));
        Bytes payload;
        Subrecord(payload, "EDID", {'P', 'a', 'y', 'l', 'o', 'a', 'd', 0});
        append(0x802, compressed(payload));
        append(0x803, {0, 0, 0});
        append(0x804, {1, 0, 0, 0});
        append(0x805, {0xFF, 0xFF, 0xFF, 0xFF});
        auto truncated = compressed(payload);
        truncated.pop_back();
        append(0x806, truncated);
        auto wrongSize = compressed(payload);
        ++wrongSize.front();
        append(0x807, wrongSize);
        append(0x808, {});
        const auto path = root / "Compression.esp";
        std::ofstream output(path, std::ios::binary);
        output.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        output.close();

        for (const bool includeReferences : {false, true})
        {
            const auto resolved = ResolveLoadOrder(
                {.dataDirectory = root, .plugins = {path}, .indexReferencesAndNavmeshes = includeReferences});
            assert(resolved.records.size() == 9);
            assert(resolved.diagnostics.size() == 6);
            for (const auto id : {0x800U, 0x801U})
            {
                const auto *record = resolved.FindWinning(id);
                assert(record && record->raw && record->raw->compressed);
                assert(record->raw->decodedPayload.empty() && record->raw->subrecords.empty());
                assert(!record->raw->filePayload.empty());
            }
            assert(resolved.FindWinning(0x800)->winning.deleted);
            assert(resolved.FindWinning(0x802)->editorId == "Payload");
            for (std::uint32_t id = 0x803; id <= 0x808; ++id)
            {
                std::ostringstream identity;
                identity << "CELL 0x00000" << std::uppercase << std::hex << id << " at file offset ";
                const auto failure =
                    std::find_if(resolved.diagnostics.begin(), resolved.diagnostics.end(), [&](const auto &diagnostic)
                                 { return diagnostic.message.starts_with(identity.str()); });
                assert(failure != resolved.diagnostics.end());
                assert(failure->plugin == "Compression.esp" && failure->kind == DiagnosticKind::DecompressionFailure);
                if (id == 0x803 || id == 0x808)
                {
                    assert(failure->message.find("lacks its uncompressed-size prefix") != std::string::npos);
                }
            }
        }
    }

    void TestFileLocalIndices(const std::filesystem::path &root)
    {
        // Empty plugins consume global slots. Master order is deliberately different
        // from load order, and both canonical and excess self indices name the same base.
        WritePlugin(root / "Base.esm", {}, false, {{"STAT", 0xA00, {}}});
        WritePlugin(root / "Empty.esp", {}, false, {});
        WritePlugin(root / "EmptyLight.esl", {}, true, {});
        WritePlugin(root / "Light.esl", {"Base.esm"}, true,
                    {{"STAT", 0x7F000900, {}}, {"REFR", 0x01000800, BaseReference(0x04000900)}});
        WritePlugin(root / "Source.esp", {"Light.esl", "Base.esm"}, false,
                    {{"STAT", 0x02000900, {}},
                     {"REFR", 0x04000801, BaseReference(0x04000900)},
                     {"REFR", 0x02000802, BaseReference(0x00000900)},
                     {"REFR", 0x02000803, BaseReference(0x01000A00)},
                     {"REFR", 0x02000804, BaseReference(0)},
                     {"REFR", 0x02000805, BaseReference(0x02000900)}});
        const LoadOrderInput input{.dataDirectory = root,
                                   .plugins = {"Base.esm", "Empty.esp", "EmptyLight.esl", "Light.esl", "Source.esp"}};
        const auto resolved = ResolveLoadOrder(input);
        assert(resolved.diagnostics.empty());
        const auto expectBase = [&](std::uint32_t reference, std::uint32_t base)
        {
            const auto *record = resolved.FindWinning(reference);
            assert(record && record->referencedFormIds == std::vector<std::uint32_t>{base});
            assert(record->winning.baseFormId == base);
            if (base)
            {
                assert(resolved.FindWinning(base));
            }
        };
        expectBase(0xFE001800, 0xFE001900);
        expectBase(0x02000801, 0x02000900);
        expectBase(0x02000802, 0xFE001900);
        expectBase(0x02000803, 0x00000A00);
        expectBase(0x02000804, 0);
        expectBase(0x02000805, 0x02000900);
        const auto *source = resolved.FindWinning(0x02000801);
        assert(source->raw && source->raw->subrecords.front().data == Bytes({0, 9, 0, 4}));

        WritePlugin(root / "Override.esp", {"Base.esm", "Source.esp", "Light.esl"}, false,
                    {{"REFR", 0x01000801, BaseReference(0x02000900)}});
        auto overriddenInput = input;
        overriddenInput.plugins.push_back("Override.esp");
        const auto overridden = ResolveLoadOrder(overriddenInput);
        assert(overridden.diagnostics.empty());
        const auto *winner = overridden.FindWinning(0x02000801);
        assert(winner && winner->origins.size() == 2 && winner->winning.baseFormId == 0xFE001900);
        assert(winner->origins.front().baseFormId == 0x02000900);
    }

    void TestMissingMasterIdentity(const std::filesystem::path &root)
    {
        // The missing base's raw identity aliases a real record in the first active
        // plugin. Failed conversion must not expose that unrelated global base.
        WritePlugin(root / "Unrelated.esm", {}, false, {{"STAT", 0x900, {}}});
        WritePlugin(root / "Padding.esp", {}, false, {});
        WritePlugin(root / "MissingDependency.esp", {"Absent.esm"}, false,
                    {{"REFR", 0x01000800, BaseReference(0x00000900)}, {"STAT", 0x00000901, {}}});
        const auto resolved = ResolveLoadOrder(
            {.dataDirectory = root, .plugins = {"Unrelated.esm", "Padding.esp", "MissingDependency.esp"}});
        const auto *reference = resolved.FindWinning(0x02000800);
        assert(reference && reference->referencedFormIds == std::vector<std::uint32_t>{0});
        assert(reference->winning.baseFormId == 0);
        const auto diagnostic = std::find_if(resolved.diagnostics.begin(), resolved.diagnostics.end(),
                                             [](const auto &item)
                                             {
                                                 return item.kind == DiagnosticKind::UnresolvedFormId &&
                                                        item.message.find("referenced") != std::string::npos;
                                             });
        assert(diagnostic != resolved.diagnostics.end() && diagnostic->plugin == "MissingDependency.esp");
        assert(diagnostic->message.find("source-local FormID 0x00000900") != std::string::npos);
        assert(diagnostic->message.find("REFR 0x01000800 (load-order FormID 0x02000800)") != std::string::npos);
        assert(diagnostic->message.find("master 'Absent.esm'") != std::string::npos);
        assert(diagnostic->message.find("absent from the load order") != std::string::npos);
        assert(std::count_if(resolved.diagnostics.begin(), resolved.diagnostics.end(),
                             [](const auto &item) { return item.kind == DiagnosticKind::UnresolvedFormId; }) == 2);
    }

    void TestUnicodeInputs(const std::filesystem::path &root)
    {
        const std::string legacyName = "Master\xB0.esm";
        const std::string utf8Name = "Master\xC2\xB0.esm";
        const auto masterPath = navmesh::core::PathFromUtf8(utf8Name);
        const auto patchPath = std::filesystem::path(u8"Patch_\u65E5\u672C.esp");
        WritePlugin(root / masterPath, {}, false, {{"STAT", 0x900, {}}});
        Bytes model;
        Subrecord(model, "MODL", {'f', 'o', 'l', 'd', 'e', 'r', '\\', 'b', 'a', 'g', 0xB0, '.', 'n', 'i', 'f', 0});
        WritePlugin(root / patchPath, {legacyName}, false, {{"STAT", 0x1000900, model}});
        const auto manifest = root / "unicode-load-order.txt";
        std::ofstream(manifest) << utf8Name << '\n' << navmesh::core::PathToUtf8(patchPath) << '\n';
        const auto resolved = ResolveLoadOrder({.dataDirectory = root, .plugins = ReadLoadOrderManifest(manifest)});
        assert(resolved.diagnostics.empty());
        assert(resolved.plugins == std::vector<std::string>({utf8Name, navmesh::core::PathToUtf8(patchPath)}));
        const auto *record = resolved.FindWinning(0x1000900);
        assert(record && record->modelPath == "folder\\bag\xC2\xB0.nif");
        assert(record->raw->subrecords.front().data ==
               Bytes({'f', 'o', 'l', 'd', 'e', 'r', '\\', 'b', 'a', 'g', 0xB0, '.', 'n', 'i', 'f', 0}));
        assert(EncodePluginText(utf8Name) == legacyName);
        const auto unicode = navmesh::core::PathToUtf8(patchPath);
        assert(DecodePluginText(unicode) == unicode && EncodePluginText(unicode) == unicode);
        const std::string legacySymbols = "\x80\x91\x96\xB0\xE9";
        assert(DecodePluginText(legacySymbols) == "\xE2\x82\xAC\xE2\x80\x98\xE2\x80\x93\xC2\xB0\xC3\xA9");
        assert(EncodePluginText(DecodePluginText(legacySymbols)) == legacySymbols);

        // Unexpected reader exceptions must name the plugin in both resolution stages.
        class FailingReader final : public IPluginReader
        {
          public:
            bool failMetadata{};
            bool ReadMetadata(const std::filesystem::path &, std::vector<std::string> &, bool &,
                              std::vector<Diagnostic> &) const override
            {
                if (failMetadata)
                {
                    throw std::runtime_error("synthetic conversion failure");
                }
                return true;
            }
            bool Read(const std::filesystem::path &, std::vector<ResolvedRecord> &, std::vector<std::string> &, bool &,
                      std::vector<Diagnostic> &, bool) const override
            {
                throw std::runtime_error("synthetic conversion failure");
            }
        } reader;
        for (const bool metadata : {false, true})
        {
            reader.failMetadata = metadata;
            try
            {
                (void)ResolveLoadOrder({.dataDirectory = root, .plugins = {patchPath}}, reader);
                assert(false);
            }
            catch (const std::runtime_error &error)
            {
                const std::string message = error.what();
                assert(message.find(unicode) != std::string::npos);
                assert(message.find(metadata ? "metadata" : "records") != std::string::npos);
                assert(message.find("synthetic conversion failure") != std::string::npos);
            }
        }
    }
} // namespace

void TestFormIdResolution()
{
    const auto root =
        std::filesystem::temp_directory_path() /
        ("navmesh-form-id-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);
    TestFileLocalIndices(root);
    TestMissingMasterIdentity(root);
    TestCompressedRecordBoundaries(root);
    TestUnicodeInputs(root);
    std::filesystem::remove_all(root);
}
