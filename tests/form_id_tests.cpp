#ifdef NDEBUG
#undef NDEBUG
#endif

#include "skyrim/parser/plugin_parser.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <fstream>
#include <tuple>

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
} // namespace

void TestFormIdResolution()
{
    const auto root =
        std::filesystem::temp_directory_path() /
        ("navmesh-form-id-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);
    TestFileLocalIndices(root);
    TestMissingMasterIdentity(root);
    std::filesystem::remove_all(root);
}
