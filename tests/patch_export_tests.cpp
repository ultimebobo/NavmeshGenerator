#ifdef NDEBUG
#undef NDEBUG
#endif

#include "skyrim/parser/patch_export.h"
#include "skyrim/parser/plugin_writer.h"

#include <cassert>

namespace
{
    using Bytes = std::vector<std::uint8_t>;

    void Put32(Bytes &bytes, std::uint32_t value)
    {
        for (int shift = 0; shift < 32; shift += 8)
        {
            bytes.push_back(static_cast<std::uint8_t>(value >> shift));
        }
    }

    void PutSubrecord(Bytes &bytes, const char *type, const Bytes &data)
    {
        bytes.insert(bytes.end(), type, type + 4);
        bytes.push_back(static_cast<std::uint8_t>(data.size()));
        bytes.push_back(static_cast<std::uint8_t>(data.size() >> 8));
        bytes.insert(bytes.end(), data.begin(), data.end());
    }

    void CheckLoadOrderExportFailure(const std::filesystem::path &root)
    {
        using namespace navmesh::skyrim;
        ResolvedLoadOrder input;
        input.diagnostics = {{DiagnosticKind::UnsupportedVersion, "SupportedInput.esm", "Preserved NAVM version"},
                             {DiagnosticKind::MalformedInput, "Unrelated.esp",
                              "STAT 0x01000800 at file offset 66: subrecord EDID exceeds decoded payload"},
                             {DiagnosticKind::UnresolvedFormId, "MissingDependency.esp", "Missing master identity"}};
        std::filesystem::path written = "must-be-cleared.esp";
        std::string error;
        // The input guard must report its evidence before touching any replacement or output file.
        assert(!WriteNavmeshOverrides(root, {}, input, {{}}, written, error));
        assert(written.empty());
        assert(error.contains("Unrelated.esp: STAT 0x01000800 at file offset 66"));
        assert(error.contains("subrecord EDID exceeds decoded payload"));
        assert(error.contains("MissingDependency.esp: Missing master identity"));
        assert(error.contains("outside the selected cells"));
        assert(error.contains("mod manager") && error.contains("bug report"));
        assert(!error.contains("SupportedInput.esm"));

        for (int index = 0; index < 10; ++index)
        {
            input.diagnostics.push_back({DiagnosticKind::MalformedInput, "More.esp", "Additional input failure"});
        }
        assert(!WriteNavmeshOverrides(root, {}, input, {{}}, written, error));
        assert(error.contains("Unrelated.esp: STAT 0x01000800"));
        assert(error.contains("Additional blocking diagnostics:"));
        assert(error.size() < 1500);

        input.diagnostics.resize(1);
        assert(!WriteNavmeshOverrides(root, {}, input, {{}}, written, error));
        assert(error == "Batch contains an invalid or duplicate CELL.");
    }
} // namespace

void TestNavigationPatchExport()
{
    using namespace navmesh::skyrim;
    const auto root = std::filesystem::current_path() / "build" / "patch-export-tests";
    std::filesystem::create_directories(root);
    CheckLoadOrderExportFailure(root);
    // A single retained base-object array can require more owners than any file can encode.
    // No grouping strategy may discard those references or publish part of that record.
    ResolvedLoadOrder input;
    input.plugins.push_back("Base.esm");
    input.pluginFormPrefixes.emplace("base.esm", 0);
    Bytes objects;
    for (std::uint32_t index = 0; index < 255; ++index)
    {
        const auto name = "resource" + std::to_string(index) + ".esl";
        input.plugins.push_back(name);
        const auto prefix = 0xfe000000U | index << 12;
        input.pluginFormPrefixes.emplace(name, prefix);
        Put32(objects, prefix | 0x800);
    }
    Bytes body;
    Put32(body, 12);
    Put32(body, 0xA5E9A03C);
    Put32(body, 0x400);
    Put32(body, 0);
    for (int array = 0; array < 4; ++array)
    {
        Put32(body, 0);
    }
    Bytes payload;
    PutSubrecord(payload, "NVNM", body);
    PutSubrecord(payload, "ONAM", objects);
    Bytes record{'N', 'A', 'V', 'M'};
    Put32(record, static_cast<std::uint32_t>(payload.size()));
    Put32(record, 0);
    Put32(record, 0x100);
    Put32(record, 0);
    Put32(record, 44);
    record.insert(record.end(), payload.begin(), payload.end());
    std::vector<std::filesystem::path> written;
    std::string error;
    assert(!detail::WriteNavigationPatchSet(root, "generated-navmesh.esp", input, record, {}, {}, 0x800, {}, true,
                                            written, error));
    assert(error.find("single navigation record exceeds") != std::string::npos);
    assert(written.empty());
    assert(!std::filesystem::exists(root / "generated-navmesh.esp"));
    assert(!std::filesystem::exists(root / "plugin-manifest.json"));
    assert(!std::filesystem::exists(root / ".navmesh-export"));
}
