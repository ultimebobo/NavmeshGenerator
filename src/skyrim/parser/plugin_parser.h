#pragma once

#include "core/world/types.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace navmesh::skyrim::offline
{
    enum class DiagnosticKind { MissingMaster, Cycle, DuplicatePlugin, UnresolvedFormId, UnsupportedRecord, InvalidPlugin, MalformedInput, DecompressionFailure, UnsupportedVersion };
    struct Diagnostic { DiagnosticKind kind{}; std::string plugin; std::string message; };
    struct ByteRange { std::uint64_t offset{}; std::uint64_t size{}; };
    // Raw bytes and ranges are retained for every indexed record.  For a compressed
    // record, decodedPayload is the subrecord stream and filePayload is the exact
    // on-disk compressed stream, allowing a future writer to preserve either form.
    struct Subrecord { std::string type; ByteRange encodedRange; ByteRange dataRange; std::vector<std::uint8_t> encodedBytes; std::vector<std::uint8_t> data; bool extendedSize{}; };
    struct PluginRecord { std::string type; std::uint32_t flags{}; ByteRange headerRange; ByteRange filePayloadRange; bool compressed{}; std::vector<std::uint8_t> filePayload; std::vector<std::uint8_t> decodedPayload; std::vector<Subrecord> subrecords; };
    struct RecordOrigin { std::string plugin; std::uint32_t formId{}; ByteRange headerRange; };
    struct NavmLayout {
        std::uint32_t version{}; std::uint16_t declaredBodySize{}; std::uint32_t vertexCount{}; std::uint32_t triangleCount{};
        ByteRange header; ByteRange vertices; ByteRange triangles; ByteRange trailingData; bool supported{};
    };
    struct ResolvedRecord {
        std::string type; std::uint32_t formId{}; std::string editorId; std::string name; RecordOrigin winning; std::vector<RecordOrigin> origins;
        std::optional<std::uint32_t> cellFormId; std::optional<std::uint32_t> worldspaceFormId;
        std::vector<std::uint32_t> referencedFormIds;
        std::optional<std::array<std::int32_t, 2>> exteriorCoordinates; bool persistent{}; bool temporary{};
        // These model the roadmap record subset without discarding unrecognised fields.
        std::optional<PluginRecord> raw;
        std::optional<std::string> modelPath; std::optional<std::array<float, 6>> transform; std::optional<float> referenceScale;
        std::vector<std::uint32_t> linkedFormIds; std::optional<NavmLayout> navm;
    };
    struct ResolvedLoadOrder {
        std::vector<std::string> plugins; std::vector<ResolvedRecord> records; std::vector<core::Cell> cells; std::vector<Diagnostic> diagnostics;
        [[nodiscard]] const ResolvedRecord* FindWinning(std::uint32_t formId) const;
    };
    using LoadOrderProgressCallback = std::function<void(std::size_t completedPlugins, std::size_t totalPlugins, const std::filesystem::path& currentPlugin)>;
    struct LoadOrderInput { std::filesystem::path dataDirectory; std::vector<std::filesystem::path> plugins; bool indexReferencesAndNavmeshes{ true }; LoadOrderProgressCallback progress{}; };

    // Parser boundary: a mature parser can replace DirectPluginReader without changing load-order callers.
    class IPluginReader {
    public:
        virtual ~IPluginReader() = default;
        virtual bool Read(const std::filesystem::path& path, std::vector<ResolvedRecord>& records, std::vector<std::string>& masters,
            bool& isLight, std::vector<Diagnostic>& diagnostics, bool includeReferencesAndNavmeshes = true) const = 0;
    };
    class DirectPluginReader final : public IPluginReader {
    public:
        bool Read(const std::filesystem::path& path, std::vector<ResolvedRecord>& records, std::vector<std::string>& masters,
            bool& isLight, std::vector<Diagnostic>& diagnostics, bool includeReferencesAndNavmeshes = true) const override;
    };
    [[nodiscard]] std::vector<std::filesystem::path> ReadLoadOrderManifest(const std::filesystem::path& manifest);
    [[nodiscard]] ResolvedLoadOrder ResolveLoadOrder(const LoadOrderInput& input, const IPluginReader& reader = DirectPluginReader{});

    [[nodiscard]] std::vector<core::Cell> ListCells(
        const std::filesystem::path& pluginPath);

    [[nodiscard]] std::optional<core::Cell> LoadCell(
        const std::filesystem::path& pluginPath,
        const std::string& targetCell = {},
        const std::string& targetWorldspace = {},
        const std::optional<std::int32_t>& cellX = std::nullopt,
        const std::optional<std::int32_t>& cellY = std::nullopt,
        const std::optional<std::uint32_t>& cellFormId = std::nullopt,
        const std::string& targetEditorId = {});
}
