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
#include <unordered_map>

namespace navmesh::skyrim::offline
{
    enum class DiagnosticKind { MissingMaster, Cycle, DuplicatePlugin, UnresolvedFormId, UnsupportedRecord, InvalidPlugin, MalformedInput, DecompressionFailure, UnsupportedVersion };
    struct Diagnostic { DiagnosticKind kind{}; std::string plugin; std::string message; };
    struct ByteRange { std::uint64_t offset{}; std::uint64_t size{}; };
    /// Loss-aware subrecord with encoded bytes and file ranges retained for
    /// future round trips.
    struct Subrecord { std::string type; ByteRange encodedRange; ByteRange dataRange; std::vector<std::uint8_t> encodedBytes; std::vector<std::uint8_t> data; bool extendedSize{}; };
    /// Indexed record with original file payload and decoded subrecord payload.
    struct PluginRecord { std::string type; std::uint32_t flags{}; ByteRange headerRange; ByteRange filePayloadRange; bool compressed{}; std::vector<std::uint8_t> filePayload; std::vector<std::uint8_t> decodedPayload; std::vector<Subrecord> subrecords; };
    /// Provenance and compact placement evidence for each version of a record.
    struct RecordOrigin {
        std::string plugin; std::uint32_t formId{}; ByteRange headerRange;
        /// Resolved owner identities at this version, including moved/deleted references.
        std::optional<std::uint32_t> cellFormId, worldspaceFormId, baseFormId;
        /// Reference position in Skyrim world units, when DATA is present.
        std::optional<core::Vec3> position;
        /// Rotation-independent model bound radius from OBND, in model-local units.
        std::optional<float> modelRadius;
        /// Whether this version names a model asset, including models removed by overrides.
        bool hasModel{};
        /// Placed-reference scale for this version; multiplies model-local distances.
        float scale{1.0F};
    };
    struct NavmLayout {
        std::uint32_t version{}; std::uint16_t declaredBodySize{}; std::uint32_t vertexCount{}; std::uint32_t triangleCount{};
        ByteRange header; ByteRange vertices; ByteRange triangles; ByteRange trailingData; bool supported{};
    };
    struct ResolvedRecord {
        std::string type; std::uint32_t formId{}; std::string editorId; std::string name; RecordOrigin winning; std::vector<RecordOrigin> origins;
        std::optional<std::uint32_t> cellFormId; std::optional<std::uint32_t> worldspaceFormId;
        std::vector<std::uint32_t> referencedFormIds;
        std::optional<std::array<std::int32_t, 2>> exteriorCoordinates; bool persistent{}; bool temporary{};
        /// Source GRUP headers from outermost to innermost, retained for override placement.
        std::vector<std::array<std::uint8_t, 24>> groupHeaders;
        // These model the roadmap record subset without discarding unrecognised fields.
        std::optional<PluginRecord> raw;
        std::optional<std::string> modelPath; std::optional<std::array<float, 6>> transform; std::optional<float> referenceScale;
        std::vector<std::uint32_t> linkedFormIds; std::optional<NavmLayout> navm;
    };
    struct ResolvedLoadOrder {
        std::vector<std::string> plugins; std::vector<ResolvedRecord> records; std::vector<core::Cell> cells; std::vector<Diagnostic> diagnostics;
        /// FormID-to-record offsets built by ResolveLoadOrder; records must retain their order.
        std::unordered_map<std::uint32_t, std::size_t> recordIndex;
        /// Look up a resolved FormID without copying payloads; returns nullptr when
        /// absent. Uses the resolver index or scans manually constructed fixtures.
        [[nodiscard]] const ResolvedRecord* FindWinning(std::uint32_t formId) const;
    };
    using LoadOrderProgressCallback = std::function<void(std::size_t completedPlugins, std::size_t totalPlugins, const std::filesystem::path& currentPlugin)>;
    struct LoadOrderInput { std::filesystem::path dataDirectory; std::vector<std::filesystem::path> plugins; bool indexReferencesAndNavmeshes{ true }; LoadOrderProgressCallback progress{}; };

    /// Parser boundary for replacing the direct reader without changing load-order callers.
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
    /// Resolve winning records and cells from an ordered plugin list.
    /// @param input Data directory and plugins in load order, plus optional progress callback.
    /// @param reader Plugin reader implementation; defaults to the direct reader.
    /// @return Winning records, cells, and explicit diagnostics for unsupported input.
    [[nodiscard]] ResolvedLoadOrder ResolveLoadOrder(const LoadOrderInput& input, const IPluginReader& reader = DirectPluginReader{});

    [[nodiscard]] std::vector<core::Cell> ListCells(
        const std::filesystem::path& pluginPath);

    /// Load a cell selected by form ID, editor ID, or exterior coordinates.
    /// @return The resolved cell, or no value when the selection cannot be found.
    [[nodiscard]] std::optional<core::Cell> LoadCell(
        const std::filesystem::path& pluginPath,
        const std::string& targetCell = {},
        const std::string& targetWorldspace = {},
        const std::optional<std::int32_t>& cellX = std::nullopt,
        const std::optional<std::int32_t>& cellY = std::nullopt,
        const std::optional<std::uint32_t>& cellFormId = std::nullopt,
        const std::string& targetEditorId = {});
}
