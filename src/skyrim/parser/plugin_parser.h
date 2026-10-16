#pragma once

#include "core/world/types.h"
#include "core/io/shared_bytes.h"

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <vector>
#include <unordered_map>

/// Skyrim plugin input, asset extraction, and guarded navigation export.
namespace navmesh::skyrim
{
    /** Decode the external and door tables from supported NVNM trailing bytes for inspection.
     * @param trailing Bytes immediately after the owning mesh's triangle array.
     * @param mesh Geometry whose edge flags consume the external table; connection arrays are replaced.
     * @return False for truncated tables or invalid consuming indices, leaving connection arrays empty.
     * FormIDs retain plugin-local indices; load-order assembly resolves them before scene export.
     * Unconsumed external entries and later cover/grid sections remain uninterpreted.
     */
    [[nodiscard]] bool DecodeNavmeshConnections(std::span<const std::uint8_t> trailing, core::NavMesh &mesh);

    enum class DiagnosticKind
    {
        MissingMaster,
        Cycle,
        DuplicatePlugin,
        UnresolvedFormId,
        UnsupportedRecord,
        InvalidPlugin,
        MalformedInput,
        DecompressionFailure,
        UnsupportedVersion
    };
    struct Diagnostic
    {
        DiagnosticKind kind{};
        std::string plugin;
        std::string message;
    };
    struct ByteRange
    {
        std::uint64_t offset{};
        std::uint64_t size{};
    };
    /// Loss-aware subrecord with encoded bytes and file ranges retained for
    /// future round trips.
    struct Subrecord
    {
        std::string type;
        ByteRange encodedRange;
        ByteRange dataRange;
        core::SharedBytes encodedBytes;
        core::SharedBytes data;
        bool extendedSize{};
    };
    /// Indexed record with original file payload and decoded subrecord payload.
    struct PluginRecord
    {
        std::string type;
        std::uint32_t flags{};
        ByteRange headerRange;
        ByteRange filePayloadRange;
        bool compressed{};
        core::SharedBytes filePayload;
        core::SharedBytes decodedPayload;
        std::vector<Subrecord> subrecords;
    };
    /// Provenance and compact placement evidence for each version of a record.
    struct RecordOrigin
    {
        std::string plugin;
        std::uint32_t formId{};
        ByteRange headerRange;
        /// Resolved owner identities at this version, including moved/deleted references.
        std::optional<std::uint32_t> cellFormId, worldspaceFormId, baseFormId;
        /// Reference position in Skyrim world units, when DATA is present.
        std::optional<core::Vec3> position;
        /// Rotation-independent model bound radius from OBND, in model-local units.
        std::optional<float> modelRadius;
        /// Base-object bounds in model-local Skyrim units; absent without a valid three-dimensional OBND.
        std::optional<core::AABB> localBounds;
        /// Resolved XTEL destination reference; absent without a supported teleport subrecord.
        std::optional<std::uint32_t> teleportDestinationId;
        /// XTEL standing position in the destination's Skyrim world space; absent for invalid coordinates.
        std::optional<core::Vec3> teleportDestinationPosition;
        /// Whether this version names a model asset, including models removed by overrides.
        bool hasModel{};
        /// Placed-reference scale for this version; multiplies model-local distances.
        float scale{1.0F};
        /// Whether this version changes navigation inputs relative to its predecessor.
        /// New records and manually supplied origins conservatively default to true.
        bool navigationChanged{true};
        /// Reference DATA angles in radians, using the Skyrim placement convention.
        core::Vec3 rotation;
        /// Model asset named by this version; empty means no reference-local or base model.
        std::string modelPath;
        /// Header state at this version; deleted/initially disabled placements supply no collision.
        bool deleted{}, initiallyDisabled{};
        /// CELL water enable flag and finite explicit CELL/WRLD water height in Skyrim world Z units.
        bool hasWater{};
        std::optional<float> waterHeight;
        /// CELLs without an explicit height inherit their worldspace's water surface.
        bool usesWorldWater{true};
        /// Resolved parent worldspace and terrain/water inheritance at this WRLD version.
        std::optional<std::uint32_t> parentWorldFormId;
        /// Parent LAND at matching exterior coordinates can supply terrain when the child has none.
        bool inheritsParentLand{};
        bool inheritsParentWater{};
        /// Shared decoded VHGT bytes for this LAND version, including inherited height evidence.
        core::SharedBytes terrainHeights;
    };
    struct NavmLayout
    {
        std::uint32_t version{};
        std::uint16_t declaredBodySize{};
        std::uint32_t vertexCount{};
        std::uint32_t triangleCount{};
        ByteRange header;
        ByteRange vertices;
        ByteRange triangles;
        ByteRange trailingData;
        bool supported{};
    };
    /// A placed door's XNDP association; mesh identity uses the containing reader's FormID space.
    struct NavmeshDoorAssociation
    {
        std::uint32_t navmeshId{};
        /// Zero-based triangle index in the associated NAVM.
        std::uint16_t triangle{};
    };
    struct ResolvedRecord
    {
        std::string type;
        std::uint32_t formId{};
        std::string editorId;
        std::string name;
        RecordOrigin winning;
        std::vector<RecordOrigin> origins;
        std::optional<std::uint32_t> cellFormId;
        std::optional<std::uint32_t> worldspaceFormId;
        std::vector<std::uint32_t> referencedFormIds;
        std::optional<std::array<std::int32_t, 2>> exteriorCoordinates;
        bool persistent{};
        bool temporary{};
        /// Source GRUP headers from outermost to innermost, retained for override placement.
        std::vector<std::array<std::uint8_t, 24>> groupHeaders;
        // These model the roadmap record subset without discarding unrecognised fields.
        std::optional<PluginRecord> raw;
        std::optional<std::string> modelPath;
        std::optional<std::array<float, 6>> transform;
        std::optional<float> referenceScale;
        std::vector<std::uint32_t> linkedFormIds;
        /// Resolved possible LVLI item bases; level/chance restrictions are conservatively ignored for runtime swaps.
        std::vector<std::uint32_t> leveledBaseIds;
        std::optional<NavmLayout> navm;
        /// Winning XNDP, with a resolved mesh identity after load-order assembly.
        std::optional<NavmeshDoorAssociation> navmeshDoor;
    };
    /// Original plugin placement and effective runtime alternatives used for collision-impact selection.
    struct RuntimePlacementChange
    {
        /// Owning CELL and worldspace remain plugin identities; position is Skyrim world space.
        RecordOrigin owner;
        core::Reference original;
        std::vector<core::Reference> alternatives;
    };

    /// File identity captured before plugin resolution, used to reject concurrent source edits during export.
    struct PluginInputSnapshot
    {
        std::filesystem::path path;
        /// Source byte length and filesystem modification time, without changing source metadata.
        std::uintmax_t size{};
        std::filesystem::file_time_type modified;
    };
    struct ResolvedLoadOrder
    {
        std::vector<std::string> plugins;
        std::vector<ResolvedRecord> records;
        std::vector<core::Cell> cells;
        std::vector<Diagnostic> diagnostics;
        /// Physical input states in load order; absent for synthetic readers without backing files.
        std::vector<PluginInputSnapshot> inputSnapshots;
        /// FormID-to-record offsets built by ResolveLoadOrder; records must retain their order.
        std::unordered_map<std::uint32_t, std::size_t> recordIndex;
        /// Ordered winning LAND record offsets by resolved CELL identity; records must retain their order.
        std::unordered_map<std::uint32_t, std::vector<std::size_t>> landIndex;
        /// Lowercase active filename to global full/light FormID prefix, including empty plugins.
        std::unordered_map<std::string, std::uint32_t> pluginFormPrefixes;
        /// Runtime configuration changes kept separate from the real plugin override history.
        std::vector<RuntimePlacementChange> runtimePlacements;
        /// Winning LAND records in source order, with a scan fallback for manually assembled fixtures.
        [[nodiscard]] std::vector<const ResolvedRecord *> LandRecords(std::uint32_t cellId) const;
        /// Look up a resolved FormID without copying payloads; returns nullptr when
        /// absent. Uses the resolver index or scans manually constructed fixtures.
        [[nodiscard]] const ResolvedRecord *FindWinning(std::uint32_t formId) const;
    };
    using LoadOrderProgressCallback = std::function<void(std::size_t completedPlugins, std::size_t totalPlugins,
                                                         const std::filesystem::path &currentPlugin)>;
    struct LoadOrderInput
    {
        std::filesystem::path dataDirectory;
        std::vector<std::filesystem::path> plugins;
        bool indexReferencesAndNavmeshes{true};
        LoadOrderProgressCallback progress{};
    };

    /// Parser boundary for replacing the direct reader without changing load-order callers.
    class IPluginReader
    {
      public:
        virtual ~IPluginReader() = default;
        /** Read only master order and light-plugin identity before record resolution.
         * The default implementation delegates to Read without reference indexing and discards records.
         * @return False on header failure; appends input diagnostics without changing game files.
         */
        virtual bool ReadMetadata(const std::filesystem::path &path, std::vector<std::string> &masters, bool &isLight,
                                  std::vector<Diagnostic> &diagnostics) const;
        /// Read owned indexed records and master names; false on unusable input, with diagnostics.
        virtual bool Read(const std::filesystem::path &path, std::vector<ResolvedRecord> &records,
                          std::vector<std::string> &masters, bool &isLight, std::vector<Diagnostic> &diagnostics,
                          bool includeReferencesAndNavmeshes = true) const = 0;
    };
    class DirectPluginReader final : public IPluginReader
    {
      public:
        /// Read just the TES4 range; validates declared length and master subrecords before returning metadata.
        bool ReadMetadata(const std::filesystem::path &path, std::vector<std::string> &masters, bool &isLight,
                          std::vector<Diagnostic> &diagnostics) const override;
        /// Read loss-aware records with immutable shared payload ranges and explicit failures.
        bool Read(const std::filesystem::path &path, std::vector<ResolvedRecord> &records,
                  std::vector<std::string> &masters, bool &isLight, std::vector<Diagnostic> &diagnostics,
                  bool includeReferencesAndNavmeshes = true) const override;
    };
    [[nodiscard]] std::vector<std::filesystem::path> ReadLoadOrderManifest(const std::filesystem::path &manifest);
    /// Resolve winning records and cells from an ordered plugin list.
    /// @param input Data directory and plugins in load order, plus optional progress callback.
    /// @param reader Plugin reader implementation; defaults to the direct reader.
    /// @return Winning records, cells, and explicit diagnostics for unsupported input.
    /// References include valid model-local base bounds and reciprocal teleport arrival positions;
    /// Missing or deleted return links supply no arrival anchor; initially disabled quest doors retain theirs.
    [[nodiscard]] ResolvedLoadOrder ResolveLoadOrder(const LoadOrderInput &input,
                                                     const IPluginReader &reader = DirectPluginReader{});

    /** Check that resolved input plugins still have their captured file sizes and modification times.
     * @param resolved Snapshot used for geometry extraction and record resolution.
     * @param error Receives the changed or unavailable source filename on failure.
     * @return True when all physical inputs still match; performs no writes.
     */
    [[nodiscard]] bool LoadOrderSnapshotMatches(const ResolvedLoadOrder &resolved, std::string &error);

    /** Identify cells with winning NAVM records without requiring decodable geometry.
     * @param resolved Snapshot indexed with references and navmeshes enabled.
     * @return Resolved CELL FormIDs owning any NAVM, including empty, unsupported,
     * or deleted records. Tombstones protect authored identities from recreation.
     */
    [[nodiscard]] std::set<std::uint32_t> CellsWithExistingNavmesh(const ResolvedLoadOrder &resolved);

    [[nodiscard]] std::vector<core::Cell> ListCells(const std::filesystem::path &pluginPath);

    /// Load a cell selected by form ID, editor ID, or exterior coordinates.
    /// @return The resolved cell, or no value when the selection cannot be found.
    [[nodiscard]] std::optional<core::Cell> LoadCell(const std::filesystem::path &pluginPath,
                                                     const std::string &targetCell = {},
                                                     const std::string &targetWorldspace = {},
                                                     const std::optional<std::int32_t> &cellX = std::nullopt,
                                                     const std::optional<std::int32_t> &cellY = std::nullopt,
                                                     const std::optional<std::uint32_t> &cellFormId = std::nullopt,
                                                     const std::string &targetEditorId = {});
} // namespace navmesh::skyrim
