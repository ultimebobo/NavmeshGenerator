#ifdef NDEBUG
#undef NDEBUG
#endif

#include "skyrim/extraction/collision_impact.h"

#include <NifFile.hpp>
#include <bhk.hpp>

#include <cassert>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <source_location>
#include <chrono>
#include <fstream>
#include <stdexcept>

namespace
{
    using namespace navmesh;

    void Require(bool condition, const std::source_location location = std::source_location::current())
    {
        if (!condition)
        {
            std::fprintf(stderr, "Collision impact requirement failed at %s:%u\n", location.file_name(),
                         location.line());
            std::exit(EXIT_FAILURE);
        }
    }

    void WriteModel(const std::filesystem::path &path, bool collision, float width = 100)
    {
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        const std::vector<nifly::Vector3> render = {{0, 0, 0}, {100, 0, 0}, {0, 100, 20000}};
        const std::vector<nifly::Triangle> faces = {{0, 1, 2}};
        Require(nif.CreateShapeFromData("Visual", &render, &faces, nullptr));
        if (collision)
        {
            // A tall, narrow solid isolates horizontal collision coverage from its vertical extent and OBND sphere.
            constexpr float units = 69.99125F;
            auto data = std::make_unique<nifly::hkPackedNiTriStripsData>();
            data->numVerts = 3;
            data->compressedVertData = {{0, 0, 0}, {width / units, 0, 0}, {0, 100 / units, 20000 / units}};
            data->keyCount = 1;
            data->triData.resize(1);
            data->triData.front().tri = {0, 1, 2};
            const auto dataId = nif.GetHeader().AddBlock(std::move(data));
            auto shape = std::make_unique<nifly::bhkPackedNiTriStripsShape>();
            shape->dataRef.index = dataId;
            const auto shapeId = nif.GetHeader().AddBlock(std::move(shape));
            auto body = std::make_unique<nifly::bhkRigidBody>();
            body->shapeRef.index = shapeId;
            const auto bodyId = nif.GetHeader().AddBlock(std::move(body));
            auto object = std::make_unique<nifly::bhkCollisionObject>();
            object->bodyRef.index = bodyId;
            object->targetRef.index = nif.GetBlockID(nif.GetRootNode());
            nif.GetRootNode()->collisionRef.index = nif.GetHeader().AddBlock(std::move(object));
        }
        std::filesystem::create_directories(path.parent_path());
        Require(nif.Save(path) == 0);
    }

    skyrim::ResolvedLoadOrder Fixture()
    {
        skyrim::ResolvedLoadOrder order;
        order.plugins = {"Base.esm", "Edit.esp", "Later.esp"};
        for (int x = -1; x < 4; ++x)
        {
            const auto id = static_cast<std::uint32_t>(101 + x);
            order.cells.push_back({.id = id, .exteriorCoordinates = std::array<std::int32_t, 2>{x, 0}});
            skyrim::ResolvedRecord cell{.type = "CELL", .formId = id, .worldspaceFormId = 1};
            cell.winning = {.plugin = "Base.esm", .formId = id, .worldspaceFormId = 1};
            cell.origins = {cell.winning};
            order.records.push_back(std::move(cell));
        }
        skyrim::ResolvedRecord base{.type = "STAT", .formId = 10, .modelPath = "TallSolid.nif"};
        base.winning = {.plugin = "Base.esm",
                        .formId = 10,
                        .modelRadius = 50000.0F,
                        .hasModel = true,
                        .modelPath = "TallSolid.nif"};
        base.origins = {base.winning};
        order.records.push_back(std::move(base));
        return order;
    }

    void WriteHearthModel(const std::filesystem::path &path, float halfLength)
    {
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        const std::vector<nifly::Triangle> faces = {{0, 1, 2}, {0, 2, 3}};
        const std::vector<nifly::Vector3> floor = {
            {-6000, -1024, 0}, {6000, -1024, 0}, {6000, 1024, 0}, {-6000, 1024, 0}};
        const std::vector<nifly::Vector3> bed = {
            {-halfLength, -128, -16}, {halfLength, -128, -16}, {halfLength, 128, -16}, {-halfLength, 128, -16}};
        Require(nif.CreateShapeFromData("Floor", &floor, &faces, nullptr));
        Require(nif.CreateShapeFromData("RecessedBed", &bed, &faces, nullptr));
        std::filesystem::create_directories(path.parent_path());
        Require(nif.Save(path) == 0);
    }

    void PutReference(skyrim::ResolvedLoadOrder &order, std::vector<skyrim::RecordOrigin> origins)
    {
        skyrim::ResolvedRecord reference{.type = "REFR", .formId = 20};
        reference.origins = std::move(origins);
        reference.winning = reference.origins.back();
        order.records.push_back(std::move(reference));
    }

    std::set<std::uint32_t> Select(
        const skyrim::ResolvedLoadOrder &order, const std::filesystem::path &root,
        const skyrim::ModelAssetSources *assets = nullptr, std::set<std::string> changedModels = {},
        skyrim::ImpactSelectionStatistics *outputStatistics = nullptr,
        std::function<bool(const skyrim::GeometryFailure &, const skyrim::RecordOrigin &)> failedGeometry = {})
    {
        const skyrim::CellImpactIndex index(order);
        skyrim::ModelGeometryCache cache;
        skyrim::ImpactSelectionStatistics statistics;
        auto &counts = outputStatistics ? *outputStatistics : statistics;
        const auto targets = skyrim::SelectCollisionAffectedCells(order, index,
                                                                  {.plugin = "Edit.esp",
                                                                   .dataDirectory = root,
                                                                   .cacheDirectory = root / "cache",
                                                                   .previousCacheDirectory = root / "previous-cache",
                                                                   .assets = assets,
                                                                   .changedModels = std::move(changedModels),
                                                                   .failedGeometry = std::move(failedGeometry)},
                                                                  cache, counts);
        std::set<std::uint32_t> ids;
        for (const auto *cell : targets)
        {
            ids.insert(cell->id);
        }
        return ids;
    }
} // namespace

void TestCollisionImpactSelection()
{
    const auto root =
        std::filesystem::temp_directory_path() /
        ("navmesh-collision-impact-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);
    struct Cleanup
    {
        std::filesystem::path root;
        ~Cleanup()
        {
            std::error_code error;
            std::filesystem::remove_all(root, error);
        }
    } cleanup{root};
    WriteModel(root / "meshes/TallSolid.nif", true);
    WriteModel(root / "meshes/VisualOnly.nif", false);
    {
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        auto sphere = std::make_unique<nifly::bhkSphereShape>();
        sphere->radius = 1;
        auto body = std::make_unique<nifly::bhkRigidBody>();
        body->shapeRef.index = nif.GetHeader().AddBlock(std::move(sphere));
        auto collision = std::make_unique<nifly::bhkCollisionObject>();
        collision->bodyRef.index = nif.GetHeader().AddBlock(std::move(body));
        collision->targetRef.index = nif.GetBlockID(nif.GetRootNode());
        nif.GetRootNode()->collisionRef.index = nif.GetHeader().AddBlock(std::move(collision));
        Require(nif.Save(root / "meshes/UnsupportedSolid.nif") == 0);
        core::Cell cell;
        cell.references.push_back({.id = 90, .modelPath = "UnsupportedSolid.nif", .baseRecordType = "STAT"});
        const auto unsupported = skyrim::ExtractGeometry(root, cell, root / "cache", {}, {}, nullptr, nullptr, true);
        Require(!unsupported.failures.empty() && unsupported.failures.front().category == "unsupported");
        cell.references.front().modelPath = "VisualOnly.nif";
        Require(skyrim::ExtractGeometry(root, cell, root / "cache", {}, {}, nullptr, nullptr, true).failures.empty());
    }
    {
        nifly::NifFile fire;
        fire.Create({nifly::V20_2_0_7, 12, 130});
        const std::vector<nifly::Vector3> fuel = {{-64, -64, 0}, {64, -64, 0}, {64, 64, 8}};
        const std::vector<nifly::Triangle> faces = {{0, 1, 2}};
        Require(fire.CreateShapeFromData("Coals", &fuel, &faces, nullptr));
        std::filesystem::create_directories(root / "meshes/effects");
        Require(fire.Save(root / "meshes/effects/fxfirefixture.nif") == 0);
    }

    const skyrim::RecordOrigin added{.plugin = "Edit.esp",
                                     .formId = 20,
                                     .cellFormId = 101,
                                     .worldspaceFormId = 1,
                                     .baseFormId = 10,
                                     .position = core::Vec3{1024, 1024, 0}};
    {
        // Fixed clutter footprints use decoded convex collision for additions, moves and removal.
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        auto shape = std::make_unique<nifly::bhkConvexVerticesShape>();
        const std::vector<nifly::Vector4> vertices = {{-1, -1, 0, 0}, {1, -1, 0, 0}, {1, 1, 0, 0}, {-1, 1, 0, 0},
                                                      {-1, -1, 1, 0}, {1, -1, 1, 0}, {1, 1, 1, 0}, {-1, 1, 1, 0}};
        shape->verts.resize(static_cast<std::uint32_t>(vertices.size()));
        for (std::uint32_t index = 0; index < vertices.size(); ++index)
        {
            shape->verts[index] = vertices[index];
        }
        auto body = std::make_unique<nifly::bhkRigidBody>();
        body->shapeRef.index = nif.GetHeader().AddBlock(std::move(shape));
        auto object = std::make_unique<nifly::bhkCollisionObject>();
        object->bodyRef.index = nif.GetHeader().AddBlock(std::move(body));
        object->targetRef.index = nif.GetBlockID(nif.GetRootNode());
        nif.GetRootNode()->collisionRef.index = nif.GetHeader().AddBlock(std::move(object));
        std::filesystem::create_directories(root / "meshes/clutter");
        Require(nif.Save(root / "meshes/clutter/solid.nif") == 0);
        auto order = Fixture();
        auto &base = order.records.back();
        base.modelPath = "clutter/solid.nif";
        base.winning.modelPath = *base.modelPath;
        base.origins.front().modelPath = *base.modelPath;
        PutReference(order, {added});
        Require(Select(order, root) == std::set<std::uint32_t>{101});
        auto before = added;
        before.plugin = "Base.esm";
        auto moved = added;
        moved.position = core::Vec3{9216, 1024, 0};
        order.records.pop_back();
        PutReference(order, {before, moved});
        Require(Select(order, root) == std::set<std::uint32_t>{101, 103});
        moved.deleted = true;
        order.records.back().winning = moved;
        order.records.back().origins.back() = moved;
        Require(Select(order, root) == std::set<std::uint32_t>{101});
        order.records[order.records.size() - 2].type = "MISC";
        Require(Select(order, root).empty());
        // Authored noncollidable shapes cannot select rebuild targets, regardless of supported shape or base class.
        order.records[order.records.size() - 2].type = "FLOR";
        for (std::uint32_t block = 0; block < nif.GetHeader().GetNumBlocks(); ++block)
        {
            if (auto *nonblockingBody = nif.GetHeader().GetBlock<nifly::bhkRigidBody>(block))
            {
                nonblockingBody->collisionFilter.layer = 15;
                nonblockingBody->collisionFilterCopy.layer = 15;
            }
        }
        Require(nif.Save(root / "meshes/clutter/solid.nif") == 0);
        Require(Select(order, root).empty());
    }
    {
        auto order = Fixture();
        auto &base = order.records.back();
        base.type = "DOOR";
        base.winning.localBounds = core::AABB{{-8, -128, -8}, {8, 128, 256}};
        base.origins.front().localBounds = base.winning.localBounds;
        auto before = added;
        before.plugin = "Base.esm";
        before.teleportDestinationId = 21;
        before.teleportDestinationPosition = core::Vec3{5120, 1024, 0};
        auto moved = before;
        moved.plugin = "Edit.esp";
        moved.position = core::Vec3{9216, 1024, 0};
        PutReference(order, {before, moved});
        skyrim::ResolvedRecord partner{.type = "REFR", .formId = 21};
        partner.winning = before;
        partner.winning.formId = 21;
        partner.winning.cellFormId = 102;
        partner.winning.position = core::Vec3{5120, 1024, 0};
        partner.winning.teleportDestinationId = 20;
        partner.winning.teleportDestinationPosition = core::Vec3{1024, 1024, 0};
        partner.origins = {partner.winning};
        order.records.push_back(partner);
        Require(Select(order, root) == std::set<std::uint32_t>{101, 102, 103});

        // Moving an arrival marker changes the destination entrance even when neither door volume moves.
        order.records[order.records.size() - 2].winning = before;
        order.records[order.records.size() - 2].origins = {before};
        partner.winning.plugin = "Edit.esp";
        partner.winning.teleportDestinationPosition = core::Vec3{2048, 1024, 0};
        partner.origins.push_back(partner.winning);
        order.records.back() = partner;
        Require(Select(order, root) == std::set<std::uint32_t>{101});
    }
    {
        auto order = Fixture();
        auto &base = order.records.back();
        base.modelPath = "effects/FXFireFixture.nif";
        base.winning.modelPath = *base.modelPath;
        base.origins.front().modelPath = *base.modelPath;
        PutReference(order, {added});
        Require(Select(order, root) == std::set<std::uint32_t>{101});
        auto before = added;
        before.plugin = "Base.esm";
        auto moved = added;
        moved.position = core::Vec3{9216, 1024, 0};
        order.records.pop_back();
        PutReference(order, {before, moved});
        Require(Select(order, root) == std::set<std::uint32_t>{101, 103});
    }
    // Transient records sharing a hearth NIF do not select cells for permanent
    // collision rebuilding when their flames are added or moved.
    for (const auto *type : {"ACTI", "HAZD", "PROJ", "EXPL"})
    {
        auto order = Fixture();
        auto &base = order.records.back();
        base.type = type;
        base.modelPath = "effects/FXFireFixture.nif";
        base.winning.modelPath = *base.modelPath;
        base.origins.front().modelPath = *base.modelPath;
        PutReference(order, {added});
        Require(Select(order, root).empty());
        auto before = added;
        before.plugin = "Base.esm";
        auto moved = added;
        moved.position = core::Vec3{9216, 1024, 0};
        order.records.pop_back();
        PutReference(order, {before, moved});
        Require(Select(order, root).empty());
    }
    WriteHearthModel(root / "meshes/architecture/firepitwide.nif", 4200);
    WriteHearthModel(root / "meshes/architecture/firepitnarrow.nif", 128);
    for (const bool hearthEdited : {false, true})
    {
        auto order = Fixture();
        auto &fireBase = order.records.back();
        fireBase.modelPath = "effects/FXFireFixture.nif";
        fireBase.winning.modelPath = *fireBase.modelPath;
        fireBase.origins.front().modelPath = *fireBase.modelPath;
        skyrim::ResolvedRecord hearth{.type = "STAT", .formId = 30, .modelPath = "architecture/FirePitWide.nif"};
        hearth.winning = {.plugin = "Base.esm", .formId = 30, .hasModel = true, .modelPath = *hearth.modelPath};
        hearth.origins = {hearth.winning};
        if (hearthEdited)
        {
            hearth.origins.front().modelPath = "architecture/FirePitNarrow.nif";
            hearth.winning.plugin = "Edit.esp";
            hearth.origins.push_back(hearth.winning);
        }
        order.records.push_back(hearth);
        auto fire = added;
        fire.plugin = hearthEdited ? "Base.esm" : "Edit.esp";
        PutReference(order, {fire});
        auto host = added;
        host.plugin = "Base.esm";
        host.formId = 40;
        host.baseFormId = 30;
        skyrim::ResolvedRecord placement{.type = "REFR", .formId = host.formId};
        placement.winning = host;
        placement.origins = {host};
        order.records.push_back(placement);
        // Fire addition and a visual-only bed edit both select the fitted rectangle,
        // even where it extends beyond the fuel asset into neighboring CELLs.
        Require(Select(order, root) == std::set<std::uint32_t>{100, 101, 102});
    }
    {
        auto order = Fixture();
        auto &base = order.records.back();
        base.modelPath = "Absent.nif";
        base.origins.front().modelPath = *base.modelPath;
        base.winning = base.origins.front();
        PutReference(order, {added});
        skyrim::ImpactSelectionStatistics counts;
        bool rejected{};
        try
        {
            Select(order, root);
        }
        catch (const std::runtime_error &)
        {
            rejected = true;
        }
        Require(rejected);
        std::set<std::uint32_t> unsafeOwners;
        const auto recover = [&](const skyrim::GeometryFailure &failure, const skyrim::RecordOrigin &origin)
        {
            Require(failure.category == "missing" && failure.reference.formId == 20 && origin.cellFormId);
            unsafeOwners.insert(*origin.cellFormId);
            return true;
        };
        Require(Select(order, root, nullptr, {}, &counts, recover).empty());
        Require(counts.missingModels == std::set<std::string>{"meshes/absent.nif"});
        auto before = added;
        before.plugin = "Base.esm";
        auto moved = added;
        moved.position = core::Vec3{9216, 1024, 0};
        order.records.pop_back();
        PutReference(order, {before, moved});
        Require(Select(order, root, nullptr, {}, nullptr, recover).empty());
        Require(Select(order, root, nullptr, {}, nullptr, recover).empty());
        Require(!unsafeOwners.empty());
    }
    {
        auto order = Fixture();
        auto &base = order.records.back();
        auto replacement = base.winning;
        replacement.plugin = "Edit.esp";
        replacement.modelPath = "Absent.nif";
        base.origins.push_back(replacement);
        base.winning = replacement;
        base.modelPath = replacement.modelPath;
        auto before = added;
        before.plugin = "Base.esm";
        PutReference(order, {before});
        Require(Select(order, root, nullptr, {}, nullptr, [](const auto &, const auto &) { return true; }) ==
                std::set<std::uint32_t>{101});
    }
    {
        auto order = Fixture();
        auto &base = order.records.back();
        base.modelPath = "Unresolved.nif";
        base.origins.front().modelPath = *base.modelPath;
        base.winning = base.origins.front();
        PutReference(order, {added});
        const auto requireFailure = [&](const skyrim::ModelAssetSources *assets)
        {
            bool failed = false;
            try
            {
                Select(order, root, assets);
            }
            catch (const std::runtime_error &error)
            {
                failed = true;
                const std::string message = error.what();
                Require(message.find("in 'Edit.esp'") != std::string::npos);
                Require(message.find("in 'Base.esm'") != std::string::npos);
                Require(message.find("Unresolved.nif") != std::string::npos);
            }
            Require(failed);
        };
        std::ofstream(root / "Broken.bsa") << "invalid archive";
        skyrim::ModelAssetSources assets;
        assets.archives = {root / "Broken.bsa"};
        requireFailure(&assets);
        requireFailure(&assets);
        assets.archives.clear();
        assets.looseModels["meshes/unresolved.nif"] = root / "Unavailable.nif";
        requireFailure(&assets);
        std::ofstream(root / "meshes/Unresolved.nif") << "invalid NIF";
        requireFailure(nullptr);
    }
    {
        auto order = Fixture();
        skyrim::ResolvedRecord world{.type = "WRLD", .formId = 1};
        world.winning = {.plugin = "Edit.esp", .formId = 1};
        world.origins = {world.winning};
        order.records.push_back(std::move(world));
        for (std::uint32_t id : {101U, 102U})
        {
            skyrim::ResolvedRecord land{.type = "LAND", .formId = id + 200, .cellFormId = id, .worldspaceFormId = 1};
            land.winning = {.plugin = "Edit.esp",
                            .formId = land.formId,
                            .cellFormId = id,
                            .worldspaceFormId = 1,
                            .terrainHeights = core::SharedBytes({1})};
            land.origins = {land.winning};
            order.records.push_back(std::move(land));
        }
        Require(Select(order, root) == (std::set<std::uint32_t>{101, 102}));
        auto reference = added;
        reference.cellFormId = 102;
        reference.position = core::Vec3{5120, 1024, 0};
        PutReference(order, {reference});
        Require(Select(order, root) == (std::set<std::uint32_t>{101, 102}));
        // Terrain changes in an established world remain eligible without authored navigation.
        auto established = order;
        for (auto &record : established.records)
        {
            if (record.type == "WRLD")
            {
                record.origins.front().plugin = "Base.esm";
                record.winning = record.origins.front();
            }
        }
        established.records.pop_back();
        Require(Select(established, root) == (std::set<std::uint32_t>{101, 102}));
    }
    {
        auto order = Fixture();
        // Unchanged collision supports water-only generation; its oversized bounds cannot select empty cells.
        for (auto &record : order.records)
        {
            if (record.type != "CELL")
            {
                continue;
            }
            record.origins.front().hasWater = true;
            record.origins.front().usesWorldWater = false;
            record.origins.front().waterHeight = 1.0F;
            auto after = record.origins.front();
            after.plugin = "Edit.esp";
            after.waterHeight = 2.0F;
            record.origins.push_back(after);
            record.winning = after;
        }
        // Authored navigation in an empty CELL is not collision evidence.
        order.records.push_back({.type = "NAVM", .formId = 500, .cellFormId = 103});
        auto before = added;
        before.plugin = "Base.esm";
        PutReference(order, {before});
        order.cells[1].references.push_back({.id = 20,
                                             .baseObjectId = 10,
                                             .recordType = "REFR",
                                             .modelPath = "TallSolid.nif",
                                             .position = {1024, 1024, 0}});
        Require(Select(order, root) == std::set<std::uint32_t>{101});
        const auto base = std::find_if(order.records.begin(), order.records.end(),
                                       [](const auto &record) { return record.formId == 10; });
        base->modelPath = "VisualOnly.nif";
        base->winning.modelPath = "VisualOnly.nif";
        base->origins.front().modelPath = "VisualOnly.nif";
        Require(Select(order, root).empty());
    }
    {
        auto order = Fixture();
        PutReference(order, {added});
        Require(Select(order, root) == std::set<std::uint32_t>{101});
        auto disabled = added;
        disabled.initiallyDisabled = true;
        order.records.back().origins = {disabled};
        order.records.back().winning = disabled;
        Require(Select(order, root).empty());
    }
    {
        auto order = Fixture();
        auto before = added;
        before.plugin = "Base.esm";
        auto moved = added;
        moved.cellFormId = 103;
        moved.position = core::Vec3{9216, 1024, 0};
        PutReference(order, {before, moved});
        Require(Select(order, root) == (std::set<std::uint32_t>{101, 103}));
        order.records.back().origins.back() = before;
        order.records.back().origins.back().plugin = "Edit.esp";
        order.records.back().winning = order.records.back().origins.back();
        Require(Select(order, root).empty());
        order.records.back().origins.back().deleted = true;
        order.records.back().winning = order.records.back().origins.back();
        Require(Select(order, root) == std::set<std::uint32_t>{101});
    }
    {
        auto order = Fixture();
        auto &base = order.records.back();
        base.origins.front().modelPath = "VisualOnly.nif";
        PutReference(order, {added});
        Require(Select(order, root).empty());
        order.records[order.records.size() - 2].origins.front().modelPath = "Effects/VisualOnly.nif";
        Require(Select(order, root).empty());
    }
    {
        auto order = Fixture();
        auto before = added;
        before.plugin = "Base.esm";
        PutReference(order, {before});
        WriteModel(root / "replacement.nif", true);
        skyrim::ModelAssetSources assets;
        assets.looseModels.emplace("meshes/tallsolid.nif", root / "replacement.nif");
        Require(Select(order, root, &assets, {"meshes/tallsolid.nif"}).empty());
        WriteModel(root / "replacement.nif", true, 9000);
        Require(Select(order, root, &assets, {"meshes/tallsolid.nif"}) == (std::set<std::uint32_t>{101, 102, 103}));
    }
}
