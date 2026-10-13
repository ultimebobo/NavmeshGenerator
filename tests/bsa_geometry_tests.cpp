#ifdef NDEBUG
#undef NDEBUG
#endif

#include "skyrim/extraction/asset_cache.h"
#include "skyrim/extraction/geometry_extractor.h"
#include "core/navmesh/generator.h"

#include <NifFile.hpp>
#include <bhk.hpp>
#include <lz4frame.h>

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>

namespace
{
    void AppendInteger(std::string &bytes, std::uint64_t value, std::size_t width = 4)
    {
        for (std::size_t index{}; index < width; ++index)
        {
            bytes.push_back(static_cast<char>(value >> (index * 8)));
        }
    }

    // Build a named SE archive with one prefixed, LZ4-compressed synthetic model.
    void WriteModelArchive(const std::filesystem::path &path, const std::string &model)
    {
        const std::string folder("meshes\0", 7);
        const std::string filename("fixture.nif\0", 12);
        const std::string prefix = "meshes\\fixture.nif";
        std::string compressed(LZ4F_compressFrameBound(model.size(), nullptr), '\0');
        const auto compressedSize =
            LZ4F_compressFrame(compressed.data(), compressed.size(), model.data(), model.size(), nullptr);
        assert(!LZ4F_isError(compressedSize));
        compressed.resize(compressedSize);
        std::string payload(1, static_cast<char>(prefix.size()));
        payload += prefix;
        AppendInteger(payload, model.size());
        payload += compressed;
        std::string archive("BSA\0", 4);
        for (const auto field : {105U, 36U, 0x107U, 1U, 1U, 7U, 12U, 1U})
        {
            AppendInteger(archive, field);
        }
        AppendInteger(archive, 0, 8);
        AppendInteger(archive, 1);
        AppendInteger(archive, 0);
        AppendInteger(archive, 60, 8);
        archive.push_back(static_cast<char>(folder.size()));
        archive += folder;
        AppendInteger(archive, 0, 8);
        AppendInteger(archive, payload.size());
        AppendInteger(archive, archive.size() + 4 + filename.size());
        archive += filename;
        archive += payload;
        std::ofstream stream(path, std::ios::binary | std::ios::trunc);
        stream.write(archive.data(), static_cast<std::streamsize>(archive.size()));
        assert(stream.good());
    }

    void WriteCollisionModel(const std::filesystem::path &path, float halfWidth, float top)
    {
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        auto data = std::make_unique<nifly::hkPackedNiTriStripsData>();
        constexpr float havokScale = 69.99125F;
        data->numVerts = 4;
        data->compressedVertData = {{-halfWidth / havokScale, -halfWidth / havokScale, top / havokScale},
                                    {halfWidth / havokScale, -halfWidth / havokScale, top / havokScale},
                                    {halfWidth / havokScale, halfWidth / havokScale, top / havokScale},
                                    {-halfWidth / havokScale, halfWidth / havokScale, top / havokScale}};
        data->keyCount = 2;
        data->triData.resize(2);
        data->triData[0].tri = {0, 1, 2};
        data->triData[1].tri = {0, 2, 3};
        auto shape = std::make_unique<nifly::bhkPackedNiTriStripsShape>();
        shape->dataRef.index = nif.GetHeader().AddBlock(std::move(data));
        auto body = std::make_unique<nifly::bhkRigidBody>();
        body->shapeRef.index = nif.GetHeader().AddBlock(std::move(shape));
        auto object = std::make_unique<nifly::bhkCollisionObject>();
        object->bodyRef.index = nif.GetHeader().AddBlock(std::move(body));
        object->targetRef.index = nif.GetBlockID(nif.GetRootNode());
        nif.GetRootNode()->collisionRef.index = nif.GetHeader().AddBlock(std::move(object));
        assert(nif.Save(path) == 0);
    }

    bool HasNavigationAt(const navmesh::core::CandidateNavMesh &candidate, navmesh::core::Vec3 point)
    {
        const auto cross = [](auto a, auto b, auto c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); };
        for (const auto &polygon : candidate.mesh.polygons)
        {
            const auto a = candidate.mesh.vertices[polygon.vertices[0]];
            const auto b = candidate.mesh.vertices[polygon.vertices[1]];
            const auto c = candidate.mesh.vertices[polygon.vertices[2]];
            const auto area = cross(a, b, c);
            if (std::abs(area) < 0.001F)
            {
                continue;
            }
            const auto u = cross(point, b, c) / area;
            const auto v = cross(a, point, c) / area;
            const auto w = 1 - u - v;
            if (u >= -0.001F && v >= -0.001F && w >= -0.001F && std::abs(a.z * u + b.z * v + c.z * w - point.z) < 8)
            {
                return true;
            }
        }
        return false;
    }

    // Exercise the extraction policy and the neutral generator together. Low furniture
    // and an overlapping fuel bed would be climbable without obstacle-only evidence.
    void CheckFurnitureAndFire(const std::filesystem::path &root)
    {
        using namespace navmesh;
        std::filesystem::create_directories(root / "meshes/effects");
        std::filesystem::create_directories(root / "meshes/furniture");
        WriteCollisionModel(root / "meshes/floor.nif", 768, 0);
        WriteCollisionModel(root / "meshes/furniture/fixture.nif", 64, 24);
        nifly::NifFile nif;
        nif.Create({nifly::V20_2_0_7, 12, 130});
        nif.GetRootNode()->transform.translation = {32, 16, 0};
        const std::vector<nifly::Vector3> fuel = {{-64, -64, 0}, {64, -64, 0}, {64, 64, 8}, {-64, 64, 8}};
        const std::vector<nifly::Triangle> faces = {{0, 1, 2}, {0, 2, 3}};
        auto *bed = nif.CreateShapeFromData("L2_CoalsBed", &fuel, &faces, nullptr);
        assert(bed);
        bed->transform.translation = {16, 8, 0};
        const std::vector<nifly::Vector3> glow = {{-512, -512, 0}, {512, -512, 0}, {512, 512, 512}};
        const std::vector<nifly::Triangle> glowFaces = {{0, 1, 2}};
        assert(nif.CreateShapeFromData("glow", &glow, &glowFaces, nullptr));
        assert(nif.Save(root / "meshes/effects/fxfirefixture.nif") == 0);

        core::Cell cell;
        cell.references = {{.id = 1, .recordType = "REFR", .modelPath = "floor.nif", .baseRecordType = "STAT"},
                           {.id = 2,
                            .recordType = "REFR",
                            .modelPath = "Furniture/fixture.nif",
                            .position = {-256, 0, 0},
                            .baseRecordType = "FURN"},
                           {.id = 3,
                            .recordType = "REFR",
                            .modelPath = "MESHES/Effects/FXFireFixture.nif",
                            .position = {256, 0, -2},
                            .rotation = {0, 0, 1.57079632679F},
                            .scale = 2,
                            .baseRecordType = "STAT"}};
        skyrim::ModelGeometryCache cache;
        for (const bool navigationOnly : {false, true})
        {
            const auto geometry = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, navigationOnly);
            assert(geometry.modelsLoaded == 3 && geometry.modelsExcluded == 0);
            assert(geometry.collisionModelsLoaded == 2 && geometry.scene.HasCompleteTriangleProvenance());
            // Display mode appends a visual source after each exclusion.
            const auto source = std::find_if(
                geometry.scene.geometrySources.begin(), geometry.scene.geometrySources.end(),
                [](const auto &value) { return value.sourceType == core::GeometrySourceType::NavigationExclusion; });
            assert(source != geometry.scene.geometrySources.end() && source->navigationObstacle);
            assert(geometry.references[2].sourceType == "navigation_exclusion" &&
                   !geometry.references[2].usedRenderFallback);
            assert(geometry.references[2].triangles == 12);
            const auto offset = geometry.references[2].meshVertexOffset;
            core::AABB bounds;
            for (std::size_t index = offset; index < offset + 8; ++index)
            {
                bounds.Expand(geometry.scene.mesh.vertices[index]);
            }
            assert(std::abs(bounds.min.x - 176) < 0.01F && std::abs(bounds.max.x - 432) < 0.01F);
            assert(std::abs(bounds.min.y + 224) < 0.01F && std::abs(bounds.max.y - 32) < 0.01F);
            assert(std::abs(bounds.min.z + 2) < 0.01F && std::abs(bounds.max.z - 14) < 0.01F);
            for (const auto partitioning :
                 {core::RegionPartitioningAlgorithm::Watershed, core::RegionPartitioningAlgorithm::Monotone,
                  core::RegionPartitioningAlgorithm::Layers})
            {
                const auto candidate = core::RecastCandidateGenerator{}.Generate(
                    geometry.scene, {}, std::nullopt, {}, partitioning, {}, core::CandidateRetention::AllWalkable);
                assert(candidate.topology.valid);
                assert(!HasNavigationAt(candidate, {-256, 0, 24}));
                assert(!HasNavigationAt(candidate, {-256, 0, 0}));
                assert(!HasNavigationAt(candidate, {304, -96, 0}));
                assert(!HasNavigationAt(candidate, {304, -96, 14}));
                assert(HasNavigationAt(candidate, {0, 256, 0}));
                assert(HasNavigationAt(candidate, {304, 256, 0}));
            }
        }
        for (const auto *type : {"FURN", "CONT", "STAT"})
        {
            cell.references.resize(2);
            cell.references[1].baseRecordType = type;
            const auto geometry = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, true);
            assert(geometry.scene.geometrySources[1].navigationObstacle);
        }
        cell.references[1].modelPath = "effects/DecorativeGlow.nif";
        cell.references[1].baseRecordType = "STAT";
        const auto decorative = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, true);
        assert(decorative.modelsExcluded == 1 && decorative.modelsMissing == 0);

        // A flame-only asset still defines an exclusion; an effect without fuel or
        // flame triangles remains unsupported rather than borrowing billboard bounds.
        nifly::NifFile flame;
        flame.Create({nifly::V20_2_0_7, 12, 130});
        assert(flame.CreateShapeFromData("Flames", &fuel, &faces, nullptr));
        assert(flame.Save(root / "meshes/effects/fxfireflame.nif") == 0);
        cell.references[1].modelPath = "effects/FXFireFlame.nif";
        const auto flameOnly = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, true);
        assert(flameOnly.modelsLoaded == 2 && flameOnly.references[1].collisionType == "fire flame bounds");
        nifly::NifFile emptyFire;
        emptyFire.Create({nifly::V20_2_0_7, 12, 130});
        assert(emptyFire.CreateShapeFromData("glow", &glow, &glowFaces, nullptr));
        assert(emptyFire.Save(root / "meshes/effects/fxfireglow.nif") == 0);
        cell.references[1].modelPath = "effects/FXFireGlow.nif";
        const auto unsupported = skyrim::ExtractGeometry(root, cell, root, {}, {}, nullptr, &cache, true);
        assert(unsupported.modelsUnsupported == 1 && unsupported.modelsLoaded == 1);
        assert(unsupported.scene.coverage.back().status == core::GeometryCoverage::Unsupported);
    }
} // namespace

void TestNativeBsaGeometry()
{
    using namespace navmesh;
    const auto root =
        std::filesystem::temp_directory_path() /
        ("navmesh-bsa-geometry-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);
    struct Cleanup
    {
        std::filesystem::path path;
        ~Cleanup()
        {
            std::error_code error;
            std::filesystem::remove_all(path, error);
        }
    } cleanup{root};

    nifly::NifFile nif;
    nif.Create({nifly::V20_2_0_7, 12, 130});
    const std::vector<nifly::Vector3> vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    const std::vector<nifly::Triangle> triangles = {{0, 1, 2}};
    assert(nif.CreateShapeFromData("Fixture", &vertices, &triangles, nullptr));
    std::ostringstream model(std::ios::binary);
    assert(nif.Save(model) == 0);
    WriteModelArchive(root / "Fixture.BSA", model.str());
    const auto snapshot = skyrim::ModelAssetCacheDirectory(root, nullptr, root / "cache");
    assert(skyrim::ModelArchives(root, nullptr).size() == 1);
    core::Cell cell;
    cell.references.push_back({.id = 1,
                               .baseObjectId = 2,
                               .recordType = "REFR",
                               .modelPath = "fixture.nif",
                               .position = {10, 20, 30},
                               .baseRecordType = "STAT"});
    skyrim::ModelGeometryCache cache;
    const auto geometry = skyrim::ExtractGeometry(root, cell, snapshot, {}, {}, nullptr, &cache);
    assert(geometry.modelsLoaded == 1 && geometry.scene.mesh.triangles.size() == 1);
    assert(geometry.scene.mesh.vertices[0].x == 10 && geometry.scene.mesh.vertices[0].z == 30);
    assert(geometry.scene.HasCompleteTriangleProvenance());
    // Decoded geometry remains valid if disk retention evicts its source NIF.
    skyrim::TrimModelAssetCache(snapshot, 0);
    assert(!std::filesystem::exists(snapshot / "meshes/fixture.nif"));
    const auto repeated = skyrim::ExtractGeometry(root, cell, snapshot, {}, {}, nullptr, &cache);
    assert(repeated.modelsLoaded == 1 && cache.Statistics().modelHits == 1);
    assert(!std::filesystem::exists(snapshot / "meshes/fixture.nif"));

    skyrim::ModelAssetSources assets;
    assets.archives = {root / "Fixture.BSA"};
    std::set<std::string> changed;
    assert(skyrim::ChangedArchiveModels(root, assets, snapshot, {assets.archives[0]}, changed));
    assert(changed == std::set<std::string>{"meshes/fixture.nif"});
    changed.clear();
    assets.looseModels["meshes/fixture.nif"] = root / "loose.nif";
    assert(skyrim::ChangedArchiveModels(root, assets, snapshot, {assets.archives[0]}, changed));
    assert(changed.empty());
    WriteModelArchive(assets.archives[0], model.str() + "revision");
    assert(skyrim::ModelAssetCacheDirectory(root, &assets, root / "cache") != snapshot);
    std::ofstream(assets.archives[0], std::ios::binary | std::ios::trunc) << "broken";
    changed.insert("sentinel");
    assert(!skyrim::ChangedArchiveModels(root, assets, snapshot, {assets.archives[0]}, changed));
    assert(changed == std::set<std::string>{"sentinel"});

    // Asset classification belongs to Skyrim extraction; the neutral generator consumes only the tag.
    const auto rockDirectory = root / "meshes/Landscape/Rocks";
    std::filesystem::create_directories(rockDirectory);
    std::ofstream(rockDirectory / "fixture.nif", std::ios::binary) << model.str();
    for (const auto *logicalPath : {"Landscape\\Rocks\\fixture.nif", "MESHES/LANDSCAPE/ROCKS/fixture.nif"})
    {
        cell.references[0].modelPath = logicalPath;
        const auto rock = skyrim::ExtractGeometry(root, cell, snapshot);
        assert(rock.modelsLoaded == 1 && rock.scene.geometrySources[0].navigationObstacle);
    }
    std::filesystem::create_directories(root / "meshes/Architecture");
    std::ofstream(root / "meshes/Architecture/RockStairs.nif", std::ios::binary) << model.str();
    cell.references[0].modelPath = "Architecture/RockStairs.nif";
    const auto architecture = skyrim::ExtractGeometry(root, cell, snapshot);
    assert(architecture.modelsLoaded == 1 && !architecture.scene.geometrySources[0].navigationObstacle);
    CheckFurnitureAndFire(root);
}
