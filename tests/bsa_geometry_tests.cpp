#ifdef NDEBUG
#undef NDEBUG
#endif

#include "skyrim/extraction/asset_cache.h"
#include "skyrim/extraction/geometry_extractor.h"

#include <NifFile.hpp>
#include <lz4frame.h>

#include <cassert>
#include <chrono>
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
}
