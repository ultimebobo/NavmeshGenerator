#ifdef NDEBUG
#undef NDEBUG
#endif

#include "app/candidate_cache.h"
#include "app/candidate_artifacts.h"
#include "core/io/content_hash.h"
#include "core/io/shared_bytes.h"
#include "skyrim/parser/affected_cells.h"

#include <cassert>
#include <fstream>
#include <limits>
#include <sstream>
#include <zlib.h>

void TestPerformanceCaches()
{
    using namespace navmesh;
    core::ContentHash empty;
    assert(empty.Hex() == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    core::ContentHash split;
    split.Add("a");
    split.Add("bc");
    assert(split.Hex() == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    assert(split.Hex() == split.Hex());
    core::SharedBytes bytes(std::vector<std::uint8_t>{1, 2, 3, 4});
    auto slice = bytes.Slice(1, 2);
    assert(slice.data() == bytes.data() + 1);
    bytes = {};
    assert(slice[0] == 2 && slice[1] == 3);
    auto moved = std::move(slice);
    assert(slice.empty() && slice.data() == nullptr && moved.size() == 2);

    core::Scene scene;
    scene.mesh.vertices = {{0, 0, 0}, {256, 0, 0}, {256, 256, 0}, {0, 256, 0}};
    scene.mesh.triangles = {{{0, 1, 2}}, {{0, 2, 3}}};
    scene.geometrySources.push_back({.sourceType = core::GeometrySourceType::Terrain});
    scene.triangleProvenance = {{0, 0}, {0, 1}};
    auto profile = core::NavigationProfile{};
    profile.agentRadius = 0;
    profile.minimumRegionArea = 0;
    const auto key = app::detail::CandidateFingerprint(scene, profile, {}, {}, {}, "watershed");
    assert(!key.empty());
    const auto original = scene.mesh.vertices[0].z;
    scene.mesh.vertices[0].z += 1;
    assert(app::detail::CandidateFingerprint(scene, profile, {}, {}, {}, "watershed") != key);
    scene.mesh.vertices[0].z = original;
    assert(app::detail::CandidateFingerprint(scene, profile, {}, {}, {}, "monotone") != key);
    auto neighbor = core::NavMesh{.id = 7, .vertices = {{0, 0, 0}}};
    assert(app::detail::CandidateFingerprint(scene, profile, {}, {}, {neighbor}, "watershed") != key);

    auto candidate = core::GenerateCandidate(scene, profile);
    assert(candidate.topology.valid && candidate.mesh.polygons.size() == 2);
    const auto root = std::filesystem::temp_directory_path() / "navmesh-cache-round-trip";
    std::filesystem::create_directories(root);
    const auto path = root / "candidate.gz";
    std::error_code error;
    std::filesystem::remove(path, error);
    assert(app::detail::StoreCandidate(path, candidate, scene));
    core::CandidateNavMesh loaded;
    core::Scene evidence;
    assert(app::detail::LoadCandidate(path, loaded, evidence));
    std::ostringstream before, after;
    assert(core::WriteCandidateJson(before, candidate, scene, "{}"));
    assert(core::WriteCandidateJson(after, loaded, evidence, "{}"));
    assert(before.str() == after.str());
    assert(app::detail::WriteCompressedCandidateJson(root / "inspection.json.gz", loaded, evidence, "{}"));
#ifdef _WIN32
    auto gzip = gzopen_w((root / "inspection.json.gz").c_str(), "rb");
#else
    auto gzip = gzopen((root / "inspection.json.gz").c_str(), "rb");
#endif
    assert(gzip);
    std::string text(before.str().size(), '\0');
    assert(gzread(gzip, text.data(), static_cast<unsigned int>(text.size())) == static_cast<int>(text.size()));
    assert(gzclose(gzip) == Z_OK && text == before.str());
    std::filesystem::remove(path, error);
    auto invalid = candidate;
    invalid.regions.front().sourceTriangles.push_back(scene.triangleProvenance.size());
    assert(app::detail::StoreCandidate(path, invalid, scene));
    assert(!app::detail::LoadCandidate(path, loaded, evidence));
    std::ofstream(path, std::ios::binary | std::ios::trunc) << "corrupt";
    assert(!app::detail::LoadCandidate(path, loaded, evidence));

    skyrim::offline::ResolvedLoadOrder resolved;
    resolved.cells.push_back({.id = 1, .isInterior = true});
    resolved.records.push_back(
        {.type = "STAT", .formId = 10, .winning = {.modelRadius = 128.0F}, .modelPath = "bounded.nif"});
    resolved.cells[0].references = {
        {.id = 20, .baseObjectId = 10, .modelPath = "bounded.nif", .position = {64, 64, 0}},
        {.id = 21, .baseObjectId = 10, .modelPath = "bounded.nif", .position = {8192, 8192, 0}},
        {.id = 22, .baseObjectId = 11, .modelPath = "unknown.nif", .position = {8192, 8192, 0}},
        {.id = 23, .baseObjectId = 10, .modelPath = "different.nif", .position = {8192, 8192, 0}},
        {.id = 24, .baseObjectId = 10, .modelPath = "bounded.nif", .scale = std::numeric_limits<float>::infinity()}};
    const skyrim::offline::CellImpactIndex index(resolved);
    const auto filtered =
        index.GeometryCell(resolved.cells[0], core::AABB{.min = {0, 0, -1000}, .max = {256, 256, 1000}});
    assert(filtered.references.size() == 4);
    assert(filtered.references[0].id == 20 && filtered.references[1].id == 22 && filtered.references[2].id == 23);
}
