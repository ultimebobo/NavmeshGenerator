#ifdef NDEBUG
#undef NDEBUG
#endif

#include "app/cell_scene.h"

#include <cassert>
#include <chrono>
#include <fstream>
#include <limits>

void TestCellSceneAccumulation()
{
    using namespace navmesh;
    const auto directory =
        std::filesystem::temp_directory_path() /
        ("navmesh-cell-scene-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(directory);
    core::Scene scene;
    scene.geometrySources.push_back({.modelPath = "fixture.nif",
                                     .sourceType = core::GeometrySourceType::Collision,
                                     .reference = {"Fixture.esp", 0x500, "REFR"}});
    scene.mesh = {.vertices = {{0, 0, 0}, {10, 0, 0}, {0, 10, 0}}, .triangles = {{{0, 1, 2}}}};
    scene.triangleProvenance.push_back({0, 0});
    scene.geometrySources.push_back({.modelPath = "fixture.nif",
                                     .sourceType = core::GeometrySourceType::RenderFallback,
                                     .reference = {"Fixture.esp", 0x500, "REFR"}});
    scene.renderFallbackMesh = scene.mesh;
    scene.renderFallbackTriangleProvenance.push_back({1, 0});
    core::Cell cell;
    cell.id = 0x100;
    cell.editorId = "FixtureCell";
    cell.navMeshes.push_back(
        {.id = 0x200, .vertices = scene.mesh.vertices, .polygons = {{{0, 1, 2}}}, .doorLinks = {{0, 0x600}}});
    core::Cell supplier;
    supplier.id = 0x101;
    core::Reference exit;
    exit.id = 0x600;
    exit.position = {5, 5, 0};
    supplier.references.push_back(exit);
    app::detail::CellScene combined;
    combined.Append(scene, {cell, supplier});
    combined.Append(scene, {supplier, cell});
    // Separate placements at the same positions must remain independent sources.
    scene.geometrySources[0].reference.formId = 0x501;
    scene.geometrySources[1].reference.formId = 0x501;
    combined.Append(scene, {cell, supplier});
    scene.mesh.vertices[0].z = std::numeric_limits<float>::quiet_NaN();
    combined.Append(scene, {});
    app::Options options;
    const auto path = directory / "scene.glb";
    assert(combined.Write(path, options, {&cell, &supplier}));
    const auto read = [](const std::filesystem::path &input)
    {
        std::ifstream stream(input, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(stream), {});
    };
    const auto metadata = read(path.string() + ".metadata.json");
    assert(metadata.contains("\"geometry_triangles\": 2"));
    assert(metadata.contains("\"selected_cells\""));
    const auto provenance = read(path.string() + ".provenance.json");
    assert(provenance.contains("Render fallback"));
    assert(provenance.contains("Door 00000600"));
    assert(provenance.contains("Existing NAVM 00000200"));
    std::filesystem::create_directories(directory / "blocked.glb");
    assert(!combined.Write(directory / "blocked.glb", options, {&cell}));
}
