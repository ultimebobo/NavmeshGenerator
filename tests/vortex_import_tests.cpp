// Fixture setup and checks require assertions in every build mode.
#ifdef NDEBUG
#undef NDEBUG
#endif

#include "skyrim/vortex/vortex_importer.h"

#include <cassert>
#include <chrono>
#include <fstream>

void TestVortexDeploymentImport()
{
    using namespace navmesh;
    const auto root =
        std::filesystem::temp_directory_path() /
        ("navmesh-vortex-fixture-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto game = root / "Game";
    const auto data = game / "Data";
    const auto plugins = root / "Application data";
    std::filesystem::create_directories(data / "meshes" / "Fixture");
    std::filesystem::create_directories(plugins);
    const auto write = [](const std::filesystem::path &path, const std::string &text)
    {
        std::ofstream file(path, std::ios::binary | std::ios::trunc);
        file << text;
        assert(file.good());
    };
    write(data / "Skyrim.esm", "native fixture");
    write(data / "Update.esm", "native fixture");
    write(data / "Creation.esl", "creation fixture");
    write(data / "First.esp", "plugin fixture");
    write(data / "Last.esp", "plugin fixture");
    write(data / "Disabled.esp", "plugin fixture");
    write(data / "Skyrim - Meshes.bsa", "archive fixture");
    write(data / "First.bsa", "archive fixture");
    write(data / "Last - Main.bsa", "archive fixture");
    write(data / "Disabled.bsa", "archive fixture");
    write(data / "meshes" / "Fixture" / "Floor.NIF", "model fixture");
    write(game / "Skyrim.ccc", "Creation.esl\n");
    write(plugins / "plugins.txt",
          "\xEF\xBB\xBF# active fixture\r\nSkyrim.esm\r\n*FIRST.ESP\r\nDisabled.esp\r\n*Last.esp\r\n");
    // Deliberately different: SSE activation and ordering both come from plugins.txt.
    write(plugins / "loadorder.txt", "Last.esp\nDisabled.esp\nFirst.esp\n");
    const auto imported = skyrim::ImportVortexDeployment(game, plugins);
    assert(imported.diagnostics.empty());
    assert((imported.pluginPaths == std::vector<std::filesystem::path>{data / "Skyrim.esm", data / "Update.esm",
                                                                       data / "Creation.esl", data / "First.esp",
                                                                       data / "Last.esp"}));
    assert(
        (imported.assets.archives == std::vector<std::filesystem::path>{data / "Skyrim - Meshes.bsa",
                                                                        data / "First.bsa", data / "Last - Main.bsa"}));
    assert(imported.assets.looseModels.at("meshes/fixture/floor.nif") == data / "meshes" / "Fixture" / "Floor.NIF");
    assert(skyrim::VortexSnapshotMatches(imported));
    assert(skyrim::WriteInputReport(root / "input-report.json", imported));

    write(data / "meshes" / "Fixture" / "Added.nif", "added model");
    assert(!skyrim::VortexSnapshotMatches(imported));
    const auto revised = skyrim::ImportVortexDeployment(game, plugins);
    assert(revised.diagnostics.empty() && revised.snapshotHash != imported.snapshotHash);
    write(plugins / "plugins.txt", "*First.esp\n*Missing.esp\n");
    assert(!skyrim::VortexSnapshotMatches(revised));
    auto invalid = skyrim::ImportVortexDeployment(game, plugins);
    assert(invalid.pluginPaths.empty() && !invalid.diagnostics.empty());
    write(plugins / "plugins.txt", "*First.esp\n*FIRST.ESP\n");
    invalid = skyrim::ImportVortexDeployment(game, plugins);
    assert(invalid.pluginPaths.empty() && !invalid.diagnostics.empty());
    write(plugins / "plugins.txt", "*../First.esp\n");
    invalid = skyrim::ImportVortexDeployment(game, plugins);
    assert(invalid.pluginPaths.empty() && !invalid.diagnostics.empty());
    std::filesystem::remove(plugins / "plugins.txt");
    invalid = skyrim::ImportVortexDeployment(game, plugins);
    assert(invalid.pluginPaths.empty() && !invalid.diagnostics.empty());
    invalid = skyrim::ImportVortexDeployment(root / "Absent game", plugins);
    assert(invalid.pluginPaths.empty() && !invalid.diagnostics.empty());
    std::filesystem::remove_all(root);
}
