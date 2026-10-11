#include "skyrim/vortex/vortex_importer.h"
#include "core/io/content_hash.h"
#include "core/reproducibility/export_metadata.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>

namespace
{
    using namespace navmesh;

    std::string PathText(const std::filesystem::path &path)
    {
        const auto utf8 = path.generic_u8string();
        return {reinterpret_cast<const char *>(utf8.data()), utf8.size()};
    }

    std::string Lower(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
        return value;
    }

    bool IsPlugin(const std::filesystem::path &path)
    {
        const auto extension = Lower(PathText(path.extension()));
        return extension == ".esp" || extension == ".esm" || extension == ".esl";
    }

    /// Read UTF-8 game lists with a possible BOM; strip blank lines and full-line comments only.
    std::vector<std::string> ReadList(const std::filesystem::path &path, bool required)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file)
        {
            if (required || std::filesystem::exists(path))
            {
                throw std::runtime_error("Cannot read " + PathText(path) +
                                         ". Activate the Vortex profile and deploy mods.");
            }
            return {};
        }
        std::vector<std::string> lines;
        std::string line;
        bool firstLine = true;
        while (std::getline(file, line))
        {
            if (firstLine && line.starts_with("\xEF\xBB\xBF"))
            {
                line.erase(0, 3);
            }
            firstLine = false;
            const auto first = line.find_first_not_of(" \t\r\n");
            if (first == std::string::npos || line[first] == '#')
            {
                continue;
            }
            lines.push_back(line.substr(first, line.find_last_not_of(" \t\r\n") - first + 1));
        }
        if (file.bad())
        {
            throw std::runtime_error("Cannot completely read " + PathText(path));
        }
        return lines;
    }

    std::string PluginKey(const std::string &name)
    {
        if (name.find_first_of("/\\:") != std::string::npos ||
            std::any_of(name.begin(), name.end(), [](unsigned char character) { return character < 0x20; }) ||
            !IsPlugin(std::filesystem::path(std::u8string(name.begin(), name.end()))))
        {
            throw std::runtime_error("Invalid plugin filename in game list: " + name);
        }
        return Lower(name);
    }

    std::filesystem::path DefaultPluginsDirectory()
    {
#ifdef _WIN32
        wchar_t *value{};
        std::size_t length{};
        const auto status = _wdupenv_s(&value, &length, L"LOCALAPPDATA");
        const std::unique_ptr<wchar_t, decltype(&std::free)> local(value, &std::free);
        if (status != 0 || !local || !*local)
#else
        const auto local = std::getenv("LOCALAPPDATA");
        if (!local || !*local)
#endif
        {
            throw std::runtime_error("Local application data is unavailable; specify --vortex-plugins.");
        }
#ifdef _WIN32
        return std::filesystem::path(local.get()) / "Skyrim Special Edition";
#else
        return std::filesystem::path(local) / "Skyrim Special Edition";
#endif
    }

    /// Snapshot membership and revisions deterministically; game files are never opened for writing.
    std::string Snapshot(const skyrim::VortexInput &input)
    {
        core::ContentHash hash;
        hash.Add("navmesh-vortex-deployment-v1\n");
        hash.Add(PathText(input.gameRoot) + "\n" + PathText(input.pluginsDirectory) + "\n");
        for (const auto &path : {input.pluginsDirectory / "plugins.txt", input.pluginsDirectory / "loadorder.txt",
                                 input.gameRoot / "Skyrim.ccc"})
        {
            hash.Add(PathText(path) + "\n");
            if (!std::filesystem::exists(path))
            {
                hash.Add("missing\n");
                continue;
            }
            std::ifstream file(path, std::ios::binary);
            if (!file)
            {
                throw std::runtime_error("Cannot snapshot " + PathText(path));
            }
            hash.Add(std::string(std::istreambuf_iterator<char>(file), {}));
            if (file.bad())
            {
                throw std::runtime_error("Cannot completely snapshot " + PathText(path));
            }
            hash.Add("\n");
        }
        std::vector<std::filesystem::path> files;
        for (const auto &entry : std::filesystem::directory_iterator(input.gameData))
        {
            if (entry.is_regular_file() &&
                (IsPlugin(entry.path()) || Lower(PathText(entry.path().extension())) == ".bsa"))
            {
                files.push_back(entry.path());
            }
        }
        const auto meshes = input.gameData / "meshes";
        if (std::filesystem::is_directory(meshes))
        {
            for (const auto &entry : std::filesystem::recursive_directory_iterator(meshes))
            {
                if (entry.is_regular_file() && Lower(PathText(entry.path().extension())) == ".nif")
                {
                    files.push_back(entry.path());
                }
            }
        }
        std::sort(files.begin(), files.end());
        for (const auto &path : files)
        {
            hash.Add(PathText(path) + "\n" + std::to_string(std::filesystem::file_size(path)) + "\n" +
                     std::to_string(std::filesystem::last_write_time(path).time_since_epoch().count()) + "\n");
        }
        return hash.Hex();
    }

    /// SSE uses plugins.txt order. Native files load first even when omitted or unstarred in that list.
    void ResolvePlugins(skyrim::VortexInput &input)
    {
        std::map<std::string, std::filesystem::path> deployed;
        for (const auto &entry : std::filesystem::directory_iterator(input.gameData))
        {
            if (entry.is_regular_file() && IsPlugin(entry.path()))
            {
                if (!deployed.emplace(Lower(PathText(entry.path().filename())), entry.path()).second)
                {
                    throw std::runtime_error("Duplicate deployed plugin filename: " + PathText(entry.path()));
                }
            }
        }
        std::vector<std::string> order;
        std::set<std::string> included;
        for (const auto *native : {"Skyrim.esm", "Update.esm", "Dawnguard.esm", "HearthFires.esm", "Dragonborn.esm"})
        {
            const auto key = Lower(native);
            if (deployed.contains(key))
            {
                order.push_back(key);
                included.insert(key);
            }
        }
        for (const auto &name : ReadList(input.gameRoot / "Skyrim.ccc", false))
        {
            const auto key = PluginKey(name);
            if (deployed.contains(key) && included.insert(key).second)
            {
                order.push_back(key);
            }
        }
        std::set<std::string> listed;
        for (auto name : ReadList(input.pluginsDirectory / "plugins.txt", true))
        {
            const bool active = name.front() == '*';
            if (active)
            {
                name.erase(0, 1);
            }
            const auto key = PluginKey(name);
            if (!listed.insert(key).second)
            {
                throw std::runtime_error("Duplicate plugin in plugins.txt: " + name);
            }
            if (active && included.insert(key).second)
            {
                if (!deployed.contains(key))
                {
                    throw std::runtime_error("Active plugin is missing from deployed Data: " + name +
                                             ". Deploy mods in Vortex.");
                }
                order.push_back(key);
            }
        }
        for (const auto &name : order)
        {
            input.pluginPaths.push_back(deployed.at(name));
        }
        if (input.pluginPaths.empty())
        {
            throw std::runtime_error(
                "No active deployed plugins were found. Activate the Vortex profile and deploy mods.");
        }
    }

    /// Loose files are deployed winners; base archives precede archives attached to active plugins.
    void ResolveAssets(skyrim::VortexInput &input)
    {
        const auto meshes = input.gameData / "meshes";
        if (std::filesystem::is_directory(meshes))
        {
            for (const auto &entry : std::filesystem::recursive_directory_iterator(meshes))
            {
                if (entry.is_regular_file() && Lower(PathText(entry.path().extension())) == ".nif")
                {
                    const auto logical = Lower(PathText(entry.path().lexically_relative(input.gameData)));
                    if (!input.assets.looseModels.emplace(logical, entry.path()).second)
                    {
                        throw std::runtime_error("Duplicate deployed model filename: " + logical);
                    }
                }
            }
        }
        std::map<std::string, std::filesystem::path> archives;
        for (const auto &entry : std::filesystem::directory_iterator(input.gameData))
        {
            if (entry.is_regular_file() && Lower(PathText(entry.path().extension())) == ".bsa")
            {
                archives.emplace(Lower(PathText(entry.path().stem())), entry.path());
            }
        }
        const auto addArchives = [&](const std::string &stem)
        {
            for (auto iterator = archives.begin(); iterator != archives.end();)
            {
                if (iterator->first == stem || iterator->first.starts_with(stem + " - "))
                {
                    input.assets.archives.push_back(iterator->second);
                    iterator = archives.erase(iterator);
                }
                else
                {
                    ++iterator;
                }
            }
        };
        addArchives("skyrim");
        for (const auto &plugin : input.pluginPaths)
        {
            addArchives(Lower(PathText(plugin.stem())));
        }
    }
} // namespace

namespace navmesh::skyrim
{
    VortexInput ImportVortexDeployment(const std::filesystem::path &gameRoot,
                                       const std::filesystem::path &pluginsDirectory)
    {
        VortexInput input;
        input.gameRoot = std::filesystem::absolute(gameRoot).lexically_normal();
        input.gameData = input.gameRoot / "Data";
        input.pluginsDirectory = pluginsDirectory;
        try
        {
            if (input.pluginsDirectory.empty())
            {
                input.pluginsDirectory = DefaultPluginsDirectory();
            }
            input.pluginsDirectory = std::filesystem::absolute(input.pluginsDirectory).lexically_normal();
            if (!std::filesystem::is_directory(input.gameData))
            {
                throw std::runtime_error(
                    "Skyrim installation has no Data folder. Select the game folder deployed by Vortex.");
            }
            input.snapshotHash = Snapshot(input);
            ResolvePlugins(input);
            ResolveAssets(input);
        }
        catch (const std::exception &error)
        {
            input.pluginPaths.clear();
            input.assets = {};
            input.diagnostics.push_back({DiagnosticKind::InvalidPlugin, PathText(input.gameRoot), error.what()});
        }
        return input;
    }

    bool VortexSnapshotMatches(const VortexInput &input)
    {
        try
        {
            return !input.snapshotHash.empty() && input.snapshotHash == Snapshot(input);
        }
        catch (const std::exception &)
        {
            return false;
        }
    }

    bool WriteInputReport(const std::filesystem::path &outputPath, const VortexInput &input, bool includeAssetWinners)
    {
        std::ofstream output(outputPath, std::ios::binary | std::ios::trunc);
        if (!output)
        {
            return false;
        }
        const auto quoted = [](const std::string &text) { return "\"" + reproducibility::EscapeJson(text) + "\""; };
        output << "{\n  \"mod_manager\": \"vortex\",\n  \"game_root\": " << quoted(PathText(input.gameRoot))
               << ",\n  \"game_data\": " << quoted(PathText(input.gameData))
               << ",\n  \"plugins_directory\": " << quoted(PathText(input.pluginsDirectory))
               << ",\n  \"snapshot_hash\": " << quoted(input.snapshotHash)
               << ",\n  \"limitations\": [\"Uses the active deployed Vortex profile. Loose-only mod changes cannot be "
                  "attributed "
                  "to a plugin; select their cells explicitly. Custom INI archive lists are not imported.\"],"
               << "\n  \"active_plugins\": [";
        for (std::size_t index = 0; index < input.pluginPaths.size(); ++index)
        {
            output << (index ? "," : "") << "\n    " << quoted(PathText(input.pluginPaths[index]));
        }
        output << "\n  ],\n  \"archives\": [";
        for (std::size_t index = 0; index < input.assets.archives.size(); ++index)
        {
            output << (index ? "," : "") << "\n    " << quoted(PathText(input.assets.archives[index]));
        }
        output << "\n  ],\n  \"loose_asset_count\": " << input.assets.looseModels.size()
               << ",\n  \"asset_winners_included\": " << (includeAssetWinners ? "true" : "false")
               << ",\n  \"loose_asset_winners\": [";
        if (includeAssetWinners)
        {
            // Stable report ordering is independent of the lookup map's hash buckets.
            const std::map<std::string, std::filesystem::path> ordered(input.assets.looseModels.begin(),
                                                                       input.assets.looseModels.end());
            bool first = true;
            for (const auto &[logical, physical] : ordered)
            {
                output << (first ? "" : ",") << "\n    {\"logical_path\": " << quoted(logical)
                       << ", \"physical_path\": " << quoted(PathText(physical)) << ", \"source\": \"Deployed Data\"}";
                first = false;
            }
        }
        output << "\n  ],\n  \"diagnostics\": [";
        for (std::size_t index = 0; index < input.diagnostics.size(); ++index)
        {
            const auto &diagnostic = input.diagnostics[index];
            output << (index ? "," : "") << "\n    {\"subject\": " << quoted(diagnostic.plugin)
                   << ", \"message\": " << quoted(diagnostic.message) << '}';
        }
        output << "\n  ]\n}\n";
        return static_cast<bool>(output);
    }
} // namespace navmesh::skyrim
