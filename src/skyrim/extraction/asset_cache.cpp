#include "skyrim/extraction/asset_cache.h"

#include "core/io/content_hash.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>

namespace navmesh::skyrim::offline
{
    std::string QuoteAssetPath(const std::filesystem::path &path)
    {
        auto value = path.string();
#ifdef _WIN32
        // Windows filenames cannot contain a quote. Reject shell expansions rather than interpreting paths as code.
        if (value.find_first_of("\"\r\n%") != std::string::npos)
        {
            throw std::invalid_argument("Asset helper path contains shell expansion characters");
        }
        return "\"" + value + "\"";
#else
        std::string quoted{"'"};
        for (const auto character : value)
        {
            quoted += character == '\'' ? "'\"'\"'" : std::string(1, character);
        }
        return quoted + "'";
#endif
    }

    bool RunAssetHelper(const std::string &arguments)
    {
        const auto script = std::filesystem::current_path() / "tools" / "extract_bsa_models.py";
        if (!std::filesystem::exists(script))
        {
            std::cerr << "Cannot locate tools/extract_bsa_models.py; launch from the application project directory.\n";
            return false;
        }
        std::string python = "python";
#ifdef _WIN32
        char *configured{};
        std::size_t size{};
        if (_dupenv_s(&configured, &size, "NAVMESH_PYTHON") == 0 && configured && *configured)
        {
            python = QuoteAssetPath(configured);
        }
        std::free(configured);
#else
        if (const auto *configured = std::getenv("NAVMESH_PYTHON"); configured && *configured)
        {
            python = QuoteAssetPath(configured);
        }
#endif
        return std::system((python + " " + QuoteAssetPath(script) + " " + arguments).c_str()) == 0;
    }

    std::filesystem::path ModelAssetCacheDirectory(const std::filesystem::path &dataDirectory,
                                                   const ModelAssetSources *assets,
                                                   const std::filesystem::path &cacheRoot)
    {
        std::vector<std::filesystem::path> archives;
        if (assets)
        {
            archives = assets->archives;
        }
        else if (std::filesystem::is_directory(dataDirectory))
        {
            for (const auto &entry : std::filesystem::directory_iterator(dataDirectory))
            {
                auto extension = entry.path().extension().string();
                std::transform(extension.begin(), extension.end(), extension.begin(),
                               [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
                if (entry.is_regular_file() && extension == ".bsa")
                {
                    archives.push_back(entry.path());
                }
            }
            std::sort(archives.begin(), archives.end());
        }
        core::ContentHash hash;
        hash.Add("navmesh-assets-schema-1\n");
        hash.Add(std::filesystem::absolute(dataDirectory).lexically_normal().generic_string());
        for (const auto &archive : archives)
        {
            hash.Add("\n");
            hash.Add(std::filesystem::absolute(archive).lexically_normal().generic_string());
            std::error_code error;
            hash.Add(":" + std::to_string(std::filesystem::file_size(archive, error)));
            hash.Add(":" + std::to_string(std::filesystem::last_write_time(archive, error).time_since_epoch().count()));
        }
        const auto root =
            cacheRoot.empty() ? std::filesystem::temp_directory_path() / "NavmeshGenerator" / "assets" : cacheRoot;
        const auto snapshot = root / hash.Hex();
        std::filesystem::create_directories(snapshot);
        std::ofstream marker(snapshot / ".navmesh-assets.json", std::ios::trunc);
        marker << "{\"schema\":1}";
        std::ofstream manifest(snapshot / "archives.txt", std::ios::trunc);
        for (const auto &archive : archives)
        {
            manifest << archive.string() << '\n';
        }
        if (!marker || !manifest)
        {
            throw std::runtime_error("Cannot prepare shared model asset cache");
        }
        return snapshot;
    }

    bool ChangedArchiveModels(const std::filesystem::path &dataDirectory, const ModelAssetSources &assets,
                              const std::filesystem::path &snapshot,
                              const std::set<std::filesystem::path> &changedArchives, std::set<std::string> &models)
    {
        if (changedArchives.empty())
        {
            return true;
        }
        const auto changed = snapshot / "changed-archives.txt";
        const auto output = snapshot / "changed-models.txt";
        std::ofstream stream(changed, std::ios::trunc);
        for (const auto &archive : changedArchives)
        {
            stream << archive.string() << '\n';
        }
        stream.close();
        if (!RunAssetHelper("--data " + QuoteAssetPath(dataDirectory) + " --output " + QuoteAssetPath(snapshot) +
                            " --archives " + QuoteAssetPath(snapshot / "archives.txt") + " --changed-archives " +
                            QuoteAssetPath(changed) + " --changed-models " + QuoteAssetPath(output)))
        {
            return false;
        }
        std::ifstream names(output);
        if (!names)
        {
            return false;
        }
        for (std::string name; std::getline(names, name);)
        {
            std::replace(name.begin(), name.end(), '\\', '/');
            if (!assets.looseModels.contains(name))
            {
                models.insert(name);
            }
        }
        return names.eof();
    }

    namespace
    {
        bool IsDigest(std::string_view value)
        {
            return (value.size() == 64 || value.size() == 16) &&
                   std::all_of(value.begin(), value.end(),
                               [](unsigned char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'); });
        }
        bool OwnedCacheFile(const std::filesystem::path &relative, bool snapshot)
        {
            const auto name = relative.filename().string();
            if (!snapshot)
            {
                return (relative.extension() == ".json" && IsDigest(relative.stem().string())) ||
                       (name.starts_with("loose-assets-") && name.ends_with(".tsv") &&
                        IsDigest(std::string_view(name).substr(13, name.size() - 17)));
            }
            const auto first = relative.begin()->string();
            return (first == "meshes" && relative.extension() == ".nif") ||
                   (first == "candidates" && relative.extension() == ".gz" && IsDigest(relative.stem().string())) ||
                   name == "archives.txt" || name == "changed-archives.txt" || name == "changed-models.txt" ||
                   name == ".missing-models.txt";
        }
    } // namespace

    void TrimModelAssetCache(const std::filesystem::path &snapshot, std::size_t byteBudget, bool protectCandidates)
    {
        struct Entry
        {
            std::filesystem::path path;
            std::filesystem::file_time_type used;
            std::uintmax_t size{};
            bool protectedCandidate{};
        };
        std::error_code error;
        const auto root = std::filesystem::weakly_canonical(snapshot.parent_path(), error);
        if (error || !std::filesystem::is_directory(root))
        {
            return;
        }
        std::vector<Entry> entries;
        std::uintmax_t total{};
        for (const auto &directory : std::filesystem::directory_iterator(root))
        {
            if (!directory.is_directory(error) || directory.is_symlink(error))
            {
                continue;
            }
            const auto name = directory.path().filename().string();
            const bool ownedSnapshot = name.size() == 64 && IsDigest(name) &&
                                       std::filesystem::is_regular_file(directory.path() / ".navmesh-assets.json");
            if (!ownedSnapshot && name != ".indexes" && name != ".mo2")
            {
                continue;
            }
            for (auto iterator = std::filesystem::recursive_directory_iterator(
                     directory.path(), std::filesystem::directory_options::skip_permission_denied, error);
                 !error && iterator != std::filesystem::recursive_directory_iterator(); iterator.increment(error))
            {
                if (iterator->is_symlink(error))
                {
                    iterator.disable_recursion_pending();
                    continue;
                }
                if (!iterator->is_regular_file(error))
                {
                    continue;
                }
                const auto relative = iterator->path().lexically_relative(directory.path());
                if (!OwnedCacheFile(relative, ownedSnapshot))
                {
                    continue;
                }
                const auto size = iterator->file_size(error);
                const auto used = iterator->last_write_time(error);
                if (!error)
                {
                    const bool pinned = protectCandidates && *relative.begin() == "candidates";
                    entries.push_back({iterator->path(), used, size, pinned});
                    total += size;
                }
            }
            error.clear();
        }
        std::sort(entries.begin(), entries.end(),
                  [](const auto &left, const auto &right) { return left.used < right.used; });
        for (const auto &entry : entries)
        {
            if (total <= byteBudget)
            {
                break;
            }
            if (entry.protectedCandidate)
            {
                continue;
            }
            // Recheck canonical containment at the deletion boundary, including links introduced during a run.
            const auto target = std::filesystem::weakly_canonical(entry.path, error);
            const auto relative = target.lexically_relative(root);
            if (!error && !relative.empty() && *relative.begin() != ".." &&
                !std::filesystem::is_symlink(entry.path, error) && std::filesystem::remove(entry.path, error))
            {
                total -= entry.size;
            }
            error.clear();
        }
    }
} // namespace navmesh::skyrim::offline
