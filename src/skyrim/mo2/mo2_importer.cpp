#include "skyrim/mo2/mo2_importer.h"
#include "core/io/content_hash.h"

#include <algorithm>
#include <cctype>
#include <format>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <string_view>

namespace
{
    using namespace navmesh::skyrim;
    std::string Trim(std::string text)
    {
        const auto isTrim = [](const unsigned char c) { return std::isspace(c) != 0 || c == '\0'; };
        const auto first = std::find_if_not(text.begin(), text.end(), isTrim);
        if (first == text.end())
        {
            return {};
        }
        const auto last = std::find_if_not(text.rbegin(), text.rend(), isTrim).base();
        return {first, last};
    }
    std::string Lower(std::string text)
    {
        std::transform(text.begin(), text.end(), text.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return text;
    }
    std::string Text(const std::vector<std::uint8_t> &bytes, std::size_t offset, std::size_t size)
    {
        return offset <= bytes.size() && size <= bytes.size() - offset
                   ? std::string(reinterpret_cast<const char *>(bytes.data() + offset), size)
                   : "";
    }
    std::string PathUtf8(const std::filesystem::path &path)
    {
        const auto value = path.generic_u8string();
        return {reinterpret_cast<const char *>(value.data()), value.size()};
    }
    std::string Escape(const std::string &text)
    {
        constexpr char hex[] = "0123456789abcdef";
        std::string out;
        for (const unsigned char c : text)
        {
            if (c == '\\')
            {
                out += "\\\\";
            }
            else if (c == '\"')
            {
                out += "\\\"";
            }
            else if (c == '\n')
            {
                out += "\\n";
            }
            else if (c == '\r')
            {
                out += "\\r";
            }
            else if (c == '\t')
            {
                out += "\\t";
            }
            else if (c < 0x20)
            {
                out += "\\u00";
                out += hex[c >> 4];
                out += hex[c & 0x0F];
            }
            else
            {
                out += static_cast<char>(c);
            }
        }
        return out;
    }
    std::string DecodeQtPath(std::string value)
    {
        constexpr std::string_view prefix = "@ByteArray(";
        value = Trim(std::move(value));
        if (value.starts_with(prefix) && value.ends_with(')'))
        {
            value = value.substr(prefix.size(), value.size() - prefix.size() - 1);
        }
        std::string decoded;
        decoded.reserve(value.size());
        for (std::size_t index = 0; index < value.size(); ++index)
        {
            if (value[index] == '\\' && index + 1 < value.size() && value[index + 1] == '\\')
            {
                decoded.push_back('\\');
                ++index;
            }
            else
            {
                decoded.push_back(value[index]);
            }
        }
        // QSettings serializes Windows paths in @ByteArray values with escaped
        // backslashes. Normalize after unescaping so both portable and global
        // MO2 instances hand std::filesystem a canonical absolute path.
        std::replace(decoded.begin(), decoded.end(), '\\', '/');
        return decoded;
    }
    std::string ReadIniText(const std::filesystem::path &path)
    {
        std::ifstream input(path, std::ios::binary);
        std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
        if (bytes.size() >= 2 && ((bytes[0] == 0xFF && bytes[1] == 0xFE) || (bytes[0] == 0xFE && bytes[1] == 0xFF)))
        {
            const bool littleEndian = bytes[0] == 0xFF;
            std::string utf8;
            for (std::size_t index = 2; index + 1 < bytes.size(); index += 2)
            {
                const auto unit = static_cast<std::uint16_t>(
                    littleEndian ? bytes[index] | (static_cast<std::uint16_t>(bytes[index + 1]) << 8)
                                 : bytes[index + 1] | (static_cast<std::uint16_t>(bytes[index]) << 8));
                if (unit < 0x80)
                {
                    utf8.push_back(static_cast<char>(unit));
                }
                else if (unit < 0x800)
                {
                    utf8.push_back(static_cast<char>(0xC0 | (unit >> 6)));
                    utf8.push_back(static_cast<char>(0x80 | (unit & 0x3F)));
                }
                else
                {
                    utf8.push_back(static_cast<char>(0xE0 | (unit >> 12)));
                    utf8.push_back(static_cast<char>(0x80 | ((unit >> 6) & 0x3F)));
                    utf8.push_back(static_cast<char>(0x80 | (unit & 0x3F)));
                }
            }
            return utf8;
        }
        if (bytes.size() >= 3 && bytes[0] == 0xEF && bytes[1] == 0xBB && bytes[2] == 0xBF)
        {
            return std::string(reinterpret_cast<const char *>(bytes.data() + 3), bytes.size() - 3);
        }
        return std::string(reinterpret_cast<const char *>(bytes.data()), bytes.size());
    }
    std::map<std::string, std::string> ReadIni(const std::filesystem::path &path)
    {
        std::map<std::string, std::string> values;
        std::istringstream input(ReadIniText(path));
        std::string line;
        while (std::getline(input, line))
        {
            line = Trim(line);
            if (line.empty() || line[0] == '#' || line[0] == ';' || line[0] == '[')
            {
                continue;
            }
            const auto equals = line.find('=');
            if (equals != std::string::npos)
            {
                values[Lower(Trim(line.substr(0, equals)))] = DecodeQtPath(Trim(line.substr(equals + 1)));
            }
        }
        return values;
    }
    std::filesystem::path ConfigPath(const std::map<std::string, std::string> &ini, const std::string &key,
                                     const std::filesystem::path &root, const std::filesystem::path &base,
                                     const std::filesystem::path &fallback)
    {
        const auto found = ini.find(Lower(key));
        if (found == ini.end() || found->second.empty())
        {
            return fallback;
        }
        const std::filesystem::path value(found->second);
        if (value.is_absolute())
        {
            return value.lexically_normal();
        }
        const auto rootRelative = root / value;
        const auto baseRelative = base / value;
        // MO2 portable roots resolve paths from the executable; split instances often use base_directory.
        if (std::filesystem::exists(rootRelative) || !std::filesystem::exists(baseRelative))
        {
            return rootRelative.lexically_normal();
        }
        return baseRelative.lexically_normal();
    }
    std::uint64_t Fnv(std::uint64_t value, const std::string &text)
    {
        for (const auto c : text)
        {
            value ^= static_cast<unsigned char>(c);
            value *= 1099511628211ULL;
        }
        return value;
    }
    void AddSnapshot(std::uint64_t &hash, const std::filesystem::path &path)
    {
        hash = Fnv(hash, PathUtf8(path.lexically_normal()));
        std::ifstream input(path, std::ios::binary);
        std::ostringstream data;
        data << input.rdbuf();
        hash = Fnv(hash, data.str());
        if (std::filesystem::exists(path))
        {
            hash = Fnv(hash, std::to_string(std::filesystem::last_write_time(path).time_since_epoch().count()));
        }
    }
    void AddFiles(std::map<std::string, VirtualFile> &winners, const std::filesystem::path &root,
                  const std::string &source)
    {
        if (!std::filesystem::is_directory(root))
        {
            return;
        }
        std::error_code error;
        std::filesystem::recursive_directory_iterator iterator(
            root, std::filesystem::directory_options::skip_permission_denied, error),
            end;
        while (iterator != end)
        {
            if (error)
            {
                error.clear();
                iterator.increment(error);
                continue;
            }
            const auto entry = *iterator;
            if (entry.is_regular_file(error) && !error)
            {
                const auto logical = Lower(PathUtf8(entry.path().lexically_relative(root)));
                winners[logical] = {logical, entry.path(), source};
            }
            error.clear();
            iterator.increment(error);
        }
    }
    std::filesystem::path LooseAssetCachePath(const std::filesystem::path &directory, const std::string &snapshotHash)
    {
        return directory / ("loose-assets-" + snapshotHash + ".tsv");
    }
    bool ReadLooseAssetCache(const std::filesystem::path &path, std::vector<VirtualFile> &files)
    {
        std::vector<VirtualFile> cached;
        std::ifstream input(path, std::ios::binary);
        std::string line;
        if (!input || !std::getline(input, line) || line != "navmesh-mo2-loose-assets-v1")
        {
            return false;
        }
        while (std::getline(input, line))
        {
            const auto first = line.find('\t');
            const auto second = first == std::string::npos ? std::string::npos : line.find('\t', first + 1);
            if (first == std::string::npos || second == std::string::npos)
            {
                return false;
            }
            VirtualFile file{line.substr(0, first), std::filesystem::path(line.substr(first + 1, second - first - 1)),
                             line.substr(second + 1)};
            if (!std::filesystem::is_regular_file(file.physicalPath))
            {
                return false;
            }
            cached.push_back(std::move(file));
        }
        if (!static_cast<bool>(input) && !input.eof())
        {
            return false;
        }
        files = std::move(cached);
        std::error_code error;
        std::filesystem::last_write_time(path, std::filesystem::file_time_type::clock::now(), error);
        return true;
    }
    void WriteLooseAssetCache(const std::filesystem::path &path, const std::vector<VirtualFile> &files)
    {
        std::error_code error;
        std::filesystem::create_directories(path.parent_path(), error);
        if (error)
        {
            return;
        }
        std::ofstream output(path, std::ios::trunc | std::ios::binary);
        if (!output)
        {
            return;
        }
        output << "navmesh-mo2-loose-assets-v1\n";
        for (const auto &file : files)
        {
            output << file.logicalPath << '\t' << PathUtf8(file.physicalPath) << '\t' << file.source << '\n';
        }
    }
    void AddTopLevelPluginAndArchiveFiles(std::map<std::string, VirtualFile> &winners,
                                          const std::filesystem::path &root, const std::string &source)
    {
        std::error_code error;
        if (!std::filesystem::is_directory(root, error))
        {
            return;
        }
        for (std::filesystem::directory_iterator
                 iterator(root, std::filesystem::directory_options::skip_permission_denied, error),
             end;
             iterator != end; iterator.increment(error))
        {
            if (error)
            {
                error.clear();
                continue;
            }
            const auto &entry = *iterator;
            if (!entry.is_regular_file(error) || error)
            {
                error.clear();
                continue;
            }
            const auto extension = Lower(PathUtf8(entry.path().extension()));
            if (extension == ".esm" || extension == ".esp" || extension == ".esl" || extension == ".bsa")
            {
                const auto logical = Lower(PathUtf8(entry.path().filename()));
                winners[logical] = {logical, entry.path(), source};
            }
        }
    }
    bool PluginName(const std::filesystem::path &path)
    {
        const auto ext = Lower(PathUtf8(path.extension()));
        return ext == ".esm" || ext == ".esp" || ext == ".esl";
    }
    std::uint16_t U16(const std::vector<std::uint8_t> &bytes, std::size_t offset)
    {
        return static_cast<std::uint16_t>(bytes[offset]) | static_cast<std::uint16_t>(bytes[offset + 1]) << 8;
    }
    std::uint32_t U32(const std::vector<std::uint8_t> &bytes, std::size_t offset)
    {
        return static_cast<std::uint32_t>(bytes[offset]) | static_cast<std::uint32_t>(bytes[offset + 1]) << 8 |
               static_cast<std::uint32_t>(bytes[offset + 2]) << 16 |
               static_cast<std::uint32_t>(bytes[offset + 3]) << 24;
    }
    bool Has(const std::vector<std::uint8_t> &bytes, std::size_t offset, std::size_t size)
    {
        return offset <= bytes.size() && size <= bytes.size() - offset;
    }
    std::vector<std::string> ReadTes4Masters(const std::filesystem::path &path)
    {
        std::ifstream file(path, std::ios::binary);
        std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(file)), {});
        if (!Has(bytes, 0, 24) || Text(bytes, 0, 4) != "TES4")
        {
            return {};
        }
        const auto end = static_cast<std::size_t>(24) + U32(bytes, 4);
        if (end > bytes.size())
        {
            return {};
        }
        std::vector<std::string> masters;
        for (std::size_t offset = 24; offset < end;)
        {
            if (!Has(bytes, offset, 6))
            {
                return {};
            }
            const auto type = Text(bytes, offset, 4);
            const auto size = U16(bytes, offset + 4);
            const auto data = offset + 6;
            if (!Has(bytes, data, size))
            {
                return {};
            }
            if (type == "MAST")
            {
                masters.push_back(Trim(Text(bytes, data, size)));
            }
            offset = data + size;
        }
        return masters;
    }
} // namespace

namespace navmesh::skyrim
{
    bool ProfileSnapshotMatches(const Mo2ProfileInput &input)
    {
        std::uint64_t hash = 1469598103934665603ULL;
        for (const auto &path : {input.instanceRoot / "ModOrganizer.ini", input.profileDirectory / "modlist.txt",
                                 input.profileDirectory / "plugins.txt", input.profileDirectory / "loadorder.txt"})
        {
            AddSnapshot(hash, path);
        }
        std::ostringstream digest;
        digest << std::hex << std::setw(16) << std::setfill('0') << hash;
        return digest.str() == input.snapshotHash;
    }

    Mo2ProfileInput ImportMo2Profile(const std::filesystem::path &requestedRoot, const std::string &profile,
                                     const std::optional<std::filesystem::path> &modsDirectoryOverride,
                                     const std::filesystem::path &cacheDirectory)
    {
        Mo2ProfileInput result{.instanceRoot = requestedRoot, .profile = profile};
        const auto iniPath = requestedRoot / "ModOrganizer.ini";
        if (!std::filesystem::is_regular_file(iniPath))
        {
            result.diagnostics.push_back({DiagnosticKind::InvalidPlugin, PathUtf8(requestedRoot),
                                          "MO2 root does not contain ModOrganizer.ini."});
            return result;
        }
        const auto ini = ReadIni(iniPath);
        const auto siblingStorage = requestedRoot.parent_path() / "MODS";
        const auto base = ConfigPath(ini, "base_directory", requestedRoot, requestedRoot,
                                     std::filesystem::is_directory(siblingStorage) ? siblingStorage : requestedRoot);
        const auto profiles = ConfigPath(ini, "profiles_directory", requestedRoot, base,
                                         std::filesystem::is_directory(base / "profiles") ? base / "profiles"
                                                                                          : requestedRoot / "profiles");
        const auto configuredMods =
            ConfigPath(ini, "mods_directory", requestedRoot, base,
                       std::filesystem::is_directory(base / "mods") ? base / "mods" : requestedRoot / "mods");
        const auto mods = modsDirectoryOverride ? *modsDirectoryOverride : configuredMods;
        const auto overwrite = ConfigPath(
            ini, "overwrite_directory", requestedRoot, base,
            std::filesystem::is_directory(base / "overwrite") ? base / "overwrite" : requestedRoot / "overwrite");
        const auto game = ConfigPath(ini, "gamepath", requestedRoot, base, {});
        result.gameData = std::filesystem::is_directory(game / "Data") ? game / "Data" : game;
        result.profileDirectory = profiles / profile;
        result.modsDirectory = mods;
        if (game.empty())
        {
            const auto configured = ini.find("gamepath");
            std::string keys;
            for (const auto &[key, _] : ini)
            {
                if (!keys.empty())
                {
                    keys += ", ";
                }
                keys += key;
            }
            result.diagnostics.push_back({DiagnosticKind::InvalidPlugin, "gamePath",
                                          configured == ini.end()
                                              ? "MO2 gamePath is absent from ModOrganizer.ini; parsed keys: " + keys
                                              : "MO2 gamePath decoded to an empty path."});
        }
        if (modsDirectoryOverride && !std::filesystem::is_directory(mods))
        {
            result.diagnostics.push_back(
                {DiagnosticKind::InvalidPlugin, PathUtf8(mods), "--mods-dir does not name an existing directory."});
        }
        const auto modlist = result.profileDirectory / "modlist.txt";
        const auto plugins = result.profileDirectory / "plugins.txt";
        const auto loadorder = result.profileDirectory / "loadorder.txt";
        for (const auto &file : {modlist, plugins, loadorder})
        {
            if (!std::filesystem::is_regular_file(file))
            {
                result.diagnostics.push_back(
                    {DiagnosticKind::InvalidPlugin, PathUtf8(file), "Required MO2 profile file is missing."});
            }
        }
        if (!std::filesystem::is_directory(result.gameData))
        {
            result.diagnostics.push_back({DiagnosticKind::InvalidPlugin, PathUtf8(result.gameData),
                                          "Configured game Data directory is missing; check MO2 gamePath."});
        }
        std::uint64_t hash = 1469598103934665603ULL;
        for (const auto &file : {iniPath, modlist, plugins, loadorder})
        {
            AddSnapshot(hash, file);
        }
        std::ifstream modInput(modlist);
        std::string line;
        std::size_t priority{};
        while (std::getline(modInput, line))
        {
            line = Trim(line);
            if (line.size() < 2 || line[0] != '+')
            {
                continue;
            }
            const auto name = Trim(line.substr(1));
            const auto path = mods / name;
            result.enabledMods.push_back({name, path, priority++});
            if (!std::filesystem::is_directory(path))
            {
                result.diagnostics.push_back(
                    {DiagnosticKind::InvalidPlugin, name, "Enabled MO2 mod directory is missing: " + PathUtf8(path)});
            }
        }
        std::set<std::string> active;
        std::ifstream activeInput(plugins);
        while (std::getline(activeInput, line))
        {
            line = Trim(line);
            if (line.size() > 1 && line[0] == '*')
            {
                active.insert(Lower(Trim(line.substr(1))));
            }
        }
        std::vector<std::string> order;
        std::ifstream orderInput(loadorder);
        while (std::getline(orderInput, line))
        {
            line = Trim(line);
            if (!line.empty() && active.contains(Lower(line)))
            {
                order.push_back(line);
            }
        }
        for (const auto &plugin : active)
        {
            if (std::none_of(order.begin(), order.end(), [&](const auto &p) { return Lower(p) == plugin; }))
            {
                result.diagnostics.push_back(
                    {DiagnosticKind::UnsupportedRecord, plugin, "Active plugin is absent from loadorder.txt."});
            }
        }
        std::ostringstream digest;
        digest << std::hex << std::setw(16) << std::setfill('0') << hash;
        result.snapshotHash = digest.str();
        std::map<std::string, VirtualFile> winners;
        // Profile identity alone cannot detect loose files added to an enabled mod. Directory stamps
        // cover catalog membership changes; NIF content revisions are checked by geometry cache keys.
        core::ContentHash catalogHash;
        catalogHash.Add("navmesh-mo2-catalog-2:" + result.snapshotHash);
        std::vector<std::filesystem::path> roots{result.gameData};
        for (const auto &mod : result.enabledMods)
        {
            roots.push_back(mod.path);
        }
        roots.push_back(overwrite);
        result.looseAssetRoots = roots;
        for (const auto &root : roots)
        {
            catalogHash.Add("\n" + PathUtf8(root));
            std::error_code error;
            catalogHash.Add(std::to_string(std::filesystem::last_write_time(root, error).time_since_epoch().count()));
            if (std::filesystem::is_directory(root, error))
            {
                for (auto iterator = std::filesystem::recursive_directory_iterator(
                         root, std::filesystem::directory_options::skip_permission_denied, error);
                     !error && iterator != std::filesystem::recursive_directory_iterator(); iterator.increment(error))
                {
                    if (iterator->is_directory(error))
                    {
                        catalogHash.Add("\n" + PathUtf8(iterator->path()));
                        catalogHash.Add(std::to_string(iterator->last_write_time(error).time_since_epoch().count()));
                    }
                }
            }
        }
        const auto cachePath =
            cacheDirectory.empty() ? std::filesystem::path{} : LooseAssetCachePath(cacheDirectory, catalogHash.Hex());
        result.looseAssetCachePath = cachePath;
        const auto cacheLoaded = !cachePath.empty() && ReadLooseAssetCache(cachePath, result.looseAssetWinners);
        result.looseAssetCacheUsed = cacheLoaded;
        if (cacheLoaded)
        {
            for (const auto &file : result.looseAssetWinners)
            {
                winners[file.logicalPath] = file;
            }
        }
        else
        {
            AddFiles(winners, result.gameData, "game Data");
            for (const auto &mod : result.enabledMods)
            {
                AddFiles(winners, mod.path, mod.name);
            }
            AddFiles(winners, overwrite, "Overwrite");
        }
        AddTopLevelPluginAndArchiveFiles(winners, result.gameData, "game Data");
        for (const auto &mod : result.enabledMods)
        {
            AddTopLevelPluginAndArchiveFiles(winners, mod.path, mod.name);
        }
        AddTopLevelPluginAndArchiveFiles(winners, overwrite, "Overwrite");
        for (const auto &[logical, file] : winners)
        {
            if (Lower(PathUtf8(file.physicalPath.extension())) == ".bsa")
            {
                continue;
            }
            if (PluginName(file.physicalPath))
            {
                continue;
            }
            if (!cacheLoaded)
            {
                result.looseAssetWinners.push_back(file);
            }
        }
        // Preserve archive priority for on-demand model extraction. The winner
        // map alone is sorted by logical name, which loses MO2 mod priority.
        const auto addArchives = [&](const std::filesystem::path &directory)
        {
            if (!std::filesystem::is_directory(directory))
            {
                return;
            }
            std::vector<std::filesystem::path> matched;
            for (const auto &entry : std::filesystem::directory_iterator(directory))
            {
                if (!entry.is_regular_file() || Lower(PathUtf8(entry.path().extension())) != ".bsa")
                {
                    continue;
                }
                const auto winner = winners.find(Lower(PathUtf8(entry.path().filename())));
                if (winner != winners.end() && winner->second.physicalPath == entry.path())
                {
                    matched.push_back(entry.path());
                }
            }
            std::sort(matched.begin(), matched.end(), [](const auto &a, const auto &b)
                      { return Lower(PathUtf8(a.filename())) < Lower(PathUtf8(b.filename())); });
            result.archivePaths.insert(result.archivePaths.end(), matched.begin(), matched.end());
        };
        addArchives(result.gameData);
        for (const auto &mod : result.enabledMods)
        {
            addArchives(mod.path);
        }
        addArchives(overwrite);
        // Skyrim's core masters may be implicit in an MO2 profile. Derive the
        // complete master closure from on-disk TES4 headers and place each master
        // before its dependant without changing the profile or its files.
        std::set<std::string> included;
        for (const auto &name : order)
        {
            included.insert(Lower(name));
        }
        std::set<std::string> reportedMissing;
        for (std::size_t index = 0; index < order.size();)
        {
            const auto plugin = winners.find(Lower(order[index]));
            if (plugin == winners.end())
            {
                ++index;
                continue;
            }
            bool insertedMaster{};
            for (const auto &master : ReadTes4Masters(plugin->second.physicalPath))
            {
                const auto masterKey = Lower(master);
                if (included.contains(masterKey))
                {
                    continue;
                }
                const auto source = winners.find(masterKey);
                if (source == winners.end())
                {
                    if (reportedMissing.insert(masterKey).second)
                    {
                        result.diagnostics.push_back(
                            {DiagnosticKind::MissingMaster, order[index],
                             "Required master has no physical winner in MO2's virtual file map: '" + master + "'."});
                    }
                    continue;
                }
                order.insert(order.begin() + static_cast<std::ptrdiff_t>(index), master);
                included.insert(masterKey);
                insertedMaster = true;
                break;
            }
            if (!insertedMaster)
            {
                ++index;
            }
        }
        for (const auto &name : order)
        {
            const auto it = winners.find(Lower(name));
            if (it == winners.end())
            {
                result.diagnostics.push_back({DiagnosticKind::MissingMaster, name,
                                              "Active plugin has no physical winner in MO2's virtual file map."});
            }
            else
            {
                result.pluginPaths.push_back(it->second.physicalPath);
            }
        }
        if (!cachePath.empty() && !cacheLoaded)
        {
            WriteLooseAssetCache(cachePath, result.looseAssetWinners);
        }
        return result;
    }

    namespace
    {
        void WriteJson(std::ostream &json, const Mo2ProfileInput &input, bool includeAssetWinners = true)
        {
            json << "{\n  \"snapshot_hash\": \"" << input.snapshotHash
                 << "\",\n  \"loose_asset_cache_used\": " << (input.looseAssetCacheUsed ? "true" : "false")
                 << ",\n  \"profile\": \"" << Escape(input.profile) << "\",\n  \"instance_root\": \""
                 << Escape(PathUtf8(input.instanceRoot)) << "\",\n  \"game_data\": \""
                 << Escape(PathUtf8(input.gameData)) << "\",\n  \"mods_directory\": \""
                 << Escape(PathUtf8(input.modsDirectory)) << "\",\n  \"profile_files\": [\n";
            const std::vector<std::filesystem::path> profileFiles = {input.profileDirectory / "modlist.txt",
                                                                     input.profileDirectory / "plugins.txt",
                                                                     input.profileDirectory / "loadorder.txt"};
            for (std::size_t i = 0; i < profileFiles.size(); ++i)
            {
                const auto exists = std::filesystem::exists(profileFiles[i]);
                const auto stamp =
                    exists
                        ? std::to_string(std::filesystem::last_write_time(profileFiles[i]).time_since_epoch().count())
                        : "missing";
                json << std::format("    {{\"path\": \"{}\", \"timestamp\": \"{}\"}}{}\n",
                                    Escape(PathUtf8(profileFiles[i])), stamp, i + 1 == profileFiles.size() ? "" : ",");
            }
            json << "  ],\n  \"active_plugins\": [\n";
            for (std::size_t i = 0; i < input.pluginPaths.size(); ++i)
            {
                json << "    \"" << Escape(PathUtf8(input.pluginPaths[i])) << "\""
                     << (i + 1 == input.pluginPaths.size() ? "" : ",") << "\n";
            }
            json << "  ],\n  \"enabled_mods\": [\n";
            for (std::size_t i = 0; i < input.enabledMods.size(); ++i)
            {
                const auto &mod = input.enabledMods[i];
                json << std::format("    {{\"priority\": {}, \"name\": \"{}\", \"path\": \"{}\"}}{}\n", mod.priority,
                                    Escape(mod.name), Escape(PathUtf8(mod.path)),
                                    i + 1 == input.enabledMods.size() ? "" : ",");
            }
            json << "  ],\n  \"loose_asset_winners\": [\n";
            const auto winnerCount = includeAssetWinners ? input.looseAssetWinners.size() : 0;
            for (std::size_t i = 0; i < winnerCount; ++i)
            {
                const auto &file = input.looseAssetWinners[i];
                json << std::format(
                    "    {{\"logical_path\": \"{}\", \"physical_path\": \"{}\", \"source\": \"{}\"}}{}\n",
                    Escape(file.logicalPath), Escape(PathUtf8(file.physicalPath)), Escape(file.source),
                    i + 1 == input.looseAssetWinners.size() ? "" : ",");
            }
            json << "  ],\n  \"loose_asset_count\": " << input.looseAssetWinners.size()
                 << ",\n  \"loose_asset_catalog\": \"" << Escape(PathUtf8(input.looseAssetCachePath))
                 << "\",\n  \"asset_winners_included\": " << (includeAssetWinners ? "true" : "false")
                 << ",\n  \"archives\": [\n";
            for (std::size_t i{}; i < input.archivePaths.size(); ++i)
            {
                json << "    \"" << Escape(PathUtf8(input.archivePaths[i])) << "\""
                     << (i + 1 == input.archivePaths.size() ? "" : ",") << '\n';
            }
            json << "  ],\n  \"diagnostics\": [\n";
            for (std::size_t i = 0; i < input.diagnostics.size(); ++i)
            {
                const auto &d = input.diagnostics[i];
                json << std::format("    {{\"subject\": \"{}\", \"message\": \"{}\"}}{}\n", Escape(d.plugin),
                                    Escape(d.message), i + 1 == input.diagnostics.size() ? "" : ",");
            }
            json << "  ]\n}\n";
        }
    } // namespace

    bool WriteInputReport(const std::filesystem::path &outputPath, const Mo2ProfileInput &input,
                          bool includeAssetWinners)
    {
        std::ofstream output(outputPath, std::ios::trunc | std::ios::binary);
        if (!output)
        {
            return false;
        }
        WriteJson(output, input, includeAssetWinners);
        return static_cast<bool>(output);
    }

    std::string ToJson(const Mo2ProfileInput &input)
    {
        std::ostringstream json;
        WriteJson(json, input);
        return json.str();
    }
} // namespace navmesh::skyrim
