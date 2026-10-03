#include "skyrim/extraction/bsa_archive.h"

#include <lz4frame.h>
#include <zlib.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <stdexcept>
#include <string_view>
#include <unordered_map>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

namespace navmesh::skyrim
{
    namespace
    {
        constexpr std::uint64_t MaximumAssetBytes = 256ULL * 1024 * 1024;

        std::string Normalize(std::string value)
        {
            for (auto &character : value)
            {
                character =
                    character == '\\' ? '/' : static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
            }
            return value;
        }

        std::filesystem::path ModelPath(const std::string &logical)
        {
            if (!logical.starts_with("meshes/") || !logical.ends_with(".nif"))
            {
                throw std::runtime_error("Unsupported BSA model path: " + logical);
            }
            std::size_t start{};
            while (start < logical.size())
            {
                const auto end = logical.find('/', start);
                const auto part = logical.substr(start, end == std::string::npos ? end : end - start);
                if (part.empty() || part == "." || part == ".." || part.find(':') != std::string::npos ||
                    part.find('\0') != std::string::npos)
                {
                    throw std::runtime_error("Unsafe BSA model path: " + logical);
                }
                if (end == std::string::npos)
                {
                    break;
                }
                start = end + 1;
            }
            return std::filesystem::path(std::u8string(logical.begin(), logical.end()));
        }

        std::uint32_t U32(std::span<const char> bytes, std::size_t position)
        {
            std::uint32_t value{};
            for (std::size_t index{}; index < 4; ++index)
            {
                value |= static_cast<std::uint32_t>(static_cast<unsigned char>(bytes[position + index])) << (index * 8);
            }
            return value;
        }

        std::vector<char> Read(std::istream &stream, std::uint64_t count)
        {
            if (count > MaximumAssetBytes)
            {
                throw std::runtime_error("Unsafe BSA table or entry size");
            }
            std::vector<char> bytes(static_cast<std::size_t>(count));
            if (count && !stream.read(bytes.data(), static_cast<std::streamsize>(count)))
            {
                throw std::runtime_error("Truncated BSA table or entry");
            }
            return bytes;
        }

        struct Entry
        {
            std::uint32_t offset{}, size{};
            bool compressed{};
        };

        struct Index
        {
            std::uint32_t version{};
            bool prefixed{};
            std::unordered_map<std::string, Entry> entries;
        };

        Index ReadIndex(const std::filesystem::path &path, std::uint64_t &bytesRead)
        {
            const auto fileSize = std::filesystem::file_size(path);
            std::ifstream stream(path, std::ios::binary);
            const auto header = Read(stream, 36);
            const auto version = U32(header, 4);
            const auto offset = U32(header, 8);
            const auto flags = U32(header, 12);
            const auto folders = U32(header, 16);
            const auto files = U32(header, 20);
            const auto folderNames = U32(header, 24);
            const auto nameBytes = U32(header, 28);
            if (std::string_view(header.data(), 4) != std::string_view("BSA\0", 4) ||
                (version != 103 && version != 104 && version != 105) || (flags & 0x40) || (flags & 3) != 3)
            {
                throw std::runtime_error("Unsupported BSA header or unnamed directory tables");
            }
            const std::uint64_t recordSize = version == 105 ? 24 : 16;
            const auto tableBytes =
                folders * recordSize + static_cast<std::uint64_t>(files) * 16 + folderNames + folders + nameBytes;
            if (offset < 36 || offset > fileSize || tableBytes > fileSize - offset)
            {
                throw std::runtime_error("BSA counts exceed archive size");
            }

            // Named desktop BSAs store folder records, folder blocks, then one file-name
            // table in record order. Buffer only this metadata, never unrelated payloads.
            stream.seekg(offset);
            const auto table = Read(stream, tableBytes);
            bytesRead += 36 + tableBytes;
            std::uint64_t totalFiles{};
            for (std::uint32_t folder{}; folder < folders; ++folder)
            {
                totalFiles += U32(table, static_cast<std::size_t>(folder * recordSize + 8));
            }
            if (totalFiles != files)
            {
                throw std::runtime_error("BSA folder and file counts disagree");
            }
            Index output{.version = version, .prefixed = (flags & 0x100) != 0};
            std::size_t position = static_cast<std::size_t>(folders * recordSize);
            const auto namesStart = table.size() - nameBytes;
            std::size_t namePosition = namesStart;
            std::uint64_t actualFolderNames{};
            for (std::uint32_t folder{}; folder < folders; ++folder)
            {
                if (position >= namesStart)
                {
                    throw std::runtime_error("Truncated BSA folder block");
                }
                const auto length = static_cast<unsigned char>(table[position++]);
                if (!length || length > namesStart - position || table[position + length - 1] != '\0')
                {
                    throw std::runtime_error("Invalid BSA folder name");
                }
                const auto directory = Normalize(std::string(table.data() + position, length - 1));
                position += length;
                actualFolderNames += length;
                const auto count = U32(table, static_cast<std::size_t>(folder * recordSize + 8));
                if (static_cast<std::uint64_t>(count) * 16 > namesStart - position)
                {
                    throw std::runtime_error("Truncated BSA file records");
                }
                for (std::uint32_t file{}; file < count; ++file)
                {
                    const auto encodedSize = U32(table, position + 8);
                    const auto payloadOffset = U32(table, position + 12);
                    const auto size = encodedSize & 0x3fffffff;
                    position += 16;
                    if (payloadOffset > fileSize || size > fileSize - payloadOffset)
                    {
                        throw std::runtime_error("BSA entry exceeds archive size");
                    }
                    const auto end = std::find(table.begin() + namePosition, table.end(), '\0');
                    if (end == table.end())
                    {
                        throw std::runtime_error("Unterminated BSA filename");
                    }
                    const auto nameLength = static_cast<std::size_t>(end - (table.begin() + namePosition));
                    const auto logical =
                        directory + '/' + Normalize(std::string(table.data() + namePosition, nameLength));
                    namePosition += nameLength + 1;
                    if (logical.starts_with("meshes/") && logical.ends_with(".nif"))
                    {
                        (void)ModelPath(logical);
                        output.entries[logical] = {payloadOffset, size,
                                                   bool(flags & 4) != bool(encodedSize & 0x40000000)};
                    }
                }
            }
            if (position != namesStart || actualFolderNames != folderNames ||
                !std::all_of(table.begin() + namePosition, table.end(), [](char value) { return value == '\0'; }))
            {
                throw std::runtime_error("BSA name lengths disagree with directory tables");
            }
            return output;
        }

        std::span<const char> StripPrefix(const Index &index, std::span<const char> payload)
        {
            if (index.prefixed)
            {
                if (payload.empty() || payload.size() <= static_cast<unsigned char>(payload[0]))
                {
                    throw std::runtime_error("Invalid BSA filename prefix");
                }
                payload = payload.subspan(static_cast<unsigned char>(payload[0]) + 1);
            }
            return payload;
        }

        std::vector<char> Decode(const Index &index, std::span<const char> payload)
        {
            if (payload.size() < 4)
            {
                throw std::runtime_error("Missing BSA decoded size");
            }
            const auto expected = U32(payload, 0);
            if (expected > MaximumAssetBytes)
            {
                throw std::runtime_error("Unsafe BSA decoded size");
            }
            payload = payload.subspan(4);
            // One extra byte detects streams expanding beyond the declared decoded size.
            std::vector<char> output(static_cast<std::size_t>(expected) + 1);
            std::size_t decoded{};
            if (index.version == 105)
            {
                LZ4F_dctx *context{};
                if (LZ4F_isError(LZ4F_createDecompressionContext(&context, LZ4F_VERSION)))
                {
                    throw std::runtime_error("Cannot create BSA LZ4 decoder");
                }
                const auto owner = std::unique_ptr<LZ4F_dctx, decltype(&LZ4F_freeDecompressionContext)>(
                    context, &LZ4F_freeDecompressionContext);
                std::size_t consumed{};
                std::size_t remaining = 1;
                while (remaining && consumed < payload.size() && decoded < output.size())
                {
                    auto inputBytes = payload.size() - consumed;
                    auto outputBytes = output.size() - decoded;
                    remaining = LZ4F_decompress(context, output.data() + decoded, &outputBytes,
                                                payload.data() + consumed, &inputBytes, nullptr);
                    if (LZ4F_isError(remaining) || (!inputBytes && !outputBytes))
                    {
                        throw std::runtime_error("Invalid BSA LZ4 frame");
                    }
                    consumed += inputBytes;
                    decoded += outputBytes;
                }
                if (remaining || consumed != payload.size())
                {
                    throw std::runtime_error("Incomplete BSA LZ4 frame or trailing data");
                }
            }
            else
            {
                z_stream stream{};
                stream.next_in = reinterpret_cast<Bytef *>(const_cast<char *>(payload.data()));
                stream.avail_in = static_cast<uInt>(payload.size());
                stream.next_out = reinterpret_cast<Bytef *>(output.data());
                stream.avail_out = static_cast<uInt>(output.size());
                if (inflateInit(&stream) != Z_OK)
                {
                    throw std::runtime_error("Cannot create BSA zlib decoder");
                }
                const auto result = inflate(&stream, Z_FINISH);
                decoded = stream.total_out;
                const auto consumed = stream.total_in;
                inflateEnd(&stream);
                if (result != Z_STREAM_END || consumed != payload.size())
                {
                    throw std::runtime_error("Incomplete BSA zlib stream or trailing data");
                }
            }
            if (decoded != expected)
            {
                throw std::runtime_error("BSA decoded size mismatch");
            }
            output.resize(decoded);
            return output;
        }

        void Publish(const std::filesystem::path &path, std::span<const char> bytes)
        {
            static std::atomic<std::uint64_t> sequence{};
            auto temporary = path;
            temporary += "." + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "." +
                         std::to_string(++sequence) + ".tmp";
            struct Cleanup
            {
                std::filesystem::path path;
                ~Cleanup()
                {
                    std::error_code error;
                    std::filesystem::remove(path, error);
                }
            } cleanup{temporary};
            std::filesystem::create_directories(path.parent_path());
            std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
            stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
            stream.close();
            if (!stream)
            {
                throw std::runtime_error("Cannot write BSA cache file");
            }
            // Replace atomically so readers never observe a partially written model,
            // negative-cache list, or statistics file, including concurrent publishers.
#ifdef _WIN32
            if (!MoveFileExW(temporary.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING))
            {
                throw std::runtime_error("Cannot publish BSA cache file: " + std::to_string(GetLastError()));
            }
#else
            std::filesystem::rename(temporary, path);
#endif
        }

        void ExtractEntry(std::ifstream &stream, const std::filesystem::path &archive, const Index &index,
                          const Entry &entry, const std::filesystem::path &destination)
        {
            if (!stream.is_open())
            {
                stream.open(archive, std::ios::binary);
                if (!stream)
                {
                    throw std::runtime_error("Cannot open BSA payload stream");
                }
            }
            stream.clear();
            stream.seekg(entry.offset);
            const auto payload = Read(stream, entry.size);
            const auto bytes = StripPrefix(index, payload);
            if (entry.compressed)
            {
                const auto decoded = Decode(index, bytes);
                Publish(destination, decoded);
            }
            else
            {
                Publish(destination, bytes);
            }
        }

        std::string ArchiveKey(const std::filesystem::path &path)
        {
            auto key = std::filesystem::absolute(path).lexically_normal().generic_string();
#ifdef _WIN32
            key = Normalize(key);
#endif
            return key;
        }
    } // namespace

    struct BsaModelExtractor::Impl
    {
        struct Provider
        {
            std::filesystem::path path;
            std::optional<Index> index;
        };
        std::vector<Provider> providers;
        std::filesystem::path snapshot;
        std::set<std::string> missing;
        std::array<std::uint64_t, 4> statistics{};
        static constexpr std::array<const char *, 4> CounterNames = {"metadata_bytes_read", "payload_bytes_read",
                                                                     "entries_extracted", "index_hits"};

        Index &GetIndex(Provider &provider)
        {
            if (!provider.index)
            {
                provider.index = ReadIndex(provider.path, statistics[0]);
            }
            else
            {
                ++statistics[3];
            }
            return *provider.index;
        }

        bool ExtractProvider(Provider &provider, std::set<std::string> &remaining)
        {
            bool reliable = true;
            try
            {
                const auto &index = GetIndex(provider);
                std::ifstream stream;
                for (auto name = remaining.begin(); name != remaining.end();)
                {
                    const auto found = index.entries.find(*name);
                    if (found == index.entries.end())
                    {
                        ++name;
                        continue;
                    }
                    // Claim precedence before decoding: a broken winning entry cannot
                    // substitute different geometry from a lower-priority archive.
                    const auto logical = *name;
                    name = remaining.erase(name);
                    try
                    {
                        ExtractEntry(stream, provider.path, index, found->second, snapshot / ModelPath(logical));
                        statistics[1] += found->second.size;
                        ++statistics[2];
                    }
                    catch (const std::exception &error)
                    {
                        reliable = false;
                        std::cerr << "BSA extraction: unreadable winner " << provider.path.filename().string() << '/'
                                  << logical << ": " << error.what() << '\n';
                    }
                }
            }
            catch (const std::exception &error)
            {
                reliable = false;
                std::cerr << "BSA extraction: skipped " << provider.path.filename().string() << ": " << error.what()
                          << '\n';
            }
            return reliable;
        }

        void SaveStatistics() const
        {
            std::string json = "{";
            for (std::size_t index{}; index < statistics.size(); ++index)
            {
                json += (index ? "," : "") + std::string("\"") + CounterNames[index] +
                        "\":" + std::to_string(statistics[index]);
            }
            json += '}';
            Publish(snapshot / ".archive-statistics.json", json);
        }
    };

    BsaModelExtractor::BsaModelExtractor(std::vector<std::filesystem::path> archives, std::filesystem::path snapshot)
        : impl_(std::make_unique<Impl>())
    {
        impl_->snapshot = std::move(snapshot);
        std::filesystem::create_directories(impl_->snapshot);
        for (auto &path : archives)
        {
            impl_->providers.push_back({std::move(path), {}});
        }
        std::ifstream missing(impl_->snapshot / ".missing-models.txt");
        for (std::string name; std::getline(missing, name);)
        {
            impl_->missing.insert(Normalize(name));
        }
        std::ifstream statistics(impl_->snapshot / ".archive-statistics.json");
        const std::string json((std::istreambuf_iterator<char>(statistics)), {});
        for (std::size_t index{}; index < Impl::CounterNames.size(); ++index)
        {
            auto position = json.find(std::string("\"") + Impl::CounterNames[index] + '"');
            if (position != std::string::npos)
            {
                position = json.find(':', position);
                if (position != std::string::npos)
                {
                    position = json.find_first_not_of(" \t\r\n", position + 1);
                    if (position != std::string::npos)
                    {
                        std::from_chars(json.data() + position, json.data() + json.size(), impl_->statistics[index]);
                    }
                }
            }
        }
    }

    BsaModelExtractor::~BsaModelExtractor() = default;

    bool BsaModelExtractor::Extract(const std::set<std::string> &requested)
    {
        bool reliable = true;
        try
        {
            std::set<std::string> remaining;
            for (const auto &name : requested)
            {
                const auto logical = Normalize(name);
                const auto relative = ModelPath(logical);
                if (!impl_->missing.contains(logical) && !std::filesystem::is_regular_file(impl_->snapshot / relative))
                {
                    remaining.insert(logical);
                }
            }
            if (remaining.empty())
            {
                return true;
            }
            for (auto provider = impl_->providers.rbegin(); provider != impl_->providers.rend() && !remaining.empty();
                 ++provider)
            {
                reliable = impl_->ExtractProvider(*provider, remaining) && reliable;
            }
            if (reliable && !remaining.empty())
            {
                auto missing = impl_->missing;
                missing.insert(remaining.begin(), remaining.end());
                std::string names;
                for (auto name : missing)
                {
                    std::replace(name.begin(), name.end(), '/', '\\');
                    names += name + '\n';
                }
                Publish(impl_->snapshot / ".missing-models.txt", names);
                impl_->missing = std::move(missing);
            }
            impl_->SaveStatistics();
        }
        catch (const std::exception &error)
        {
            std::cerr << "BSA extraction: " << error.what() << '\n';
            reliable = false;
        }
        return reliable;
    }

    bool BsaModelExtractor::ChangedModels(const std::set<std::filesystem::path> &changed, std::set<std::string> &models)
    {
        try
        {
            std::set<std::string> changedKeys;
            for (const auto &path : changed)
            {
                changedKeys.insert(ArchiveKey(path));
            }
            std::set<std::string> seen;
            std::set<std::string> result;
            for (auto provider = impl_->providers.rbegin(); provider != impl_->providers.rend(); ++provider)
            {
                const auto &index = impl_->GetIndex(*provider);
                const auto changedProvider = changedKeys.contains(ArchiveKey(provider->path));
                for (const auto &[name, entry] : index.entries)
                {
                    if (seen.insert(name).second && changedProvider)
                    {
                        result.insert(name);
                    }
                }
            }
            impl_->SaveStatistics();
            models = std::move(result);
            return true;
        }
        catch (const std::exception &error)
        {
            std::cerr << "BSA model indexing: " << error.what() << '\n';
            return false;
        }
    }
} // namespace navmesh::skyrim
