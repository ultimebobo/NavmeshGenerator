#include "skyrim/parser/patch_export.h"

#include "core/io/file_path.h"
#include "skyrim/parser/navigation_validation.h"
#include "skyrim/parser/navmesh_info.h"
#include "skyrim/parser/reference_navigation.h"
#include "skyrim/parser/text_encoding.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace
{
    using namespace navmesh::skyrim;
    using Bytes = std::vector<std::uint8_t>;
    using Remap = detail::ReferenceFormIdRebaser;
    constexpr std::size_t MaximumMasters = 254;

    std::uint32_t Read32(std::span<const std::uint8_t> bytes, std::size_t offset)
    {
        if (offset > bytes.size() || bytes.size() - offset < 4)
        {
            throw std::runtime_error("Navigation patch contains a truncated FormID or count.");
        }
        return bytes[offset] | std::uint32_t(bytes[offset + 1]) << 8 | std::uint32_t(bytes[offset + 2]) << 16 |
               std::uint32_t(bytes[offset + 3]) << 24;
    }

    void Put32(Bytes &bytes, std::uint32_t value)
    {
        for (int shift = 0; shift < 32; shift += 8)
        {
            bytes.push_back(static_cast<std::uint8_t>(value >> shift));
        }
    }

    void Set32(Bytes &bytes, std::size_t offset, std::uint32_t value)
    {
        (void)Read32(bytes, offset);
        for (int shift = 0; shift < 32; shift += 8)
        {
            bytes[offset + shift / 8] = static_cast<std::uint8_t>(value >> shift);
        }
    }

    void Subrecord(Bytes &bytes, const std::string &type, const Bytes &data)
    {
        const bool extended = data.size() > 65535;
        if (extended)
        {
            bytes.insert(bytes.end(), {'X', 'X', 'X', 'X', 4, 0});
            Put32(bytes, static_cast<std::uint32_t>(data.size()));
        }
        bytes.insert(bytes.end(), type.begin(), type.end());
        const auto size = extended ? 0 : data.size();
        bytes.push_back(static_cast<std::uint8_t>(size));
        bytes.push_back(static_cast<std::uint8_t>(size >> 8));
        bytes.insert(bytes.end(), data.begin(), data.end());
    }

    Bytes Record(const std::string &type, std::uint32_t id, const Bytes &payload, std::uint32_t flags = 0)
    {
        Bytes bytes(type.begin(), type.end());
        Put32(bytes, static_cast<std::uint32_t>(payload.size()));
        Put32(bytes, flags);
        Put32(bytes, id);
        Put32(bytes, 0);
        Put32(bytes, 44);
        bytes.insert(bytes.end(), payload.begin(), payload.end());
        return bytes;
    }

    Bytes Header(const std::vector<std::string> &masters, bool light, std::uint32_t nextId, std::size_t count)
    {
        Bytes hedr;
        const float version = 1.7F;
        hedr.resize(4);
        std::memcpy(hedr.data(), &version, 4);
        Put32(hedr, static_cast<std::uint32_t>(count));
        Put32(hedr, nextId);
        Bytes payload;
        Subrecord(payload, "HEDR", hedr);
        for (const auto &name : masters)
        {
            const auto encoded = EncodePluginText(name);
            Bytes text(encoded.begin(), encoded.end());
            text.push_back(0);
            Subrecord(payload, "MAST", text);
            Subrecord(payload, "DATA", Bytes(8));
        }
        return Record("TES4", 0, payload, light ? 0x200U : 0U);
    }

    bool WriteFile(const std::filesystem::path &path, const Bytes &bytes)
    {
        std::ofstream stream(path, std::ios::binary);
        stream.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        stream.close();
        return !stream.fail();
    }

    std::vector<ResolvedRecord> ReadFile(const std::filesystem::path &path, std::vector<std::string> &masters,
                                         bool &light)
    {
        std::vector<ResolvedRecord> records;
        std::vector<Diagnostic> diagnostics;
        if (!DirectPluginReader{}.Read(path, records, masters, light, diagnostics) || !diagnostics.empty())
        {
            throw std::runtime_error("Cannot independently read navigation patch: " + navmesh::core::PathToUtf8(path));
        }
        return records;
    }

    /// Only explicit FormID fields are rewritten. Counts, coordinates, CRCs and opaque grid bytes remain intact.
    bool RewriteNavmesh(Bytes &body, const Remap &remap)
    {
        const auto field = [&](std::size_t offset)
        {
            const auto value = Read32(body, offset);
            const auto mapped = value ? remap(value) : std::optional<std::uint32_t>{0};
            if (!mapped)
            {
                return false;
            }
            Set32(body, offset, *mapped);
            return true;
        };
        const bool exterior = Read32(body, 8) != 0;
        if (Read32(body, 0) != 12 || !field(exterior ? 8 : 12))
        {
            return false;
        }
        std::size_t offset = 16;
        const auto array = [&](std::size_t width)
        {
            const auto count = Read32(body, offset);
            offset += 4;
            if (count > (body.size() - offset) / width)
            {
                throw std::runtime_error("Navigation patch array exceeds its payload.");
            }
            const auto start = offset;
            offset += count * width;
            return std::pair{start, count};
        };
        array(12);
        array(16);
        const auto [external, links] = array(10);
        for (std::size_t i = 0; i < links; ++i)
        {
            if (!field(external + i * 10 + 4))
            {
                return false;
            }
        }
        const auto [doors, count] = array(10);
        for (std::size_t i = 0; i < count; ++i)
        {
            if (!field(doors + i * 10 + 6))
            {
                return false;
            }
        }
        return true;
    }

    struct EncodedRecord
    {
        std::uint32_t id{};
        Bytes payload;
        std::vector<std::array<std::uint8_t, 24>> groups;
    };

    /// Use the same reference-field decoder for dependency discovery, serialization and reverse read-back.
    EncodedRecord Rewrite(const ResolvedRecord &record, const Remap &remap)
    {
        const auto id = remap(record.winning.formId);
        if (!id || !record.raw)
        {
            throw std::runtime_error("Cannot encode a navigation record identity.");
        }
        EncodedRecord output{.id = *id, .groups = record.groupHeaders};
        if (record.type == "REFR")
        {
            auto association = record.navmeshDoor;
            if (association)
            {
                const auto mesh = remap(association->navmeshId);
                if (!mesh)
                {
                    throw std::runtime_error("Cannot encode a door's NAVM identity.");
                }
                association->navmeshId = *mesh;
            }
            std::string error;
            if (!detail::RewriteReferenceNavigation(*record.raw, remap, association, false, false, output.payload,
                                                    error))
            {
                throw std::runtime_error(error);
            }
        }
        else if (record.type == "NAVM")
        {
            for (const auto &sub : record.raw->subrecords)
            {
                Bytes data(sub.data.begin(), sub.data.end());
                bool valid = true;
                if (sub.type == "NVNM")
                {
                    valid = RewriteNavmesh(data, remap);
                }
                else if (sub.type == "ONAM" && data.size() % 4 == 0)
                {
                    for (std::size_t offset = 0; offset < data.size(); offset += 4)
                    {
                        const auto original = Read32(data, offset);
                        const auto value = original ? remap(original) : std::optional<std::uint32_t>{0};
                        if (!value)
                        {
                            valid = false;
                            break;
                        }
                        Set32(data, offset, *value);
                    }
                }
                else if (sub.type != "EDID" && sub.type != "PNAM" && sub.type != "NNAM")
                {
                    valid = false;
                }
                if (!valid)
                {
                    throw std::runtime_error("Cannot safely rewrite navigation field " + sub.type + ".");
                }
                Subrecord(output.payload, sub.type, data);
            }
        }
        else
        {
            throw std::runtime_error("Unexpected record in navigation export plan: " + record.type);
        }
        for (auto &header : output.groups)
        {
            Bytes bytes(header.begin(), header.end());
            const auto type = Read32(bytes, 12);
            if (type == 1 || type == 6 || type == 8 || type == 9 || type == 10)
            {
                const auto mapped = remap(Read32(bytes, 8));
                if (!mapped)
                {
                    throw std::runtime_error("Cannot encode navigation group placement.");
                }
                Set32(bytes, 8, *mapped);
                std::copy(bytes.begin(), bytes.end(), header.begin());
            }
        }
        return output;
    }

    struct Group
    {
        Bytes header;
        Bytes records;
        std::vector<Group> children;
    };

    void Add(Group &root, const EncodedRecord &record, const Bytes &bytes)
    {
        auto *current = &root;
        for (const auto &header : record.groups)
        {
            auto child =
                std::find_if(current->children.begin(), current->children.end(), [&](const Group &item)
                             { return std::equal(header.begin() + 8, header.begin() + 16, item.header.begin() + 8); });
            if (child == current->children.end())
            {
                current->children.push_back({Bytes(header.begin(), header.end())});
                child = std::prev(current->children.end());
            }
            current = &*child;
        }
        current->records.insert(current->records.end(), bytes.begin(), bytes.end());
    }

    /// Recreate persistent parent loading context independently in every file that contains door overrides.
    Bytes Serialize(const Group &group, std::size_t &count, bool world = false)
    {
        Bytes body = group.records;
        const auto type = group.header.empty() ? 0U : Read32(group.header, 12);
        world = world || type == 1;
        bool hasDoor{};
        for (const auto &child : group.children)
        {
            hasDoor = hasDoor || Read32(child.header, 12) == 8;
            auto bytes = Serialize(child, count, world);
            body.insert(body.end(), bytes.begin(), bytes.end());
        }
        if (group.header.empty())
        {
            return body;
        }
        ++count;
        auto bytes = group.header;
        Set32(bytes, 4, static_cast<std::uint32_t>(24 + body.size()));
        bytes.insert(bytes.end(), body.begin(), body.end());
        const bool parentCell = type == 6 && hasDoor;
        // A world parent is needed only when its subtree contains a persistent door group.
        const auto containsPersistent = [&](const auto &self, const Group &node) -> bool
        {
            return (!node.header.empty() && Read32(node.header, 12) == 8) ||
                   std::any_of(node.children.begin(), node.children.end(),
                               [&](const auto &child) { return self(self, child); });
        };
        if (parentCell || (type == 1 && containsPersistent(containsPersistent, group)))
        {
            auto parent = Record(parentCell ? "CELL" : "WRLD", Read32(group.header, 8), {},
                                 0x4000U | (parentCell && world ? 0x400U : 0U));
            parent.insert(parent.end(), bytes.begin(), bytes.end());
            ++count;
            return parent;
        }
        return bytes;
    }

    struct Patch
    {
        std::set<std::string> dependencies;
        std::vector<std::size_t> records;
        std::string filename;
        std::vector<std::string> masters;
        std::set<std::uint32_t> identities;
        Bytes navigation;
        std::uint32_t navigationId{};
        bool light{};
    };

    /// The stage owns only files created by this export. Never remove a pre-existing directory or output.
    struct Publication
    {
        std::filesystem::path stage;
        std::vector<std::filesystem::path> created;
        std::size_t published{};
        bool complete{};
        ~Publication()
        {
            std::error_code ignored;
            if (!complete)
            {
                for (std::size_t index = 0; index < published; ++index)
                {
                    std::filesystem::remove(created[index], ignored);
                }
            }
            std::filesystem::directory_iterator entry(stage, ignored), end;
            while (!ignored && entry != end)
            {
                std::filesystem::remove(entry->path(), ignored);
                entry.increment(ignored);
            }
            std::filesystem::remove(stage, ignored);
        }
    };
    using OwnerLookup = std::function<std::optional<std::string>(std::uint32_t)>;

    std::string Lower(std::string name)
    {
        std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return char(std::tolower(c)); });
        return name;
    }

    [[noreturn]] void Reject(const std::string &reason)
    {
        throw std::runtime_error(reason);
    }

    /// Pack finalized records, counting their own identities, preserved fields and source-order constraints.
    std::vector<Patch> PlanPatches(const std::vector<ResolvedRecord> &records, const ResolvedLoadOrder &input,
                                   const OwnerLookup &owner, std::optional<std::uint32_t> navigation,
                                   const std::set<std::uint32_t> &newMeshes, bool autoSplit)
    {
        std::vector<Patch> patches(1);
        for (std::size_t index = 0; index < records.size(); ++index)
        {
            const auto &record = records[index];
            std::set<std::string> dependencies;
            const auto collect = [&](std::uint32_t id) -> std::optional<std::uint32_t>
            {
                if (!id)
                {
                    return 0;
                }
                const auto name = owner(id);
                if (!name)
                {
                    return std::nullopt;
                }
                if (!name->empty())
                {
                    dependencies.insert(*name);
                }
                return id;
            };
            (void)Rewrite(record, collect);
            if (navigation && !collect(*navigation))
            {
                Reject("Cannot resolve the navigation-map owner.");
            }
            const auto *source = input.FindWinning(record.winning.formId);
            if (!source && record.cellFormId)
            {
                source = input.FindWinning(*record.cellFormId);
            }
            if (source)
            {
                dependencies.insert(Lower(source->winning.plugin));
            }
            if (dependencies.size() > MaximumMasters)
            {
                Reject("A single navigation record exceeds the master limit; splitting cannot represent it.");
            }
            // First-fit packing is stable in finalized record order and counts every embedded reference.
            auto target = patches.end();
            for (auto it = patches.begin(); it != patches.end(); ++it)
            {
                auto combined = it->dependencies;
                combined.insert(dependencies.begin(), dependencies.end());
                if (combined.size() <= MaximumMasters)
                {
                    target = it;
                    break;
                }
            }
            if (target == patches.end())
            {
                if (!newMeshes.empty())
                {
                    Reject("Automatic splitting with newly allocated NAVM identities is unsupported. "
                           "Use a smaller Cell selection or Plugin scope.");
                }
                if (!autoSplit)
                {
                    Reject("Too many masters for one navigation patch; enable automatic splitting or reduce the "
                           "selection.");
                }
                patches.emplace_back();
                target = std::prev(patches.end());
            }
            target->dependencies.insert(dependencies.begin(), dependencies.end());
            target->records.push_back(index);
        }
        return patches;
    }

    /// Reserve all final filenames and derive stable per-file master order before staging payloads.
    void NamePatches(std::vector<Patch> &patches, const ResolvedLoadOrder &input,
                     const std::filesystem::path &directory, const std::string &filename)
    {
        // Allocate names and master orders for the whole set before writing any final output.
        std::set<std::string> reserved;
        for (const auto &name : input.plugins)
        {
            reserved.insert(Lower(name));
        }
        std::size_t suffix = 1;
        for (std::size_t index = 0; index < patches.size(); ++index)
        {
            auto &patch = patches[index];
            patch.filename = index == 0 ? filename : "generated-navmesh-part-" + std::to_string(++suffix) + ".esp";
            while (reserved.contains(Lower(patch.filename)))
            {
                patch.filename = "generated-navmesh-part-" + std::to_string(++suffix) + ".esp";
            }
            reserved.insert(Lower(patch.filename));
            if (std::filesystem::exists(directory / navmesh::core::PathFromUtf8(patch.filename)))
            {
                Reject("An output plugin already exists; use a fresh output directory.");
            }
            for (const auto &name : input.plugins)
            {
                if (patch.dependencies.contains(Lower(name)))
                {
                    patch.masters.push_back(name);
                }
            }
            if (patch.masters.size() != patch.dependencies.size())
            {
                Reject("A required navigation dependency is absent from the active input order.");
            }
        }
    }

    /// Encode one partition, preserving finalized geometry and deriving NAVI from exactly its emitted meshes.
    void StagePatch(Patch &patch, const std::vector<ResolvedRecord> &records, const Bytes &preparedBytes,
                    const OwnerLookup &owner, std::optional<std::uint32_t> navigation, std::uint32_t nextObjectId,
                    const std::filesystem::path &stagePath)
    {
        std::string error;
        const auto encode = [&](std::uint32_t id) -> std::optional<std::uint32_t>
        {
            if (!id)
            {
                return 0;
            }
            const auto name = owner(id);
            if (!name)
            {
                return std::nullopt;
            }
            const auto object = id >> 24 == 0xfeU ? id & 0xfffU : id & 0xffffffU;
            if (name->empty())
            {
                return static_cast<std::uint32_t>(patch.masters.size()) << 24 | object;
            }
            const auto found = std::find_if(patch.masters.begin(), patch.masters.end(),
                                            [&](const auto &master) { return Lower(master) == *name; });
            if (found == patch.masters.end())
            {
                return std::nullopt;
            }
            return static_cast<std::uint32_t>(found - patch.masters.begin()) << 24 | object;
        };
        Group root;
        std::map<std::uint32_t, Bytes> geometry;
        for (const auto offset : patch.records)
        {
            const auto &record = records[offset];
            auto encoded = Rewrite(record, encode);
            if (record.type == "NAVM")
            {
                const auto nvnm = std::find_if(record.raw->subrecords.begin(), record.raw->subrecords.end(),
                                               [](const auto &sub) { return sub.type == "NVNM"; });
                Bytes body(nvnm->data.begin(), nvnm->data.end());
                if (!RewriteNavmesh(body, encode))
                {
                    Reject("Cannot encode navigation-map geometry.");
                }
                geometry.emplace(encoded.id, std::move(body));
            }
            // Preserve the source record header's revision and version fields while changing its payload and identity.
            const auto start = static_cast<std::size_t>(record.raw->headerRange.offset);
            Bytes encodedBytes(preparedBytes.begin() + start, preparedBytes.begin() + start + 24);
            Set32(encodedBytes, 4, static_cast<std::uint32_t>(encoded.payload.size()));
            Set32(encodedBytes, 12, encoded.id);
            encodedBytes.insert(encodedBytes.end(), encoded.payload.begin(), encoded.payload.end());
            Add(root, encoded, encodedBytes);
            patch.identities.insert(record.winning.formId);
        }
        if (!detail::BuildNavmeshInfo(geometry, nullptr, patch.navigation, error))
        {
            Reject(error);
        }
        const auto nextId = nextObjectId + (navigation ? 0 : 1);
        patch.navigationId =
            navigation ? *encode(*navigation) : static_cast<std::uint32_t>(patch.masters.size()) << 24 | nextObjectId;
        patch.light = patch.masters.size() <= 253 && nextId <= 0x1000U;
        EncodedRecord navi;
        std::array<std::uint8_t, 24> group{};
        std::memcpy(group.data(), "GRUP", 4);
        std::memcpy(group.data() + 8, "NAVI", 4);
        navi.groups.push_back(group);
        Add(root, navi, Record("NAVI", patch.navigationId, patch.navigation));
        std::size_t count = patch.records.size() + 1;
        const auto body = Serialize(root, count);
        auto bytes = Header(patch.masters, patch.light, nextId, count);
        bytes.insert(bytes.end(), body.begin(), body.end());
        if (!WriteFile(stagePath / navmesh::core::PathFromUtf8(patch.filename), bytes))
        {
            Reject("Cannot write a staged navigation patch.");
        }
    }

    /// Reverse read-back must reproduce every planned payload and placement, including cross-file destinations.
    void VerifyPatch(const Patch &patch, const std::map<std::uint32_t, const ResolvedRecord *> &records,
                     const ResolvedLoadOrder &input, const std::filesystem::path &stagePath)
    {
        std::vector<std::string> masters;
        bool light{};
        const auto read = ReadFile(stagePath / navmesh::core::PathFromUtf8(patch.filename), masters, light);
        if (masters != patch.masters || light != patch.light)
        {
            Reject("Navigation patch header failed read-back verification.");
        }
        const auto decode = [&](std::uint32_t id) -> std::optional<std::uint32_t>
        {
            if (!id)
            {
                return 0;
            }
            const auto index = id >> 24;
            if (index == masters.size())
            {
                return 0xff000000U | (id & 0xffffffU);
            }
            if (index > masters.size())
            {
                return std::nullopt;
            }
            const auto prefix = input.pluginFormPrefixes.find(Lower(masters[index]));
            if (prefix == input.pluginFormPrefixes.end())
            {
                return std::nullopt;
            }
            return prefix->second | (id & (prefix->second >> 24 == 0xfeU ? 0xfffU : 0xffffffU));
        };
        std::set<std::uint32_t> seen;
        std::size_t navigationCount{};
        for (const auto &record : read)
        {
            if (record.type == "NAVI")
            {
                ++navigationCount;
                if (record.winning.formId != patch.navigationId || !record.raw ||
                    record.raw->decodedPayload != navmesh::core::SharedBytes(patch.navigation))
                {
                    Reject("Navigation-map read-back differs from the planned file.");
                }
            }
            else if (record.type == "NAVM" || record.type == "REFR")
            {
                const auto canonical = Rewrite(record, decode);
                const auto expected = records.find(canonical.id);
                if (expected == records.end() || !patch.identities.contains(canonical.id) ||
                    !seen.insert(canonical.id).second)
                {
                    Reject("Navigation patch has an unexpected or duplicate record.");
                }
                const auto original = Rewrite(*expected->second, [](std::uint32_t id) { return std::optional{id}; });
                if (record.type != expected->second->type || record.raw->flags != expected->second->raw->flags ||
                    canonical.payload != original.payload || canonical.groups.size() != original.groups.size())
                {
                    Reject("Navigation patch record failed reverse read-back verification.");
                }
                for (std::size_t i = 0; i < canonical.groups.size(); ++i)
                {
                    if (!std::equal(canonical.groups[i].begin() + 8, canonical.groups[i].begin() + 16,
                                    original.groups[i].begin() + 8))
                    {
                        Reject("Navigation patch group placement changed during export.");
                    }
                }
            }
        }
        if (seen.size() != patch.records.size() || navigationCount != 1)
        {
            Reject("Navigation patch is missing planned records or its navigation map.");
        }
    }

} // namespace

bool navmesh::skyrim::detail::WriteNavigationPatchSet(const std::filesystem::path &directory,
                                                      const std::string &filename, const ResolvedLoadOrder &input,
                                                      const std::vector<std::uint8_t> &groups,
                                                      const std::set<std::uint32_t> &newMeshes,
                                                      std::optional<std::uint32_t> navigation,
                                                      std::uint32_t nextObjectId,
                                                      const std::map<std::uint32_t, float> &stepHeights, bool autoSplit,
                                                      std::vector<std::filesystem::path> &written, std::string &error)
{
    written.clear();
    const auto fail = [&](const std::string &reason)
    {
        error = reason;
        return false;
    };
    try
    {
        std::filesystem::create_directories(directory);
        const auto manifest = directory / "plugin-manifest.json";
        if (std::filesystem::exists(manifest))
        {
            return fail("Plugin manifest already exists; use a fresh output directory.");
        }
        const auto stagePath = directory / ".navmesh-export";
        if (!std::filesystem::create_directory(stagePath))
        {
            return fail("Navigation export staging directory already exists; use a fresh output directory.");
        }
        Publication publication{stagePath};
        auto preparedBytes = Header({}, false, nextObjectId, 0);
        preparedBytes.insert(preparedBytes.end(), groups.begin(), groups.end());
        const auto preparedPath = stagePath / "prepared.esp";
        if (!WriteFile(preparedPath, preparedBytes))
        {
            return fail("Cannot stage finalized navigation records.");
        }
        std::vector<std::string> ignoredMasters;
        bool ignoredLight{};
        const auto records = ReadFile(preparedPath, ignoredMasters, ignoredLight);

        // Resolved identities preserve full/light owners without an intermediate byte-sized master index.
        std::map<std::uint32_t, std::string> owners;
        for (const auto &[name, prefix] : input.pluginFormPrefixes)
        {
            if (prefix >> 24 == 0xffU)
            {
                return fail("Input plugins occupy the reserved navigation identity prefix.");
            }
            owners.emplace(prefix, name);
        }
        const auto owner = [&](std::uint32_t id) -> std::optional<std::string>
        {
            if (id >> 24 == 0xffU)
            {
                return std::string{};
            }
            const auto prefix = id >> 24 == 0xfeU ? id & 0xfffff000U : id & 0xff000000U;
            const auto found = owners.find(prefix);
            if (found != owners.end())
            {
                return found->second;
            }
            if (const auto *source = input.FindWinning(id); source && !source->origins.empty())
            {
                auto name = source->origins.front().plugin;
                std::transform(name.begin(), name.end(), name.begin(),
                               [](unsigned char c) { return char(std::tolower(c)); });
                return name;
            }
            return std::nullopt;
        };
        auto patches = PlanPatches(records, input, owner, navigation, newMeshes, autoSplit);
        std::set<std::uint32_t> meshes, references;
        std::map<std::uint32_t, const ResolvedRecord *> recordsByIdentity;
        for (const auto &record : records)
        {
            if (!recordsByIdentity.emplace(record.winning.formId, &record).second)
            {
                return fail("Finalized navigation contains a duplicate record identity.");
            }
            (record.type == "NAVM" ? meshes : references).insert(record.winning.formId);
        }
        if (!ValidateNavigationGraph(records, {}, filename, input, meshes, references, error, stepHeights, true))
        {
            return false;
        }

        NamePatches(patches, input, directory, filename);
        for (auto &patch : patches)
        {
            StagePatch(patch, records, preparedBytes, owner, navigation, nextObjectId, stagePath);
        }

        for (const auto &patch : patches)
        {
            VerifyPatch(patch, recordsByIdentity, input, stagePath);
        }
        const auto stagedManifest = stagePath / "plugin-manifest.json";
        std::ofstream manifestOutput(stagedManifest, std::ios::binary);
        manifestOutput << "{\n  \"schema_version\": 1,\n  \"complete\": true,\n  \"plugins\": [\n";
        for (std::size_t i = 0; i < patches.size(); ++i)
        {
            manifestOutput << "    " << std::quoted(patches[i].filename) << (i + 1 == patches.size() ? "\n" : ",\n");
        }
        manifestOutput
            << "  ],\n  \"installation\": \"Enable every listed plugin in this order after the input plugins.\"\n}\n";
        manifestOutput.close();
        if (manifestOutput.fail() || !LoadOrderSnapshotMatches(input, error))
        {
            return fail(error.empty() ? "Cannot write navigation plugin manifest." : error);
        }
        for (const auto &patch : patches)
        {
            publication.created.push_back(directory / navmesh::core::PathFromUtf8(patch.filename));
        }
        publication.created.push_back(manifest);
        written.assign(publication.created.begin(), publication.created.end() - 1);
        for (const auto &patch : patches)
        {
            const auto destination = directory / navmesh::core::PathFromUtf8(patch.filename);
            std::filesystem::rename(stagePath / navmesh::core::PathFromUtf8(patch.filename), destination);
            ++publication.published;
        }
        std::filesystem::rename(stagedManifest, manifest);
        ++publication.published;
        publication.complete = true;
        return true;
    }
    catch (const std::exception &exception)
    {
        written.clear();
        return fail(exception.what());
    }
}
