#include "skyrim/parser/base_object_swapper.h"
#include "skyrim/parser/text_encoding.h"
#include "core/io/file_path.h"

#include "core/io/content_hash.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <format>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numbers>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace navmesh::skyrim
{
    namespace
    {
        std::string Lower(std::string text)
        {
            std::transform(text.begin(), text.end(), text.begin(),
                           [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
            return text;
        }

        std::vector<std::string> Split(const std::string &text, char separator)
        {
            std::vector<std::string> result;
            if (text.empty())
            {
                return result;
            }
            std::size_t begin{};
            for (;;)
            {
                const auto end = text.find(separator, begin);
                result.push_back(text.substr(begin, end - begin));
                if (end == std::string::npos)
                {
                    return result;
                }
                begin = end + 1;
            }
        }

        bool ConfigName(const std::filesystem::path &path)
        {
            return path.extension() == ".ini" &&
                   navmesh::core::PathToUtf8(path.filename()).find("_SWAP") != std::string::npos;
        }

        std::string Read(const std::filesystem::path &path)
        {
            std::ifstream file(path, std::ios::binary);
            if (!file)
            {
                throw std::runtime_error("Cannot read BOS configuration: " + navmesh::core::PathToUtf8(path));
            }
            std::string contents{std::istreambuf_iterator<char>(file), {}};
            if (file.bad())
            {
                throw std::runtime_error("Cannot completely read BOS configuration: " +
                                         navmesh::core::PathToUtf8(path));
            }
            return contents;
        }

        struct Range
        {
            float minimum{}, maximum{};
            bool Variable() const
            {
                return minimum != maximum;
            }
        };

        float Number(const std::string &text)
        {
            // REX::STR::TO_NUM accepts the numeric prefix and propagates conversion failures.
            return std::stof(text);
        }

        std::uint32_t Hex(const std::string &text)
        {
            // BOS uses Windows unsigned long conversions. Validate the magnitude before
            // unsigned negation so this identity contract is independent of host long width.
            const auto value = std::stoull(text, nullptr, 16);
            const auto sign = text.find_first_not_of(" \t\r\n\f\v");
            const bool negative = sign != std::string::npos && text[sign] == '-';
            const auto magnitude = negative ? 0ULL - value : value;
            if (magnitude > UINT32_MAX)
            {
                throw std::runtime_error("Invalid BOS FormID or flag: " + text);
            }
            const auto narrowed = static_cast<std::uint32_t>(magnitude);
            return negative ? 0U - narrowed : narrowed;
        }

        bool IsHex(const std::string &text)
        {
            constexpr auto begin = 2U;
            return Lower(text).starts_with("0x") && begin < text.size() &&
                   std::all_of(
                       text.begin() + begin, text.end(), [](unsigned char value) { return std::isxdigit(value) != 0; });
        }

        Range ParseRange(const std::string &text)
        {
            const auto parts = Split(text, '/');
            if (parts.empty())
            {
                throw std::runtime_error("Cannot interpret an empty BOS numeric range");
            }
            const auto first = Number(parts.front());
            const auto second = parts.size() > 1 ? Number(parts[1]) : first;
            return {first, second};
        }

        Range EncloseRange(Range range, const std::string &text)
        {
            if (!std::isfinite(range.minimum) || !std::isfinite(range.maximum))
            {
                throw std::runtime_error("Cannot represent non-finite BOS transform range: " + text);
            }
            // Only the first two slash fields reach BOS's sampler. Enclose either endpoint
            // order without assigning a particular runtime random result.
            return {std::min(range.minimum, range.maximum), std::max(range.minimum, range.maximum)};
        }

        struct PointProperty
        {
            std::array<Range, 3> values;
            bool relative{};
            bool Variable() const
            {
                return std::any_of(values.begin(), values.end(), [](const auto &range) { return range.Variable(); });
            }
        };

        struct Properties
        {
            std::optional<PointProperty> position, rotation;
            std::optional<Range> scale;
            bool absoluteScale{};
            std::uint32_t setFlags{}, clearFlags{};
            bool Valid() const
            {
                return position || rotation || scale || setFlags || clearFlags;
            }
            bool Variable() const
            {
                return (position && position->Variable()) || (rotation && rotation->Variable()) ||
                       (scale && scale->Variable());
            }
        };

        /// The expressions mirror BOS v3.5.0's searches, including partial tokens and
        /// commas protected by a following closing parenthesis rather than balanced nesting.
        const std::regex PropertySeparator{R"(,\s*(?![^()]*\)))"};
        const std::regex PropertyArgument{R"(\((.*?)\))"};
        const std::regex PointArguments{R"(\((.*?),(.*?),(.*?)\))"};

        PointProperty ParsePointProperty(const std::string &text, bool rotation)
        {
            PointProperty result{};
            result.relative = text.find('R') != std::string::npos;
            std::smatch match;
            if (std::regex_search(text, match, PointArguments))
            {
                for (std::size_t component{}; component < result.values.size(); ++component)
                {
                    const auto argument = match[component + 1].str();
                    result.values[component] = EncloseRange(ParseRange(argument), argument);
                }
            }
            // An unmatched transform pattern leaves BOS's default zero vector intact.
            if (rotation)
            {
                const bool sampled = result.Variable();
                for (auto &range : result.values)
                {
                    constexpr auto radians = std::numbers::pi_v<float> / 180.0F;
                    constexpr auto limit = std::numbers::pi_v<float> * 2;
                    range.minimum *= radians;
                    range.maximum *= radians;
                    if (!std::isfinite(range.minimum) || !std::isfinite(range.maximum))
                    {
                        throw std::runtime_error("Cannot represent non-finite BOS rotation: " + text);
                    }
                    // BOS clamps every component only when the whole vector is sampled.
                    if (sampled)
                    {
                        range.minimum = std::clamp(range.minimum, -limit, limit);
                        range.maximum = std::clamp(range.maximum, -limit, limit);
                    }
                }
            }
            return result;
        }

        /// Substrings choose the first matching property class; modifier letters anywhere
        /// in that token control relative transforms, absolute scale and cleared flags.
        Properties ParseProperties(const std::string &text)
        {
            Properties result;
            if (text.empty() || Lower(text) == "none")
            {
                return result;
            }
            const std::sregex_token_iterator end;
            for (auto token = std::sregex_token_iterator(text.begin(), text.end(), PropertySeparator, -1); token != end;
                 ++token)
            {
                const auto property = token->str();
                if (property.find("pos") != std::string::npos)
                {
                    result.position = ParsePointProperty(property, false);
                }
                else if (property.find("rot") != std::string::npos)
                {
                    result.rotation = ParsePointProperty(property, true);
                }
                else if (property.find("scale") != std::string::npos)
                {
                    std::smatch match;
                    auto range =
                        std::regex_search(property, match, PropertyArgument) ? ParseRange(match[1].str()) : Range{};
                    range.minimum = std::clamp(range.minimum, 0.0F, 1000.0F);
                    range.maximum = std::clamp(range.maximum, 0.0F, 1000.0F);
                    result.scale = EncloseRange(range, property);
                    result.absoluteScale = property.find('A') != std::string::npos;
                }
                else if (property.find("flags") != std::string::npos)
                {
                    std::smatch match;
                    if (std::regex_search(property, match, PropertyArgument))
                    {
                        auto &flags = property.find('C') != std::string::npos ? result.clearFlags : result.setFlags;
                        for (const auto &argument : Split(match[1].str(), ','))
                        {
                            flags |= Hex(argument);
                        }
                    }
                }
            }
            return result;
        }

        struct Rule
        {
            std::vector<std::uint32_t> replacements;
            Properties properties;
            float chance{100};
            std::vector<std::string> conditions;
            std::string conditionKey, evidence;
            /// Empty filter lists still belong to conditional priority groups.
            bool conditional{};
            bool leveled{};
            bool randomAssignment{};
            /// A selected unresolved or non-bound replacement lets BOS try lower-priority swaps.
            bool lookupMayFail{};
        };

        using RuleMap = std::unordered_map<std::uint32_t, std::vector<Rule>>;
        struct Rules
        {
            RuleMap forms, references, properties;
        };

        struct ConfigEntry
        {
            std::string section, line;
            std::size_t lineNumber{};
        };

        /// Normalize physical INI line endings without changing diagnostic line numbers.
        std::string IniNewlines(const std::string &contents)
        {
            std::string result;
            result.reserve(contents.size());
            for (std::size_t index{}; index < contents.size(); ++index)
            {
                if (contents[index] == '\r')
                {
                    result += '\n';
                    if (index + 1 < contents.size() && contents[index + 1] == '\n')
                    {
                        ++index;
                    }
                }
                else
                {
                    result += contents[index];
                }
            }
            return result;
        }

        /// SimpleIni merges repeated sections and visits them in first-occurrence order.
        /// Sectionless keys belong to the unnamed section, which BOS interprets as reference swaps.
        std::vector<ConfigEntry> ConfigEntries(const BaseObjectSwapConfig &config,
                                               std::vector<std::string> &diagnostics)
        {
            auto contents = IniNewlines(config.contents);
            if (const auto terminator = contents.find('\0'); terminator != std::string::npos)
            {
                contents.resize(terminator);
            }
            std::vector<std::size_t> lineStarts{0};
            for (std::size_t offset{}; offset < contents.size(); ++offset)
            {
                if (contents[offset] == '\n')
                {
                    lineStarts.push_back(offset + 1);
                }
            }
            const auto lineNumber = [&](const char *position)
            {
                const auto offset = static_cast<std::size_t>(position - contents.data());
                return static_cast<std::size_t>(std::upper_bound(lineStarts.begin(), lineStarts.end(), offset) -
                                                lineStarts.begin());
            };
            const auto space = [](char value) { return value == ' ' || value == '\t' || value == '\n'; };
            struct Section
            {
                std::string name;
                std::vector<ConfigEntry> entries;
                std::set<std::string> keys;
            };
            std::vector<Section> sections;
            std::map<std::string, std::size_t> indices;
            const auto addEntry = [&](const char *section, const char *key, std::size_t sourceLine)
            {
                const auto [found, inserted] = indices.emplace(Lower(section), sections.size());
                if (inserted)
                {
                    sections.push_back({section});
                }
                auto &destination = sections[found->second];
                if (key && *key && destination.keys.insert(Lower(key)).second)
                {
                    destination.entries.push_back({destination.name, key, sourceLine});
                }
            };
            auto *cursor = contents.data();
            if (contents.starts_with("\xEF\xBB\xBF"))
            {
                cursor += 3;
            }
            // SimpleIni's section pointer refers to the mutable input. Even an incomplete
            // header changes that pointer; later key/value terminators delimit its name.
            const char *section = "";
            while (*cursor)
            {
                while (space(*cursor))
                {
                    ++cursor;
                }
                if (!*cursor)
                {
                    break;
                }
                if (*cursor == ';' || *cursor == '#')
                {
                    while (*cursor && *cursor != '\n')
                    {
                        ++cursor;
                    }
                    continue;
                }
                const auto sourceLine = lineNumber(cursor);
                if (*cursor == '[')
                {
                    ++cursor;
                    while (space(*cursor))
                    {
                        ++cursor;
                    }
                    section = cursor;
                    while (*cursor && *cursor != ']' && *cursor != '\n')
                    {
                        ++cursor;
                    }
                    if (*cursor != ']')
                    {
                        const auto source = config.path.empty() ? config.name : navmesh::core::PathToUtf8(config.path);
                        diagnostics.push_back(source + ":" + std::to_string(sourceLine) +
                                              ": ignored unterminated BOS section header");
                        continue;
                    }
                    auto *end = cursor;
                    while (end > section && space(end[-1]))
                    {
                        --end;
                    }
                    *end = '\0';
                    ++cursor;
                    while (*cursor && *cursor != '\n')
                    {
                        ++cursor;
                    }
                    addEntry(section, nullptr, sourceLine);
                    continue;
                }
                auto *key = cursor;
                while (*cursor && *cursor != '=' && *cursor != '\n')
                {
                    ++cursor;
                }
                const bool hasValue = *cursor == '=';
                if (hasValue && key == cursor)
                {
                    while (*cursor && *cursor != '\n')
                    {
                        ++cursor;
                    }
                    continue;
                }
                auto *keyEnd = cursor;
                while (keyEnd > key && space(keyEnd[-1]))
                {
                    --keyEnd;
                }
                if (hasValue)
                {
                    *keyEnd = '\0';
                    ++cursor;
                    while (*cursor && *cursor != '\n' && space(*cursor))
                    {
                        ++cursor;
                    }
                    auto *value = cursor;
                    while (*cursor && *cursor != '\n')
                    {
                        ++cursor;
                    }
                    auto *valueEnd = cursor;
                    if (*cursor)
                    {
                        ++cursor;
                    }
                    while (valueEnd > value && space(valueEnd[-1]))
                    {
                        --valueEnd;
                    }
                    *valueEnd = '\0';
                }
                else if (*cursor)
                {
                    ++cursor;
                }
                *keyEnd = '\0';
                // BOS visits unique keys in their first load position and ignores INI values.
                addEntry(section, key, sourceLine);
            }
            std::vector<ConfigEntry> result;
            for (auto &sectionEntries : sections)
            {
                auto &entries = sectionEntries.entries;
                result.insert(result.end(), std::make_move_iterator(entries.begin()),
                              std::make_move_iterator(entries.end()));
            }
            return result;
        }

        /// Build identity lookups once; qualified IDs use the owning plugin prefix rather than its winning override.
        class Identities
        {
          public:
            explicit Identities(const ResolvedLoadOrder &resolved) : resolved_(resolved)
            {
                for (const auto &record : resolved.records)
                {
                    if (!record.editorId.empty())
                    {
                        editors_[Lower(record.editorId)] = record.formId;
                    }
                }
            }

            std::optional<std::uint32_t> Find(const std::string &text) const
            {
                const auto parts = Split(text, '~');
                if (parts.size() == 2)
                {
                    const auto plugin = resolved_.pluginFormPrefixes.find(Lower(parts[1]));
                    if (plugin == resolved_.pluginFormPrefixes.end())
                    {
                        return std::nullopt;
                    }
                    // BOS calls TESDataHandler::LookupFormID, which adds the supplied ID to the
                    // plugin prefix, including carries into other plugin indices and unsigned wrap.
                    return plugin->second + Hex(parts[0]);
                }
                if (IsHex(text))
                {
                    return Hex(text);
                }
                const auto editor = editors_.find(Lower(text));
                return editor == editors_.end() ? std::nullopt : std::optional(editor->second);
            }

          private:
            const ResolvedLoadOrder &resolved_;
            std::unordered_map<std::string, std::uint32_t> editors_;
        };

        float ParseChance(const std::string &text)
        {
            if (text.empty() || Lower(text) == "none")
            {
                return 100;
            }
            // BOS leaves its default chance intact unless the text contains "chance"
            // and a parenthesized value. Bare numeric fields have no runtime effect.
            std::smatch match;
            if (text.find("chance") == std::string::npos || !std::regex_search(text, match, PropertyArgument))
            {
                return 100;
            }
            const auto options = Split(match[1].str(), ',');
            if (options.empty())
            {
                throw std::runtime_error("Cannot interpret an empty BOS chance value");
            }
            const auto chance = Number(options.front());
            if (options.size() > 1)
            {
                // The offline envelope does not sample the seed, but BOS still parses it.
                (void)std::stoull(options[1]);
            }
            return chance < 100 ? chance : 100; // NaN also bypasses BOS's chance test.
        }

        /// Enumerate possible leveled outcomes without evaluating runtime level, chance-none or encounter zones.
        void ExpandLeveledReplacement(const ResolvedRecord &entry, const ResolvedLoadOrder &resolved,
                                      std::set<std::uint32_t> &visited, std::vector<std::uint32_t> &replacements)
        {
            if (!visited.insert(entry.formId).second)
            {
                return;
            }
            if (entry.type != "LVLI")
            {
                replacements.push_back(entry.formId);
                return;
            }
            for (const auto child : entry.leveledBaseIds)
            {
                const auto *record = resolved.FindWinning(child);
                if (!record)
                {
                    throw std::runtime_error("Leveled BOS replacement contains an unresolved base");
                }
                ExpandLeveledReplacement(*record, resolved, visited, replacements);
            }
        }

        Rules ParseRules(const BaseObjectSwapInput &input, const ResolvedLoadOrder &resolved,
                         const Identities &identities, std::vector<std::string> &diagnostics)
        {
            Rules result;
            for (const auto &config : input.configs)
            {
                for (const auto &entry : ConfigEntries(config, diagnostics))
                {
                    const auto &section = entry.section;
                    const auto &line = entry.line;
                    const auto evidence = config.name + ":" + std::to_string(entry.lineNumber);
                    const auto source = config.path.empty() ? config.name : navmesh::core::PathToUtf8(config.path);
                    try
                    {
                        const auto sectionParts = Split(section, '|');
                        const auto &sectionName = sectionParts.empty() ? section : sectionParts.front();
                        const bool conditional = section.find('|') != std::string::npos;
                        // BOS routes every conditional non-Forms section to base properties;
                        // unconditional sections other than Forms/Transforms/Properties are reference swaps.
                        const bool property = conditional ? sectionName != "Forms"
                                                          : sectionName == "Transforms" || sectionName == "Properties";
                        const auto fields = Split(line, '|');
                        if (fields.size() < 2)
                        {
                            diagnostics.push_back(source + ":" + std::to_string(entry.lineNumber) +
                                                  ": ignored BOS entry without a replacement or properties");
                            continue;
                        }
                        Rule rule;
                        rule.evidence = evidence;
                        rule.conditional = conditional;
                        // BOS attempts the entire source as one identity first. Only a failed
                        // swap-source lookup expands commas; property sources never expand them.
                        const auto singleSource = identities.Find(fields[0]);
                        const bool sourceSet =
                            !property && (!singleSource || !*singleSource) && fields[0].find(',') != std::string::npos;
                        const auto sourceNames = sourceSet ? Split(fields[0], ',') : std::vector{fields[0]};
                        std::set<std::uint32_t> sourceIds;
                        for (const auto &name : sourceNames)
                        {
                            const auto id = identities.Find(name);
                            if (!id || !*id)
                            {
                                diagnostics.push_back(evidence + ": source is unavailable: " + name);
                                continue;
                            }
                            if (!resolved.FindWinning(*id))
                            {
                                diagnostics.push_back(evidence + ": source is unavailable: " + name);
                            }
                            sourceIds.insert(*id);
                        }
                        // An unavailable source cannot affect any indexed placement. Do not interpret
                        // its property/chance bundle, which BOS also constructs only after source lookup.
                        if (std::none_of(sourceIds.begin(), sourceIds.end(),
                                         [&](const auto id) { return resolved.FindWinning(id) != nullptr; }))
                        {
                            continue;
                        }
                        if (conditional)
                        {
                            rule.conditions = Split(sectionParts[1], ',');
                            rule.conditionKey = config.name + "|" + sectionParts[1];
                        }
                        if (!property)
                        {
                            std::set<std::uint32_t> replacementIds;
                            for (const auto &name : Split(fields[1], ','))
                            {
                                const auto id = identities.Find(name);
                                const auto *base = id ? resolved.FindWinning(*id) : nullptr;
                                if (!id || !*id)
                                {
                                    diagnostics.push_back(evidence + ": replacement is unavailable: " + name);
                                    continue;
                                }
                                if (!replacementIds.insert(*id).second)
                                {
                                    continue;
                                }
                                if (!base || base->type == "REFR" || base->type == "ACHR" || base->type == "CELL" ||
                                    base->type == "WRLD" || base->type == "NAVM" || base->type == "LAND")
                                {
                                    diagnostics.push_back(evidence + ": replacement base is unavailable: " + name);
                                    rule.lookupMayFail = true;
                                    continue;
                                }
                                // Leveled swaps depend on runtime level and may produce no replacement.
                                // Retain every reachable base plus the original outcome rather than evaluate game state.
                                if (base->type == "LVLI")
                                {
                                    rule.leveled = true;
                                    std::set<std::uint32_t> visited;
                                    ExpandLeveledReplacement(*base, resolved, visited, rule.replacements);
                                }
                                else
                                {
                                    rule.replacements.push_back(*id);
                                }
                            }
                            if (rule.replacements.empty() && !rule.leveled)
                            {
                                continue;
                            }
                            std::sort(rule.replacements.begin(), rule.replacements.end());
                            rule.replacements.erase(std::unique(rule.replacements.begin(), rule.replacements.end()),
                                                    rule.replacements.end());
                            if (sourceSet && replacementIds.size() > 1)
                            {
                                if (replacementIds.size() < sourceIds.size())
                                {
                                    diagnostics.push_back(
                                        evidence + ": BOS ignores a replacement set smaller than its source set");
                                    continue;
                                }
                                // BOS pairs source sets with unique random replacements at configuration load,
                                // discarding the entry's per-reference chance for this assignment branch.
                                rule.randomAssignment = true;
                            }
                            if (!sourceSet && fields[1].find(',') == std::string::npos &&
                                rule.replacements.size() == 1 && rule.replacements.front() == *sourceIds.begin() &&
                                (fields.size() < 3 || fields[2].empty() || Lower(fields[2]) == "none"))
                            {
                                continue;
                            }
                        }
                        // BOS constructs the bundle only after source/replacement lookup succeeds.
                        rule.properties =
                            ParseProperties(fields.size() > (property ? 1U : 2U) ? fields[property ? 1 : 2] : "");
                        const auto chanceIndex = property ? 2U : 3U;
                        const auto chance = ParseChance(fields.size() > chanceIndex ? fields[chanceIndex] : "");
                        rule.chance = rule.randomAssignment ? 100 : chance;
                        auto &destination = property                 ? result.properties
                                            : sectionName == "Forms" ? result.forms
                                                                     : result.references;
                        for (const auto id : sourceIds)
                        {
                            destination[id].push_back(rule);
                        }
                    }
                    catch (const std::bad_alloc &)
                    {
                        throw;
                    }
                    catch (const std::exception &error)
                    {
                        throw std::runtime_error(source + ":" + std::to_string(entry.lineNumber) + ": " + error.what());
                    }
                }
            }
            return result;
        }

        enum class Match
        {
            No,
            Yes,
            Unknown
        };

        Match Condition(const std::string &name, const ResolvedRecord &owner, const Identities &identities,
                        const ResolvedLoadOrder &resolved)
        {
            const auto identity = identities.Find(name);
            const auto *record = identity ? resolved.FindWinning(*identity) : nullptr;
            if (record && record->type == "CELL")
            {
                return owner.cellFormId == identity ? Match::Yes : Match::No;
            }
            if (record && record->type == "WRLD")
            {
                const auto *world = owner.worldspaceFormId ? resolved.FindWinning(*owner.worldspaceFormId) : nullptr;
                return owner.worldspaceFormId == identity || (world && world->winning.parentWorldFormId == identity)
                           ? Match::Yes
                           : Match::No;
            }
            if (record && record->type != "LCTN" && record->type != "REGN" && record->type != "KYWD")
            {
                return Match::No;
            }
            const auto *cell = owner.cellFormId ? resolved.FindWinning(*owner.cellFormId) : nullptr;
            if (cell && Lower(cell->editorId) == Lower(name))
            {
                return Match::Yes;
            }
            // Runtime keywords, region memberships and location ancestry are not guessed from display names.
            return Match::Unknown;
        }

        Match Conditions(const Rule &rule, const ResolvedRecord &owner, const Identities &identities,
                         const ResolvedLoadOrder &resolved)
        {
            bool positive{}, positiveMatch{}, unknownPositive{}, unknownNegative{};
            for (auto name : rule.conditions)
            {
                const bool exclude = name.starts_with('-');
                if (exclude)
                {
                    name.erase(0, 1);
                }
                const auto match = Condition(name, owner, identities, resolved);
                if (exclude && match == Match::Yes)
                {
                    return Match::No;
                }
                if (exclude)
                {
                    unknownNegative = unknownNegative || match == Match::Unknown;
                }
                else
                {
                    positive = true;
                    positiveMatch = positiveMatch || match == Match::Yes;
                    unknownPositive = unknownPositive || match == Match::Unknown;
                }
            }
            if (positive && !positiveMatch && !unknownPositive)
            {
                return Match::No;
            }
            return unknownNegative || (positive && !positiveMatch) ? Match::Unknown : Match::Yes;
        }

        /// BOS conditional groups use reverse condition-key order; unconditional entries use reverse insertion order.
        void AppendRules(std::vector<const Rule *> &destination, const RuleMap &map, std::uint32_t id, bool conditional)
        {
            const auto found = map.find(id);
            if (found == map.end())
            {
                return;
            }
            std::vector<const Rule *> group;
            for (auto rule = found->second.rbegin(); rule != found->second.rend(); ++rule)
            {
                if (rule->conditional == conditional)
                {
                    group.push_back(&*rule);
                }
            }
            if (conditional)
            {
                std::stable_sort(group.begin(), group.end(),
                                 [](const auto *a, const auto *b) { return a->conditionKey > b->conditionKey; });
            }
            destination.insert(destination.end(), group.begin(), group.end());
        }

        std::vector<const Rule *> PossibleRules(const std::vector<const Rule *> &rules, const ResolvedRecord &owner,
                                                const Identities &identities, const ResolvedLoadOrder &resolved,
                                                bool properties)
        {
            std::vector<const Rule *> result;
            for (const auto *rule : rules)
            {
                if (rule->chance < 0 || (properties && !rule->properties.Valid()))
                {
                    continue;
                }
                const auto match = Conditions(*rule, owner, identities, resolved);
                if (match == Match::No)
                {
                    continue;
                }
                result.push_back(rule);
                if (match == Match::Yes && rule->chance >= 100 && !rule->leveled && !rule->lookupMayFail)
                {
                    return result;
                }
            }
            result.push_back(nullptr);
            return result;
        }

        std::pair<core::Vec3, core::Vec3> PointRange(core::Vec3 original, const PointProperty &property)
        {
            core::Vec3 low{property.values[0].minimum, property.values[1].minimum, property.values[2].minimum};
            core::Vec3 high{property.values[0].maximum, property.values[1].maximum, property.values[2].maximum};
            if (property.relative)
            {
                low = low + original;
                high = high + original;
            }
            return {low, high};
        }

        void ApplyProperties(core::Reference &reference, const Properties &properties, bool uncertain)
        {
            core::PlacementEnvelope envelope{reference.position, reference.position, reference.scale};
            bool uncertainScale{};
            if (properties.position)
            {
                const auto [low, high] = PointRange(reference.position, *properties.position);
                reference.position = low;
                envelope.minimumPosition = low;
                envelope.maximumPosition = high;
            }
            if (properties.rotation)
            {
                reference.rotation = PointRange(reference.rotation, *properties.rotation).first;
                envelope.arbitraryRotation = properties.rotation->Variable();
            }
            if (properties.scale)
            {
                // BOS stores absolute scale as an integer percentage and multiplies that field for relative scale.
                const auto percentage = std::floor(reference.scale * 100.0F);
                const auto factor = properties.absoluteScale ? 1.0F : percentage;
                const auto maximum = properties.scale->maximum * factor;
                if (!std::isfinite(maximum))
                {
                    throw std::runtime_error("BOS scale is not finite");
                }
                uncertainScale = maximum > UINT16_MAX;
                // Out-of-range floating conversions have no portable runtime result.
                // Cover the entire integer field instead of sampling or rejecting a random scale.
                reference.scale = uncertainScale ? 0.0F : std::floor(properties.scale->minimum * factor) / 100.0F;
                envelope.maximumScale = uncertainScale ? UINT16_MAX / 100.0F : std::floor(maximum) / 100.0F;
            }
            const auto updateFlag = [&](bool original, std::uint32_t flag)
            { return (original && !(properties.clearFlags & flag)) || (properties.setFlags & flag); };
            reference.initiallyDisabled = updateFlag(reference.initiallyDisabled, 1U << 11);
            reference.deleted = updateFlag(reference.deleted, 1U << 5);
            // Flags outside the offline visibility model can alter collision semantics.
            // Keep possible collision as obstruction rather than promise a walkable platform.
            constexpr auto visibilityFlags = (1U << 11) | (1U << 5);
            const bool unknownFlags = ((properties.setFlags | properties.clearFlags) & ~visibilityFlags) != 0;
            if (uncertain || properties.Variable() || unknownFlags || uncertainScale)
            {
                reference.envelope = envelope;
                reference.teleportExit = false;
            }
        }

        /// Runtime armor sex can select a distinct ground mesh. Read it from retained subrecords
        /// only when a BOS placement needs it, without enlarging every indexed record.
        std::string AlternateArmorModel(const ResolvedRecord *base)
        {
            if (base && base->type == "ARMO" && base->raw)
            {
                for (const auto &sub : base->raw->subrecords)
                {
                    if (sub.type == "MOD4" && !sub.data.empty())
                    {
                        const std::string_view bytes(reinterpret_cast<const char *>(sub.data.data()), sub.data.size());
                        auto model = DecodePluginText(bytes.substr(0, bytes.find('\0')));
                        if (!model.empty() && Lower(model) != Lower(base->modelPath.value_or("")))
                        {
                            return model;
                        }
                    }
                }
            }
            return {};
        }

        std::vector<core::Reference> Alternatives(const core::Reference &original, const ResolvedRecord &owner,
                                                  const Rules &rules, const Identities &identities,
                                                  const ResolvedLoadOrder &resolved)
        {
            std::vector<const Rule *> swaps;
            AppendRules(swaps, rules.references, original.id, false);
            AppendRules(swaps, rules.forms, original.baseObjectId, true);
            AppendRules(swaps, rules.forms, original.baseObjectId, false);
            std::vector<const Rule *> properties;
            AppendRules(properties, rules.properties, original.id, false);
            AppendRules(properties, rules.properties, original.baseObjectId, true);
            AppendRules(properties, rules.properties, original.baseObjectId, false);
            const auto swapChoices = PossibleRules(swaps, owner, identities, resolved, false);
            const auto propertyChoices = PossibleRules(properties, owner, identities, resolved, true);
            std::vector<core::Reference> result;
            for (const auto *swap : swapChoices)
            {
                auto replacements = swap ? swap->replacements : std::vector{original.baseObjectId};
                if (swap && swap->leveled &&
                    std::find(replacements.begin(), replacements.end(), original.baseObjectId) == replacements.end())
                {
                    // A failed leveled evaluation leaves the reference unchanged even when lower-priority swaps exist.
                    replacements.push_back(original.baseObjectId);
                }
                const auto chosenProperties = swap && swap->properties.Valid() ? std::vector{swap} : propertyChoices;
                for (const auto replacement : replacements)
                {
                    for (const auto *property : chosenProperties)
                    {
                        auto reference = original;
                        if (swap)
                        {
                            const auto *base = resolved.FindWinning(replacement);
                            reference.baseObjectId = replacement;
                            if (base)
                            {
                                reference.baseRecordType = base->type;
                                reference.basePlugin = base->winning.plugin;
                                reference.modelPath = base->modelPath.value_or("");
                                reference.automaticDoor = base->winning.automaticDoor;
                            }
                            reference.localBounds.reset();
                            reference.inputConfig = swap->evidence;
                        }
                        if (property)
                        {
                            if (!reference.inputConfig.empty() && property != swap)
                            {
                                reference.inputConfig += ";";
                            }
                            if (property != swap)
                            {
                                reference.inputConfig += property->evidence;
                            }
                        }
                        const bool uncertain = (swap && (swap->leveled || swap->randomAssignment)) ||
                                               swapChoices.size() > 1 || replacements.size() > 1 ||
                                               chosenProperties.size() > 1;
                        ApplyProperties(reference, property ? property->properties : Properties{}, uncertain);
                        const auto alternateModel =
                            !reference.inputConfig.empty() || reference.envelope
                                ? AlternateArmorModel(resolved.FindWinning(reference.baseObjectId))
                                : std::string{};
                        if (!alternateModel.empty())
                        {
                            auto alternate = reference;
                            alternate.modelPath = alternateModel;
                            result.push_back(std::move(alternate));
                        }
                        result.push_back(std::move(reference));
                    }
                }
            }
            // All outcomes are obstacles when any possible outcome differs; a random platform is never a floor.
            if (result.size() > 1)
            {
                for (auto &reference : result)
                {
                    if (!reference.envelope)
                    {
                        ApplyProperties(reference, {}, true);
                    }
                }
            }
            return result;
        }

        std::string Json(const std::string &text)
        {
            std::string result = "\"";
            for (const auto character : text)
            {
                switch (character)
                {
                case '\\':
                    result += "\\\\";
                    break;
                case '"':
                    result += "\\\"";
                    break;
                case '\n':
                    result += "\\n";
                    break;
                case '\r':
                    result += "\\r";
                    break;
                case '\t':
                    result += "\\t";
                    break;
                default:
                    if (static_cast<unsigned char>(character) < 0x20)
                    {
                        result += std::format("\\u{:04x}", static_cast<unsigned char>(character));
                    }
                    else
                    {
                        result += character;
                    }
                }
            }
            return result + "\"";
        }
    } // namespace

    BaseObjectSwapInput DiscoverBaseObjectSwaps(const std::vector<std::filesystem::path> &roots)
    {
        BaseObjectSwapInput result{.roots = roots};
        std::map<std::string, std::filesystem::path> winners;
        for (const auto &root : roots)
        {
            if (!std::filesystem::exists(root))
            {
                continue;
            }
            for (const auto &entry : std::filesystem::directory_iterator(root))
            {
                if (entry.is_regular_file() && Lower(navmesh::core::PathToUtf8(entry.path().extension())) == ".ini")
                {
                    winners[Lower(navmesh::core::PathToUtf8(entry.path().filename()))] = entry.path();
                }
            }
        }
        core::ContentHash hash;
        hash.Add("bos-v3.5/conservative-envelope-v2");
        for (const auto &[logicalName, path] : winners)
        {
            if (!ConfigName(path))
            {
                continue;
            }
            auto contents = Read(path);
            result.configs.push_back({navmesh::core::PathToUtf8(path.filename()), path, std::move(contents)});
        }
        std::sort(result.configs.begin(), result.configs.end(),
                  [](const auto &a, const auto &b) { return a.name < b.name; });
        for (const auto &config : result.configs)
        {
            hash.Add(config.name);
            hash.Add(navmesh::core::PathToUtf8(config.path));
            hash.Add(config.contents);
        }
        result.fingerprint = hash.Hex();
        return result;
    }

    std::vector<std::string> ApplyBaseObjectSwaps(ResolvedLoadOrder &resolved, const BaseObjectSwapInput &input)
    {
        if (!resolved.runtimePlacements.empty())
        {
            throw std::runtime_error("BOS inputs have already been applied to this load order");
        }
        std::vector<std::string> diagnostics;
        if (input.configs.empty())
        {
            return diagnostics;
        }
        const Identities identities(resolved);
        const auto rules = ParseRules(input, resolved, identities, diagnostics);
        std::size_t conservative{};
        for (auto &cell : resolved.cells)
        {
            std::vector<core::Reference> effective;
            effective.reserve(cell.references.size());
            for (const auto &reference : cell.references)
            {
                if (!rules.references.contains(reference.id) && !rules.forms.contains(reference.baseObjectId) &&
                    !rules.properties.contains(reference.id) && !rules.properties.contains(reference.baseObjectId))
                {
                    effective.push_back(reference);
                    continue;
                }
                const auto *owner = resolved.FindWinning(reference.id);
                if (!owner)
                {
                    effective.push_back(reference);
                    continue;
                }
                auto alternatives = Alternatives(reference, *owner, rules, identities, resolved);
                const bool changed = std::any_of(alternatives.begin(), alternatives.end(), [](const auto &placement)
                                                 { return !placement.inputConfig.empty() || placement.envelope; });
                if (changed)
                {
                    if (std::any_of(alternatives.begin(), alternatives.end(),
                                    [](const auto &placement) { return placement.envelope.has_value(); }))
                    {
                        ++conservative;
                    }
                    resolved.runtimePlacements.push_back({owner->winning, reference, alternatives});
                }
                effective.insert(effective.end(), std::make_move_iterator(alternatives.begin()),
                                 std::make_move_iterator(alternatives.end()));
            }
            cell.references = std::move(effective);
        }
        if (conservative)
        {
            diagnostics.push_back(std::format(
                "BOS: {} references use conservative obstruction envelopes for uncertain "
                "outcomes, runtime conditions, scale conversions or collision flags; these objects supply no floors.",
                conservative));
        }
        return diagnostics;
    }

    bool BaseObjectSwapSnapshotMatches(const BaseObjectSwapInput &input)
    {
        return DiscoverBaseObjectSwaps(input.roots).fingerprint == input.fingerprint;
    }

    bool WriteBaseObjectSwapReport(const std::filesystem::path &path, const BaseObjectSwapInput &input,
                                   const ResolvedLoadOrder &resolved, const std::vector<std::string> &diagnostics)
    {
        std::ofstream out(path);
        out << std::setprecision(std::numeric_limits<float>::max_digits10);
        out << "{\"schema\":\"bos-input-v1\",\"policy\":\"conservative-collision-envelope\",\"fingerprint\":"
            << Json(input.fingerprint) << ",\"configs\":[";
        bool comma{};
        for (const auto &config : input.configs)
        {
            out << (comma ? "," : "") << "{\"name\":" << Json(config.name)
                << ",\"path\":" << Json(navmesh::core::PathToUtf8(config.path)) << "}";
            comma = true;
        }
        out << "],\"placements\":[";
        comma = false;
        for (const auto &change : resolved.runtimePlacements)
        {
            out << (comma ? "," : "") << "{\"reference_id\":" << change.original.id
                << ",\"cell_id\":" << change.owner.cellFormId.value_or(0)
                << ",\"original\":{\"base_id\":" << change.original.baseObjectId
                << ",\"model\":" << Json(change.original.modelPath) << ",\"position\":[" << change.original.position.x
                << ',' << change.original.position.y << ',' << change.original.position.z << "],\"rotation_radians\":["
                << change.original.rotation.x << ',' << change.original.rotation.y << ',' << change.original.rotation.z
                << "],\"scale\":" << change.original.scale << "},\"alternatives\":[";
            bool alternativeComma{};
            for (const auto &reference : change.alternatives)
            {
                out << (alternativeComma ? "," : "") << "{\"base_id\":" << reference.baseObjectId
                    << ",\"config\":" << Json(reference.inputConfig) << ",\"model\":" << Json(reference.modelPath)
                    << ",\"position\":[" << reference.position.x << ',' << reference.position.y << ','
                    << reference.position.z << "],\"rotation_radians\":[" << reference.rotation.x << ','
                    << reference.rotation.y << ',' << reference.rotation.z << "],\"scale\":" << reference.scale
                    << ",\"disabled\":" << (reference.initiallyDisabled ? "true" : "false")
                    << ",\"deleted\":" << (reference.deleted ? "true" : "false")
                    << ",\"conservative\":" << (reference.envelope ? "true" : "false");
                if (reference.envelope)
                {
                    const auto &envelope = *reference.envelope;
                    out << ",\"envelope\":{\"minimum_position\":[" << envelope.minimumPosition.x << ','
                        << envelope.minimumPosition.y << ',' << envelope.minimumPosition.z << "],\"maximum_position\":["
                        << envelope.maximumPosition.x << ',' << envelope.maximumPosition.y << ','
                        << envelope.maximumPosition.z << "],\"maximum_scale\":" << envelope.maximumScale
                        << ",\"arbitrary_rotation\":" << (envelope.arbitraryRotation ? "true" : "false") << '}';
                }
                out << '}';
                alternativeComma = true;
            }
            out << "]}";
            comma = true;
        }
        out << "],\"diagnostics\":[";
        comma = false;
        for (const auto &diagnostic : diagnostics)
        {
            out << (comma ? "," : "") << Json(diagnostic);
            comma = true;
        }
        out << "]}\n";
        return static_cast<bool>(out);
    }
} // namespace navmesh::skyrim
