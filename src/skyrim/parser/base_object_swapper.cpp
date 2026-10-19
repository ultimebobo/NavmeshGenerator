#include "skyrim/parser/base_object_swapper.h"

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
#include <set>
#include <sstream>
#include <stdexcept>

namespace navmesh::skyrim
{
    namespace
    {
        std::string Trim(std::string text)
        {
            const auto begin = text.find_first_not_of(" \t\r\n");
            return begin == std::string::npos ? "" : text.substr(begin, text.find_last_not_of(" \t\r\n") - begin + 1);
        }

        std::string Lower(std::string text)
        {
            std::transform(text.begin(), text.end(), text.begin(),
                           [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
            return text;
        }

        std::vector<std::string> Split(const std::string &text, char separator)
        {
            std::vector<std::string> result;
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
            return path.extension() == ".ini" && path.filename().string().find("_SWAP") != std::string::npos;
        }

        std::string Read(const std::filesystem::path &path)
        {
            std::ifstream file(path, std::ios::binary);
            if (!file)
            {
                throw std::runtime_error("Cannot read BOS configuration: " + path.string());
            }
            std::string contents{std::istreambuf_iterator<char>(file), {}};
            if (file.bad())
            {
                throw std::runtime_error("Cannot completely read BOS configuration: " + path.string());
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
            const auto numeric = Trim(text);
            std::size_t consumed{};
            const auto value = std::stof(numeric, &consumed);
            if (consumed != numeric.size() || !std::isfinite(value))
            {
                throw std::runtime_error("Expected a finite BOS numeric value: " + text);
            }
            return value;
        }

        std::uint32_t Hex(const std::string &text)
        {
            const auto numeric = Trim(text);
            std::size_t consumed{};
            const auto value = std::stoull(numeric, &consumed, 16);
            if (consumed != numeric.size() || value > UINT32_MAX)
            {
                throw std::runtime_error("Invalid BOS FormID or flag: " + text);
            }
            return static_cast<std::uint32_t>(value);
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
            if (parts.size() > 2)
            {
                throw std::runtime_error("Invalid BOS range: " + text);
            }
            Range result{Number(parts.front()), Number(parts.back())};
            if (result.minimum > result.maximum)
            {
                throw std::runtime_error("Reversed BOS range: " + text);
            }
            return result;
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

        /// Split properties at closing parentheses; commas inside a vector are component separators.
        Properties ParseProperties(const std::string &text)
        {
            Properties result;
            if (text.empty() || Lower(text) == "none")
            {
                return result;
            }
            std::size_t begin{};
            while (begin < text.size())
            {
                const auto open = text.find('(', begin);
                const auto close = text.find(')', open);
                if (open == std::string::npos || close == std::string::npos)
                {
                    throw std::runtime_error("Malformed BOS property: " + text.substr(begin));
                }
                const auto name = Trim(text.substr(begin, open - begin));
                const auto arguments = Split(text.substr(open + 1, close - open - 1), ',');
                if (name == "pos" || name == "posA" || name == "posR" || name == "rot" || name == "rotA" ||
                    name == "rotR")
                {
                    if (arguments.size() != 3)
                    {
                        throw std::runtime_error("BOS position/rotation requires three components");
                    }
                    PointProperty point{{ParseRange(arguments[0]), ParseRange(arguments[1]), ParseRange(arguments[2])},
                                        name.ends_with('R')};
                    if (name.starts_with("rot"))
                    {
                        const bool sampled = point.Variable();
                        for (auto &range : point.values)
                        {
                            constexpr auto radians = std::numbers::pi_v<float> / 180.0F;
                            constexpr auto limit = std::numbers::pi_v<float> * 2;
                            range.minimum *= radians;
                            range.maximum *= radians;
                            // BOS clamps sampled vectors, while exact vectors bypass sampling.
                            if (sampled)
                            {
                                range.minimum = std::clamp(range.minimum, -limit, limit);
                                range.maximum = std::clamp(range.maximum, -limit, limit);
                            }
                        }
                        result.rotation = point;
                    }
                    else
                    {
                        result.position = point;
                    }
                }
                else if (name == "scale" || name == "scaleA" || name == "scaleR")
                {
                    if (arguments.size() != 1)
                    {
                        throw std::runtime_error("BOS scale requires one value or range");
                    }
                    auto range = ParseRange(arguments[0]);
                    range.minimum = std::clamp(range.minimum, 0.0F, 1000.0F);
                    range.maximum = std::clamp(range.maximum, 0.0F, 1000.0F);
                    result.scale = range;
                    result.absoluteScale = name == "scaleA";
                }
                else if (name == "flags" || name == "flagsC")
                {
                    for (const auto &argument : arguments)
                    {
                        (name == "flags" ? result.setFlags : result.clearFlags) |= Hex(argument);
                    }
                }
                else
                {
                    throw std::runtime_error("Unsupported BOS property: " + name);
                }
                begin = close + 1;
                while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin])))
                {
                    ++begin;
                }
                if (begin < text.size())
                {
                    if (text[begin] != ',')
                    {
                        throw std::runtime_error("Expected a comma between BOS properties");
                    }
                    ++begin;
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
        std::vector<ConfigEntry> ConfigEntries(const BaseObjectSwapConfig &config)
        {
            std::istringstream stream(IniNewlines(config.contents));
            struct Section
            {
                std::string name;
                std::vector<ConfigEntry> entries;
                std::set<std::string> keys;
            };
            std::vector<Section> sections;
            std::map<std::string, std::size_t> indices;
            std::string section, line;
            std::size_t lineNumber{}, current{};
            while (std::getline(stream, line))
            {
                ++lineNumber;
                if (lineNumber == 1 && line.starts_with("\xEF\xBB\xBF"))
                {
                    line.erase(0, 3);
                }
                line = Trim(line);
                if (line.empty() || line.front() == ';' || line.front() == '#')
                {
                    continue;
                }
                if (line.front() == '[')
                {
                    const auto close = line.find(']');
                    if (close == std::string::npos)
                    {
                        const auto source = config.path.empty() ? config.name : config.path.generic_string();
                        throw std::runtime_error(source + ":" + std::to_string(lineNumber) +
                                                 ": Unterminated BOS section header");
                    }
                    // SimpleIni trims the section name and ignores text after its closing bracket.
                    section = Trim(line.substr(1, close - 1));
                    const auto [found, inserted] = indices.emplace(Lower(section), sections.size());
                    if (inserted)
                    {
                        sections.push_back({section});
                    }
                    current = found->second;
                    continue;
                }
                if (sections.empty())
                {
                    indices.emplace("", 0);
                    sections.emplace_back();
                }
                // BOS visits unique case-insensitive keys, using their first spelling and load position.
                // INI values after '=' are not part of the rule supplied to BOS.
                line = Trim(line.substr(0, line.find('=')));
                if (!line.empty() && sections[current].keys.insert(Lower(line)).second)
                {
                    sections[current].entries.push_back({sections[current].name, line, lineNumber});
                }
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
                    const auto prefix = plugin->second;
                    const auto mask = (prefix >> 24) == 0xFE ? 0xFFFU : 0xFFFFFFU;
                    const auto local = Hex(parts[0]);
                    if (local > mask)
                    {
                        throw std::runtime_error("BOS qualified FormID exceeds its plugin's local identity range: " +
                                                 text);
                    }
                    return prefix | local;
                }
                if (parts.size() != 1)
                {
                    throw std::runtime_error("Invalid BOS identity: " + text);
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
            const auto modifier = Trim(text);
            if (modifier.empty() || Lower(modifier) == "none")
            {
                return 100;
            }
            const auto open = modifier.find('(');
            const auto close = modifier.find(')');
            if (!modifier.starts_with("chance") || open == std::string::npos || close != modifier.size() - 1)
            {
                throw std::runtime_error("Invalid BOS chance modifier");
            }
            const auto chance = Number(Split(modifier.substr(open + 1, close - open - 1), ',').front());
            return chance;
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
                for (const auto &entry : ConfigEntries(config))
                {
                    const auto &section = entry.section;
                    const auto &line = entry.line;
                    const auto evidence = config.name + ":" + std::to_string(entry.lineNumber);
                    const auto source = config.path.empty() ? config.name : config.path.generic_string();
                    try
                    {
                        const auto sectionParts = Split(section, '|');
                        // BOS routes every conditional non-Forms section to base properties;
                        // unconditional sections other than Forms/Transforms/Properties are reference swaps.
                        const bool property = sectionParts.size() > 1
                                                  ? sectionParts[0] != "Forms"
                                                  : sectionParts[0] == "Transforms" || sectionParts[0] == "Properties";
                        const auto fields = Split(line, '|');
                        if (fields.size() < 2 || fields.size() > (property ? 3U : 4U))
                        {
                            throw std::runtime_error("Invalid BOS entry field count");
                        }
                        Rule rule;
                        rule.evidence = evidence;
                        const auto sourceNames = Split(fields[0], ',');
                        if (property && sourceNames.size() > 1)
                        {
                            throw std::runtime_error("BOS property entries require a single source identity");
                        }
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
                        if (sectionParts.size() == 2)
                        {
                            rule.conditions = Split(sectionParts[1], ',');
                            rule.conditionKey = config.name + "|" + sectionParts[1];
                        }
                        else if (sectionParts.size() != 1)
                        {
                            throw std::runtime_error("Invalid BOS conditional section");
                        }
                        rule.properties =
                            ParseProperties(fields.size() > (property ? 1U : 2U) ? fields[property ? 1 : 2] : "");
                        const auto chanceIndex = property ? 2U : 3U;
                        rule.chance = ParseChance(fields.size() > chanceIndex ? fields[chanceIndex] : "");
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
                            if (sourceNames.size() > 1 && replacementIds.size() > 1)
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
                                rule.chance = 100;
                            }
                        }
                        auto &destination = property                     ? result.properties
                                            : sectionParts[0] == "Forms" ? result.forms
                                                                         : result.references;
                        for (const auto id : sourceIds)
                        {
                            if (!property && sourceNames.size() == 1 && fields[1].find(',') == std::string::npos &&
                                rule.replacements.size() == 1 && rule.replacements.front() == id &&
                                (fields.size() < 3 || fields[2].empty() || Lower(fields[2]) == "none"))
                            {
                                continue;
                            }
                            destination[id].push_back(rule);
                        }
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
                if (rule->conditions.empty() != conditional)
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
                if (entry.is_regular_file() && Lower(entry.path().extension().string()) == ".ini")
                {
                    winners[Lower(entry.path().filename().string())] = entry.path();
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
            result.configs.push_back({path.filename().string(), path, std::move(contents)});
        }
        std::sort(result.configs.begin(), result.configs.end(),
                  [](const auto &a, const auto &b) { return a.name < b.name; });
        for (const auto &config : result.configs)
        {
            hash.Add(config.name);
            hash.Add(config.path.generic_string());
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
                << ",\"path\":" << Json(config.path.generic_string()) << "}";
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
