#include "skyrim/parser/reference_navigation.h"

#include <algorithm>
#include <cstring>
#include <set>
#include <span>

namespace
{
    using Bytes = std::vector<std::uint8_t>;
    using navmesh::skyrim::detail::ReferenceFormIdRebaser;

    std::uint16_t Read16(std::span<const std::uint8_t> bytes, std::size_t offset)
    {
        return static_cast<std::uint16_t>(bytes[offset] | bytes[offset + 1] << 8);
    }

    std::uint32_t Read32(std::span<const std::uint8_t> bytes, std::size_t offset)
    {
        std::uint32_t value{};
        std::memcpy(&value, bytes.data() + offset, sizeof(value));
        return value;
    }

    void Set32(Bytes &bytes, std::size_t offset, std::uint32_t value)
    {
        std::memcpy(bytes.data() + offset, &value, sizeof(value));
    }

    /// Encode the output-local NAVM identity and triangle at the REFR's navigation-field position.
    void AppendDoorAssociation(Bytes &output, const navmesh::skyrim::NavmeshDoorAssociation &association)
    {
        output.insert(output.end(), {'X', 'N', 'D', 'P', 8, 0});
        const auto start = output.size();
        output.resize(start + 8);
        Set32(output, start, association.navmeshId);
        output[start + 4] = static_cast<std::uint8_t>(association.triangle);
        output[start + 5] = static_cast<std::uint8_t>(association.triangle >> 8);
    }

    bool RebaseAt(Bytes &bytes, std::size_t offset, const ReferenceFormIdRebaser &rebase)
    {
        if (offset > bytes.size() || bytes.size() - offset < 4)
        {
            return false;
        }
        const auto original = Read32(bytes, offset);
        if (!original)
        {
            return true;
        }
        const auto result = rebase(original);
        if (!result)
        {
            return false;
        }
        Set32(bytes, offset, *result);
        return true;
    }

    bool SkipString(std::span<const std::uint8_t> bytes, std::size_t &offset)
    {
        if (offset > bytes.size() || bytes.size() - offset < 2)
        {
            return false;
        }
        const auto count = Read16(bytes, offset);
        offset += 2;
        if (count > bytes.size() - offset)
        {
            return false;
        }
        offset += count;
        return true;
    }

    /// REFR VMAD contains scripts and typed properties, without quest/scene fragment tails.
    /// Object format selects the FormID's position inside its fixed-width object/alias tuple.
    bool RebaseScripts(Bytes &bytes, const ReferenceFormIdRebaser &rebase)
    {
        if (bytes.size() < 6 || Read16(bytes, 0) != 5)
        {
            return false;
        }
        const auto format = Read16(bytes, 2);
        if (format != 1 && format != 2)
        {
            return false;
        }
        const auto scripts = Read16(bytes, 4);
        std::size_t offset = 6;
        for (std::uint32_t script{}; script < scripts; ++script)
        {
            if (!SkipString(bytes, offset) || bytes.size() - offset < 3)
            {
                return false;
            }
            ++offset;
            const auto properties = Read16(bytes, offset);
            offset += 2;
            for (std::uint32_t property{}; property < properties; ++property)
            {
                if (!SkipString(bytes, offset) || bytes.size() - offset < 2)
                {
                    return false;
                }
                auto type = bytes[offset];
                offset += 2;
                if (type == 0)
                {
                    continue;
                }
                std::uint32_t count = 1;
                if (type >= 11 && type <= 15)
                {
                    type -= 10;
                    if (bytes.size() - offset < 4)
                    {
                        return false;
                    }
                    count = Read32(bytes, offset);
                    offset += 4;
                }
                const std::size_t width = type == 1 ? 8 : type == 2 ? 2 : type == 5 ? 1 : 4;
                if (type < 1 || type > 5 || count > (bytes.size() - offset) / width)
                {
                    return false;
                }
                for (std::uint32_t element{}; element < count; ++element)
                {
                    if (type == 2)
                    {
                        if (!SkipString(bytes, offset))
                        {
                            return false;
                        }
                    }
                    else
                    {
                        if (bytes.size() - offset < width ||
                            (type == 1 && !RebaseAt(bytes, offset + (format == 2 ? 4 : 0), rebase)))
                        {
                            return false;
                        }
                        offset += width;
                    }
                }
            }
        }
        return offset == bytes.size();
    }

    bool RebaseField(const navmesh::skyrim::Subrecord &sub, Bytes &bytes, const ReferenceFormIdRebaser &rebase)
    {
        static const std::set<std::string> singleForms = {"NAME", "XOWN", "XGLB", "XLCN", "XEZN", "XLIB",
                                                          "XLRL", "XLRM", "XEMI", "XMBR", "XLTW", "XATR",
                                                          "INAM", "LNAM", "XTNM", "XCZR", "XCZC", "XSPC"};
        static const std::set<std::string> plainFields = {
            "EDID", "FULL", "XMBO", "XPRM", "XOCP", "XPTL", "XRMR", "XMBP", "XRGD", "XRGB", "XRDS", "XLIG", "XALP",
            "XWCN", "XWCS", "XWCU", "XCVL", "XCVR", "XCZA", "XSCL", "XAPD", "XLCM", "XTRI", "XRNK", "XCNT", "XCHG",
            "XPRD", "XPPA", "XIS2", "XACT", "XHTW", "XFVC", "ONAM", "XMRK", "FNAM", "TNAM", "XLOD", "DATA"};
        if (singleForms.contains(sub.type))
        {
            return bytes.size() == 4 && RebaseAt(bytes, 0, rebase);
        }
        if (sub.type == "VMAD")
        {
            return RebaseScripts(bytes, rebase);
        }
        if (sub.type == "XORD" || sub.type == "XPOD" || sub.type == "XLRT" || sub.type == "XLKR")
        {
            if (bytes.size() % 4 != 0 || (sub.type == "XLKR" && bytes.size() != 4 && bytes.size() != 8))
            {
                return false;
            }
            for (std::size_t offset{}; offset < bytes.size(); offset += 4)
            {
                if (!RebaseAt(bytes, offset, rebase))
                {
                    return false;
                }
            }
            return true;
        }
        if (sub.type == "XESP" || sub.type == "XAPR" || sub.type == "XPWR" || sub.type == "XTEL")
        {
            const bool sizeValid = sub.type == "XTEL"   ? (bytes.size() == 28 || bytes.size() == 32)
                                   : sub.type == "XPWR" ? (bytes.size() == 4 || bytes.size() == 8)
                                                        : bytes.size() == 8;
            return sizeValid && RebaseAt(bytes, 0, rebase);
        }
        if (sub.type == "XLOC")
        {
            return (bytes.size() == 12 || bytes.size() == 16 || bytes.size() == 20) && RebaseAt(bytes, 4, rebase);
        }
        if (sub.type == "PDTO")
        {
            // Topic references carry FormIDs; topic subtypes carry literal four-character signatures.
            return bytes.size() == 8 && (Read32(bytes, 0) == 0 ? RebaseAt(bytes, 4, rebase) : Read32(bytes, 0) == 1);
        }
        return plainFields.contains(sub.type);
    }
} // namespace

bool navmesh::skyrim::detail::RewriteReferenceNavigation(const PluginRecord &source,
                                                         const ReferenceFormIdRebaser &rebase,
                                                         std::optional<NavmeshDoorAssociation> association,
                                                         bool retainsStringTable, bool localizedSource,
                                                         std::vector<std::uint8_t> &output, std::string &error)
{
    output.clear();
    const auto bases = std::count_if(source.subrecords.begin(), source.subrecords.end(),
                                     [](const auto &sub) { return sub.type == "NAME"; });
    if (bases > 1 || (!(source.flags & 0x20U) && bases != 1))
    {
        error = "A live door reference must contain exactly one base-object field.";
        return false;
    }
    bool associationWritten = false;
    for (const auto &sub : source.subrecords)
    {
        // Preserve an authored navigation slot, or insert before the final DATA
        // placement field when the reference receives its first association.
        if (association && !associationWritten && (sub.type == "XNDP" || sub.type == "DATA"))
        {
            AppendDoorAssociation(output, *association);
            associationWritten = true;
        }
        if (sub.type == "XNDP")
        {
            continue;
        }
        if (sub.type == "FULL" && localizedSource && (!retainsStringTable || sub.data.size() != 4))
        {
            error = "Cannot copy a localized REFR name without its original string-table resources.";
            return false;
        }
        Bytes bytes(sub.data.begin(), sub.data.end());
        if (!RebaseField(sub, bytes, rebase))
        {
            error = "Cannot safely rebase REFR " + sub.type + "; its payload or a dependency is unsupported.";
            return false;
        }
        const auto start = output.size();
        output.insert(output.end(), sub.encodedBytes.begin(), sub.encodedBytes.end());
        const auto header = sub.encodedBytes.size() - sub.data.size();
        std::copy(bytes.begin(), bytes.end(), output.begin() + start + header);
    }
    if (association && !associationWritten)
    {
        AppendDoorAssociation(output, *association);
    }
    return true;
}
