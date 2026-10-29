#include "skyrim/parser/plugin_parser.h"
#include "core/navmesh/candidate.h"

#include <bit>
#include <cmath>
#include <cctype>

namespace navmesh::skyrim
{
    float ResolveNpcSafeDropHeight(const ResolvedLoadOrder &resolved)
    {
        for (const auto &record : resolved.records)
        {
            auto name = record.editorId;
            for (auto &letter : name)
            {
                letter = static_cast<char>(std::tolower(static_cast<unsigned char>(letter)));
            }
            if (record.type != "GMST" || name != "fjumpfallheightminnpc")
            {
                continue;
            }
            if (!record.raw || (record.raw->flags & (1U << 5)))
            {
                return 0;
            }
            for (const auto &sub : record.raw->subrecords)
            {
                if (sub.type == "DATA" && sub.data.size() == 4)
                {
                    const auto bits = static_cast<std::uint32_t>(sub.data[0]) |
                                      (static_cast<std::uint32_t>(sub.data[1]) << 8) |
                                      (static_cast<std::uint32_t>(sub.data[2]) << 16) |
                                      (static_cast<std::uint32_t>(sub.data[3]) << 24);
                    const auto value = std::bit_cast<float>(bits);
                    return std::isfinite(value) && value >= 0 ? value : 0;
                }
            }
            return 0;
        }
        return core::NavigationProfile{}.maximumDropHeight;
    }
} // namespace navmesh::skyrim
