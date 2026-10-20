#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>

namespace navmesh::skyrim
{
    namespace detail
    {
        // Undefined Windows-1252 slots retain their control-code identity for lossless round trips.
        inline constexpr std::array<std::uint32_t, 32> Windows1252Controls = {
            0x20AC, 0x81,   0x201A, 0x192,  0x201E, 0x2026, 0x2020, 0x2021, 0x2C6,  0x2030, 0x160,
            0x2039, 0x152,  0x8D,   0x17D,  0x8F,   0x90,   0x2018, 0x2019, 0x201C, 0x201D, 0x2022,
            0x2013, 0x2014, 0x2DC,  0x2122, 0x161,  0x203A, 0x153,  0x9D,   0x17E,  0x178};

        /// Consume one Unicode scalar, rejecting truncated, overlong, surrogate and out-of-range UTF-8.
        inline bool ReadUtf8(std::string_view text, std::size_t &position, std::uint32_t &scalar)
        {
            const auto first = static_cast<unsigned char>(text[position++]);
            if (first < 0x80)
            {
                scalar = first;
                return true;
            }
            const auto count = first >= 0xC2 && first <= 0xDF   ? 1
                               : first >= 0xE0 && first <= 0xEF ? 2
                               : first >= 0xF0 && first <= 0xF4 ? 3
                                                                : 0;
            if (!count || text.size() - position < static_cast<std::size_t>(count))
            {
                return false;
            }
            scalar = first & (0x7F >> (count + 1));
            for (int index{}; index < count; ++index)
            {
                const auto next = static_cast<unsigned char>(text[position++]);
                if ((next & 0xC0) != 0x80)
                {
                    return false;
                }
                scalar = (scalar << 6) | (next & 0x3F);
            }
            const std::uint32_t minimum = count == 1 ? 0x80 : count == 2 ? 0x800 : 0x10000;
            return scalar >= minimum && scalar <= 0x10FFFF && !(scalar >= 0xD800 && scalar <= 0xDFFF);
        }

        /// Append one Windows-1252 mapping's BMP scalar as UTF-8.
        inline void AppendUtf8(std::string &text, std::uint32_t scalar)
        {
            if (scalar < 0x80)
            {
                text.push_back(static_cast<char>(scalar));
            }
            else if (scalar < 0x800)
            {
                text.push_back(static_cast<char>(0xC0 | (scalar >> 6)));
                text.push_back(static_cast<char>(0x80 | (scalar & 0x3F)));
            }
            else
            {
                text.push_back(static_cast<char>(0xE0 | (scalar >> 12)));
                text.push_back(static_cast<char>(0x80 | ((scalar >> 6) & 0x3F)));
                text.push_back(static_cast<char>(0x80 | (scalar & 0x3F)));
            }
        }
    } // namespace detail

    /** Decode embedded plugin text or BSA names to UTF-8, independently of the host locale.
     * Valid UTF-8 is retained for mods authored with that encoding; other byte strings
     * use Windows-1252. Raw record payloads must be retained separately for copying.
     */
    [[nodiscard]] inline std::string DecodePluginText(std::string_view bytes)
    {
        std::size_t position{};
        std::uint32_t scalar{};
        bool valid = true;
        while (position < bytes.size() && valid)
        {
            valid = detail::ReadUtf8(bytes, position, scalar);
        }
        if (valid)
        {
            return std::string(bytes);
        }
        std::string text;
        for (const unsigned char byte : bytes)
        {
            detail::AppendUtf8(text, byte >= 0x80 && byte < 0xA0 ? detail::Windows1252Controls[byte - 0x80] : byte);
        }
        return text;
    }

    /** Encode generated embedded plugin text as Windows-1252 when representable.
     * @param text Valid UTF-8 application text.
     * @return Legacy bytes, or the complete UTF-8 input when any scalar is unrepresentable.
     */
    [[nodiscard]] inline std::string EncodePluginText(std::string_view text)
    {
        std::string bytes;
        std::size_t position{};
        std::uint32_t scalar{};
        while (position < text.size())
        {
            if (!detail::ReadUtf8(text, position, scalar))
            {
                return std::string(text);
            }
            if (scalar < 0x80 || (scalar >= 0xA0 && scalar <= 0xFF))
            {
                bytes.push_back(static_cast<char>(scalar));
                continue;
            }
            std::size_t index{};
            while (index < detail::Windows1252Controls.size() && detail::Windows1252Controls[index] != scalar)
            {
                ++index;
            }
            if (index == detail::Windows1252Controls.size())
            {
                return std::string(text);
            }
            bytes.push_back(static_cast<char>(0x80 + index));
        }
        return bytes;
    }
} // namespace navmesh::skyrim
