#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace navmesh::core
{
    /// Incremental SHA-256 identity for cache dependencies; byte order is supplied by the caller.
    class ContentHash
    {
      public:
        /// Create an empty digest.
        ContentHash();
        /// Append bytes without retaining their storage.
        void Add(std::span<const std::uint8_t> bytes);
        /// Append UTF-8 or binary string bytes, without a terminator.
        void Add(std::string_view bytes);
        /// Return lowercase hexadecimal SHA-256; leaves this digest available for further additions.
        [[nodiscard]] std::string Hex() const;

      private:
        void Transform(const std::uint8_t *block);
        std::array<std::uint32_t, 8> state_;
        std::array<std::uint8_t, 64> pending_{};
        std::uint64_t size_{};
        std::size_t pendingSize_{};
    };
} // namespace navmesh::core
