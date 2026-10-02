#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>
#include <utility>

namespace navmesh::core
{
    /// Immutable byte range with shared ownership; copies and slices keep their backing storage alive.
    class SharedBytes
    {
      public:
        /// Construct an empty range without allocating storage.
        SharedBytes() = default;
        /// Copies share ownership of the same immutable range.
        SharedBytes(const SharedBytes &) = default;
        /// Assignment shares ownership of the same immutable range.
        SharedBytes &operator=(const SharedBytes &) = default;
        /// Transfer ownership and leave the source empty.
        SharedBytes(SharedBytes &&other) noexcept
            : storage_(std::move(other.storage_)), offset_(std::exchange(other.offset_, 0)),
              length_(std::exchange(other.length_, 0))
        {
        }
        /// Transfer ownership and leave the source empty; self-assignment preserves the range.
        SharedBytes &operator=(SharedBytes &&other) noexcept
        {
            if (this != &other)
            {
                storage_ = std::move(other.storage_);
                offset_ = std::exchange(other.offset_, 0);
                length_ = std::exchange(other.length_, 0);
            }
            return *this;
        }
        /// Take ownership of bytes; copies share immutable storage.
        SharedBytes(std::vector<std::uint8_t> bytes)
            : storage_(std::make_shared<const std::vector<std::uint8_t>>(std::move(bytes))), length_(storage_->size())
        {
        }
        /** Share a subrange without copying bytes.
         * @param offset Byte offset relative to this range.
         * @param length Number of bytes; must fit within this range.
         * @return Owned view; throws std::out_of_range for invalid bounds.
         */
        [[nodiscard]] SharedBytes Slice(std::size_t offset, std::size_t length) const
        {
            if (offset > length_ || length > length_ - offset)
            {
                throw std::out_of_range("Shared byte slice exceeds its backing range");
            }
            SharedBytes result;
            result.storage_ = storage_;
            result.offset_ = offset_ + offset;
            result.length_ = length;
            return result;
        }
        /// Borrow bytes synchronously; the owner must outlive the span.
        [[nodiscard]] operator std::span<const std::uint8_t>() const noexcept
        {
            return {data(), length_};
        }
        /// First readable byte, or nullptr for a default empty range.
        [[nodiscard]] const std::uint8_t *data() const noexcept
        {
            return storage_ && !storage_->empty() ? storage_->data() + offset_ : nullptr;
        }
        /// Number of readable bytes.
        [[nodiscard]] std::size_t size() const noexcept
        {
            return length_;
        }
        /// Whether this range contains no bytes.
        [[nodiscard]] bool empty() const noexcept
        {
            return length_ == 0;
        }
        /// Read a byte; index must be less than size().
        [[nodiscard]] const std::uint8_t &operator[](std::size_t index) const noexcept
        {
            return data()[index];
        }
        /// Start of the immutable byte range.
        [[nodiscard]] const std::uint8_t *begin() const noexcept
        {
            return data();
        }
        /// Past-the-end pointer; default empty ranges return nullptr.
        [[nodiscard]] const std::uint8_t *end() const noexcept
        {
            return length_ ? data() + length_ : data();
        }
        /// Compare readable contents independently of their backing storage.
        friend bool operator==(const SharedBytes &left, const SharedBytes &right)
        {
            return left.size() == right.size() && std::equal(left.begin(), left.end(), right.begin());
        }

      private:
        std::shared_ptr<const std::vector<std::uint8_t>> storage_;
        std::size_t offset_{};
        std::size_t length_{};
    };
} // namespace navmesh::core
