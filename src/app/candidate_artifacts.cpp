#include "app/candidate_artifacts.h"

#include <array>
#include <limits>
#include <ostream>
#include <streambuf>
#include <zlib.h>

namespace navmesh::app::detail
{
    namespace
    {
        /// A small buffered stream bridge keeps text export memory and staging disk independent of JSON size.
        class GzipBuffer final : public std::streambuf
        {
          public:
            explicit GzipBuffer(gzFile file) : file_(file)
            {
                setp(buffer_.data(), buffer_.data() + buffer_.size());
            }
            int sync() override
            {
                const auto count = static_cast<unsigned int>(pptr() - pbase());
                if (count && gzwrite(file_, pbase(), count) != static_cast<int>(count))
                {
                    return -1;
                }
                setp(buffer_.data(), buffer_.data() + buffer_.size());
                return 0;
            }
            int_type overflow(int_type character) override
            {
                if (sync() != 0)
                {
                    return traits_type::eof();
                }
                if (!traits_type::eq_int_type(character, traits_type::eof()))
                {
                    *pptr() = traits_type::to_char_type(character);
                    pbump(1);
                }
                return traits_type::not_eof(character);
            }

          private:
            gzFile file_;
            std::array<char, 65536> buffer_{};
        };
    } // namespace

    bool WriteCompressedCandidateJson(const std::filesystem::path &path, const core::CandidateNavMesh &candidate,
                                      const core::Scene &scene, const std::string &metadata)
    {
#ifdef _WIN32
        auto file = gzopen_w(path.c_str(), "wb");
#else
        auto file = gzopen(path.c_str(), "wb");
#endif
        if (!file)
        {
            return false;
        }
        GzipBuffer buffer(file);
        std::ostream stream(&buffer);
        const bool written = core::WriteCandidateJson(stream, candidate, scene, metadata);
        stream.flush();
        const bool good = stream.good();
        const bool closed = gzclose(file) == Z_OK;
        return written && good && closed;
    }
} // namespace navmesh::app::detail
