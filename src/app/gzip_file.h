#pragma once

#include "core/io/file_path.h"
#include <zlib.h>

namespace navmesh::app::detail
{
    /** Open gzip data with native path encoding and Windows extended-path support.
     * Relative Windows paths are resolved and normalized before adding the local
     * or UNC extended-path prefix; an existing extended prefix is preserved.
     * @param path File to read or write, including paths beyond the legacy Windows limit.
     * @param mode Valid zlib file mode, such as "rb" or "wb".
     * @return Owned zlib handle to close with gzclose, or null if opening fails.
     * @throws std::filesystem::filesystem_error If an absolute Windows path cannot be resolved.
     */
    [[nodiscard]] inline gzFile OpenGzipFile(const std::filesystem::path &path, const char *mode)
    {
#ifdef _WIN32
        return gzopen_w(core::NativeFilePath(path).c_str(), mode);
#else
        return gzopen(path.c_str(), mode);
#endif
    }
} // namespace navmesh::app::detail
