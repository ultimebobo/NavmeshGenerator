#pragma once

#include <filesystem>

namespace navmesh::core
{
    /** Prepare a file path for native I/O, including long Windows local and UNC paths.
     * Windows paths are made absolute and normalized before adding an extended
     * prefix; paths already using that prefix are preserved. Other platforms keep
     * the supplied path. This does not access or create the target file.
     * @param path File or directory path to pass to an I/O API.
     * @return Native path with the same target; Windows separators use backslashes.
     * @throws std::filesystem::filesystem_error If an absolute Windows path cannot be resolved.
     */
    [[nodiscard]] inline std::filesystem::path NativeFilePath(const std::filesystem::path &path)
    {
#ifdef _WIN32
        if (path.native().starts_with(L"\\\\?\\"))
        {
            return path;
        }
        const auto native = std::filesystem::absolute(path).lexically_normal().make_preferred().native();
        return native.starts_with(L"\\\\") ? L"\\\\?\\UNC\\" + native.substr(2) : L"\\\\?\\" + native;
#else
        return path;
#endif
    }
} // namespace navmesh::core
