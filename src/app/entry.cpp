#include "app/options.h"
#include "app/run.h"
#include "ui/windows_ui.h"

#include <exception>
#include <iostream>
#include <filesystem>
#include <string>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

#ifdef _WIN32
int wmain(int argc, wchar_t **wideArguments)
#else
int main(int argc, char **argv)
#endif
{
#ifdef _WIN32
    // Keep command-line text and console diagnostics in the same encoding as manifests and the desktop adapter.
    SetConsoleOutputCP(CP_UTF8);
    std::vector<std::string> arguments;
    arguments.reserve(argc);
    for (int index{}; index < argc; ++index)
    {
        const auto bytes = std::filesystem::path(wideArguments[index]).u8string();
        arguments.emplace_back(reinterpret_cast<const char *>(bytes.data()), bytes.size());
    }
    std::vector<char *> pointers;
    pointers.reserve(argc);
    for (auto &argument : arguments)
    {
        pointers.push_back(argument.data());
    }
    auto **argv = pointers.data();
#endif
    // No command-line switches means the desktop adapter is the default experience.
    if (argc == 1)
    {
        return navmesh::ui::RunWindowsUi({});
    }
    try
    {
        return navmesh::app::Run(navmesh::app::ParseCommandLine(argc, argv),
                                 [](int percent, std::string_view status)
                                 {
                                     if (percent >= 78)
                                     {
                                         std::cerr << percent << "% " << status << '\n';
                                     }
                                 });
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
