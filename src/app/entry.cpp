#include "app/options.h"
#include "app/run.h"
#include "ui/windows_ui.h"

#include <exception>
#include <iostream>

int main(int argc, char** argv)
{
    // No command-line switches means the desktop adapter is the default experience.
    if (argc == 1) return navmesh::ui::RunWindowsUi({});
    try {
        return navmesh::app::Run(navmesh::app::ParseCommandLine(argc, argv),
            [](int percent, std::string_view status) {
                if (percent >= 78) std::cerr << percent << "% " << status << '\n';
            });
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
