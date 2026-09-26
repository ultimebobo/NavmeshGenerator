#include "app/options.h"
#include "app/run.h"
#include "ui/windows_ui.h"

int main(int argc, char** argv)
{
    // No command-line switches means the desktop adapter is the default experience.
    if (argc == 1) return navmesh::ui::RunWindowsUi({});
    return navmesh::app::Run(navmesh::app::ParseCommandLine(argc, argv));
}
