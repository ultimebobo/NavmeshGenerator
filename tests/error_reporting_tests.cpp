// Error/status contracts must be checked with assertions enabled in every build mode.
#ifdef NDEBUG
#undef NDEBUG
#endif

#include "app/batch_runner.h"
#include "app/run.h"
#include "cli/inspection_report.h"

#include <cassert>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace
{
    navmesh::app::Options Parse(std::vector<std::string> arguments)
    {
        arguments.insert(arguments.begin(), "NavmeshGenerator");
        std::vector<char *> pointers;
        for (auto &argument : arguments)
        {
            pointers.push_back(argument.data());
        }
        return navmesh::app::ParseCommandLine(static_cast<int>(pointers.size()), pointers.data());
    }

    void CheckParseError(std::vector<std::string> arguments, std::string_view option, std::string_view detail)
    {
        try
        {
            (void)Parse(std::move(arguments));
            assert(false && "Invalid CLI input must be rejected before running");
        }
        catch (const std::invalid_argument &error)
        {
            const std::string_view message = error.what();
            assert(message.contains(option));
            assert(message.contains(detail));
        }
    }
} // namespace

void TestActionableErrors()
{
    using namespace navmesh;
    CheckParseError({"--agent-radius", "oops"}, "--agent-radius", "oops");
    CheckParseError({"--cell-formid", "100000000"}, "--cell-formid", "hexadecimal");
    CheckParseError({"--cell-formid", "100junk"}, "--cell-formid", "100junk");
    CheckParseError({"--cell-formid", "-1"}, "--cell-formid", "hexadecimal");
    CheckParseError({"--cell-x", "1junk"}, "--cell-x", "whole number");
    CheckParseError({"--cell-y", "9999999999999999"}, "--cell-y", "whole number");
    CheckParseError({"--workers", " -1"}, "--workers", "nonnegative");
    CheckParseError({"--workers", "99999999999999999999999"}, "--workers", "whole number");
    CheckParseError({"--max-slope", "nan"}, "--max-slope", "finite");
    CheckParseError({"--max-support-distance", "5junk"}, "--max-support-distance", "5junk");
    CheckParseError({"--output"}, "--output", "requires a value");
    CheckParseError({"--mo2", "--profile", "Fixture"}, "--mo2", "requires a value");
    CheckParseError({"--scene-bounds", "0", "0", "--generate-candidate"}, "--scene-bounds", "four numbers");
    CheckParseError({"--generate-plguin"}, "--generate-plguin", "Unknown option");
    CheckParseError({"--cells", "Fixture", "unquoted"}, "unquoted", "quote paths or names");
    const auto parsed = Parse({"--cell-formid", "0xFFFFFFFF", "--cell-x", "-2", "--cell-y", "3", "--scene-bounds",
                               "-10", "-20", "30", "40", "--workers", "2"});
    assert(parsed.cellFormId == 0xFFFFFFFF && parsed.cellX == -2 && parsed.cellY == 3 && parsed.workers == 2);
    assert(parsed.sceneBounds && (*parsed.sceneBounds)[0] == -10);

    const auto root =
        std::filesystem::temp_directory_path() /
        ("navmesh-error-fixture-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);
    app::Options options;
    options.mo2 = root / "missing-instance";
    options.profile = "Fixture";
    options.output = root / "output";
    options.assetCache = root / "cache";
    options.listCells = true;
    std::string terminal;
    const app::ProgressCallback progress = [&](int, std::string_view message) { terminal = message; };

    // Desktop status and CLI stderr must expose the same concrete failure and correction.
    options.workers = 0;
    std::ostringstream console;
    auto *previousBuffer = std::cerr.rdbuf(console.rdbuf());
    const auto status = app::Run(options, progress);
    std::cerr.rdbuf(previousBuffer);
    assert(status == 1);
    assert(terminal.contains("--workers: 0") && terminal.contains("Enter"));
    assert(console.str() == terminal + "\n");
    options.workers = app::Options{}.workers;
    assert(app::Run(options, progress) == 2);
    assert(terminal.contains("ModOrganizer.ini") && terminal.contains("missing-instance"));
    assert(terminal.contains("Select") && terminal.contains("input-report.json"));
    assert(std::filesystem::is_regular_file(options.output / "input-report.json"));

    // A report destination occupied by a directory must not be presented as a saved report.
    std::filesystem::remove(options.output / "input-report.json");
    std::filesystem::create_directory(options.output / "input-report.json");
    assert(app::Run(options, progress) == 2);
    assert(terminal.contains("could not be saved") && !terminal.contains("Input report:"));

    const auto blocked = root / "blocked-output";
    std::ofstream(blocked) << "fixture";
    options.output = blocked;
    assert(app::Run(options, progress) == 1);
    assert(terminal.contains("blocked-output") && terminal.contains("Preparing output folder"));
    assert(terminal.contains("accessible"));

    options.output = root / "batch";
    std::filesystem::create_directory(options.output);
    options.listCells = false;
    options.cellSelection = "MissingCell";
    const skyrim::ResolvedLoadOrder resolved;
    assert(app::detail::RunBatch(options, resolved, nullptr, {}, progress, {}) == 2);
    assert(terminal.contains("MissingCell") && terminal.contains("batch-report.json"));
    assert(terminal.contains("List cells"));

    // Catalog writers expose open failures so callers can avoid a false success message.
    assert(!cli::WriteCellsJson(root / "missing-parent" / "cells.json", {}));
    assert(!cli::WriteLoadOrderJson(root / "missing-parent" / "load-order.json", resolved));
    assert(cli::WriteCellsJson(root / "cells.json", {}));
    assert(cli::WriteLoadOrderJson(root / "load-order.json", resolved));
    std::filesystem::remove_all(root);
}
