#include "ui/windows_ui.h"

#include "app/run.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <commctrl.h>
#include <uxtheme.h>
#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <string>
#include <thread>
#include <memory>
#include <stdexcept>

namespace
{
    constexpr UINT ProgressMessage = WM_APP + 1;
    constexpr UINT CompleteMessage = WM_APP + 2;
    constexpr int RunButton = 900;
    constexpr int ListButton = 901;
    constexpr int CancelButton = 902;
    constexpr int FieldBase = 100;
    constexpr int CheckBase = 200;
    constexpr int TargetBase = 300;
    constexpr int PartitioningAlgorithmControl = 400;
    constexpr int RebuildScopeControl = 401;
    constexpr int PartitioningAlgorithmControlHeight = 100;

    struct Field
    {
        const char *key;
        const char *label;
        const char *hint;
    };
    constexpr std::array Fields{
        Field{"mo2", "Mod Organizer 2 folder", "An existing MO2 instance or portable root. Requires a profile."},
        Field{"profile", "MO2 profile", "Name of the existing MO2 profile to import."},
        Field{"mods", "Moved mods folder", "Optional read-only recovery location for moved MO2 mod folders."},
        Field{"plugin", "Plugin file", "Direct plugin input. Use with the legacy/direct reader."},
        Field{"data", "Game Data folder", "Data directory used with a direct load-order manifest."},
        Field{"load", "Load-order manifest", "Developer/direct input listing plugins in load order."},
        Field{"output", "Output folder", "Folder that receives reports, OBJ exports, diagnostics, and cached assets."},
        Field{"geometry", "Geometry OBJ path", "Optional custom path for the extracted support geometry OBJ."},
        Field{"analysis", "Analysis OBJ path", "Optional custom path for the navmesh/support comparison OBJ."},
        Field{"world", "Worldspace", "Optional worldspace name for direct plugin cell lookup."},
        Field{"cell", "Legacy cell selector",
              "Optional direct-reader cell selector; form ID, editor ID, or coordinates are clearer alternatives."},
        Field{"form", "Cell form ID", "Hexadecimal CELL form ID from List cells."},
        Field{"editor", "Cell editor ID", "Exact CELL editor ID from List cells."},
        Field{"x", "Exterior cell X", "Exterior CELL X coordinate. Select Coordinates to use it."},
        Field{"y", "Exterior cell Y", "Exterior CELL Y coordinate. Select Coordinates to use it."},
        Field{"radius", "Surface search radius",
              "Maximum horizontal search radius for support geometry, in game units."},
        Field{"support", "Max support distance", "Maximum vertical distance to a support surface, in game units."},
        Field{"slope", "Max slope", "Maximum support-surface slope in degrees."},
        Field{"neighboring_cell_radius", "Neighboring cells",
              "Exterior geometry and impact halo in cells. Generation always includes adjacent geometry and stays "
              "clipped to each target CELL."},
        Field{"affected_plugin", "Affected plugin",
              "Active ESP/ESM/ESL filename for Plugin scope. The full resolved load order supplies winning geometry "
              "and overrides."},
    };
    constexpr std::array Checks{
        Field{"list", "List cells only", "Discover and export cells without extracting geometry or analysis."},
        Field{"diagnostics", "Write diagnostics HTML", "Create an HTML report with representative support examples."},
        Field{"terrain", "Terrain only", "Skip reference-model geometry and export decoded exterior terrain only."},
        Field{"candidate", "Generate candidate NAVM",
              "Use Recast Navigation to rasterize terrain and supported collision, then export candidate JSON/OBJ and "
              "show it in the scene GLB."},
        Field{"generate_plugin", "Write plugin",
              "Write an ESP, ESL-flagged when eligible, after its source plugins. Matched door and border portals are "
              "written; other authored links need validation. Requires a resolved load order. Cell, Plugin, and Load "
              "order scopes are supported."},
    };

    struct WindowState
    {
        HWND window{}, tooltip{}, progress{}, status{}, percent{}, title{}, subtitle{}, lookupLabel{};
        HWND partitioningAlgorithmLabel{}, rebuildScopeLabel{};
        std::array<HWND, Fields.size()> fieldLabels{}, fieldHelps{};
        std::array<HWND, 4> sectionLabels{};
        std::array<HWND, Checks.size()> checkHelps{};
        HFONT heading{}, body{}, label{};
        std::filesystem::path config;
        std::shared_ptr<std::atomic_bool> cancelRequested;
    };
    struct RunCompletion
    {
        double elapsed{};
        std::string summary;
    };
    WindowState *State(HWND window)
    {
        return reinterpret_cast<WindowState *>(GetWindowLongPtrA(window, GWLP_USERDATA));
    }
    std::string Text(HWND window, int id)
    {
        char value[4096]{};
        GetWindowTextA(GetDlgItem(window, id), value, static_cast<int>(std::size(value)));
        return value;
    }
    std::string Trim(std::string value)
    {
        const auto whitespace = [](unsigned char character) { return std::isspace(character) != 0; };
        const auto first = std::find_if_not(value.begin(), value.end(), whitespace);
        if (first == value.end())
        {
            return {};
        }
        return {first, std::find_if_not(value.rbegin(), value.rend(), whitespace).base()};
    }
    void SetText(HWND window, int id, const std::string &value)
    {
        SetWindowTextA(GetDlgItem(window, id), value.c_str());
    }
    std::filesystem::path ConfigPath()
    {
        char appData[MAX_PATH]{};
        GetEnvironmentVariableA("LOCALAPPDATA", appData, MAX_PATH);
        auto directory = std::filesystem::path(appData) / "NavmeshGenerator";
        std::filesystem::create_directories(directory);
        return directory / "config.ini";
    }
    std::string ReadConfig(const std::filesystem::path &path, const char *key, const std::string &fallback = {})
    {
        char value[4096]{};
        GetPrivateProfileStringA("options", key, fallback.c_str(), value, static_cast<DWORD>(std::size(value)),
                                 path.string().c_str());
        return value;
    }
    void AddTooltip(WindowState &state, HWND control, const char *text)
    {
        TOOLINFOA item{.cbSize = sizeof(TOOLINFOA),
                       .uFlags = TTF_IDISHWND | TTF_SUBCLASS,
                       .hwnd = state.window,
                       .uId = reinterpret_cast<UINT_PTR>(control),
                       .lpszText = const_cast<char *>(text)};
        SendMessageA(state.tooltip, TTM_ADDTOOLA, 0, reinterpret_cast<LPARAM>(&item));
    }
    void Theme(HWND control)
    {
        SetWindowTheme(control, L"Explorer", nullptr);
    }
    void Font(HWND control, HFONT font)
    {
        SendMessageA(control, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
    }
    HWND Label(WindowState &state, const char *text, int x, int y, int width, HFONT font = nullptr)
    {
        auto label = CreateWindowA("STATIC", text, WS_CHILD | WS_VISIBLE, x, y, width, 18, state.window, nullptr,
                                   nullptr, nullptr);
        Font(label, font ? font : state.label);
        return label;
    }
    void AddSection(WindowState &state, int index, const char *title, int y)
    {
        state.sectionLabels[index] = Label(state, title, 30, y, 600, state.label);
    }
    void AddField(WindowState &state, int index, int x, int y)
    {
        const auto &field = Fields[index];
        state.fieldLabels[index] = Label(state, field.label, x, y, 250);
        auto edit = CreateWindowExA(WS_EX_CLIENTEDGE, "EDIT", "", WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL,
                                    x, y + 19, 254, 29, state.window,
                                    reinterpret_cast<HMENU>(static_cast<INT_PTR>(FieldBase + index)), nullptr, nullptr);
        auto help = CreateWindowA("BUTTON", "?", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW, x + 262, y + 21, 25,
                                  25, state.window, nullptr, nullptr, nullptr);
        state.fieldHelps[index] = help;
        Theme(edit);
        Font(edit, state.body);
        AddTooltip(state, help, field.hint);
        SendMessageA(edit, EM_SETCUEBANNER, 0, reinterpret_cast<LPARAM>(field.hint));
    }
    void Move(HWND control, int x, int y, int width, int height)
    {
        MoveWindow(control, x, y, width, height, TRUE);
    }
    void Layout(WindowState &state)
    {
        RECT client{};
        GetClientRect(state.window, &client);
        constexpr int margin = 30, gap = 18, helpWidth = 25;
        const int width = static_cast<int>(client.right), height = static_cast<int>(client.bottom);
        const int usableWidth = std::max(720, width - margin * 2);
        const int columnWidth = (usableWidth - gap * 2) / 3;
        const int columns[] = {margin, margin + columnWidth + gap, margin + (columnWidth + gap) * 2};
        Move(state.title, margin, 23, width - margin * 2, 30);
        Move(state.subtitle, margin + 1, 55, width - margin * 2, 22);
        const int sectionY[] = {107, 253, 342, 511};
        for (int index{}; index < 4; ++index)
        {
            Move(state.sectionLabels[index], margin, sectionY[index], width - margin * 2, 18);
        }
        for (int index{}; index < static_cast<int>(Fields.size()); ++index)
        {
            int y{};
            if (index < 6)
            {
                y = 130 + (index / 3) * 57;
            }
            else if (index < 9)
            {
                y = 276;
            }
            else if (index < 15)
            {
                y = 365 + ((index - 9) / 3) * 57;
            }
            else
            {
                y = 534 + ((index - 15) / 3) * 57;
            }
            const int x = columns[index == 19 ? 2 : index % 3];
            Move(state.fieldLabels[index], x, y, columnWidth - helpWidth - 8, 18);
            Move(GetDlgItem(state.window, FieldBase + index), x, y + 19, columnWidth - helpWidth - 8, 29);
            Move(state.fieldHelps[index], x + columnWidth - helpWidth, y + 21, helpWidth, helpWidth);
        }
        Move(state.lookupLabel, margin, 654, 120, 20);
        Move(state.partitioningAlgorithmLabel, columns[1], 591, columnWidth - helpWidth - 8, 18);
        Move(GetDlgItem(state.window, PartitioningAlgorithmControl), columns[1], 610, columnWidth - helpWidth - 8,
             PartitioningAlgorithmControlHeight);
        Move(state.rebuildScopeLabel, columns[2], 650, columnWidth - 8, 18);
        Move(GetDlgItem(state.window, RebuildScopeControl), columns[2], 669, columnWidth - 8, 120);
        const int selectionX = margin + 127;
        for (int index{}; index < 3; ++index)
        {
            Move(GetDlgItem(state.window, TargetBase + index), selectionX + index * 125, 652, 116, 24);
        }
        for (int index{}; index < static_cast<int>(Checks.size()); ++index)
        {
            const int x = columns[index % 3], y = index < 3 ? 715 : 749;
            Move(GetDlgItem(state.window, CheckBase + index), x, y, columnWidth - 30, 24);
            Move(state.checkHelps[index], x + columnWidth - 25, y, 22, 22);
        }
        const int footerTop = std::max(805, height - 145);
        Move(state.progress, margin, footerTop, std::max(300, width - margin * 2 - 75), 20);
        Move(state.percent, width - margin - 60, footerTop, 60, 20);
        Move(state.status, margin, footerTop + 29, width - margin * 2, 22);
        Move(GetDlgItem(state.window, ListButton), width - margin - 425, footerTop + 68, 130, 38);
        Move(GetDlgItem(state.window, CancelButton), width - margin - 275, footerTop + 68, 120, 38);
        Move(GetDlgItem(state.window, RunButton), width - margin - 145, footerTop + 68, 145, 38);
    }
    navmesh::app::Options ReadOptions(HWND window, bool listOnly)
    {
        navmesh::app::Options result;
        result.mo2 = Text(window, FieldBase);
        result.profile = Text(window, FieldBase + 1);
        result.modsDirectory = Text(window, FieldBase + 2);
        result.plugin = Text(window, FieldBase + 3);
        result.data = Text(window, FieldBase + 4);
        result.loadOrder = Text(window, FieldBase + 5);
        result.output = Text(window, FieldBase + 6);
        result.exportGeometry = Text(window, FieldBase + 7);
        result.exportAnalysis = Text(window, FieldBase + 8);
        result.worldspace = Trim(Text(window, FieldBase + 9));
        result.cell = Trim(Text(window, FieldBase + 10));
        const auto scope = static_cast<int>(SendMessageA(GetDlgItem(window, RebuildScopeControl), CB_GETCURSEL, 0, 0));
        result.rebuildScope = listOnly     ? navmesh::app::RebuildScope::Cell
                              : scope == 1 ? navmesh::app::RebuildScope::Plugin
                              : scope == 2 ? navmesh::app::RebuildScope::LoadOrder
                                           : navmesh::app::RebuildScope::Cell;
        result.affectedPlugin = Trim(Text(window, FieldBase + 19));
        if (result.rebuildScope != navmesh::app::RebuildScope::Cell)
        {
            result.cell.clear();
        }
        const bool lookupCell = !listOnly && result.rebuildScope == navmesh::app::RebuildScope::Cell;
        const auto target = static_cast<int>(SendMessageA(GetDlgItem(window, TargetBase), BM_GETCHECK, 0, 0));
        try
        {
            if (lookupCell && target == BST_CHECKED)
            {
                const auto formId = Trim(Text(window, FieldBase + 11));
                if (formId.empty())
                {
                    throw std::runtime_error("Enter a cell form ID or select another lookup method.");
                }
                result.cellFormId = static_cast<std::uint32_t>(std::stoul(formId, nullptr, 16));
            }
            else if (lookupCell && SendMessageA(GetDlgItem(window, TargetBase + 1), BM_GETCHECK, 0, 0) == BST_CHECKED)
            {
                result.editorId = Trim(Text(window, FieldBase + 12));
                if (result.editorId.empty())
                {
                    throw std::runtime_error("Enter a CELL editor ID or select another lookup method.");
                }
            }
            else if (lookupCell && SendMessageA(GetDlgItem(window, TargetBase + 2), BM_GETCHECK, 0, 0) == BST_CHECKED)
            {
                result.cellX = std::stoi(Text(window, FieldBase + 13));
                result.cellY = std::stoi(Text(window, FieldBase + 14));
            }
            result.surfaceSearchRadius = std::stof(Text(window, FieldBase + 15));
            result.maxSupportDistance = std::stof(Text(window, FieldBase + 16));
            result.maxSlope = std::stof(Text(window, FieldBase + 17));
            result.neighboringCellRadius = std::stoi(Text(window, FieldBase + 18));
            if (result.neighboringCellRadius < 0)
            {
                throw std::invalid_argument("negative neighboring-cell radius");
            }
        }
        catch (...)
        {
            throw std::runtime_error("Cell coordinates, analysis thresholds, and neighboring-cell radius must be valid "
                                     "numbers. Neighboring cells cannot be negative.");
        }
        result.listCells = listOnly || SendMessageA(GetDlgItem(window, CheckBase), BM_GETCHECK, 0, 0) == BST_CHECKED;
        result.diagnostics = SendMessageA(GetDlgItem(window, CheckBase + 1), BM_GETCHECK, 0, 0) == BST_CHECKED;
        result.terrainOnly = SendMessageA(GetDlgItem(window, CheckBase + 2), BM_GETCHECK, 0, 0) == BST_CHECKED;
        result.generateCandidate = SendMessageA(GetDlgItem(window, CheckBase + 3), BM_GETCHECK, 0, 0) == BST_CHECKED;
        result.generatePlugin =
            !listOnly && SendMessageA(GetDlgItem(window, CheckBase + 4), BM_GETCHECK, 0, 0) == BST_CHECKED;
        if (result.generatePlugin)
        {
            result.generateCandidate = true;
        }
        if (result.rebuildScope != navmesh::app::RebuildScope::Cell)
        {
            result.generateCandidate = true;
            if (result.rebuildScope == navmesh::app::RebuildScope::Plugin && result.affectedPlugin.empty())
            {
                throw std::runtime_error("Enter the active plugin filename for Plugin scope.");
            }
        }
        if (result.generatePlugin && result.mo2.empty() && result.loadOrder.empty())
        {
            throw std::runtime_error("Plugin generation needs an MO2 profile or developer load-order input.");
        }
        const auto algorithm =
            static_cast<int>(SendMessageA(GetDlgItem(window, PartitioningAlgorithmControl), CB_GETCURSEL, 0, 0));
        result.partitioningAlgorithm = algorithm == 1   ? navmesh::core::RegionPartitioningAlgorithm::Monotone
                                       : algorithm == 2 ? navmesh::core::RegionPartitioningAlgorithm::Layers
                                                        : navmesh::core::RegionPartitioningAlgorithm::Watershed;
        return result;
    }
    void Save(HWND window)
    {
        const auto path = State(window)->config.string();
        for (size_t i{}; i < Fields.size(); ++i)
        {
            WritePrivateProfileStringA("options", Fields[i].key, Text(window, FieldBase + static_cast<int>(i)).c_str(),
                                       path.c_str());
        }
        for (size_t i{}; i < Checks.size(); ++i)
        {
            WritePrivateProfileStringA(
                "options", Checks[i].key,
                SendMessageA(GetDlgItem(window, CheckBase + static_cast<int>(i)), BM_GETCHECK, 0, 0) == BST_CHECKED
                    ? "1"
                    : "0",
                path.c_str());
        }
        const auto algorithm =
            static_cast<int>(SendMessageA(GetDlgItem(window, PartitioningAlgorithmControl), CB_GETCURSEL, 0, 0));
        const char *algorithmName = algorithm == 1 ? "monotone" : algorithm == 2 ? "layers" : "watershed";
        WritePrivateProfileStringA("options", "partitioning_algorithm", algorithmName, path.c_str());
        const auto scope = SendMessageA(GetDlgItem(window, RebuildScopeControl), CB_GETCURSEL, 0, 0);
        WritePrivateProfileStringA("options", "rebuild_scope",
                                   scope == 1   ? "plugin"
                                   : scope == 2 ? "load_order"
                                                : "cell",
                                   path.c_str());
        for (int i{}; i != 3; ++i)
        {
            if (SendMessageA(GetDlgItem(window, TargetBase + i), BM_GETCHECK, 0, 0) == BST_CHECKED)
            {
                WritePrivateProfileStringA("options", "target", std::to_string(i).c_str(), path.c_str());
            }
        }
    }
    void Start(HWND window, bool listOnly)
    {
        try
        {
            const auto options = ReadOptions(window, listOnly);
            Save(window);
            EnableWindow(GetDlgItem(window, RunButton), FALSE);
            EnableWindow(GetDlgItem(window, ListButton), FALSE);
            EnableWindow(GetDlgItem(window, CancelButton), TRUE);
            SendMessageA(State(window)->progress, PBM_SETPOS, 0, 0);
            SetWindowTextA(State(window)->status, "Starting…");
            State(window)->cancelRequested = std::make_shared<std::atomic_bool>(false);
            const auto cancelRequested = State(window)->cancelRequested;
            std::thread(
                [window, options, cancelRequested]
                {
                    const auto started = std::chrono::steady_clock::now();
                    int result{};
                    std::string finalStatus;
                    try
                    {
                        result = navmesh::app::Run(
                            options,
                            [window, &finalStatus](int percent, std::string_view status)
                            {
                                finalStatus = status;
                                auto *message = new std::string(status);
                                PostMessageA(window, ProgressMessage, static_cast<WPARAM>(percent),
                                             reinterpret_cast<LPARAM>(message));
                            },
                            [cancelRequested] { return cancelRequested->load(); });
                    }
                    catch (const std::exception &error)
                    {
                        result = 255;
                        auto *message = new std::string("Failed: " + std::string(error.what()));
                        PostMessageA(window, ProgressMessage, 0, reinterpret_cast<LPARAM>(message));
                    }
                    const auto elapsed =
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
                    PostMessageA(window, CompleteMessage, static_cast<WPARAM>(result),
                                 reinterpret_cast<LPARAM>(new RunCompletion{elapsed, std::move(finalStatus)}));
                })
                .detach();
        }
        catch (const std::exception &error)
        {
            MessageBoxA(window, error.what(), "Invalid options", MB_ICONWARNING);
        }
    }
    LRESULT CALLBACK Procedure(HWND window, UINT message, WPARAM wParam, LPARAM lParam)
    {
        if (message == WM_NCCREATE)
        {
            SetWindowLongPtrA(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(new WindowState{}));
        }
        auto *state = State(window);
        switch (message)
        {
        case WM_CREATE:
        {
            state->window = window;
            state->config = ConfigPath();
            state->heading =
                CreateFontA(-25, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                            CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, "Segoe UI");
            state->body = CreateFontA(-15, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                                      CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, "Segoe UI");
            state->label =
                CreateFontA(-12, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
                            CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, "Segoe UI");
            state->tooltip = CreateWindowExA(WS_EX_TOPMOST, TOOLTIPS_CLASSA, nullptr, WS_POPUP | TTS_ALWAYSTIP, 0, 0, 0,
                                             0, window, nullptr, nullptr, nullptr);
            state->title = Label(*state, "NAVMESH GENERATOR", 30, 23, 500, state->heading);
            state->subtitle =
                Label(*state, "Offline Skyrim navmesh inspection and support analysis", 31, 55, 600, state->body);
            AddSection(*state, 0, "INPUT SOURCE", 107);
            const int columns[] = {30, 330, 630};
            for (int i{}; i < 6; ++i)
            {
                AddField(*state, i, columns[i % 3], 130 + (i / 3) * 57);
            }
            AddSection(*state, 1, "OUTPUT", 253);
            for (int i{}; i < 3; ++i)
            {
                AddField(*state, i + 6, columns[i], 276);
            }
            AddSection(*state, 2, "CELL", 342);
            for (int i{}; i < 6; ++i)
            {
                AddField(*state, i + 9, columns[i % 3], 365 + (i / 3) * 57);
            }
            AddSection(*state, 3, "ANALYSIS THRESHOLDS", 511);
            for (int i{}; i < 3; ++i)
            {
                AddField(*state, i + 15, columns[i], 534);
            }
            AddField(*state, 18, columns[0], 591);
            AddField(*state, 19, columns[2], 591);
            state->rebuildScopeLabel = Label(*state, "Rebuild scope", columns[2], 650, 250);
            auto scope =
                CreateWindowExA(WS_EX_CLIENTEDGE, "COMBOBOX", "", WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST,
                                columns[2], 669, 254, 120, window,
                                reinterpret_cast<HMENU>(static_cast<INT_PTR>(RebuildScopeControl)), nullptr, nullptr);
            Font(scope, state->body);
            Theme(scope);
            for (const char *text : {"Cell", "Plugin", "Load order"})
            {
                SendMessageA(scope, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(text));
            }
            AddTooltip(*state, scope,
                       "Plugin: cells possibly affected by that active plugin. Load order: changes after the first "
                       "baseline plugin. Both include adjacent impact cells.");
            const auto savedScope = ReadConfig(state->config, "rebuild_scope", "cell");
            SendMessageA(scope, CB_SETCURSEL, savedScope == "plugin" ? 1 : savedScope == "load_order" ? 2 : 0, 0);
            state->partitioningAlgorithmLabel = Label(*state, "Partitioning algorithm", columns[1], 591, 250);
            auto algorithm = CreateWindowExA(
                WS_EX_CLIENTEDGE, "COMBOBOX", "", WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST, columns[1],
                610, 254, PartitioningAlgorithmControlHeight, window,
                reinterpret_cast<HMENU>(static_cast<INT_PTR>(PartitioningAlgorithmControl)), nullptr, nullptr);
            Theme(algorithm);
            Font(algorithm, state->body);
            for (const char *value : {"Watershed", "Monotone", "Layers"})
            {
                SendMessageA(algorithm, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(value));
            }
            AddTooltip(*state, algorithm, "Recast region partitioning strategy used for candidate NAVM generation.");
            state->lookupLabel = CreateWindowA("STATIC", "LOOK UP CELL BY", WS_CHILD | WS_VISIBLE, 30, 654, 120, 20,
                                               window, nullptr, nullptr, nullptr);
            Font(state->lookupLabel, state->label);
            const char *targets[] = {"Form ID", "Editor ID", "Coordinates"};
            for (int i{}; i < 3; ++i)
            {
                auto button =
                    CreateWindowA("BUTTON", targets[i],
                                  WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTORADIOBUTTON | (i == 0 ? WS_GROUP : 0),
                                  157 + i * 125, 652, 116, 24, window,
                                  reinterpret_cast<HMENU>(static_cast<INT_PTR>(TargetBase + i)), nullptr, nullptr);
                Theme(button);
                Font(button, state->body);
                AddTooltip(*state, button,
                           i == 0   ? "Select exactly one cell-lookup method."
                           : i == 1 ? "Use this method to select a CELL by editor ID."
                                    : "Use this method to select an exterior CELL by X/Y coordinates.");
            }
            for (size_t i{}; i < Checks.size(); ++i)
            {
                auto check = CreateWindowA(
                    "BUTTON", Checks[i].label, WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
                    535 + static_cast<int>(i) * 125, 685, 104, 24, window,
                    reinterpret_cast<HMENU>(static_cast<INT_PTR>(CheckBase + static_cast<int>(i))), nullptr, nullptr);
                Theme(check);
                Font(check, state->body);
                auto help =
                    CreateWindowA("BUTTON", "?", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW,
                                  640 + static_cast<int>(i) * 125, 685, 22, 22, window, nullptr, nullptr, nullptr);
                state->checkHelps[i] = help;
                AddTooltip(*state, help, Checks[i].hint);
            }
            state->progress = CreateWindowExA(0, PROGRESS_CLASSA, nullptr, WS_CHILD | WS_VISIBLE, 30, 772, 770, 20,
                                              window, nullptr, nullptr, nullptr);
            Theme(state->progress);
            SendMessageA(state->progress, PBM_SETRANGE32, 0, 100);
            state->percent = Label(*state, "0%", 815, 772, 90, state->label);
            state->status = CreateWindowA("STATIC", "Ready to analyze", WS_CHILD | WS_VISIBLE, 30, 801, 860, 22, window,
                                          nullptr, nullptr, nullptr);
            Font(state->status, state->body);
            auto list =
                CreateWindowA("BUTTON", "List cells", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW, 480, 840, 130,
                              38, window, reinterpret_cast<HMENU>(static_cast<INT_PTR>(ListButton)), nullptr, nullptr);
            Font(list, state->body);
            auto cancel =
                CreateWindowA("BUTTON", "Cancel", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW, 620, 840, 120, 38,
                              window, reinterpret_cast<HMENU>(static_cast<INT_PTR>(CancelButton)), nullptr, nullptr);
            Font(cancel, state->body);
            EnableWindow(cancel, FALSE);
            auto run = CreateWindowA("BUTTON", "Run analysis", WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW, 760,
                                     840, 145, 38, window, reinterpret_cast<HMENU>(static_cast<INT_PTR>(RunButton)),
                                     nullptr, nullptr);
            Font(run, state->body);
            for (size_t i{}; i < Fields.size(); ++i)
            {
                SetText(window, FieldBase + static_cast<int>(i),
                        ReadConfig(state->config, Fields[i].key,
                                   i == 6    ? "."
                                   : i == 15 ? "64"
                                   : i == 16 ? "32"
                                   : i == 17 ? "45"
                                   : i == 18 ? "1"
                                             : ""));
            }
            for (size_t i{}; i < Checks.size(); ++i)
            {
                SendMessageA(GetDlgItem(window, CheckBase + static_cast<int>(i)), BM_SETCHECK,
                             ReadConfig(state->config, Checks[i].key) == "1" ? BST_CHECKED : BST_UNCHECKED, 0);
            }
            const auto savedAlgorithm = ReadConfig(state->config, "partitioning_algorithm", "watershed");
            const int algorithmIndex = savedAlgorithm == "monotone" ? 1 : savedAlgorithm == "layers" ? 2 : 0;
            SendMessageA(GetDlgItem(window, PartitioningAlgorithmControl), CB_SETCURSEL, algorithmIndex, 0);
            const auto target = std::clamp(std::stoi(ReadConfig(state->config, "target", "0")), 0, 2);
            SendMessageA(GetDlgItem(window, TargetBase + target), BM_SETCHECK, BST_CHECKED, 0);
            Layout(*state);
            return 0;
        }
        case WM_GETMINMAXINFO:
        {
            auto *info = reinterpret_cast<MINMAXINFO *>(lParam);
            info->ptMinTrackSize.x = 960;
            info->ptMinTrackSize.y = 1000;
            return 0;
        }
        case WM_SIZE:
            if (state && state->progress)
            {
                Layout(*state);
                InvalidateRect(window, nullptr, TRUE);
            }
            return 0;
        case WM_ERASEBKGND:
            return TRUE;
        case WM_PAINT:
        {
            PAINTSTRUCT paint{};
            auto device = BeginPaint(window, &paint);
            RECT client{};
            GetClientRect(window, &client);
            HBRUSH page = CreateSolidBrush(RGB(246, 248, 252));
            FillRect(device, &client, page);
            DeleteObject(page);
            RECT header{0, 0, client.right, 91};
            HBRUSH navy = CreateSolidBrush(RGB(15, 23, 42));
            FillRect(device, &header, navy);
            DeleteObject(navy);
            RECT accent{0, 88, client.right, 91};
            HBRUSH blue = CreateSolidBrush(RGB(59, 130, 246));
            FillRect(device, &accent, blue);
            DeleteObject(blue);
            EndPaint(window, &paint);
            return 0;
        }
        case WM_CTLCOLORSTATIC:
        {
            auto device = reinterpret_cast<HDC>(wParam);
            SetBkMode(device, TRANSPARENT);
            SetTextColor(device, reinterpret_cast<HWND>(lParam) == state->title      ? RGB(248, 250, 252)
                                 : reinterpret_cast<HWND>(lParam) == state->subtitle ? RGB(191, 219, 254)
                                                                                     : RGB(51, 65, 85));
            return reinterpret_cast<LRESULT>(GetStockObject(NULL_BRUSH));
        }
        case WM_CTLCOLOREDIT:
        {
            auto device = reinterpret_cast<HDC>(wParam);
            SetTextColor(device, RGB(30, 41, 59));
            SetBkColor(device, RGB(255, 255, 255));
            return reinterpret_cast<LRESULT>(GetStockObject(WHITE_BRUSH));
        }
        case WM_COMMAND:
            if (LOWORD(wParam) == RunButton)
            {
                Start(window, false);
                return 0;
            }
            else if (LOWORD(wParam) == ListButton)
            {
                Start(window, true);
                return 0;
            }
            else if (LOWORD(wParam) == CancelButton && state->cancelRequested)
            {
                state->cancelRequested->store(true);
                EnableWindow(GetDlgItem(window, CancelButton), FALSE);
                SetWindowTextA(state->status, "Cancelling after the current safe operation…");
                return 0;
            }
            break;
        case WM_DRAWITEM:
        {
            const auto *item = reinterpret_cast<DRAWITEMSTRUCT *>(lParam);
            if (item->CtlType != ODT_BUTTON)
            {
                break;
            }
            if (item->CtlID == RunButton || item->CtlID == ListButton || item->CtlID == CancelButton)
            {
                const auto primary = item->CtlID == RunButton;
                HBRUSH brush = CreateSolidBrush(primary ? RGB(37, 99, 235) : RGB(226, 232, 240));
                HPEN pen = CreatePen(PS_SOLID, 1, primary ? RGB(37, 99, 235) : RGB(203, 213, 225));
                auto oldBrush = SelectObject(item->hDC, brush);
                auto oldPen = SelectObject(item->hDC, pen);
                RoundRect(item->hDC, item->rcItem.left, item->rcItem.top, item->rcItem.right, item->rcItem.bottom, 10,
                          10);
                SelectObject(item->hDC, oldBrush);
                SelectObject(item->hDC, oldPen);
                DeleteObject(brush);
                DeleteObject(pen);
                RECT bounds = item->rcItem;
                SetBkMode(item->hDC, TRANSPARENT);
                SetTextColor(item->hDC, primary ? RGB(255, 255, 255) : RGB(30, 41, 59));
                const char *text = primary ? "Run analysis" : item->CtlID == CancelButton ? "Cancel" : "List cells";
                DrawTextA(item->hDC, text, -1, &bounds, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
                return TRUE;
            }
            HBRUSH brush = CreateSolidBrush(RGB(37, 99, 235));
            auto previousBrush = SelectObject(item->hDC, brush);
            Ellipse(item->hDC, item->rcItem.left + 2, item->rcItem.top + 2, item->rcItem.right - 2,
                    item->rcItem.bottom - 2);
            SelectObject(item->hDC, previousBrush);
            DeleteObject(brush);
            RECT bounds = item->rcItem;
            SetBkMode(item->hDC, TRANSPARENT);
            SetTextColor(item->hDC, RGB(255, 255, 255));
            DrawTextA(item->hDC, "?", 1, &bounds, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            return TRUE;
        }
        case ProgressMessage:
        {
            std::unique_ptr<std::string> status(reinterpret_cast<std::string *>(lParam));
            SendMessageA(state->progress, PBM_SETPOS, wParam, 0);
            SetWindowTextA(state->percent, (std::to_string(wParam) + "%").c_str());
            SetWindowTextA(state->status, status->c_str());
            RedrawWindow(window, nullptr, nullptr, RDW_INVALIDATE | RDW_ERASE | RDW_UPDATENOW | RDW_ALLCHILDREN);
            return 0;
        }
        case CompleteMessage:
        {
            std::unique_ptr<RunCompletion> completed(reinterpret_cast<RunCompletion *>(lParam));
            char elapsed[80]{};
            std::snprintf(elapsed, sizeof(elapsed), "%s in %.2f seconds.",
                          wParam == 0   ? "Completed"
                          : wParam == 3 ? "Cancelled"
                                        : "Stopped",
                          completed->elapsed);
            std::string status = elapsed;
            if ((wParam == 0 || completed->summary.starts_with("Plugin generation failed")) &&
                !completed->summary.empty())
            {
                status += " " + completed->summary;
            }
            SetWindowTextA(state->status, status.c_str());
            SendMessageA(state->progress, PBM_SETPOS,
                         wParam == 3 ? SendMessageA(state->progress, PBM_GETPOS, 0, 0) : 100, 0);
            SetWindowTextA(state->percent,
                           wParam == 3 ? (std::to_string(SendMessageA(state->progress, PBM_GETPOS, 0, 0)) + "%").c_str()
                                       : "100%");
            EnableWindow(GetDlgItem(window, RunButton), TRUE);
            EnableWindow(GetDlgItem(window, ListButton), TRUE);
            EnableWindow(GetDlgItem(window, CancelButton), FALSE);
            RedrawWindow(window, nullptr, nullptr, RDW_INVALIDATE | RDW_ERASE | RDW_UPDATENOW | RDW_ALLCHILDREN);
            return 0;
        }
        case WM_DESTROY:
            DeleteObject(state->heading);
            DeleteObject(state->body);
            DeleteObject(state->label);
            delete state;
            PostQuitMessage(0);
            return 0;
        }
        return DefWindowProcA(window, message, wParam, lParam);
    }
} // namespace

int navmesh::ui::RunWindowsUi(const navmesh::app::Options &)
{
    INITCOMMONCONTROLSEX controls{.dwSize = sizeof(controls), .dwICC = ICC_PROGRESS_CLASS | ICC_WIN95_CLASSES};
    InitCommonControlsEx(&controls);
    const WNDCLASSA klass{.lpfnWndProc = Procedure,
                          .hInstance = GetModuleHandleA(nullptr),
                          .hCursor = LoadCursor(nullptr, IDC_ARROW),
                          .hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1),
                          .lpszClassName = "NavmeshGeneratorWindow"};
    RegisterClassA(&klass);
    auto window = CreateWindowExA(0, klass.lpszClassName, "Navmesh Generator", WS_OVERLAPPEDWINDOW, CW_USEDEFAULT,
                                  CW_USEDEFAULT, 1040, 1020, nullptr, nullptr, klass.hInstance, nullptr);
    ShowWindow(window, SW_SHOW);
    UpdateWindow(window);
    MSG message;
    while (GetMessageA(&message, nullptr, 0, 0) > 0)
    {
        TranslateMessage(&message);
        DispatchMessageA(&message);
    }
    return static_cast<int>(message.wParam);
}
