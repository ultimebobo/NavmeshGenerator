#include "ui/windows_ui.h"
#include "app/run.h"
#include "ui/options_model.h"
#include "ui/windows_resources.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <d3d11.h>
#include <dwmapi.h>
#include <shobjidl.h>
#include <wrl/client.h>

#include <imgui.h>
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>
#include <misc/cpp/imgui_stdlib.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <format>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND, UINT, WPARAM, LPARAM);

namespace
{
    using Microsoft::WRL::ComPtr;
    using navmesh::app::Options;
    using navmesh::app::RebuildScope;

    std::wstring Wide(std::string_view text)
    {
        if (text.empty())
        {
            return {};
        }
        const auto count = MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0);
        std::wstring result(count, L'\0');
        MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), result.data(), count);
        return result;
    }

    std::string Utf8(std::wstring_view text)
    {
        if (text.empty())
        {
            return {};
        }
        const auto count =
            WideCharToMultiByte(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), nullptr, 0, nullptr, nullptr);
        std::string result(count, '\0');
        WideCharToMultiByte(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), result.data(), count, nullptr,
                            nullptr);
        return result;
    }

    std::string PathText(const std::filesystem::path &path)
    {
        return Utf8(path.wstring());
    }

    std::filesystem::path ConfigPath()
    {
        std::array<wchar_t, 32768> buffer{};
        const auto count = GetEnvironmentVariableW(L"LOCALAPPDATA", buffer.data(), static_cast<DWORD>(buffer.size()));
        if (!count || count >= buffer.size())
        {
            throw std::runtime_error("Cannot locate the local application settings folder.");
        }
        const auto directory = std::filesystem::path(buffer.data()) / "NavmeshGenerator";
        std::filesystem::create_directories(directory);
        return directory / "config.ini";
    }

    std::string ReadConfig(const std::filesystem::path &path, const char *key, const std::string &fallback)
    {
        std::array<wchar_t, 32768> buffer{};
        GetPrivateProfileStringW(L"options", Wide(key).c_str(), Wide(fallback).c_str(), buffer.data(),
                                 static_cast<DWORD>(buffer.size()), path.c_str());
        return Utf8(buffer.data());
    }

    void WriteConfig(const std::filesystem::path &path, const char *key, const std::string &value)
    {
        if (!WritePrivateProfileStringW(L"options", Wide(key).c_str(), Wide(value).c_str(), path.c_str()))
        {
            throw std::runtime_error("Could not save desktop settings.");
        }
    }

    struct NumericField
    {
        const char *key;
        const char *label;
        const char *help;
        float *value;
    };

    auto AnalysisFields(Options &options)
    {
        return std::array{
            NumericField{"radius", "Surface search radius", "Horizontal support search distance in Skyrim units.",
                         &options.surfaceSearchRadius},
            NumericField{"support", "Max support distance", "Vertical support distance in Skyrim units.",
                         &options.maxSupportDistance},
            NumericField{"slope", "Analysis max slope", "Support-surface slope limit in degrees.", &options.maxSlope}};
    }

    auto RecastFields(Options &options)
    {
        auto &profile = options.navigationProfile;
        auto &recast = options.recastSettings;
        return std::array{
            NumericField{"agent_radius", "Agent radius", "Horizontal agent radius in Skyrim units.",
                         &profile.agentRadius},
            NumericField{"agent_height", "Agent height", "Standing agent height in Skyrim units.",
                         &profile.agentHeight},
            NumericField{"agent_clearance", "Agent clearance",
                         "Required headroom in Skyrim units; combined with height.", &profile.clearance},
            NumericField{"agent_step_height", "Step height",
                         "Maximum traversable climb in Skyrim units, rounded down to whole vertical voxels.",
                         &profile.stepHeight},
            NumericField{"agent_max_slope", "Walkable slope", "Maximum walking slope in degrees, below a right angle.",
                         &profile.maxSlopeDegrees},
            NumericField{"minimum_region_area", "Minimum region area",
                         "Disconnected island cutoff in square Skyrim units.", &profile.minimumRegionArea},
            NumericField{"weld_tolerance", "Weld tolerance",
                         "Positive distance tolerance for output geometry and border matching, in Skyrim units.",
                         &profile.weldTolerance},
            NumericField{"recast_cell_size", "Horizontal voxel size",
                         "Requested voxel width in Skyrim units. Coarse voxels can lose narrow stair treads. "
                         "Large scenes increase it to bound grid dimensions.",
                         &recast.cellSize},
            NumericField{"recast_cell_height", "Vertical voxel size",
                         "Positive voxel height in Skyrim units. Finer voxels preserve small steps and headroom; "
                         "step height is rounded down to whole voxels.",
                         &recast.cellHeight},
            NumericField{"recast_simplification_error", "Contour simplification error",
                         "Maximum contour deviation in horizontal voxels. Larger values reduce triangles; "
                         "smaller values follow obstacle outlines more closely. Collapsed regions trigger refinement.",
                         &recast.maxSimplificationError},
            NumericField{"recast_max_edge_length", "Maximum contour edge length",
                         "Edge subdivision limit in Skyrim units. Zero disables subdivision.", &recast.maxEdgeLength},
            NumericField{"recast_merge_area_multiplier", "Region merge multiplier",
                         "Merge area as a multiple of minimum region area. Used by watershed and monotone.",
                         &recast.mergeRegionAreaMultiplier}};
    }

    auto BooleanFields(Options &options)
    {
        return std::array{std::pair{"candidate", &options.generateCandidate},
                          std::pair{"make_scene", &options.makeScene},
                          std::pair{"generate_plugin", &options.generatePlugin},
                          std::pair{"skip_existing_navmesh", &options.skipExistingNavmesh},
                          std::pair{"triangle_tagging", &options.tagTriangles},
                          std::pair{"copy_plugin", &options.copyPlugin},
                          std::pair{"estimate_only", &options.estimateOnly}};
    }

    auto IntegerFields(Options &options)
    {
        return std::array{std::pair{"cache_budget_mib", &options.cacheBudgetMiB},
                          std::pair{"working_memory_mib", &options.workingMemoryMiB},
                          std::pair{"workers", &options.workers}};
    }

    /// Worker evidence is published under a mutex; the UI thread alone owns controls and rendering.
    struct RunStatus
    {
        std::mutex mutex;
        std::atomic_bool cancel{};
        int percent{}, code{};
        std::string text{"Ready. Select an MO2 profile and a rebuild target."};
        bool complete{};
    };

    struct Workspace
    {
        Options draft;
        std::map<std::string, std::string> text;
        std::filesystem::path config;
        bool busy{};
        RunStatus status;
        std::thread worker;

        ~Workspace()
        {
            status.cancel = true;
            if (worker.joinable())
            {
                worker.join();
            }
        }
    };

    void LoadSettings(Workspace &workspace)
    {
        auto &options = workspace.draft;
        workspace.config = ConfigPath();
        workspace.text = {{"mo2", PathText(options.mo2)},
                          {"profile", options.profile},
                          {"mods", PathText(options.modsDirectory)},
                          {"output", PathText(options.output)},
                          {"asset_cache", PathText(options.assetCache)},
                          {"form", options.cellFormId ? std::format("{:08X}", *options.cellFormId) : ""},
                          {"editor", options.editorId},
                          {"affected_plugin", options.affectedPlugin},
                          {"copy_plugin_source", options.copySourcePlugin}};
        for (auto &[key, value] : workspace.text)
        {
            value = ReadConfig(workspace.config, key.c_str(), value);
        }
        const auto savedIdentification = ReadConfig(workspace.config, "target", options.editorId.empty() ? "0" : "1");
        const auto cellFallback = options.cellSelection.empty()
                                      ? workspace.text.at(savedIdentification == "1" ? "editor" : "form")
                                      : options.cellSelection;
        workspace.text["selected_cells"] = ReadConfig(workspace.config, "selected_cells", cellFallback);
        std::replace(workspace.text.at("selected_cells").begin(), workspace.text.at("selected_cells").end(), ';', '\n');
        for (const auto &field : BooleanFields(options))
        {
            const auto value = ReadConfig(workspace.config, field.first, *field.second ? "1" : "0");
            if (value == "0" || value == "1")
            {
                *field.second = value == "1";
            }
        }
        const auto readNumbers = [&](const auto &fields)
        {
            for (const auto &field : fields)
            {
                try
                {
                    std::size_t end{};
                    const auto text = ReadConfig(workspace.config, field.key, std::format("{:.9g}", *field.value));
                    const auto number = std::stof(text, &end);
                    if (end == text.size() && std::isfinite(number))
                    {
                        *field.value = number;
                    }
                }
                catch (const std::exception &)
                {
                    // Malformed persisted numbers retain the shared option defaults.
                }
            }
        };
        readNumbers(AnalysisFields(options));
        readNumbers(RecastFields(options));
        for (const auto &field : IntegerFields(options))
        {
            try
            {
                std::size_t end{};
                const auto text = ReadConfig(workspace.config, field.first, std::to_string(*field.second));
                const auto number = std::stoull(text, &end);
                if (!text.empty() && text.front() != '-' && end == text.size())
                {
                    *field.second = number;
                }
            }
            catch (const std::exception &)
            {
            }
        }
        try
        {
            const auto text =
                ReadConfig(workspace.config, "neighboring_cell_radius", std::to_string(options.neighboringCellRadius));
            std::size_t end{};
            const auto radius = std::stoi(text, &end);
            if (end == text.size())
            {
                options.neighboringCellRadius = radius;
            }
        }
        catch (const std::exception &)
        {
        }
        const char *scopes[] = {"cell", "plugin", "load_order"};
        const auto scope =
            ReadConfig(workspace.config, "rebuild_scope", scopes[static_cast<int>(options.rebuildScope)]);
        if (scope == "selected_cells")
        {
            workspace.text.at("copy_plugin_source") =
                ReadConfig(workspace.config, "copy_plugin_source", workspace.text.at("affected_plugin"));
        }
        options.rebuildScope = scope == "plugin"       ? RebuildScope::Plugin
                               : scope == "load_order" ? RebuildScope::LoadOrder
                                                       : RebuildScope::Cell;
        const char *algorithms[] = {"watershed", "monotone", "layers"};
        const auto algorithm = ReadConfig(workspace.config, "partitioning_algorithm",
                                          algorithms[static_cast<int>(options.partitioningAlgorithm)]);
        options.partitioningAlgorithm = algorithm == "monotone" ? navmesh::core::RegionPartitioningAlgorithm::Monotone
                                        : algorithm == "layers" ? navmesh::core::RegionPartitioningAlgorithm::Layers
                                                                : navmesh::core::RegionPartitioningAlgorithm::Watershed;
        options.batchOutput = ReadConfig(workspace.config, "batch_output", options.batchOutput);
        if (options.batchOutput != "auto" && options.batchOutput != "full" && options.batchOutput != "compact" &&
            options.batchOutput != "plugin_only")
        {
            options.batchOutput = "auto";
        }
    }

    void SaveSettings(Workspace &workspace)
    {
        for (const auto &[key, value] : workspace.text)
        {
            if (key == "selected_cells")
            {
                // INI values occupy one line; delimiter encoding preserves the cell list across sessions.
                std::string stored;
                for (const auto &identifier : navmesh::app::ParseCellSelection(value))
                {
                    if (!stored.empty())
                    {
                        stored += ';';
                    }
                    stored += identifier;
                }
                WriteConfig(workspace.config, key.c_str(), stored);
            }
            else
            {
                WriteConfig(workspace.config, key.c_str(), value);
            }
        }
        for (const auto &field : BooleanFields(workspace.draft))
        {
            WriteConfig(workspace.config, field.first, *field.second ? "1" : "0");
        }
        for (const auto &field : IntegerFields(workspace.draft))
        {
            WriteConfig(workspace.config, field.first, std::to_string(*field.second));
        }
        for (const auto &field : AnalysisFields(workspace.draft))
        {
            WriteConfig(workspace.config, field.key, std::format("{:.9g}", *field.value));
        }
        for (const auto &field : RecastFields(workspace.draft))
        {
            WriteConfig(workspace.config, field.key, std::format("{:.9g}", *field.value));
        }
        const char *scopes[] = {"cell", "plugin", "load_order"};
        const char *algorithms[] = {"watershed", "monotone", "layers"};
        WriteConfig(workspace.config, "rebuild_scope", scopes[static_cast<int>(workspace.draft.rebuildScope)]);
        WriteConfig(workspace.config, "partitioning_algorithm",
                    algorithms[static_cast<int>(workspace.draft.partitioningAlgorithm)]);
        WriteConfig(workspace.config, "batch_output", workspace.draft.batchOutput);
        WriteConfig(workspace.config, "neighboring_cell_radius", std::to_string(workspace.draft.neighboringCellRadius));
    }

    Options ReadOptions(Workspace &workspace, bool listOnly)
    {
        auto options = workspace.draft;
        const auto value = [&](const char *key)
        {
            const auto &text = workspace.text.at(key);
            const auto first = text.find_first_not_of(" \t\r\n");
            return first == std::string::npos ? std::string{}
                                              : text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
        };
        options.mo2 = Wide(value("mo2"));
        options.profile = value("profile");
        options.modsDirectory = Wide(value("mods"));
        options.output = Wide(value("output"));
        options.assetCache = Wide(value("asset_cache"));
        options.affectedPlugin = value("affected_plugin");
        options.cellSelection = value("selected_cells");
        options.copySourcePlugin = value("copy_plugin_source");
        options.cellFormId.reset();
        options.editorId.clear();
        options = navmesh::ui::PrepareDesktopOptions(options, listOnly);
        return options;
    }

    /// Format a nonnegative steady-clock interval with nonzero hours/minutes and always include fractional seconds.
    std::string FormatElapsedTime(std::chrono::steady_clock::duration elapsed)
    {
        // Round before splitting units so fractional seconds carry into minutes and hours at their boundaries.
        const std::chrono::hh_mm_ss time(std::chrono::round<std::chrono::duration<long long, std::centi>>(elapsed));
        std::string result;
        if (time.hours().count() != 0)
        {
            result += std::format("{}h ", time.hours().count());
        }
        if (time.minutes().count() != 0)
        {
            result += std::format("{}m ", time.minutes().count());
        }
        result += std::format("{}.{:02}s", time.seconds().count(), time.subseconds().count());
        return result;
    }

    void Execute(Workspace &workspace, const Options &options, bool listOnly)
    {
        const auto started = std::chrono::steady_clock::now();
        int code{};
        std::string summary;
        try
        {
            code = navmesh::app::Run(
                options,
                [&](int percent, std::string_view text)
                {
                    summary = text;
                    std::lock_guard lock(workspace.status.mutex);
                    workspace.status.percent = std::clamp(percent, 0, 100);
                    workspace.status.text = text;
                },
                [&] { return workspace.status.cancel.load(); });
        }
        catch (const std::exception &error)
        {
            code = 1;
            summary = error.what();
        }
        const auto elapsed = FormatElapsedTime(std::chrono::steady_clock::now() - started);
        std::lock_guard lock(workspace.status.mutex);
        workspace.status.code = code;
        if (code == 0)
        {
            workspace.status.percent = 100;
            workspace.status.text =
                std::format("Completed in {}. {}", elapsed,
                            listOnly ? "Cell catalog saved to " + PathText(options.output / "cells.json") : summary);
        }
        else
        {
            workspace.status.text = std::format("{} in {}. {}", code == 3 ? "Cancelled" : "Stopped", elapsed, summary);
        }
        workspace.status.complete = true;
    }

    void Start(Workspace &workspace, bool listOnly)
    {
        try
        {
            const auto options = ReadOptions(workspace, listOnly);
            SaveSettings(workspace);
            if (workspace.worker.joinable())
            {
                workspace.worker.join();
            }
            workspace.status.cancel = false;
            {
                std::lock_guard lock(workspace.status.mutex);
                workspace.status.percent = 0;
                workspace.status.code = 0;
                workspace.status.text = listOnly ? "Exporting cell catalog..." : "Starting...";
                workspace.status.complete = false;
            }
            workspace.worker = std::thread([&workspace, options, listOnly] { Execute(workspace, options, listOnly); });
            workspace.busy = true;
        }
        catch (const std::exception &error)
        {
            std::lock_guard lock(workspace.status.mutex);
            workspace.status.text = error.what();
            workspace.status.code = 1;
        }
    }

    void Help(const char *text)
    {
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort))
        {
            ImGui::SetTooltip("%s", text);
        }
    }

    void BrowseFolder(HWND window, std::string &value)
    {
        ComPtr<IFileOpenDialog> dialog;
        if (FAILED(CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&dialog))))
        {
            return;
        }
        dialog->SetOptions(FOS_PICKFOLDERS | FOS_FORCEFILESYSTEM | FOS_NOCHANGEDIR);
        if (FAILED(dialog->Show(window)))
        {
            return;
        }
        ComPtr<IShellItem> folder;
        PWSTR path{};
        if (SUCCEEDED(dialog->GetResult(&folder)) && SUCCEEDED(folder->GetDisplayName(SIGDN_FILESYSPATH, &path)))
        {
            value = Utf8(path);
            CoTaskMemFree(path);
        }
    }

    void TextField(Workspace &workspace, HWND window, const char *key, const char *label, const char *hint,
                   bool folder = false)
    {
        ImGui::PushID(key);
        ImGui::TextUnformatted(label);
        ImGui::SetNextItemWidth(folder ? ImGui::GetContentRegionAvail().x - 92 * ImGui::GetStyle().FontScaleDpi : -1);
        ImGui::InputTextWithHint("##value", hint, &workspace.text.at(key));
        Help(hint);
        if (folder)
        {
            ImGui::SameLine();
            if (ImGui::Button("Browse", ImVec2(-1, 0)))
            {
                BrowseFolder(window, workspace.text.at(key));
            }
        }
        ImGui::PopID();
    }

    /// Retain only the plugin filename; the shared runner verifies membership in the active profile.
    void BrowsePlugin(HWND window, std::string &value)
    {
        ComPtr<IFileOpenDialog> dialog;
        if (FAILED(CoCreateInstance(CLSID_FileOpenDialog, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&dialog))))
        {
            return;
        }
        const COMDLG_FILTERSPEC filters[] = {{L"Skyrim plugins", L"*.esp;*.esm;*.esl"}};
        dialog->SetFileTypes(1, filters);
        dialog->SetOptions(FOS_FORCEFILESYSTEM | FOS_FILEMUSTEXIST | FOS_NOCHANGEDIR);
        if (FAILED(dialog->Show(window)))
        {
            return;
        }
        ComPtr<IShellItem> file;
        PWSTR path{};
        if (SUCCEEDED(dialog->GetResult(&file)) && SUCCEEDED(file->GetDisplayName(SIGDN_FILESYSPATH, &path)))
        {
            value = PathText(std::filesystem::path(path).filename());
            CoTaskMemFree(path);
        }
    }

    void PluginField(Workspace &workspace, HWND window, const char *key, const char *label)
    {
        ImGui::PushID(key);
        ImGui::TextUnformatted(label);
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 92 * ImGui::GetStyle().FontScaleDpi);
        ImGui::InputTextWithHint("##value", "Active ESP, ESM or ESL filename", &workspace.text.at(key));
        ImGui::SameLine();
        if (ImGui::Button("Browse", ImVec2(-1, 0)))
        {
            BrowsePlugin(window, workspace.text.at(key));
        }
        ImGui::PopID();
    }

    /// Auto-sized cards flow with contextual rows, so hidden controls leave no fixed-position gaps.
    void BeginCard(const char *id, const char *title, const char *description = nullptr)
    {
        ImGui::BeginChild(id, ImVec2(0, 0), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::SeparatorText(title);
        if (description)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
            ImGui::TextWrapped("%s", description);
            ImGui::PopStyleColor();
        }
        ImGui::Spacing();
    }

    void EndCard()
    {
        ImGui::EndChild();
        ImGui::Spacing();
    }

    template <std::size_t Size> void DrawNumbers(const std::array<NumericField, Size> &fields, bool layers = false)
    {
        if (ImGui::BeginTable("numbers", 2, ImGuiTableFlags_SizingStretchSame))
        {
            for (const auto &field : fields)
            {
                if (layers && std::string_view(field.key) == "recast_merge_area_multiplier")
                {
                    continue;
                }
                ImGui::TableNextColumn();
                ImGui::PushID(field.key);
                ImGui::TextUnformatted(field.label);
                ImGui::SetNextItemWidth(-1);
                ImGui::InputFloat("##value", field.value, 0, 0, "%.6g");
                Help(field.help);
                ImGui::PopID();
            }
            ImGui::EndTable();
        }
    }

    void DrawAdvanced(Workspace &workspace, HWND window, bool cellScope, bool generation)
    {
        auto &options = workspace.draft;
        if (!ImGui::CollapsingHeader("Advanced settings"))
        {
            return;
        }
        BeginCard("advanced", "PERFORMANCE & GEOMETRY", "Distances use Skyrim units; areas use square Skyrim units.");
        if (ImGui::Button("Reset"))
        {
            navmesh::ui::ResetAdvancedNumericalOptions(options);
            SaveSettings(workspace);
        }
        Help("Restore all advanced numerical values to the application defaults, including hidden controls.");
        TextField(workspace, window, "mods", "Moved mods folder (optional)",
                  "Recovery location for moved MO2 mod folders", true);
        TextField(workspace, window, "asset_cache", "Shared cache folder (optional)",
                  "Leave empty to use temporary storage", true);
        ImGui::TextUnformatted("Neighboring cell radius");
        ImGui::SetNextItemWidth(-1);
        ImGui::InputInt("##neighboring", &options.neighboringCellRadius);
        Help("Geometry input neighborhood in CELL units. Batch targets follow changed terrain or collision; generation "
             "remains clipped to each target cell.");
        const auto integers = IntegerFields(options);
        const char *labels[] = {"Cache disk budget (MiB)", "Working memory budget (MiB)", "Generation workers"};
        const char *hints[] = {"Generated cache retention only. Zero retains none after safe consumption.",
                               "Model cache and generation admission budget; an oversized target runs alone.",
                               "Maximum independent batch generation tasks; final serialization stays ordered."};
        for (std::size_t index = 0; index < integers.size(); ++index)
        {
            if (cellScope && index == 2)
            {
                continue;
            }
            ImGui::TextUnformatted(labels[index]);
            ImGui::SetNextItemWidth(-1);
            const auto id = "##" + std::string(integers[index].first);
            ImGui::InputScalar(id.c_str(), ImGuiDataType_U64, integers[index].second);
            Help(hints[index]);
        }
        if (cellScope)
        {
            ImGui::SeparatorText("Analysis");
            ImGui::PushID("analysis");
            DrawNumbers(AnalysisFields(options));
            ImGui::PopID();
        }
        if (generation)
        {
            ImGui::SeparatorText("Recast generation");
            ImGui::Checkbox("Tag water and preferred path triangles", &options.tagTriangles);
            Help("Classify submerged floors using exterior water levels and preserve nearby authored preferred "
                 "routes on the same floor. Without a supported water plane, authored water tags provide evidence.");
            int algorithm = static_cast<int>(options.partitioningAlgorithm);
            ImGui::TextUnformatted("Region partitioning");
            ImGui::SetNextItemWidth(-1);
            if (ImGui::Combo("##partitioning", &algorithm, "Watershed\0Monotone\0Layers\0"))
            {
                options.partitioningAlgorithm = static_cast<navmesh::core::RegionPartitioningAlgorithm>(algorithm);
            }
            ImGui::PushID("recast");
            DrawNumbers(RecastFields(options), algorithm == 2);
            ImGui::PopID();
        }
        EndCard();
    }

    void DrawTarget(Workspace &workspace, HWND window)
    {
        auto &options = workspace.draft;
        BeginCard("target", "REBUILD TARGET");
        int scope = static_cast<int>(options.rebuildScope);
        ImGui::TextUnformatted("Rebuild scope");
        ImGui::SetNextItemWidth(-1);
        if (ImGui::Combo("##scope", &scope, "Cell\0Plugin\0Load order\0"))
        {
            options.rebuildScope = static_cast<RebuildScope>(scope);
        }
        if (options.rebuildScope == RebuildScope::Plugin || options.rebuildScope == RebuildScope::LoadOrder)
        {
            Help("Generate navigation for cells affected by terrain, water or collision, including new worldspaces "
                 "and cells without navmesh. Water-only changes require supported terrain or model collision. "
                 "Enable Skip cells with existing navmesh to preserve authored cells.");
        }
        if (options.rebuildScope == RebuildScope::Cell)
        {
            ImGui::TextUnformatted("Cells");
            ImGui::TextWrapped(
                "Paste Form IDs or editor IDs from cells.json, one per line. Commas and semicolons also work.");
            ImGui::InputTextMultiline("##selected_cells", &workspace.text.at("selected_cells"),
                                      ImVec2(-1, 130 * ImGui::GetStyle().FontScaleDpi));
            const auto identifiers = navmesh::app::ParseCellSelection(workspace.text.at("selected_cells"));
            ImGui::TextDisabled("%zu cell identifiers", identifiers.size());
            ImGui::SameLine();
            if (ImGui::SmallButton("Clear list"))
            {
                workspace.text.at("selected_cells").clear();
            }
            ImGui::TextWrapped(
                "Only listed cells are processed. Duplicate identifiers are combined; unknown or ambiguous "
                "identifiers stop the run. Use List cells to file to find identifiers.");
            options.cellSelection = workspace.text.at("selected_cells");
        }
        else if (options.rebuildScope == RebuildScope::Plugin)
        {
            PluginField(workspace, window, "affected_plugin", "Affected plugin");
        }
        else
        {
            ImGui::TextWrapped(
                "Rebuild cells affected by changes after the baseline plugin and MO2 model replacements.");
        }
        EndCard();
    }

    void DrawOutput(Workspace &workspace, HWND window)
    {
        auto &options = workspace.draft;
        const bool cellScope = options.rebuildScope == RebuildScope::Cell;
        const bool copyAllowed = cellScope || options.rebuildScope == RebuildScope::Plugin;
        const bool batchGeneration = navmesh::app::UsesBatchGeneration(options);
        BeginCard("output", "OUTPUT", "Reports and exports are saved under this folder.");
        TextField(workspace, window, "output", "Output folder", "Folder for this run's results", true);
        if (cellScope)
        {
            ImGui::Checkbox("Make scene", &options.makeScene);
            Help("Write scene.glb for all selected cells, with terrain, models and NAVMs. Generated NAVMs are "
                 "included when enabled. Cells keep their native coordinates; separate interiors or worldspaces "
                 "can overlap.");
            bool candidate = options.generateCandidate || options.generatePlugin || options.copyPlugin;
            ImGui::BeginDisabled(options.generatePlugin || options.copyPlugin);
            if (ImGui::Checkbox("Generate candidate NAVM", &candidate))
            {
                options.generateCandidate = candidate;
            }
            ImGui::EndDisabled();
            Help("Generate navigation using supported terrain and collision with the advanced Recast settings.");
        }
        bool writing = options.generatePlugin || (copyAllowed && options.copyPlugin);
        ImGui::BeginDisabled(copyAllowed && options.copyPlugin);
        if (ImGui::Checkbox("Write plugin", &writing))
        {
            options.generatePlugin = writing;
        }
        ImGui::EndDisabled();
        Help("Write a verified NAVM patch; light format is selected automatically when eligible.");
        if (copyAllowed)
        {
            ImGui::Checkbox("Copy selected plugin", &options.copyPlugin);
            Help("Enables plugin writing. Preserve other records and the filename; install this copy in place of the "
                 "source.");
            if (cellScope && options.copyPlugin)
            {
                PluginField(workspace, window, "copy_plugin_source", "Plugin to copy");
            }
        }
        if (!cellScope || options.generateCandidate || options.generatePlugin || options.copyPlugin)
        {
            ImGui::Checkbox("Skip cells with existing navmesh", &options.skipExistingNavmesh);
            Help("Generate only for uncovered cells. Any winning NAVM record protects its cell.");
        }
        if (batchGeneration)
        {
            ImGui::BeginDisabled(cellScope && options.makeScene);
            ImGui::Checkbox("Estimate batch cost", &options.estimateOnly);
            ImGui::EndDisabled();
            Help("Sample eligible targets, cache the work and write a cost report before a full rebuild.");
            const bool minimalAllowed =
                options.generatePlugin || (copyAllowed && options.copyPlugin) || options.estimateOnly;
            const char *policies[] = {"auto", "full", "compact", "plugin_only"};
            const char *labels[] = {"Automatic", "Full inspection (JSON / OBJ)", "Compressed inspection (gzip JSON)",
                                    "Plugin and reports only"};
            int policy{};
            for (int index = 0; index < 4; ++index)
            {
                if (options.batchOutput == policies[index])
                {
                    policy = index;
                }
            }
            if (!minimalAllowed && policy == 3)
            {
                options.batchOutput = "auto";
                policy = 0;
            }
            ImGui::TextUnformatted("Batch output");
            ImGui::SetNextItemWidth(-1);
            if (ImGui::BeginCombo("##batch", labels[policy]))
            {
                for (int index = 0; index < (minimalAllowed ? 4 : 3); ++index)
                {
                    if (ImGui::Selectable(labels[index], policy == index))
                    {
                        options.batchOutput = policies[index];
                    }
                }
                ImGui::EndCombo();
            }
        }
        EndCard();
    }

    void DrawActions(Workspace &workspace)
    {
        const float scale = ImGui::GetStyle().FontScaleDpi;
        int percent{}, code{};
        std::string status;
        {
            std::lock_guard lock(workspace.status.mutex);
            percent = workspace.status.percent;
            code = workspace.status.code;
            status = workspace.status.text;
            if (workspace.status.complete)
            {
                workspace.busy = false;
            }
        }
        ImGui::Separator();
        ImGui::ProgressBar(percent / 100.0F, ImVec2(-1, 7 * scale), "");
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0, 0, 0, 0));
        ImGui::BeginChild("status", ImVec2(0, 58 * scale));
        ImGui::PopStyleColor();
        const bool error = code != 0 && !workspace.busy;
        if (error)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 0.58F, 0.5F, 1));
        }
        ImGui::TextWrapped("%s", status.c_str());
        if (error)
        {
            ImGui::PopStyleColor();
        }
        ImGui::EndChild();
        ImGui::BeginDisabled(workspace.busy);
        if (ImGui::Button("List cells to file", ImVec2(172 * scale, 36 * scale)))
        {
            Start(workspace, true);
        }
        Help("Write cells.json in the output folder. Cell selection and generation settings are ignored.");
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!workspace.busy || workspace.status.cancel.load());
        if (ImGui::Button("Cancel", ImVec2(90 * scale, 36 * scale)))
        {
            workspace.status.cancel = true;
            std::lock_guard lock(workspace.status.mutex);
            workspace.status.text = "Cancelling after the current safe operation...";
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(workspace.busy);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.16F, 0.41F, 0.85F, 1));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.23F, 0.51F, 0.96F, 1));
        const char *action = workspace.draft.rebuildScope != RebuildScope::Cell && workspace.draft.estimateOnly
                                 ? "Estimate batch"
                                 : "Run";
        if (ImGui::Button(action, ImVec2(-1, 36 * scale)))
        {
            Start(workspace, false);
        }
        ImGui::PopStyleColor(2);
        ImGui::EndDisabled();
    }

    void DrawWorkspace(Workspace &workspace, HWND window)
    {
        const float scale = ImGui::GetStyle().FontScaleDpi;
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
        ImGui::Begin("Workspace", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);
        ImGui::TextColored(ImVec4(0.38F, 0.67F, 1, 1), "SKYRIM  /  NAVMESH WORKSPACE");
        ImGui::PushFont(nullptr, 28);
        ImGui::TextUnformatted("Navmesh Generator");
        ImGui::PopFont();
        ImGui::TextDisabled("Inspect cells and rebuild navigation from your MO2 profile.");
        ImGui::Spacing();
        // Settings scroll independently so progress, cancellation and run actions stay visible.
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0, 0, 0, 0));
        ImGui::BeginChild("settings", ImVec2(0, -158 * scale));
        ImGui::PopStyleColor();
        ImGui::BeginDisabled(workspace.busy);
        const int columns = ImGui::GetContentRegionAvail().x >= 760 * scale ? 2 : 1;
        if (ImGui::BeginTable("setup", columns, ImGuiTableFlags_SizingStretchSame))
        {
            ImGui::TableNextColumn();
            BeginCard("source", "INPUT PROFILE", "Use an existing MO2 instance and profile.");
            TextField(workspace, window, "mo2", "Mod Organizer 2 folder", "Instance folder or portable MO2 root", true);
            TextField(workspace, window, "profile", "MO2 profile", "Existing profile name");
            EndCard();
            ImGui::TableNextColumn();
            DrawTarget(workspace, window);
            ImGui::EndTable();
        }
        DrawOutput(workspace, window);
        const auto &options = workspace.draft;
        const bool cellScope = options.rebuildScope == RebuildScope::Cell;
        const bool generation = !cellScope || options.generateCandidate || options.generatePlugin || options.copyPlugin;
        DrawAdvanced(workspace, window, cellScope && !navmesh::app::UsesBatchGeneration(options), generation);
        ImGui::EndDisabled();
        ImGui::EndChild();
        DrawActions(workspace);
        ImGui::End();
    }

    /// Windows/DirectX resources stay on the UI thread; resizing releases all backbuffer references first.
    struct DesktopHost
    {
        HWND window{};
        ComPtr<ID3D11Device> device;
        ComPtr<ID3D11DeviceContext> context;
        ComPtr<IDXGISwapChain> swapChain;
        ComPtr<ID3D11RenderTargetView> renderTarget;
        UINT width{}, height{};
        bool closeRequested{};
        float dpiScale{1};

        void CreateRenderTarget()
        {
            ComPtr<ID3D11Texture2D> buffer;
            if (FAILED(swapChain->GetBuffer(0, IID_PPV_ARGS(&buffer))) ||
                FAILED(device->CreateRenderTargetView(buffer.Get(), nullptr, &renderTarget)))
            {
                throw std::runtime_error("Cannot create the desktop render target.");
            }
        }

        void Initialize()
        {
            DXGI_SWAP_CHAIN_DESC description{};
            description.BufferCount = 2;
            description.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
            description.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
            description.OutputWindow = window;
            description.SampleDesc.Count = 1;
            description.Windowed = TRUE;
            description.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;
            const auto create = [&](D3D_DRIVER_TYPE driver)
            {
                return D3D11CreateDeviceAndSwapChain(nullptr, driver, nullptr, 0, nullptr, 0, D3D11_SDK_VERSION,
                                                     &description, &swapChain, &device, nullptr, &context);
            };
            if (FAILED(create(D3D_DRIVER_TYPE_HARDWARE)) && FAILED(create(D3D_DRIVER_TYPE_WARP)))
            {
                throw std::runtime_error("DirectX 11 is unavailable for the desktop workspace.");
            }
            CreateRenderTarget();
        }
    };

    LRESULT CALLBACK WindowProcedure(HWND window, UINT message, WPARAM wParam, LPARAM lParam)
    {
        if (ImGui_ImplWin32_WndProcHandler(window, message, wParam, lParam))
        {
            return TRUE;
        }
        auto *host = reinterpret_cast<DesktopHost *>(GetWindowLongPtrW(window, GWLP_USERDATA));
        if (message == WM_NCCREATE)
        {
            host = static_cast<DesktopHost *>(reinterpret_cast<CREATESTRUCTW *>(lParam)->lpCreateParams);
            SetWindowLongPtrW(window, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(host));
        }
        if (host)
        {
            switch (message)
            {
            case WM_SIZE:
                if (wParam != SIZE_MINIMIZED)
                {
                    host->width = LOWORD(lParam);
                    host->height = HIWORD(lParam);
                }
                return 0;
            case WM_GETMINMAXINFO:
                reinterpret_cast<MINMAXINFO *>(lParam)->ptMinTrackSize = {static_cast<LONG>(700 * host->dpiScale),
                                                                          static_cast<LONG>(660 * host->dpiScale)};
                return 0;
            case WM_DPICHANGED:
            {
                host->dpiScale = HIWORD(wParam) / 96.0F;
                const auto *rect = reinterpret_cast<RECT *>(lParam);
                SetWindowPos(window, nullptr, rect->left, rect->top, rect->right - rect->left, rect->bottom - rect->top,
                             SWP_NOACTIVATE | SWP_NOZORDER);
                return 0;
            }
            case WM_CLOSE:
                host->closeRequested = true;
                return 0;
            case WM_DESTROY:
                PostQuitMessage(0);
                return 0;
            case WM_SYSCOMMAND:
                if ((wParam & 0xfff0) == SC_KEYMENU)
                {
                    return 0;
                }
                break;
            }
        }
        return DefWindowProcW(window, message, wParam, lParam);
    }

    void ApplyTheme(float scale)
    {
        auto &style = ImGui::GetStyle();
        style = ImGuiStyle{};
        ImGui::StyleColorsDark();
        style.WindowPadding = ImVec2(24, 20);
        style.FramePadding = ImVec2(12, 9);
        style.ItemSpacing = ImVec2(12, 10);
        style.ItemInnerSpacing = ImVec2(8, 8);
        style.WindowRounding = 0;
        style.ChildRounding = 12;
        style.FrameRounding = 6;
        style.PopupRounding = 8;
        style.ScrollbarRounding = 8;
        style.ChildBorderSize = 0;
        style.FrameBorderSize = 1;
        style.Colors[ImGuiCol_WindowBg] = ImVec4(0.055F, 0.071F, 0.102F, 1);
        style.Colors[ImGuiCol_ChildBg] = ImVec4(0.083F, 0.11F, 0.16F, 1);
        style.Colors[ImGuiCol_FrameBg] = ImVec4(0.12F, 0.16F, 0.22F, 1);
        style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.16F, 0.21F, 0.29F, 1);
        style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.18F, 0.25F, 0.34F, 1);
        style.Colors[ImGuiCol_Border] = ImVec4(0.2F, 0.26F, 0.35F, 1);
        style.Colors[ImGuiCol_Text] = ImVec4(0.9F, 0.93F, 0.98F, 1);
        style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.56F, 0.64F, 0.75F, 1);
        style.Colors[ImGuiCol_Button] = ImVec4(0.16F, 0.21F, 0.29F, 1);
        style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.21F, 0.3F, 0.43F, 1);
        style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.17F, 0.36F, 0.65F, 1);
        style.Colors[ImGuiCol_Header] = ImVec4(0.12F, 0.17F, 0.25F, 1);
        style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.17F, 0.27F, 0.41F, 1);
        style.Colors[ImGuiCol_CheckMark] = ImVec4(0.38F, 0.67F, 1, 1);
        style.Colors[ImGuiCol_PlotHistogram] = ImVec4(0.26F, 0.53F, 0.96F, 1);
        style.ScaleAllSizes(scale);
        style.FontScaleDpi = scale;
    }
} // namespace

int navmesh::ui::RunWindowsUi(const app::Options &initialOptions)
{
    const auto com = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    ImGui_ImplWin32_EnableDpiAwareness();
    DesktopHost host;
    const auto instance = GetModuleHandleW(nullptr);
    const auto largeIcon =
        static_cast<HICON>(LoadImageW(instance, MAKEINTRESOURCEW(IDI_NAVMESH_GENERATOR), IMAGE_ICON,
                                      GetSystemMetrics(SM_CXICON), GetSystemMetrics(SM_CYICON), LR_SHARED));
    const auto smallIcon =
        static_cast<HICON>(LoadImageW(instance, MAKEINTRESOURCEW(IDI_NAVMESH_GENERATOR), IMAGE_ICON,
                                      GetSystemMetrics(SM_CXSMICON), GetSystemMetrics(SM_CYSMICON), LR_SHARED));
    const WNDCLASSEXW klass{sizeof(WNDCLASSEXW),
                            CS_CLASSDC,
                            WindowProcedure,
                            0,
                            0,
                            instance,
                            largeIcon,
                            LoadCursor(nullptr, IDC_ARROW),
                            nullptr,
                            nullptr,
                            L"NavmeshGeneratorWorkspace",
                            smallIcon};
    if (!RegisterClassExW(&klass))
    {
        if (SUCCEEDED(com))
        {
            CoUninitialize();
        }
        return 1;
    }
    bool platformInitialized{}, rendererInitialized{};
    int result{};
    try
    {
        Workspace workspace;
        workspace.draft = initialOptions;
        LoadSettings(workspace);
        host.dpiScale = ImGui_ImplWin32_GetDpiScaleForMonitor(MonitorFromPoint(POINT{0, 0}, MONITOR_DEFAULTTOPRIMARY));
        host.window = CreateWindowExW(0, klass.lpszClassName, L"Navmesh Generator", WS_OVERLAPPEDWINDOW, CW_USEDEFAULT,
                                      CW_USEDEFAULT, static_cast<int>(880 * host.dpiScale),
                                      static_cast<int>(950 * host.dpiScale), nullptr, nullptr, klass.hInstance, &host);
        if (!host.window)
        {
            throw std::runtime_error("Cannot create the desktop workspace window.");
        }
        host.Initialize();
        const BOOL darkTitle = TRUE;
        DwmSetWindowAttribute(host.window, DWMWA_USE_IMMERSIVE_DARK_MODE, &darkTitle, sizeof(darkTitle));
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        auto &io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
        io.IniFilename = nullptr;
        io.LogFilename = nullptr;
        std::array<wchar_t, MAX_PATH> windowsDirectory{};
        GetWindowsDirectoryW(windowsDirectory.data(), static_cast<UINT>(windowsDirectory.size()));
        const auto font = std::filesystem::path(windowsDirectory.data()) / "Fonts" / "segoeui.ttf";
        if (std::filesystem::exists(font))
        {
            io.Fonts->AddFontFromFileTTF(PathText(font).c_str(), 18);
        }
        ApplyTheme(host.dpiScale);
        platformInitialized = ImGui_ImplWin32_Init(host.window);
        rendererInitialized = ImGui_ImplDX11_Init(host.device.Get(), host.context.Get());
        if (!platformInitialized || !rendererInitialized)
        {
            throw std::runtime_error("Cannot initialize Dear ImGui desktop rendering.");
        }
        ShowWindow(host.window, SW_SHOWDEFAULT);
        UpdateWindow(host.window);
        bool quit{};
        while (!quit)
        {
            MSG message{};
            while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE))
            {
                TranslateMessage(&message);
                DispatchMessageW(&message);
                quit = quit || message.message == WM_QUIT;
            }
            if (quit)
            {
                break;
            }
            if (host.closeRequested)
            {
                workspace.status.cancel = true;
                std::lock_guard lock(workspace.status.mutex);
                if (!workspace.busy || workspace.status.complete)
                {
                    break;
                }
                workspace.status.text = "Closing after the current safe operation...";
            }
            if (IsIconic(host.window))
            {
                Sleep(30);
                continue;
            }
            if (host.width && host.height)
            {
                host.context->OMSetRenderTargets(0, nullptr, nullptr);
                host.renderTarget.Reset();
                if (FAILED(host.swapChain->ResizeBuffers(0, host.width, host.height, DXGI_FORMAT_UNKNOWN, 0)))
                {
                    throw std::runtime_error("Cannot resize the desktop workspace.");
                }
                host.width = host.height = 0;
                host.CreateRenderTarget();
            }
            if (ImGui::GetStyle().FontScaleDpi != host.dpiScale)
            {
                ApplyTheme(host.dpiScale);
            }
            ImGui_ImplDX11_NewFrame();
            ImGui_ImplWin32_NewFrame();
            ImGui::NewFrame();
            DrawWorkspace(workspace, host.window);
            ImGui::Render();
            const float clear[] = {0.055F, 0.071F, 0.102F, 1};
            auto *target = host.renderTarget.Get();
            host.context->OMSetRenderTargets(1, &target, nullptr);
            host.context->ClearRenderTargetView(target, clear);
            ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
            if (FAILED(host.swapChain->Present(1, 0)))
            {
                throw std::runtime_error("The desktop graphics device stopped rendering.");
            }
        }
        SaveSettings(workspace);
    }
    catch (const std::exception &error)
    {
        MessageBoxW(host.window, Wide(error.what()).c_str(), L"Navmesh Generator", MB_OK | MB_ICONERROR);
        result = 1;
    }
    if (rendererInitialized)
    {
        ImGui_ImplDX11_Shutdown();
    }
    if (platformInitialized)
    {
        ImGui_ImplWin32_Shutdown();
    }
    if (ImGui::GetCurrentContext())
    {
        ImGui::DestroyContext();
    }
    if (host.window)
    {
        DestroyWindow(host.window);
    }
    UnregisterClassW(klass.lpszClassName, klass.hInstance);
    if (SUCCEEDED(com))
    {
        CoUninitialize();
    }
    return result;
}
