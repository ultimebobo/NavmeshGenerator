#include "app/run_progress.h"
#include "app/options.h"
#include "core/io/file_path.h"
#include "core/reproducibility/export_metadata.h"

#include <format>
#include <stdexcept>

namespace navmesh::app::detail
{
    namespace
    {
        std::string_view EventName(RunEventKind kind)
        {
            switch (kind)
            {
            case RunEventKind::Heartbeat:
                return "heartbeat";
            case RunEventKind::Detail:
                return "detail";
            case RunEventKind::CellCompleted:
                return "cell_completed";
            case RunEventKind::Warning:
                return "warning";
            case RunEventKind::Finished:
                return "finished";
            default:
                return "progress";
            }
        }

        // Preserve one physical log line per event even when a diagnostic includes multiple lines.
        std::string LogText(std::string_view message)
        {
            std::string result;
            for (const char character : message)
            {
                if (character == '\n')
                {
                    result += "\\n";
                }
                else if (character == '\r')
                {
                    result += "\\r";
                }
                else
                {
                    result += character;
                }
            }
            return result;
        }

        /// Escape control and UTF-8 bytes so the byte budget also bounds pasted characters on support sites.
        std::string ReportText(std::string_view text, std::size_t limit, bool multiline = false)
        {
            constexpr std::string_view truncated = "...[truncated]";
            std::string result;
            for (const unsigned char character : text)
            {
                const auto encoded = (multiline && character == '\n') || (character >= 32 && character <= 126)
                                         ? std::string(1, static_cast<char>(character))
                                         : std::format("\\x{:02X}", character);
                if (result.size() + encoded.size() + truncated.size() > limit)
                {
                    result += truncated;
                    return result;
                }
                result += encoded;
            }
            return result;
        }
    } // namespace

    std::string FormatBugReport(const Options &options, std::string_view failure, int code)
    {
        // Leave room for the reporter's description without requiring attachments or a full load-order listing.
        constexpr std::size_t maximumBytes = 4500;
        const auto &profile = options.navigationProfile;
        const auto &recast = options.recastSettings;
        std::string report = std::format("NavmeshGenerator bug report v1\nversion={} build={} {} exit={}\n",
                                         reproducibility::kToolVersion, __DATE__, __TIME__, code);
        report += std::format("input={} scope={} plugin='{}' copy_source='{}'\n",
                              !options.mo2.empty()      ? "MO2"
                              : !options.vortex.empty() ? "Vortex"
                              : !options.plugin.empty() ? "plugin"
                                                        : "load-order",
                              options.rebuildScope == RebuildScope::Cell     ? "cell"
                              : options.rebuildScope == RebuildScope::Plugin ? "plugin"
                              : options.rebuildScope == RebuildScope::Repair ? "repair"
                                                                             : "load-order",
                              ReportText(options.affectedPlugin.empty() ? core::PathToUtf8(options.plugin.filename())
                                                                        : options.affectedPlugin,
                                         160),
                              ReportText(options.copySourcePlugin, 160));
        report += std::format(
            "selection='{}' cell='{}' editor='{}' form={} world='{}' xy={},{}\n",
            ReportText(options.cellSelection, 200), ReportText(options.cell, 100), ReportText(options.editorId, 100),
            options.cellFormId ? std::format("{:08X}", *options.cellFormId) : "unset",
            ReportText(options.worldspace, 100), options.cellX ? std::to_string(*options.cellX) : "unset",
            options.cellY ? std::to_string(*options.cellY) : "unset");
        report += std::format(
            "write={} copy={} auto_split={} skip_existing={} tag_water={} tag_preferred_paths={} terrain_only={} "
            "cell_errors={} neighbors={} partition={} scene={} estimate={} output={}\n",
            options.generatePlugin, options.copyPlugin, options.autoSplitPlugins, options.skipExistingNavmesh,
            options.tagWater, options.tagPreferredPaths, options.terrainOnly, ReportText(options.cellErrorPolicy, 40),
            options.neighboringCellRadius, static_cast<int>(options.partitioningAlgorithm), options.makeScene,
            options.estimateOnly, ReportText(options.batchOutput, 40));
        report +=
            std::format("profile='{}' radius={} height={} slope_deg={} step={} clearance={} weld={} "
                        "min_area={} contour_tolerance={} border='{}'\n",
                        ReportText(profile.name, 60), profile.agentRadius, profile.agentHeight, profile.maxSlopeDegrees,
                        profile.stepHeight, profile.clearance, profile.weldTolerance, profile.minimumRegionArea,
                        profile.contourSimplificationTolerance, ReportText(profile.cellBorderPolicy, 60));
        report += std::format("recast cell_size={} cell_height={} simplification={} edge_length={} merge_area={}\n"
                              "workers={} memory_mib={} cache_mib={} search_radius={} support_distance={} "
                              "analysis_slope={}\nFailure:\n",
                              recast.cellSize, recast.cellHeight, recast.maxSimplificationError, recast.maxEdgeLength,
                              recast.mergeRegionAreaMultiplier, options.workers, options.workingMemoryMiB,
                              options.cacheBudgetMiB, options.surfaceSearchRadius, options.maxSupportDistance,
                              options.maxSlope);
        report += ReportText(failure, maximumBytes - report.size() - 1, true);
        report += '\n';
        return report;
    }

    RunProgress::RunProgress(const std::filesystem::path &output, ProgressCallback progress, RunEventCallback events,
                             std::chrono::milliseconds heartbeatPeriod)
        : progress_(std::move(progress)), events_(std::move(events)), started_(std::chrono::steady_clock::now()),
          operationStarted_(started_), lastActivity_(started_)
    {
        if (heartbeatPeriod <= std::chrono::milliseconds::zero())
        {
            throw std::invalid_argument("Heartbeat interval must be positive");
        }
        std::error_code error;
        std::filesystem::create_directories(output, error);
        if (!error)
        {
            log_.open(core::NativeFilePath(output / "run.log"), std::ios::app | std::ios::binary);
        }
        Publish({.message = std::format("Run started; activity log: {}", core::PathToUtf8(output / "run.log"))});
        if (!log_.is_open())
        {
            logFailed_ = true;
            Publish(
                {.kind = RunEventKind::Warning,
                 .message = "Cannot write run.log; check output-folder permissions and free space. Run continues."});
        }
        heartbeat_ = std::thread(
            [this, heartbeatPeriod]
            {
                std::unique_lock lock(mutex_);
                while (!wake_.wait_for(lock, heartbeatPeriod, [this] { return stopping_; }))
                {
                    const auto now = std::chrono::steady_clock::now();
                    if (now - lastActivity_ >= heartbeatPeriod)
                    {
                        auto event = latest_;
                        event.kind = RunEventKind::Heartbeat;
                        Emit(std::move(event), now);
                    }
                }
            });
    }

    RunProgress::~RunProgress()
    {
        Stop();
    }

    void RunProgress::Stop()
    {
        {
            std::lock_guard lock(mutex_);
            stopping_ = true;
        }
        wake_.notify_all();
        if (heartbeat_.joinable())
        {
            heartbeat_.join();
        }
    }

    void RunProgress::Publish(RunEvent event)
    {
        std::lock_guard lock(mutex_);
        const auto now = std::chrono::steady_clock::now();
        if (event.message != latest_.message && event.kind != RunEventKind::Warning)
        {
            operationStarted_ = now;
        }
        lastActivity_ = now;
        if (event.kind == RunEventKind::CellCompleted)
        {
            lastCompletion_ = now;
        }
        if (!event.batchStatus.empty())
        {
            batchStatus_ = event.batchStatus;
        }
        // Warnings are visible events but must not replace the operation repeated by a heartbeat.
        if (event.kind != RunEventKind::Warning)
        {
            latest_ = event;
        }
        // Keep the latest identity for idle reporting without flushing or rendering every reference in a dense CELL.
        if (event.kind == RunEventKind::Detail)
        {
            if (now - lastDetail_ < std::chrono::milliseconds(250))
            {
                return;
            }
            lastDetail_ = now;
        }
        Emit(std::move(event), now);
    }

    void RunProgress::Emit(RunEvent event, std::chrono::steady_clock::time_point now)
    {
        event.elapsedSeconds = std::chrono::duration<double>(now - started_).count();
        event.operationSeconds = std::chrono::duration<double>(now - operationStarted_).count();
        event.idleSeconds = std::chrono::duration<double>(now - lastActivity_).count();
        event.batchStatus = batchStatus_;
        if (lastCompletion_)
        {
            event.sinceCompletionSeconds = std::chrono::duration<double>(now - *lastCompletion_).count();
        }
        auto status = std::format("[elapsed {:.0f}s] {}{}", event.elapsedSeconds,
                                  event.kind == RunEventKind::Heartbeat ? "Still running: " : "", event.message);
        if (!event.batchStatus.empty())
        {
            status += " | " + event.batchStatus;
        }
        if (event.kind == RunEventKind::Heartbeat)
        {
            status += std::format(" (operation {:.0f}s; last activity {:.0f}s ago", event.operationSeconds,
                                  event.idleSeconds);
            if (event.sinceCompletionSeconds)
            {
                status += std::format("; last CELL completion {:.0f}s ago", *event.sinceCompletionSeconds);
            }
            status += ")";
        }
        if (log_.is_open() && !logFailed_)
        {
            const auto timestamp = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
            log_ << std::format("{:%FT%TZ} {} {}% {}\n", timestamp, EventName(event.kind), event.percent,
                                LogText(status));
            log_.flush();
            if (!log_)
            {
                logFailed_ = true;
                Emit({.kind = RunEventKind::Warning,
                      .percent = event.percent,
                      .message = "Cannot continue writing run.log; check free space and folder access. Run continues."},
                     now);
            }
        }
        if (events_)
        {
            events_(event);
        }
        if (progress_)
        {
            progress_(event.percent, status);
        }
    }

    void RunProgress::Finish(int code)
    {
        Stop();
        auto event = latest_;
        event.kind = RunEventKind::Finished;
        event.message = std::format("{} (exit code {}): {}",
                                    code == 0   ? "Completed"
                                    : code == 3 ? "Cancelled"
                                                : "Failed",
                                    code, event.message);
        Publish(std::move(event));
    }
} // namespace navmesh::app::detail
