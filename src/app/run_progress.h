#pragma once

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

namespace navmesh::app
{
    struct Options;
    /// Serialized status sink; calls can come from the processing or heartbeat thread. Do not throw or reenter Run.
    using ProgressCallback = std::function<void(int percent, std::string_view status)>;

    /// Activity classification; a heartbeat establishes liveness without claiming processing progress.
    enum class RunEventKind
    {
        Progress,
        Detail,
        Heartbeat,
        CellCompleted,
        Warning,
        Finished
    };

    /// Owned activity snapshot shared by console, desktop and persistent logging.
    struct RunEvent
    {
        RunEventKind kind{RunEventKind::Progress};
        /// Pipeline progress indication; heartbeat events retain the latest value.
        int percent{};
        /// Current operation or diagnostic, including input identities supplied by the pipeline.
        std::string message;
        /// Latest batch counters and worker snapshot; empty outside a batch or when unchanged by the publisher.
        std::string batchStatus;
        /// Steady-clock seconds since run start, operation change and latest pipeline event, respectively.
        double elapsedSeconds{}, operationSeconds{}, idleSeconds{};
        /// Steady-clock seconds since the latest collected CELL result; absent before any result completes.
        std::optional<double> sinceCompletionSeconds;
    };

    /// Serialized borrowed-event sink. Do not throw, reenter Run, or retain the event reference.
    using RunEventCallback = std::function<void(const RunEvent &)>;

    namespace detail
    {
        /** Format a pasteable failure report for text-only support sites.
         * @param options Shared CLI/desktop run inputs; selected filenames and selectors are bounded.
         * @param failure Terminal failure with record and connection evidence when available.
         * @param code Process failure status, excluding successful and cancelled runs.
         * @return ASCII text bounded below the support site's character limit, with explicit truncation markers.
         * Includes build identity, selection and generation settings. Does not read inputs or perform I/O.
         */
        [[nodiscard]] std::string FormatBugReport(const Options &options, std::string_view failure, int code);

        /** Own one run's serialized activity stream and append-only UTF-8 run.log.
         * Logging failures warn consumers without changing generation or export success.
         * Heartbeat callbacks run on a private thread and stop before Finish returns or destruction completes.
         */
        class RunProgress
        {
          public:
            /** Start logging and heartbeat delivery; create only the requested output directory.
             * @param output Run output directory; an unavailable log is reported through the sinks.
             * @param progress Optional human-readable status sink.
             * @param events Optional structured sink, serialized with progress delivery.
             * @param heartbeatPeriod Positive interval between idle liveness reports; configurable for tests.
             */
            RunProgress(const std::filesystem::path &output, ProgressCallback progress, RunEventCallback events,
                        std::chrono::milliseconds heartbeatPeriod = std::chrono::seconds(5));
            /// Stop and join heartbeat delivery without publishing a success result.
            ~RunProgress();
            /// Publish pipeline activity. CellCompleted resets the last-completion clock; heartbeats never do.
            void Publish(RunEvent event);
            /// Join heartbeat delivery and publish a terminal event retaining the last operation or failure reason.
            void Finish(int code);

          private:
            void Emit(RunEvent event, std::chrono::steady_clock::time_point now);
            void Stop();

            ProgressCallback progress_;
            RunEventCallback events_;
            std::ofstream log_;
            std::mutex mutex_;
            std::condition_variable wake_;
            bool stopping_{}, logFailed_{};
            RunEvent latest_;
            std::string batchStatus_;
            std::chrono::steady_clock::time_point started_, operationStarted_, lastActivity_;
            std::chrono::steady_clock::time_point lastDetail_{};
            std::optional<std::chrono::steady_clock::time_point> lastCompletion_;
            std::thread heartbeat_;
        };
    } // namespace detail
} // namespace navmesh::app
