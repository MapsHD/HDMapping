#pragma once

#include "child_process.h"
#include "export.h"
#include "preflight.h"
#include "progress_parser.h"
#include "settings.h"

#include <atomic>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace easy_gui
{
    struct PipelineRequest
    {
        std::filesystem::path recording;
        std::filesystem::path output_folder;
        std::filesystem::path step1_executable;
        Settings settings;
        //! Dev/test only (ExportRequest::check_band_outer_m): > 0 adds alignment-check points to the export.
        double check_band_outer_m = 0.0;
        //! Batch mode: the output folder may hold an unfinished earlier attempt (holdsUnfinishedOutput); everything in
        //! it except easy_gui.log is removed before the run, and the log is continued.
        bool resume_unfinished = false;
    };

    enum class PipelineState
    {
        Idle,
        Running,
        Succeeded,
        Failed,
        Cancelled
    };

    enum class PipelineStage
    {
        Checking,
        Staging,
        Odometry,
        Export,
        Cleanup
    };

    struct PipelineStatus
    {
        PipelineState state = PipelineState::Idle;
        PipelineStage stage = PipelineStage::Checking;
        Step1Progress step1;
        std::string message; // plain-language outcome / failure reason
        std::string detail; // technical detail for the "details" view
        double elapsed_s = 0.0;
        std::filesystem::path session_folder; // step 1 session (session.mjs) once odometry succeeded
        double export_fraction = 0.0; // 0..1 during PipelineStage::Export
        std::string export_step; // what the export is doing right now
        ExportStats export_stats; // filled once the export succeeded
    };

    //! Layout of an Easy GUI output folder.
    struct OutputLayout
    {
        explicit OutputLayout(const std::filesystem::path& root);
        std::filesystem::path root;
        std::filesystem::path log_file; // easy_gui.log: everything, including the full step 1 output
        std::filesystem::path work; // work/: intermediate data
        std::filesystem::path staging; // work/staging: links to the recording; step 1 puts cache/ and preview/ here
        std::filesystem::path step1_parameters; // work/easy_gui_step1.toml
        std::filesystem::path session; // work/lio_session: step 1 output (session.mjs, scan_lio_N.laz, ...)
        std::filesystem::path point_cloud; // point_cloud.laz: the exported point cloud
        std::filesystem::path trajectory; // trajectory.csv
        std::filesystem::path readme; // README.txt: describes the two files above
        std::filesystem::path done_marker; // easy_gui_done.txt: written by batch mode once the result is complete
    };

    //! Locates lidar_odometry_step_1 next to the Easy GUI executable (plain build name or the installed,
    //! HDMAPPING_BINARY_PREFIX-ed one). The HDMAPPING_STEP1_EXECUTABLE environment variable overrides the search.
    std::filesystem::path findStep1Executable();

    //! Runs the whole Easy GUI processing on a worker thread: re-check, stage, step 1 in a child process, export, clean
    //! up.
    //! The GUI polls status() every frame; cancel() kills the child process.
    class Pipeline
    {
    public:
        ~Pipeline();

        bool start(const PipelineRequest& request, std::string& error);
        void cancel();
        //! Back to Idle after a finished run (no-op while busy).
        void reset();
        bool busy() const
        {
            return busy_.load();
        }

        PipelineStatus status() const;
        //! Last lines of the log, oldest first.
        std::vector<std::string> logTail() const;

    private:
        void run();
        void runExport(const PreflightResult& check, const OutputLayout& out, const std::function<void(bool)>& cleanupWork);
        void log(const std::string& line);
        void setStage(PipelineStage stage);
        void finish(PipelineState state, const std::string& message, const std::string& detail = "");

        PipelineRequest request_;
        std::thread thread_;
        std::atomic<bool> busy_{ false };
        std::atomic<bool> cancel_requested_{ false };

        mutable std::mutex mutex_;
        PipelineStatus status_;
        std::deque<std::string> log_tail_;
        std::chrono::steady_clock::time_point started_;
        ChildProcess* child_ = nullptr; // valid while the odometry child runs; guarded by mutex_
        std::ofstream log_file_; // only touched by the worker thread
    };
} // namespace easy_gui
