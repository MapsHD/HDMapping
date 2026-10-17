#include "pipeline.h"

#include "forestry_preset.h"
#include "staging.h"

#include <HDMapping/HDMAPPING_ConfigureInfo.hpp>

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

namespace easy_gui
{
    namespace
    {
        constexpr size_t kLogTailLines = 400;

        std::string timestamp()
        {
            const std::time_t now = std::time(nullptr);
            std::tm tm{};
#ifdef _WIN32
            localtime_s(&tm, &now);
#else
            localtime_r(&now, &tm);
#endif
            std::ostringstream out;
            out << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
            return out.str();
        }

        const char* severityTag(Severity s)
        {
            return s == Severity::Error ? "ERROR" : s == Severity::Warning ? "WARNING" : "OK";
        }
    } // namespace

    OutputLayout::OutputLayout(const fs::path& root_)
        : root(root_)
        , log_file(root_ / "easy_gui.log")
        , work(root_ / "work")
        , staging(root_ / "work" / "staging")
        , step1_parameters(root_ / "work" / "easy_gui_step1.toml")
        , session(root_ / "work" / "lio_session")
        , point_cloud(root_ / "point_cloud.laz")
        , trajectory(root_ / "trajectory.csv")
        , readme(root_ / "README.txt")
    {
    }

    fs::path findStep1Executable()
    {
        if (const char* overridden = std::getenv("HDMAPPING_STEP1_EXECUTABLE"); overridden && *overridden)
            return fs::path(overridden);

        const fs::path dir = currentExecutableDirectory();
#ifdef _WIN32
        const char* suffix = ".exe";
#else
        const char* suffix = "";
#endif
        for (const std::string name :
             { std::string("lidar_odometry_step_1"), std::string(HDMAPPING_BINARY_PREFIX) + "lidar_odometry_step_1" })
        {
            const fs::path candidate = dir / (name + suffix);
            std::error_code ec;
            if (fs::is_regular_file(candidate, ec))
                return candidate;
        }
        return {};
    }

    Pipeline::~Pipeline()
    {
        cancel();
        if (thread_.joinable())
            thread_.join();
    }

    bool Pipeline::start(const PipelineRequest& request, std::string& error)
    {
        if (busy_)
        {
            error = "Processing is already running.";
            return false;
        }
        if (thread_.joinable())
            thread_.join();

        request_ = request;
        request_.settings.clamp();
        cancel_requested_ = false;
        {
            std::lock_guard lock(mutex_);
            status_ = PipelineStatus{};
            status_.state = PipelineState::Running;
            log_tail_.clear();
            started_ = std::chrono::steady_clock::now();
        }
        busy_ = true;
        thread_ = std::thread(&Pipeline::run, this);
        return true;
    }

    void Pipeline::cancel()
    {
        cancel_requested_ = true;
        std::lock_guard lock(mutex_);
        if (child_)
            child_->kill();
    }

    void Pipeline::reset()
    {
        if (busy_)
            return;
        if (thread_.joinable())
            thread_.join();
        std::lock_guard lock(mutex_);
        status_ = PipelineStatus{};
        log_tail_.clear();
    }

    PipelineStatus Pipeline::status() const
    {
        std::lock_guard lock(mutex_);
        PipelineStatus s = status_;
        if (s.state == PipelineState::Running)
            s.elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
        return s;
    }

    std::vector<std::string> Pipeline::logTail() const
    {
        std::lock_guard lock(mutex_);
        return { log_tail_.begin(), log_tail_.end() };
    }

    void Pipeline::log(const std::string& line)
    {
        if (log_file_.is_open())
            log_file_ << line << '\n';
        std::lock_guard lock(mutex_);
        log_tail_.push_back(line);
        while (log_tail_.size() > kLogTailLines)
            log_tail_.pop_front();
    }

    void Pipeline::setStage(PipelineStage stage)
    {
        std::lock_guard lock(mutex_);
        status_.stage = stage;
    }

    void Pipeline::finish(PipelineState state, const std::string& message, const std::string& detail)
    {
        log("[easy_gui] " + timestamp() + " " + (state == PipelineState::Succeeded ? "finished: " : "stopped: ") + message);
        if (!detail.empty())
            log("[easy_gui] detail: " + detail);
        log_file_.close(); // finish() is the last thing a run logs
        std::lock_guard lock(mutex_);
        status_.state = state;
        status_.message = message;
        status_.detail = detail;
        status_.elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
    }

    void Pipeline::run()
    {
        const OutputLayout out(request_.output_folder);
        std::error_code ec;

        auto cleanupWork = [&](bool success)
        {
            setStage(PipelineStage::Cleanup);
            if (request_.settings.keep_work_files)
                return;
            // The session stays on success: the export reads it, and an expert can open it in step 2. Everything
            // else in work/ (links, step 1's cache/ and preview/) is disposable, and the cache can be gigabytes.
            fs::remove_all(success ? out.staging : out.work, ec);
        };

        // --- 1. re-check: the recording or the disk may have changed since the user pressed "Check"
        setStage(PipelineStage::Checking);
        const PreflightResult check = runPreflight(request_.recording, request_.output_folder);
        if (!check.ok())
        {
            std::string first_error;
            for (const auto& f : check.findings)
                if (f.severity == Severity::Error)
                {
                    first_error = f.message;
                    break;
                }
            finish(PipelineState::Failed, first_error);
            busy_ = false;
            return;
        }

        if (!fs::create_directories(out.root, ec) && ec)
        {
            finish(PipelineState::Failed, "Could not create the output folder '" + out.root.string() + "' (" + ec.message() + ").");
            busy_ = false;
            return;
        }
        log_file_.open(out.log_file, std::ios::out | std::ios::app);

        log("[easy_gui] HDMapping Easy GUI " + std::string(HDMAPPING_CONFIGURE_PROJECT_VERSION) + ", started " + timestamp());
        log("[easy_gui] recording: " + request_.recording.string());
        log("[easy_gui] output: " + out.root.string());
        log("[easy_gui] step 1 program: " + request_.step1_executable.string());
        {
            std::ostringstream s;
            s << "[easy_gui] settings: near_min_range_m=" << request_.settings.near_min_range_m
              << " place_by_raw_time=" << request_.settings.place_by_raw_time << " keep_work_files=" << request_.settings.keep_work_files;
            log(s.str());
        }
        for (const auto& f : check.findings)
            log(std::string("[easy_gui] check ") + severityTag(f.severity) + ": " + f.message);

        // --- 2. staging folder + parameter file
        setStage(PipelineStage::Staging);
        std::string error;
        std::string method;
        if (!createStagingFolder(check.files, out.staging, method, error))
        {
            cleanupWork(false);
            finish(PipelineState::Failed, error);
            busy_ = false;
            return;
        }
        log("[easy_gui] staged " + std::to_string(check.files.pairs) + " file pairs as " + method);
        if (!writeStep1ParameterFile(out.step1_parameters, error))
        {
            cleanupWork(false);
            finish(PipelineState::Failed, error);
            busy_ = false;
            return;
        }

        // --- 3. lidar odometry in a child process
        setStage(PipelineStage::Odometry);
        if (request_.step1_executable.empty() || !fs::is_regular_file(request_.step1_executable, ec))
        {
            cleanupWork(false);
            finish(
                PipelineState::Failed,
                "Easy GUI could not find the lidar odometry program (lidar_odometry_step_1) next to itself. Please reinstall HDMapping.",
                "looked for: " + request_.step1_executable.string());
            busy_ = false;
            return;
        }

        ChildProcess child;
        const std::vector<std::string> args = { out.staging.string(), out.step1_parameters.string(), out.session.string() };
        log("[easy_gui] running: " + request_.step1_executable.string() + " " + args[0] + " " + args[1] + " " + args[2]);
        if (cancel_requested_ || !child.start(request_.step1_executable, args, error))
        {
            cleanupWork(false);
            if (cancel_requested_)
                finish(PipelineState::Cancelled, "Processing was cancelled.");
            else
                finish(PipelineState::Failed, "The lidar odometry program could not be started.", error);
            busy_ = false;
            return;
        }
        {
            std::lock_guard lock(mutex_);
            child_ = &child;
        }
        if (cancel_requested_) // cancel() may have run between start() and publishing child_
            child.kill();

        ProgressParser parser;
        std::string line;
        while (child.readLine(line))
        {
            parser.feed(line);
            log(line);
            std::lock_guard lock(mutex_);
            status_.step1 = parser.progress();
        }
        const int exit_code = child.wait();
        {
            std::lock_guard lock(mutex_);
            child_ = nullptr;
            status_.step1 = parser.progress();
        }

        const bool session_written = fs::is_regular_file(out.session / "session.mjs", ec);
        if (cancel_requested_ || child.wasKilled())
        {
            cleanupWork(false);
            finish(PipelineState::Cancelled, "Processing was cancelled. Nothing was kept except the log file.");
        }
        // NOTE: step 1 exits with 0 after a caught exception, so the session file is the success criterion, not the
        // exit code alone.
        else if (exit_code != 0 || !session_written || parser.progress().failed)
        {
            std::string message = parser.progress().failure;
            if (message.empty())
                message = exit_code != 0 ? "Lidar odometry stopped unexpectedly (exit code " + std::to_string(exit_code) + ")."
                                         : "Lidar odometry finished without writing a result.";
            cleanupWork(false);
            finish(PipelineState::Failed, message, parser.progress().last_error_line);
        }
        else
        {
            {
                std::lock_guard lock(mutex_);
                status_.session_folder = out.session;
            }
            log("[easy_gui] " + timestamp() + " lidar odometry finished, exporting");
            runExport(check, out, cleanupWork);
        }
        busy_ = false;
    }

    void Pipeline::runExport(const PreflightResult& check, const OutputLayout& out, const std::function<void(bool)>& cleanupWork)
    {
        setStage(PipelineStage::Export);
        ExportRequest request;
        request.session_file = out.session / "session.mjs";
        request.recording = check.files;
        request.lidar_id = std::max(0, check.lidar_id);
        request.near_min_range_m = request_.settings.near_min_range_m;
        request.check_band_outer_m = request_.check_band_outer_m;
        request.place_by_raw_time = request_.settings.place_by_raw_time;
        request.laz_file = out.point_cloud;
        request.trajectory_file = out.trajectory;
        request.readme_file = out.readme;

        std::string last_phase;
        auto progress = [&](double fraction, const std::string& step)
        {
            // One log line per phase, not per scan/file: "Writing the point cloud (scan 3 of 113)" -> "Writing the point cloud"
            const std::string phase = step.substr(0, step.find(" ("));
            if (phase != last_phase)
                log("[easy_gui] " + timestamp() + " export: " + phase);
            last_phase = phase;
            std::lock_guard lock(mutex_);
            status_.export_fraction = fraction;
            status_.export_step = step;
        };

        ExportStats stats;
        std::string error;
        const bool ok = exportSession(request, cancel_requested_, progress, stats, error);
        std::error_code ec;
        if (!ok)
        {
            for (const auto& f : { out.point_cloud, out.trajectory, out.readme })
                fs::remove(f, ec);
            if (cancel_requested_)
            {
                cleanupWork(false);
                finish(PipelineState::Cancelled, "Processing was cancelled. Nothing was kept except the log file.");
                return;
            }
            // NOTE: unlike the other failures, the odometry session stays: it took most of the run, and step 2 can
            // open and export it.
            cleanupWork(true);
            finish(
                PipelineState::Failed,
                "Lidar odometry finished, but the export failed: " + error + " The lidar odometry result was kept in '" +
                    out.session.string() + "'.");
            return;
        }

        {
            std::ostringstream s;
            s << "[easy_gui] export: " << stats.odometry_points << " odometry points + " << stats.near_points << " near points";
            if (stats.check_band_points)
                s << " + " << stats.check_band_points << " check-band points";
            s << " (" << stats.raw_no_return << " of " << stats.raw_points_read << " raw points were no-return, "
              << stats.raw_outside_trajectory << " candidates outside the trajectory), range " << stats.range_min_m << " - "
              << stats.range_max_m << " m, " << stats.trajectory_nodes << " trajectory poses; point cloud " << stats.laz_bytes
              << " bytes, trajectory " << stats.trajectory_bytes << " bytes";
            log(s.str());
        }
        {
            std::lock_guard lock(mutex_);
            status_.export_fraction = 1.0;
            status_.export_stats = stats;
        }
        cleanupWork(true);
        finish(PipelineState::Succeeded, "Done. The point cloud and trajectory are in '" + out.root.string() + "'.");
    }
} // namespace easy_gui
