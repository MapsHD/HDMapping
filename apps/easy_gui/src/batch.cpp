#include "batch.h"

#include "sleep_blocker.h"
#include "version.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <numeric>
#include <set>
#include <sstream>

namespace fs = std::filesystem;

namespace easy_gui
{
    namespace
    {
        constexpr size_t kLogTailLines = 400;
        constexpr const char* kInvalidNameCharacters = "/\\:*?\"<>|";

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

        std::string lowerAscii(std::string s)
        {
            for (char& c : s)
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            return s;
        }

        bool hasRecordingFiles(const RecordingFiles& files)
        {
            return !files.laz_files.empty() && !files.csv_files.empty();
        }

        bool ignoredFolder(const std::string& name)
        {
            return name.empty() || name[0] == '.' || name[0] == '$' || name == "System Volume Information";
        }

        std::string notARecordingReason(const fs::path& folder, const RecordingFiles& files)
        {
            std::error_code ec;
            if (fs::exists(folder / BatchRunner::kSummaryFileName, ec) || fs::exists(folder / "easy_gui.log", ec))
                return "Easy Forest HDMapping results, not a recording.";
            std::string reason = files.laz_files.empty() && files.csv_files.empty()
                ? "No point cloud files (lidar*.laz) and no IMU files (imu*.csv)."
                : files.laz_files.empty() ? "No point cloud files (lidar*.laz)."
                                          : "No IMU files (imu*.csv).";
            for (const auto& entry : fs::directory_iterator(folder, ec))
                if (entry.is_directory(ec) && !ignoredFolder(entry.path().filename().string()))
                {
                    reason += " Recordings are only looked for one folder level down.";
                    break;
                }
            return reason;
        }

        //! Folder name with the characters outputNameProblem() rejects replaced.
        std::string sanitizedName(const std::string& name)
        {
            std::string out;
            for (char c : name)
                out += (static_cast<unsigned char>(c) < 32 || std::strchr(kInvalidNameCharacters, c)) ? '_' : c;
            while (!out.empty() && (out.back() == ' ' || out.back() == '.'))
                out.pop_back();
            if (!outputNameProblem(out).empty())
                out = "recording_" + out;
            return outputNameProblem(out).empty() ? out : "recording";
        }

        std::string csvField(const std::string& s)
        {
            if (s.find_first_of(",\"\r\n") == std::string::npos)
                return s;
            std::string out = "\"";
            for (char c : s)
                out += c == '"' ? std::string("\"\"") : std::string(1, c);
            return out + "\"";
        }

        std::string formatSeconds(double s)
        {
            std::ostringstream out;
            out << std::fixed << std::setprecision(0) << s;
            return out.str();
        }

        class PipelineJob : public RecordingJob
        {
        public:
            bool start(const PipelineRequest& request, std::string& error) override
            {
                return pipeline_.start(request, error);
            }
            bool busy() const override
            {
                return pipeline_.busy();
            }
            PipelineStatus status() const override
            {
                return pipeline_.status();
            }
            void cancel() override
            {
                pipeline_.cancel();
            }
            std::vector<std::string> logTail() const override
            {
                return pipeline_.logTail();
            }

        private:
            Pipeline pipeline_;
        };
    } // namespace

    std::vector<DiscoveredFolder> discoverRecordings(const fs::path& parent)
    {
        std::vector<DiscoveredFolder> found;
        std::error_code ec;
        if (!fs::is_directory(parent, ec))
            return found;

        const RecordingFiles own = classifyRecordingFiles(parent);
        if (hasRecordingFiles(own))
        {
            const fs::path normal = parent.lexically_normal();
            const std::string name = normal.filename().empty() ? normal.parent_path().filename().string() : normal.filename().string();
            found.push_back({ parent, name.empty() ? std::string("recording") : name, true, {} });
        }

        std::vector<fs::path> subfolders;
        for (const auto& entry : fs::directory_iterator(parent, ec))
            if (entry.is_directory(ec) && !ignoredFolder(entry.path().filename().string()))
                subfolders.push_back(entry.path());
        std::sort(subfolders.begin(), subfolders.end());
        for (const auto& folder : subfolders)
        {
            const RecordingFiles files = classifyRecordingFiles(folder);
            DiscoveredFolder d{ folder, folder.filename().string(), hasRecordingFiles(files), {} };
            if (!d.is_recording)
                d.skip_reason = notARecordingReason(folder, files);
            found.push_back(std::move(d));
        }
        return found;
    }

    std::string outputNameProblem(const std::string& name)
    {
        if (name.empty())
            return "Enter a name.";
        if (name == "." || name == "..")
            return "'" + name + "' cannot be used as a folder name.";
        for (char c : name)
            if (static_cast<unsigned char>(c) < 32 || std::strchr(kInvalidNameCharacters, c))
                return "A folder name cannot contain / \\ : * ? \" < > |.";
        if (name.back() == ' ' || name.back() == '.')
            return "A folder name cannot end with a space or a dot.";
        if (name.size() > 120)
            return "The name is too long.";
        // Device names Windows reserves, with or without an extension.
        const std::string stem = lowerAscii(name.substr(0, name.find('.')));
        static const std::set<std::string> kReserved = { "con",  "prn",  "aux",  "nul",  "com1", "com2", "com3", "com4",
                                                         "com5", "com6", "com7", "com8", "com9", "lpt1", "lpt2", "lpt3",
                                                         "lpt4", "lpt5", "lpt6", "lpt7", "lpt8", "lpt9" };
        if (kReserved.contains(stem))
            return "'" + name + "' is reserved by Windows.";
        return {};
    }

    bool sameOutputName(const std::string& a, const std::string& b)
    {
        return lowerAscii(a) == lowerAscii(b);
    }

    std::string recordingFingerprint(const RecordingFiles& files)
    {
        uint64_t bytes = 0;
        std::error_code ec;
        for (size_t i = 0; i < files.pairs; i++)
            bytes += fs::file_size(files.laz_files[i], ec) + fs::file_size(files.csv_files[i], ec);
        // The first IMU timestamp tells apart recordings of the same size (e.g. two equally long walks).
        std::string first_sample;
        if (!files.csv_files.empty())
        {
            std::ifstream imu(files.csv_files.front());
            std::string line;
            while (first_sample.empty() && std::getline(imu, line))
                if (!line.empty() && std::isdigit(static_cast<unsigned char>(line[0])))
                    first_sample = line.substr(0, line.find_first_of(" ,\t\r"));
        }
        return std::to_string(files.pairs) + " file pairs, " + std::to_string(bytes) + " bytes, first IMU sample " +
            (first_sample.empty() ? "?" : first_sample);
    }

    std::vector<std::string> defaultOutputNames(const std::vector<DiscoveredFolder>& recordings, const fs::path& output_root)
    {
        std::vector<std::string> names;
        for (const auto& r : recordings)
        {
            const std::string base = sanitizedName(r.name);
            const std::string fingerprint = recordingFingerprint(classifyRecordingFiles(r.path));
            auto usable = [&](const std::string& candidate)
            {
                for (const auto& n : names)
                    if (sameOutputName(n, candidate))
                        return false;
                if (output_root.empty())
                    return true;
                const fs::path folder = output_root / candidate;
                std::error_code ec;
                if (!fs::exists(folder, ec) || fs::is_empty(folder, ec) || holdsUnfinishedOutput(folder))
                    return true;
                const auto marker = readDoneMarker(folder);
                return marker && marker->fingerprint == fingerprint;
            };
            std::string candidate = base;
            for (int i = 2; !usable(candidate); i++)
                candidate = base + "_" + std::to_string(i);
            names.push_back(candidate);
        }
        return names;
    }

    bool writeDoneMarker(const fs::path& output_folder, const DoneMarker& m, std::string& error)
    {
        const fs::path file = OutputLayout(output_folder).done_marker;
        std::ofstream out(file, std::ios::binary);
        out << "# Easy Forest HDMapping: this folder holds a finished result. Batch mode skips the recording when the\n"
               "# same batch is started again; delete this file to process it again.\n"
            << "version=" << kEasyForestVersion << "\n"
            << "recording=" << m.recording << "\n"
            << "fingerprint=" << m.fingerprint << "\n"
            << "finished=" << m.finished << "\n"
            << "duration_s=" << formatSeconds(m.duration_s) << "\n"
            << "raw_points=" << m.raw_points << "\n"
            << "exported_points=" << m.exported_points << "\n";
        out.close();
        if (!out)
        {
            error = "Could not write '" + file.string() + "'.";
            return false;
        }
        return true;
    }

    std::optional<DoneMarker> readDoneMarker(const fs::path& output_folder)
    {
        std::ifstream in(OutputLayout(output_folder).done_marker, std::ios::binary);
        if (!in)
            return std::nullopt;
        DoneMarker m;
        std::string line;
        while (std::getline(in, line))
        {
            if (!line.empty() && line.back() == '\r')
                line.pop_back();
            const auto eq = line.find('=');
            if (line.empty() || line[0] == '#' || eq == std::string::npos)
                continue;
            const std::string key = line.substr(0, eq), value = line.substr(eq + 1);
            if (key == "recording")
                m.recording = value;
            else if (key == "fingerprint")
                m.fingerprint = value;
            else if (key == "finished")
                m.finished = value;
            else if (key == "duration_s")
                m.duration_s = std::atof(value.c_str());
            else if (key == "raw_points")
                m.raw_points = std::strtoull(value.c_str(), nullptr, 10);
            else if (key == "exported_points")
                m.exported_points = std::strtoull(value.c_str(), nullptr, 10);
        }
        if (m.fingerprint.empty())
            return std::nullopt;
        return m;
    }

    BatchDiskNeed batchDiskNeed(const std::vector<const PreflightResult*>& recordings)
    {
        BatchDiskNeed need;
        for (const PreflightResult* r : recordings)
        {
            need.work_bytes = std::max(need.work_bytes, r->estimated_work_bytes);
            need.result_bytes += r->estimated_result_bytes;
        }
        return need;
    }

    Finding batchDiskFinding(const BatchDiskNeed& need, bool space_known, uint64_t available)
    {
        const std::string text = "about " + formatBytes(need.total()) + " (" + formatBytes(need.work_bytes) +
            " temporary for the largest recording + " + formatBytes(need.result_bytes) + " of results for all of them)";
        if (!space_known)
            return { Severity::Warning, "Could not determine the free disk space. The batch needs " + text + "." };
        if (available < need.total())
            return { Severity::Error,
                     "Not enough free disk space for the batch: " + text + " needed, " + formatBytes(available) +
                         " free. Free some space, choose an output folder on another disk, or select fewer recordings." };
        if (available < need.total() + need.total() / 2)
            return { Severity::Warning,
                     "Free disk space is tight for the batch: " + text + " needed, " + formatBytes(available) + " free." };
        return { Severity::Ok, "Disk space for the batch: " + text + " needed, " + formatBytes(available) + " free." };
    }

    std::string batchOutputProblem(const fs::path& output_root, const std::vector<DiscoveredFolder>& folders)
    {
        if (output_root.empty())
            return "Choose an output folder.";
        for (const auto& f : folders)
            if (f.is_recording && isSameOrInside(output_root, f.path))
                return "The output folder is inside the recording '" + f.name +
                    "'. Choose a folder elsewhere: Easy Forest HDMapping never writes into a recording.";
        return {};
    }

    BatchCheck checkBatchRecording(const fs::path& recording, const fs::path& output_folder)
    {
        BatchCheck check;
        check.preflight = runPreflight(recording, output_folder, true);
        check.fingerprint = recordingFingerprint(check.preflight.files);
        if (auto marker = readDoneMarker(output_folder); marker && marker->fingerprint == check.fingerprint)
            check.done = marker;
        return check;
    }

    std::string firstError(const PreflightResult& result)
    {
        for (const auto& f : result.findings)
            if (f.severity == Severity::Error)
                return f.message;
        return {};
    }

    const char* batchStateToken(BatchItemState state)
    {
        switch (state)
        {
        case BatchItemState::Excluded:
            return "excluded";
        case BatchItemState::Pending:
            return "pending";
        case BatchItemState::Running:
            return "running";
        case BatchItemState::Succeeded:
            return "succeeded";
        case BatchItemState::Failed:
            return "failed";
        case BatchItemState::Cancelled:
            return "cancelled";
        case BatchItemState::AlreadyDone:
            return "already_done";
        case BatchItemState::NotRun:
            return "not_run";
        }
        return "";
    }

    double pipelineFraction(const PipelineStatus& s)
    {
        constexpr double kOdometryShare = 0.9;
        if (s.state == PipelineState::Succeeded)
            return 1.0;
        switch (s.stage)
        {
        case PipelineStage::Checking:
        case PipelineStage::Staging:
            return 0.0;
        case PipelineStage::Odometry:
            return kOdometryShare * std::clamp(s.step1.overallFraction(), 0.0, 1.0);
        case PipelineStage::Export:
            return kOdometryShare + (1.0 - kOdometryShare) * std::clamp(s.export_fraction, 0.0, 1.0);
        case PipelineStage::Cleanup:
            return s.session_folder.empty() ? 0.0 : 1.0;
        }
        return 0.0;
    }

    // ------------------------------------------------------------------------------------------------------------

    BatchRunner::BatchRunner(RecordingJobFactory factory)
        : factory_(std::move(factory))
    {
        if (!factory_)
            factory_ = []
            {
                return std::make_unique<PipelineJob>();
            };
    }

    BatchRunner::~BatchRunner()
    {
        stopNow();
        wait();
    }

    bool BatchRunner::start(const BatchRequest& request, std::string& error)
    {
        if (busy_)
        {
            error = "A batch is already running.";
            return false;
        }
        wait();
        std::error_code ec;
        if (!fs::create_directories(request.output_root, ec) && ec)
        {
            error = "Could not create the output folder '" + request.output_root.string() + "' (" + ec.message() + ").";
            return false;
        }

        request_ = request;
        request_.settings.clamp();
        {
            std::lock_guard lock(mutex_);
            status_ = BatchStatus{};
            status_.running = true;
            status_.items.resize(request_.items.size());
            std::vector<uint64_t> known;
            for (const auto& item : request_.items)
                if (item.include && item.total_points > 0)
                    known.push_back(item.total_points);
            const double average =
                known.empty() ? 1.0 : static_cast<double>(std::accumulate(known.begin(), known.end(), uint64_t{ 0 })) / known.size();
            weights_.assign(request_.items.size(), 0.0);
            for (size_t i = 0; i < request_.items.size(); i++)
            {
                const auto& item = request_.items[i];
                status_.items[i].output_folder = request_.output_root / item.output_name;
                status_.items[i].raw_points = item.total_points;
                if (!item.include)
                {
                    status_.items[i].state = BatchItemState::Excluded;
                    status_.items[i].error = item.exclude_reason;
                }
                else
                    weights_[i] = item.total_points > 0 ? static_cast<double>(item.total_points) : average;
            }
            processing_s_ = 0.0;
            processed_weight_ = 0.0;
            log_tail_.clear();
            started_ = std::chrono::steady_clock::now();
        }
        busy_ = true;
        thread_ = std::thread(&BatchRunner::run, this);
        return true;
    }

    void BatchRunner::stopAfterCurrent()
    {
        std::lock_guard lock(mutex_);
        status_.stop_after_current = true;
    }

    void BatchRunner::stopNow()
    {
        std::lock_guard lock(mutex_);
        status_.stop_after_current = true;
        status_.stop_now = true;
        if (job_)
            job_->cancel();
    }

    void BatchRunner::wait()
    {
        if (thread_.joinable())
            thread_.join();
    }

    BatchStatus BatchRunner::status() const
    {
        std::lock_guard lock(mutex_);
        BatchStatus s = status_;
        if (s.running)
            s.elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
        return s;
    }

    std::vector<std::string> BatchRunner::logTail() const
    {
        std::lock_guard lock(mutex_);
        return { log_tail_.begin(), log_tail_.end() };
    }

    std::vector<std::string> BatchRunner::currentLogTail() const
    {
        std::lock_guard lock(mutex_);
        return job_ ? job_->logTail() : std::vector<std::string>{};
    }

    void BatchRunner::log(const std::string& line)
    {
        const std::string stamped = "[" + timestamp() + "] " + line;
        if (log_file_.is_open())
            log_file_ << stamped << std::endl; // flushed: the log must survive a crash or power loss mid-batch
        std::lock_guard lock(mutex_);
        log_tail_.push_back(stamped);
        while (log_tail_.size() > kLogTailLines)
            log_tail_.pop_front();
    }

    void BatchRunner::updateProgress()
    {
        const double total = std::accumulate(weights_.begin(), weights_.end(), 0.0);
        double done = 0.0;
        for (size_t i = 0; i < weights_.size(); i++)
        {
            const BatchItemState st = status_.items[i].state;
            if (st != BatchItemState::Pending && st != BatchItemState::Running)
                done += weights_[i];
        }
        double current_done = 0.0, current_s = 0.0;
        if (status_.current >= 0)
        {
            current_done = weights_[status_.current] * pipelineFraction(status_.current_status);
            current_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - item_started_).count();
        }
        status_.fraction = total > 0.0 ? std::clamp((done + current_done) / total, 0.0, 1.0) : 1.0;

        // ETA from the processing rate of the recordings that really ran (already-done ones take no time).
        double remaining = 0.0;
        for (size_t i = 0; i < weights_.size(); i++)
            if (status_.items[i].state == BatchItemState::Pending)
                remaining += weights_[i];
        if (status_.current >= 0)
            remaining += weights_[status_.current] - current_done;
        const double processed = processed_weight_ + current_done;
        status_.eta_s = (total > 0.0 && processed > 0.02 * total) ? (processing_s_ + current_s) * remaining / processed : -1.0;
    }

    void BatchRunner::run()
    {
        // Several hours of unattended processing: the computer must not go to sleep in between.
        SleepBlocker no_sleep("Easy Forest HDMapping is processing recordings");

        log_file_.open(request_.output_root / kLogFileName, std::ios::out | std::ios::app);
        log("batch started: " + versionString());
        log("output folder: " + request_.output_root.string());
        log("lidar odometry program: " + request_.step1_executable.string());
        {
            std::ostringstream s;
            s << "settings: near_min_range_m=" << request_.settings.near_min_range_m
              << " place_by_raw_time=" << request_.settings.place_by_raw_time << " keep_work_files=" << request_.settings.keep_work_files;
            log(s.str());
        }
        log(std::string("preventing system sleep: ") + (no_sleep.active() ? "yes" : "no (not supported or refused)"));
        size_t selected = 0;
        for (const auto& item : request_.items)
            selected += item.include ? 1 : 0;
        log(std::to_string(request_.items.size()) + " recordings found, " + std::to_string(selected) + " selected");
        for (const auto& item : request_.items)
            if (!item.include)
                log("excluded: " + item.name + (item.exclude_reason.empty() ? "" : " (" + item.exclude_reason + ")"));
        for (const auto& folder : request_.skipped)
            log("skipped: " + folder.name + " (" + folder.skip_reason + ")");
        writeSummary();

        for (size_t i = 0; i < request_.items.size(); i++)
        {
            if (!request_.items[i].include)
                continue;
            bool stop = false;
            {
                std::lock_guard lock(mutex_);
                stop = status_.stop_after_current;
                if (stop)
                    status_.items[i].state = BatchItemState::NotRun;
            }
            if (!stop)
                processItem(i);
            writeSummary();
        }

        BatchStatus final_status = status();
        size_t counts[8] = {};
        for (const auto& r : final_status.items)
            counts[static_cast<int>(r.state)]++;
        std::ostringstream summary;
        summary << "batch " << (final_status.stop_after_current ? "stopped" : "finished") << " after "
                << formatSeconds(final_status.elapsed_s) << " s: " << counts[static_cast<int>(BatchItemState::Succeeded)] << " succeeded, "
                << counts[static_cast<int>(BatchItemState::Failed)] << " failed, " << counts[static_cast<int>(BatchItemState::AlreadyDone)]
                << " already done, " << counts[static_cast<int>(BatchItemState::Cancelled)] << " cancelled, "
                << counts[static_cast<int>(BatchItemState::NotRun)] << " not run, " << counts[static_cast<int>(BatchItemState::Excluded)]
                << " excluded";
        log(summary.str());
        log_file_.close();
        {
            std::lock_guard lock(mutex_);
            status_.elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
            status_.running = false;
            status_.current = -1;
            updateProgress();
        }
        busy_ = false;
    }

    void BatchRunner::processItem(size_t index)
    {
        const BatchItem& item = request_.items[index];
        const fs::path out = request_.output_root / item.output_name;
        {
            std::lock_guard lock(mutex_);
            status_.current = static_cast<int>(index);
            status_.current_status = PipelineStatus{};
            status_.items[index].state = BatchItemState::Running;
            item_started_ = std::chrono::steady_clock::now();
        }
        log("recording " + item.name + ": " + item.recording.string() + " -> " + out.string());

        auto finishItem = [&](BatchItemState state, const std::string& error, bool ran)
        {
            std::lock_guard lock(mutex_);
            BatchItemResult& r = status_.items[index];
            r.state = state;
            r.error = error;
            if (ran)
            {
                r.duration_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - item_started_).count();
                processing_s_ += r.duration_s;
                processed_weight_ += weights_[index];
            }
            status_.current = -1;
            updateProgress();
        };

        const BatchCheck check = checkBatchRecording(item.recording, out);
        {
            std::lock_guard lock(mutex_);
            status_.items[index].raw_points = check.preflight.total_points;
        }
        if (check.done)
        {
            {
                std::lock_guard lock(mutex_);
                status_.items[index].duration_s = check.done->duration_s;
                status_.items[index].exported_points = check.done->exported_points;
            }
            finishItem(BatchItemState::AlreadyDone, "", false);
            log("recording " + item.name + ": already done (finished " + check.done->finished + "), skipped");
            return;
        }
        if (!check.preflight.ok())
        {
            const std::string error = firstError(check.preflight);
            finishItem(BatchItemState::Failed, error, false);
            log("recording " + item.name + ": FAILED the check: " + error);
            return;
        }

        std::unique_ptr<RecordingJob> job = factory_();
        PipelineRequest request{ item.recording, out, request_.step1_executable, request_.settings, request_.check_band_outer_m };
        request.resume_unfinished = true;
        std::string error;
        if (!job->start(request, error))
        {
            finishItem(BatchItemState::Failed, error, true);
            log("recording " + item.name + ": FAILED to start: " + error);
            return;
        }
        {
            std::lock_guard lock(mutex_);
            job_ = job.get();
            if (status_.stop_now) // stopNow() may have run before job_ was published
                job_->cancel();
        }
        PipelineStage last_stage = PipelineStage::Checking;
        while (job->busy())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            const PipelineStatus s = job->status();
            if (s.stage != last_stage && s.state == PipelineState::Running)
            {
                static const char* kStageNames[] = { "checking", "preparing", "lidar odometry", "export", "cleaning up" };
                log("recording " + item.name + ": " + kStageNames[static_cast<int>(s.stage)]);
                last_stage = s.stage;
            }
            std::lock_guard lock(mutex_);
            status_.current_status = s;
            updateProgress();
        }
        const PipelineStatus s = job->status();
        {
            std::lock_guard lock(mutex_);
            job_ = nullptr;
            status_.current_status = s;
        }
        job.reset();

        if (s.state == PipelineState::Succeeded)
        {
            const ExportStats& e = s.export_stats;
            const uint64_t exported = e.odometry_points + e.near_points + e.gap_points + e.check_band_points;
            DoneMarker marker{
                item.recording.string(), check.fingerprint, timestamp(), s.elapsed_s, check.preflight.total_points, exported
            };
            std::string marker_error;
            const bool marked = writeDoneMarker(out, marker, marker_error);
            {
                std::lock_guard lock(mutex_);
                status_.items[index].exported_points = exported;
            }
            finishItem(BatchItemState::Succeeded, marked ? "" : marker_error, true);
            log("recording " + item.name + ": succeeded in " + formatSeconds(s.elapsed_s) + " s, " + std::to_string(exported) +
                " points exported" + (marked ? "" : "; " + marker_error));
        }
        else if (s.state == PipelineState::Cancelled)
        {
            finishItem(BatchItemState::Cancelled, s.message, true);
            log("recording " + item.name + ": cancelled");
        }
        else
        {
            const std::string message = s.message.empty() ? "Processing failed." : s.message;
            finishItem(BatchItemState::Failed, message, true);
            log("recording " + item.name + ": FAILED: " + message + (s.detail.empty() ? "" : " (" + s.detail + ")"));
        }
    }

    void BatchRunner::writeSummary()
    {
        const BatchStatus s = status();
        const fs::path file = request_.output_root / kSummaryFileName;
        const fs::path temporary = request_.output_root / (std::string(kSummaryFileName) + ".tmp");
        {
            std::ofstream out(temporary, std::ios::binary);
            out << "recording,output_name,status,duration_s,raw_points,exported_points,output_path,error\n";
            for (size_t i = 0; i < request_.items.size(); i++)
            {
                const BatchItem& item = request_.items[i];
                const BatchItemResult& r = s.items[i];
                std::error_code ec;
                const bool has_output = r.state != BatchItemState::Excluded && fs::exists(r.output_folder, ec);
                out << csvField(item.recording.string()) << ',' << csvField(item.output_name) << ',' << batchStateToken(r.state) << ','
                    << (r.duration_s > 0.0 ? formatSeconds(r.duration_s) : "") << ',' << (r.raw_points ? std::to_string(r.raw_points) : "")
                    << ',' << (r.exported_points ? std::to_string(r.exported_points) : "") << ','
                    << csvField(has_output ? r.output_folder.string() : "") << ',' << csvField(r.error) << '\n';
            }
            for (const auto& folder : request_.skipped)
                out << csvField(folder.path.string()) << ",,skipped,,,,," << csvField(folder.skip_reason) << '\n';
            out.close();
            if (!out)
            {
                log("WARNING: could not write " + temporary.string());
                return;
            }
        }
        std::error_code ec;
        fs::rename(temporary, file, ec);
        if (ec) // e.g. the summary is open in a spreadsheet program on Windows
            log("WARNING: could not update " + file.string() + " (" + ec.message() + ")");
    }
} // namespace easy_gui
