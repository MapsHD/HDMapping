// HDMapping Easy GUI: raw Mandeye recording -> lidar odometry (precision forestry preset) -> point cloud + trajectory.
// See apps/easy_gui/docs/DESIGN.md for the decisions behind the pipeline.

#include "batch.h"
#include "child_process.h"
#include "export.h"
#include "pipeline.h"
#include "preflight.h"
#include "settings.h"
#include "version.h"

#include <Core/pfd_wrapper.hpp>
#include <RaylibWidgets/WindowFit.h>

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>
#include <raylib.h>
#include <rlImGui.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <future>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using namespace easy_gui;

namespace
{
    const ImVec4 kGreen(0.35f, 0.80f, 0.40f, 1.0f);
    const ImVec4 kAmber(0.95f, 0.70f, 0.20f, 1.0f);
    const ImVec4 kRed(0.95f, 0.35f, 0.35f, 1.0f);
    const ImVec4 kGrey(0.60f, 0.60f, 0.60f, 1.0f);

    fs::path homeFolder()
    {
#ifdef _WIN32
        const char* home = std::getenv("USERPROFILE");
#else
        const char* home = std::getenv("HOME");
#endif
        return home ? fs::path(home) : fs::current_path();
    }

    //! ~/Documents/Easy Forest HDMapping/<recording name>, with _2, _3, ... appended if that already exists. Never next to
    //! the recording: that may be the scanner's USB stick.
    fs::path defaultOutputFolder(const fs::path& recording)
    {
        const fs::path base = homeFolder() / "Documents" / "Easy Forest HDMapping";
        const std::string name = recording.filename().empty() ? recording.parent_path().filename().string() : recording.filename().string();
        std::error_code ec;
        fs::path candidate = base / name;
        for (int i = 2; fs::exists(candidate, ec); i++)
            candidate = base / (name + "_" + std::to_string(i));
        return candidate;
    }

    std::string formatDuration(double seconds)
    {
        const int s = static_cast<int>(seconds + 0.5);
        char buffer[32];
        if (s >= 3600)
            std::snprintf(buffer, sizeof(buffer), "%dh %02dm", s / 3600, (s / 60) % 60);
        else
            std::snprintf(buffer, sizeof(buffer), "%dm %02ds", s / 60, s % 60);
        return buffer;
    }

    void wrappedColored(const ImVec4& color, const std::string& text)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TextWrapped("%s", text.c_str());
        ImGui::PopStyleColor();
    }

    std::string folderName(const fs::path& folder)
    {
        const fs::path normal = folder.lexically_normal();
        return normal.filename().empty() ? normal.parent_path().filename().string() : normal.filename().string();
    }

    //! ~/Documents/Easy Forest HDMapping/<parent folder name>. Unlike single mode, an existing folder from an earlier
    //! batch is reused (restarting the same batch continues it); _2, _3, ... only when the folder holds something else.
    fs::path defaultBatchOutputFolder(const fs::path& parent)
    {
        const fs::path base = homeFolder() / "Documents" / "Easy Forest HDMapping";
        const std::string name = folderName(parent);
        std::error_code ec;
        fs::path candidate = base / name;
        for (int i = 2;
             fs::exists(candidate, ec) && !fs::is_empty(candidate, ec) && !fs::exists(candidate / BatchRunner::kSummaryFileName, ec);
             i++)
            candidate = base / (name + "_" + std::to_string(i));
        return candidate;
    }

    //! Checks the batch table's rows one at a time on a worker thread: they read the recordings, which may be on a
    //! slow USB stick, and the window must stay responsive. Each result carries the generation it was requested for,
    //! so results for a parent folder the user has since replaced are dropped.
    class RowChecker
    {
    public:
        struct Result
        {
            size_t row;
            int generation;
            fs::path output_folder; // the check was made against this output folder
            BatchCheck check;
        };

        RowChecker()
        {
            thread_ = std::thread(&RowChecker::run, this);
        }
        ~RowChecker()
        {
            {
                std::lock_guard lock(mutex_);
                stop_ = true;
            }
            cv_.notify_all();
            thread_.join();
        }

        void request(size_t row, int generation, const fs::path& recording, const fs::path& output_folder)
        {
            std::lock_guard lock(mutex_);
            std::erase_if(
                queue_,
                [&](const Job& j)
                {
                    return j.generation != generation || j.row == row;
                });
            queue_.push_back({ row, generation, recording, output_folder });
            cv_.notify_all();
        }

        std::vector<Result> take()
        {
            std::lock_guard lock(mutex_);
            return std::exchange(results_, {});
        }

    private:
        struct Job
        {
            size_t row;
            int generation;
            fs::path recording;
            fs::path output_folder;
        };

        void run()
        {
            std::unique_lock lock(mutex_);
            while (true)
            {
                cv_.wait(
                    lock,
                    [&]
                    {
                        return stop_ || !queue_.empty();
                    });
                if (stop_)
                    return;
                const Job job = queue_.front();
                queue_.pop_front();
                lock.unlock();
                BatchCheck check = checkBatchRecording(job.recording, job.output_folder);
                lock.lock();
                results_.push_back({ job.row, job.generation, job.output_folder, std::move(check) });
            }
        }

        std::mutex mutex_;
        std::condition_variable cv_;
        std::deque<Job> queue_;
        std::vector<Result> results_;
        bool stop_ = false;
        std::thread thread_; // last: starts after the members above exist
    };

    struct BatchRow
    {
        DiscoveredFolder folder;
        std::string output_name;
        bool name_edited = false; // keep the user's name when the output folder changes
        bool selected = true; // the user's choice; a recording that fails the check is never included
        std::optional<BatchCheck> check;
        fs::path checked_output;

        bool checkCurrent(const fs::path& output_root) const
        {
            return check && checked_output == output_root / output_name;
        }
        bool usable() const
        {
            return check && (check->done || check->preflight.ok());
        }
        bool included() const
        {
            return selected && usable();
        }
        //! Will write into its output folder, as far as is known: included, or selected and not checked yet.
        bool active(const fs::path& output_root) const
        {
            return selected && (!checkCurrent(output_root) || usable());
        }
    };

    struct BatchUi
    {
        fs::path parent;
        fs::path output_root;
        bool output_chosen_by_user = false;
        std::vector<DiscoveredFolder> found;
        std::vector<BatchRow> rows; // the recordings
        std::vector<DiscoveredFolder> skipped; // the other folders
        int generation = 0;
        RowChecker checker;

        bool disk_known = false;
        uint64_t available = 0;
        double disk_checked_at = -1e9;

        BatchRunner runner;
        bool started = false; // the runner was started; the table shows the status column until "new batch"
        std::string start_error;

        void setParent(const fs::path& folder)
        {
            parent = folder;
            found = discoverRecordings(folder);
            rows.clear();
            skipped.clear();
            std::vector<DiscoveredFolder> recordings;
            for (const auto& f : found)
                (f.is_recording ? recordings : skipped).push_back(f);
            if (!output_chosen_by_user)
                output_root = defaultBatchOutputFolder(folder);
            const auto names = defaultOutputNames(recordings, output_root);
            for (size_t i = 0; i < recordings.size(); i++)
                rows.push_back({ recordings[i], names[i] });
            generation++;
            checkAll();
        }

        void setOutputRoot(const fs::path& folder)
        {
            output_root = folder;
            output_chosen_by_user = true;
            std::vector<DiscoveredFolder> recordings;
            for (const auto& r : rows)
                recordings.push_back(r.folder);
            const auto names = defaultOutputNames(recordings, output_root);
            for (size_t i = 0; i < rows.size(); i++)
                if (!rows[i].name_edited)
                    rows[i].output_name = names[i];
            checkAll();
        }

        void checkRow(size_t i)
        {
            if (outputNameProblem(rows[i].output_name).empty())
                checker.request(i, generation, rows[i].folder.path, output_root / rows[i].output_name);
        }

        void checkAll()
        {
            for (size_t i = 0; i < rows.size(); i++)
                checkRow(i);
            disk_checked_at = -1e9;
        }

        void poll()
        {
            for (auto& r : checker.take())
                if (r.generation == generation && r.row < rows.size())
                {
                    rows[r.row].check = std::move(r.check);
                    rows[r.row].checked_output = r.output_folder;
                    disk_checked_at = -1e9;
                }
            // Free space changes while other programs run; refreshed every few seconds before the start.
            if (!runner.busy() && !started && ImGui::GetTime() - disk_checked_at > 3.0 && !output_root.empty())
            {
                fs::path checked;
                disk_known = availableSpace(output_root, available, checked);
                disk_checked_at = ImGui::GetTime();
            }
        }

        bool checking() const
        {
            for (const auto& r : rows)
                if (!r.checkCurrent(output_root) && outputNameProblem(r.output_name).empty())
                    return true;
            return false;
        }

        //! Why row `i`'s output name cannot be used, or empty.
        std::string nameProblem(size_t i) const
        {
            if (std::string p = outputNameProblem(rows[i].output_name); !p.empty())
                return p;
            for (size_t j = 0; j < rows.size(); j++)
                if (j != i && rows[j].active(output_root) && sameOutputName(rows[i].output_name, rows[j].output_name))
                    return "Another recording uses the same output folder name.";
            return {};
        }

        std::vector<const PreflightResult*> toProcess() const
        {
            std::vector<const PreflightResult*> out;
            for (const auto& r : rows)
                if (r.included() && r.checkCurrent(output_root) && !r.check->done)
                    out.push_back(&r.check->preflight);
            return out;
        }

        Finding diskFinding() const
        {
            return batchDiskFinding(batchDiskNeed(toProcess()), disk_known, available);
        }

        bool readyToStart() const
        {
            if (runner.busy() || started || rows.empty() || checking() || !batchOutputProblem(output_root, found).empty())
                return false;
            bool any = false;
            for (size_t i = 0; i < rows.size(); i++)
            {
                if (!rows[i].active(output_root))
                    continue;
                if (!nameProblem(i).empty())
                    return false;
                any = any || rows[i].included();
            }
            return any && diskFinding().severity != Severity::Error;
        }

        BatchRequest makeRequest(const Settings& settings) const
        {
            BatchRequest request;
            request.output_root = output_root;
            request.skipped = skipped;
            request.step1_executable = findStep1Executable();
            request.settings = settings;
            for (const auto& r : rows)
            {
                BatchItem item{ r.folder.path, r.folder.name, r.output_name, r.included() };
                item.total_points = r.check ? r.check->preflight.total_points : 0;
                if (!r.selected)
                    item.exclude_reason = "Not selected.";
                else if (!r.usable())
                    item.exclude_reason = "Check failed: " + firstError(r.check->preflight);
                request.items.push_back(std::move(item));
            }
            return request;
        }

        void reset()
        {
            runner.wait();
            parent.clear();
            found.clear();
            rows.clear();
            skipped.clear();
            generation++;
            if (!output_chosen_by_user)
                output_root.clear();
            started = false;
            start_error.clear();
        }
    };

    enum class Mode
    {
        Single,
        Batch
    };

    struct App
    {
        Mode mode = Mode::Single;
        BatchUi batch;

        fs::path recording;
        fs::path output_folder;
        bool output_chosen_by_user = false;
        Settings settings;

        std::future<PreflightResult> preflight_future;
        std::optional<PreflightResult> preflight;
        fs::path preflight_for_recording;
        fs::path preflight_for_output;

        Pipeline pipeline;
        std::string start_error;
        bool show_log = false;

        void setRecording(const fs::path& folder)
        {
            recording = folder;
            if (!output_chosen_by_user)
                output_folder = defaultOutputFolder(folder);
            startPreflight();
        }

        void startPreflight()
        {
            if (recording.empty())
                return;
            preflight.reset();
            preflight_for_recording = recording;
            preflight_for_output = output_folder;
            preflight_future = std::async(std::launch::async, runPreflight, recording, output_folder, false);
        }

        bool checking() const
        {
            return preflight_future.valid();
        }

        void poll()
        {
            if (preflight_future.valid() && preflight_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
                preflight = preflight_future.get();
        }

        bool readyToStart() const
        {
            return preflight && preflight->ok() && preflight_for_recording == recording && preflight_for_output == output_folder &&
                !pipeline.busy();
        }
    };

    void drawAdvancedSettings(Settings& settings)
    {
        if (ImGui::CollapsingHeader("Advanced settings"))
        {
            ImGui::Indent();
            float near = static_cast<float>(settings.near_min_range_m);
            ImGui::SetNextItemWidth(220);
            if (ImGui::SliderFloat(
                    "Closest points kept [m]",
                    &near,
                    static_cast<float>(Settings::kNearMinRangeLowest),
                    static_cast<float>(Settings::kNearMinRangeHighest),
                    "%.2f m"))
            {
                settings.near_min_range_m = near;
                settings.clamp();
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Points closer to the scanner than this are left out of the point cloud.\n"
                    "0.4 m (default) removes the operator's body and the handle.\n"
                    "Lower values keep more of the close surroundings but may include the operator.");
            ImGui::Checkbox("Place all points by their measurement time (experimental)", &settings.place_by_raw_time);
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Off (default): the point cloud uses lidar odometry's own points.\n"
                    "On: every point is taken from the raw recording and placed at its exact measurement time.");
            ImGui::Checkbox("Keep intermediate files (for troubleshooting)", &settings.keep_work_files);
            ImGui::Unindent();
        }
    }

    void drawInputs(App& app, bool locked)
    {
        ImGui::BeginDisabled(locked);

        ImGui::SeparatorText("1. Recording");
        ImGui::TextWrapped("The folder the Mandeye scanner wrote (lidar*.laz and imu*.csv files). You can also drag it onto this window.");
        if (ImGui::Button("Choose recording..."))
        {
            const std::string folder = mandeye::fd::SelectFolder("Choose the Mandeye recording folder");
            if (!folder.empty())
                app.setRecording(folder);
        }
        ImGui::SameLine();
        ImGui::TextUnformatted(app.recording.empty() ? "(none chosen)" : app.recording.string().c_str());

        ImGui::SeparatorText("2. Output folder");
        ImGui::TextWrapped(
            "Results go here. It must be a new or empty folder outside the recording; nothing is ever written into the recording.");
        if (ImGui::Button("Choose output folder..."))
        {
            const std::string folder = mandeye::fd::SelectFolder("Choose where to save the results");
            if (!folder.empty())
            {
                app.output_folder = folder;
                app.output_chosen_by_user = true;
                app.startPreflight();
            }
        }
        ImGui::SameLine();
        ImGui::TextUnformatted(
            app.output_folder.empty() ? "(set automatically once a recording is chosen)" : app.output_folder.string().c_str());

        drawAdvancedSettings(app.settings);

        ImGui::EndDisabled();
    }

    void drawPreflight(App& app)
    {
        ImGui::SeparatorText("3. Check");
        if (app.recording.empty())
        {
            ImGui::TextDisabled("Choose a recording first.");
            return;
        }
        if (app.checking())
        {
            ImGui::TextUnformatted("Checking the recording...");
            return;
        }
        if (!app.preflight)
        {
            if (ImGui::Button("Check recording"))
                app.startPreflight();
            return;
        }
        if (app.preflight_for_recording != app.recording || app.preflight_for_output != app.output_folder)
        {
            if (ImGui::Button("Check again"))
                app.startPreflight();
            return;
        }
        for (const auto& f : app.preflight->findings)
        {
            const char* tag = f.severity == Severity::Error ? "[X]" : f.severity == Severity::Warning ? "[!]" : "[OK]";
            const ImVec4& color = f.severity == Severity::Error ? kRed : f.severity == Severity::Warning ? kAmber : kGreen;
            ImGui::TextColored(color, "%s", tag);
            ImGui::SameLine();
            ImGui::TextWrapped("%s", f.message.c_str());
        }
        if (!app.preflight->ok() && ImGui::Button("Check again"))
            app.startPreflight();
    }

    void stageRow(const char* label, int index, int current, bool failed)
    {
        if (index < current)
            ImGui::TextColored(kGreen, "[done]");
        else if (index == current)
            ImGui::TextColored(failed ? kRed : kAmber, failed ? "[failed]" : "[ .. ]");
        else
            ImGui::TextColored(kGrey, "[    ]");
        ImGui::SameLine();
        ImGui::TextUnformatted(label);
    }

    //! What the running pipeline is doing right now: a label and a progress bar.
    void drawStageProgress(const PipelineStatus& s)
    {
        if (s.stage == PipelineStage::Odometry)
        {
            const double phase = s.step1.phaseFraction();
            std::string label = phaseLabel(s.step1.phase);
            if (s.step1.phase == Step1Phase::Odometry && s.step1.chunks_total > 0)
                label += " (" + std::to_string(s.step1.chunks_done) + " / " + std::to_string(s.step1.chunks_total) + ")";
            ImGui::TextUnformatted(label.c_str());
            if (phase >= 0.0)
                ImGui::ProgressBar(static_cast<float>(phase), ImVec2(-1, 0));
            else
                ImGui::ProgressBar(-1.0f * static_cast<float>(ImGui::GetTime()), ImVec2(-1, 0), "working...");
        }
        else if (s.stage == PipelineStage::Export)
        {
            ImGui::TextUnformatted(s.export_step.empty() ? "Exporting..." : s.export_step.c_str());
            ImGui::ProgressBar(static_cast<float>(s.export_fraction), ImVec2(-1, 0));
        }
        else
        {
            ImGui::TextUnformatted(s.stage == PipelineStage::Cleanup ? "Cleaning up..." : "Preparing...");
            ImGui::ProgressBar(-1.0f * static_cast<float>(ImGui::GetTime()), ImVec2(-1, 0), "working...");
        }
    }

    void drawRun(App& app)
    {
        const PipelineStatus s = app.pipeline.status();
        ImGui::SeparatorText("4. Process");

        if (s.state == PipelineState::Idle)
        {
            ImGui::BeginDisabled(!app.readyToStart());
            if (ImGui::Button("Start processing", ImVec2(220, 0)))
            {
                PipelineRequest request{ app.recording, app.output_folder, findStep1Executable(), app.settings };
                app.start_error.clear();
                app.pipeline.start(request, app.start_error);
            }
            ImGui::EndDisabled();
            if (!app.start_error.empty())
                wrappedColored(kRed, app.start_error);
            return;
        }

        // Stage list. Stage indices: 0 check+prepare, 1 odometry, 2 export, 3 = all done.
        const bool failed = s.state == PipelineState::Failed;
        int current = 0;
        if (s.stage == PipelineStage::Odometry)
            current = 1;
        if (s.stage == PipelineStage::Export || (s.stage == PipelineStage::Cleanup && !s.session_folder.empty()))
            current = 2;
        if (s.state == PipelineState::Succeeded)
            current = 3;
        stageRow("Check and prepare the recording", 0, current, failed && current == 0);
        stageRow("Lidar odometry (precision forestry settings)", 1, current, failed && current == 1);
        stageRow("Export point cloud and trajectory", 2, current, failed && current == 2);

        if (s.state == PipelineState::Running)
        {
            ImGui::Spacing();
            drawStageProgress(s);
            if (s.stage == PipelineStage::Odometry)
            {
                const double overall = s.step1.overallFraction();
                std::string eta = "Elapsed " + formatDuration(s.elapsed_s);
                if (overall > 0.05)
                    eta += ", about " + formatDuration(s.elapsed_s * (1.0 - overall) / overall) + " left";
                ImGui::TextUnformatted("Overall");
                ImGui::ProgressBar(static_cast<float>(overall), ImVec2(-1, 0), eta.c_str());
            }
            else if (s.stage == PipelineStage::Export)
                ImGui::TextUnformatted(("Elapsed " + formatDuration(s.elapsed_s)).c_str());

            ImGui::Spacing();
            if (ImGui::Button("Cancel", ImVec2(120, 0)))
                ImGui::OpenPopup("Cancel processing?");
            if (ImGui::BeginPopupModal("Cancel processing?", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::TextUnformatted("Stop processing? The work done so far will be discarded.");
                if (ImGui::Button("Yes, stop", ImVec2(120, 0)))
                {
                    app.pipeline.cancel();
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("No, continue", ImVec2(120, 0)))
                    ImGui::CloseCurrentPopup();
                ImGui::EndPopup();
            }
            return;
        }

        ImGui::Spacing();
        if (s.state == PipelineState::Succeeded)
        {
            wrappedColored(kGreen, s.message + " (took " + formatDuration(s.elapsed_s) + ")");
            const OutputLayout out(app.output_folder);
            const ExportStats& e = s.export_stats;
            ImGui::BulletText(
                "%s: %.1f million points (%.1f million from lidar odometry, %.1f million near and %.1f million gap points "
                "added), %.0f MB",
                out.point_cloud.filename().string().c_str(),
                (e.odometry_points + e.near_points + e.gap_points + e.check_band_points) / 1e6,
                e.odometry_points / 1e6,
                e.near_points / 1e6,
                e.gap_points / 1e6,
                e.laz_bytes / 1e6);
            ImGui::BulletText("%s: %zu scanner poses", out.trajectory.filename().string().c_str(), e.trajectory_nodes);
            ImGui::BulletText("%s describes every field of both files.", out.readme.filename().string().c_str());
        }
        else if (s.state == PipelineState::Cancelled)
            wrappedColored(kAmber, s.message);
        else
        {
            wrappedColored(kRed, s.message);
            if (!s.detail.empty())
                ImGui::TextDisabled("Technical detail: %s", s.detail.c_str());
            ImGui::TextDisabled("The full log is in %s", OutputLayout(app.output_folder).log_file.string().c_str());
        }

        if (ImGui::Button("Open output folder"))
            OpenURL(app.output_folder.string().c_str());
        ImGui::SameLine();
        if (ImGui::Button("Process another recording"))
        {
            app.pipeline.reset();
            app.recording.clear();
            app.output_folder.clear();
            app.output_chosen_by_user = false;
            app.preflight.reset();
        }
    }

    void drawLog(App& app)
    {
        ImGui::Spacing();
        if (!ImGui::CollapsingHeader("Details (processing log)"))
            return;
        const auto lines = app.pipeline.logTail();
        ImGui::BeginChild("log", ImVec2(0, 240), ImGuiChildFlags_Borders, ImGuiWindowFlags_HorizontalScrollbar);
        for (const auto& line : lines)
            ImGui::TextUnformatted(line.c_str());
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 5.0f) // follow the end unless the user scrolled up
            ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Batch mode ("Several recordings")

    void drawBatchInputs(App& app, bool locked)
    {
        BatchUi& b = app.batch;
        ImGui::BeginDisabled(locked);

        ImGui::SeparatorText("1. Folder with recordings");
        ImGui::TextWrapped(
            "A folder that contains several recording folders, for example a copy of the scanner's USB stick. Recordings are looked "
            "for one folder level down. You can also drag the folder onto this window.");
        if (ImGui::Button("Choose folder..."))
        {
            const std::string folder = mandeye::fd::SelectFolder("Choose the folder that contains the recordings");
            if (!folder.empty())
                b.setParent(folder);
        }
        ImGui::SameLine();
        ImGui::TextUnformatted(b.parent.empty() ? "(none chosen)" : b.parent.string().c_str());

        ImGui::SeparatorText("2. Output folder");
        ImGui::TextWrapped(
            "Each recording's results go into a folder of its own in here, named in the table below. Choosing the same output folder "
            "again continues an interrupted batch: finished recordings are skipped.");
        if (ImGui::Button("Choose output folder..."))
        {
            const std::string folder = mandeye::fd::SelectFolder("Choose where to save the results");
            if (!folder.empty())
                b.setOutputRoot(folder);
        }
        ImGui::SameLine();
        ImGui::TextUnformatted(b.output_root.empty() ? "(set automatically once a folder is chosen)" : b.output_root.string().c_str());

        drawAdvancedSettings(app.settings);
        ImGui::EndDisabled();
    }

    void drawCheckCell(const BatchRow& row, bool current)
    {
        if (!current)
        {
            ImGui::TextDisabled(outputNameProblem(row.output_name).empty() ? "checking..." : "-");
            return;
        }
        const BatchCheck& c = *row.check;
        if (c.done)
        {
            ImGui::TextColored(kGreen, "already done");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Finished on %s. It will be skipped.", c.done->finished.c_str());
            return;
        }
        size_t warnings = 0;
        for (const auto& f : c.preflight.findings)
            warnings += f.severity == Severity::Warning ? 1 : 0;
        if (!c.preflight.ok())
            ImGui::TextColored(kRed, "[X] problem");
        else if (warnings > 0)
            ImGui::TextColored(kAmber, "[!] %zu note%s", warnings, warnings > 1 ? "s" : "");
        else
            ImGui::TextColored(kGreen, "[OK]");
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 40.0f);
            for (const auto& f : c.preflight.findings)
            {
                const char* tag = f.severity == Severity::Error ? "[X]" : f.severity == Severity::Warning ? "[!]" : "[OK]";
                ImGui::TextWrapped("%s %s", tag, f.message.c_str());
            }
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
    }

    void drawStatusCell(const BatchItemResult& r, bool current, const BatchStatus& s)
    {
        switch (r.state)
        {
        case BatchItemState::Excluded:
            ImGui::TextDisabled("not selected");
            break;
        case BatchItemState::Pending:
            ImGui::TextDisabled("waiting");
            break;
        case BatchItemState::Running:
            ImGui::TextColored(kAmber, "running %d%%", current ? static_cast<int>(pipelineFraction(s.current_status) * 100) : 0);
            break;
        case BatchItemState::Succeeded:
            ImGui::TextColored(kGreen, "done (%s)", formatDuration(r.duration_s).c_str());
            break;
        case BatchItemState::AlreadyDone:
            ImGui::TextColored(kGreen, "already done");
            break;
        case BatchItemState::Failed:
            ImGui::TextColored(kRed, "failed");
            break;
        case BatchItemState::Cancelled:
            ImGui::TextColored(kAmber, "stopped");
            break;
        case BatchItemState::NotRun:
            ImGui::TextDisabled("not run");
            break;
        }
        if (!r.error.empty() && ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 40.0f);
            ImGui::TextWrapped("%s", r.error.c_str());
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
    }

    void drawBatchTable(App& app, bool locked)
    {
        BatchUi& b = app.batch;
        ImGui::SeparatorText("3. Recordings");
        if (b.parent.empty())
        {
            ImGui::TextDisabled("Choose a folder first.");
            return;
        }
        if (b.rows.empty())
        {
            wrappedColored(
                kRed,
                "No recordings were found in this folder or in the folders directly inside it. A recording folder contains "
                "lidar*.laz and imu*.csv files.");
        }

        const BatchStatus s = b.runner.status();
        const bool show_status = b.started && s.items.size() == b.rows.size();
        if (!b.rows.empty())
        {
            const int columns = show_status ? 6 : 5;
            const float row_height = ImGui::GetFrameHeightWithSpacing();
            const float height = row_height * static_cast<float>(std::min<size_t>(b.rows.size(), 10) + 1) + 6.0f;
            if (ImGui::BeginTable(
                    "recordings",
                    columns,
                    ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp,
                    ImVec2(0, height)))
            {
                ImGui::TableSetupScrollFreeze(0, 1);
                ImGui::TableSetupColumn("Include", ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableSetupColumn("Recording", ImGuiTableColumnFlags_WidthStretch, 3.0f);
                ImGui::TableSetupColumn("Length", ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableSetupColumn("Check", ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableSetupColumn("Output folder name", ImGuiTableColumnFlags_WidthStretch, 3.0f);
                if (show_status)
                    ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthFixed);
                ImGui::TableHeadersRow();

                for (size_t i = 0; i < b.rows.size(); i++)
                {
                    BatchRow& row = b.rows[i];
                    const bool current = row.checkCurrent(b.output_root);
                    ImGui::PushID(static_cast<int>(i));
                    ImGui::TableNextRow();

                    ImGui::TableNextColumn();
                    ImGui::BeginDisabled(locked || (current && !row.usable()));
                    bool include = current ? row.included() : row.selected;
                    if (ImGui::Checkbox("##include", &include))
                        row.selected = include;
                    ImGui::EndDisabled();

                    ImGui::TableNextColumn();
                    ImGui::AlignTextToFramePadding();
                    ImGui::TextUnformatted(row.folder.name.c_str());
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("%s", row.folder.path.string().c_str());

                    ImGui::TableNextColumn();
                    if (row.check)
                        ImGui::TextUnformatted(formatDuration(row.check->preflight.duration_s).c_str());
                    else
                        ImGui::TextDisabled("...");

                    ImGui::TableNextColumn();
                    drawCheckCell(row, current);

                    ImGui::TableNextColumn();
                    const std::string problem = row.active(b.output_root) ? b.nameProblem(i) : std::string();
                    ImGui::BeginDisabled(locked);
                    if (!problem.empty())
                        ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.45f, 0.12f, 0.12f, 1.0f));
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    ImGui::InputText("##output_name", &row.output_name);
                    if (ImGui::IsItemDeactivatedAfterEdit())
                    {
                        row.name_edited = true;
                        b.checkRow(i);
                    }
                    if (!problem.empty())
                    {
                        ImGui::PopStyleColor();
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("%s", problem.c_str());
                    }
                    ImGui::EndDisabled();

                    if (show_status)
                    {
                        ImGui::TableNextColumn();
                        drawStatusCell(s.items[i], s.current == static_cast<int>(i), s);
                    }
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }
        }

        if (!b.started)
        {
            // Problems in words, below the table (the table only has room for a mark).
            for (size_t i = 0; i < b.rows.size(); i++)
            {
                const BatchRow& row = b.rows[i];
                if (const std::string problem = row.active(b.output_root) ? b.nameProblem(i) : std::string(); !problem.empty())
                    wrappedColored(kRed, row.folder.name + ": output folder name: " + problem);
                if (!row.checkCurrent(b.output_root) || row.check->done)
                    continue;
                for (const auto& f : row.check->preflight.findings)
                    if (f.severity != Severity::Ok)
                        wrappedColored(f.severity == Severity::Error ? kRed : kAmber, row.folder.name + ": " + f.message);
            }
            if (const std::string problem = batchOutputProblem(b.output_root, b.found); !problem.empty())
                wrappedColored(kRed, problem);
            else if (!b.rows.empty() && !b.checking())
            {
                const Finding disk = b.diskFinding();
                wrappedColored(
                    disk.severity == Severity::Error         ? kRed
                        : disk.severity == Severity::Warning ? kAmber
                                                             : kGreen,
                    disk.message);
            }
        }

        if (!b.skipped.empty() &&
            ImGui::TreeNode("skipped", "%zu folder%s skipped (not recordings)", b.skipped.size(), b.skipped.size() > 1 ? "s" : ""))
        {
            for (const auto& f : b.skipped)
                ImGui::BulletText("%s: %s", f.name.c_str(), f.skip_reason.c_str());
            ImGui::TreePop();
        }
    }

    void drawBatchRun(App& app)
    {
        BatchUi& b = app.batch;
        ImGui::SeparatorText("4. Process");
        if (!b.started)
        {
            size_t selected = 0;
            for (const auto& r : b.rows)
                selected += r.included() ? 1 : 0;
            ImGui::BeginDisabled(!b.readyToStart());
            const std::string label = "Start processing " + std::to_string(selected) + " recording" + (selected == 1 ? "" : "s");
            if (ImGui::Button(label.c_str(), ImVec2(260, 0)))
            {
                b.start_error.clear();
                if (b.runner.start(b.makeRequest(app.settings), b.start_error))
                    b.started = true;
            }
            ImGui::EndDisabled();
            if (!b.start_error.empty())
                wrappedColored(kRed, b.start_error);
            return;
        }

        const BatchStatus s = b.runner.status();
        const BatchRequest& request = b.runner.request();
        size_t selected = 0, finished = 0;
        size_t counts[8] = {};
        for (size_t i = 0; i < s.items.size(); i++)
        {
            counts[static_cast<int>(s.items[i].state)]++;
            if (!request.items[i].include)
                continue;
            selected++;
            if (s.items[i].state != BatchItemState::Pending && s.items[i].state != BatchItemState::Running)
                finished++;
        }

        if (s.running)
        {
            std::string overall = std::to_string(finished) + " of " + std::to_string(selected) + " recordings finished, elapsed " +
                formatDuration(s.elapsed_s);
            if (s.eta_s >= 0.0)
                overall += ", about " + formatDuration(s.eta_s) + " left";
            ImGui::TextUnformatted("All recordings");
            ImGui::ProgressBar(static_cast<float>(s.fraction), ImVec2(-1, 0), overall.c_str());
            ImGui::Spacing();
            if (s.current >= 0)
            {
                ImGui::Text(
                    "Now: %s (%d%% of this recording)",
                    request.items[s.current].name.c_str(),
                    static_cast<int>(pipelineFraction(s.current_status) * 100));
                drawStageProgress(s.current_status);
            }
            else
                ImGui::TextUnformatted("Checking the next recording...");

            ImGui::Spacing();
            ImGui::BeginDisabled(s.stop_after_current);
            if (ImGui::Button(s.stop_after_current ? "Stopping after this recording..." : "Stop after this recording", ImVec2(260, 0)))
                b.runner.stopAfterCurrent();
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Stop now", ImVec2(120, 0)))
                ImGui::OpenPopup("Stop now?");
            if (ImGui::BeginPopupModal("Stop now?", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
            {
                ImGui::TextUnformatted(
                    "Stop now? The recording being processed is discarded.\nRecordings that are already finished are kept.");
                if (ImGui::Button("Yes, stop now", ImVec2(140, 0)))
                {
                    b.runner.stopNow();
                    ImGui::CloseCurrentPopup();
                }
                ImGui::SameLine();
                if (ImGui::Button("No, continue", ImVec2(140, 0)))
                    ImGui::CloseCurrentPopup();
                ImGui::EndPopup();
            }
            ImGui::TextDisabled("The computer is kept from going to sleep while the batch runs.");
            return;
        }

        const size_t failed = counts[static_cast<int>(BatchItemState::Failed)] + counts[static_cast<int>(BatchItemState::Cancelled)];
        const size_t not_run = counts[static_cast<int>(BatchItemState::NotRun)];
        std::string summary = std::string(s.stop_after_current ? "Batch stopped" : "Batch finished") + " after " +
            formatDuration(s.elapsed_s) + ": " + std::to_string(counts[static_cast<int>(BatchItemState::Succeeded)]) + " succeeded";
        if (counts[static_cast<int>(BatchItemState::AlreadyDone)])
            summary += ", " + std::to_string(counts[static_cast<int>(BatchItemState::AlreadyDone)]) + " already done before";
        if (failed)
            summary += ", " + std::to_string(failed) + " failed or stopped";
        if (not_run)
            summary += ", " + std::to_string(not_run) + " not run";
        summary += ".";
        wrappedColored(failed || not_run ? kAmber : kGreen, summary);
        if (failed)
            ImGui::TextWrapped(
                "Point at 'failed' in the table to see why. Each recording's own easy_gui.log has the details. Start the same batch "
                "again to retry the recordings that did not finish.");
        ImGui::TextDisabled("batch_summary.csv and batch.log are in %s", request.output_root.string().c_str());

        if (ImGui::Button("Open output folder"))
            OpenURL(request.output_root.string().c_str());
        ImGui::SameLine();
        if (ImGui::Button("Process another batch"))
            b.reset();
    }

    void drawLogLines(const char* id, const std::vector<std::string>& lines)
    {
        ImGui::BeginChild(id, ImVec2(0, 200), ImGuiChildFlags_Borders, ImGuiWindowFlags_HorizontalScrollbar);
        for (const auto& line : lines)
            ImGui::TextUnformatted(line.c_str());
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 5.0f) // follow the end unless the user scrolled up
            ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }

    void drawBatchLog(App& app)
    {
        ImGui::Spacing();
        if (!ImGui::CollapsingHeader("Details (processing log)"))
            return;
        ImGui::TextUnformatted("Batch (batch.log)");
        drawLogLines("batch_log", app.batch.runner.logTail());
        ImGui::TextUnformatted("Current recording (its easy_gui.log)");
        drawLogLines("recording_log", app.batch.runner.currentLogTail());
    }

    void drawModeSwitch(App& app, bool locked)
    {
        ImGui::BeginDisabled(locked);
        int mode = static_cast<int>(app.mode);
        ImGui::RadioButton("One recording", &mode, static_cast<int>(Mode::Single));
        ImGui::SameLine();
        ImGui::RadioButton("Several recordings", &mode, static_cast<int>(Mode::Batch));
        app.mode = static_cast<Mode>(mode);
        ImGui::EndDisabled();
        ImGui::Spacing();
    }
} // namespace

namespace
{
    //! `easy_gui --headless <recording> <output folder>`: the same pipeline without a window, for scripted runs
    //! (the dev validation script) and remote machines. Exit code 0 = success, 1 = failed / refused, 2 = usage.
    int runHeadless(const fs::path& recording, const fs::path& output_folder, const Settings& settings, double check_band_outer_m)
    {
        // Unbuffered, so progress shows up as it happens even when redirected to a file or pipe (MSVC treats _IOLBF as
        // full buffering); the output is a line every few seconds.
        std::setvbuf(stdout, nullptr, _IONBF, 0);
        const PreflightResult check = runPreflight(recording, output_folder);
        for (const auto& f : check.findings)
            std::printf(
                "%s %s\n",
                f.severity == Severity::Error         ? "[X]"
                    : f.severity == Severity::Warning ? "[!]"
                                                      : "[OK]",
                f.message.c_str());
        if (!check.ok())
            return 1;

        Pipeline pipeline;
        std::string error;
        if (!pipeline.start(PipelineRequest{ recording, output_folder, findStep1Executable(), settings, check_band_outer_m }, error))
        {
            std::printf("[X] %s\n", error.c_str());
            return 1;
        }
        std::string last;
        while (pipeline.busy())
        {
            const PipelineStatus s = pipeline.status();
            char line[256];
            const double f = s.step1.phaseFraction();
            if (s.stage == PipelineStage::Export)
                std::snprintf(line, sizeof(line), "Export: %.0f%%", s.export_fraction * 100.0);
            else
                std::snprintf(
                    line,
                    sizeof(line),
                    "%s: %s overall %.0f%%",
                    phaseLabel(s.step1.phase),
                    f >= 0.0 ? (std::to_string(static_cast<int>(f * 100)) + "%,").c_str() : "",
                    s.step1.overallFraction() * 100.0);
            if (last != line)
                std::printf("[%s] %s\n", formatDuration(s.elapsed_s).c_str(), line);
            last = line;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
        const PipelineStatus s = pipeline.status();
        std::printf(
            "%s %s (%s)\n", s.state == PipelineState::Succeeded ? "[OK]" : "[X]", s.message.c_str(), formatDuration(s.elapsed_s).c_str());
        if (!s.detail.empty())
            std::printf("    detail: %s\n", s.detail.c_str());
        return s.state == PipelineState::Succeeded ? 0 : 1;
    }

    std::atomic<int> g_interrupts{ 0 };

    extern "C" void onInterrupt(int)
    {
        g_interrupts++;
        std::signal(SIGINT, onInterrupt); // some C libraries reset the handler after each signal
    }

    std::string pointsText(uint64_t points)
    {
        char buffer[32];
        std::snprintf(buffer, sizeof(buffer), "%.1f M", points / 1e6);
        return buffer;
    }

    //! `easy_gui --batch <parent folder> <output folder>`: batch mode without a window, with the GUI's rules: every
    //! recording that passes the check is processed, finished ones are skipped. Ctrl+C once stops after the current
    //! recording, twice stops now. Exit code 0 = every selected recording succeeded (or was already done), 1 = some
    //! failed, were not run or the batch was refused, 2 = usage.
    int runBatchHeadless(const fs::path& parent, const fs::path& output_root, const Settings& settings, double check_band_outer_m)
    {
        std::setvbuf(stdout, nullptr, _IONBF, 0);
        std::error_code ec;
        if (!fs::is_directory(parent, ec))
        {
            std::printf("[X] The folder '%s' does not exist or is not a folder.\n", parent.string().c_str());
            return 1;
        }
        std::printf("Looking for recordings in '%s' and checking them...\n", parent.string().c_str());
        BatchPlan plan = planBatch(parent, output_root);
        BatchRequest& request = plan.request;
        request.step1_executable = findStep1Executable();
        request.settings = settings;
        request.check_band_outer_m = check_band_outer_m;
        for (const auto& f : request.skipped)
            std::printf("[skipped] %s: %s\n", f.name.c_str(), f.skip_reason.c_str());
        if (request.items.empty())
        {
            std::printf("[X] No recordings found in '%s' or its subfolders.\n", parent.string().c_str());
            return 1;
        }
        if (const std::string problem = batchOutputProblem(output_root, plan.found); !problem.empty())
        {
            std::printf("[X] %s\n", problem.c_str());
            return 1;
        }

        for (size_t i = 0; i < request.items.size(); i++)
        {
            const BatchItem& item = request.items[i];
            const BatchCheck& c = plan.checks[i];
            const std::string label = item.name + (item.output_name == item.name ? "" : " -> " + item.output_name);
            if (c.done)
                std::printf("[done] %s: already processed (finished %s)\n", label.c_str(), c.done->finished.c_str());
            else if (!item.include)
                std::printf("[X] %s: %s\n", label.c_str(), firstError(c.preflight).c_str());
            else
            {
                std::printf(
                    "[OK] %s: %s, %s points\n",
                    label.c_str(),
                    formatDuration(c.preflight.duration_s).c_str(),
                    pointsText(c.preflight.total_points).c_str());
                for (const auto& f : c.preflight.findings)
                    if (f.severity == Severity::Warning)
                        std::printf("     [!] %s\n", f.message.c_str());
            }
        }
        if (std::none_of(
                request.items.begin(),
                request.items.end(),
                [](const BatchItem& item)
                {
                    return item.include;
                }))
        {
            std::printf("[X] None of the recordings passed the check; nothing to process.\n");
            return 1;
        }

        uint64_t available = 0;
        fs::path checked;
        const bool known = availableSpace(output_root, available, checked);
        const Finding disk = batchDiskFinding(batchDiskNeed(plan.toProcess()), known, available);
        std::printf(
            "%s %s\n",
            disk.severity == Severity::Error         ? "[X]"
                : disk.severity == Severity::Warning ? "[!]"
                                                     : "[OK]",
            disk.message.c_str());
        if (disk.severity == Severity::Error)
            return 1;

        BatchRunner runner;
        std::string error;
        if (!runner.start(request, error))
        {
            std::printf("[X] %s\n", error.c_str());
            return 1;
        }
        std::signal(SIGINT, onInterrupt);
        int interrupts_seen = 0;
        std::string last;
        auto last_print = std::chrono::steady_clock::now() - std::chrono::seconds(5);
        while (runner.busy())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if (const int n = g_interrupts.load(); n != interrupts_seen)
            {
                interrupts_seen = n;
                if (n == 1)
                {
                    runner.stopAfterCurrent();
                    std::printf("Stopping after the current recording. Press Ctrl+C again to stop now.\n");
                }
                else
                {
                    runner.stopNow();
                    std::printf("Stopping now.\n");
                }
            }
            const BatchStatus s = runner.status();
            std::string line = "overall " + std::to_string(static_cast<int>(s.fraction * 100)) + "%";
            if (s.eta_s >= 0.0)
                line += ", about " + formatDuration(s.eta_s) + " left";
            if (s.current >= 0)
            {
                const PipelineStatus& p = s.current_status;
                std::string stage = p.stage == PipelineStage::Odometry ? phaseLabel(p.step1.phase)
                    : p.stage == PipelineStage::Export                 ? "Export"
                                                                       : "Preparing";
                line += " | " + std::to_string(s.current + 1) + "/" + std::to_string(request.items.size()) + " " +
                    request.items[s.current].name + ": " + stage + " " + std::to_string(static_cast<int>(pipelineFraction(p) * 100)) + "%";
            }
            const auto now = std::chrono::steady_clock::now();
            if (line != last && now - last_print >= std::chrono::seconds(5))
            {
                std::printf("[%s] %s\n", formatDuration(s.elapsed_s).c_str(), line.c_str());
                last = line;
                last_print = now;
            }
        }
        std::signal(SIGINT, SIG_DFL);

        const BatchStatus s = runner.status();
        bool all_good = true;
        for (size_t i = 0; i < request.items.size(); i++)
        {
            const BatchItemResult& r = s.items[i];
            const bool good = r.state == BatchItemState::Succeeded || r.state == BatchItemState::AlreadyDone;
            if (request.items[i].include && !good)
                all_good = false;
            std::printf(
                "%s %s: %s%s%s\n",
                good                                      ? "[OK]"
                    : r.state == BatchItemState::Excluded ? "[--]"
                                                          : "[X]",
                request.items[i].name.c_str(),
                batchStateToken(r.state),
                r.error.empty() ? "" : ": ",
                r.error.c_str());
        }
        std::printf(
            "Batch %s after %s. Summary: %s\n",
            s.stop_after_current ? "stopped" : "finished",
            formatDuration(s.elapsed_s).c_str(),
            (output_root / BatchRunner::kSummaryFileName).string().c_str());
        return all_good ? 0 : 1;
    }

    //! Dev aid: `easy_gui --export-only <recording> <session.mjs> <output folder> [--check-band <m>]` runs only the
    //! export, on an existing step 1 session of that recording (step 1 takes ~20 min on a 10-min walk; the export
    //! about a minute). The output folder may exist; the three result files in it are overwritten.
    int runExportOnly(
        const fs::path& recording,
        const fs::path& session_file,
        const fs::path& output_folder,
        const Settings& settings,
        double check_band_outer_m)
    {
        std::setvbuf(stdout, nullptr, _IONBF, 0);
        const PreflightResult check = runPreflight(recording, output_folder);
        if (check.files.pairs == 0)
        {
            std::printf("[X] no lidar/IMU file pairs in '%s'\n", recording.string().c_str());
            return 1;
        }
        const OutputLayout out(output_folder);
        ExportRequest request;
        request.session_file = session_file;
        request.recording = check.files;
        request.lidar_id = std::max(0, check.lidar_id);
        request.near_min_range_m = settings.near_min_range_m;
        request.check_band_outer_m = check_band_outer_m;
        request.place_by_raw_time = settings.place_by_raw_time;
        request.laz_file = out.point_cloud;
        request.trajectory_file = out.trajectory;
        request.readme_file = out.readme;

        std::atomic<bool> cancel{ false };
        int last_percent = -1;
        ExportStats stats;
        std::string error;
        const bool ok = exportSession(
            request,
            cancel,
            [&](double fraction, const std::string& step)
            {
                const int percent = static_cast<int>(fraction * 100);
                if (percent / 10 != last_percent / 10)
                    std::printf("[%3d%%] %s\n", percent, step.c_str());
                last_percent = percent;
            },
            stats,
            error);
        if (!ok)
        {
            std::printf("[X] %s\n", error.c_str());
            return 1;
        }
        std::printf(
            "[OK] %llu odometry + %llu near + %llu gap + %llu check-band points; raw read %llu, no-return %llu, outside trajectory %llu; "
            "range %.3f - %.3f m; %zu poses; LAZ %llu bytes, CSV %llu bytes\n",
            static_cast<unsigned long long>(stats.odometry_points),
            static_cast<unsigned long long>(stats.near_points),
            static_cast<unsigned long long>(stats.gap_points),
            static_cast<unsigned long long>(stats.check_band_points),
            static_cast<unsigned long long>(stats.raw_points_read),
            static_cast<unsigned long long>(stats.raw_no_return),
            static_cast<unsigned long long>(stats.raw_outside_trajectory),
            stats.range_min_m,
            stats.range_max_m,
            stats.trajectory_nodes,
            static_cast<unsigned long long>(stats.laz_bytes),
            static_cast<unsigned long long>(stats.trajectory_bytes));
        return 0;
    }

    //! Removes `--check-band <m>` from the arguments; returns the value, or 0 when absent.
    double takeCheckBand(std::vector<std::string>& args)
    {
        for (size_t i = 0; i + 1 < args.size(); i++)
            if (args[i] == "--check-band")
            {
                const double value = std::atof(args[i + 1].c_str());
                args.erase(args.begin() + static_cast<long>(i), args.begin() + static_cast<long>(i) + 2);
                return value;
            }
        return 0.0;
    }

    //! Removes `flag` from the arguments; returns whether it was there.
    bool takeFlag(std::vector<std::string>& args, const std::string& flag)
    {
        const auto it = std::find(args.begin(), args.end(), flag);
        if (it == args.end())
            return false;
        args.erase(it);
        return true;
    }
} // namespace

int main(int argc, char* argv[])
{
    std::vector<std::string> args(argv + 1, argv + argc);
    if (!args.empty() && (args[0] == "--headless" || args[0] == "--export-only" || args[0] == "--batch"))
    {
        attachToParentConsole();
        const double check_band = takeCheckBand(args);
        Settings settings;
        settings.place_by_raw_time = takeFlag(args, "--place-by-raw-time");
        if (args[0] == "--headless" && args.size() == 3)
            return runHeadless(args[1], args[2], settings, check_band);
        if (args[0] == "--export-only" && args.size() == 4)
            return runExportOnly(args[1], args[2], args[3], settings, check_band);
        if (args[0] == "--batch" && args.size() == 3)
            return runBatchHeadless(args[1], args[2], settings, check_band);
        std::printf(
            "usage: %s --headless <recording folder> <output folder> [--check-band <m>] [--place-by-raw-time]\n"
            "       %s --batch <parent folder> <output folder> [--check-band <m>] [--place-by-raw-time]\n"
            "       %s --export-only <recording folder> <session.mjs> <output folder> [--check-band <m>] [--place-by-raw-time]\n"
            "--batch: processes every recording in <parent folder> (one folder level down; the parent itself too, if it is a\n"
            "         recording) into <output folder>/<recording name>/. Recordings finished in an earlier run are skipped.\n"
            "         Ctrl+C stops after the current recording, a second Ctrl+C stops now.\n"
            "--check-band <m> (testing): also export the raw points between the 1.0 m load filter and <m>, flagged\n"
            "                 point_origin = 3, to check that near points line up with the odometry points.\n"
            "--place-by-raw-time: place all points from the raw files at their real time (export option, default off).\n",
            argv[0],
            argv[0],
            argv[0]);
        return 2;
    }

    // Dev aid: `--screenshot <file.png> [recording | --batch <parent folder>]` renders ~2 s, saves the window and exits (layout checks
    // without capturing the whole desktop).
    std::string screenshot_file;
    int first_positional = 1;
    if (argc > 2 && std::string(argv[1]) == "--screenshot")
    {
        screenshot_file = argv[2];
        first_positional = 3;
    }

    App app;
    if (argc > first_positional + 1 && std::string(argv[first_positional]) == "--batch" && fs::is_directory(argv[first_positional + 1]))
    {
        app.mode = Mode::Batch;
        app.batch.setParent(argv[first_positional + 1]);
    }
    else if (argc > first_positional && fs::is_directory(argv[first_positional]))
        app.setRecording(argv[first_positional]);

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(900, 780, versionString().c_str());
    raylib_widgets::fitWindowToScreen();
    // raylib's default exit key (Esc) would close the window mid-run.
    SetExitKey(KEY_NULL);
    SetTargetFPS(30);
    rlImGuiSetup(true);

    int frame = 0;
    while (!WindowShouldClose())
    {
        app.poll();
        app.batch.poll();
        const bool single_locked = app.pipeline.busy() || app.pipeline.status().state != PipelineState::Idle;
        const bool batch_locked = app.batch.runner.busy() || app.batch.started;
        const bool locked = app.mode == Mode::Single ? single_locked : batch_locked;

        if (IsFileDropped())
        {
            FilePathList dropped = LoadDroppedFiles();
            if (dropped.count > 0 && !locked)
            {
                fs::path p(dropped.paths[0]);
                std::error_code ec;
                const fs::path folder = fs::is_directory(p, ec) ? p : p.parent_path();
                if (app.mode == Mode::Single)
                    app.setRecording(folder);
                else
                    app.batch.setParent(folder);
            }
            UnloadDroppedFiles(dropped);
        }

        BeginDrawing();
        ClearBackground(Color{ 30, 30, 30, 255 });
        rlImGuiBegin();

        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::Begin(
            "Easy Forest HDMapping",
            nullptr,
            // A scrollbar appears when the content is taller than the window (batch table, open log).
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoSavedSettings);

        ImGui::TextUnformatted("Easy Forest HDMapping");
        ImGui::TextDisabled("Mandeye recording -> lidar odometry (precision forestry settings) -> point cloud and trajectory");
        ImGui::Spacing();
        drawModeSwitch(app, single_locked || batch_locked);

        if (app.mode == Mode::Single)
        {
            drawInputs(app, locked);
            drawPreflight(app);
            drawRun(app);
            drawLog(app);
        }
        else
        {
            drawBatchInputs(app, locked);
            drawBatchTable(app, locked);
            drawBatchRun(app);
            drawBatchLog(app);
        }

        ImGui::End();
        rlImGuiEnd();
        EndDrawing();

        // Dev aid for documentation screenshots of a running batch: EASY_GUI_SCREENSHOT_START=1 starts it once ready,
        // EASY_GUI_SCREENSHOT_FRAME=<n> takes the screenshot at frame n (30 frames per second) instead of 60.
        static const bool auto_start = !screenshot_file.empty() && std::getenv("EASY_GUI_SCREENSHOT_START");
        if (auto_start && app.mode == Mode::Batch && app.batch.readyToStart())
        {
            if (app.batch.runner.start(app.batch.makeRequest(app.settings), app.batch.start_error))
                app.batch.started = true;
        }
        static const int screenshot_frame =
            std::getenv("EASY_GUI_SCREENSHOT_FRAME") ? std::max(1, std::atoi(std::getenv("EASY_GUI_SCREENSHOT_FRAME"))) : 60;
        if (!screenshot_file.empty() && ++frame == screenshot_frame)
        {
            TakeScreenshot(screenshot_file.c_str());
            break;
        }
    }

    // Closing the window while processing stops the child process (Pipeline's destructor cancels and joins).
    app.pipeline.cancel();
    app.batch.runner.stopNow();
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
