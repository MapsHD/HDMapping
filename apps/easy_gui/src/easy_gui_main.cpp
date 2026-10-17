// HDMapping Easy GUI: raw Mandeye recording -> lidar odometry (precision forestry preset) -> point cloud + trajectory.
// See CLAUDE.md ("Easy GUI") for the decisions behind the pipeline.

#include "export.h"
#include "pipeline.h"
#include "preflight.h"
#include "settings.h"

#include <Core/pfd_wrapper.hpp>
#include <HDMapping/HDMAPPING_ConfigureInfo.hpp>
#include <RaylibWidgets/WindowFit.h>

#include <imgui.h>
#include <raylib.h>
#include <rlImGui.h>

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <future>
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

    //! ~/Documents/HDMapping Easy/<recording name>, with _2, _3, ... appended if that already exists. Never next to
    //! the recording: that may be the scanner's USB stick.
    fs::path defaultOutputFolder(const fs::path& recording)
    {
        const fs::path base = homeFolder() / "Documents" / "HDMapping Easy";
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

    struct App
    {
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
            preflight_future = std::async(std::launch::async, runPreflight, recording, output_folder);
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

        if (ImGui::CollapsingHeader("Advanced settings"))
        {
            ImGui::Indent();
            float near = static_cast<float>(app.settings.near_min_range_m);
            ImGui::SetNextItemWidth(220);
            if (ImGui::SliderFloat(
                    "Closest points kept [m]",
                    &near,
                    static_cast<float>(Settings::kNearMinRangeLowest),
                    static_cast<float>(Settings::kNearMinRangeHighest),
                    "%.2f m"))
            {
                app.settings.near_min_range_m = near;
                app.settings.clamp();
            }
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Points closer to the scanner than this are left out of the point cloud.\n"
                    "0.4 m (default) removes the operator's body and the handle.\n"
                    "Lower values keep more of the close surroundings but may include the operator.");
            ImGui::Checkbox("Place all points by their measurement time (experimental)", &app.settings.place_by_raw_time);
            ImGui::SameLine();
            ImGui::TextDisabled("(?)");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Off (default): the point cloud uses lidar odometry's own points.\n"
                    "On: every point is taken from the raw recording and placed at its exact measurement time.");
            ImGui::Checkbox("Keep intermediate files (for troubleshooting)", &app.settings.keep_work_files);
            ImGui::Unindent();
        }

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

                const double overall = s.step1.overallFraction();
                std::string eta = "Elapsed " + formatDuration(s.elapsed_s);
                if (overall > 0.05)
                    eta += ", about " + formatDuration(s.elapsed_s * (1.0 - overall) / overall) + " left";
                ImGui::TextUnformatted("Overall");
                ImGui::ProgressBar(static_cast<float>(overall), ImVec2(-1, 0), eta.c_str());
            }
            else if (s.stage == PipelineStage::Export)
            {
                ImGui::TextUnformatted(s.export_step.empty() ? "Exporting..." : s.export_step.c_str());
                ImGui::ProgressBar(static_cast<float>(s.export_fraction), ImVec2(-1, 0));
                ImGui::TextUnformatted(("Elapsed " + formatDuration(s.elapsed_s)).c_str());
            }
            else
            {
                ImGui::TextUnformatted(s.stage == PipelineStage::Cleanup ? "Cleaning up..." : "Preparing...");
                ImGui::ProgressBar(-1.0f * static_cast<float>(ImGui::GetTime()), ImVec2(-1, 0), "working...");
            }

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
    if (!args.empty() && (args[0] == "--headless" || args[0] == "--export-only"))
    {
        const double check_band = takeCheckBand(args);
        Settings settings;
        settings.place_by_raw_time = takeFlag(args, "--place-by-raw-time");
        if (args[0] == "--headless" && args.size() == 3)
            return runHeadless(args[1], args[2], settings, check_band);
        if (args[0] == "--export-only" && args.size() == 4)
            return runExportOnly(args[1], args[2], args[3], settings, check_band);
        std::printf(
            "usage: %s --headless <recording folder> <output folder> [--check-band <m>] [--place-by-raw-time]\n"
            "       %s --export-only <recording folder> <session.mjs> <output folder> [--check-band <m>] [--place-by-raw-time]\n"
            "--check-band <m> (testing): also export the raw points between the 1.0 m load filter and <m>, flagged\n"
            "                 point_origin = 3, to check that near points line up with the odometry points.\n"
            "--place-by-raw-time: place all points from the raw files at their real time (export option, default off).\n",
            argv[0],
            argv[0]);
        return 2;
    }

    // Dev aid: `--screenshot <file.png> [recording]` renders ~2 s, saves the window and exits (layout checks without
    // capturing the whole desktop).
    std::string screenshot_file;
    int first_positional = 1;
    if (argc > 2 && std::string(argv[1]) == "--screenshot")
    {
        screenshot_file = argv[2];
        first_positional = 3;
    }

    App app;
    if (argc > first_positional && fs::is_directory(argv[first_positional]))
        app.setRecording(argv[first_positional]);

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(900, 780, ("HDMapping Easy GUI " + std::string(HDMAPPING_CONFIGURE_PROJECT_VERSION)).c_str());
    raylib_widgets::fitWindowToScreen();
    // raylib's default exit key (Esc) would close the window mid-run.
    SetExitKey(KEY_NULL);
    SetTargetFPS(30);
    rlImGuiSetup(true);

    int frame = 0;
    while (!WindowShouldClose())
    {
        app.poll();
        const bool locked = app.pipeline.busy() || app.pipeline.status().state != PipelineState::Idle;

        if (IsFileDropped())
        {
            FilePathList dropped = LoadDroppedFiles();
            if (dropped.count > 0 && !locked)
            {
                fs::path p(dropped.paths[0]);
                std::error_code ec;
                app.setRecording(fs::is_directory(p, ec) ? p : p.parent_path());
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
            "HDMapping Easy GUI", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings);

        ImGui::TextUnformatted("HDMapping Easy GUI");
        ImGui::TextDisabled("Mandeye recording -> lidar odometry (precision forestry settings) -> point cloud and trajectory");
        ImGui::Spacing();

        drawInputs(app, locked);
        drawPreflight(app);
        drawRun(app);
        drawLog(app);

        ImGui::End();
        rlImGuiEnd();
        EndDrawing();

        if (!screenshot_file.empty() && ++frame == 60)
        {
            TakeScreenshot(screenshot_file.c_str());
            break;
        }
    }

    // Closing the window while processing stops the child process (Pipeline's destructor cancels and joins).
    app.pipeline.cancel();
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
