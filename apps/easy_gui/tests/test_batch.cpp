#include <doctest.h>

#include "batch.h"
#include "test_helpers.h"

#include <algorithm>
#include <condition_variable>
#include <sstream>

using namespace easy_gui;
using easy_gui_test::TempDir;
namespace fs = std::filesystem;

namespace
{
    std::vector<std::string> readLines(const fs::path& file)
    {
        std::ifstream in(file);
        std::vector<std::string> lines;
        std::string line;
        while (std::getline(in, line))
            lines.push_back(line);
        return lines;
    }

    const DiscoveredFolder* findFolder(const std::vector<DiscoveredFolder>& folders, const std::string& name)
    {
        for (const auto& f : folders)
            if (f.name == name)
                return &f;
        return nullptr;
    }

    //! Stands in for the Pipeline: recordings whose folder name contains "fail" fail, the others succeed and leave
    //! a result file. With `block` set, a job stays busy until release() or cancel().
    struct FakeJobs
    {
        std::mutex mutex;
        std::condition_variable cv;
        std::vector<PipelineRequest> started;
        bool block = false;
        bool released = false;

        void release()
        {
            std::lock_guard lock(mutex);
            released = true;
            cv.notify_all();
        }

        size_t startedCount()
        {
            std::lock_guard lock(mutex);
            return started.size();
        }

        RecordingJobFactory factory()
        {
            return [this]
            {
                return std::make_unique<Job>(*this);
            };
        }

        class Job : public RecordingJob
        {
        public:
            explicit Job(FakeJobs& owner)
                : owner_(owner)
            {
            }
            ~Job() override
            {
                if (thread_.joinable())
                    thread_.join();
            }
            bool start(const PipelineRequest& request, std::string&) override
            {
                {
                    std::lock_guard lock(owner_.mutex);
                    owner_.started.push_back(request);
                }
                status_.state = PipelineState::Running;
                status_.stage = PipelineStage::Odometry;
                busy_ = true;
                thread_ = std::thread(
                    [this, request]
                    {
                        bool cancelled = false;
                        {
                            std::unique_lock lock(owner_.mutex);
                            owner_.cv.wait(
                                lock,
                                [&]
                                {
                                    return !owner_.block || owner_.released || cancel_;
                                });
                            cancelled = cancel_;
                        }
                        std::lock_guard lock(status_mutex_);
                        if (cancelled)
                        {
                            status_.state = PipelineState::Cancelled;
                            status_.message = "Processing was cancelled.";
                        }
                        else if (request.recording.filename().string().find("fail") != std::string::npos)
                        {
                            status_.state = PipelineState::Failed;
                            status_.message = "Lidar odometry stopped unexpectedly (exit code 3).";
                        }
                        else
                        {
                            fs::create_directories(request.output_folder);
                            easy_gui_test::writeText(request.output_folder / "point_cloud.laz", "fake");
                            status_.state = PipelineState::Succeeded;
                            status_.export_stats.odometry_points = 1000;
                            status_.export_stats.near_points = 20;
                            status_.export_stats.gap_points = 3;
                            status_.elapsed_s = 12.0;
                        }
                        busy_ = false;
                    });
                return true;
            }
            bool busy() const override
            {
                return busy_;
            }
            PipelineStatus status() const override
            {
                std::lock_guard lock(status_mutex_);
                return status_;
            }
            void cancel() override
            {
                std::lock_guard lock(owner_.mutex);
                cancel_ = true;
                owner_.cv.notify_all();
            }

        private:
            FakeJobs& owner_;
            std::thread thread_;
            std::atomic<bool> busy_{ false };
            bool cancel_ = false; // guarded by owner_.mutex
            mutable std::mutex status_mutex_;
            PipelineStatus status_;
        };
    };

    BatchRequest prepareBatch(const fs::path& parent, const fs::path& output_root)
    {
        return planBatch(parent, output_root).request;
    }

    BatchStatus runToEnd(BatchRunner& runner, const BatchRequest& request)
    {
        std::string error;
        REQUIRE(runner.start(request, error));
        runner.wait();
        return runner.status();
    }

    BatchStatus waitFor(const BatchRunner& runner, bool (*done)(const BatchStatus&))
    {
        for (int i = 0; i < 200; i++)
        {
            const BatchStatus s = runner.status();
            if (done(s))
                return s;
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        return runner.status();
    }
} // namespace

TEST_CASE("discoverRecordings finds recordings one level down and explains the other folders")
{
    TempDir tmp;
    const fs::path parent = tmp.path / "field_day";
    easy_gui_test::writeRecording(parent / "plot_b");
    easy_gui_test::writeRecording(parent / "plot_a");
    fs::create_directories(parent / "notes");
    easy_gui_test::writeText(parent / "notes" / "todo.txt", "x");
    easy_gui_test::writeRecording(parent / "day2" / "plot_c"); // nested two levels: not searched
    fs::create_directories(parent / "only_imu");
    easy_gui_test::writeImuCsv(parent / "only_imu" / "imu0000.csv", 10, 1783328629.0);
    easy_gui_test::writeRecording(parent / ".Trashes"); // hidden: ignored even though it looks like a recording
    easy_gui_test::writeText(parent / "readme.txt", "a file, not a folder");

    const auto found = discoverRecordings(parent);
    REQUIRE(found.size() == 5);
    CHECK(found[0].name == "day2"); // sorted by name
    CHECK_FALSE(found[0].is_recording);
    CHECK(found[0].skip_reason.find("one folder level down") != std::string::npos);
    CHECK(findFolder(found, "notes")->skip_reason.find("No point cloud files") != std::string::npos);
    CHECK(findFolder(found, "only_imu")->skip_reason == "No point cloud files (lidar*.laz).");
    CHECK(findFolder(found, "plot_a")->is_recording);
    CHECK(findFolder(found, "plot_b")->is_recording);
    CHECK(findFolder(found, ".Trashes") == nullptr);
    CHECK(findFolder(found, "plot_c") == nullptr);

    CHECK(discoverRecordings(tmp.path / "missing").empty());
}

TEST_CASE("discoverRecordings accepts the parent itself as a recording")
{
    TempDir tmp;
    const fs::path parent = tmp.path / "continousScanning_0012";
    easy_gui_test::writeRecording(parent);
    SUBCASE("alone")
    {
        const auto found = discoverRecordings(parent);
        REQUIRE(found.size() == 1);
        CHECK(found[0].is_recording);
        CHECK(found[0].name == "continousScanning_0012");
        CHECK(found[0].path == parent);
    }
    SUBCASE("with a trailing separator and a recording inside it, listed first")
    {
        easy_gui_test::writeRecording(parent / "continousScanning_0012");
        const auto found = discoverRecordings(parent.string() + "/");
        REQUIRE(found.size() == 2);
        CHECK(found[0].name == "continousScanning_0012");
        CHECK(found[1].name == "continousScanning_0012");
        CHECK(found[1].path.filename() == "continousScanning_0012");

        // The same name twice: the default output names differ.
        const auto names = defaultOutputNames(found, tmp.path / "out");
        CHECK(names == std::vector<std::string>{ "continousScanning_0012", "continousScanning_0012_2" });
    }
}

TEST_CASE("discoverRecordings recognises Easy Forest HDMapping results inside the parent")
{
    TempDir tmp;
    easy_gui_test::writeRecording(tmp.path / "rec");
    fs::create_directories(tmp.path / "results");
    easy_gui_test::writeText(tmp.path / "results" / "batch_summary.csv", "x");
    const auto found = discoverRecordings(tmp.path);
    CHECK(findFolder(found, "results")->skip_reason == "Easy Forest HDMapping results, not a recording.");
}

TEST_CASE("output names: validation and collisions")
{
    CHECK(outputNameProblem("plot_12").empty());
    CHECK(outputNameProblem("Plot 12 (north)").empty());
    CHECK_FALSE(outputNameProblem("").empty());
    CHECK_FALSE(outputNameProblem("a/b").empty());
    CHECK_FALSE(outputNameProblem("a\\b").empty());
    CHECK_FALSE(outputNameProblem("a:b").empty());
    CHECK_FALSE(outputNameProblem("..").empty());
    CHECK_FALSE(outputNameProblem("plot.").empty());
    CHECK_FALSE(outputNameProblem("plot ").empty());
    CHECK_FALSE(outputNameProblem("CON").empty());
    CHECK_FALSE(outputNameProblem("com1.txt").empty());
    CHECK(outputNameProblem("console").empty());
    CHECK(sameOutputName("Plot_A", "plot_a"));
    CHECK_FALSE(sameOutputName("plot_a", "plot_b"));

    TempDir tmp;
    easy_gui_test::writeRecording(tmp.path / "x1");
    easy_gui_test::writeRecording(tmp.path / "x2", 1783329629.0);
    easy_gui_test::writeRecording(tmp.path / "x3", 1783330629.0);
    const fs::path out = tmp.path / "out";

    SUBCASE("names that differ only in case get a suffix; invalid characters are replaced")
    {
        const std::vector<DiscoveredFolder> recordings = { { tmp.path / "x1", "Plot", true, {} },
                                                           { tmp.path / "x2", "plot", true, {} },
                                                           { tmp.path / "x3", "a:b.", true, {} } };
        CHECK(defaultOutputNames(recordings, out) == std::vector<std::string>{ "Plot", "plot_2", "a_b" });
    }
    SUBCASE("a folder holding something else is passed over; this recording's result and unfinished attempts are reused")
    {
        fs::create_directories(out / "x1");
        easy_gui_test::writeText(out / "x1" / "my_notes.txt", "not ours");
        fs::create_directories(out / "x2");
        easy_gui_test::writeText(out / "x2" / "easy_gui.log", "unfinished attempt");
        fs::create_directories(out / "x3");
        std::string error;
        DoneMarker marker;
        marker.fingerprint = recordingFingerprint(classifyRecordingFiles(tmp.path / "x3"));
        REQUIRE(writeDoneMarker(out / "x3", marker, error));
        const std::vector<DiscoveredFolder> recordings = { { tmp.path / "x1", "x1", true, {} },
                                                           { tmp.path / "x2", "x2", true, {} },
                                                           { tmp.path / "x3", "x3", true, {} } };
        CHECK(defaultOutputNames(recordings, out) == std::vector<std::string>{ "x1_2", "x2", "x3" });

        // ... but another recording's finished result is not reused.
        const std::vector<DiscoveredFolder> other = { { tmp.path / "x1", "x3", true, {} } };
        CHECK(defaultOutputNames(other, out) == std::vector<std::string>{ "x3_2" });
    }
}

TEST_CASE("done markers round-trip and identify the recording by its files, not its path")
{
    TempDir tmp;
    easy_gui_test::writeRecording(tmp.path / "rec");
    const auto files = classifyRecordingFiles(tmp.path / "rec");
    const std::string fingerprint = recordingFingerprint(files);
    CHECK(fingerprint.find("2 file pairs") == 0);

    fs::create_directories(tmp.path / "out");
    DoneMarker m{ "/Volumes/USB/rec", fingerprint, "2026-10-18 10:00:00", 1234.0, 4000, 3900 };
    std::string error;
    REQUIRE(writeDoneMarker(tmp.path / "out", m, error));
    const auto r = readDoneMarker(tmp.path / "out");
    REQUIRE(r);
    CHECK(r->recording == m.recording);
    CHECK(r->fingerprint == fingerprint);
    CHECK(r->finished == m.finished);
    CHECK(r->duration_s == 1234.0);
    CHECK(r->raw_points == 4000);
    CHECK(r->exported_points == 3900);
    CHECK_FALSE(readDoneMarker(tmp.path / "nowhere"));

    // The same files at another path (copied off the USB stick) are the same recording.
    fs::copy(tmp.path / "rec", tmp.path / "copy");
    CHECK(checkBatchRecording(tmp.path / "copy", tmp.path / "out").done.has_value());
    // A different recording is not.
    easy_gui_test::writeRecording(tmp.path / "other");
    easy_gui_test::writeRawLaz(tmp.path / "other" / "lidar0001.laz", 2100, 1783328629.0 + 30.0);
    CHECK_FALSE(checkBatchRecording(tmp.path / "other", tmp.path / "out").done.has_value());
}

TEST_CASE("the batch disk check counts the largest temporary need once and every result")
{
    PreflightResult a, b, c;
    a.estimated_work_bytes = 10'000'000'000;
    a.estimated_result_bytes = 2'000'000'000;
    b.estimated_work_bytes = 14'000'000'000;
    b.estimated_result_bytes = 3'000'000'000;
    c.estimated_work_bytes = 6'000'000'000;
    c.estimated_result_bytes = 1'000'000'000;
    const BatchDiskNeed need = batchDiskNeed({ &a, &b, &c });
    CHECK(need.work_bytes == 14'000'000'000);
    CHECK(need.result_bytes == 6'000'000'000);
    CHECK(need.total() == 20'000'000'000);
    CHECK(batchDiskNeed({}).total() == 0);

    CHECK(batchDiskFinding(need, true, 19'000'000'000).severity == Severity::Error);
    CHECK(batchDiskFinding(need, true, 25'000'000'000).severity == Severity::Warning);
    const Finding ok = batchDiskFinding(need, true, 40'000'000'000);
    CHECK(ok.severity == Severity::Ok);
    CHECK(ok.message.find("14.0 GB temporary for the largest recording + 6.0 GB of results") != std::string::npos);
    CHECK(batchDiskFinding(need, false, 0).severity == Severity::Warning);
}

TEST_CASE("the batch output folder must not be inside a recording")
{
    TempDir tmp;
    easy_gui_test::writeRecording(tmp.path / "parent");
    easy_gui_test::writeRecording(tmp.path / "parent" / "sub");
    const auto found = discoverRecordings(tmp.path / "parent");
    CHECK_FALSE(batchOutputProblem(tmp.path / "parent" / "results", found).empty()); // the parent is a recording
    CHECK_FALSE(batchOutputProblem(tmp.path / "parent" / "sub" / "out", found).empty());
    CHECK(batchOutputProblem(tmp.path / "results", found).empty());
    CHECK_FALSE(batchOutputProblem({}, found).empty());

    // A plain container parent may hold the output folder.
    easy_gui_test::writeRecording(tmp.path / "container" / "rec");
    CHECK(batchOutputProblem(tmp.path / "container" / "results", discoverRecordings(tmp.path / "container")).empty());
}

TEST_CASE("a failed recording does not stop the batch; the summary and the log record every folder")
{
    TempDir tmp;
    const fs::path parent = tmp.path / "parent";
    easy_gui_test::writeRecording(parent / "a_plot");
    easy_gui_test::writeRecording(parent / "b_fail", 1783329629.0);
    easy_gui_test::writeRecording(parent / "c_plot, north", 1783330629.0); // comma: quoted in the CSV
    fs::create_directories(parent / "d_damaged");
    easy_gui_test::writeText(parent / "d_damaged" / "lidar0000.laz", "not a LAZ file");
    easy_gui_test::writeImuCsv(parent / "d_damaged" / "imu0000.csv", 6000, 1783328629.0);
    fs::create_directories(parent / "e_notes");
    const fs::path out = tmp.path / "out";

    BatchRequest request = prepareBatch(parent, out);
    REQUIRE(request.items.size() == 4);
    CHECK_FALSE(request.items[3].include); // d_damaged fails the check and is not selected
    CHECK(request.items[3].exclude_reason.find("cannot be read") != std::string::npos);
    REQUIRE(request.skipped.size() == 1);

    FakeJobs jobs;
    BatchRunner runner(jobs.factory());
    const BatchStatus s = runToEnd(runner, request);
    CHECK_FALSE(s.running);
    CHECK(s.items[0].state == BatchItemState::Succeeded);
    CHECK(s.items[1].state == BatchItemState::Failed);
    CHECK(s.items[1].error == "Lidar odometry stopped unexpectedly (exit code 3).");
    CHECK(s.items[2].state == BatchItemState::Succeeded);
    CHECK(s.items[3].state == BatchItemState::Excluded);
    CHECK(s.items[0].exported_points == 1023);
    CHECK(s.items[0].raw_points == 4000);
    CHECK(s.fraction == doctest::Approx(1.0));
    REQUIRE(jobs.started.size() == 3);
    CHECK(jobs.started[0].output_folder == out / "a_plot");
    CHECK(jobs.started[0].resume_unfinished);

    CHECK(readDoneMarker(out / "a_plot"));
    CHECK_FALSE(readDoneMarker(out / "b_fail"));
    CHECK(readDoneMarker(out / "c_plot, north"));

    const auto csv = readLines(out / BatchRunner::kSummaryFileName);
    REQUIRE(csv.size() == 6);
    CHECK(csv[0] == "recording,output_name,status,duration_s,raw_points,exported_points,output_path,error");
    CHECK(csv[1].find(",a_plot,succeeded,") != std::string::npos);
    CHECK(csv[1].find(",4000,1023,") != std::string::npos);
    CHECK(csv[2].find(",b_fail,failed,") != std::string::npos);
    CHECK(csv[2].find("exit code 3") != std::string::npos);
    CHECK(csv[3].find(",\"c_plot, north\",succeeded,") != std::string::npos);
    CHECK(csv[4].find(",d_damaged,excluded,") != std::string::npos);
    CHECK(csv[5].find(",,skipped,,,,,") != std::string::npos);
    CHECK_FALSE(fs::exists(out / "batch_summary.csv.tmp"));

    const auto log = readLines(out / BatchRunner::kLogFileName);
    auto logHas = [&](const std::string& text)
    {
        return std::any_of(
            log.begin(),
            log.end(),
            [&](const std::string& l)
            {
                return l.find(text) != std::string::npos;
            });
    };
    CHECK(logHas("batch started"));
    CHECK(logHas("recording b_fail: FAILED"));
    CHECK(logHas("skipped: e_notes"));
    CHECK(logHas("excluded: d_damaged"));
    CHECK(logHas("1 failed"));
}

TEST_CASE("restarting the same batch skips finished recordings and retries the others")
{
    TempDir tmp;
    const fs::path parent = tmp.path / "parent";
    easy_gui_test::writeRecording(parent / "a_plot");
    easy_gui_test::writeRecording(parent / "b_fail", 1783329629.0);
    easy_gui_test::writeRecording(parent / "c_plot", 1783330629.0);
    const fs::path out = tmp.path / "out";
    {
        FakeJobs jobs;
        BatchRunner runner(jobs.factory());
        runToEnd(runner, prepareBatch(parent, out));
        REQUIRE(jobs.started.size() == 3);
    }
    // b_fail left an unfinished attempt (a log and a half-written point cloud); it is retried, not refused.
    fs::create_directories(out / "b_fail");
    easy_gui_test::writeText(out / "b_fail" / "easy_gui.log", "first attempt\n");
    easy_gui_test::writeText(out / "b_fail" / "point_cloud.laz", "partial");

    const BatchRequest again = prepareBatch(parent, out);
    REQUIRE(again.items.size() == 3);
    CHECK(again.items[0].output_name == "a_plot"); // same names as before, so the markers are found
    CHECK(again.items[1].include);

    FakeJobs jobs;
    BatchRunner runner(jobs.factory());
    const BatchStatus s = runToEnd(runner, again);
    CHECK(s.items[0].state == BatchItemState::AlreadyDone);
    CHECK(s.items[0].exported_points == 1023); // from the marker
    CHECK(s.items[1].state == BatchItemState::Failed);
    CHECK(s.items[2].state == BatchItemState::AlreadyDone);
    REQUIRE(jobs.started.size() == 1);
    CHECK(jobs.started[0].recording == parent / "b_fail");
    const auto csv = readLines(out / BatchRunner::kSummaryFileName);
    CHECK(csv[1].find(",a_plot,already_done,") != std::string::npos);
}

TEST_CASE("a recording whose output folder holds another recording's result fails and the batch continues")
{
    TempDir tmp;
    const fs::path parent = tmp.path / "parent";
    easy_gui_test::writeRecording(parent / "a_plot");
    easy_gui_test::writeRecording(parent / "b_plot", 1783329629.0);
    const fs::path out = tmp.path / "out";
    BatchRequest request = prepareBatch(parent, out);
    // Simulates a stale table: the folder was filled after the check.
    fs::create_directories(out / "a_plot");
    easy_gui_test::writeText(out / "a_plot" / "my_notes.txt", "not ours");

    FakeJobs jobs;
    BatchRunner runner(jobs.factory());
    const BatchStatus s = runToEnd(runner, request);
    CHECK(s.items[0].state == BatchItemState::Failed);
    CHECK(s.items[0].error.find("already contains files") != std::string::npos);
    CHECK(s.items[1].state == BatchItemState::Succeeded);
    CHECK(jobs.startedCount() == 1);
}

TEST_CASE("stop after current lets the running recording finish; stop now cancels it")
{
    TempDir tmp;
    const fs::path parent = tmp.path / "parent";
    easy_gui_test::writeRecording(parent / "a_plot");
    easy_gui_test::writeRecording(parent / "b_plot", 1783329629.0);
    easy_gui_test::writeRecording(parent / "c_plot", 1783330629.0);
    const fs::path out = tmp.path / "out";

    FakeJobs jobs;
    jobs.block = true;
    BatchRunner runner(jobs.factory());
    std::string error;
    REQUIRE(runner.start(prepareBatch(parent, out), error));
    const BatchStatus running = waitFor(
        runner,
        [](const BatchStatus& s)
        {
            return s.current == 0 && s.current_status.state == PipelineState::Running;
        });
    REQUIRE(running.current == 0);
    CHECK(runner.busy());

    SUBCASE("stop after current")
    {
        runner.stopAfterCurrent();
        CHECK(runner.status().stop_after_current);
        jobs.release();
        runner.wait();
        const BatchStatus s = runner.status();
        CHECK(s.items[0].state == BatchItemState::Succeeded);
        CHECK(s.items[1].state == BatchItemState::NotRun);
        CHECK(s.items[2].state == BatchItemState::NotRun);
        CHECK(readDoneMarker(out / "a_plot"));
    }
    SUBCASE("stop now")
    {
        runner.stopNow();
        runner.wait();
        const BatchStatus s = runner.status();
        CHECK(s.items[0].state == BatchItemState::Cancelled);
        CHECK(s.items[1].state == BatchItemState::NotRun);
        CHECK(s.items[2].state == BatchItemState::NotRun);
        CHECK_FALSE(readDoneMarker(out / "a_plot"));
        const auto csv = readLines(out / BatchRunner::kSummaryFileName);
        CHECK(csv[1].find(",a_plot,cancelled,") != std::string::npos);
        CHECK(csv[2].find(",b_plot,not_run,") != std::string::npos);
    }
    CHECK(jobs.startedCount() == 1);
}

TEST_CASE("pipelineFraction weights odometry and export")
{
    PipelineStatus s;
    s.state = PipelineState::Running;
    s.stage = PipelineStage::Checking;
    CHECK(pipelineFraction(s) == 0.0);
    s.stage = PipelineStage::Export;
    s.export_fraction = 0.5;
    CHECK(pipelineFraction(s) == doctest::Approx(0.95));
    s.state = PipelineState::Succeeded;
    CHECK(pipelineFraction(s) == 1.0);
}
