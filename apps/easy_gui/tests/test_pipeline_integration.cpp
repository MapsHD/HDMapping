#include <doctest.h>

#include "batch.h"
#include "pipeline.h"
#include "test_helpers.h"

#include <chrono>
#include <cstdlib>
#include <thread>

using namespace easy_gui;
namespace fs = std::filesystem;

// Integration tests against the real lidar_odometry_step_1 (built next to this test binary). They need a real
// Mandeye recording, so they only run when EASY_GUI_TEST_RECORDING points to one (a few file pairs are enough).
namespace
{
    const char* testRecording()
    {
        return std::getenv("EASY_GUI_TEST_RECORDING");
    }

    PipelineStatus waitUntil(const Pipeline& pipeline, bool (*done)(const PipelineStatus&), std::chrono::seconds timeout)
    {
        const auto t0 = std::chrono::steady_clock::now();
        PipelineStatus s = pipeline.status();
        while (!done(s) && std::chrono::steady_clock::now() - t0 < timeout)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            s = pipeline.status();
        }
        return s;
    }
} // namespace

TEST_CASE("cancel stops a real step 1 run during odometry and cleans up" * doctest::skip(testRecording() == nullptr))
{
    easy_gui_test::TempDir tmp;
    const fs::path out = tmp.path / "out";
    Pipeline pipeline;
    std::string error;
    REQUIRE(pipeline.start(PipelineRequest{ testRecording(), out, findStep1Executable(), Settings{} }, error));

    const PipelineStatus during = waitUntil(
        pipeline,
        [](const PipelineStatus& s)
        {
            return s.state != PipelineState::Running || (s.step1.phase == Step1Phase::Odometry && s.step1.chunks_done > 0);
        },
        std::chrono::seconds(300));
    REQUIRE(during.state == PipelineState::Running);
    REQUIRE(during.step1.phase == Step1Phase::Odometry);

    const auto t_cancel = std::chrono::steady_clock::now();
    pipeline.cancel();
    const PipelineStatus after = waitUntil(
        pipeline,
        [](const PipelineStatus& s)
        {
            return s.state != PipelineState::Running;
        },
        std::chrono::seconds(30));
    CHECK(after.state == PipelineState::Cancelled);
    CHECK(std::chrono::steady_clock::now() - t_cancel < std::chrono::seconds(10));
    CHECK_FALSE(fs::exists(out / "work")); // links, step 1 cache and partial session are gone
    CHECK(fs::exists(out / "easy_gui.log")); // the log stays
    CHECK(fs::exists(fs::path(testRecording()))); // and the recording, of course
}

namespace
{
    //! One file pair of real data, cut to its first 200 points: it passes the pre-flight check, so step 1 really runs,
    //! finds no usable data and must fail gracefully.
    void writeTooShortRecording(const fs::path& rec)
    {
        fs::create_directories(rec);
        const fs::path source(testRecording());
        fs::copy_file(source / "imu0000.csv", rec / "imu0000.csv");
        fs::copy_file(source / "lidar0000.sn", rec / "lidar0000.sn");
        // Timestamps inside the IMU range, so the pre-flight check passes and step 1 really runs on unusable data.
        std::ifstream imu(rec / "imu0000.csv");
        std::string header, first_sample;
        std::getline(imu, header);
        std::getline(imu, first_sample);
        const double imu_t0 = std::stod(first_sample.substr(0, first_sample.find(' '))) / 1e9;
        easy_gui_test::writeRawLaz(rec / "lidar0000.laz", 200, imu_t0 + 1.0);
    }
} // namespace

TEST_CASE("a recording too short for odometry fails with a message, not a crash" * doctest::skip(testRecording() == nullptr))
{
    easy_gui_test::TempDir tmp;
    const fs::path rec = tmp.path / "rec";
    writeTooShortRecording(rec);

    Pipeline pipeline;
    std::string error;
    REQUIRE(pipeline.start(PipelineRequest{ rec, tmp.path / "out", findStep1Executable(), Settings{} }, error));
    const PipelineStatus s = waitUntil(
        pipeline,
        [](const PipelineStatus& st)
        {
            return st.state != PipelineState::Running;
        },
        std::chrono::seconds(120));
    CHECK(s.state == PipelineState::Failed);
    CHECK_FALSE(s.message.empty());
    MESSAGE("message shown to the user: " << s.message);
}

TEST_CASE("a real run ends with the point cloud, the trajectory and the README" * doctest::skip(testRecording() == nullptr))
{
    easy_gui_test::TempDir tmp;
    const fs::path out = tmp.path / "out";
    Pipeline pipeline;
    std::string error;
    REQUIRE(pipeline.start(PipelineRequest{ testRecording(), out, findStep1Executable(), Settings{} }, error));
    const PipelineStatus s = waitUntil(
        pipeline,
        [](const PipelineStatus& st)
        {
            return st.state != PipelineState::Running;
        },
        std::chrono::seconds(600));
    REQUIRE_MESSAGE(s.state == PipelineState::Succeeded, s.message);
    const OutputLayout layout(out);
    CHECK(fs::file_size(layout.point_cloud) == s.export_stats.laz_bytes);
    CHECK(fs::file_size(layout.trajectory) == s.export_stats.trajectory_bytes);
    CHECK(fs::exists(layout.readme));
    CHECK(s.export_stats.odometry_points > 0);
    CHECK(s.export_stats.near_points > 0);
    CHECK(s.export_stats.gap_points > 0);
    CHECK(s.export_stats.check_band_points == 0);
    CHECK(s.export_stats.range_min_m >= Settings{}.near_min_range_m);
    CHECK(fs::exists(layout.session / "session.mjs")); // kept for step 2
    CHECK_FALSE(fs::exists(layout.staging));
}

TEST_CASE(
    "batch: two copies of a real recording and a broken one; the broken one fails, a restart skips the rest" *
    doctest::skip(testRecording() == nullptr))
{
    easy_gui_test::TempDir tmp;
    const fs::path parent = tmp.path / "field_day";
    for (const char* copy : { "a_copy", "c_copy" })
    {
        fs::create_directories(parent / copy);
        for (const auto& entry : fs::directory_iterator(testRecording()))
            if (entry.is_regular_file())
                fs::create_symlink(fs::canonical(entry.path()), parent / copy / entry.path().filename());
    }
    writeTooShortRecording(parent / "b_broken");
    fs::create_directories(parent / "notes");
    const fs::path out = tmp.path / "results";

    BatchPlan plan = planBatch(parent, out);
    REQUIRE(plan.request.items.size() == 3);
    REQUIRE(plan.request.skipped.size() == 1);
    for (const auto& item : plan.request.items)
        REQUIRE_MESSAGE(item.include, item.exclude_reason);
    plan.request.step1_executable = findStep1Executable();

    {
        BatchRunner runner;
        std::string error;
        REQUIRE(runner.start(plan.request, error));
        runner.wait();
        const BatchStatus s = runner.status();
        CHECK_MESSAGE(s.items[0].state == BatchItemState::Succeeded, s.items[0].error);
        CHECK(s.items[1].state == BatchItemState::Failed);
        CHECK_FALSE(s.items[1].error.empty());
        CHECK_MESSAGE(s.items[2].state == BatchItemState::Succeeded, s.items[2].error);
        MESSAGE("broken recording: " << s.items[1].error);
        for (const char* copy : { "a_copy", "c_copy" })
        {
            const OutputLayout layout(out / copy);
            CHECK(fs::file_size(layout.point_cloud) > 0);
            CHECK(fs::exists(layout.trajectory));
            CHECK(fs::exists(layout.done_marker));
            CHECK_FALSE(fs::exists(layout.staging));
        }
        CHECK(s.items[0].exported_points == s.items[2].exported_points); // step 1 is deterministic
        CHECK(fs::exists(out / "b_broken" / "easy_gui.log"));
        CHECK_FALSE(fs::exists(OutputLayout(out / "b_broken").done_marker));
        CHECK(fs::exists(out / BatchRunner::kLogFileName));
    }

    // Restart: the copies are skipped, the broken one is retried (and fails again).
    BatchPlan again = planBatch(parent, out);
    REQUIRE(again.request.items.size() == 3);
    CHECK(again.checks[0].done.has_value());
    CHECK_FALSE(again.checks[1].done.has_value());
    CHECK(again.request.items[1].include); // its unfinished attempt (the log) does not block it
    again.request.step1_executable = findStep1Executable();
    BatchRunner runner;
    std::string error;
    REQUIRE(runner.start(again.request, error));
    runner.wait();
    const BatchStatus s = runner.status();
    CHECK(s.items[0].state == BatchItemState::AlreadyDone);
    CHECK(s.items[1].state == BatchItemState::Failed);
    CHECK(s.items[2].state == BatchItemState::AlreadyDone);

    std::ifstream csv(out / BatchRunner::kSummaryFileName);
    std::string header, a, b, c, notes;
    std::getline(csv, header);
    std::getline(csv, a);
    std::getline(csv, b);
    std::getline(csv, c);
    std::getline(csv, notes);
    CHECK(a.find(",a_copy,already_done,") != std::string::npos);
    CHECK(b.find(",b_broken,failed,") != std::string::npos);
    CHECK(c.find(",c_copy,already_done,") != std::string::npos);
    CHECK(notes.find(",,skipped,") != std::string::npos);
}
