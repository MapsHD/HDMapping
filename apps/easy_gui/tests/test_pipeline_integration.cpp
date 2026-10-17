#include <doctest.h>

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

TEST_CASE("a recording too short for odometry fails with a message, not a crash" * doctest::skip(testRecording() == nullptr))
{
    // One file pair of real data, cut to its first 200 points: step 1 starts, finds no usable data and must fail
    // gracefully through Easy GUI.
    easy_gui_test::TempDir tmp;
    const fs::path rec = tmp.path / "rec";
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
